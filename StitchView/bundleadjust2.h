#ifndef BUNDLEADJUST2_H
#define BUNDLEADJUST2_H
#include "featext.h"
#include "bundleadjustinf.h"
#include <queue>
#include <deque>
#include <map>

#define COST_BIND			10000000
#define COST_BIGER_THAN_AVG 50000
#define COST_BOUNDARY		(2 * COST_BIGER_THAN_AVG)

class Edge2 {
public:
	static int image_width, image_height;
	const EdgeDiff * diff;
	Mat_<float> cost;
	float hard_score_x, hard_score_y; //higher means not easy to move,negative means easy to move
	float estimat_cost;
	int mls[2]; //min location shift, mls0 for y, mls1 for x
	int flagb; //scale, BIND_X_MASK, BIND_Y_MASK
	Point idea_pos;//it is nearby image top-left point - base image top-left point
	void get_4corner_idx(unsigned & idx1, unsigned & idx2) {
		unsigned edge_idx = diff->edge_idx;
		if (edge_idx & 0x80000000) {
			idx1 = (edge_idx & 0x7fffffff) + 1;
			idx2 = (edge_idx & 0x7fffffff) + 0x10001;
		}
		else {
			idx1 = edge_idx + 0x10000;
			idx2 = edge_idx + 0x10001;
		}
	}

	float get_point_cost(Point o, int scale) {
		if (o.x < 0 || o.x >= cost.cols || o.y < 0 || o.y >= cost.rows) {
			if (!FIX_EDGE_ISBIND(flagb))
				return COST_BOUNDARY;
			int nearby = FIX_EDGE_SCALE(flagb) / scale;
			int valid_x = FIX_EDGE_BINDX(flagb) ? nearby / 2 : 100000;
			int valid_y = FIX_EDGE_BINDY(flagb) ? nearby / 2 : 100000;
			if (diff->img_num == 0) { //img_num==0 same as free move
				valid_x = 100000;
				valid_y = 100000;
			}
			int ret = 0;
			if (valid_x == 100000 && (o.x < -1 || o.x >= cost.cols + 1)) //even for blue fix, we hope it in range
				ret = COST_BOUNDARY;
			if (valid_y == 100000 && (o.y < -1 || o.y >= cost.rows + 1)) //even for blue fix, we hope it in range
				ret = COST_BOUNDARY;
			Point pos = idea_pos - diff->offset;
			pos.x = pos.x / scale;
			pos.y = pos.y / scale;
			if (abs(o.y - pos.y) <= valid_y && abs(o.x - pos.x) <= valid_x) {
				Point pos1 = o * scale + diff->offset; //change point o to offset 
				if (EDGE_E(diff->edge_idx)) {
					if (pos1.x >= image_width || abs(pos1.y) >= image_height /4)
						return COST_BOUNDARY * 2;
					else
						return ret;
				}
				else {
					if (pos1.y >= image_height || abs(pos1.x) >= image_width /4 )
						return COST_BOUNDARY * 2;
					else
						return ret;
				}
			}				
			else
				return COST_BIND;
		}
		return cost(o);
	}
	Point get_current_point(int scale) {
		Point pos = idea_pos - diff->offset;
		pos.x = pos.x / scale + mls[1];
		pos.y = pos.y / scale + mls[0];
		return pos;
	}
};
typedef double COST_TYPE;
struct SourceCost {
	COST_TYPE cost;
	unsigned fa;
	int update_num;
	int update_state; //1 means it has updated value to propogate, 2 means it is invalid, need to be updated
	void reset(COST_TYPE _cost) {
		cost = _cost;
		fa = 0xffffffff;
		update_num = 0;
		update_state = 0;
	}
};

struct FourCorner {
	unsigned idx;	//it is fixed after init
	int res_sft[2]; //residual shift, res_sft0 for y, res_sft1 for x
	int bd; //boundary distance
	vector<vector<SourceCost> > cs; //cs[i] refer to each source[i],cs[i][j] refer to source[i] shift j
	bool visited; //for loop search
	bool already_inqueue;  //or of all cs[i].update_state
	unsigned change_id;
	unsigned change_x, change_y;
	/*up is 0, right is 1, down is 2, left is 3, return edge index which can be used by get_edge*/
	unsigned get_edge_idx(int dir) {
		switch (dir) {
		case DIR_LEFT:
			if (idx >= 0x10001)
				return idx - 0x10001;
			break;

		case DIR_UP:
			if (idx >= 0x10001)
				return (idx - 0x10001) | 0x80000000;
			break;

		case DIR_RIGHT:
			if (idx >= 0x10000)
				return idx - 0x10000;
			break;

		case DIR_DOWN:
			if (idx >= 1)
				return (idx - 1) | 0x80000000;
			break;
		default:
			CV_Assert(0);
		}
		return 0xffffffff;
	}

	//from idx to corner_idx, choose which dir
	unsigned get_dir(unsigned corner_idx) {
		if (CORNER_Y(idx) == CORNER_Y(corner_idx)) {
			if (CORNER_X(idx) > CORNER_X(corner_idx))
				return DIR_LEFT;
			else
				return DIR_RIGHT;
		}
		else {
			if (CORNER_Y(idx) > CORNER_Y(corner_idx))
				return DIR_UP;
			else
				return DIR_DOWN;
		}
	}
	unsigned get_4corner_idx(int dir) {
		switch (dir) {
		case DIR_UP:
			if (idx >= 0x10000)
				return idx - 0x10000;
			break;

		case DIR_RIGHT:
			return idx + 1;

		case DIR_DOWN:
			return idx + 0x10000;

		case DIR_LEFT:
			if (idx & 0x7fff)
				return idx - 1;
			break;
		}
		return 0xffffffff;
	}
};
typedef pair<unsigned long long, unsigned long long> Bundle;

struct SourceInfo {
	unsigned idx;
	int sign; //sign =0 means border, sign=1, mean pos
	int isx; //isx=1 means x, =0 means y
	int cap;
	SourceInfo(unsigned _idx, int _sign, int _isx, int _cap) {
		idx = _idx;
		sign = _sign;
		isx = _isx;
		cap = _cap;
	}
};
struct FourCornerPatch {
	int res_sft[2];
	unsigned change_id;
	unsigned change_x, change_y;
	FourCornerPatch() {}
	FourCornerPatch(FourCorner * pc) {
		res_sft[0] = pc->res_sft[0];
		res_sft[1] = pc->res_sft[1];
		change_id = pc->change_id;
		change_x = pc->change_x;
		change_y = pc->change_y;
	}
};

class UndoPatch {
public:
	map<unsigned, pair<int, int> > edge_patch;
	map<unsigned, FourCornerPatch> corner_patch;

	void add_patch(unsigned edge_idx, Edge2 * pe) {
		if (edge_patch.find(edge_idx) == edge_patch.end())
			edge_patch[edge_idx] = make_pair(pe->mls[0], pe->mls[1]);
	}
	void add_patch(unsigned fc_idx, FourCorner * pc) {
		if (corner_patch.find(fc_idx) == corner_patch.end())
			corner_patch[fc_idx] = FourCornerPatch(pc);
	}
	void clear() {
		edge_patch.clear();
		corner_patch.clear();
	}
};
class BundleAdjust2 : public BundleAdjustInf
{
protected:
	vector<Edge2> eds[2]; //0 for up-down edge
	vector<FourCorner> fc; //(rows+1) * (cols+1)
	int img_num_h, img_num_w, scale;
	Mat_<Vec2i> best_offset;
	Mat_<unsigned long long> corner_info;
	unsigned change_id;

protected:
	Edge2 * get_edge(int i, int y, int x);
	Edge2 * get_edge(FourCorner * pc0, FourCorner * pc1);
	Edge2 * get_edge(int idx);
	FourCorner * get_4corner(int y, int x);
	FourCorner * get_4corner(int idx);
	void compute_edge_cost(Edge2 * pe, float alpha, bool weak);
	void compute_edge_cost2(Edge2 * pe, float alpha, bool weak);
	void print_4corner_stat();
	Point2f check_res_sft(bool update_corner_info);
	void align();
	void init(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe, Size s, bool week_border, bool need_align);
	void undo(const UndoPatch & patch);
	void adjust_edge_mls(FourCorner * ps, FourCorner * pt, int sidx, int modify, queue<unsigned> & rq, vector<SourceInfo> & source, int change_id, UndoPatch * patch, float cost_th, bool border_is_source);
	void relax(FourCorner * pc, const Rect & range, queue<unsigned> & rq, vector<SourceInfo> & source, UndoPatch * patch, float cost_th, bool border_is_source);
	void check_relax(const Rect & rect, vector<SourceInfo> & source, float cost_th);
	void check_fix_edge();
	Bundle search_bundle(FourCorner * pc, int len_limit, int width_limit);
	bool merge_one_bundle(Bundle b);
	void merge_bundles();
	void merge_all();
	void optimize_corner(unsigned corner_idx, int max_shift_x, int max_shift_y);
	void optimize_corner(int max_shift_x, int max_shift_y);
	void output();

public:
	int merge_square_area(const Rect & src_rect, const Rect & tgt_rect, const Rect & outer, int src_posx, int src_posy, float cost_th);
	BundleAdjust2() {}
	~BundleAdjust2() {}
	Mat_<Vec2i> get_best_offset() {
		return best_offset.clone();
	}
	Mat_<unsigned long long> get_corner() {
		return corner_info.clone();
	}
	int arrange(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe, Size s, int week_border);
};

#endif // BUNDLEADJUST2_H
