#ifndef BUNDLEADJUST2_H
#define BUNDLEADJUST2_H
#include "featext.h"
#include "bundleadjustinf.h"

#define MAKE_CORNER_IDX(x, y) ((y) << 16 | (x))
#define CORNER_Y(idx) ((idx) >> 16 & 0x7fff)
#define CORNER_X(idx) ((idx) & 0x7fff)

struct Edge2 {
	const EdgeDiff * diff;
	int mls[2]; //min location shift, mls0 for y, mls1 for x
	float ca[2]; //cost alpha
	Point idea_pos;
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
};

struct FourCorner {
	unsigned idx;	//it is fixed after init
	int res_sft[2]; //residual shift, res_sft0 for y, res_sft1 for x
	int bd; //boundary distance
	float cost;
	unsigned fa;
	int depth;
	int type[2];
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

class BundleAdjust2 : public BundleAdjustInf
{
protected:	
	vector<Edge2> eds[2];
	vector<FourCorner> fc; //(rows+1) * (cols+1)
	int img_num_h, img_num_w, scale;
	Mat_<Vec2i> best_offset;
	Mat_<int> corner_info;
	vector<FourCorner *> adjust_queue[2];

protected:
	Edge2 * get_edge(int i, int y, int x);
	Edge2 * get_edge(int idx);
	FourCorner * get_4corner(int idx);
	void compute_edge_cost_ratio(Edge2 * pe, int dim);
	void init(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe);
	void adjust_edge_mls(FourCorner * pc, int dim, int modify);
	void adjust_one_4corner(int dim, FourCorner * pc);
	void adjust();

public:
	BundleAdjust2() {}
	~BundleAdjust2() {}
	Mat_<Vec2i> get_best_offset() {
		return best_offset.clone();
	}
	Mat_<int> get_corner() {
		return corner_info.clone();
	}
	int arrange(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe);
};

#endif // BUNDLEADJUST2_H
