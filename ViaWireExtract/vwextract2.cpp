#include "vwextract2.h"
#include <list>
#include <algorithm>
#include <QDateTime>
#include <QScopedPointer>
#include <QDir>
#include <QtConcurrent>
#include <set>
#include <queue>
#include "vwextract_public.h"
#include <deque>

#define SAVE_RST_TO_FILE	0
#ifdef QT_DEBUG
#define DEBUG_LEFT_TOP_IMG  1
#else
#define DEBUG_LEFT_TOP_IMG  0
#endif


#define OPT_PARALLEL_SEARCH		1
#define OPT_DEBUG_EN		0x8000
#define OPT_DEBUG_OUT_EN	0x4000

#define VIA_NEAR_IV_SCORE	9
#define VIA_NEAR_NO_WIRE_SCORE	8
#define VIA_NEAR_WIRE_SCORE 1

#define ADJUST_GRAY_LVL_MANUAL_SET				1

#define REMOVE_VIA_MASK							1
#define CLEAR_REMOVE_VIA_MASK					2
#define REMOVE_VIA_NOCHG_IMG					4

#define COARSE_LINE_CLEAR_PROB					2
#define COARSE_LINE_UPDATE_PROB					1
#define COARSE_LINE_UPDATE_WITH_MASK			4
#define COARSE_LINE_SEARCH_CLEAR_COLOR			1

#define HOTLINE_CLEAR_MASK						1

#define FINE_LINE_SEARCH_CLEAR_PROB				1
#define FINE_LINE_SEARCH_CLEAR_COLOR			2
#define FINE_LINE_SEARCH_NO_VIA					4
#define FILE_LINE_SEARCH_CHECK_VIA_WIRE_CONNECT	8

#define ASSEMBLE_LINE_NO_VIA					1

#define ASSEMBLE_VIA_ALLOW_45					1
#define ASSEMBLE_VIA_SEARCH_SKIN				2
#define ASSEMBLE_BRANCH_ALLOW_45				1
#define ASSEMBLE_BRANCH_SEARCH_CROSS			2
#define ASSEMBLE_BRANCH_NOT_REMARK_BORDER		4

#define CHECK_VW_HOTLINE_CLEAR_MASK				1
#define CHECK_VW_SEARCH_CLEAR_COLOR				2
#define ASSEMBLE_FIX_DISTANCE					2
#define GUARD_FOR_JOINT							15

#define JOINT_POINT_MAX							23
#define H_SHAPE_CHECK							1
#define VIA_CONNECT_CHECK						2
#define CHECK_LOOP								1

#define EDGE_DETECT_HAS_VIA_MASK				1
#define EDGE_DETECT_WIDE_2						2
#define EDGE_DETECT_ERODE						4
#define IMG_ENHANCE_HAS_VIA_MASK				1

typedef pair<unsigned long long, unsigned long long> PAIR_ULL;

#define POINT_DIS(p0, p1) (abs(p0.x - p1.x) + abs(p0.y -p1.y))
enum {
	TYPE_NONE,
	TYPE_GRAY_LEVEL,
	TYPE_COARSE_WIRE,
	TYPE_VIA_MASK,
	TYPE_FINE_WIRE_MASK,
	TYPE_REMOVE_VIA_MASK,
	TYPE_SHADOW_PROB,
	TYPE_HOT_POINT_MASK,
	TYPE_EDGE_MASK
};

enum {
	VIA_SUBTYPE_2CIRCLE = 0,
	VIA_SUBTYPE_3CIRCLE,
	VIA_SUBTYPE_4CIRCLE,
	VIA_SUBTYPE_2CIRCLEX,
	VIA_SUBTYPE_3CIRCLEX,
};

enum {
	WIRE_SUBTYPE_13RECT = 0,
	WIRE_SUBTYPE_25RECT = 1,
	WIRE_SUBTYPE_MSBRANCH = 3
};


enum {
	GRAY_ZERO = 0,
	GRAY_L1,
	GRAY_L0,
	GRAY_L2,
	GRAY_M1,
	GRAY_M0,
	GRAY_M2,
	GRAY_H1,
	GRAY_H0,
	GRAY_H2,
	GRAY_FULL
};


#define SUSPECT_BRICK	4


/*     31..24  23..16   15..8   7..0
opt0:		0  subtype   type  pattern
opt1:  pair_d  arfactor remove_rd guard
opt2:    rd3    rd2      rd1     rd0
opt3:   gray3  gray2    gray1   gray0
opt4:	connect_d cgray_d cgray_ratio connnect_rd
opt5:							swide_min
opt6:
*/
struct ViaParameter {
	int pattern;
	int type;
	int subtype;
	int pair_distance; //via pair distance
	int arfactor; //area gamma factor
	int guard; //guard radius
	int remove_rd; //remove radius	
	int rd0; //inter via radius
	int rd1; //middle via radius
	int rd2; //outter via radius
	int rd3; //outter via radius
	int gray0, gray1, gray2, gray3;
	int connect_d; //for check_via_wire_connect via wire distance check 
	int cgray_d; //for check_via_wire_connect via wire border check
	int cgray_ratio; //skin th
	int connect_rd; //used for skin points
	int swide_min, opt1;
	float optf;
};

/*     31..24  23..16   15..8   7..0
opt0:		0  subtype   type  pattern
opt1:				   arfactor guard
opt2:  w_wide  w_wide1  w_high  w_high1
opt3:	 dis_vw0  dis_vw1 i_wide  i_high
opt4:   wgid3   wgid2   wgid1   wgid0
opt5:   wgid7   wgid6   wgid5   wgid4
opt6:				  uni_id(h) uni_id(l)
*/
struct WireParameter {
	int pattern;
	int type;
	int subtype;
	int arfactor; //area gamma factor
	int guard; //guard radius for unique
	int w_wide, w_wide1, w_high, w_high1; //normally w_high1 <w_high & w_wide1 <w_wide, these is for wire
	int i_wide, i_high;	//this is for insu
	int gray_w, gray_i; //this is not set by ProcessParameter
	int dis_vw0, dis_vw1; //this is via wire distance, dis_vw0 is qiexiang
	int wgid[8], uni_id;
};

/*		31..24  23..16   15..8   7..0
opt0:		0  subtype   type  pattern
opt1:				   arfactor guard
opt2:   i_high i_wide mwide_max mwide_min
opt3: slen_max slen_min swide_max swide_min
opt4:            sth   mth_high mth_low
opt5:
*/
struct WireParameter2 {
	int pattern;
	int type;
	int subtype;
	int arfactor; //area gamma factor
	int guard; //guard radius for unique
	int i_wide, i_high;	//this is for insu
	int mwide_max, mwide_min; //main branch wide max and min
	int slen_min; //skin len need to reach slen_min to become siding branch
	int slen_max; //search max skin len depth
	int swide_max, swide_min; //siding branch wide
	int s_th; //siding branch threshold
	int mth_high, mth_low; //main branch threshold
	int gray_w, gray_i; //this is not set by ProcessParameter
};

union VWParameter {
	struct ViaParameter v;
	struct WireParameter w;
	struct WireParameter2 w2;
};

class VWSet {
	vector <VWParameter> vws;
public:
	VWParameter * get_vw_para(int pattern, int type, int sub_type) {
		for (int i = 0; i < (int)vws.size(); i++)
		if (vws[i].v.type == type && vws[i].v.pattern == pattern && vws[i].v.subtype == sub_type)
			return &vws[i];
		return NULL;
	}
	void set_vw_para(ProcessParameter cpara) {
		int pattern = cpara.opt0 & 0xff;
		int type = cpara.opt0 >> 8 & 0xff;
		int sub_type = cpara.opt0 >> 16 & 0xff;
		VWParameter * vw = get_vw_para(pattern, type, sub_type);
		if (vw == NULL) {
			VWParameter v;
			v.v.type = type;
			v.v.pattern = pattern;
			v.v.subtype = sub_type;
			vws.push_back(v);
			vw = get_vw_para(pattern, type, sub_type);
		}
		if (pattern < 224) {
			if (sub_type < WIRE_SUBTYPE_MSBRANCH) {
				vw->w.guard = cpara.opt1 & 0xff;
				vw->w.arfactor = cpara.opt1 >> 8 & 0xff;
				vw->w.w_wide = cpara.opt2 >> 24 & 0xff;
				vw->w.w_wide1 = cpara.opt2 >> 16 & 0xff;
				vw->w.w_high = cpara.opt2 >> 8 & 0xff;
				vw->w.w_high1 = cpara.opt2 & 0xff;
				vw->w.i_wide = cpara.opt3 >> 8 & 0xff;
				vw->w.i_high = cpara.opt3 & 0xff;
				vw->w.dis_vw0 = cpara.opt3 >> 24 & 0xff;
				vw->w.dis_vw1 = cpara.opt3 >> 16 & 0xff;
				vw->w.wgid[0] = cpara.opt4 & 0xff;
				vw->w.wgid[1] = cpara.opt4 >> 8 & 0xff;
				vw->w.wgid[2] = cpara.opt4 >> 16 & 0xff;
				vw->w.wgid[3] = cpara.opt4 >> 24 & 0xff;
				vw->w.wgid[4] = cpara.opt5 & 0xff;
				vw->w.wgid[5] = cpara.opt5 >> 8 & 0xff;
				vw->w.wgid[6] = cpara.opt5 >> 16 & 0xff;
				vw->w.wgid[7] = cpara.opt5 >> 24 & 0xff;
				vw->w.uni_id = cpara.opt6 & 0xffff;
				qInfo("set_wire_para, guard=%d, arfactor=%d, w_wide=%d, w_wide1=%d, w_high=%d, w_high1=%d, i_wide=%d, i_high=%d, dis_vw0=%d, dis_vw1=%d",
					vw->w.guard, vw->w.arfactor, vw->w.w_wide, vw->w.w_wide1, vw->w.w_high, vw->w.w_high1,
					vw->w.i_wide, vw->w.i_high, vw->w.dis_vw0, vw->w.dis_vw1);
				qInfo("set_wire_para, id0=%d, id1=%d, id2=%d, id3=%d, id4=%d, id5=%d, id6=%d, id7=%d, uni_id=%d",
					vw->w.wgid[0], vw->w.wgid[1], vw->w.wgid[2], vw->w.wgid[3], vw->w.wgid[4],
					vw->w.wgid[5], vw->w.wgid[6], vw->w.wgid[7], vw->w.uni_id);
			}
			else {
				vw->w2.arfactor = cpara.opt1 >> 8 & 0xff;
				vw->w2.guard = cpara.opt1 & 0xff;
				vw->w2.i_high = cpara.opt2 >> 24 & 0xff;
				vw->w2.i_wide = cpara.opt2 >> 16 & 0xff;
				vw->w2.mwide_max = cpara.opt2 >> 8 & 0xff;
				vw->w2.mwide_min = cpara.opt2 & 0xff;
				vw->w2.slen_max = cpara.opt3 >> 24 & 0xff;
				vw->w2.slen_min = cpara.opt3 >> 16 & 0xff;
				vw->w2.swide_max = cpara.opt3 >> 8 & 0xff;
				vw->w2.swide_min = cpara.opt3 & 0xff;
				vw->w2.s_th = cpara.opt4 >> 16 & 0xff;
				vw->w2.mth_high = cpara.opt4 >> 8 & 0xff;
				vw->w2.mth_low = cpara.opt4 & 0xff;
				qInfo("set_wire2_para, guard=%d, arfactor=%d, mwide_max=%d, mwide_min=%d, i_wide=%d, i_high=%d",
					vw->w2.guard, vw->w2.arfactor, vw->w2.mwide_max, vw->w2.mwide_min, vw->w2.i_wide, vw->w2.i_high);
				qInfo("set_wire2_para, slen_max=%d, slen_min=%d, swide_max=%d, swide_min=%d, s_th=%d, mth_high=%d, mth_low=%d",
					vw->w2.slen_max, vw->w2.slen_min, vw->w2.swide_max, vw->w2.swide_min, vw->w2.s_th, vw->w2.mth_high, vw->w2.mth_low);
			}
		}
		else {
			vw->v.guard = cpara.opt1 & 0xff;
			vw->v.remove_rd = cpara.opt1 >> 8 & 0xff;
			vw->v.arfactor = cpara.opt1 >> 16 & 0xff;
			vw->v.pair_distance = cpara.opt1 >> 24 & 0xff;
			vw->v.rd0 = cpara.opt2 & 0xff;
			vw->v.rd1 = cpara.opt2 >> 8 & 0xff;
			vw->v.rd2 = cpara.opt2 >> 16 & 0xff;
			vw->v.rd3 = cpara.opt2 >> 24 & 0xff;
			vw->v.gray0 = cpara.opt3 & 0xff;
			vw->v.gray1 = cpara.opt3 >> 8 & 0xff;
			vw->v.gray2 = cpara.opt3 >> 16 & 0xff;
			vw->v.gray3 = cpara.opt3 >> 24 & 0xff;
			vw->v.connect_d = cpara.opt4 >> 24 & 0xff;
			vw->v.cgray_d = cpara.opt4 >> 16 & 0xff;
			vw->v.cgray_ratio = cpara.opt4 >> 8 & 0xff;
			vw->v.connect_rd = cpara.opt4 & 0xff;
			vw->v.swide_min = cpara.opt5 & 0xff;
			vw->v.optf = cpara.opt_f0;
			qInfo("set_via_para, guard=%d, remove_rd=%d, rd0=%d, rd1=%d, rd2=%d, rd3=%d",
				vw->v.guard, vw->v.remove_rd, vw->v.rd0, vw->v.rd1, vw->v.rd2, vw->v.rd3);
		}
	}
};

#define MAKE_PROB(s, x, y) ((unsigned long long)(s) << 32 | (y) << 16 | (x))
#define PROB_X(p) ((int)((p) & 0xffff))
#define PROB_Y(p) ((int)((p) >> 16 & 0xffff))
#define PROB_XY(p) ((int)((p) & 0xffffffff))
#define PROB_S(p) ((unsigned)((p) >> 32 & 0xffffffff))
#define PROB_SHAPE(p) ((unsigned)((p) >> 32 & 0xff))
#define PROB_TYPE(p) ((unsigned)((p) >> 40 & 0xff))
#define PROB_SCORE(p) ((int)((p) >> 48 & 0xffff))
#define S_SCORE(p) ((int)((p) >> 16 & 0xffff))
#define S_TYPE(p) ((unsigned)((p) >> 8 & 0xff))
#define S_SHAPE(p) ((unsigned)((p) & 0xff))
#define MAKE_S(score, type, shape) ((unsigned)(score) << 16 | (unsigned)(type) << 8 | (shape))
#define PROB_TYPESHAPE(p) ((unsigned)((p) >> 32 & 0xffff))
#define SET_PROB_TYPE(p, t) (p = (p & 0xffff00ffffffffffULL) | (unsigned long long) (t) << 40)
#define SET_PROB_X(p, x) (p = (p & 0xffffffffffff0000ULL) | (unsigned long long) (x))
#define SET_PROB_Y(p, y) (p = (p & 0xffffffff0000ffffULL) | (unsigned long long) (y) << 16)
#define SET_PROB_SCORE(p, score) (p = (p & 0x0000ffffffffffffULL) | (unsigned long long) (score) << 48)
#define SET_PROB_SHAPE(p, shape) (p = (p & 0xffffff00ffffffffULL) | (unsigned long long) (shape) << 32)
#define SET_PROB_S(p, s) (p = (p & 0x00000000ffffffffULL) | (unsigned long long) (s) << 32)
#define MARK_X(p) ((int)((p) & 0xffff))
#define SET_MARK_X(p, x) (p = (p & 0xffffffffffff0000ULL) | (unsigned long long) (x))
#define MARK_Y(p) ((int)((p) >> 16 & 0xffff))
#define SET_MARK_Y(p, y) (p = (p & 0xffffffff0000ffffULL) | (unsigned long long) (y) << 16)
#define MARK_ISSKIN(m) ((int)((m) >> 63 & 1))
#define SET_MARK_ISSKIN(m, isskin) (m = (m & 0x7fffffffffffffffULL) | (unsigned long long) (isskin) << 63)
#define MARK_BRANCH(m) ((int)((m) >> 49 & 0x3fff))
#define SET_MARK_BRANCH(m, branch) (m = (m & 0x8001ffffffffffffULL) | (unsigned long long) (branch) << 49)
#define MARK_LEN(m) ((int)((m) >> 40 & 0x1ff))
#define SET_MARK_LEN(m, len) (m = (m & 0xfffe00ffffffffffULL) | (unsigned long long) (len) << 40)
#define MARK_DIRMASK(m) ((int)((m) >> 32 & 0xff))
#define SET_MARK_DIRMASk(m, dir) (m = (m & 0xffffff00ffffffffULL) | (unsigned long long) (dir) << 32)
#define MAKE_MARK(branch, len, dir, is_skin, x, y) ((unsigned long long) (is_skin) << 63 | (unsigned long long)(branch) << 49 \
	| (unsigned long long)(len) << 40 | (unsigned long long)(dir) << 32 | (unsigned long long)(y) << 16 | (unsigned long long) (x))
#define MARK2_BRANCH(m) ((int)((m) & 0x3fff))
#define SET_MARK2_BRANCH(m, b) (m = (m & 0x7fffc000) | (b))
#define MARK2_LEN(m) ((int)((m) >> 14 & 0x1ff))
#define SET_MARK2_LEN(m, l) (m = (m & 0xff803fff) | (l) << 14)
#define MARK2_FLAG(m) ((int)((m) >> 31 & 0x1))
#define SET_MAKR2_FLAG(m, f) (m = (m & 0x7fffffff) | (f) << 31)
#define MARK2_VIA(m) ((int)((m) >> 30 & 0x1))
#define SET_MAKR2_VIA(m, v) (m = (m & 0xbfffffff) | (v) << 30)
#define MAKE_MARK2(b, l, f, v) ((unsigned) (b) | (l) << 14 | (f) << 31 | (v) << 30)
#define POINT_M(m) (Point(MARK_X(m), MARK_Y(m)))
#define EXIST(vec, key) (find(vec.begin(), vec.end(), key)!=vec.end())
#define INVALID_BRANCH 0x3fff
#define MARK_MAX_LEN 0x1ff
//Input x, y, score, for new prob.
//Input gs is grid size, should be match with prob
//return true if (x,y) prob is changed
bool push_new_prob(Mat & prob, int x, int y, unsigned s, unsigned gs) {
	bool ret = false;
	int y0 = y / gs, x0 = x / gs;
	unsigned long long * p_prob = prob.ptr<unsigned long long>(y0, x0);
	CV_Assert(prob.type() == CV_64FC2 && y >= 0 && x >= 0 && x0 < prob.cols && y0 < prob.rows);
	unsigned long long new_s = MAKE_PROB(s, x, y);
	if (p_prob[0] > new_s) {
		ret = true;
		p_prob[1] = p_prob[0];
		p_prob[0] = new_s;
	}
	else
	if (p_prob[1] > new_s) {
		ret = true;
		p_prob[1] = new_s;
	}

	return ret;
}

bool push_new_prob(Mat & prob, unsigned long long score, unsigned gs) {
	return push_new_prob(prob, PROB_X(score), PROB_Y(score), PROB_S(score), gs);
}

unsigned long long * get_prob(Mat & prob, int x, int y, int gs) {
	int y0 = y / gs, x0 = x / gs;
	unsigned long long * p_prob = prob.ptr<unsigned long long>(y0, x0);
	CV_Assert(prob.type() == CV_64FC2 && y >= 0 && x >= 0 && x0 < prob.cols && y0 < prob.rows);
	return p_prob;
}

struct ViaInfo {
	Point xy;
	int type, subtype;
	int via_adj_mask;
	int extend_wire_mask; //it means via's dir has wire
	int pair_d;
	int pattern;
	unsigned via_s;
	int v_pair[4];
	ViaInfo(Point _xy, int _type, int _subtype, int _pattern, int _pair_d, unsigned _via_s) {
		xy = _xy;
		type = _type;
		subtype = _subtype;
		pattern = _pattern;
		pair_d = _pair_d;
		via_adj_mask = 0;
		extend_wire_mask = 0;
		via_s = _via_s;
		for (int i = 0; i < 4; i++)
			v_pair[i] = -1;
	}
};

//obj_sets points unit is pixel
#define OBJ_WIRE			0
#define OBJ_VIA				1
typedef void(*ObjProcessFunc)(void * objs, int layer, int obj_type, ProcessParameter & cpara);
struct ObjProcessHook {
	ObjProcessFunc func; //it process assemble_line output
	ProcessParameter cpara;
};

class WireTypeSet {
	vector<int> types, subtypes, patterns, wides;
public:
	int get_type(int idx) {
		CV_Assert(idx < types.size());
		return types[idx];
	}
	int get_subtype(int idx) {
		CV_Assert(idx < subtypes.size());
		return subtypes[idx];
	}
	int get_pattern(int idx) {
		CV_Assert(idx < patterns.size());
		return patterns[idx];
	}
	int get_wide(int idx) {
		CV_Assert(idx < wides.size());
		return wides[idx];
	}
	/*Input: type, subtype, pattern, wide
	return existing idx
	*/
	int find(int type, int subtype, int pattern, int wide) {
		for (int i = 0; i < (int)types.size(); i++)
		if (types[i] == type && subtypes[i] == subtype && patterns[i] == pattern && wides[i] == wide)
			return i;
		return -1;
	}
	/*Input: idx
	Output type, subtype, pattern, wide
	*/
	void get_all(int idx, int & type, int & subtype, int & pattern, int & wide) {
		CV_Assert(idx < types.size());
		type = types[idx];
		subtype = subtypes[idx];
		pattern = patterns[idx];
		wide = wides[idx];
	}

	/*Input: type, subtype, pattern, wide
	if existing, return existing idx; else add new*/
	int add(int type, int subtype, int pattern, int wide) {
		int ret = find(type, subtype, pattern, wide);
		if (ret >= 0)
			return ret;
		types.push_back(type);
		subtypes.push_back(subtype);
		patterns.push_back(pattern);
		wides.push_back(wide);
		return (int)types.size() - 1;
	}
};

/*Input, p0, dir0, line 1
Input, p1, dir1, line2
output: pis, return point in intersection of line(p0, dir0) and line(p1, dir1)
Return true if pis is valid else two line parallel
*/
static bool intersect_line(Point p0, int dir0, Point p1, int dir1, Point & pis)
{
	if (dir0 == DIR_UP || dir0 == DIR_DOWN) {
		if (dir1 == DIR_LEFT || dir1 == DIR_RIGHT) {
			pis = Point(p0.x, p1.y);
			return true;
		}
		if (dir1 == DIR_UPLEFT || dir1 == DIR_DOWNRIGHT) {
			pis = Point(p0.x, p1.y - p1.x + p0.x);
			return true;
		}
		if (dir1 == DIR_UPRIGHT || dir1 == DIR_DOWNLEFT) {
			pis = Point(p0.x, p1.y + p1.x - p0.x);
			return true;
		}
		CV_Assert(dir1 == DIR_UP || dir1 == DIR_DOWN);
		if (p0.x == p1.x) {
			pis = p0;
			return true;
		}
		return false;
	}

	if (dir1 == DIR_UP || dir1 == DIR_DOWN) {
		if (dir0 == DIR_LEFT || dir0 == DIR_RIGHT) {
			pis = Point(p1.x, p0.y);
			return true;
		}
		if (dir0 == DIR_UPLEFT || dir0 == DIR_DOWNRIGHT) {
			pis = Point(p1.x, p0.y - p0.x + p1.x);
			return true;
		}
		if (dir0 == DIR_UPRIGHT || dir0 == DIR_DOWNLEFT) {
			pis = Point(p1.x, p0.y + p0.x - p1.x);
			return true;
		}
		return false;
	}

	if (dir0 == DIR_LEFT || dir0 == DIR_RIGHT) {
		if (dir1 == DIR_UPLEFT || dir1 == DIR_DOWNRIGHT) {
			pis = Point(p1.x - p1.y + p0.y, p0.y);
			return true;
		}
		if (dir1 == DIR_UPRIGHT || dir1 == DIR_DOWNLEFT) {
			pis = Point(p1.y + p1.x - p0.y, p0.y);
			return true;
		}
		CV_Assert(dir1 == DIR_LEFT || dir1 == DIR_RIGHT);
		if (p0.y == p1.y) {
			pis = p0;
			return true;
		}
		return false;
	}

	if (dir1 == DIR_LEFT || dir1 == DIR_RIGHT) {
		if (dir0 == DIR_UPLEFT || dir0 == DIR_DOWNRIGHT) {
			pis = Point(p0.x - p0.y + p1.y, p1.y);
			return true;
		}
		if (dir0 == DIR_UPRIGHT || dir0 == DIR_DOWNLEFT) {
			pis = Point(p0.y + p0.x - p1.y, p1.y);
			return true;
		}
		return false;
	}

	if (dir0 == DIR_UPLEFT || dir0 == DIR_DOWNRIGHT) {
		if (dir1 == DIR_UPRIGHT || dir1 == DIR_DOWNLEFT) {
			pis = Point(p0.x - p0.y + (p1.x + p1.y + p0.y - p0.x) / 2, (p1.x + p1.y + p0.y - p0.x) / 2);
			return true;
		}
		CV_Assert(dir1 == DIR_UPLEFT || dir1 == DIR_DOWNRIGHT);
		if (p0.x - p0.y == p1.x - p1.y) {
			pis = p0;
			return true;
		}
		return false;
	}

	if (dir1 == DIR_UPLEFT || dir1 == DIR_DOWNRIGHT) {
		if (dir0 == DIR_UPRIGHT || dir0 == DIR_DOWNLEFT) {
			pis = Point(p1.x - p1.y + (p0.x + p0.y + p1.y - p1.x) / 2, (p0.x + p0.y + p1.y - p1.x) / 2);
			return true;
		}
		return false;
	}

	CV_Assert(dir0 == DIR_UPRIGHT || dir0 == DIR_DOWNLEFT);
	CV_Assert(dir1 == DIR_UPRIGHT || dir1 == DIR_DOWNLEFT);
	if (p0.x + p0.y == p1.x + p1.y) {
		pis = p0;
		return true;
	}
	return false;
}

/*
Return dir for vector (dx,dy)
*/
static int check_dir(int dx, int dy, bool allow45)
{
	CV_Assert(dx != 0 || dy != 0);
	if (allow45) {
		if (abs(dx) >= abs(dy) * 2)
			return (dx > 0) ? DIR_RIGHT : DIR_LEFT;
		else
		if (abs(dy) >= abs(dx) * 2)
			return (dy > 0) ? DIR_DOWN : DIR_UP;
		else {
			if (dx > 0)
				return (dy > 0) ? DIR_DOWNRIGHT : DIR_UPRIGHT;
			else
				return (dy > 0) ? DIR_DOWNLEFT : DIR_UPLEFT;
		}
	}
	else {
		if (abs(dx) > abs(dy))
			return (dx > 0) ? DIR_RIGHT : DIR_LEFT;
		else
			return (dy > 0) ? DIR_DOWN : DIR_UP;
	}
}

static const int cnear[8][2] = {
	//y, x
	{ 0, 1 },	//up
	{ 1, 0 },	//right
	{ 0, 1 },	//down
	{ 1, 0 },	//left
	{ 1, 1 },	//right_up
	{ 1, -1 },	//right_down
	{ 1, 1 },	//left_down
	{ 1, -1 }	//left_up
};

/*
input pt1, pt2
output pts, line points include [pt1,pt2)
*/
static void get_line_pts(Point pt1, Point pt2, vector <Point> & pts)
{
	int dx = abs(pt2.x - pt1.x);
	int dy = abs(pt2.y - pt1.y);
	bool dir = dx > dy;
	int ix = (pt2.x - pt1.x > 0) ? 1 : -1;
	int iy = (pt2.y - pt1.y > 0) ? 1 : -1;
	pts.clear();
	if (dir) {
		int dy2 = 2 * dy;
		int dy2dx2 = 2 * dy - 2 * dx;
		int d = 2 * dy - dx;
		int x = pt1.x, y = pt1.y;
		for (; x != pt2.x; x += ix) {
			pts.push_back(Point(x, y));
			if (d < 0)
				d += dy2;
			else {
				d += dy2dx2;
				y += iy;
			}
		}
		CV_Assert(y == pt2.y);
	}
	else {
		int dx2 = 2 * dx;
		int dx2dy2 = 2 * dx - 2 * dy;
		int d = 2 * dx - dy;
		int x = pt1.x, y = pt1.y;
		for (; y != pt2.y; y += iy) {
			pts.push_back(Point(x, y));
			if (d < 0)
				d += dx2;
			else {
				d += dx2dy2;
				x += ix;
			}
		}
		CV_Assert(x == pt2.x);
	}
}

/*
Input line
Input p0
Input prefer_dir
Input allow45
Return correct dir from p0 to go to line
*/
static int find_nearest_dir(pair<Point, Point> line, Point p0, int prefer_dir, bool allow45)
{
	int dir = check_dir(line.second.x - line.first.x, line.second.y - line.first.y, allow45);
	int cl = cnear[dir][0] * line.first.y + cnear[dir][1] * line.first.x; //line c-value
	int cp = cnear[dir][0] * p0.y + cnear[dir][1] * p0.x; //point c-value
	int dir_limit = allow45 ? 8 : 4;
	int ret = 100;
	for (int i = 0; i < dir_limit; i++) {
		Point p1 = p0 + Point(dxy[i][1], dxy[i][0]);
		int cs = cnear[dir][0] * p1.y + cnear[dir][1] * p1.x;
		if (abs(cs - cl) < abs(cp - cl)) { //correct direction
			if (abs(ret - prefer_dir) > abs(i - prefer_dir))
				ret = i;
		}
	}
	return ret;
}
/*
Input pts: point path
out lines: line set, lines[0].first is begin point, lines.back.first is end point
input err: distance from pts to lines within [0,err]
input len: how much length for approximate
*/
static void approximate_line(const vector<Point> & pts, vector<pair<Point, int> > & lines, bool allow45, bool fix_end, int err, int len)
{
	CV_Assert(len >= 3 && err >= 1);
	int org = 0;
	lines.clear();
	while (1) {
		int end = min(org + len - 1, (int)pts.size() - 1);
		int dx = pts[end].x - pts[org].x;
		int dy = pts[end].y - pts[org].y;
		int dir;
		dir = check_dir(dx, dy, allow45);
		int avg = 0, avgx = 0, avgy = 0; //start to compute avg
		if (fix_end && (org == 0 || end == (int)pts.size() - 1)) {
			if (org == 0) {
				avgx = pts[0].x;
				avgy = pts[0].y;
			}
			else {
				avgx = pts.back().x;
				avgy = pts.back().y;
			}
		}
		else {
			for (int i = org; i <= end; i++) {
				avgx += pts[i].x;
				avgy += pts[i].y;
			}
			avgx = avgx / (end - org + 1);
			avgy = avgy / (end - org + 1);
		}
		avg = cnear[dir][0] * avgy + cnear[dir][1] * avgx;
		Point new_pt(avgx, avgy); //new_pt is new line point
		if (!lines.empty()) {
			if (lines.back().second == dir || lines.back().second == dir_1[dir]) //push qiexian dir
				lines.push_back(make_pair(pts[org - 1], dir_2[dir]));
		}
		lines.push_back(make_pair(new_pt, dir));
		for (org = end + 1; org < (int)pts.size(); org++)
		if (abs(avg - cnear[dir][0] * pts[org].y - cnear[dir][1] * pts[org].x) >err)
			break;
		if (org >= pts.size()) {
			if (!fix_end || abs(avg - cnear[dir][0] * pts.back().y - cnear[dir][1] * pts.back().x) == 0)
				break;
		}

		org = min(org - err, (int)pts.size() - 2);
		org = max(org, end - 1);
	}
	if (lines.size() == 1) //push end point
		lines.push_back(make_pair(pts.back(), lines[0].second));
	if (fix_end)
		lines.back().first = pts.back();
}

struct LinkPoint {
	Point pt0, pt1, pt2, pt3; //pt0 belong to branch, pt1 belong to bone or skin, pt2 belong to other bone or skin pt3 belong to other branch, 
	int len;
	int branch, ano_bra;
	LinkPoint() {
	}
	LinkPoint(const Point & _pt0, const Point & _pt1, const Point & _pt2, const Point & _pt3, int _len, int _branch, int _ano_bra) {
		if (_branch < _ano_bra) {
			pt0 = _pt0;
			pt1 = _pt1;
			pt2 = _pt2;
			pt3 = (_ano_bra == INVALID_BRANCH) ? _pt2 : _pt3;
			len = _len;
			branch = _branch;
			ano_bra = _ano_bra;
		}
		else {
			pt0 = _pt3;
			pt1 = _pt2;
			pt2 = _pt1;
			pt3 = (_branch == INVALID_BRANCH) ? _pt1 : _pt0;
			len = _len;
			branch = _ano_bra;
			ano_bra = _branch;
		}
	}
	bool operator< (const LinkPoint & lp) {
		return len < lp.len;
	}
};

class JointPoint {
	vector<LinkPoint> lps;
	vector<int> bs;
	friend class BranchMark;
protected:
	//return index of CP in lps, return -1 if not found, return big number if half found
	int have(const LinkPoint & cp, int guard) {
		int b0 = cp.branch, b1 = cp.ano_bra;
		Point pt0 = cp.pt0, pt3 = cp.pt3;

		int ret_match = -1;
		for (int i = 0; i < lps.size(); i++) {
			bool match00 = (lps[i].branch == b0 && abs(lps[i].pt0.x - pt0.x) < guard && abs(lps[i].pt0.y - pt0.y) < guard);
			bool match11 = (lps[i].ano_bra == b1 && abs(lps[i].pt3.x - pt3.x) < guard && abs(lps[i].pt3.y - pt3.y) < guard);
			bool match01 = (lps[i].branch == b1 && abs(lps[i].pt0.x - pt3.x) < guard && abs(lps[i].pt0.y - pt3.y) < guard);
			bool match10 = (lps[i].ano_bra == b0 && abs(lps[i].pt3.x - pt0.x) < guard && abs(lps[i].pt3.y - pt0.y) < guard);
			if (lps[i].branch == b0 && lps[i].ano_bra == b1)
				return i;
			if (match00 || match01 || match10 || match11 && b1 != INVALID_BRANCH)
				ret_match = 0x20000000;
		}
		return ret_match;
	}
public:
	JointPoint() {
	}

	JointPoint(const LinkPoint & cp) {
		lps.push_back(cp);
		bs.push_back(cp.branch);
		if (cp.ano_bra != INVALID_BRANCH)
			bs.push_back(cp.ano_bra);
	}

	/*
	Input cp, link point to check
	Input guard, check if cp is belong to this joint point
	Return 0: no merge
	1: total merge
	2: half merge
	*/
	int merge(const LinkPoint & cp, int guard, JointPoint * pjp, bool force = false) {
		int idx = have(cp, guard);
		if (idx < 0) {
			if (force) {
				lps.push_back(cp);
				if (!EXIST(bs, cp.branch))
					bs.push_back(cp.branch);
				if (cp.ano_bra != INVALID_BRANCH)
				if (!EXIST(bs, cp.ano_bra))
					bs.push_back(cp.ano_bra);
			}
			return 0;
		}
		if (pjp == NULL) {
			if (idx > lps.size()) { //not exist same link
				lps.push_back(cp); //push new link point
				if (!EXIST(bs, cp.branch)) { //push cp.branch  
					if (bs[0] != cp.ano_bra) {
						for (int i = 1; i < (int)bs.size(); i++)
						if (bs[i] == cp.ano_bra)
							swap(bs[0], bs[i]);
					}
					CV_Assert(bs[0] == cp.ano_bra); //make cp.ano_bra head for easy draw_mark2
					bs.push_back(cp.branch);
				}
				else
				if (!EXIST(bs, cp.ano_bra)) { //push cp.ano_bra
					if (bs[0] != cp.branch) {
						for (int i = 1; i < (int)bs.size(); i++)
						if (bs[i] == cp.branch)
							swap(bs[0], bs[i]);
					}
					CV_Assert(bs[0] == cp.branch); //make cp.branch head for easy draw_mark2
					if (cp.ano_bra != INVALID_BRANCH)
						bs.push_back(cp.ano_bra);
				}
				return 2;
			}
			else { //exist same link
				if (lps[idx].len > cp.len && cp.ano_bra != INVALID_BRANCH ||
					lps[idx].len < cp.len && cp.ano_bra == INVALID_BRANCH)
					lps[idx] = cp; //replace existing link point
				return 1;
			}
		}
		else { //already half merge, now merge two joint point            
			for (int i = 0; i < lps.size(); i++)
				pjp->merge(lps[i], guard, NULL, true);
			return 2;
		}
	}

};

#define IS_NOT_INTERSECT(a1, a2, b1, b2) ((a1) > (b2) || (b1) > (a2))

struct BranchMeta {
#define GET_MINMAX(first, second, min, max) if (first > second) {min=second, max=first;} else {max=second, min=first;}
	enum {
		ALEADY_OUTPUT, //means deleted
		READY_OUTPUT, //before output, will be transfer to ALEADY_OUTPUT soon
		MERGED, //means merged, and will be transfer to ALEADY_OUTPUT automatically
		FPFD, //fix point, fix direction, means final_out not empty
		MainBranch,
		SubBranch
	};
	pair<Point, Point> bch; //branch start and end point
	int dir; //dir=-1, this layer via, dir=-2 down layer via
	int map_bra; //if other BranchMeta merge me, map_bra point to that one.
	int state, idx;
	int fa_idx; //fa_idx point to mainbch, it also means connect to local network
	float prob;
	vector<int> dc; //dirrect connect branch
	vector<Point> final_out;

	BranchMeta() {
		idx = -1;
		map_bra = -1;
		prob = 1;
	}
	BranchMeta(int _idx, Point p0, Point p1, int is_via, bool main, int _fa_idx) {
		if (is_via == 0)
			dir = check_dir(p1.x - p0.x, p1.y - p0.y, true);
		else
			dir = -is_via;
		bch.first = p0;
		bch.second = p1;
		if (bch.first.y > bch.second.y || bch.first.y == bch.second.y && bch.first.x > bch.second.x)
			swap(bch.first, bch.second);
		if (main)
			state = MainBranch;
		else
			state = SubBranch;
		map_bra = -1;
		idx = _idx;
		fa_idx = _fa_idx;
		prob = 1;
	}

	void print_info() {
		qInfo("state=%d, dir=%d, idx=%d, map=%d, (%d,%d)->(%d,%d)", state, dir, idx, map_bra,
			bch.first.x, bch.first.y, bch.second.x, bch.second.y);
		string dc_str;
		for (int i = 0; i < dc.size(); i++) {
			char dcs[10];
			sprintf(dcs, "%d,", dc[i]);
			dc_str.append(dcs);
		}
		qInfo("dc=%s", dc_str.c_str());
	}
	void push_dc(int _idx) {
		if (_idx != idx && !EXIST(dc, _idx))
			dc.push_back(_idx);
	}
	void del_dc(int _idx) {
		vector<int>::iterator it = find(dc.begin(), dc.end(), _idx);
		if (it != dc.end())
			dc.erase(it);
	}
	void extend(Point o) {
		int cl = cnear[dir][0] * bch.first.y + cnear[dir][1] * bch.first.x; //line c-value
		int cp = cnear[dir][0] * o.y + cnear[dir][1] * o.x; //via point c-value
		CV_Assert(cl == cp);
		int d2 = dir_2[dir];
		int df = cnear[d2][0] * bch.first.y + cnear[d2][1] * bch.first.x;
		int ds = cnear[d2][0] * bch.second.y + cnear[d2][1] * bch.second.x;
		int dp = cnear[d2][0] * o.y + cnear[d2][1] * o.x;
		if ((ds - dp) * (df - dp) <= 0)
			return;
		if (POINT_DIS(bch.first, o) < POINT_DIS(bch.second, o))
			bch.first = o;
		else
			bch.second = o;
	}
	void push_out(Point p) {
		if (dir < 0) {//via
			if (p != bch.first && !EXIST(final_out, p))
				final_out.push_back(p);
		}
		else { //wire
			CV_Assert(state != ALEADY_OUTPUT);
			state = FPFD;
			if (final_out.empty())
				final_out.push_back(p);
			int loc = (int)final_out.size(); //loc is insert location, following do sort
			if (dir == DIR_LEFT || dir == DIR_RIGHT) {
				CV_Assert(p.y == bch.first.y);
				if (bch.first.x > bch.second.x)
					swap(bch.first.x, bch.second.x);
				if (p.x < bch.first.x) {
					if (bch.first.x - p.x > 15) //should be near first.x
						qWarning("push_out extend first x (%d,%d) too long to (%d,%d)", bch.first.x, bch.first.y, p.x, p.y);
					bch.first.x = p.x;
				}
				if (p.x > bch.second.x) {
					if (p.x - bch.second.x > 15) //should be neary second.x
						qWarning("push_out extend second (%d,%d) too long to (%d,%d)", bch.second.x, bch.second.y, p.x, p.y);
					bch.second.x = p.x;
				}
				for (int i = 0; i < final_out.size(); i++)
				if (final_out[i].x >= p.x) {
					loc = (final_out[i].x == p.x) ? -1 : i; //-1 means final_out already exist
					break;
				}
			}
			else {
				if (dir == DIR_UP || dir == DIR_DOWN) {
					CV_Assert(p.x == bch.first.x);
					if (bch.first.y > bch.second.y)
						swap(bch.first.y, bch.second.y);
					if (p.y < bch.first.y) {
						if (bch.first.y - p.y > 15) //should be near first.y
							qWarning("push_out extend first y (%d,%d) too long to (%d,%d)", bch.first.x, bch.first.y, p.x, p.y);
						bch.first.y = p.y;
					}
					if (p.y > bch.second.y) {
						if (p.y - bch.second.y > 15) //should be neary second.y
							qWarning("push_out extend second y (%d,%d) too long to (%d,%d)", bch.second.x, bch.second.y, p.x, p.y);
						bch.second.y = p.y;
					}
				}
				for (int i = 0; i < final_out.size(); i++)
				if (final_out[i].y >= p.y) {
					loc = (final_out[i].y == p.y) ? -1 : i; //-1 means final_out already exist
					break;
				}
			}
			if (loc >= 0)
				final_out.insert(final_out.begin() + loc, p);
		}
	}
	/*
	Input other, it is parallel with this
	Output d0, d1, difference length
	Output o, overlap length
	*/
	void parallel_overlap(const BranchMeta & other, int & d0, int & o, int & d1) {
		CV_Assert(dir >= 0 && (dir == other.dir || dir_1[dir] == other.dir));
		int  a1, a2, b1, b2;
		switch (dir) {
		case DIR_UP:
		case DIR_DOWN:
			GET_MINMAX(bch.first.y, bch.second.y, a1, a2);
			GET_MINMAX(other.bch.first.y, other.bch.second.y, b1, b2);
			break;
		default:
			GET_MINMAX(bch.first.x, bch.second.x, a1, a2);
			GET_MINMAX(other.bch.first.x, other.bch.second.x, b1, b2);
			break;
		}
		if (IS_NOT_INTERSECT(a1, a2, b1, b2)) {
			d0 = (a1 > b1) ? b2 - b1 : a2 - a1;
			d1 = (a1 > b1) ? a2 - a1 : b2 - b1;
			o = 0;
		}
		else {
			d0 = a1 - b1;
			d1 = b2 - a2;
			o = min(a2, b2) - max(a1, b1);
		}
	}
	/*
	Input other, it is parallel with this
	Output new_bm, new_bm intersect with this and other
	*/
	bool parallel_intersect(const BranchMeta & other, BranchMeta & new_bm) {
		CV_Assert(dir >= 0 && (dir == other.dir || dir_1[dir] == other.dir));
		int  a1, a2, b1, b2, c1;
		new_bm.dir = dir_2[dir];
		new_bm.state = SubBranch;
		switch (dir) {
		case DIR_UP:
		case DIR_DOWN:
			if (bch.first.x == other.bch.first.x)
				return false;
			GET_MINMAX(bch.first.y, bch.second.y, a1, a2);
			GET_MINMAX(other.bch.first.y, other.bch.second.y, b1, b2);
			if (IS_NOT_INTERSECT(a1, a2, b1, b2)) {
				qWarning("parallel_intersect not intersect dir=%d, line1 (%d,%d) (%d,%d) and line2 (%d,%d) (%d,%d)", dir,
					bch.first.x, bch.first.y, bch.second.x, bch.second.y,
					other.bch.first.x, other.bch.first.y, other.bch.second.x, other.bch.second.y);
			}
			if (b2 - a1 < a2 - b1)
				c1 = (b2 + a1) / 2;
			else
				c1 = (a2 + b1) / 2;
			new_bm.bch.first.x = bch.first.x;
			new_bm.bch.second.x = other.bch.first.x;
			new_bm.bch.first.y = c1;
			new_bm.bch.second.y = c1;
			break;
		case DIR_LEFT:
		case DIR_RIGHT:
			if (bch.first.y == other.bch.first.y)
				return false;
			GET_MINMAX(bch.first.x, bch.second.x, a1, a2);
			GET_MINMAX(other.bch.first.x, other.bch.second.x, b1, b2);
			if (IS_NOT_INTERSECT(a1, a2, b1, b2))
				return false;
			if (b2 - a1 < a2 - b1)
				c1 = (b2 + a1) / 2;
			else
				c1 = (a2 + b1) / 2;
			new_bm.bch.first.y = bch.first.y;
			new_bm.bch.second.y = other.bch.first.y;
			new_bm.bch.first.x = c1;
			new_bm.bch.second.x = c1;
			break;
		default:
			int d1 = dxy[dir][0] * bch.first.y + dxy[dir][1] * bch.first.x;
			int d2 = dxy[dir][0] * other.bch.first.y + dxy[dir][1] * other.bch.first.x;
			if (d1 == d2)
				return false;
			GET_MINMAX(bch.first.x, bch.second.x, a1, a2);
			GET_MINMAX(other.bch.first.x, other.bch.second.x, b1, b2);
			if (IS_NOT_INTERSECT(a1, a2, b1, b2))
				return false;
			if (b2 - a1 < a2 - b1)
				c1 = (b2 + a1) / 2;
			else
				c1 = (a2 + b1) / 2;
			new_bm.bch.first.x = c1;
			new_bm.bch.second.x = c1;
			new_bm.bch.first.y = (d1 - c1 * dxy[dir][1]) * dxy[dir][0];
			new_bm.bch.second.y = (d2 - c1 * dxy[dir][1]) * dxy[dir][0];
			break;
		}
		return true;
	}

	//pri=true, I eat other if in same state
	void merge(const BranchMeta & other, bool pri) {
		if (dir >= 0) { //wire case
			CV_Assert(state > MERGED && other.state > MERGED);
			CV_Assert(dir == other.dir || dir_1[dir] == other.dir);
			if (state == FPFD && other.state == FPFD)
				CV_Assert(cnear[dir][0] * bch.first.y + cnear[dir][1] * bch.first.x ==
				cnear[dir][0] * other.bch.first.y + cnear[dir][1] * other.bch.first.x);
			//1 compute new_bch
			pair<Point, Point> new_bch; //new merge branch
			pair<Point, Point> del_bch;
			if (state < other.state || (state == other.state && pri)) {
				new_bch = bch; //I eat other
				del_bch = other.bch;
			}
			else {
				new_bch = other.bch; //other eat me
				del_bch = bch;
				state = other.state;
			}
			int  minx, maxx, miny, maxy, c;
			switch (dir) {
			case DIR_UP:
			case DIR_DOWN:
				miny = min(min(bch.first.y, bch.second.y), min(other.bch.first.y, other.bch.second.y));
				maxy = max(max(bch.first.y, bch.second.y), max(other.bch.first.y, other.bch.second.y));
				new_bch.first.y = miny;
				new_bch.second.y = maxy;
				break;

			case DIR_LEFT:
			case DIR_RIGHT:
				minx = min(min(bch.first.x, bch.second.x), min(other.bch.first.x, other.bch.second.x));
				maxx = max(max(bch.first.x, bch.second.x), max(other.bch.first.x, other.bch.second.x));
				new_bch.first.x = minx;
				new_bch.second.x = maxx;
				break;
			default:
				minx = min(min(bch.first.x, bch.second.x), min(other.bch.first.x, other.bch.second.x));
				maxx = max(max(bch.first.x, bch.second.x), max(other.bch.first.x, other.bch.second.x));
				c = cnear[dir][0] * bch.first.y + cnear[dir][1] * bch.first.x;
				new_bch.first.x = minx;
				new_bch.first.y = (c - cnear[dir][1] * minx) * cnear[dir][0];
				new_bch.second.x = maxx;
				new_bch.second.y = (c - cnear[dir][1] * maxx) * cnear[dir][0];
			}
			bch = new_bch;
		}
		else { //via case
			CV_Assert(dir == other.dir);
			if (state < other.state || (state == other.state && pri)) { //I eat other				
			}
			else //other eat me
				bch = other.bch;
		}
		//merge dc and final out
		for (int i = 0; i < other.dc.size(); i++)
			push_dc(other.dc[i]);

		if (final_out.empty())
			final_out = other.final_out;
		else {
			for (int i = 0; i < other.final_out.size(); i++)
				push_out(other.final_out[i]);
		}
	}

	void cut(const BranchMeta & other) {
		if (dir < 0)
			return;
		pair<Point, Point> new_bch = bch;
		CV_Assert(dir == other.dir || dir_1[dir] == other.dir);
		switch (dir) {
		case DIR_UP:
		case DIR_DOWN:
			if (new_bch.first.y + new_bch.second.y < other.bch.first.y + other.bch.second.y) {
				new_bch.first.y = min(new_bch.first.y, new_bch.second.y);
				new_bch.second.y = min(other.bch.first.y, other.bch.second.y);
			}
			else {
				new_bch.second.y = max(new_bch.first.y, new_bch.second.y);
				new_bch.first.y = max(other.bch.first.y, other.bch.second.y);
			}
			break;
		case DIR_LEFT:
		case DIR_RIGHT:
			if (new_bch.first.x + new_bch.second.x < other.bch.first.x + other.bch.second.x) {
				new_bch.first.x = min(new_bch.first.x, new_bch.second.x);
				new_bch.second.x = min(other.bch.first.x, other.bch.second.x);
			}
			else {
				new_bch.second.x = max(new_bch.first.x, new_bch.second.x);
				new_bch.first.x = max(other.bch.first.x, other.bch.second.x);
			}
			break;
		default:
			if (new_bch.first.x + new_bch.second.x < other.bch.first.x + other.bch.second.x) {
				new_bch.first.x = min(new_bch.first.x, new_bch.second.x);
				new_bch.second.x = min(other.bch.first.x, other.bch.second.x);
			}
			else {
				new_bch.second.x = max(new_bch.first.x, new_bch.second.x);
				new_bch.first.x = max(other.bch.first.x, other.bch.second.x);
			}
			int c = cnear[dir][0] * bch.first.y + cnear[dir][1] * bch.first.x;
			new_bch.first.y = (c - cnear[dir][1] * new_bch.first.x) * cnear[dir][0];
			new_bch.second.y = (c - cnear[dir][1] * new_bch.second.x) * cnear[dir][0];
			break;
		}
		bch = new_bch;
	}
	/* for two wire case
	if parallel, if distance <= guard return distance  else return -2
	if orthogonal intersected, return -1, if not intersected, return -2
	for two via case
	if distance <= guard return distance  else return -2
	*/
	int intersect(const BranchMeta & other, int guard) {
		int ret;
		if (dir >= 0 && other.dir >= 0) { //two wire case
			int minx, maxx, miny, maxy;
			int ominx, omaxx, ominy, omaxy;
			GET_MINMAX(bch.first.x, bch.second.x, minx, maxx);
			GET_MINMAX(bch.first.y, bch.second.y, miny, maxy);
			GET_MINMAX(other.bch.first.x, other.bch.second.x, ominx, omaxx);
			GET_MINMAX(other.bch.first.y, other.bch.second.y, ominy, omaxy);
			if (dir == other.dir || dir_1[dir] == other.dir) {	//two wire parallel		
				switch (dir) {
				case DIR_UP:
				case DIR_DOWN:
					if (IS_NOT_INTERSECT(miny, maxy, ominy, omaxy)) //overlap y
						return -2;
					else {
						CV_Assert(ominx == omaxx && minx == maxx);
						ret = abs(minx - ominx);
						return ret > guard ? -2 : ret;
					}

				case DIR_LEFT:
				case DIR_RIGHT:
					if (IS_NOT_INTERSECT(minx, maxx, ominx, omaxx)) //overlap x
						return -2;
					else {
						CV_Assert(ominy == omaxy && miny == maxy);
						ret = abs(miny - ominy);
						return ret > guard ? -2 : ret;
					}

				case DIR_UPLEFT:
				case DIR_DOWNRIGHT:
					if (IS_NOT_INTERSECT(minx, maxx, ominx, omaxx)) //overlap x
						return -2;
					else {
						ret = abs(bch.first.x + bch.first.y - other.bch.first.x - other.bch.first.y);
						return ret > guard ? -2 : ret;
					}

				case DIR_UPRIGHT:
				case DIR_DOWNLEFT:
					if (IS_NOT_INTERSECT(minx, maxx, ominx, omaxx)) //overlap x
						return -2;
					else {
						ret = abs(bch.first.x - bch.first.y - other.bch.first.x + other.bch.first.y);
						return ret > guard ? -2 : ret;
					}
				}
			}
			else {//two wire not parallel
				if (IS_NOT_INTERSECT(minx, maxx, ominx, omaxx) || IS_NOT_INTERSECT(miny, maxy, ominy, omaxy))
					return -2;
				int cl = cnear[dir][0] * bch.first.y + cnear[dir][1] * bch.first.x; //line c-value
				int cp1 = cnear[dir][0] * other.bch.first.y + cnear[dir][1] * other.bch.first.x; //point c-value
				int cp2 = cnear[dir][0] * other.bch.second.y + cnear[dir][1] * other.bch.second.x; //point c-value
				if ((cl - cp1) * (cl - cp2) > 0)
					return -2;
				cl = cnear[other.dir][0] * other.bch.first.y + cnear[other.dir][1] * other.bch.first.x; //other line c-value
				cp1 = cnear[other.dir][0] * bch.first.y + cnear[other.dir][1] * bch.first.x;
				cp2 = cnear[other.dir][0] * bch.second.y + cnear[other.dir][1] * bch.second.x;
				if ((cl - cp1) * (cl - cp2) > 0)
					return -2;
				return -1;
			}
		}
		if (dir < 0 && other.dir < 0 && dir == other.dir) { //two via case
			ret = POINT_DIS(bch.first, other.bch.first);
			return ret > guard ? -2 : ret;
		}
		return -2;
	}

	int distance(Point o) {
		if (dir >= 0) {
			int ret;
			int cl = cnear[dir][0] * bch.first.y + cnear[dir][1] * bch.first.x; //line c-value
			int cp = cnear[dir][0] * o.y + cnear[dir][1] * o.x; //via point c-value
			int d2 = dir_2[dir];
			int df = cnear[d2][0] * bch.first.y + cnear[d2][1] * bch.first.x;
			int ds = cnear[d2][0] * bch.second.y + cnear[d2][1] * bch.second.x;
			int dp = cnear[d2][0] * o.y + cnear[d2][1] * o.x;
			if ((ds - dp) * (df - dp) <= 0)
				ret = 0;
			else
				ret = min(abs(ds - dp), abs(df - dp));
			ret += abs(cl - cp);
			return ret;
		}
		else
			return POINT_DIS(bch.first, o);
	}

	int length() {
		return max(abs(bch.first.x - bch.second.x), abs(bch.first.y - bch.second.y));
	}
};
/*
For this layer wire, dir>=0, xy contain all point
For via, dir=-1, xy[0] contain location, xy[1] contain via index and layer
*/
class Branch {
	vector<Point> xy; //(x,y) for main branch in origianl image, it may not be in final main branch
	int idx; //branch index
	int skin_wide, th; //skin_wide and th to search skin
	int dir; //branch dir, if dir=-1, then it is this layer via, -2 for down layer via, here DIR_UP treat same as DIR_DOWN, DIR_LEFT same as DIR_RIGHT
	int type;
	vector<int> children; //sub branch index
	friend class BranchMark;
public:
	Branch() {
		idx = 0;
		skin_wide = 3;
	}

	Branch(int _idx, int _type, int _skin_wide, int _th, int _dir) {
		idx = _idx;
		type = _type;
		skin_wide = _skin_wide;
		th = _th;
		dir = _dir;
	}

	int get_skin_wide() {
		return skin_wide;
	}

	int get_th() {
		return th;
	}

	int get_dir() {
		return dir;
	}

	void push_xy(Point &_xy) {
		xy.push_back(_xy);
	}

	/*
	return which sub_branch has cross point*/
	int get_sub_branch(const vector<BranchMeta> & sbs, const Point & c) {
		int id = -1; //id is which sub_branch to insert cross point
		for (int i = 0; i < (int)children.size(); i++) {
			int d0y = sbs[children[i]].bch.first.y - c.y;
			int d0x = sbs[children[i]].bch.first.x - c.x;
			int d1y = sbs[children[i]].bch.second.y - c.y;
			int d1x = sbs[children[i]].bch.second.x - c.x;

			if (d0y * d1x == d0x * d1y && d0x * d1x <= 0 && d0y * d1y <= 0) {
				id = children[i];
				break;
			}
		}
		return id;
	}

	void compute_main_branch(vector<BranchMeta> & sbs) {
		if (dir >= 0) {
			int avg = 0, minx = 0x7ffff, miny = 0x7ffff, maxx = 0, maxy = 0;
			pair<Point, Point> main_bch;
			for (int i = 0; i < (int)xy.size(); i++) {
				avg += cnear[dir][0] * xy[i].y + cnear[dir][1] * xy[i].x;
				minx = min(xy[i].x, minx);
				maxx = max(xy[i].x, maxx);
				miny = min(xy[i].y, miny);
				maxy = max(xy[i].y, maxy);
			}
			avg = avg / (int)xy.size();
			switch (dir) {
			case DIR_LEFT:
			case DIR_RIGHT:
				main_bch = make_pair(Point(minx, avg), Point(maxx, avg));
				break;
			case DIR_UP:
			case DIR_DOWN:
				main_bch = make_pair(Point(avg, miny), Point(avg, maxy));
				break;
			default:
				main_bch = make_pair(Point((avg - cnear[dir][0] * miny) * cnear[dir][1], miny),
					Point((avg - cnear[dir][0] * maxy) * cnear[dir][1], maxy));
				break;
			}
			sbs[idx] = BranchMeta(idx, main_bch.first, main_bch.second, 0, true, idx);
			if (children.empty())
				children.push_back(idx);
		}
		else {
			sbs[idx] = BranchMeta(idx, xy[0], xy[0], -dir, true, idx);
			if (children.empty())
				children.push_back(idx);
		}
	}
};

class BranchMark {
protected:
	typedef pair<Point, unsigned long long> PPULL;
	vector<Branch> bs;
	Mat mark, mark2;
	vector<PPULL> sq[MARK_MAX_LEN];
	int current_sq;
	vector<JointPoint> jps;
	int oft1[8], oft2[8]; //8 dir offset
	int prev_jp_idx;
	struct CbchLink {
		vector<int> cbch; //local connect branch set based on merge, < local_sbs_num
		vector<int> cbch_link; //first is distance, second is link point, < local_sbs_num
		vector<int> cbch2; //local connect branch set based on distance
	};
	int merge_num;
	friend class SearchWireBranch;
	friend class SearchViaBranch;
	vector<BranchMeta> sbs;
protected:
	//tgt = total - src
	void diff_set(const vector<int> & total, const vector<int> & src, vector<int> & tgt) {
		tgt.clear();
		for (int i = 0; i < (int)total.size(); i++)
		if (!EXIST(src, total[i]))
			tgt.push_back(total[i]);
	}

	/*
	Input b0, b1, branch b0 and b1 meet
	Input p0, p1, sub branch point, p0 belong to b0, p1 belong to b1
	new branch will be added as b0's child
	compute_main_branch mush be called before calling this
	Return ano_bra;
	*/
	int add_sub_branch(int b0, int b1, Point p0, Point p1) {
		int bra = bs[b0].get_sub_branch(sbs, p0); //bra is b0 sub-branch
		CV_Assert(bra >= 0);
		int dir = check_dir(p1.x - p0.x, p1.y - p0.y, true);
		int new_idx = (int)sbs.size();
		if (sbs[bra].dir == dir || sbs[bra].dir == dir_1[dir]) { //p0 and p1 are both in bra, extend bra to p1
			new_idx = bra;
			sbs[bra].extend(p1);
		}
		else { //add new_idx to connect bra, and new_idx belong to b0's child
			sbs[bra].push_dc(new_idx);
			sbs.push_back(BranchMeta(new_idx, p0, p1, 0, false, b0));
			sbs[new_idx].push_dc(bra);
			bs[b0].children.push_back(new_idx);
		}

		if (b1 != b0) { //connect new_idx and ano_bra
			int ano_bra = bs[b1].get_sub_branch(sbs, p1); //ano_bra is b1 sub_branch
			CV_Assert(ano_bra > 0);
			sbs[ano_bra].push_dc(new_idx);
			sbs[new_idx].push_dc(ano_bra);
		}
		return new_idx;
	}

	/*
	Input:p0, p1, from p0 to p1
	Input:branch, draw line branch
	Output: link_bch, which branch is linked
	*/
	void draw_mark2_line(Point p0, Point p1, int branch, vector<int> & link_bch, bool draw_main_bch) {
		if (p0 == p1)
			return;
		Point porg = p0;
		int dir = check_dir(p1.x - p0.x, p1.y - p0.y, true);
		CV_Assert(cnear[dir][0] * p0.y + cnear[dir][1] * p0.x == cnear[dir][0] * p1.y + cnear[dir][1] * p1.x);
		int len = (dir == DIR_UP || dir == DIR_DOWN) ? abs(p1.y - p0.y) : abs(p1.x - p0.x);
		len++;
		unsigned * p_mark2 = mark2.ptr<unsigned>(p0.y, p0.x);
		int oft_0 = (dxy[dir][0] * (int)mark2.step.p[0] + dxy[dir][1] * (int)mark2.step.p[1]) / sizeof(int);//branch dir
		int oft_1 = (dxy[dir_2[dir]][0] * (int)mark2.step.p[0] + dxy[dir_2[dir]][1] * (int)mark2.step.p[1]) / sizeof(int); //branch bone offset
		int oft_2 = (dxy[dir_1[dir_2[dir]]][0] * (int)mark2.step.p[0] + dxy[dir_1[dir_2[dir]]][1] * (int)mark2.step.p[1]) / sizeof(int); //branch bone offset

		for (int i = 0; i < len; i++, p_mark2 += oft_0) {
			if (p_mark2[0] == 0 || MARK2_LEN(p_mark2[0])) {
				p_mark2[0] = MAKE_MARK2(branch, 0, 0, 0); //draw the sub-branch
				if (!draw_main_bch && i == len - 1) {
					CV_Assert(p0 == p1);
					add_sub_branch(branch, branch, porg, p0);
				}
			}
			else { //meet another branch
				int ano_bch = MARK2_BRANCH(p_mark2[0]);
				if (ano_bch != branch) {
					if (!draw_main_bch) { //add sub-branch to connect branch and ano_bch
						if (i != 0) //in i==0 case, add_sub_branch is called in previous calling draw_mark2_line
							add_sub_branch(branch, ano_bch, porg, p0);
						porg = p0;
					}
					if (!EXIST(link_bch, ano_bch))
						link_bch.push_back(ano_bch);
				}
			}
			if (p_mark2[oft_1] == 0)
				p_mark2[oft_1] = MAKE_MARK2(branch, 1, 0, 0);
			if (p_mark2[oft_2] == 0)
				p_mark2[oft_2] = MAKE_MARK2(branch, 1, 0, 0);
			/*for INVALID_BRANCH search
			if (!draw_main_bch && i!=0)
			push_mark(p0, branch, 0, 1, 1, p0.x, p0.y, false);
			*/
			p0 += Point(dxy[dir][1], dxy[dir][0]);
		}
	}

	void draw_mark2_via(Point p0, int branch) {
		unsigned * p_mark2 = mark2.ptr<unsigned>(p0.y, p0.x);
		if (p_mark2[0] != 0 && MARK2_LEN(p_mark2[0]) == 0) {
			int ano_bch = MARK2_BRANCH(p_mark2[0]); //meet another branch
			CV_Assert(branch != ano_bch);
			sbs[branch].push_dc(ano_bch);
			sbs[ano_bch].push_dc(branch);
			add_joint_point(p0, p0, branch, p0, p0, ano_bch, GUARD_FOR_JOINT, 0, 20);
		}
		if (!MARK2_VIA(p_mark2[0]))
			p_mark2[0] = MAKE_MARK2(branch, 0, 0, 1);
	}
	/*
	Input: lines, line set
	input: branch, draw line branch
	Output: link_bch, which branch is linked
	return true if all line are drawed
	*/
	bool draw_sub_branch(vector<pair<Point, int> > lines, vector<int> & link_bch) {
		Point p0 = lines.front().first; //start cross point
		Point p1;
		int branch = MARK2_BRANCH(mark2.at<int>(p0.y, p0.x));
		CV_Assert(branch > 0 && branch < bs.size());
		vector<Point> sb; //sb are subbranch points, following compute sb
		sb.push_back(p0);
		for (int i = 0; i < (int)lines.size() - 1; i++) {
			bool ret = intersect_line(lines[i].first, lines[i].second, lines[i + 1].first, lines[i + 1].second, p1);
			if (!ret)
				return false;
			if (p1 != p0) {
				p0 = p1;
				sb.push_back(p0);
			}
		}
		CV_Assert(p0 != lines.back().first);
		p0 = lines.back().first;
		sb.push_back(p0);

		for (int i = 0; i < (int)sb.size() - 1; i++)
			draw_mark2_line(sb[i], sb[i + 1], branch, link_bch, false);
		return true;
	}
	//output skin xy
	bool get_expand_skin(PPULL & skin) {
		while (1) {
			while (current_sq < MARK_MAX_LEN && sq[current_sq].empty())
				current_sq++;
			if (current_sq >= MARK_MAX_LEN)
				return false;
			while (!sq[current_sq].empty()) {
				skin = sq[current_sq].back();
				sq[current_sq].pop_back();
				if (skin.second == mark.at<unsigned long long>(skin.first.y, skin.first.x))
					return true;
			}
		}
	}

	/*
	Input xy0, b0 branch point
	Input skin0, b0 skin point
	Input xy1, b1 branch point
	Input skin1, b1 skin point
	Input guard, guard for same link point
	Input guard2, for discard link to INVALID BRANCH
	Return 0, new joint point, 1 merge joint point, 2 discard joint point
	*/
	int add_joint_point(const Point & xy0, const Point & skin0, int b0, const Point & xy1, const Point & skin1, int b1, int guard, int len, int guard2) {
		JointPoint * merged_jp = NULL;
		LinkPoint lp(xy0, skin0, skin1, xy1, len, b0, b1);
		if (lp.ano_bra == INVALID_BRANCH && len <= guard2)
			return 2;
		int ret = 0;
		if (prev_jp_idx >= 0) {
			ret = jps[prev_jp_idx].merge(lp, guard, merged_jp);
			if (ret == 1) //found exactly same link point
				merged_jp = &jps[prev_jp_idx];
			if (ret == 2) //link point is merge to jps[i]
				merged_jp = &jps[prev_jp_idx]; //continue to search if need to merge to other joint point			
		}
		if (ret != 1)
		for (int i = 0; i < (int)jps.size(); i++)
		if (i != prev_jp_idx) {
			ret = jps[i].merge(lp, guard, merged_jp);
			if (ret == 1) { //found exactly same link point
				merged_jp = &jps[i];
				prev_jp_idx = i;
				break;
			}
			if (ret == 2) { //link point is merge to jps[i]
				if (merged_jp == NULL) {
					merged_jp = &jps[i]; //continue to search if need to merge to other joint point
					prev_jp_idx = i;
				}
				else {
					jps[i] = jps.back(); //merge and then delete another joint point
					jps.pop_back();
					if (prev_jp_idx == jps.size())
						prev_jp_idx = i;
					break;
				}
			}
		}
		if (merged_jp == NULL) {
			jps.push_back(JointPoint(lp));
			return 0;
		}
		else
			return 1;
	}

public:
	BranchMark() {
		merge_num = 0;
		current_sq = 0;
		bs.push_back(Branch(0, 0, 0, 0, 0));
	}

	void clear_mark(Mat & img) {
		mark.create(img.rows, img.cols, CV_64F);
		for (int y = 0; y < mark.rows; y++) {
			unsigned long long * p_mark = mark.ptr<unsigned long long>(y);
			for (int x = 0; x < mark.cols; x++)
				p_mark[x] = 0xffffffffffffffffULL;
		}
		mark2.create(img.rows, img.cols, CV_32S);
		mark2 = Scalar::all(0);

		for (int i = 0; i < 8; i++) {
			oft1[i] = (dxy[i][0] * (int)mark.step.p[0] + dxy[i][1] * (int)mark.step.p[1]) / sizeof(unsigned long long);
			oft2[i] = (dxy[i][0] * (int)mark2.step.p[0] + dxy[i][1] * (int)mark2.step.p[1]) / sizeof(int);
		}
	}
	const Mat & get_mark() {
		return mark;
	}
	const Mat & get_mark2() {
		return mark2;
	}
	vector<BranchMeta> & get_sbs() {
		return sbs;
	}
	/*
	input xy, mark xy
	input branch_idx
	input len, distance to branch, if len=0, this point is branch
	input dir, how to reach branch, it's dir mask
	input is_skin, if 1, this point is skin, else it is bone.
	via_center overwrite everything except previous via_center,
	*/
	bool push_mark(Point xy, int branch_idx, int len, int dir, int is_skin, int org_x, int org_y, bool is_via) {
		CV_Assert(xy.y < mark.rows && xy.x < mark.cols && len < MARK_MAX_LEN);
		if (dir == 0) //means via center
			CV_Assert(is_via && len == 0);
		bool ret = false;

		unsigned long long * p_mark = mark.ptr<unsigned long long>(xy.y, xy.x);
		unsigned long long prev_mark = p_mark[0];
		int prev_len = MARK_LEN(prev_mark);
		int prev_isskin = MARK_ISSKIN(prev_mark);
		int prev_dir = MARK_DIRMASK(prev_mark);
		int prev_branch = MARK_BRANCH(prev_mark);
		if (prev_dir == 0) //via center
			return false;
		if (prev_len > len || dir == 0) { //shorter or via center
			p_mark[0] = MAKE_MARK(branch_idx, len, dir, is_skin, org_x, org_y);
			ret = true;
		}
		else
		if (prev_isskin == is_skin && prev_branch == branch_idx && prev_len == len && (dir & prev_dir) == 0) {
			p_mark[0] = MAKE_MARK(branch_idx, len, dir | prev_dir, is_skin, org_x, org_y);
			ret = true;
		}

		if (ret && is_skin) {
			sq[len].push_back(make_pair(xy, p_mark[0]));
			current_sq = min(current_sq, len);
		}
		return ret;
	}

	void remark_border_skin(int border) {
		Rect l_r(0, 0, border, mark.rows);
		Rect t_r(0, 0, mark.cols, border);

		for (int i = (int)bs.size(); i < (int)sbs.size(); i++)
		if (l_r.contains(sbs[i].bch.first) || l_r.contains(sbs[i].bch.second) ||
			t_r.contains(sbs[i].bch.first) || t_r.contains(sbs[i].bch.second)) {
			vector<Point> pts;
			get_line_pts(sbs[i].bch.first, sbs[i].bch.second, pts);
			for (int j = 0; j < pts.size(); j++)
			if (l_r.contains(pts[j]) || t_r.contains(pts[j]))
				push_mark(pts[j], i, 0, 0x80, 1, pts[j].x, pts[j].y, false);
		}
	}

	bool is_sub_branch(int branch) {
		return (branch >= bs.size() && branch < sbs.size());
	}

	/*input allow45 allow 45 degree line
	input guard, pass to add_joint_point
	input guard2, pass to add_joint_point
	*/
	void search_cross_points(bool allow45, bool search_inv, int guard, int guard2) {
		CV_Assert(guard2 >= 10);
		jps.clear();
		prev_jp_idx = -1;
#define DISTANCE_BASE(m0, m1) ((MARK_BRANCH(m0) ==INVALID_BRANCH ? 0 : MARK_LEN(m0)) + (MARK_BRANCH(m1) ==INVALID_BRANCH ? 0 : MARK_LEN(m1)))
#define DISTANCE_LR(m0, m1) (DISTANCE_BASE(m0, m1) + 1 + ((MARK_DIRMASK(m0) & DIR_LEFT1_MASK || MARK_LEN(m0)==0) ? 0 : 2) + ((MARK_DIRMASK(m1) & DIR_RIGHT1_MASK || MARK_LEN(m1)==0) ? 0 : 2))
#define DISTANCE_UD(m0, m1) (DISTANCE_BASE(m0, m1) + 1 + ((MARK_DIRMASK(m0) & DIR_UP1_MASK || MARK_LEN(m0)==0) ? 0 : 2) + ((MARK_DIRMASK(m1) & DIR_DOWN1_MASK || MARK_LEN(m1)==0) ? 0 : 2))
#define DISTANCE_LU_RD(m0, m1) (DISTANCE_BASE(m0, m1) + 2 + ((MARK_DIRMASK(m0) & DIR_UPLEFT_MASK || MARK_LEN(m0)==0) ? 0 : 2) + ((MARK_DIRMASK(m1) & DIR_DOWNRIGHT_MASK || MARK_LEN(m1)==0) ? 0 : 2))
#define DISTANCE_RU_LD(m0, m1) (DISTANCE_BASE(m0, m1) + 2 + ((MARK_DIRMASK(m0) & DIR_UPRIGHT_MASK || MARK_LEN(m0)==0) ? 0 : 2) + ((MARK_DIRMASK(m1) & DIR_DOWNLEFT_MASK || MARK_LEN(m1)==0) ? 0 : 2))

		for (int y = 1; y < mark.rows - 1; y++) {
			unsigned long long * p_mark_u = mark.ptr<unsigned long long>(y - 1);
			unsigned long long * p_mark = mark.ptr<unsigned long long>(y);
			unsigned long long * p_mark_d = mark.ptr<unsigned long long>(y + 1);
			for (int x = 1; x < mark.cols - 1; x++) {
				int bidx = MARK_BRANCH(p_mark[x]);
				if (bidx != INVALID_BRANCH) {
					if (bidx < MARK_BRANCH(p_mark[x - 1]) && ((search_inv && MARK_BRANCH(p_mark[x - 1]) == INVALID_BRANCH) ||
						(!search_inv && MARK_BRANCH(p_mark[x - 1]) != INVALID_BRANCH)))
						add_joint_point(POINT_M(p_mark[x]), Point(x, y), MARK_BRANCH(p_mark[x]), POINT_M(p_mark[x - 1]),
						Point(x - 1, y), MARK_BRANCH(p_mark[x - 1]), guard, DISTANCE_LR(p_mark[x - 1], p_mark[x]), guard2);
					if (bidx < MARK_BRANCH(p_mark[x + 1]) && ((search_inv && MARK_BRANCH(p_mark[x + 1]) == INVALID_BRANCH) ||
						(!search_inv && MARK_BRANCH(p_mark[x + 1]) != INVALID_BRANCH)))
						add_joint_point(POINT_M(p_mark[x]), Point(x, y), MARK_BRANCH(p_mark[x]), POINT_M(p_mark[x + 1]),
						Point(x + 1, y), MARK_BRANCH(p_mark[x + 1]), guard, DISTANCE_LR(p_mark[x], p_mark[x + 1]), guard2);
					if (bidx < MARK_BRANCH(p_mark_u[x]) && ((search_inv && MARK_BRANCH(p_mark_u[x]) == INVALID_BRANCH) ||
						(!search_inv && MARK_BRANCH(p_mark_u[x]) != INVALID_BRANCH)))
						add_joint_point(POINT_M(p_mark[x]), Point(x, y), MARK_BRANCH(p_mark[x]), POINT_M(p_mark_u[x]),
						Point(x, y - 1), MARK_BRANCH(p_mark_u[x]), guard, DISTANCE_UD(p_mark_u[x], p_mark[x]), guard2);
					if (bidx < MARK_BRANCH(p_mark_d[x]) && ((search_inv && MARK_BRANCH(p_mark_d[x]) == INVALID_BRANCH) ||
						(!search_inv && MARK_BRANCH(p_mark_d[x]) != INVALID_BRANCH)))
						add_joint_point(POINT_M(p_mark[x]), Point(x, y), MARK_BRANCH(p_mark[x]), POINT_M(p_mark_d[x]),
						Point(x, y + 1), MARK_BRANCH(p_mark_d[x]), guard, DISTANCE_UD(p_mark[x], p_mark_d[x]), guard2);
					if (allow45) {
						if (bidx < MARK_BRANCH(p_mark_u[x - 1]) && ((search_inv && MARK_BRANCH(p_mark_u[x - 1]) == INVALID_BRANCH) ||
							(!search_inv && MARK_BRANCH(p_mark_u[x - 1]) != INVALID_BRANCH))) {
							bool not_connect = false; //not_connect = true means 3 branch join
							if (MARK_BRANCH(p_mark_u[x]) != INVALID_BRANCH && MARK_BRANCH(p_mark_u[x]) != bidx
								&& MARK_BRANCH(p_mark_u[x]) != MARK_BRANCH(p_mark_u[x - 1])) //check for 3 branch join
								not_connect = true;
							if (MARK_BRANCH(p_mark[x - 1]) != INVALID_BRANCH && MARK_BRANCH(p_mark[x - 1]) != bidx
								&& MARK_BRANCH(p_mark[x - 1]) != MARK_BRANCH(p_mark_u[x - 1])) //check for 3 branch join
								not_connect = true;
							if (!not_connect)
								add_joint_point(POINT_M(p_mark[x]), Point(x, y), MARK_BRANCH(p_mark[x]), POINT_M(p_mark_u[x - 1]),
								Point(x - 1, y - 1), MARK_BRANCH(p_mark_u[x - 1]), guard, DISTANCE_LU_RD(p_mark_u[x - 1], p_mark[x]), guard2);
						}
						if (bidx < MARK_BRANCH(p_mark_d[x + 1]) && ((search_inv && MARK_BRANCH(p_mark_d[x + 1]) == INVALID_BRANCH) ||
							(!search_inv && MARK_BRANCH(p_mark_d[x + 1]) != INVALID_BRANCH))) {
							bool not_connect = false;
							if (MARK_BRANCH(p_mark_d[x]) != INVALID_BRANCH && MARK_BRANCH(p_mark_d[x]) != bidx
								&& MARK_BRANCH(p_mark_d[x]) != MARK_BRANCH(p_mark_d[x + 1])) //check for 3 branch join
								not_connect = true;
							if (MARK_BRANCH(p_mark[x + 1]) != INVALID_BRANCH && MARK_BRANCH(p_mark[x + 1]) != bidx
								&& MARK_BRANCH(p_mark[x + 1]) != MARK_BRANCH(p_mark_d[x + 1]))
								not_connect = true;
							if (!not_connect)
								add_joint_point(POINT_M(p_mark[x]), Point(x, y), MARK_BRANCH(p_mark[x]), POINT_M(p_mark_d[x + 1]),
								Point(x + 1, y + 1), MARK_BRANCH(p_mark_d[x + 1]), guard, DISTANCE_LU_RD(p_mark[x], p_mark_d[x + 1]), guard2);
						}
						if (bidx < MARK_BRANCH(p_mark_u[x + 1]) && ((search_inv && MARK_BRANCH(p_mark_u[x + 1]) == INVALID_BRANCH) ||
							(!search_inv && MARK_BRANCH(p_mark_u[x + 1]) != INVALID_BRANCH))) {
							bool not_connect = false;
							if (MARK_BRANCH(p_mark_u[x]) != INVALID_BRANCH && MARK_BRANCH(p_mark_u[x]) != bidx
								&& MARK_BRANCH(p_mark_u[x]) != MARK_BRANCH(p_mark_u[x + 1])) //check for 3 branch join
								not_connect = true;
							if (MARK_BRANCH(p_mark[x + 1]) != INVALID_BRANCH && MARK_BRANCH(p_mark[x + 1]) != bidx
								&& MARK_BRANCH(p_mark[x + 1]) != MARK_BRANCH(p_mark_u[x + 1]))
								not_connect = true;
							if (!not_connect)
								add_joint_point(POINT_M(p_mark[x]), Point(x, y), MARK_BRANCH(p_mark[x]), POINT_M(p_mark_u[x + 1]),
								Point(x + 1, y - 1), MARK_BRANCH(p_mark_u[x + 1]), guard, DISTANCE_RU_LD(p_mark_u[x + 1], p_mark[x]), guard2);
						}
						if (bidx < MARK_BRANCH(p_mark_d[x - 1]) && ((search_inv && MARK_BRANCH(p_mark_d[x - 1]) == INVALID_BRANCH) ||
							(!search_inv && MARK_BRANCH(p_mark_d[x - 1]) != INVALID_BRANCH))) {
							bool not_connect = false;
							if (MARK_BRANCH(p_mark_d[x]) != INVALID_BRANCH && MARK_BRANCH(p_mark_d[x]) != bidx
								&& MARK_BRANCH(p_mark_d[x]) != MARK_BRANCH(p_mark_d[x - 1])) //check for 3 branch join
								not_connect = true;
							if (MARK_BRANCH(p_mark[x - 1]) != INVALID_BRANCH && MARK_BRANCH(p_mark[x - 1]) != bidx
								&& MARK_BRANCH(p_mark[x - 1]) != MARK_BRANCH(p_mark_d[x - 1]))
								not_connect = true;
							if (!not_connect)
								add_joint_point(POINT_M(p_mark[x]), Point(x, y), MARK_BRANCH(p_mark[x]), POINT_M(p_mark_d[x - 1]),
								Point(x - 1, y + 1), MARK_BRANCH(p_mark_d[x - 1]), guard, DISTANCE_RU_LD(p_mark[x], p_mark_d[x - 1]), guard2);
						}
					}
				}
			}
		}
	}

	int get_skin_wide(int branch)
	{
		if (branch < bs.size())
			return bs[branch].get_skin_wide();
		while (branch != sbs[branch].fa_idx)
			branch = sbs[branch].fa_idx;
		CV_Assert(branch > 0);
		if (branch < bs.size())
			return bs[branch].get_skin_wide();
		else
			return bs[1].get_skin_wide();
	}
	int get_th(int branch)
	{
		if (branch < bs.size())
			return bs[branch].get_th();
		while (branch != sbs[branch].fa_idx)
			branch = sbs[branch].fa_idx;
		CV_Assert(branch > 0);
		if (branch < bs.size())
			return bs[branch].get_th();
		else
			return bs[1].get_th();
	}
	/*
	input ig
	input allow45
	*/
	void search_skin(const Mat & ig, bool allow45) {
		PPULL skin;
		int dir_limit = allow45 ? 8 : 4;
		while (get_expand_skin(skin)) {
			CV_Assert(MARK_BRANCH(skin.second) != INVALID_BRANCH);
			int swide = get_skin_wide(MARK_BRANCH(skin.second));
			int th = get_th(MARK_BRANCH(skin.second));
			int len0 = MARK_LEN(skin.second) + 1;
			int dir_mask = MARK_DIRMASK(skin.second);
			for (int dir = 0; dir < dir_limit; dir++)
			if (!(1 << dir & dir_mask)) { //fix me, not allow two 45 line meet
				Point sxy = Point(skin.first.x + dxy[dir][1], skin.first.y + dxy[dir][0]);
				if (sxy.x < swide / 2 || sxy.y < swide / 2 || sxy.y + swide - swide / 2 >= ig.rows || sxy.x + swide - swide / 2 >= ig.cols)
					continue;
				int sum = ig.at<int>(sxy.y - swide / 2, sxy.x - swide / 2)
					+ ig.at<int>(sxy.y + swide - swide / 2, sxy.x + swide - swide / 2)
					- ig.at<int>(sxy.y + swide - swide / 2, sxy.x - swide / 2)
					- ig.at<int>(sxy.y - swide / 2, sxy.x + swide - swide / 2);
				CV_Assert(sum >= 0);
				if (sum > th) {
					int len = len0 + ((1 << dir_1[dir] & dir_mask) ? 0 : 2) + (dir > 3 ? 1 : 0);
					len = min(len, MARK_MAX_LEN - 1);
					push_mark(sxy, MARK_BRANCH(skin.second), len, 1 << dir_1[dir], 1, MARK_X(skin.second), MARK_Y(skin.second), false);
				}
			}
		}
	}


	/*
	input: lp, link point pair
	input: src_bch, source branch
	output: path, target path
	inout: tgt_bch, target branch
	*/
	bool search_path(const LinkPoint & lp, vector <int> & src_bch, vector<int> & tgt_bch, vector<pair<Point, int> > & lines, bool allow45) {
		vector<Point> path;
		Point ps = lp.pt1, pt = lp.pt2; //ps is source skin, pt is target skin
		int pt_bch = lp.ano_bra, ps_bch = lp.branch;
		for (int i = 0; i < (int)tgt_bch.size(); i++)
		if (lp.branch == tgt_bch[i]) {
			pt = lp.pt1;
			ps = lp.pt2;
			pt_bch = lp.branch;
			ps_bch = lp.ano_bra;
			break;
		}

		int dir_limit = allow45 ? 8 : 4;
		for (int sch = 0; sch < 2; sch++) { //sch=0, search pt to src_bch; sch=1, search ps to tgt_bch
			Point pt0 = (sch == 0) ? pt : ps; //pt0 is previous point
			Point pt1 = (sch == 0) ? ps : pt; //pt1 is current point
			int pt1_bch = (sch == 0) ? ps_bch : pt_bch;
			if (sch == 1)
				reverse(path.begin(), path.end());
			if (pt1_bch == INVALID_BRANCH)
				continue;
			const vector <int> & exp_bch = (sch == 0) ? src_bch : tgt_bch;
			const vector <int> & org_bch = (sch == 1) ? src_bch : tgt_bch;
			unsigned long long * p_mark = mark.ptr<unsigned long long>(pt1.y, pt1.x);
			unsigned * p_mark2 = mark2.ptr<unsigned>(pt1.y, pt1.x);
			int prev_dir = check_dir(pt1.x - pt0.x, pt1.y - pt0.y, allow45);
			path.push_back(pt1);
			bool navigation_valid = true;
			bool search_failed = false;
			if (p_mark2[0] && MARK2_LEN(p_mark2[0]) == 0) { //pt1 is on branch
				if (EXIST(exp_bch, MARK2_BRANCH(p_mark2[0]))) //reach target
					continue; //reach
				else {
					if (EXIST(org_bch, MARK2_BRANCH(p_mark2[0]))) {//start from orgin branch
						for (int i = 0; i < (int)path.size(); i++) {
							unsigned * p_m2 = mark2.ptr<unsigned>(path[i].y, path[i].x);
							SET_MAKR2_FLAG(p_m2[0], 0);
						}
						path.clear();
						path.push_back(pt1);
					}
					else
						search_failed = true;
				}
			}
			SET_MAKR2_FLAG(p_mark2[0], 1);
			CV_Assert(pt1_bch == MARK_BRANCH(p_mark[0]));
			int extend_step = 0;
			while (1) {
				if (navigation_valid && MARK_LEN(p_mark[0]) <= 5 && sbs[pt1_bch].dir >= 0) {//nearly reach target
					navigation_valid = false;
					prev_dir = find_nearest_dir(sbs[pt1_bch].bch, pt1, prev_dir, allow45);
				}
				if (!navigation_valid) {
					int dir = sbs[pt1_bch].dir;
					int cl = cnear[dir][0] * sbs[pt1_bch].bch.first.y + cnear[dir][1] * sbs[pt1_bch].bch.first.x; //line c-value
					int cp = cnear[dir][0] * pt1.y + cnear[dir][1] * pt1.x; //point c-value

					if (cl == cp) {
						int d1 = POINT_DIS(pt1, sbs[pt1_bch].bch.first);
						int d2 = POINT_DIS(pt1, sbs[pt1_bch].bch.second);
						if (d1 > 5 && d2 > 5) { //can't extend sbs
							qCritical("search path extend point (%d,%d) can't reach (%x,%x) (%x,%x)", pt1.x, pt1.y,
								sbs[pt1_bch].bch.first.x, sbs[pt1_bch].bch.first.y, sbs[pt1_bch].bch.second.x, sbs[pt1_bch].bch.second.y);
							search_failed = true;
						}
						else {
							vector <Point> extend;
							if (d1 < d2) {
								get_line_pts(pt1, sbs[pt1_bch].bch.first, extend);
								extend.push_back(sbs[pt1_bch].bch.first);
							}
							else {
								get_line_pts(pt1, sbs[pt1_bch].bch.second, extend);
								extend.push_back(sbs[pt1_bch].bch.second);
							}
							path.insert(path.end(), extend.begin(), extend.end());
							break;
						}
					}
					extend_step++;
					if (extend_step > 15) {
						qCritical("search path failed with big extend_step, from (x=%d,y=%d) to (%d,%d)", lp.pt0.x, lp.pt0.y, lp.pt3.x, lp.pt3.y);
						search_failed = true;
					}
				}
				if (search_failed) {
					for (int i = 0; i < (int)path.size(); i++) {
						unsigned * p_m2 = mark2.ptr<unsigned>(path[i].y, path[i].x);
						SET_MAKR2_FLAG(p_m2[0], 0);
					}
					return false; //search fail
				}
				int best_score = 0, best_dir = -1;
				for (int dir = 0; dir < dir_limit; dir++)
				if (MARK2_FLAG(p_mark2[oft2[dir]]) == 0) {
					int score = 0;
					if (p_mark2[oft2[dir]]) {
						if (EXIST(exp_bch, MARK2_BRANCH(p_mark2[oft2[dir]]))) //reach target
							score += MARK2_LEN(p_mark2[oft2[dir]]) ? 1 << 8 : 1 << 9;
						else
							score -= 1 << 10;
					}
					if (score < 1 << 8 && score >= 0 && navigation_valid) {
						if (MARK_LEN(p_mark[oft1[dir]]) == 0) {
							if (EXIST(exp_bch, MARK_BRANCH(p_mark[oft1[dir]]))) //nearly reach target
								score += 1 << 7;
						}

						score += (1 << dir & MARK_DIRMASK(p_mark[0])) ? 1 << 6 : 0; //in dir to target

						if (MARK_BRANCH(p_mark[oft1[dir]]) == pt1_bch) { //in target branch
							if (MARK_LEN(p_mark[oft1[dir]]) < MARK_LEN(p_mark[0])) //len to target become smaller
								score += 1 << 5;
							if (MARK_LEN(p_mark[oft1[dir]]) == MARK_LEN(p_mark[0])) //len to target is same
								score += 1 << 4;
						}
						else {
							if (MARK_BRANCH(p_mark[oft1[dir]]) == INVALID_BRANCH) //not allow dir
								score -= 1 << 10;
							else
							if (!EXIST(exp_bch, MARK_BRANCH(p_mark[oft1[dir]])))
								score -= 1 << 10; //meet non-expected branch
							else {
								if (MARK_LEN(p_mark[oft1[dir]]) < MARK_LEN(p_mark[0]))
									score += 1 << 3; //meet expected branch
								else
									score += 1;
							}
						}
					}
					score += (dir == prev_dir) ? 1 << 2 : 0; //in prev dir
					if (best_score < score) {
						best_score = score;
						best_dir = dir;
					}
				}
				if (best_score > 0) {
					pt0 = pt1;
					pt1 += Point(dxy[best_dir][1], dxy[best_dir][0]);
					p_mark += oft1[best_dir];
					p_mark2 += oft2[best_dir];
					prev_dir = best_dir;
					path.push_back(pt1);
					SET_MAKR2_FLAG(p_mark2[0], 1);
					if (best_score & 1 << 9) //reach target, stop					
						break;
					if (navigation_valid)
						pt1_bch = MARK_BRANCH(p_mark[0]);
				}
				else { //don't know how to go
					qInfo("search path failed, don't know next step, from (x=%d,y=%d) to (%d,%d)", lp.pt0.x, lp.pt0.y, lp.pt3.x, lp.pt3.y);
					for (int i = 0; i < (int)path.size(); i++) {
						unsigned * p_m2 = mark2.ptr<unsigned>(path[i].y, path[i].x);
						SET_MAKR2_FLAG(p_m2[0], 0);
					}
					return false; //search fail
				}
			}
		}
		//here we find path
		for (int i = 0; i < (int)path.size(); i++) {
			unsigned * p_m2 = mark2.ptr<unsigned>(path[i].y, path[i].x);
			SET_MAKR2_FLAG(p_m2[0], 0);
		}
		approximate_line(path, lines, allow45, true, 2, 5);
		return true;
	}

	void draw_mark2(bool draw_main_bch, bool allow45) {
		if (draw_main_bch) {
			sbs.reserve(bs.size() * 2);
			sbs.resize(bs.size());
			for (int i = 1; i < (int)bs.size(); i++) {	
				CV_Assert(bs[i].idx == i);
				if (bs[i].dir >= 0) { //draw main branch
					bs[i].compute_main_branch(sbs);
					vector<int> link_bch;					
					draw_mark2_line(sbs[bs[i].idx].bch.first, sbs[bs[i].idx].bch.second, bs[i].idx, link_bch, true);
					CV_Assert(bs[i].idx == i);
					for (int j = 0; j < link_bch.size(); j++) {
						sbs[i].push_dc(link_bch[j]);
						sbs[link_bch[j]].push_dc(i);
						Point pis;
						bool ret=intersect_line(sbs[i].bch.first, sbs[i].dir, sbs[link_bch[j]].bch.first, sbs[link_bch[j]].dir, pis);
						if (ret)
							add_joint_point(pis, pis, i, pis, pis, link_bch[j], GUARD_FOR_JOINT, 0, 20);
					}
				}
				else {//draw via
					bs[i].compute_main_branch(sbs);
					draw_mark2_via(bs[i].xy[0], bs[i].idx);
				}
			}
		}
		for (int i = 0; i < (int)jps.size(); i++) {
			vector<LinkPoint> & lps = jps[i].lps;
			if (jps[i].lps.size() >= JOINT_POINT_MAX)
				continue;
			if (lps.size() > 2)
				sort(lps.begin(), lps.end());
			vector<int> src, tgt;
			src.push_back(jps[i].bs[0]);
			diff_set(jps[i].bs, src, tgt);
			int start_choose = 0;
			while (1) {
				CV_Assert(src.size() + tgt.size() == jps[i].bs.size());
				int choose = -1;
				for (int cho = start_choose; cho < (int)lps.size(); cho++) {
					if (EXIST(src, lps[cho].branch) && (lps[cho].ano_bra == INVALID_BRANCH || EXIST(tgt, lps[cho].ano_bra))) {
						choose = cho;
						break;
					}
					if (EXIST(tgt, lps[cho].branch) && EXIST(src, lps[cho].ano_bra)) {
						choose = cho;
						break;
					}
				}
				if (choose < 0)
					break;
				vector<pair<Point, int> > lines;
				bool found = false;
				if (lps[choose].len <= 8 || POINT_DIS(lps[choose].pt0, lps[choose].pt3) <= 5) { //check if it is already connected
					int branch = lps[choose].branch;
					int ano_bra = lps[choose].ano_bra;
					CV_Assert(branch != ano_bra);
					if (EXIST(sbs[branch].dc, ano_bra))
						found = true;
					else {
						if (sbs[branch].dir >= 0) {
							int dis = sbs[branch].distance(lps[choose].pt3);
							if (dis <= 5) { //two branch is near, direct_connect
								found = true;
								CV_Assert(ano_bra != INVALID_BRANCH);
								sbs[branch].push_dc(ano_bra);
								sbs[ano_bra].push_dc(branch);
							}
						}
					}
					if (found) {
						if (EXIST(src, branch))
							src.push_back(ano_bra);
						else
							src.push_back(branch);
						lps.erase(lps.begin() + choose);
					}
				}
				if (!found) { //not directly connected, search path
					found = search_path(lps[choose], src, tgt, lines, allow45);
					if (!found) { //not path found, search next linkpoint
						start_choose = choose + 1;
						continue;
					}
					bool ret = draw_sub_branch(lines, src);
					lps.erase(lps.begin() + choose);
					CV_Assert(ret);
				}

				diff_set(jps[i].bs, src, tgt);
				start_choose = 0;
			}
			if (!tgt.empty()) {
				qCritical("branch not connected at (x=%d,y=%d)", lps[0].pt0.x, lps[0].pt0.y);
			}
		}
	}

	int new_branch(int type, int skin_wide, int th, int dir) {
		int idx = (int)bs.size();
		bs.push_back(Branch(idx, type, skin_wide, th, dir));
		return idx;
	}

	void release() {
		mark.release();
		mark2.release();
		bs.clear();
		jps.clear();
		for (int i = 0; i < (int) sizeof(sq) / sizeof(sq[0]); i++) {
			vector<PPULL> tmp;
			tmp.swap(sq[i]);
			tmp.clear();
		}
	}


	void stitch_at(Point o) {
		for (int i = 1; i < sbs.size(); i++) {
			sbs[i].bch.first += o;
			sbs[i].bch.second += o;
		}
	}

	void self_check() {
		for (int i = 1; i < sbs.size(); i++) {
			CV_Assert(sbs[i].bch.first.x >= 0 && sbs[i].bch.first.y >= 0 && sbs[i].idx == i);
			CV_Assert(sbs[i].bch.second.x >= 0 && sbs[i].bch.second.y >= 0);
			for (int j = 0; j < sbs[i].dc.size(); j++) {
				int k = sbs[i].dc[j];
				CV_Assert(k != i);

				if (!EXIST(sbs[k].dc, i)) {
					sbs[k].print_info();
					sbs[i].print_info();
					CV_Assert(0);
				}

				if ((sbs[i].dir + 0.5) *(sbs[k].dir + 0.5) < 0) //wire and via 
					continue;
				int ret = sbs[i].intersect(sbs[k], 10);
				if (ret == -2) {
					if (sbs[i].distance(sbs[k].bch.first) > 8 && sbs[i].distance(sbs[k].bch.second) > 8
						&& sbs[k].distance(sbs[i].bch.first) > 8 && sbs[k].distance(sbs[i].bch.second) > 8)
						qWarning("Detect two line not intersect (%d,%d),(%d,%d) and (%d,%d),(%d,%d)",
						sbs[i].bch.first.x, sbs[i].bch.first.y, sbs[i].bch.second.x, sbs[i].bch.second.y,
						sbs[k].bch.first.x, sbs[k].bch.first.y, sbs[k].bch.second.x, sbs[k].bch.second.y);
				}
			}
		}
	}
	/*
	Input right_limit, right border for output, exceed right border won't output
	Input bottom_limit, bottom border for output, exceed bottom border won't output
	end_len_limit, to merge [bch.first, bch.second] with intersection
	get_result output mark_outbij within Rect(0,0,right_limit,bottom_limit)
	*/
	void get_result(int right_limit, int bottom_limit, int end_len_limit, int layer, vector<MarkObj> & obj_sets, vector<ObjProcessHook> & obj_process) {
		self_check();
		//1 following compute intersection
		for (int i = 1; i < (int)sbs.size(); i++) {
			if (sbs[i].state == BranchMeta::ALEADY_OUTPUT) //if it is deleted, continue
				continue;
#if 0
			if (sbs[i].bch.first.x >= 13629 && sbs[i].bch.first.x <= 13650 &&
				sbs[i].bch.first.y >= 13210 && sbs[i].bch.first.y <= 13220 &&
				sbs[i].bch.second.x >= 13629 && sbs[i].bch.second.x <= 13650 &&
				sbs[i].bch.second.y >= 13210 && sbs[i].bch.second.y <= 13220)
				sbs[i].bch.first.y = sbs[i].bch.first.y * 2 - sbs[i].bch.first.y;
#endif
			for (int j = 0; j < sbs[i].dc.size(); j++) {
				int k = sbs[i].dc[j];
				if (k < i) { //compute intersection of sbs[i] and sbs[k] 
					if (sbs[k].state == BranchMeta::ALEADY_OUTPUT) {
						qCritical("(%d,%d) (%d,%d) meet delete line (%d,%d) (%d,%d)",
							sbs[i].bch.first.x, sbs[i].bch.first.y, sbs[i].bch.second.x, sbs[i].bch.second.y,
							sbs[k].bch.first.x, sbs[k].bch.first.y, sbs[k].bch.second.x, sbs[k].bch.second.y);
						continue;
					}
					Point pis;
					if (sbs[i].dir < 0 && sbs[k].dir < 0) { //two via connect
						if (sbs[i].bch.first.x < right_limit && sbs[i].bch.first.y < bottom_limit)
							sbs[k].push_out(sbs[i].bch.first);
						else
						if (sbs[k].bch.first.x < right_limit && sbs[k].bch.first.y < bottom_limit)
							sbs[i].push_out(sbs[k].bch.first);
						continue;
					}
					if (sbs[i].dir >= 0 && sbs[k].dir >= 0) { //two wire branch connect						
						bool connected = intersect_line(sbs[i].bch.first, sbs[i].dir, sbs[k].bch.first, sbs[k].dir, pis);
						if (!connected) { //deal with parallel case
							for (int l = 0; l < sbs[i].dc.size(); l++)
							if (EXIST(sbs[k].dc, sbs[i].dc[l]))
								connected = true;
							if (!connected) {
								BranchMeta bm;
								bool ret = sbs[i].parallel_intersect(sbs[k], bm);
								if (ret) { //create new bm to connect sbs[i] and sbs[k]
									if (bm.bch.first.x >= right_limit && bm.bch.second.x >= right_limit &&
										bm.bch.first.y >= bottom_limit && bm.bch.second.y >= bottom_limit)
										continue;
									j--;
									sbs[i].del_dc(k);
									sbs[k].del_dc(i);
									sbs[i].push_dc((int)sbs.size());
									sbs[k].push_dc((int)sbs.size());
									bm.idx = sbs.size();
									bm.push_dc(i);
									bm.push_dc(k);
									sbs.push_back(bm);
								}
							}
						}
						else { //deal with intersect case
							if (pis.x < right_limit && pis.y < bottom_limit) {
								sbs[i].push_out(pis);
								sbs[k].push_out(pis);
							}
						}
						continue;
					}
					if (sbs[k].dir >= 0 && sbs[i].dir < 0) { //via connect to wire branch
						bool connected = intersect_line(sbs[k].bch.first, sbs[k].dir, sbs[i].bch.first, dir_2[sbs[k].dir], pis);
						CV_Assert(connected);
						if (pis.x < right_limit && pis.y < bottom_limit) {
							sbs[k].push_out(pis);
							sbs[i].push_out(pis);
						}
						continue;
					}
					if (sbs[i].dir >= 0 && sbs[k].dir < 0) { //via connect to wire branch
						bool connected = intersect_line(sbs[i].bch.first, sbs[i].dir, sbs[k].bch.first, dir_2[sbs[i].dir], pis);
						CV_Assert(connected);
						if (pis.x < right_limit && pis.y < bottom_limit) {
							sbs[k].push_out(pis);
							sbs[i].push_out(pis);
						}
						continue;
					}
				}
			}
		}
		for (int i = 1; i < (int)sbs.size(); i++)
		if (sbs[i].bch.first.x < right_limit && sbs[i].bch.second.x < right_limit &&
			sbs[i].bch.first.y < bottom_limit && sbs[i].bch.second.y < bottom_limit &&
			sbs[i].state != BranchMeta::ALEADY_OUTPUT)
			sbs[i].state = BranchMeta::READY_OUTPUT;
		self_check();
		//2 call hook
		for (int i = 0; i < (int)obj_process.size(); i++) {
			obj_process[i].func(this, layer, 0, obj_process[i].cpara);
		}
		MarkObj wire;
		wire.type = OBJ_LINE;
		wire.type2 = LINE_WIRE_AUTO_EXTRACT;
		wire.type3 = layer;
		wire.state = 0;
		MarkObj via;
		via.type = OBJ_POINT;
		via.type2 = POINT_VIA_AUTO_EXTRACT;
		via.type3 = layer;
		via.state = 0;
		for (int i = 1; i < (int)sbs.size(); i++) {
			if (sbs[i].state == BranchMeta::ALEADY_OUTPUT) //if it is deleted, continue
				continue;
			//3.1 following check if sbs[i] can output
			if (sbs[i].state != BranchMeta::READY_OUTPUT)
				continue;
			//2 following create MarkObj
			if (sbs[i].dir >= 0) { //wire
				Point p0, p1;
				int start = 0;
				if (sbs[i].final_out.empty() ||
					sbs[i].final_out[0].x - sbs[i].bch.first.x + sbs[i].final_out[0].y - sbs[i].bch.first.y > end_len_limit) {
					p0 = sbs[i].bch.first;
					start = 0;
				}
				else { //get rid of bch.first because bch.first overlap final_out[0]
					p0 = sbs[i].final_out[0];
					start = 1;
				}
				for (int j = start; j <= sbs[i].final_out.size(); j++) {
					if (j == sbs[i].final_out.size())
						p1 = sbs[i].bch.second;
					else
						p1 = sbs[i].final_out[j];
					if (!sbs[i].final_out.empty() && j == sbs[i].final_out.size() &&
						p1.x - p0.x + p1.y - p0.y < end_len_limit) //get rid of bch.second because bch.second overlap final_out.back
						break;
					wire.prob = sbs[i].prob; // (j == 0 || j == sbs[i].final_out.size()) ? 0.5 : 1;
					wire.p0 = QPoint(p0.x, p0.y);
					wire.p1 = QPoint(p1.x, p1.y);
					if (p0!=p1)
						obj_sets.push_back(wire);
					p0 = p1;
				}
			}
			else { //via
				if (sbs[i].dir == -1) {
					via.prob = 1;
					via.p0 = QPoint(sbs[i].bch.first.x, sbs[i].bch.first.y);
					via.p1 = via.p0;
					obj_sets.push_back(via);
				}
				for (int j = 0; j < sbs[i].final_out.size(); j++) {
					wire.prob = sbs[i].prob;
					wire.p0 = QPoint(sbs[i].bch.first.x, sbs[i].bch.first.y);
					if (sbs[i].final_out[j].x != sbs[i].bch.first.x) {	//connect in x axis
						wire.p1 = QPoint(sbs[i].final_out[j].x, sbs[i].bch.first.y);
						obj_sets.push_back(wire);
						wire.p0 = wire.p1;
					}
					if (sbs[i].final_out[j].y != sbs[i].bch.first.y) { //connect in y axis
						wire.p1 = QPoint(sbs[i].final_out[j].x, sbs[i].final_out[j].y);
						obj_sets.push_back(wire);
					}
				}
			}
			//3 update subBranch state
			sbs[i].state = BranchMeta::ALEADY_OUTPUT;
		}
		release();
	}

	void del_branch(int idx) {
		sbs[idx].state = BranchMeta::ALEADY_OUTPUT;
		for (int i = 0; i < sbs[idx].dc.size(); i++) {
			int other = sbs[idx].dc[i];
			vector<int>::iterator it = find(sbs[other].dc.begin(), sbs[other].dc.end(), idx);
			if (it != sbs[other].dc.end()) //if other contain idx
				sbs[other].dc.erase(it);
		}
		sbs[idx].dc.clear();
	}

	/*Let branch idx's connected branch redirect to idx2, normally used after merge
	sbs[idx2].merge(sbs[idx]);
	replace_barnch(idx, idx2);*/
	void replace_branch(int idx, int idx2) {
		sbs[idx].state = BranchMeta::ALEADY_OUTPUT;
		for (int i = 0; i < sbs[idx].dc.size(); i++) {
			int other = sbs[idx].dc[i];
			vector<int>::iterator it = find(sbs[other].dc.begin(), sbs[other].dc.end(), idx);
			if (it != sbs[other].dc.end()) //if other contain idx
				sbs[other].dc.erase(it);
			sbs[other].push_dc(idx2);
		}
		sbs[idx].dc.clear();
	}

	/*
	Inout o, other sbs
	Input up_merge, up merge or left merge
	Input guard1 for two wire cover distance
	Input guard3 for two wire merge distance
	Input guard4 for two via merge distance
	Input overlap, overlap rect between local and sbs
	Input limit, if up_merge, it is limit_x right; if left merge, it is limit_y bottom*/
	void merge(BranchMark & o, bool up_merge, int guard1, int guard3, int guard4, const Rect & overlap, int bound_min, int limit) {
		bool force = o.merge_num > 0;
		o.merge_num++;
		Point lt = overlap.tl();
		vector<int> ml_set, mv_set; //merge line set and merge via set
		int local_sbs_num = (int)sbs.size();
		//1 following collect local boundary branch ml_set and mv_idx
		int bound_min2 = (up_merge) ? overlap.y : overlap.x;
		int bound_max = (up_merge) ? overlap.y + overlap.height - 1 : overlap.x + overlap.width - 1;
		if (up_merge) {
			int miny, maxy;
			for (int i = 1; i < sbs.size(); i++) {
				GET_MINMAX(sbs[i].bch.first.y, sbs[i].bch.second.y, miny, maxy);
				if (IS_NOT_INTERSECT(miny, maxy, bound_min, bound_max))
					continue;
#if 0
				if (sbs[i].bch.first.x >= 4081 && sbs[i].bch.first.x <= 4087 &&
					sbs[i].bch.first.y >= 26067 && sbs[i].bch.first.y <= 26537 &&
					sbs[i].bch.second.x >= 4081 && sbs[i].bch.second.x <= 4087 &&
					sbs[i].bch.second.y >= 26067 && sbs[i].bch.second.y <= 26537)
					sbs[i].bch.first.x = sbs[i].bch.first.x * 2 - sbs[i].bch.first.x;
#endif
				if (sbs[i].dir >= 0)
					ml_set.push_back(i);
				else
					mv_set.push_back(i);
			}
		}
		else {
			int minx, maxx;
			for (int i = 1; i < sbs.size(); i++) {
				GET_MINMAX(sbs[i].bch.first.x, sbs[i].bch.second.x, minx, maxx);
				if (IS_NOT_INTERSECT(minx, maxx, bound_min, bound_max))
					continue;
#if 0
				if (sbs[i].bch.first.x >= 4081 && sbs[i].bch.first.x <= 4087 &&
					sbs[i].bch.first.y >= 26067 && sbs[i].bch.first.y <= 26537 &&
					sbs[i].bch.second.x >= 4081 && sbs[i].bch.second.x <= 4087 &&
					sbs[i].bch.second.y >= 26067 && sbs[i].bch.second.y <= 26537)
					sbs[i].bch.first.x = sbs[i].bch.first.x * 2 - sbs[i].bch.first.x;
#endif
				if (sbs[i].dir >= 0)
					ml_set.push_back(i);
				else
					mv_set.push_back(i);
			}
		}
		int local_ov_wire_num = (int)ml_set.size();
		int local_ov_via_num = (int)mv_set.size();
		//2 following collect other boundary branch to ml_set, mv_set
		for (int i = 1; i < o.sbs.size(); i++) {
			if (o.sbs[i].state == BranchMeta::ALEADY_OUTPUT) //fetch all left branch
				continue;
			if (force) {
				if (up_merge) {
					int miny, maxy;
					GET_MINMAX(o.sbs[i].bch.first.y, o.sbs[i].bch.second.y, miny, maxy);
					if (IS_NOT_INTERSECT(miny, maxy, bound_min, bound_max)) //print some error, normally it should not happen
						qCritical("fetch line (%d,%d) (%d,%d) not in y boundary (%d, %d)", o.sbs[i].bch.first.x, o.sbs[i].bch.first.y,
						o.sbs[i].bch.second.x, o.sbs[i].bch.second.y, bound_min, bound_max);
				}
				else {
					int minx, maxx;
					GET_MINMAX(o.sbs[i].bch.first.x, o.sbs[i].bch.second.x, minx, maxx);
					if (IS_NOT_INTERSECT(minx, maxx, bound_min, bound_max)) //print some error, normally it should not happen
						qCritical("fetch line (%d,%d) (%d,%d) not in x boundary (%d, %d)", o.sbs[i].bch.first.x, o.sbs[i].bch.first.y,
						o.sbs[i].bch.second.x, o.sbs[i].bch.second.y, bound_min, bound_max);
				}
			}
			else { //fetch boundary branch
				if (up_merge) {
					int miny, maxy;
					GET_MINMAX(o.sbs[i].bch.first.y, o.sbs[i].bch.second.y, miny, maxy);
					CV_Assert(maxy <= bound_max);
					if (IS_NOT_INTERSECT(miny, maxy, bound_min, bound_max))
						continue;
				}
				else {
					int minx, maxx;
					GET_MINMAX(o.sbs[i].bch.first.x, o.sbs[i].bch.second.x, minx, maxx);
					CV_Assert(maxx <= bound_max);
					if (IS_NOT_INTERSECT(minx, maxx, bound_min, bound_max))
						continue;
				}
			}
			BranchMeta & bm = o.sbs[i]; //here bm intersect with boundary
#if 0
			if (bm.bch.first.x >= 4081 && bm.bch.first.x <= 4087 &&
				bm.bch.first.y >= 26067 && bm.bch.first.y <= 26537 &&
				bm.bch.second.x >= 4081 && bm.bch.second.x <= 4087 &&
				bm.bch.second.y >= 26067 && bm.bch.second.y <= 26537)
				bm.bch.first.y = bm.bch.first.y * 2 - bm.bch.first.y;
#endif
			CV_Assert(bm.state != BranchMeta::ALEADY_OUTPUT);
			vector<int> direct_con; //will replace bm.dc
			vector<int> & m_set = (bm.dir >= 0) ? ml_set : mv_set;
			//2.1 recompute direct_con for other boundary branch 
			for (int j = 0; j < bm.dc.size(); j++)
			if (o.sbs[bm.dc[j]].state == BranchMeta::MERGED) {
				int map_bra = o.sbs[bm.dc[j]].map_bra;
				CV_Assert(map_bra >= local_sbs_num);
				direct_con.push_back(map_bra);
			}
			bm.dc.swap(direct_con);
			sbs.push_back(bm);
			sbs.back().idx = (int)sbs.size() - 1;
			sbs.back().map_bra = -1;
			sbs.back().fa_idx = -1; //init as not connect to local
			bm.map_bra = (int)sbs.size() - 1;
			m_set.push_back(bm.map_bra);
			bm.state = BranchMeta::MERGED;
			for (int i = 0; i<(int)bm.dc.size(); i++) {
				int idx = bm.dc[i];
				sbs[idx].push_dc(bm.map_bra);
			}
			bm.dc.swap(direct_con);
		}

		vector<CbchLink> cbch_set(sbs.size() - local_sbs_num);
		//3 now ml_set and mv_set contain all overlap obj, now delete same via
		for (int i = local_ov_via_num; i < (int) mv_set.size(); i++) {
			CV_Assert(mv_set[i] >= local_sbs_num && sbs[mv_set[i]].state !=BranchMeta::ALEADY_OUTPUT);
			int find = -1;
			for (int j = 0; j < local_ov_via_num; j++)
			if (sbs[mv_set[i]].dir == sbs[mv_set[j]].dir && POINT_DIS(sbs[mv_set[i]].bch.first, sbs[mv_set[j]].bch.first) <= guard4) {
				find = mv_set[j];
				break;
			}
			if (find != -1) {
				bool pri;
				if (up_merge) {
					int dy1 = bound_max - sbs[mv_set[i]].bch.first.y;
					int dy2 = sbs[find].bch.first.y - bound_min2;
					pri = (dy1 >= dy2); //y<=(bound_max+bound_min2) /2 
				}
				else {
					int dx1 = bound_max - sbs[mv_set[i]].bch.first.x;
					int dx2 = sbs[find].bch.first.x - bound_min2;
					pri = (dx1 >= dx2); //x<=(bound_max+bound_min2) /2 
				}
				sbs[mv_set[i]].merge(sbs[find], pri);
				sbs[mv_set[i]].fa_idx = find;
				replace_branch(find, mv_set[i]);
				sbs[find].map_bra = mv_set[i];
			}

			Point pt = sbs[mv_set[i]].bch.first - lt;
			if (pt.x >= 0 && pt.y >= 0) {
				unsigned long long m = mark.at<unsigned long long>(pt.y, pt.x);
				if (MARK_BRANCH(m) != INVALID_BRANCH) {
					cbch_set[mv_set[i] - local_sbs_num].cbch.push_back(MARK_BRANCH(m));
					cbch_set[mv_set[i] - local_sbs_num].cbch_link.push_back(MARK_LEN(m));
				}
			}
		}
		//4 delete same wire		
		for (int i = local_ov_wire_num; i < ml_set.size(); i++) {			
			BranchMeta & bm = sbs[ml_set[i]];
			CV_Assert(ml_set[i] >= local_sbs_num && bm.state != BranchMeta::ALEADY_OUTPUT);
#if 0
			if (bm.bch.first.x >= 4081 && bm.bch.first.x <= 4087 &&
				bm.bch.first.y >= 26067 && bm.bch.first.y <= 26537 &&
				bm.bch.second.x >= 4081 && bm.bch.second.x <= 4087 &&
				bm.bch.second.y >= 26067 && bm.bch.second.y <= 26537)
				bm.bch.first.y = bm.bch.first.y * 2 - bm.bch.first.y;
#endif
			pair<Point, Point> line = bm.bch; //other boundary line in local mark
			line.first.x = max(0, line.first.x - lt.x); //TODO: how to do with 45 line
			line.first.y = max(0, line.first.y - lt.y);
			line.second.x = max(0, line.second.x - lt.x);
			line.second.y = max(0, line.second.y - lt.y);
			CV_Assert(line.first.x < overlap.width && line.second.x < overlap.width &&
				line.first.y < overlap.height && line.second.y < overlap.height);
			//4.1 compute local connect branch set
			vector<int> cbch, cbch2; //local connect branch set
			vector<int> cbch_link; //distance
			if (!(line.first.x == 0 && line.second.x == 0 || line.first.y == 0 && line.second.y == 0)) {
				vector<Point> pts;
				get_line_pts(line.first, line.second, pts);
				int prev_branch = -1, prev_cbch_idx;
				for (int j = 0; j < (int)pts.size(); j += 2) { //collect connect branch set by mark
					unsigned long long m = mark.at<unsigned long long>(pts[j].y, pts[j].x);
					if (MARK_BRANCH(m) != INVALID_BRANCH) {
						if (MARK_BRANCH(m) != prev_branch) {
							prev_branch = MARK_BRANCH(m);
							CV_Assert(prev_branch < local_sbs_num);
							prev_cbch_idx = -1;
							for (int i = 0; i < cbch.size(); i++)
							if (cbch[i] == prev_branch) {
								prev_cbch_idx = i;
								break;
							}
							if (prev_cbch_idx < 0) {
								prev_cbch_idx = (int)cbch.size();
								cbch.push_back(prev_branch);
								cbch_link.push_back(MARK_LEN(m));
							}
						}
						CV_Assert(cbch[prev_cbch_idx] == prev_branch);
						if (cbch_link[prev_cbch_idx] >  MARK_LEN(m))
							cbch_link[prev_cbch_idx] = MARK_LEN(m);
					}
				}
			}
			for (int j = 0; j < local_ov_wire_num; j++)
			if (sbs[ml_set[j]].state != BranchMeta::ALEADY_OUTPUT) { //collect connect branch set by distance				
				int ret = sbs[ml_set[j]].intersect(bm, guard3);
				if (ret == -1)
					ret = 0;
				if (ret >= 0) {
					if (ret <= 3) {
						if (!EXIST(cbch, ml_set[j])) {
							cbch.push_back(ml_set[j]);
							cbch_link.push_back(ret);
						}
					}
					else
					if (!EXIST(cbch, ml_set[j]))
						cbch2.push_back(ml_set[j]);
				}
			}
			//4.2 First merge and delete extend line
			for (int j = 0; j < (int)cbch.size(); j++)
			if (bm.dir == sbs[cbch[j]].dir || dir_1[bm.dir] == sbs[cbch[j]].dir) { //parallel
				int k = cbch[j];
				CV_Assert(k < local_sbs_num);
				while (k >= 0 && sbs[k].state == BranchMeta::ALEADY_OUTPUT)
					k = sbs[k].map_bra;
				if (k<0 || k == ml_set[i]) {
					if (k<0)
						qCritical("merge extend found invalid map_bra (%d,%d) (%d,%d)", 
						sbs[cbch[j]].bch.first.x, sbs[cbch[j]].bch.first.y, sbs[cbch[j]].bch.second.x, sbs[cbch[j]].bch.second.y);
					continue;
				}
				int ret = bm.intersect(sbs[k], 1);
				if (ret == 0 || (ret == 1 && sbs[k].length() < mark.cols - 200 && //in local rect
					(bm.state > BranchMeta::FPFD || sbs[k].state > BranchMeta::FPFD))) {
					bm.merge(sbs[k], true);
					bm.fa_idx = cbch[j];
					replace_branch(k, ml_set[i]);
					sbs[k].map_bra = ml_set[i];
					cbch.erase(cbch.begin() + j);
					cbch_link.erase(cbch_link.begin() + j);
					j--;
				}
			}

			//4.3 Then merge and delete cover line
			for (int j = 0; j < (int)cbch.size(); j++)
			if (bm.dir == sbs[cbch[j]].dir || dir_1[bm.dir] == sbs[cbch[j]].dir) { //parallel
				int k = cbch[j];
				while (k >= 0 && sbs[k].state == BranchMeta::ALEADY_OUTPUT)
					k = sbs[k].map_bra;
				if (k<0 || k == ml_set[i]) {
					if (k<0)
						qCritical("merge cover found invalid map_bra (%d,%d) (%d,%d)", 
						sbs[cbch[j]].bch.first.x, sbs[cbch[j]].bch.first.y, sbs[cbch[j]].bch.second.x, sbs[cbch[j]].bch.second.y);
					continue;
				}
				int ret = bm.intersect(sbs[k], guard3);
				if (ret >= 0) {
					int d0, o, d1;
					bm.parallel_overlap(sbs[k], d0, o, d1);
					bool cover = d0 < guard1 && d1 < guard1; //bm cover local
					bool covered = -d0 < guard1 && -d1 < guard1; //local cover bm
					if (cover && sbs[k].state > BranchMeta::FPFD || covered && bm.state > BranchMeta::FPFD) {
						bm.merge(sbs[k], cover && sbs[k].state > BranchMeta::FPFD);
						bm.fa_idx = cbch[j];
						replace_branch(k, ml_set[i]);
						sbs[k].map_bra = ml_set[i];
						cbch.erase(cbch.begin() + j);
						cbch_link.erase(cbch_link.begin() + j);
						j = -1;
					}
				}
			}
			CV_Assert(cbch.size() == cbch_link.size());
			cbch_set[ml_set[i] - local_sbs_num].cbch.swap(cbch);
			cbch_set[ml_set[i] - local_sbs_num].cbch2.swap(cbch2);
			cbch_set[ml_set[i] - local_sbs_num].cbch_link.swap(cbch_link);
		}
		//4.4 Then merge and delete cover line further
		for (int i = local_ov_wire_num; i < (int)ml_set.size(); i++) 
		if (sbs[ml_set[i]].state != BranchMeta::ALEADY_OUTPUT) {
			CV_Assert(ml_set[i] >= local_sbs_num);
			BranchMeta & bm = sbs[ml_set[i]];
			vector<int> & cbch2 = cbch_set[ml_set[i] - local_sbs_num].cbch2; //local connect branch set
			for (int j = 0; j < (int)cbch2.size(); j++)
			if (bm.dir == sbs[cbch2[j]].dir || dir_1[bm.dir] == sbs[cbch2[j]].dir) {
				int k = cbch2[j];
				CV_Assert(k < local_sbs_num);
				while (k >= 0 && sbs[k].state == BranchMeta::ALEADY_OUTPUT)
					k = sbs[k].map_bra;
				if (k < 0 || k == ml_set[i]) {
					if (k<0)
						qCritical("merge parallel found invalid map_bra (%d,%d) (%d,%d)",
						sbs[cbch2[j]].bch.first.x, sbs[cbch2[j]].bch.first.y, sbs[cbch2[j]].bch.second.x, sbs[cbch2[j]].bch.second.y);
					continue;
				}
				bool connected = false;
				for (int m = 0; m < (int)sbs[k].dc.size(); m++)
				if (EXIST(bm.dc, sbs[k].dc[m])) //TODO: this connect check is wrong in some case
					connected = true;
				if (connected) {
					int d0, o, d1;
					bm.parallel_overlap(sbs[k], d0, o, d1);
					bool cover = d0 < guard1 && d1 < guard1; //bm cover local
					bool covered = -d0 < guard1 && -d1 < guard1; //local cover bm
					if (cover && sbs[k].state > BranchMeta::FPFD || covered && bm.state > BranchMeta::FPFD) {
						bm.merge(sbs[k], cover && sbs[k].state > BranchMeta::FPFD);
						bm.fa_idx = cbch2[j];
						replace_branch(k, ml_set[i]);
						sbs[k].map_bra = ml_set[i];
						j = -1;
					}
				}
				else {
					bm.prob = 0.5;
					sbs[k].prob = 0.5;
				}
			}
		}
		qInfo("merge finish delete same");
		//5 Now all same via & wire are deleted, cut parallel intersect line
		for (int i = local_ov_wire_num; i < (int)ml_set.size(); i++) 
		if (sbs[ml_set[i]].state != BranchMeta::ALEADY_OUTPUT){
			CV_Assert(ml_set[i] >= local_sbs_num);
			BranchMeta & bm = sbs[ml_set[i]];
#if 0
			if (bm.bch.first.x >= 4081 && bm.bch.first.x <= 4087 &&
				bm.bch.first.y >= 26067 && bm.bch.first.y <= 26537 &&
				bm.bch.second.x >= 4081 && bm.bch.second.x <= 4087 &&
				bm.bch.second.y >= 26067 && bm.bch.second.y <= 26537)
				bm.bch.first.y = bm.bch.first.y * 2 - bm.bch.first.y;
#endif
			vector<int> & cbch = cbch_set[ml_set[i] - local_sbs_num].cbch; //local connect branch set
			for (int j = 0; j < (int)cbch.size(); j++)
			if (bm.dir == sbs[cbch[j]].dir || dir_1[bm.dir] == sbs[cbch[j]].dir) { //parallel
				int k = cbch[j];
				while (k >= 0 && sbs[k].state == BranchMeta::ALEADY_OUTPUT)
					k = sbs[k].map_bra;
				if (k < 0 || k == ml_set[i]) {
					if (k<0)
						qCritical("merge parallel cut found invalid map_bra (%d,%d) (%d,%d)", 
						sbs[cbch[j]].bch.first.x, sbs[cbch[j]].bch.first.y, sbs[cbch[j]].bch.second.x, sbs[cbch[j]].bch.second.y);
					continue;
				}
				int ret = bm.intersect(sbs[k], guard3);
				if (ret >= 0) {
					int d0, o, d1;
					bm.parallel_overlap(sbs[k], d0, o, d1);
					bool cover = d0 <= 0 && d1 <= 0; //bm cover local
					bool covered = -d0 <= 0 && -d1 <= 0; //local cover bm
					if (!cover && !covered) {
						//5.1 cut overlap
						bm.cut(sbs[k]);
						//5.2 reassign connected dc
						for (int m = 0; m < (int)bm.dc.size(); m++) {
							int nb = bm.dc[m];
							bool cb, ck; //cb means sbs[nb] like connect to bm, ck means sbs[nb] like connect to sbs[k]
							switch (bm.dir) {
							case DIR_UP:
							case DIR_DOWN:
								cb = (sbs[nb].bch.first.y - bm.bch.first.y) * (sbs[nb].bch.first.y - bm.bch.second.y) <= 0;
								ck = (sbs[nb].bch.first.y - sbs[k].bch.first.y) * (sbs[nb].bch.first.y - sbs[k].bch.second.y) <= 0;
								break;
							default:
								cb = (sbs[nb].bch.first.x - bm.bch.first.x) * (sbs[nb].bch.first.x - bm.bch.second.x) <= 0;
								ck = (sbs[nb].bch.first.x - sbs[k].bch.first.x) * (sbs[nb].bch.first.x - sbs[k].bch.second.x) <= 0;
								break;
							}
							if (EXIST(sbs[k].dc, bm.dc[m])) { //nb connect to both bm and sbs[k]								
								if (sbs[nb].dc.size() == 2 && sbs[nb].dir >= 0) { //sbs[nb] only connet to sbs[k] and bm, delete sbs[nb]
									CV_Assert(sbs[nb].dc[0] + sbs[nb].dc[1] == k + ml_set[i]);
									m--;
									bm.del_dc(nb);
									sbs[nb].del_dc(ml_set[i]);
									sbs[k].del_dc(nb);
									sbs[nb].del_dc(k);
									sbs[nb].state = BranchMeta::ALEADY_OUTPUT; //TODO: it can make error merge meet invalid map_bra
									continue;
								}
								if (cb && !ck) { //reassign nb only to bm
									sbs[k].del_dc(nb);
									sbs[nb].del_dc(k);
								}
								else { //reassign nb only to sbs[k]
									m--;
									bm.del_dc(nb);
									sbs[nb].del_dc(ml_set[i]);
								}
							}
							else { //nb connect only to both bm 
								if (ck && !cb) { //reassign nb only to sbs[k]
									m--;
									bm.del_dc(nb);
									sbs[nb].del_dc(ml_set[i]);
									sbs[k].push_dc(nb);
									sbs[nb].push_dc(k);
									if (sbs[nb].fa_idx < 0)
										sbs[nb].fa_idx = cbch[j];
								}
							}
						}
						//5.3 add each to dc
						bm.push_dc(k);
						sbs[k].push_dc(ml_set[i]);
						bm.fa_idx = cbch[j];
					}
				}
			}
		}

		//6 connect remote sbs to local wire and via
		//6.1 Add all already localize wire and via to connected_queue
		vector<int> connected_queue;
		for (int i = local_sbs_num; i < sbs.size(); i++)
		if (sbs[i].fa_idx >= 0) //already connnected to local
			connected_queue.push_back(i);
		//6.2 explore more localize wire and via in connected_queue
		while (!connected_queue.empty()) {
			int i = connected_queue.back();
			connected_queue.pop_back();
			for (int j = 0; j < (int)sbs[i].dc.size(); j++) {
				int k = sbs[i].dc[j];
				if (sbs[k].fa_idx < 0) {
					sbs[k].fa_idx = sbs[i].fa_idx;
					connected_queue.push_back(k); //push new sbs to connected_queue
				}
			}
		}
		//6.3 Try to add dc for unlocalize wire and via
		for (int i = local_sbs_num; i < sbs.size(); i++) {
#if 0
			if (sbs[i].bch.first.x >= 4081 && sbs[i].bch.first.x <= 4087 &&
				sbs[i].bch.first.y >= 26067 && sbs[i].bch.first.y <= 26537 &&
				sbs[i].bch.second.x >= 4081 && sbs[i].bch.second.x <= 4087 &&
				sbs[i].bch.second.y >= 26067 && sbs[i].bch.second.y <= 26537)
				sbs[i].bch.first.x = sbs[i].bch.first.x * 2 - sbs[i].bch.first.x;
#endif
			if (sbs[i].fa_idx < 0 && !cbch_set[i - local_sbs_num].cbch.empty()) {
				vector<int> & cbch = cbch_set[i - local_sbs_num].cbch; //local connect branch set
				vector<int> & cbch_link = cbch_set[i - local_sbs_num].cbch_link; //first is distance, second is link point
				CV_Assert(cbch.size() == cbch_link.size());
				int min_dis = 0xfffff, cn;
				for (int j = 0; j < (int)cbch.size(); j++)
				if (cbch_link[j] < min_dis) { //search min distance
					min_dis = cbch_link[j];
					cn = cbch[j];
				}
				sbs[i].fa_idx = cn;
				while (cn >= 0 && sbs[cn].state == BranchMeta::ALEADY_OUTPUT)
					cn = sbs[cn].map_bra;
				if (cn < 0) {
					cn = sbs[i].fa_idx;
					qCritical("merge add_dc meet invalid map_bra (%d,%d) (%d,%d)",
						sbs[cn].bch.first.x, sbs[cn].bch.first.y, sbs[cn].bch.second.x, sbs[cn].bch.second.y);
					continue;
				}
				sbs[i].push_dc(cn);
				sbs[cn].push_dc(i);
				connected_queue.push_back(i);
				while (!connected_queue.empty()) {
					int ii = connected_queue.back();
					connected_queue.pop_back();
					for (int j = 0; j < (int)sbs[ii].dc.size(); j++) {
						int k = sbs[ii].dc[j];
						if (sbs[k].fa_idx < 0) {
							sbs[k].fa_idx = sbs[ii].fa_idx;
							connected_queue.push_back(k); //push new sbs to connected_queue
						}
					}
				}
			}
		}
		for (int i = local_sbs_num; i < sbs.size(); i++)
		if (sbs[i].fa_idx < 0)
			sbs[i].fa_idx = sbs[i].idx;

		//7 delete double via-wire connected
		for (int j = 0; j < (int)sbs.size(); j++) 
		if (sbs[j].dir<0 && sbs[j].dc.size() == 2) {
			int k1 = sbs[j].dc[0];
			int k2 = sbs[j].dc[1];
			if (sbs[k1].dir >= 0 && sbs[k2].dir >= 0 && sbs[k1].dir!=sbs[k2].dir && dir_1[sbs[k1].dir]!=sbs[k2].dir) {
				int d1 = sbs[k1].distance(sbs[j].bch.first);
				int d2 = sbs[k2].distance(sbs[j].bch.first);
				if (EXIST(sbs[k1].dc, k2)) {						
					if (d2 > 0 && d1 > 0) {
						if (d1 < d2) {
							sbs[j].del_dc(k2);
							sbs[k2].del_dc(j);
						}
						else {
							sbs[j].del_dc(k1);
							sbs[k1].del_dc(j);
						}
					}						
				}
				else {
					if (d2 > 0 && d1 > 0 && d1 <= 3 && d2 <= 3) {
						if (d1 < d2) {
							sbs[j].del_dc(k2);
							sbs[k2].del_dc(j);
						}
						else {
							sbs[j].del_dc(k1);
							sbs[k1].del_dc(j);
						}
						sbs[k1].push_dc(k2);
						sbs[k2].push_dc(k1);
					}
				}
			}
		}
		
		//8 change other sbs too
		for (int i = 1; i < o.sbs.size(); i++)
		if (o.sbs[i].state == BranchMeta::MERGED) {
			o.sbs[i].state = BranchMeta::ALEADY_OUTPUT;
			if ((up_merge && (o.sbs[i].bch.first.x >= limit || o.sbs[i].bch.second.x >= limit)) ||
				(!up_merge && (o.sbs[i].bch.first.y >= limit || o.sbs[i].bch.second.y >= limit))) {
				o.sbs[i].state = BranchMeta::MainBranch;
				int k = o.sbs[i].map_bra;
				while (k >= 0 && sbs[k].state == BranchMeta::ALEADY_OUTPUT)
					k = sbs[k].map_bra;
				if (k < 0) {
					qCritical("Merge meet invalid map_bra (%d,%d) (%d,%d)", 
						o.sbs[i].bch.first.x, o.sbs[i].bch.first.y, o.sbs[i].bch.second.x, o.sbs[i].bch.second.y);
					continue;
				}
				o.sbs[i].bch = sbs[k].bch;
				o.sbs[i].bch.first.x = min(o.sbs[i].bch.first.x, overlap.br().x - 2);
				o.sbs[i].bch.second.x = min(o.sbs[i].bch.second.x, overlap.br().x - 2);
				o.sbs[i].bch.first.y = min(o.sbs[i].bch.first.y, overlap.br().y - 2);
				o.sbs[i].bch.second.y = min(o.sbs[i].bch.second.y, overlap.br().y - 2);
			}
			o.sbs[i].map_bra = -1;
		}

	}
};

class PipeDataPerLayer {
public:
	Mat img, raw_img;
	Mat ig, iig, lg, llg;
	Mat prob;
	int gs;
	int border_size;
	bool allow_45;
	int img_pixel_x0, img_pixel_y0; // it is load image pixel zuobiao
	struct {
		int type;
		Mat d;
	} v[16];

	VWSet vw;
	WireTypeSet wts;
	bool ig_valid;
	int compute_border;
	struct GuardLen{
		int wire_connect, via_connect, end_wire;
		int wire_merge, via_merge;
	} guard_len;
	vector<ViaInfo> via_info; //fine_via_search output
	BranchMark bm;
	//_gs * 2 <= _compute_border
	PipeDataPerLayer() {
		ig_valid = false;
		gs = 4;
		compute_border = 24;
		border_size = 48;
		guard_len.wire_connect = 7;
		guard_len.via_connect = 8;
		guard_len.end_wire = 20;
		guard_len.wire_merge = 6;
		guard_len.via_merge = 7;
		for (int i = 0; i < 16; i++)
			v[i].type = TYPE_NONE;
	}

	void reinit(int _gs, int _compute_border, int _border_size, GuardLen & guard) {
		gs = _gs;
		compute_border = _compute_border;
		border_size = _border_size;

		prob.create((img.rows - 1) / gs + 1, (img.cols - 1) / gs + 1, CV_64FC2);
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols * 2; x++)
				p_prob[x] = 0xffffffffffffffffULL;
		}
		qInfo("via_connect=%d, via_merge=%d, wire_connect=%d, wire_merge=%d, end_wire=%d",
			guard.via_connect, guard.via_merge, guard.wire_connect, guard.wire_merge, guard.end_wire);
		guard_len = guard;
	}

	void set_raw_img(Mat & _img) {
		img = _img;
		raw_img = img.clone();
		ig_valid = false;
	}

	void validate_ig() {
		if (!ig_valid) {
			integral_square(img, ig, iig, lg, llg, true);
			ig_valid = true;
		}
	}

	void check_prob() {
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols * 2; x++)
			if (p_prob[x] != 0xffffffffffffffffULL) {
				int shape = PROB_SHAPE(p_prob[x]);
				CV_Assert(shape <= BRICK_IN_USE || shape == BRICK_VIA || shape == BRICK_FAKE_VIA || shape == BRICK_INVALID);
			}

		}
	}

	void release_temp_data() {
		img.release();
		ig.release();
		iig.release();
		lg.release();
		llg.release();
		prob.release();
		for (int i = 0; i < 16; i++)
			v[i].d.release();
		bm.release();
	}
};

class PipeData {
public:
	vector<PipeDataPerLayer> l;
	int x0, y0;
};

struct RuleParameter {
	int method;
	int method_opt;
	unsigned long long merge_rule;
	unsigned long long warning_rule;
	int dx, dy, dx1, dy1;
	int opt0, opt1;
	int opt_f0;
};

template<class T>
static int find_index(Mat & m, T key)
{
	switch (sizeof(T)) {
	case 1:
		CV_Assert(m.depth() == CV_8U || m.depth() == CV_8S);
		break;
	case 2:
		CV_Assert(m.depth() == CV_16U || m.depth() == CV_16S);
		break;
	case 4:
		CV_Assert(m.depth() == CV_32S || m.depth() == CV_32F);
		break;
	case 8:
		CV_Assert(m.depth() == CV_64F);
		break;
	}
	for (int i = 0; i < m.rows; i++)
	if (m.at<T>(i, 0) == key)
		return i;
	return -1;
}

static string get_time_str()
{
	QDateTime t = QDateTime::currentDateTime();
	return "./DImg/" + t.toString("hh.mm.ss.zzz").toStdString();
}

static void deldir(const string &path)
{
	if (path.empty())
		return;

	QDir dir(QString::fromStdString(path));
	if (!dir.exists())
		return;
	dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);
	QFileInfoList fileList = dir.entryInfoList(); //get all file info
	foreach(QFileInfo file, fileList) {
		if (file.isFile())
			file.dir().remove(file.fileName());
		else
			deldir(file.absoluteFilePath().toStdString());
	}
}

//Bresenham line draw
template<class T>
static void mark_line(Mat & m, Point pt1, Point pt2, T mask) {
	switch (sizeof(T)) {
	case 1:
		CV_Assert(m.depth() == CV_8U || m.depth() == CV_8S);
		break;
	case 2:
		CV_Assert(m.depth() == CV_16U || m.depth() == CV_16S);
		break;
	case 4:
		CV_Assert(m.depth() == CV_32S || m.depth() == CV_32F);
		break;
	case 8:
		CV_Assert(m.depth() == CV_64F);
		break;
	}
	CV_Assert(pt1.x >= 0 && pt1.x < m.cols && pt1.y >= 0 && pt1.y < m.rows);
	CV_Assert(pt2.x >= 0 && pt2.x < m.cols && pt2.y >= 0 && pt2.y < m.rows);
	int dx = abs(pt2.x - pt1.x);
	int dy = abs(pt2.y - pt1.y);
	bool dir = dx > dy;
	int ix = (pt2.x - pt1.x > 0) ? 1 : -1;
	int iy = (pt2.y - pt1.y > 0) ? 1 : -1;

	if (dir) {
		int dy2 = 2 * dy;
		int dy2dx2 = 2 * dy - 2 * dx;
		int d = 2 * dy - dx;
		int x = pt1.x, y = pt1.y;
		for (; x != pt2.x; x += ix) {
			m.at<T>(y, x) |= mask;
			if (d < 0)
				d += dy2;
			else {
				d += dy2dx2;
				y += iy;
			}
		}
		CV_Assert(y == pt2.y);
		m.at<T>(y, x) |= mask;
	}
	else {
		int dx2 = 2 * dx;
		int dx2dy2 = 2 * dx - 2 * dy;
		int d = 2 * dx - dy;
		int x = pt1.x, y = pt1.y;
		for (; y != pt2.y; y += iy) {
			m.at<T>(y, x) |= mask;
			if (d < 0)
				d += dx2;
			else {
				d += dx2dy2;
				x += ix;
			}
		}
		CV_Assert(x == pt2.x);
		m.at<T>(y, x) |= mask;
	}
}

template<class T>
static void mark_rect(Mat & m, Point pt1, Point pt2, T mask) {
	switch (sizeof(T)) {
	case 1:
		CV_Assert(m.depth() == CV_8U || m.depth() == CV_8S);
		break;
	case 2:
		CV_Assert(m.depth() == CV_16U || m.depth() == CV_16S);
		break;
	case 4:
		CV_Assert(m.depth() == CV_32S || m.depth() == CV_32F);
		break;
	case 8:
		CV_Assert(m.depth() == CV_64F);
		break;
	}
	pt1.x = max(pt1.x, 0);
	pt1.y = max(pt1.y, 0);
	pt2.x = min(pt2.x, m.cols - 1);
	pt2.y = min(pt2.y, m.rows - 1);
	for (int y = pt1.y; y <= pt2.y; y++) {
		T * pm = m.ptr<T>(y);
		for (int x = pt1.x; x <= pt2.x; x++)
			pm[x] |= mask;
	}
}

/*
input points
output lut, 256 * 1
spline hermite
*/
static void cvt_tbl_hermite(const Mat & points, Mat & lut, int debug_en = 0)
{
	CV_Assert(points.type() == CV_32SC1 && points.cols == 4);
	float x0 = 0, y0 = 0, x1 = 0, y1;
	float k0, k1;

	x0 = points.at<int>(0, 0);
	y0 = points.at<int>(0, 1);
	k0 = points.at<int>(0, 2) / 100.0;
	k1 = points.at<int>(0, 3) / 100.0;

	lut.create(1, 256, CV_8U);
	uchar *p_lut = lut.ptr<unsigned char>(0);
	for (int i = 1; x1 != 255; i++) {
		if (i < points.rows) {
			x1 = points.at<int>(i, 0);
			y1 = points.at<int>(i, 1);
		}
		else {
			x1 = 255;
			y1 = 255;
		}
		qInfo("cvt_tbl_hermite: x0=%f,y0=%f,k0=%f, x1=%f,y1=%f,k1=%f)", x0, y0, k0, x1, y1, k1);
		CV_Assert(x1 >= x0 && x1 < 256 && y1 < 256 && x0 >= 0 && y0 >= 0 && y1 >= 0);
		if (x0 != x1)
		for (int x = x0; x <= x1; x++) {
			float u = (x - x0) / (x1 - x0);
			float y = y0* (2 * u*u*u - 3 * u*u + 1) + y1*(-2 * u*u*u + 3 * u*u) + k0*(u*u*u - 2 * u*u + u) + k1*(u*u*u - u*u);
			p_lut[x] = (y < 0) ? 0 : ((y>255) ? 255 : y);
		}
		x0 = x1, y0 = y1;
		if (i < points.rows) {
			k0 = points.at<int>(i, 2) / 100.0;
			k1 = points.at<int>(i, 3) / 100.0;
		}
	}

	if (debug_en)
	for (int i = 0; i < 32; i++) {
		uchar * px = &p_lut[i * 8];
		qDebug("%3d:%d,%d,%d,%d,%d,%d,%d,%d", i * 8, px[0], px[1], px[2], px[3], px[4], px[5], px[6], px[7]);
	}
}

class ViaComputeScore {
public:
	static ViaComputeScore * create_via_compute_score(int _layer, ViaParameter &vp, PipeData &);

	int compute_circle_dx(int r, vector<int> & dx)
	{
		dx.resize(r + 1);
		int n = 0;
		for (int i = 0; i <= r; i++) {
			dx[i] = sqrt(r * r - i * i);
			n += (i == 0) ? dx[i] * 2 + 1 : dx[i] * 4 + 2;
		}
		return n;
	}

	virtual void prepare(int _layer, ViaParameter &vp, PipeData & _d) = 0;
	virtual unsigned compute(int x0, int y0) = 0;
	virtual ~ViaComputeScore() {
		qDebug("ViaComputeScore freed");
	}
};

/*
It require internal circle =gray
external circle = gray1
*/
class TwoCircleEECompute : public ViaComputeScore {
protected:
	int r, r1;
	int gray, gray1;
	vector<int> dx, dx1;
	float a, a1, b, b1;
	int layer;
	PipeData * d;
	Mat * lg;
	Mat *llg;

public:
	friend class TwoCirclelBSCompute;
	void prepare(int _layer, ViaParameter &vp, PipeData & _d) {
		layer = _layer;
		d = &_d;
		d->l[layer].validate_ig();
		lg = &(d->l[layer].lg);
		llg = &(d->l[layer].llg);
		r = vp.rd0;
		r1 = vp.rd1;
		gray = vp.gray0;
		gray1 = vp.gray1;
		float gamma = vp.arfactor / 100.0;
		qInfo("TwoCirclePrepare r0=%d,r1=%d,g0=%d,g1=%d, gamma=%f", r, r1, gray, gray1, gamma);
		if (r >= r1)
			qCritical("invalid r0 and r1");
		int n = compute_circle_dx(r, dx);
		int n1 = compute_circle_dx(r1, dx1);
		a = 1.0 / n;
		a1 = 1.0 / (n1 - n);
		b = (float)n / n1;
		b1 = 1 - b;
		float arf = pow(n1, 2 * gamma);
		b = b * arf;
		b1 = b1 * arf;
	}

	unsigned compute(int x0, int y0) {
		int s = 0, ss = 0, s1 = 0, ss1 = 0;
		for (int y = -r; y <= r; y++) {
			int x = dx[abs(y)];
			s += lg->at<int>(y + y0, x + x0 + 1) - lg->at<int>(y + y0, x0 - x);
			ss += llg->at<int>(y + y0, x + x0 + 1) - llg->at<int>(y + y0, x0 - x);
		}
		for (int y = -r1; y <= r1; y++) {
			int x = dx1[abs(y)];
			s1 += lg->at<int>(y + y0, x + x0 + 1) - lg->at<int>(y + y0, x0 - x);
			ss1 += llg->at<int>(y + y0, x + x0 + 1) - llg->at<int>(y + y0, x0 - x);
		}
		float f[4];
		f[0] = s;
		f[1] = ss;
		f[2] = s1 - s;
		f[3] = ss1 - ss;
		unsigned score = sqrt(((f[1] - f[0] * 2 * gray) * a + gray*gray) * b + ((f[3] - f[2] * 2 * gray1) * a1 + gray1*gray1) * b1);

		CV_Assert(score < 65536 && f[1] + 1 >= f[0] * f[0] * a && f[2] >= 0 && f[3] + 1 >= f[2] * f[2] * a1);
		return (score < MIN_SCORE) ? MIN_SCORE : score;
	}
};

/*
It require internal circle =gray
external circle <= gray1
*/
class TwoCirclelBSCompute : public ViaComputeScore {
protected:
	TwoCircleEECompute tcc;
	Mat lg, llg;

public:
	void prepare(int _layer, ViaParameter &vp, PipeData & _d) {
		tcc.layer = _layer;
		tcc.d = &_d;
		Mat img, ig, iig;
		img = _d.l[_layer].img.clone();
		int m0 = min(vp.gray0, vp.gray1);
		int m1 = max(vp.gray0, vp.gray1);
		for (int y = 0; y < img.rows; y++) {
			unsigned char * p_img = img.ptr<unsigned char>(y);
			for (int x = 0; x < img.cols; x++)
				p_img[x] = (p_img[x] < m0) ? m0 : ((p_img[x] > m1) ? m1 : p_img[x]);
		}

		integral_square(img, ig, iig, lg, llg, true);
		tcc.lg = &lg;
		tcc.llg = &llg;
		tcc.r = vp.rd0;
		tcc.r1 = vp.rd1;
		tcc.gray = vp.gray0;
		tcc.gray1 = vp.gray1;
		float gamma = vp.arfactor / 100.0;
		qInfo("TwoCircleXPrepare r0=%d,r1=%d,g0=%d,g1=%d, gamma=%f", tcc.r, tcc.r1, vp.gray0, vp.gray1, gamma);
		if (tcc.r >= tcc.r1)
			qCritical("invalid r0 and r1");
		if (vp.gray1 >= vp.gray0)
			qCritical("invalid gray1 >= gray0");
		int n = compute_circle_dx(tcc.r, tcc.dx);
		int n1 = compute_circle_dx(tcc.r1, tcc.dx1);
		tcc.a = 1.0 / n;
		tcc.a1 = 1.0 / (n1 - n);
		tcc.b = (float)n / n1;
		tcc.b1 = 1 - tcc.b;
		float arf = pow(n1, 2 * gamma);
		tcc.b = tcc.b * arf;
		tcc.b1 = tcc.b1 * arf;
	}

	unsigned compute(int x0, int y0) {
		return tcc.compute(x0, y0);
	}
};

/*
It require internal circle =gray
external circle = gray1
most external circle = gray2
*/
class ThreeCircleEEECompute : public ViaComputeScore {
protected:
	int r, r1, r2;
	int gray, gray1, gray2;
	vector<int> dx, dx1, dx2;
	float a, a1, a2, b, b1, b2;
	int layer;
	PipeData * d;
	Mat * lg;
	Mat * llg;

public:
	friend class ThreeCircleBSSCompute;
	void prepare(int _layer, ViaParameter &vp, PipeData & _d) {
		layer = _layer;
		d = &_d;
		d->l[layer].validate_ig();
		lg = &(d->l[layer].lg);
		llg = &(d->l[layer].llg);
		r = vp.rd0;
		r1 = vp.rd1;
		r2 = vp.rd2;
		gray = vp.gray0;
		gray1 = vp.gray1;
		gray2 = vp.gray2;
		float gamma = vp.arfactor / 100.0;
		qInfo("ThreeCirclePrepare r0=%d,r1=%d,r2=%d,g0=%d,g1=%d,g2=%d, gamma=%f", r, r1, r2, gray, gray1, gray2, gamma);
		if (r >= r1)
			qCritical("invalid r0 and r1");
		if (r1 >= r2)
			qCritical("invalid r1 and r2");
		int n = compute_circle_dx(r, dx);
		int n1 = compute_circle_dx(r1, dx1);
		int n2 = compute_circle_dx(r2, dx2);
		a = 1.0 / n;
		a1 = 1.0 / (n1 - n);
		a2 = 1.0 / (n2 - n1);
		b = (float)n / n2;
		b1 = (float)(n1 - n) / n2;
		b2 = (float)(n2 - n1) / n2;
		float arf = pow(n2, 2 * gamma);
		b = b * arf;
		b1 = b1 * arf;
		b2 = b2 * arf;
	}

	unsigned compute(int x0, int y0) {
		int s = 0, ss = 0, s1 = 0, ss1 = 0, s2 = 0, ss2 = 0;
		for (int y = -r; y <= r; y++) {
			int x = dx[abs(y)];
			s += lg->at<int>(y + y0, x + x0 + 1) - lg->at<int>(y + y0, x0 - x);
			ss += llg->at<int>(y + y0, x + x0 + 1) - llg->at<int>(y + y0, x0 - x);
		}
		for (int y = -r1; y <= r1; y++) {
			int x = dx1[abs(y)];
			s1 += lg->at<int>(y + y0, x + x0 + 1) - lg->at<int>(y + y0, x0 - x);
			ss1 += llg->at<int>(y + y0, x + x0 + 1) - llg->at<int>(y + y0, x0 - x);
		}
		for (int y = -r2; y <= r2; y++) {
			int x = dx2[abs(y)];
			s2 += lg->at<int>(y + y0, x + x0 + 1) - lg->at<int>(y + y0, x0 - x);
			ss2 += llg->at<int>(y + y0, x + x0 + 1) - llg->at<int>(y + y0, x0 - x);
		}
		float f[6];
		f[0] = s;
		f[1] = ss;
		f[2] = s1 - s;
		f[3] = ss1 - ss;
		f[4] = s2 - s1;
		f[5] = ss2 - ss1;
		CV_Assert(f[2] >= 0 && f[4] >= 0);
		CV_Assert(f[1] + 1 >= f[0] * f[0] * a && f[3] + 1 >= f[2] * f[2] * a1 && f[5] + 1 >= f[4] * f[4] * a2);
		unsigned score = sqrt(((f[1] - f[0] * 2 * gray) * a + gray*gray) * b +
			((f[3] - f[2] * 2 * gray1) * a1 + gray1*gray1) * b1 +
			((f[5] - f[4] * 2 * gray2) * a2 + gray2*gray2) * b2);
		CV_Assert(score < 65536);
		return (score < MIN_SCORE) ? MIN_SCORE : score;
	}
};

/*
It require internal circle <=gray
external circle = gray1
most external circle <= gray
*/
class ThreeCircleBSSCompute : public ViaComputeScore {
protected:
	ThreeCircleEEECompute tcc;
	Mat lg, llg;

public:
	void prepare(int _layer, ViaParameter &vp, PipeData & _d) {
		tcc.layer = _layer;
		tcc.d = &_d;
		Mat img, ig, iig;
		img = _d.l[_layer].img.clone();
		int m0 = min(min(vp.gray0, vp.gray1), vp.gray2), m1;
		if (m0 == vp.gray0)
			m1 = min(vp.gray1, vp.gray2);
		if (m0 == vp.gray1)
			m1 = min(vp.gray0, vp.gray2);
		if (m0 == vp.gray2)
			m1 = min(vp.gray0, vp.gray1);
		int m2 = max(max(vp.gray0, vp.gray1), vp.gray2);
		for (int y = 0; y < img.rows; y++) {
			unsigned char * p_img = img.ptr<unsigned char>(y);
			for (int x = 0; x < img.cols; x++)
				p_img[x] = (p_img[x] < m0) ? m0 : ((p_img[x] < m1) ? m1 : ((p_img[x] > m2) ? m2 : p_img[x]));
		}
		integral_square(img, ig, iig, lg, llg, true);
		tcc.lg = &lg;
		tcc.llg = &llg;
		tcc.r = vp.rd0;
		tcc.r1 = vp.rd1;
		tcc.r2 = vp.rd2;
		tcc.gray = vp.gray0;
		tcc.gray1 = vp.gray1;
		tcc.gray2 = vp.gray2;
		float gamma = vp.arfactor / 100.0;
		qInfo("ThreeCirclePrepare r0=%d,r1=%d,r2=%d,g0=%d,g1=%d,g2=%d, gamma=%f", tcc.r, tcc.r1, tcc.r2, vp.gray0, vp.gray1, vp.gray2, gamma);
		if (tcc.r >= tcc.r1)
			qCritical("invalid r0 and r1");
		if (tcc.r1 >= tcc.r2)
			qCritical("invalid r1 and r2");
		int n = compute_circle_dx(tcc.r, tcc.dx);
		int n1 = compute_circle_dx(tcc.r1, tcc.dx1);
		int n2 = compute_circle_dx(tcc.r2, tcc.dx2);
		tcc.a = 1.0 / n;
		tcc.a1 = 1.0 / (n1 - n);
		tcc.a2 = 1.0 / (n2 - n1);
		tcc.b = (float)n / n2;
		tcc.b1 = (float)(n1 - n) / n2;
		tcc.b2 = (float)(n2 - n1) / n2;
		float arf = pow(n2, 2 * gamma);
		tcc.b = tcc.b * arf;
		tcc.b1 = tcc.b1 * arf;
		tcc.b2 = tcc.b2 * arf;
	}

	unsigned compute(int x0, int y0) {
		return tcc.compute(x0, y0);
	}
};

class FourCircleEEEECompute : public ViaComputeScore {
protected:
	int r, r1, r2, r3;
	int gray, gray1, gray2, gray3;
	vector<int> dx, dx1, dx2, dx3;
	float a, a1, a2, a3, b, b1, b2, b3;
	int layer;
	PipeData * d;

public:
	void prepare(int _layer, ViaParameter &vp, PipeData & _d) {
		layer = _layer;
		d = &_d;
		d->l[layer].validate_ig();
		r = vp.rd0;
		r1 = vp.rd1;
		r2 = vp.rd2;
		r3 = vp.rd3;
		gray = vp.gray0;
		gray1 = vp.gray1;
		gray2 = vp.gray2;
		gray3 = vp.gray3;
		float gamma = vp.arfactor / 100.0;
		qInfo("FourCirclePrepare r0=%d,r1=%d,r2=%d,r3=%d,g0=%d,g1=%d,g2=%d,g3=%d, gamma=%f", r, r1, r2, r3, gray, gray1, gray2, gray3, gamma);
		int n = compute_circle_dx(r, dx);
		int n1 = compute_circle_dx(r1, dx1);
		int n2 = compute_circle_dx(r2, dx2);
		int n3 = compute_circle_dx(r3, dx3);
		if (r >= r1)
			qCritical("invalid r0 and r1");
		if (r1 >= r2)
			qCritical("invalid r1 and r2");
		if (r2 >= r3)
			qCritical("invalid r2 and r3");
		a = 1.0 / n;
		a1 = 1.0 / (n1 - n);
		a2 = 1.0 / (n2 - n1);
		a3 = 1.0 / (n3 - n2);
		b = (float)n / n3;
		b1 = (float)(n1 - n) / n3;
		b2 = (float)(n2 - n1) / n3;
		b3 = (float)(n3 - n2) / n3;
		float arf = pow(n3, 2 * gamma);
		b = b * arf;
		b1 = b1 * arf;
		b2 = b2 * arf;
		b3 = b3 * arf;
	}

	unsigned compute(int x0, int y0) {
		const Mat & lg = d->l[layer].lg;
		const Mat & llg = d->l[layer].llg;
		int s = 0, ss = 0, s1 = 0, ss1 = 0;
		int s2 = 0, ss2 = 0, s3 = 0, ss3 = 0;
		for (int y = -r; y <= r; y++) {
			int x = dx[abs(y)];
			s += lg.at<int>(y + y0, x + x0 + 1) - lg.at<int>(y + y0, x0 - x);
			ss += llg.at<int>(y + y0, x + x0 + 1) - llg.at<int>(y + y0, x0 - x);
		}
		for (int y = -r1; y <= r1; y++) {
			int x = dx1[abs(y)];
			s1 += lg.at<int>(y + y0, x + x0 + 1) - lg.at<int>(y + y0, x0 - x);
			ss1 += llg.at<int>(y + y0, x + x0 + 1) - llg.at<int>(y + y0, x0 - x);
		}
		for (int y = -r2; y <= r2; y++) {
			int x = dx2[abs(y)];
			s2 += lg.at<int>(y + y0, x + x0 + 1) - lg.at<int>(y + y0, x0 - x);
			ss2 += llg.at<int>(y + y0, x + x0 + 1) - llg.at<int>(y + y0, x0 - x);
		}
		for (int y = -r3; y <= r3; y++) {
			int x = dx3[abs(y)];
			s3 += lg.at<int>(y + y0, x + x0 + 1) - lg.at<int>(y + y0, x0 - x);
			ss3 += llg.at<int>(y + y0, x + x0 + 1) - llg.at<int>(y + y0, x0 - x);
		}
		float f[8];
		f[0] = s;
		f[1] = ss;
		f[2] = s1 - s;
		f[3] = ss1 - ss;
		f[4] = s2 - s1;
		f[5] = ss2 - ss1;
		f[6] = s3 - s2;
		f[7] = ss3 - ss2;
		CV_Assert(f[0] >= 0 && f[2] >= 0 && f[4] >= 0);
		CV_Assert(f[1] + 1 >= f[0] * f[0] * a && f[3] + 1 >= f[2] * f[2] * a1 && f[5] + 1 >= f[4] * f[4] * a2 && f[7] + 1 >= f[6] * f[6] * a3);
		unsigned score = sqrt(((f[1] - f[0] * 2 * gray) * a + gray*gray) * b +
			((f[3] - f[2] * 2 * gray1) * a1 + gray1*gray1) * b1 +
			((f[5] - f[4] * 2 * gray2) * a2 + gray2*gray2) * b2 +
			((f[7] - f[6] * 2 * gray3) * a3 + gray3*gray3) * b3);
		CV_Assert(score < 65536);
		return (score < MIN_SCORE) ? MIN_SCORE : score;
	}
};

ViaComputeScore * ViaComputeScore::create_via_compute_score(int _layer, ViaParameter &vp, PipeData & d)
{
	ViaComputeScore * vc = NULL;
	switch (vp.subtype) {
	case VIA_SUBTYPE_2CIRCLE:
		vc = new TwoCircleEECompute();
		vc->prepare(_layer, vp, d);
		break;
	case VIA_SUBTYPE_3CIRCLE:
		vc = new ThreeCircleEEECompute();
		vc->prepare(_layer, vp, d);
		break;
	case VIA_SUBTYPE_4CIRCLE:
		vc = new FourCircleEEEECompute();
		vc->prepare(_layer, vp, d);
		break;
	case VIA_SUBTYPE_2CIRCLEX:
		vc = new TwoCirclelBSCompute();
		vc->prepare(_layer, vp, d);
		break;
	case VIA_SUBTYPE_3CIRCLEX:
		vc = new ThreeCircleBSSCompute();
		vc->prepare(_layer, vp, d);
		break;
	default:
		qCritical("ViaComputeScore create failed, subtype=%d", vp.subtype);
		break;
	}
	return vc;
}

class ViaRemove {
public:
	static ViaRemove * create_via_remove(ViaParameter &vp, PipeDataPerLayer & d, int remove_opt, int papa);

	static void compute_circle_dd(int r, vector<int> & dd)
	{
		dd.resize(r + 1);
		for (int i = 0; i <= r; i++)
			dd[i] = sqrt(r * r - i * i);
	}

	virtual void prepare(ViaParameter &vp, PipeDataPerLayer & d, int remove_opt, int para) = 0;
	//dir==0 means remove up-down style, else remove left-right style
	virtual void remove(Mat & img, int x0, int y0, int dir) = 0;
	virtual void remove_mask(Mat & mask, int x0, int y0) = 0;
	virtual void finish(Mat & img) = 0;
	virtual ~ViaRemove() {
		qDebug("ViaRemove freed");
	}
};

class ViaCircleRemove : public ViaRemove {
protected:
	vector<int> d2;
	int gd, connect_rd, remove_rd, connect_d;
	Mat ig;	
	vector< vector<int> > offset[4];
	vector<int> avg[4];
	Point corner[4][2];
	int opt, erode_len;
public:
	void prepare(ViaParameter &vp, PipeDataPerLayer & d, int remove_opt, int para0) {
		opt = remove_opt;
		erode_len = para0;
		gd = vp.guard;
		compute_circle_dd(gd, d2);
		connect_rd = vp.connect_rd;
		remove_rd = vp.remove_rd;
		connect_d = vp.connect_d;
		qInfo("ViaCircleRemove, connect_rd=%d, remove_rd=%d, connect_d=%d", connect_rd, remove_rd, connect_d);
		if (connect_rd > remove_rd)
			qCritical("ViaCircleRemove connect_rd > remove_rd");
		corner[0][0] = Point(-connect_d / 2, -remove_rd);
		corner[0][1] = Point(connect_d / 2, -remove_rd);
		corner[1][0] = Point(remove_rd, -connect_d / 2);
		corner[1][1] = Point(remove_rd, connect_d / 2);
		corner[2][0] = Point(-connect_d / 2, remove_rd);
		corner[2][1] = Point(connect_d / 2, remove_rd);
		corner[3][0] = Point(-remove_rd, -connect_d / 2);
		corner[3][1] = Point(-remove_rd, connect_d / 2);
		for (int i = 0; i < 4; i++) {	
			offset[i].clear();
			Point oxy, dxy;
			for (int l = connect_rd; l <= remove_rd; l++) {
				switch (i) {
				case DIR_UP:
					oxy = Point(-connect_d / 2, -l);
					dxy = Point(1, 0);
					break;
				case DIR_RIGHT:
					oxy = Point(l, -connect_d / 2);
					dxy = Point(0, 1);
					break;
				case DIR_DOWN:
					oxy = Point(-connect_d / 2, l);
					dxy = Point(1, 0);
					break;
				case DIR_LEFT:
					oxy = Point(-l, -connect_d / 2);
					dxy = Point(0, 1);
					break;
				}
				vector<int> line_offset;
				for (int j = 0; j < connect_d; j++) {
					line_offset.push_back((oxy.y * (int)d.img.step.p[0] + oxy.x * (int)d.img.step.p[1]) / sizeof(char));
					oxy += dxy;
				}
				offset[i].push_back(line_offset);
			}
			avg[i].resize(offset[i].size());
		}
	}

	void remove(Mat & img, int x0, int y0, int dir) {
		Point center(x0, y0);
		unsigned char *p_img = img.ptr<unsigned char>(y0, x0);
		for (int i = 0; i < 4; i++) {
			Point corner0 = center + corner[i][0];
			Point corner1 = center + corner[i][1];
			if (corner0.x >= 0 && corner0.y >= 0 && corner0.x < img.cols && corner0.y < img.rows &&
				corner1.x >= 0 && corner1.y >= 0 && corner1.x < img.cols && corner1.y < img.rows) {
				for (int l = 0; l < (int) offset[i].size(); l++) {		
					if (erode_len >= 1 && (dir & 1 << i))
					for (int j = 0; j < (int)offset[i][l].size(); j++) {
						unsigned char min_img = p_img[offset[i][l][j]];
						for (int m = 1; m <= erode_len && l + m < offset[i].size() ; m++)
							min_img = min(min_img, p_img[offset[i][l + m][j]]);
						p_img[offset[i][l][j]] = min_img;
					}
					avg[i][l] = 0;
					for (int j = 0; j < (int)offset[i][l].size(); j++)
						avg[i][l] += p_img[offset[i][l][j]];
					avg[i][l] = avg[i][l] / (int)offset[i][l].size();
				}
			}
		}
		for (int i = 0; i < 4; i++) {
			Point corner0 = center + corner[i][0];
			Point corner1 = center + corner[i][1];
			if (corner0.x >= 0 && corner0.y >= 0 && corner0.x < img.cols && corner0.y < img.rows &&
				corner1.x >= 0 && corner1.y >= 0 && corner1.x < img.cols && corner1.y < img.rows) {
				for (int l = 0; l < (int)offset[i].size(); l++)
				for (int j = 0; j < (int)offset[i][l].size(); j++)
					p_img[offset[i][l][j]] = avg[i][l];
			}
		}
	}

	void remove_mask(Mat & mask, int x0, int y0) {
		CV_Assert(x0 >= gd && y0 >= gd && x0 + gd < mask.cols && y0 + gd < mask.rows);
		for (int y = y0 - gd; y <= y0 + gd; y++) {
			unsigned char * p_mask = mask.ptr<unsigned char>(y);
			int x1 = x0 - d2[abs(y - y0)];
			int x2 = x0 + d2[abs(y - y0)];
			for (int x = x1; x <= x2; x++)
				p_mask[x] |= 1;
		}
	}

	void finish(Mat &) {
	}
};

ViaRemove * ViaRemove::create_via_remove(ViaParameter &vp, PipeDataPerLayer & d, int remove_opt, int para)
{
	ViaRemove * vr = NULL;
	switch (vp.subtype) {
	case VIA_SUBTYPE_2CIRCLE:
	case VIA_SUBTYPE_3CIRCLE:
	case VIA_SUBTYPE_4CIRCLE:
	case VIA_SUBTYPE_2CIRCLEX:
	case VIA_SUBTYPE_3CIRCLEX:
		vr = new ViaCircleRemove();
		vr->prepare(vp, d, remove_opt, para);
		break;
	default:
		qCritical("ViaRemove create failed, subtype=%d", vp.subtype);
		break;
	}
	return vr;
}

class WireComputeScore {
public:
	static WireComputeScore * create_wire_compute_score(WireParameter &wp, PipeData & d, int layer);
	void push_min(PAIR_ULL & bs, unsigned long long s) {
		if (bs.first > s) {
			bs.second = bs.first;
			bs.first = s;
		}
		else
		if (bs.second > s)
			bs.second = s;
	}
	virtual int prepare(WireParameter & _wp, PipeData & d, int layer) = 0;
	virtual PAIR_ULL compute(int x0, int y0, const Mat & img, const Mat & ig, const Mat & iig) = 0;
	virtual int check_mark(int x0, int y0, const Mat & prob, Mat & mask, int subtype, const Mat & already_mask) = 0;
	virtual void check_via_wire_connect(Mat & remove_via_mask, Mat & prob, Mat & img, Mat & ig) = 0;
	virtual ~WireComputeScore() {
		qDebug("WireComputeScore freed");
	};
};

static struct Wire25ShapeConst {
	int group_id;
	double ratio[25];
	int shape;
} wire_25_shape[] = {
	//       0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, 00, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_NO_WIRE },
	{ 0, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, -1, 00, .8, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_i_0 },
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, .8, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_i_90 },
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, .8, 00, -1, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_i_180 },
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, .8, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_i_270 },
	{ 0, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_I_0 },
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, 01, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_I_90 },
	{ 0, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, -1, 00, 01, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_L_0 },
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, 01, 01, 01, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_L_90 },
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, 01, 00, -1, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_L_180 },
	{ 0, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, 01, 01, 01, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_L_270 },
	{ 0, { -1, -1, 01, -1, -1, -1, 01, 01, 00, -1, 00, 01, 01, 00, -1, -1, 01, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_II_0 },
	{ 0, { -1, -1, 00, -1, -1, -1, 01, 01, 01, -1, 01, 01, 01, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_II_90 },
	{ 0, { -1, -1, 01, -1, -1, -1, 00, 01, 01, -1, -1, 00, 01, 01, 00, -1, 00, 01, 01, -1, -1, -1, 01, -1, -1 }, BRICK_II_180 },
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, 01, 01, 01, -1, 01, 01, 01, -1, -1, -1, 00, -1, -1 }, BRICK_II_270 },
	{ 0, { -1, -1, -1, -1, -1, -1, 01, 01, 01, -1, -1, 01, 01, 01, -1, -1, 01, 01, 01, -1, -1, -1, -1, -1, -1 }, BRICK_FAKE_VIA },
	{ 0, { -1, -1, -1, -1, -1, -1, -1, 01, -1, -1, -1, .3, .3, .3, -1, -1, -1, 01, -1, -1, -1, -1, -1, -1, -1 }, BRICK_HOLLOW },
	{ 0, { -1, -1, -1, -1, -1, -1, -1, .3, -1, -1, -1, 01, .3, 01, -1, -1, -1, .3, -1, -1, -1, -1, -1, -1, -1 }, BRICK_HOLLOW },
	{ 0, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, 01, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_ONE_POINT },
	{ 1, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, -1, 00, 01, 01, 01, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_T_0 },
	{ 1, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, 01, 01, 01, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_T_90 },
	{ 1, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, 01, 01, 01, 00, -1, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_T_180 },
	{ 1, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, 01, 01, 01, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_T_270 },
	{ 2, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, 01, 01, 01, 01, 01, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_X_0 },
	//       0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
	{ 3, { -1, 00, 00, 00, -1, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00, 00, 00, -1 }, BRICK_NO_WIRE },
	{ 3, { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, .8, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_i_0 },
	{ 3, { -1, -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 00, .8, 01, 01, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_i_90 },
	{ 3, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, .8, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_i_180 },
	{ 3, { -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 01, 01, .8, 00, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_i_270 },
	{ 3, { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_I_0 },
	{ 3, { -1, -1, -1, -1, -1, 00, 00, 00, 00, 00, 01, 01, 01, 01, 01, 00, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_I_90 },
	{ 3, { -1, 00, 01, 00, -1, -1, 00, 01, 00, 00, -1, 00, 01, 01, 01, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_L_0 },
	{ 3, { -1, -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 00, 01, 01, 01, -1, 00, 01, 00, 00, -1, 00, 01, 00, -1 }, BRICK_L_90 },
	{ 3, { -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 01, 01, 01, 00, -1, 00, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_L_180 },
	{ 3, { -1, 00, 01, 00, -1, 00, 00, 01, 00, -1, 01, 01, 01, 00, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_L_270 },
	{ 3, { -1, 01, 01, 00, -1, -1, 01, 01, 00, -1, 00, 01, 01, 00, -1, -1, 01, 01, 00, -1, -1, 01, 01, 00, -1 }, BRICK_II_0 },
	{ 3, { -1, -1, 00, -1, -1, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 00, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_II_90 },
	{ 3, { -1, 00, 01, 01, -1, -1, 00, 01, 01, -1, -1, 00, 01, 01, 00, -1, 00, 01, 01, -1, -1, 00, 01, 01, -1 }, BRICK_II_180 },
	{ 3, { -1, -1, -1, -1, -1, 00, 00, 00, 00, 00, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, -1, -1, 00, -1, -1 }, BRICK_II_270 },
	{ 3, { -1, -1, -1, -1, -1, -1, 01, 01, 01, -1, -1, 01, 01, 01, -1, -1, 01, 01, 01, -1, -1, -1, -1, -1, -1 }, BRICK_FAKE_VIA },
	{ 3, { -1, -1, -1, -1, -1, -1, -1, 01, -1, -1, -1, .3, .3, .3, -1, -1, -1, 01, -1, -1, -1, -1, -1, -1, -1 }, BRICK_HOLLOW },
	{ 3, { -1, -1, -1, -1, -1, -1, -1, .3, -1, -1, -1, 01, .3, 01, -1, -1, -1, .3, -1, -1, -1, -1, -1, -1, -1 }, BRICK_HOLLOW },
	{ 3, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, 01, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_ONE_POINT },
	{ 4, { -1, 00, 01, 00, -1, -1, 00, 01, 00, 00, -1, 00, 01, 01, 01, -1, 00, 01, 00, 00, -1, 00, 01, 00, -1 }, BRICK_T_0 },
	{ 4, { -1, -1, -1, -1, -1, 00, 00, 00, 00, 00, 01, 01, 01, 01, 01, 00, 00, 01, 00, 00, -1, 00, 01, 00, -1 }, BRICK_T_90 },
	{ 4, { -1, 00, 01, 00, -1, 00, 00, 01, 00, -1, 01, 01, 01, 00, -1, 00, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_T_180 },
	{ 4, { -1, 00, 01, 00, -1, 00, 00, 01, 00, 00, 01, 01, 01, 01, 01, 00, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_T_270 },
	{ 5, { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, 01, 01, 01, 01, 01, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_X_0 },
	//       0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
	{ 6, { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, .1, .5, .9, 00, -1, 01, 01, .5, 00, -1, 01, 01, .1, -1, -1 }, BRICK_J_0 },
	{ 6, { 01, 01, .1, -1, -1, 01, 01, .5, 00, 00, .1, .5, .9, 01, 01, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_J_90 },
	{ 6, { -1, -1, .1, 01, 01, -1, 00, .5, 01, 01, -1, 00, .9, .5, .1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_J_180 },
	{ 6, { -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 01, 01, .9, .5, .1, 00, 00, .5, 01, 01, -1, -1, .1, 01, 01 }, BRICK_J_270 },
	{ 6, { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, .9, .5, .1, -1, 00, .5, 01, 01, -1, -1, .1, 01, 01 }, BRICK_l_0 },
	{ 6, { -1, -1, -1, -1, -1, -1, 00, 00, 00, 00, .1, .5, .9, 01, 01, 01, 01, .5, 00, 00, 01, 01, .1, -1, -1 }, BRICK_l_90 },
	{ 6, { 01, 01, .1, -1, -1, 01, 01, .5, 00, -1, .1, .5, .9, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_l_180 },
	{ 6, { -1, -1, .1, 01, 01, 00, 00, .5, 01, 01, 01, 01, .9, .5, .1, 00, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_l_270 },
	{ 7, { -1, -1, .1, 01, 01, -1, 00, .5, 01, 01, .1, .5, .9, .5, .1, 01, 01, .5, 00, -1, 01, 01, .1, -1, -1 }, BRICK_Z_0 },
	{ 7, { 01, 01, .1, -1, -1, 01, 01, .5, 00, -1, .1, .5, .9, .5, .1, -1, 00, .5, 01, 01, -1, -1, .1, 01, 01 }, BRICK_Z_90 },
	{ 8, { -1, -1, .1, 01, 01, -1, 00, .5, 01, 01, -1, 00, .9, .5, .1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_P_0 },
	{ 8, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, .9, .5, .1, -1, 00, .5, 01, 01, -1, -1, .1, 01, 01 }, BRICK_P_90 },
	{ 8, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, .1, .5, .9, 00, -1, 01, 01, .5, 00, -1, 01, 01, .1, -1, -1 }, BRICK_P_180 },
	{ 8, { 01, 01, .1, -1, -1, 01, 01, .5, 00, -1, .1, .5, .9, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_P_270 },
	{ 9, { -1, 00, 01, 00, -1, -1, 00, 01, 00, 00, -1, .5, 01, 01, 01, -1, 01, .5, 00, 00, 01, -1, -1, -1, -1 }, BRICK_Y_45 },
	{ 9, { 01, -1, -1, -1, -1, -1, 01, .5, 00, 00, -1, .5, 01, 01, 01, -1, 00, 01, 00, 00, -1, 00, 01, 00, -1 }, BRICK_Y_135 },
	{ 9, { -1, -1, -1, -1, 01, 00, 00, .5, 01, -1, 01, 01, 01, .5, -1, 00, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_Y_225 },
	{ 9, { -1, 00, 01, 00, -1, 00, 00, 01, 00, -1, 01, 01, 01, .5, -1, 00, 00, .5, 01, -1, -1, -1, -1, -1, 01 }, BRICK_Y_315 },
	//       0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
	{ 20, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, -1, 00, .9, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_L_0 },
	{ 20, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, .9, 01, 01, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_L_90 },
	{ 20, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, .9, 00, -1, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_L_180 },
	{ 20, { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, 01, 01, .9, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_L_270 },
	{ 21, { -1, -1, 01, -1, -1, -1, 00, 01, .2, -1, -1, 00, 01, 01, 01, -1, 00, 01, .2, -1, -1, -1, 01, -1, -1 }, BRICK_T_0 },
	{ 21, { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, 01, 01, 01, -1, .2, 01, .2, -1, -1, -1, 01, -1, -1 }, BRICK_T_90 },
	{ 21, { -1, -1, 01, -1, -1, -1, .2, 01, 00, -1, 01, 01, 01, 00, -1, -1, .2, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_T_180 },
	{ 21, { -1, -1, 01, -1, -1, -1, .2, 01, .2, -1, 01, 01, 01, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_T_270 },
	{ 22, { -1, -1, 01, -1, -1, -1, .2, 01, .2, -1, -1, .2, 01, .2, -1, -1, .2, 01, .2, -1, -1, -1, 01, -1, -1 }, BRICK_I_0 },
	{ 23, { -1, -1, -1, -1, -1, -1, .2, .2, .2, -1, 01, 01, 01, 01, 01, -1, .2, .2, .2, -1, -1, -1, -1, -1, -1 }, BRICK_I_90 },
	{ -1, { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, BRICK_INVALID }
	//       0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
};

struct WirePartStat { //each shape's part expect stat value
	int part, sum; //These para are compute during prepare, and after prepare they are fixed
	int err; //this is update by compute, it means this part error
	bool operator == (const WirePartStat & ano) {
		return (part == ano.part && sum == ano.sum);
	}
};

struct ShapeDeviation {
	vector<int> si; //WirePartStat index, each ShapeDeviation have 13 or 17 or 25 part(si)
	int w25_shape_idx;
};

class Wire_25RectCompute : public WireComputeScore {
protected:
	int offset[36], dx[36], dy[36]; //offset is compute based on dx and dy
	WireParameter wp;
	vector<WirePartStat> stats; //each part expected value
	vector<ShapeDeviation> sds; //each shape may have one or multipul ShapeDeviation
	int mode, gs;
	static map<int, int> wire_25_shape_map;
	double arf;

public:
	int prepare(WireParameter & _wp, PipeData & d, int layer) {
		if (wire_25_shape_map.empty()) {
			int last_id = -100;
			for (int i = 0; i < sizeof(wire_25_shape) / sizeof(wire_25_shape[0]); i++) {
				if (wire_25_shape[i].group_id != last_id) {
					wire_25_shape_map[wire_25_shape[i].group_id] = i;
					last_id = wire_25_shape[i].group_id;
				}
				if (wire_25_shape[i].shape == BRICK_II_0 || wire_25_shape[i].shape == BRICK_II_90 ||
					wire_25_shape[i].shape == BRICK_II_180 || wire_25_shape[i].shape == BRICK_II_270)
					wire_25_shape[i].shape = BRICK_FAKE_VIA;
			}
		}
		gs = d.l[layer].gs;
		d.l[layer].validate_ig();
		wp = _wp;
		float gamma = wp.arfactor / 100.0;
		qInfo("Wire_25RectCompute Prepare w=%d,h=%d,w1=%d,h1=%d,i_w=%d,i_h=%d gamma=%f", wp.w_wide, wp.w_high,
			wp.w_wide1, wp.w_high1, wp.i_wide, wp.i_high, gamma);
		if (gamma >= 1) {
			qWarning("gamma(%f) >=1, force it to 1", gamma);
			gamma = 1;
		}
		//following compute offset
		dx[0] = -wp.w_wide1 - wp.i_wide - wp.w_wide / 2;
		dx[1] = -wp.i_wide - wp.w_wide / 2;
		dx[2] = -wp.w_wide / 2;
		dx[3] = wp.w_wide - wp.w_wide / 2;
		dx[4] = wp.w_wide - wp.w_wide / 2 + wp.i_wide;
		dx[5] = wp.w_wide - wp.w_wide / 2 + wp.i_wide + wp.w_wide1;
		if (abs(dx[0]) >= d.l[layer].compute_border || abs(dx[5]) >= d.l[layer].compute_border) {
			qCritical("dx (%d, %d) exceed compute_border(%d)", dx[0], dx[5], d.l[layer].compute_border);
			return -1;
		}
		for (int i = 6; i < 36; i++)
			dx[i] = dx[i - 6];
		dy[0] = -wp.w_high1 - wp.i_high - wp.w_high / 2;
		dy[6] = -wp.i_high - wp.w_high / 2;
		dy[12] = -wp.w_high / 2;
		dy[18] = wp.w_high - wp.w_high / 2;
		dy[24] = wp.w_high - wp.w_high / 2 + wp.i_high;
		dy[30] = wp.w_high - wp.w_high / 2 + wp.i_high + wp.w_high1;
		if (abs(dy[0]) >= d.l[layer].compute_border || abs(dy[30]) >= d.l[layer].compute_border) {
			qCritical("dx (%d, %d) exceed compute_border(%d)", dy[0], dy[30], d.l[layer].compute_border);
			return -1;
		}
		for (int i = 0; i < 36; i++)
			dy[i] = dy[i / 6 * 6];
		int area[25];
		for (int i = 0, j = 0; i < 25; i++, j++) {
			if (dx[j + 1] - dx[j] < 0)
				j++;
			area[i] = (dx[j + 1] - dx[j]) * (dy[j + 6] - dy[j]);
		}
		for (int i = 0; i < 36; i++)
			offset[i] = (dy[i] * (int)d.l[layer].ig.step.p[0] + dx[i] * (int)d.l[layer].ig.step.p[1]) / sizeof(unsigned);
		//following compute expected stats and init each shape part
		stats.clear();
		sds.clear();
		mode = 0;
		//stats 0..24 means each part minimum error
		for (int i = 0; i < 25; i++) {
			WirePartStat stat;
			stat.part = i;
			stat.sum = -1;
			stats.push_back(stat);
		}
		for (int i = 0; i <= sizeof(wp.wgid) / sizeof(wp.wgid[0]); i++) {
			int w = (i == sizeof(wp.wgid) / sizeof(wp.wgid[0])) ? wp.uni_id : wp.wgid[i];
			if (i != 0 && w == 0)
				continue;
			map<int, int>::iterator idx = wire_25_shape_map.find(w);
			if (idx == wire_25_shape_map.end()) {
				qCritical("Wire_25Rect gid=%d not found", w);
				continue;
			}
			for (int j = idx->second; wire_25_shape[j].group_id == w; j++) {
				ShapeDeviation sd;
				sd.w25_shape_idx = j;
				for (int k = 0; k < 25; k++) { //following add wire_25_shape[j] part stat to shape sd
					if (wire_25_shape[j].ratio[k] < 0) { //<0 means don't care, choose minimum error to balance
						sd.si.push_back(k);
						continue;
					}
					//compute stat and check if it already exist, save computing
					WirePartStat stat;
					stat.part = k;
					stat.sum = (wp.gray_i * (1 - wire_25_shape[j].ratio[k]) + wp.gray_w * wire_25_shape[j].ratio[k]) * area[k];
					vector<WirePartStat>::iterator it = find(stats.begin(), stats.end(), stat); //check if stat already exit
					if (it == stats.end()) {
						sd.si.push_back((int)stats.size());
						stats.push_back(stat);
						if ((k == 0 || k == 4 || k == 20 || k == 24) && mode != 2)
							mode = 1;
						if (k == 1 || k == 3 || k == 5 || k == 9 || k == 15 || k == 19 || k == 21 || k == 23)
							mode = 2;
					}
					else
						sd.si.push_back(it - stats.begin());
					CV_Assert(stats[sd.si.back()] == stat);
				}
				sds.push_back(sd);
			}
		}
		//for mode2, stats[sds[i].si[0]].part=0,stats[sds[i].si[1]].part=1,stats[sds[i].si[2]].part=2, 
		if (mode == 1) { //for mode1, stats[sds[i].si[0]].part=0,stats[sds[i].si[1]].part=2,stats[sds[i].si[2]].part=4
			for (int i = 0; i<(int)sds.size(); i++) {
				sds[i].si.erase(sds[i].si.begin() + 23);
				sds[i].si.erase(sds[i].si.begin() + 21);
				sds[i].si.erase(sds[i].si.begin() + 19);
				sds[i].si.erase(sds[i].si.begin() + 15);
				sds[i].si.erase(sds[i].si.begin() + 9);
				sds[i].si.erase(sds[i].si.begin() + 5);
				sds[i].si.erase(sds[i].si.begin() + 3);
				sds[i].si.erase(sds[i].si.begin() + 1);
			}
		}

		if (mode == 0) { //for mode0, stats[sds[i].si[0]].part=2,stats[sds[i].si[1]].part=6,stats[sds[i].si[2]].part=7
			for (int i = 0; i<(int)sds.size(); i++) {
				sds[i].si.erase(sds[i].si.begin() + 24);
				sds[i].si.erase(sds[i].si.begin() + 23);
				sds[i].si.erase(sds[i].si.begin() + 21);
				sds[i].si.erase(sds[i].si.begin() + 20);
				sds[i].si.erase(sds[i].si.begin() + 19);
				sds[i].si.erase(sds[i].si.begin() + 15);
				sds[i].si.erase(sds[i].si.begin() + 9);
				sds[i].si.erase(sds[i].si.begin() + 5);
				sds[i].si.erase(sds[i].si.begin() + 4);
				sds[i].si.erase(sds[i].si.begin() + 3);
				sds[i].si.erase(sds[i].si.begin() + 1);
				sds[i].si.erase(sds[i].si.begin());
			}
		}

		int tot_area = 0;
		tot_area = area[12] + 2 * area[7] + 2 * area[11] + 4 * area[6] + 2 * area[2] + 2 * area[10];
		if (mode > 0)
			tot_area += 4 * area[0];
		if (mode > 1)
			tot_area += 4 * area[1] + 4 * area[5];
		arf = 256.0 / (sqrt(tot_area) * (wp.gray_w - wp.gray_i));
		return 0;
	}

	PAIR_ULL compute(int x0, int y0, const Mat &, const Mat & ig, const Mat &) {
		CV_Assert(x0 + dx[0] >= 0 && y0 + dy[0] >= 0 && x0 + dx[5] < ig.cols && y0 + dy[30] < ig.rows);
		PAIR_ULL ret = make_pair(0xffffffffffffffffULL, 0xffffffffffffffffULL);
		unsigned s[36];
		const unsigned *p_ig = ig.ptr<unsigned>(y0, x0);
		for (int i = 0; i < 36; i++)
			s[i] = p_ig[offset[i]];

		int sum[25], e[30];
		e[2] = s[8] - s[2];
		e[3] = s[9] - s[3];
		e[7] = s[13] - s[7];
		e[8] = s[14] - s[8];
		e[9] = s[15] - s[9];
		e[10] = s[16] - s[10];
		e[12] = s[18] - s[12];
		e[13] = s[19] - s[13];
		e[14] = s[20] - s[14];
		e[15] = s[21] - s[15];
		e[16] = s[22] - s[16];
		e[17] = s[23] - s[17];
		e[19] = s[25] - s[19];
		e[20] = s[26] - s[20];
		e[21] = s[27] - s[21];
		e[22] = s[28] - s[22];
		e[26] = s[32] - s[26];
		e[27] = s[33] - s[27];

		sum[2] = e[3] - e[2];
		sum[6] = e[8] - e[7];
		sum[7] = e[9] - e[8];
		sum[8] = e[10] - e[9];
		sum[10] = e[13] - e[12];
		sum[11] = e[14] - e[13];
		sum[12] = e[15] - e[14];
		sum[13] = e[16] - e[15];
		sum[14] = e[17] - e[16];
		sum[16] = e[20] - e[19];
		sum[17] = e[21] - e[20];
		sum[18] = e[22] - e[21];
		sum[22] = e[27] - e[26];
#ifdef QT_DEBUG
		CV_Assert(sum[2] >= 0 && sum[6] >= 0 && sum[7] >= 0 && sum[8] >= 0);
		CV_Assert(sum[10] >= 0 && sum[11] >= 0 && sum[12] >= 0);
		CV_Assert(sum[13] >= 0 && sum[14] >= 0 && sum[16] >= 0);
		CV_Assert(sum[17] >= 0 && sum[18] >= 0 && sum[22] >= 0);
#endif
		if (mode > 0) {
			e[0] = s[6] - s[0];
			e[1] = s[7] - s[1];
			e[4] = s[10] - s[4];
			e[5] = s[11] - s[5];
			e[6] = s[12] - s[6];
			e[11] = s[17] - s[11];
			e[18] = s[24] - s[18];
			e[23] = s[29] - s[23];
			e[24] = s[30] - s[24];
			e[25] = s[31] - s[25];
			e[28] = s[34] - s[28];
			e[29] = s[35] - s[29];

			sum[0] = e[1] - e[0];
			sum[4] = e[5] - e[4];
			sum[20] = e[25] - e[24];
			sum[24] = e[29] - e[28];
#ifdef QT_DEBUG
			CV_Assert(sum[0] >= 0 && sum[4] >= 0 && sum[20] >= 0 && sum[24] >= 0);
#endif
			if (mode == 2) {
				sum[1] = e[2] - e[1];
				sum[3] = e[4] - e[3];
				sum[5] = e[7] - e[6];
				sum[9] = e[11] - e[10];
				sum[15] = e[19] - e[18];
				sum[19] = e[23] - e[22];
				sum[21] = e[26] - e[25];
				sum[23] = e[28] - e[27];
#ifdef QT_DEBUG
				CV_Assert(sum[1] >= 0 && sum[3] >= 0 && sum[5] >= 0);
				CV_Assert(sum[9] >= 0 && sum[15] >= 0 && sum[19] >= 0);
				CV_Assert(sum[21] >= 0 && sum[23] >= 0);
#endif
			}
		}
		for (int i = 0; i < 25; i++)
			stats[i].err = -1;
		for (int i = 25; i < (int)stats.size(); i++) {
			stats[i].err = abs(stats[i].sum - sum[stats[i].part]);
			if (stats[stats[i].part].err < 0)
				stats[stats[i].part].err = stats[i].err;
			else
				stats[stats[i].part].err = min(stats[stats[i].part].err, stats[i].err);
		}

		int minimum = 0x7fffffff, minimum2 = 0x7fffffff;
		int bestshape = 0, bestshape2 = 0;
		for (int i = 0; i < (int)sds.size(); i++) {
			int tot_err = 0;
			for (int j = 0; j < (int)sds[i].si.size(); j++)
				tot_err += stats[sds[i].si[j]].err;
			if (tot_err < minimum) {
				minimum2 = minimum;
				bestshape2 = bestshape;
				minimum = tot_err;
				bestshape = i;
			}
			else
			if (tot_err < minimum2) {
				minimum2 = tot_err;
				bestshape2 = i;
			}
		}
		CV_Assert(minimum >= -8 && minimum2 >= -8);
		int overlap_min = minimum, overlap_min2 = minimum2;
		for (int j = 0; j < (int)sds[bestshape].si.size(); j++) {
			CV_Assert(stats[sds[bestshape].si[j]].part == stats[sds[bestshape2].si[j]].part);
			int shape_need = wire_25_shape[sds[bestshape].w25_shape_idx].ratio[stats[sds[bestshape2].si[j]].part];
			int shape_need2 = wire_25_shape[sds[bestshape2].w25_shape_idx].ratio[stats[sds[bestshape].si[j]].part];
			if (shape_need < 0 || shape_need2 < 0) {
				overlap_min -= stats[sds[bestshape].si[j]].err;
				overlap_min2 -= stats[sds[bestshape2].si[j]].err;
			}
		}

		if (overlap_min > overlap_min2) {
			swap(minimum, minimum2);
			swap(bestshape, bestshape2);
		}

		minimum = minimum * arf;
		minimum2 = minimum2 * arf;
		minimum = max(MIN_SCORE, min(minimum, 65535));
		minimum2 = max(MIN_SCORE, min(minimum2, 65535));
		ret.first = MAKE_PROB(MAKE_S(minimum, wp.type, wire_25_shape[sds[bestshape].w25_shape_idx].shape), x0, y0);
		ret.second = MAKE_PROB(MAKE_S(minimum2, wp.type, wire_25_shape[sds[bestshape2].w25_shape_idx].shape), x0, y0);
		return ret;
	}


	int check_mark(int x0, int y0, const Mat & prob, Mat & mask, int type, const Mat & already_mask)
	{
		int ret = 0;
		CV_Assert(prob.rows * gs >= mask.rows && prob.cols * gs >= mask.cols &&
			mask.type() == CV_32SC1 && prob.type() == CV_64FC2);
		unsigned long long prob0 = prob.at<unsigned long long>(y0, 2 * x0);
		int b = PROB_SHAPE(prob0);
		if (b > BRICK_IN_USE)
			return ret;
		int x = PROB_X(prob0), y = PROB_Y(prob0);
		int xx[6], yy[6];
		for (int dir = 0; dir < 8; dir++) {
			if (!bricks[b].a[dxy[dir][0] + 1][dxy[dir][1] + 1])
				continue;
			for (int i = 0; i < sizeof(xx) / sizeof(xx[0]); i++) {
				xx[i] = x;
				yy[i] = y;
			}
			if (dir == DIR_UP || dir == DIR_UPRIGHT || dir == DIR_UPLEFT) {
				yy[0] = y - wp.w_high / 2 - wp.i_high - wp.w_high1;
				yy[1] = yy[0] - 1;
				yy[2] = yy[0] - 2;
				yy[3] = yy[0] + 1;
				yy[4] = yy[0] + 2;
				yy[5] = yy[0] + 3;
			}
			if (dir == DIR_RIGHT || dir == DIR_UPRIGHT || dir == DIR_DOWNRIGHT) {
				xx[0] = x + wp.w_wide1 + wp.i_wide + wp.w_wide - wp.w_wide / 2;
				xx[1] = xx[0] + 1;
				xx[2] = xx[0] + 2;
				xx[3] = xx[0] - 1;
				xx[4] = xx[0] - 2;
				xx[5] = xx[0] - 3;
			}
			if (dir == DIR_DOWN || dir == DIR_DOWNRIGHT || dir == DIR_DOWNLEFT) {
				yy[0] = y + wp.w_high - wp.w_high / 2 + wp.i_high + wp.w_high1;
				yy[1] = yy[0] + 1;
				yy[2] = yy[0] + 2;
				yy[3] = yy[0] - 1;
				yy[4] = yy[0] - 2;
				yy[5] = yy[0] - 3;
			}
			if (dir == DIR_LEFT || dir == DIR_UPLEFT || dir == DIR_DOWNLEFT) {
				xx[0] = x - wp.w_wide / 2 - wp.i_wide - wp.w_wide1;
				xx[1] = xx[0] - 1;
				xx[2] = xx[0] - 2;
				xx[3] = xx[0] + 1;
				xx[4] = xx[0] + 2;
				xx[5] = xx[0] + 3;
			}
			for (int i = 0; i < 5; i++)
			if (!(already_mask.at<int>(yy[i], xx[i]) & (1 << type))) {
				mask.at<int>(yy[i], xx[i]) |= 1 << type;
				ret++;
			}
		}
		return ret;
	}
	void check_via_wire_connect(Mat &, Mat &, Mat &, Mat &)
	{
	}
};

map<int, int> Wire_25RectCompute::wire_25_shape_map;

struct Wire13ShapeConst {
	int sel[13];
	int shape;
} wire_13_shape[] = {
	//  0  1  2  3  4  5  6  7  8  9 10 11 12  
	{ { 2, 0, 2, 0, 1, 0, 2, 0, 2, 0, 0, 0, 0 }, BRICK_ONE_POINT },
	{ { 1, 1, 2, 0, 1, 0, 2, 0, 2, 0, 0, 0, 0 }, BRICK_i_0 },
	{ { 2, 0, 2, 0, 1, 1, 1, 0, 2, 0, 0, 0, 0 }, BRICK_i_90 },
	{ { 2, 0, 2, 0, 1, 0, 2, 1, 1, 0, 0, 0, 0 }, BRICK_i_180 },
	{ { 2, 0, 1, 1, 1, 0, 2, 0, 2, 0, 0, 0, 0 }, BRICK_i_270 },
	{ { 1, 1, 2, 0, 1, 0, 2, 1, 1, 0, 0, 0, 0 }, BRICK_I_0 },
	{ { 2, 0, 1, 1, 1, 1, 1, 0, 2, 0, 0, 0, 0 }, BRICK_I_90 },
	{ { 1, 1, 2, 0, 1, 1, 1, 0, 2, 0, 0, 0, 0 }, BRICK_L_0 },
	{ { 2, 0, 2, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_L_90 },
	{ { 2, 0, 1, 1, 1, 0, 2, 1, 1, 0, 0, 0, 0 }, BRICK_L_180 },
	{ { 1, 1, 1, 1, 1, 0, 2, 0, 2, 0, 0, 0, 0 }, BRICK_L_270 },
	{ { 1, 1, 2, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_T_0 },
	{ { 2, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_T_90 },
	{ { 1, 1, 1, 1, 1, 0, 2, 1, 1, 0, 0, 0, 0 }, BRICK_T_180 },
	{ { 1, 1, 1, 1, 1, 1, 1, 0, 2, 0, 0, 0, 0 }, BRICK_T_270 },
	{ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_X_0 },
	//  0  1  2  3  4  5  6  7  8  9 10 11 12  
	{ { 2, 2, 2, 2, 0, 2, 2, 2, 2, 0, 0, 0, 0 }, BRICK_HOLLOW },
	{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_NO_WIRE },
	{ { 2, 1, 2, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1 }, BRICK_FAKE_VIA },
	{ { 2, 1, 2, 0, 1, 1, 2, 1, 2, 0, 1, 0, 1 }, BRICK_FAKE_VIA },
	{ { 2, 1, 2, 1, 1, 0, 2, 1, 2, 1, 0, 1, 0 }, BRICK_FAKE_VIA },
	{ { 2, 0, 2, 1, 1, 1, 2, 1, 2, 0, 0, 1, 1 }, BRICK_FAKE_VIA },
	{ { 2, 1, 2, 1, 1, 1, 2, 0, 2, 1, 1, 0, 0 }, BRICK_FAKE_VIA },
	//  0  1  2  3  4  5  6  7  8  9 10 11 12  
};

class Wire_13RectCompute : public WireComputeScore {
protected:
	bool reset_offset;
	int offset[24], dx[24], dy[24]; //offset is compute based on dx and dy
	float area_1[13]; //area is 1 / rec_area
	int searchx_extend, searchy_extend;
	int gs, compute_border;
	WireParameter wp;
	float a[13]; //a is each rec_area / total_area
	float arf;

public:
	int prepare(WireParameter & _wp, PipeData & d, int layer) {
		d.l[layer].validate_ig();
		wp = _wp;
		float gamma = wp.arfactor / 100.0;
		qInfo("Wire_13Rect Prepare w=%d,h=%d,w1=%d,h1=%d,i_w=%d,i_h=%d gamma=%f", wp.w_wide, wp.w_high,
			wp.w_wide1, wp.w_high1, wp.i_wide, wp.i_high, gamma);
		if (wp.w_wide < d.l[layer].gs || wp.w_high < d.l[layer].gs)
			qWarning("w_wide(%d) or w_high(%d) < gs(%d)", wp.w_wide, wp.w_high, d.l[layer].gs);
		if (gamma >= 1) {
			qWarning("gamma(%f) >=1, force it to 1", gamma);
			gamma = 1;
		}
		reset_offset = true;
		area_1[0] = wp.w_wide * wp.w_high1;
		area_1[1] = wp.w_wide * wp.i_high;
		area_1[2] = wp.w_high * wp.w_wide1;
		area_1[3] = wp.w_high * wp.i_wide;
		area_1[4] = wp.w_wide * wp.w_high;
		area_1[5] = area_1[3];
		area_1[6] = area_1[2];
		area_1[7] = area_1[1];
		area_1[8] = area_1[0];
		area_1[9] = wp.i_wide * wp.i_high;
		area_1[10] = area_1[9];
		area_1[11] = area_1[9];
		area_1[12] = area_1[9];
		dx[0] = -wp.w_wide / 2, dy[0] = -wp.w_high1 - wp.i_high - wp.w_high / 2;
		dx[1] = wp.w_wide - wp.w_wide / 2, dy[1] = dy[0];
		dx[2] = -wp.i_wide - wp.w_wide / 2, dy[2] = -wp.i_high - wp.w_high / 2;
		dx[3] = dx[0], dy[3] = dy[2];
		dx[4] = dx[1], dy[4] = dy[2];
		dx[5] = wp.w_wide - wp.w_wide / 2 + wp.i_wide, dy[5] = dy[2];
		dx[6] = -wp.w_wide1 - wp.i_wide - wp.w_wide / 2, dy[6] = -wp.w_high / 2;
		dx[7] = dx[2], dy[7] = dy[6];
		dx[8] = dx[3], dy[8] = dy[6];
		dx[9] = dx[4], dy[9] = dy[6];
		dx[10] = dx[5], dy[10] = dy[6];
		dx[11] = wp.w_wide1 + wp.i_wide + wp.w_wide - wp.w_wide / 2, dy[11] = dy[6];
		dx[12] = dx[6], dy[12] = wp.w_high - wp.w_high / 2;
		dx[13] = dx[7], dy[13] = dy[12];
		dx[14] = dx[8], dy[14] = dy[12];
		dx[15] = dx[9], dy[15] = dy[12];
		dx[16] = dx[10], dy[16] = dy[12];
		dx[17] = dx[11], dy[17] = dy[12];
		dx[18] = dx[13], dy[18] = dy[12] + wp.i_high;
		dx[19] = dx[14], dy[19] = dy[18];
		dx[20] = dx[15], dy[20] = dy[18];
		dx[21] = dx[16], dy[21] = dy[18];
		dx[22] = dx[19], dy[22] = dy[18] + wp.w_high1;
		dx[23] = dx[20], dy[23] = dy[22];

		if (abs(dy[0]) >= d.l[layer].compute_border || abs(dy[23]) >= d.l[layer].compute_border) {
			qCritical("dy (%d, %d) exceed compute_border(%d)", dy[0], dy[23], d.l[layer].compute_border);
			return -1;
		}
		if (abs(dx[6]) >= d.l[layer].compute_border || abs(dx[11]) >= d.l[layer].compute_border) {
			qCritical("dx (%d, %d) exceed compute_border(%d)", dx[6], dx[11], d.l[layer].compute_border);
			return -1;
		}

		float area = 0;
		for (int j = 0; j < 13; j++)
			area += area_1[j];

		for (int j = 0; j < 13; j++)
			a[j] = area_1[j] / area;
		arf = pow(area, 2 * gamma);


		for (int i = 0; i < 13; i++)
			area_1[i] = 1 / area_1[i];

		gs = d.l[layer].gs;
		compute_border = d.l[layer].compute_border;
		searchx_extend = (wp.w_wide + wp.w_wide1 + wp.i_wide) / gs;
		searchy_extend = (wp.w_high + wp.w_high1 + wp.i_high) / gs;
		return 0;
	}

	//return 1st and 2nd likelihood brick for img, (x0,y0)
	PAIR_ULL compute(int x0, int y0, const Mat &, const Mat & ig, const Mat & iig) {
		PAIR_ULL ret = make_pair(0xffffffffffffffffULL, 0xffffffffffffffffULL);
		unsigned s[24], sq[24];
		int sum[13], ssum[13];
		float part[3][13];
		const unsigned * p_ig, *p_iig;
		if (reset_offset) {
			reset_offset = false;
			for (int i = 0; i < 24; i++)
				offset[i] = (dy[i] * (int)ig.step.p[0] + dx[i] * (int)ig.step.p[1]) / sizeof(unsigned);
		}
		p_ig = ig.ptr<unsigned>(y0, x0);
		p_iig = iig.ptr<unsigned>(y0, x0);
		for (int i = 0; i < 24; i++) {
			s[i] = p_ig[offset[i]];
			sq[i] = p_iig[offset[i]];
		}
		sum[0] = s[0] + s[4] - s[1] - s[3]; //area 0 gray sum
		ssum[0] = sq[0] + sq[4] - sq[1] - sq[3]; //area 0 gray square sum
		sum[9] = s[2] + s[8] - s[3] - s[7];
		ssum[9] = sq[8] + sq[2] - sq[3] - sq[7];
		sum[1] = s[3] + s[9] - s[4] - s[8];
		ssum[1] = sq[3] + sq[9] - sq[4] - sq[8];
		sum[10] = s[4] + s[10] - s[5] - s[9];
		ssum[10] = sq[4] + sq[10] - sq[5] - sq[9];
		sum[2] = s[6] + s[13] - s[7] - s[12];
		ssum[2] = sq[6] + sq[13] - sq[7] - sq[12];
		sum[3] = s[7] + s[14] - s[8] - s[13];
		ssum[3] = sq[7] + sq[14] - sq[8] - sq[13];
		sum[4] = s[8] + s[15] - s[9] - s[14];
		ssum[4] = sq[8] + sq[15] - sq[9] - sq[14];
		sum[5] = s[9] + s[16] - s[10] - s[15];
		ssum[5] = sq[9] + sq[16] - sq[10] - sq[15];
		sum[6] = s[10] + s[17] - s[11] - s[16];
		ssum[6] = sq[10] + sq[17] - sq[11] - sq[16];
		sum[11] = s[13] + s[19] - s[14] - s[18];
		ssum[11] = sq[13] + sq[19] - sq[14] - sq[18];
		sum[7] = s[14] + s[20] - s[15] - s[19];
		ssum[7] = sq[14] + sq[20] - sq[15] - sq[19];
		sum[12] = s[15] + s[21] - s[16] - s[20];
		ssum[12] = sq[15] + sq[21] - sq[16] - sq[20];
		sum[8] = s[19] + s[23] - s[20] - s[22];
		ssum[8] = sq[19] + sq[23] - sq[20] - sq[22];
		int gm = wp.gray_w, gi = wp.gray_i;
		int gmm = gm*gm, gii = gi*gi;
		for (int i = 0; i < 13; i++) {
			part[0][i] = (ssum[i] - gi*sum[i] * 2) * area_1[i] + gii;   //area i prob for insu
			part[1][i] = (ssum[i] - gm*sum[i] * 2) * area_1[i] + gmm;   //area i prob for wire
			part[2][i] = min(part[0][i], part[1][i]);					//area i prob for not-care
		}
		//compute score for each brick
		for (int i = 0; i < sizeof(wire_13_shape) / sizeof(wire_13_shape[0]); i++) { //scan for each shape
			unsigned long long score = 0;
			for (int j = 0; j < 13; j++)
				score += part[wire_13_shape[i].sel[j]][j] * a[j];
			score = score * arf;
			score = MAKE_PROB(score, wp.type, wire_13_shape[i].shape); //save sqrt call times
			push_min(ret, score);
		}
		unsigned score = PROB_S(ret.first);
		score = sqrt((float)score);
		CV_Assert(score < 65536);
		score = (score < MIN_SCORE) ? MIN_SCORE : score;
		score = MAKE_S(score, PROB_X(ret.first), PROB_Y(ret.first));
		ret.first = MAKE_PROB(score, x0, y0);
		score = PROB_S(ret.second);
		score = sqrt((float)score);
		CV_Assert(score < 65536);
		score = (score < MIN_SCORE) ? MIN_SCORE : score;
		score = MAKE_S(score, PROB_X(ret.second), PROB_Y(ret.second));
		ret.second = MAKE_PROB(score, x0, y0);
		return ret;
	}

	int check_mark(int x0, int y0, const Mat & prob, Mat & mask, int type, const Mat & already_mask)
	{
		int ret = 0;
		CV_Assert(prob.rows * gs >= mask.rows && prob.cols * gs >= mask.cols &&
			mask.type() == CV_32SC1 && prob.type() == CV_64FC2);
		unsigned long long prob0 = prob.at<unsigned long long>(y0, 2 * x0);
		int b = PROB_SHAPE(prob0);
		if (b > BRICK_IN_USE)
			return ret;
		int x = PROB_X(prob0), y = PROB_Y(prob0);
		int xx[6], yy[6];
		for (int dir = 0; dir <= 3; dir++) {
			if (!bricks[b].a[dxy[dir][0] + 1][dxy[dir][1] + 1])//if need to check extend
				continue;
			//following check if extend is already computed
#if 0	
			bool check = false;

			int extend = (dir == DIR_UP || dir == DIR_DOWN) ? searchy_extend : searchx_extend;
			int xx0 = x0 + dxy[dir][1], yy0 = y0 + dxy[dir][0];
			for (int i = 0; i < extend; i++) {
				if (yy0 <= compute_border / gs || yy0 >= (mask.rows - compute_border) / gs ||
					xx0 <= compute_border / gs || xx0 >= (mask.cols - compute_border) / gs) {
					check = true;
					break;
				}
				if (dir == DIR_UP || dir == DIR_DOWN) {
					if (PROB_S(prob.at<unsigned long long>(yy0, xx0 * 2)) != 0xffffffff &&
						brick_conn.fit(dir, PROB_SHAPE(prob.at<unsigned long long>(yy0, xx0 * 2)), b) ||
						PROB_S(prob.at<unsigned long long>(yy0, xx0 * 2 - 2)) != 0xffffffff &&
						brick_conn.fit(dir, PROB_SHAPE(prob.at<unsigned long long>(yy0, xx0 * 2 - 2)), b) ||
						PROB_S(prob.at<unsigned long long>(yy0, xx0 * 2 + 2)) != 0xffffffff &&
						brick_conn.fit(dir, PROB_SHAPE(prob.at<unsigned long long>(yy0, xx0 * 2 + 2)), b))
						check = true;
				}
				else {
					if (PROB_S(prob.at<unsigned long long>(yy0, xx0 * 2)) != 0xffffffff &&
						brick_conn.fit(dir, b, PROB_SHAPE(prob.at<unsigned long long>(yy0, xx0 * 2))) ||
						PROB_S(prob.at<unsigned long long>(yy0 - 1, xx0 * 2)) != 0xffffffff &&
						brick_conn.fit(dir, b, PROB_SHAPE(prob.at<unsigned long long>(yy0 - 1, xx0 * 2))) ||
						PROB_S(prob.at<unsigned long long>(yy0 + 1, xx0 * 2)) != 0xffffffff &&
						brick_conn.fit(dir, b, PROB_SHAPE(prob.at<unsigned long long>(yy0 + 1, xx0 * 2))))
						check = true;
				}
				xx0 += dxy[dir][1];
				yy0 += dxy[dir][0];
			}

			if (check) //if already compute, continue
				continue;
#endif	
			for (int i = 0; i < sizeof(xx) / sizeof(xx[0]); i++) {
				xx[i] = x;
				yy[i] = y;
			}
			switch (dir) {
			case DIR_UP:
				yy[0] = y - wp.w_high / 2 - wp.i_high - wp.w_high1;
				yy[1] = yy[0] - 1;
				yy[2] = yy[0] - 2;
				yy[3] = yy[0] + 1;
				yy[4] = yy[0] + 2;
				yy[5] = yy[0] + 3;
				break;
			case DIR_RIGHT:
				xx[0] = x + wp.w_wide1 + wp.i_wide + wp.w_wide - wp.w_wide / 2;
				xx[1] = xx[0] + 1;
				xx[2] = xx[0] + 2;
				xx[3] = xx[0] - 1;
				xx[4] = xx[0] - 2;
				xx[5] = xx[0] - 3;
				break;
			case DIR_DOWN:
				yy[0] = y + wp.w_high - wp.w_high / 2 + wp.i_high + wp.w_high1;
				yy[1] = yy[0] + 1;
				yy[2] = yy[0] + 2;
				yy[3] = yy[0] - 1;
				yy[4] = yy[0] - 2;
				yy[5] = yy[0] - 3;
				break;
			case DIR_LEFT:
				xx[0] = x - wp.w_wide / 2 - wp.i_wide - wp.w_wide1;
				xx[1] = xx[0] - 1;
				xx[2] = xx[0] - 2;
				xx[3] = xx[0] + 1;
				xx[4] = xx[0] + 2;
				xx[5] = xx[0] + 3;
				break;
			}
#if 0
			qInfo("new mark x=%d, y=%d for x=%d, y=%d", xx[0], yy[0], x, y);
#endif
			for (int i = 0; i < 5; i++)
			if (!(already_mask.at<int>(yy[i], xx[i]) & (1 << type))) {
				mask.at<int>(yy[i], xx[i]) |= 1 << type;
				ret++;
			}
		}
		return ret;
	}

	/*
	input: remove_via_mask
	inout: prob
	*/
	void check_via_wire_connect(Mat & via_mask, Mat & prob, Mat & img, Mat & ig)
	{
	}
};

WireComputeScore * WireComputeScore::create_wire_compute_score(WireParameter &wp, PipeData & d, int layer)
{
	int ret;
	WireComputeScore * wc = NULL;
	switch (wp.subtype) {
	case WIRE_SUBTYPE_13RECT:
		wc = new Wire_13RectCompute;
		ret = wc->prepare(wp, d, layer);
		if (ret != 0) {
			delete wc;
			wc = NULL;
		}
		break;
	case WIRE_SUBTYPE_25RECT:
		wc = new Wire_25RectCompute;
		ret = wc->prepare(wp, d, layer);
		if (ret != 0) {
			delete wc;
			wc = NULL;
		}
		break;
	default:
		qCritical("WireComputeScore create failed, subtype=%d", wp.subtype);
		break;
	}
	return wc;
}

/*		31..24  23..16		15..8		7..0
opt0:	  1  border_size compute_border gs
opt1:	via_connect via_merge wire_connect wire_merge
opt2:                         opt       end_limit
*/
void set_pipeline_param(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	qInfo("set param for layer %d", layer);
	if (cpara.opt0 >> 24 == 1) {
		int gs = cpara.opt0 & 0xff;
		int compute_border = cpara.opt0 >> 8 & 0xff;
		int border_size = cpara.opt0 >> 16 & 0xff;
		qInfo("set gs=%d, compute_border=%d, border_size=%d", gs, compute_border, border_size);

		PipeDataPerLayer::GuardLen guard;
		guard.via_connect = cpara.opt1 >> 24 & 0xff;
		guard.via_merge = cpara.opt1 >> 16 & 0xff;
		guard.wire_connect = cpara.opt1 >> 8 & 0xff;
		guard.wire_merge = cpara.opt1 & 0xff;
		guard.end_wire = cpara.opt2 & 0xff;

		if (gs > 32 || gs <= 3 || gs * 2 > compute_border || compute_border >= border_size) {
			qCritical("gs invalid");
			return;
		}

		d.l[layer].reinit(gs, compute_border, border_size, guard);

		return;
	}
	if (layer >= d.l.size() || layer < 0) {
		qCritical("layer invalid");
		return;
	}
	d.l[layer].vw.set_vw_para(cpara);
}
/*
opt0 is red ratio, better less than 10000
opt1 is green ratio, better less than 10000
opt2 is blue ratio, better less than 10000
Input d.img,
output d.img
*/
static void imgpp_RGB2gray(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	if (img.type() != CV_8UC3) {
		qCritical("imgpp_RGB2gray wrong image type(%x) !=%x", img.type(), CV_8UC3);
		return;
	}
	int total = cpara.opt0 + cpara.opt1 + cpara.opt2;
	qInfo("RGB2gray, c0=%d, c1=%d, c2=%d", cpara.opt0, cpara.opt1, cpara.opt2);
	if (total <= 0 || cpara.opt0 < 0 || cpara.opt1 < 0 || cpara.opt2 < 0) {
		qCritical("RGB2gray wrong parameter");
		return;
	}
	int c0_ratio = cpara.opt0 * 256 / total;
	int c1_ratio = cpara.opt1 * 256 / total;
	int c2_ratio = cpara.opt2 * 256 / total;

	Mat img_gray(img.rows, img.cols, CV_8UC1);
	for (int y = 0; y < img.rows; y++) {
		const unsigned char *p_img = img.ptr<unsigned char>(y);
		unsigned char * p_gray = img_gray.ptr<unsigned char>(y);
		for (int x = 0; x < img.cols; x++, p_img += 3)
			p_gray[x] = (c0_ratio * p_img[0] + c1_ratio * p_img[1] + c2_ratio * p_img[2]) >> 8;
	}
	d.l[layer].ig_valid = false;
	d.l[layer].img = img_gray;
	if (cpara.method & OPT_DEBUG_EN) {
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_imgpp_RGB2gray.jpg", d.l[layer].img);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = d.l[layer].img.clone();
		}
	}

}

/*
opt0 is grid
opt1 is min distribution percent compute
opt2 is grid_win
opt3 is k_size
opt4 is filter_method
Input d.img,
output d.img
local_min = sort d.img gray in grid_win * grid , and then choose low min_dis gray
filter_min = GaussianBlur(local_min, k_size)
k_size > grid, ksize<min(img.row,img.cols)
d.img = d.img - filter_min
*/
static void imgpp_compute_min_stat(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);
	int grid = cpara.opt0; //used for stat grid
	float min_dis = cpara.opt1 / 100.0; //used for min compute	
	int grid_win = cpara.opt2;
	int k_size = cpara.opt3; //used for min filter
	while (grid % 4 != 0)
		grid++;
	if (grid_win % 2 == 0)
		grid_win++;
	qInfo("imgpp_compute_min_stat, l=%d, min_dis=%f, grid=%d, grid_win=%d, k_size=%d", layer, min_dis, grid, grid_win, k_size);
	if (min_dis >= 1 || k_size > img.rows || k_size > img.cols || k_size <= grid || grid <= 16 || grid_win <= 0
		|| grid_win > 7 || grid * grid_win > img.rows || grid * grid_win > img.cols) {
		qCritical("imgpp_compute_min_stat wrong parameter");
		return;
	}
	k_size = k_size * 4 / grid;
	if (k_size % 2 == 0)
		k_size++;
	int sz[] = { img.rows / grid, img.cols / grid, 257 };
	Mat gstat(3, sz, CV_32S);
	gstat = Scalar(0);

	for (int y0 = grid; y0 <= img.rows; y0 += grid) {
		for (int x0 = grid; x0 <= img.cols; x0 += grid) {
			int y1 = (y0 + grid > img.rows) ? img.rows : y0;
			int x1 = (x0 + grid > img.cols) ? img.cols : x0;
			int * p_gstat = gstat.ptr<int>(y0 / grid - 1, x0 / grid - 1);
			for (int y = y0 - grid; y < y1; y++) {
				const unsigned char * p_img = img.ptr<unsigned char>(y);
				for (int x = x0 - grid; x < x1; x++) {
					p_gstat[p_img[x]]++;
					p_gstat[256]++;
				}
			}
		}
	}

	Mat local_min(img.rows / grid, img.cols / grid, CV_8UC1);
	int stat[257];
	for (int y0 = 0; y0 < gstat.size[0]; y0++) {
		for (int x0 = 0; x0 < gstat.size[1]; x0++) {
			int y1 = min(y0 + grid_win / 2, gstat.size[0] - 1);
			y1 = max(y1 - grid_win, 0);
			int x1 = min(x0 + grid_win / 2, gstat.size[1] - 1);
			x1 = max(x1 - grid_win, 0);
			memset(stat, 0, sizeof(stat));
			for (int y = y1; y < y1 + grid_win; y++)
			for (int x = x1; x < x1 + grid_win; x++) {
				int * p_gstat = gstat.ptr<int>(y, x);
				for (int i = 0; i <= 256; i++)
					stat[i] += p_gstat[i];
			}
			int th = stat[256] * min_dis;
			for (int i = 0; i < 256; i++) {
				th -= stat[i];
				if (th <= 0) {
					if (i > 0)
						i--;
					local_min.at<unsigned char>(y0, x0) = i;
					break;
				}
			}
		}
	}

	Mat ipl_min(local_min.rows * 4, local_min.cols * 4, CV_8UC1);
	for (int y = 0; y < ipl_min.rows; y++) {
		unsigned char * p_ipl_min = ipl_min.ptr<unsigned char>(y);
		unsigned char * p_min = local_min.ptr<unsigned char>(y / 4);
		for (int x = 0; x < ipl_min.cols; x++)
			p_ipl_min[x] = p_min[x / 4];
	}
	Mat filter_min;
	int filter_method = cpara.opt4 >> 24 & 0xff;
	switch (filter_method) {
	case 0:
		GaussianBlur(ipl_min, filter_min, Size(k_size, k_size), 0);
		break;
	default:
		filter_min = ipl_min;
	}

	for (int y = 0; y < img.rows; y++) {
		unsigned char * p_img = img.ptr<unsigned char>(y);
		int y1 = (y / (grid / 4) >= filter_min.rows) ? filter_min.rows - 1 : y / (grid / 4);
		unsigned char * p_min = filter_min.ptr<unsigned char>(y1);
		for (int x = 0, x1 = 0, xx = 0; x < img.cols; x++, xx++) {
			if (xx == grid / 4) {
				xx = 0;
				x1 = (x1 + 1 >= filter_min.cols) ? filter_min.cols - 1 : x1 + 1;
			}
			p_img[x] = (p_img[x] > p_min[x1]) ? p_img[x] - p_min[x1] : 0;
		}
	}
	d.l[layer].ig_valid = false;
	if (cpara.method & OPT_DEBUG_EN) {
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_imgpp_compute_min_stat_out.jpg", img);
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_imgpp_compute_min_stat_min.jpg", filter_min);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = img.clone();
		}
	}
}

/*     31..24  23..16   15..8   7..0
opt0: dis_min  wsize  sep_max  sep_min for most dark obj (may or may not be insu)
opt1: dis_min  wsize  sep_max  sep_min for middle bright obj (may or may not be wire)
opt2: dis_max  wsize  sep_max  sep_min for most bright obj (may or may not be via)
opt3: gray_lvl_opt  k2       k1      k0      k0 is dark gray level compress rate, k1 & k2 is rate between most dark and middle bright
opt4:           k2       k1      k0      k0 is middle bright level compress rate, k1 & k2 is rate between middle bright and most bright
opt5:           k2       k1      k0      k0 is most bright compress rate, k1 & k2 is rate between most bright
sep_min & sep_max means minmum and maximum gray level, L0.wsize is sep window size between L1 and L0
L1.wsize is sep window size between L2 and L1
e.g. L0.wsize > L1.sep_min - L0.sep_max, L1.wsize > L2.sep_min - L1.sep_max
normally choose k2 & k0 = 100, k1 < 100
method_opt
0: for gray level turn_points output
*/
static void imgpp_adjust_gray_lvl(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);
	int gray_lvl_opt = cpara.opt3 >> 24 & 0xff;
	int idx0 = cpara.method_opt & 0xf;
	qInfo("imgpp_adjust_gray_lvl l=%d, tp_idx=%d", layer, idx0);
	d.l[layer].v[idx0].type = TYPE_GRAY_LEVEL;
	
	struct {
		int sep_min, sep_max;
		int wsize;
		float dis_min, dis_max;
		float k0, k1, k2;
		int g0;
	} lmh[3] = {
		{ cpara.opt0 & 0xff, cpara.opt0 >> 8 & 0xff, cpara.opt0 >> 16 & 0xff, (cpara.opt0 >> 24 & 0xff) / 100.0, 1,
		(cpara.opt3 & 0xff) / 100.0, (cpara.opt3 >> 8 & 0xff) / 100.0, (cpara.opt3 >> 16 & 0xff) / 100.0 },
		{ cpara.opt1 & 0xff, cpara.opt1 >> 8 & 0xff, cpara.opt1 >> 16 & 0xff, (cpara.opt1 >> 24 & 0xff) / 100.0, 1,
		(cpara.opt4 & 0xff) / 100.0, (cpara.opt4 >> 8 & 0xff) / 100.0, (cpara.opt4 >> 16 & 0xff) / 100.0 },
		{ cpara.opt2 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt2 >> 16 & 0xff, 0, (cpara.opt2 >> 24 & 0xff) / 100.0,
		(cpara.opt5 & 0xff) / 100.0, (cpara.opt5 >> 8 & 0xff) / 100.0, (cpara.opt5 >> 16 & 0xff) / 100.0 }
	};

	Mat & m = d.l[layer].v[idx0].d;
	m.create(11, 5, CV_32SC1);
	if (gray_lvl_opt & ADJUST_GRAY_LVL_MANUAL_SET) {
		m.at<int>(0, 0) = GRAY_ZERO;
		m.at<int>(0, 1) = 0;
		m.at<int>(0, 2) = 0;
		m.at<int>(0, 3) = 0;
		m.at<int>(0, 4) = 0;
		m.at<int>(1, 0) = GRAY_L1;
		m.at<int>(1, 1) = lmh[0].sep_min;
		m.at<int>(1, 2) = lmh[0].sep_min;
		m.at<int>(1, 3) = lmh[0].k0 * 100;
		m.at<int>(1, 4) = lmh[0].k0 * 100;
		m.at<int>(2, 0) = GRAY_L0;
		m.at<int>(2, 1) = (lmh[0].sep_min + lmh[0].sep_max) / 2;
		m.at<int>(2, 2) = m.at<int>(2, 1);
		m.at<int>(2, 3) = lmh[0].k0 * 100;
		m.at<int>(2, 4) = lmh[0].k0 * 100;
		m.at<int>(3, 0) = GRAY_L2;
		m.at<int>(3, 1) = lmh[0].sep_max;
		m.at<int>(3, 2) = lmh[0].sep_max;
		m.at<int>(3, 3) = lmh[0].k1 * 100;
		m.at<int>(3, 4) = lmh[0].k2 * 100;
		m.at<int>(4, 0) = GRAY_M1;
		m.at<int>(4, 1) = lmh[1].sep_min;
		m.at<int>(4, 2) = lmh[1].sep_min;
		m.at<int>(4, 3) = lmh[1].k0 * 100;
		m.at<int>(4, 4) = lmh[1].k0 * 100;
		m.at<int>(5, 0) = GRAY_M0;
		m.at<int>(5, 1) = (lmh[1].sep_min + lmh[1].sep_max) / 2;
		m.at<int>(5, 2) = m.at<int>(5, 1);
		m.at<int>(5, 3) = lmh[1].k0 * 100;
		m.at<int>(5, 4) = lmh[1].k0 * 100;
		m.at<int>(6, 0) = GRAY_M2;
		m.at<int>(6, 1) = lmh[1].sep_max;
		m.at<int>(6, 2) = lmh[1].sep_max;
		m.at<int>(6, 3) = lmh[1].k1 * 100;
		m.at<int>(6, 4) = lmh[1].k2 * 100;
		m.at<int>(7, 0) = GRAY_H1;
		m.at<int>(7, 1) = lmh[2].sep_min;
		m.at<int>(7, 2) = lmh[2].sep_min;
		m.at<int>(7, 3) = lmh[2].k0 * 100;
		m.at<int>(7, 4) = lmh[2].k0 * 100;
		m.at<int>(8, 0) = GRAY_H0;
		m.at<int>(8, 1) = (lmh[2].sep_min + lmh[2].sep_max) / 2;;
		m.at<int>(8, 2) = m.at<int>(8, 1);
		m.at<int>(8, 3) = lmh[2].k0 * 100;
		m.at<int>(8, 4) = lmh[2].k0 * 100;
		m.at<int>(9, 0) = GRAY_H2;
		m.at<int>(9, 1) = lmh[2].sep_max;
		m.at<int>(9, 2) = lmh[2].sep_max;
		m.at<int>(9, 3) = 0;  
		m.at<int>(9, 4) = 0;  
		m.at<int>(10, 0) = GRAY_FULL;
		m.at<int>(10, 1) = 255;
		m.at<int>(10, 2) = m.at<int>(9, 2);
		return;
	}

	vector<int> stat(257, 0);
	for (int y = 0; y < img.rows; y += 2) {
		const unsigned char * p_img = img.ptr<unsigned char>(y);
		for (int x = 0; x < img.cols; x++) {
			stat[p_img[x]]++;
			stat[256]++;
		}
	}
	
	if (cpara.method & OPT_DEBUG_EN) {
		qDebug("imgpp_adjust_gray_lvl gray distribution");
		for (int i = 0; i < 32; i++) {
			int * ps = &stat[i * 8];
			qDebug("%3d:%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f", i * 8,
				(float)ps[0] / stat[256], (float)ps[1] / stat[256], (float)ps[2] / stat[256], (float)ps[3] / stat[256],
				(float)ps[4] / stat[256], (float)ps[5] / stat[256], (float)ps[6] / stat[256], (float)ps[7] / stat[256]);
		}
	}

	//following is compute lmh[0..2].g0
	int k_begin[2];
	for (int i = 0; i < 3; i++)
		qInfo("imgpp_adjust_gray_lvl %d:sep_min=%d, sep_max=%d,wsize=%d,dis_min=%f,dis_max=%f, k0=%f, k1=%f, k2=%f", i,
		lmh[i].sep_min, lmh[i].sep_max, lmh[i].wsize, lmh[i].dis_min, lmh[i].dis_max, lmh[i].k0, lmh[i].k1, lmh[i].k2);
	for (int i = 0; i < 2; i++) {
		int jb = max(lmh[i].sep_min + 1, lmh[i + 1].sep_min - lmh[i].wsize);
		int je = min(lmh[i].sep_max, lmh[i + 1].sep_max - lmh[i].wsize);
		if (jb > je)
			qCritical("param wrong for wsize(%d) < sep_min[%d](%d) - sep_max[%d](%d)", lmh[i].wsize,
			i + 1, lmh[i + 1].sep_min, i, lmh[i].sep_max);
		int min_sum = 0x7fffffff;
		for (int j = jb; j <= je; j++) {
			int sum = 0;
			for (int k = j; k < j + lmh[i].wsize; k++)
				sum += stat[k];
			if (sum < min_sum) {
				min_sum = sum;
				k_begin[i] = j;
			}
		}
		lmh[i + 1].sep_min = k_begin[i] + lmh[i].wsize;
	}
	lmh[0].sep_max = k_begin[0] - 1;
	lmh[0].wsize = lmh[0].sep_max - lmh[0].sep_min + 1;
	lmh[0].g0 = (lmh[0].sep_max + lmh[0].sep_min) / 2;
	lmh[1].sep_max = k_begin[1] - 1;
	lmh[1].wsize = lmh[1].sep_max - lmh[1].sep_min + 1;
	lmh[1].g0 = (lmh[1].sep_max + lmh[1].sep_min) / 2;
	lmh[2].wsize = lmh[2].sep_max - lmh[2].sep_min + 1;
	lmh[2].g0 = (lmh[2].sep_max + lmh[2].sep_min) / 2;
	for (int i = 0; i < 3; i++)
	if (lmh[i].wsize <= 0) {
		qCritical("Invalid param wsize=%d", lmh[i].wsize);
		return;
	}

	qInfo("imgpp_adjust_gray_lvl, l0=%d, w0=%d, m0=%d, w1=%d, h0=%d, w2=%d", lmh[0].g0, lmh[0].wsize,
		lmh[1].g0, lmh[1].wsize, lmh[2].g0, lmh[2].wsize);

	//following compute Gray level table
	m.at<int>(0, 0) = GRAY_ZERO;
	m.at<int>(0, 1) = 0;
	m.at<int>(0, 2) = max(0, lmh[0].g0 - (int)(lmh[0].wsize * lmh[0].k0 / 2));;
	m.at<int>(0, 3) = 0;
	m.at<int>(0, 4) = 0;
	m.at<int>(1, 0) = GRAY_L1;
	m.at<int>(1, 1) = max(0, lmh[0].g0 - lmh[0].wsize / 2);
	m.at<int>(1, 2) = m.at<int>(0, 2);
	m.at<int>(1, 3) = lmh[0].k0 * 100;
	m.at<int>(1, 4) = lmh[0].k0 * 100;
	m.at<int>(2, 0) = GRAY_L0;
	m.at<int>(2, 1) = lmh[0].g0;
	m.at<int>(2, 2) = lmh[0].g0;
	m.at<int>(2, 3) = lmh[0].k0 * 100;
	m.at<int>(2, 4) = lmh[0].k0 * 100;
	m.at<int>(3, 0) = GRAY_L2;
	m.at<int>(3, 1) = lmh[0].g0 + (lmh[0].wsize - 1) / 2;
	m.at<int>(3, 2) = lmh[0].g0 + (lmh[0].wsize - 1) * lmh[0].k0 / 2;
	m.at<int>(3, 3) = lmh[0].k1 * 100;
	m.at<int>(3, 4) = lmh[0].k2 * 100;
	m.at<int>(4, 0) = GRAY_M1;
	m.at<int>(4, 1) = lmh[1].g0 - lmh[1].wsize / 2;
	m.at<int>(4, 2) = lmh[1].g0 - lmh[1].wsize * lmh[1].k0 / 2;
	m.at<int>(4, 3) = lmh[1].k0 * 100;
	m.at<int>(4, 4) = lmh[1].k0 * 100;
	m.at<int>(5, 0) = GRAY_M0;
	m.at<int>(5, 1) = lmh[1].g0;
	m.at<int>(5, 2) = lmh[1].g0;
	m.at<int>(5, 3) = lmh[1].k0 * 100;
	m.at<int>(5, 4) = lmh[1].k0 * 100;
	m.at<int>(6, 0) = GRAY_M2;
	m.at<int>(6, 1) = lmh[1].g0 + (lmh[1].wsize - 1) / 2;
	m.at<int>(6, 2) = lmh[1].g0 + (lmh[1].wsize - 1) * lmh[1].k0 / 2;
	m.at<int>(6, 3) = lmh[1].k1 * 100;
	m.at<int>(6, 4) = lmh[1].k2 * 100;
	m.at<int>(7, 0) = GRAY_H1;
	m.at<int>(7, 1) = lmh[2].g0 - lmh[2].wsize / 2;
	m.at<int>(7, 2) = lmh[2].g0 - lmh[2].wsize * lmh[2].k0 / 2;
	m.at<int>(7, 3) = lmh[2].k0 * 100;
	m.at<int>(7, 4) = lmh[2].k0 * 100;
	m.at<int>(8, 0) = GRAY_H0;
	m.at<int>(8, 1) = lmh[2].g0;
	m.at<int>(8, 2) = lmh[2].g0;
	m.at<int>(8, 3) = lmh[2].k0 * 100;
	m.at<int>(8, 4) = lmh[2].k0 * 100;
	m.at<int>(9, 0) = GRAY_H2;
	m.at<int>(9, 1) = min(255, lmh[2].g0 + (lmh[2].wsize - 1) / 2);
	m.at<int>(9, 2) = m.at<int>(8, 2);//min(255, lmh[2].g0 + (int) ((lmh[2].wsize - 1) * lmh[2].k0 / 2));
	m.at<int>(9, 3) = 0;  //lmh[2].k1 * 100;
	m.at<int>(9, 4) = 0;  //lmh[2].k2 * 100;
	m.at<int>(10, 0) = GRAY_FULL;
	m.at<int>(10, 1) = 255;
	m.at<int>(10, 2) = m.at<int>(9, 2);
	Mat lut, out;
	cvt_tbl_hermite(m(Rect(1, 0, 4, m.rows)), lut, cpara.method & OPT_DEBUG_EN);

	//following do gray transform
	LUT(img, lut, out);
	img = out;
	if (cpara.method & OPT_DEBUG_EN) {
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_imgpp_adjust_gray_lvl.jpg", img);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = img.clone();
		}
	}
}

struct WireKeyPoint {
	Point offset[8];
	int * p_ig[8];
	int *p_iig[8];
	int type;
	int shape1, shape2, shape3, shape4;
	float a0, a1, a2;
	float arf;
};

struct WireDetectInfo {
	int w_subtype;
	int w_pattern;
	int w_dir;
	int w_type;
	int w_wide;
	int i_wide;
	float gamma;
};

/*		31..24  23..16   15..8   7..0
opt0:		    ed_guard ed_long detect_opt
opt1: grad_low_l grad_high_l gi_high_l gw_low_l
opt2: gw_high_l  dw_high_l grad_low_r grad_high_r 
opt3: gi_high_r gw_low_r gw_high_r dw_high_r
opt4: grad_low_u grad_high_u gi_high_u gw_low_u
opt5: gw_high_u  dw_high_u grad_low_d grad_high_d
opt6: gi_high_d gw_low_d gw_high_d dw_high_d
compute grad for [ed_wide*ed_long], ed_long chose 3..30
edge is detected with grad within [grad_low, grad_low], gi(gray insu) within [0, gi_high],
gw(gray wire) within [gw_low, gw_high], dw(wire variance) within [0, dw_high]
Then edge is erode for lr edge, erode up and down; for ud edge, erode left and right. 
Finally check ud edge has is wider than ed_guard and lr edge is higer than ed_guard.
method_opt
0: for remove via mask input
1: for edge detect output
*/
#define SEEM_BE_WIRE		0x11
#define SEEM_BE_INSU		0x10
#define SEEM_CONFLICT		0xfe
static void edge_detect(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	int idx = cpara.method_opt & 0xf;
	int idx1 = cpara.method_opt >> 4 & 0xf;
	int detect_opt = cpara.opt0 & 0xff;
	int ed_long = cpara.opt0 >> 8 & 0xff;
	int ed_guard = cpara.opt0 >> 16 & 0xff;
	int ed_wide = (detect_opt & EDGE_DETECT_WIDE_2) ? 2 : 1;
	int grad_low_l = (cpara.opt1 >> 24 & 0xff) * ed_long * ed_wide;
	int grad_high_l = (cpara.opt1 >> 16 & 0xff) * ed_long * ed_wide;
	int gi_high_l = (cpara.opt1 >> 8 & 0xff) * ed_long * ed_wide;
	int gw_low_l = (cpara.opt1 & 0xff) * ed_long * ed_wide;
	int gw_high_l = (cpara.opt2 >> 24 & 0xff) * ed_long * ed_wide;
	int dw_high_l = (cpara.opt2 >> 16 & 0xff) * ed_wide;
	dw_high_l = dw_high_l * dw_high_l * ed_long * ed_long * ed_wide * ed_wide;
	int grad_low_r = (cpara.opt2 >> 8 & 0xff) * ed_long * ed_wide;
	int grad_high_r = (cpara.opt2 & 0xff) * ed_long * ed_wide;
	int gi_high_r = (cpara.opt3 >> 24 & 0xff) * ed_long * ed_wide;
	int gw_low_r = (cpara.opt3 >> 16 & 0xff) * ed_long * ed_wide;
	int gw_high_r = (cpara.opt3 >> 8 & 0xff) * ed_long * ed_wide;
	int dw_high_r = (cpara.opt3 & 0xff);
	dw_high_r = dw_high_r * dw_high_r * ed_long * ed_long * ed_wide * ed_wide;
	int grad_low_u = (cpara.opt4 >> 24 & 0xff) * ed_long * ed_wide;
	int grad_high_u = (cpara.opt4 >> 16 & 0xff) * ed_long * ed_wide;
	int gi_high_u = (cpara.opt4 >> 8 & 0xff) * ed_long * ed_wide;
	int gw_low_u = (cpara.opt4 & 0xff) * ed_long * ed_wide;
	int gw_high_u = (cpara.opt5 >> 24 & 0xff) * ed_long * ed_wide;
	int dw_high_u = (cpara.opt5 >> 16 & 0xff);
	dw_high_u = dw_high_u * dw_high_u * ed_long * ed_long * ed_wide * ed_wide;
	int grad_low_d = (cpara.opt5 >> 8 & 0xff) * ed_long * ed_wide;
	int grad_high_d = (cpara.opt5 & 0xff) * ed_long * ed_wide;
	int gi_high_d = (cpara.opt6 >> 24 & 0xff) * ed_long * ed_wide;
	int gw_low_d = (cpara.opt6 >> 16 & 0xff) * ed_long * ed_wide;
	int gw_high_d = (cpara.opt6 >> 8 & 0xff) * ed_long * ed_wide;
	int dw_high_d = (cpara.opt6 & 0xff);
	dw_high_d = dw_high_d * dw_high_d * ed_long * ed_long * ed_wide * ed_wide;

	Mat via_mask;
	if (detect_opt & EDGE_DETECT_HAS_VIA_MASK) {
		if (d.l[layer].v[idx].type != TYPE_REMOVE_VIA_MASK) {
			qCritical("edge_detect remove_mask[%d]=%d, error", idx, d.l[layer].v[idx].type);
			return;
		}
		via_mask = d.l[layer].v[idx].d;
		CV_Assert(via_mask.size() == img.size());
	}
	d.l[layer].validate_ig();
	d.l[layer].v[idx1].type = TYPE_EDGE_MASK;
	d.l[layer].v[idx1].d.create(img.rows, img.cols, CV_8UC1);
	Mat & edge = d.l[layer].v[idx1].d;
	edge = Scalar::all(0xff);
	Mat grad(img.rows, img.cols, CV_16SC1);
	grad = Scalar::all(0);
	if (grad_low_l > 0 && grad_low_r > 0) { //following compute left right grad which satisfied grad, gray condition
		qInfo("edge_detect,ed_wide=%d,ed_long=%d,grad_low_l=%d,grad_high_l=%d,gi_high_l=%d,gw_low_l=%d,gw_high_l=%d,dw_high_l=%d",
			ed_wide, ed_long, grad_low_l, grad_high_l, gi_high_l, gw_low_l, gw_high_l, dw_high_l);
		qInfo("edge_detect,ed_guard=%d,grad_low_r=%d,grad_high_r=%d,gi_high_r=%d,gw_low_r=%d,gw_high_r=%d,dw_high_r=%d", ed_guard,
			grad_low_r, grad_high_r, gi_high_r, gw_low_r, gw_high_r, dw_high_r);
		if (grad_high_l <= grad_low_l || grad_high_r <= grad_low_r) {
			qCritical("Edge detect leftright grad Error");
			return;
		}
		if (gi_high_l > gw_high_l || gw_low_l > gw_high_l || gi_high_r > gw_high_r || gw_low_r > gw_high_r) {
			qCritical("Edge detect leftright gray Error");
			return;
		}
		for (int y = ed_long / 2; y < img.rows - ed_long / 2; y++) {
			int * p_ig0 = d.l[layer].ig.ptr<int>(y - ed_long / 2); //ed_long * ed_wide rect
			int * p_ig1 = d.l[layer].ig.ptr<int>(y + ed_long - ed_long / 2);
			int * p_iig0 = d.l[layer].iig.ptr<int>(y - ed_long / 2);
			int * p_iig1 = d.l[layer].iig.ptr<int>(y + ed_long - ed_long / 2);
			short * p_grad = grad.ptr<short>(y);
			unsigned char * p_edge = edge.ptr<unsigned char>(y);
			unsigned char * p_viamask = via_mask.empty() ? NULL : via_mask.ptr<unsigned char>(y);
			for (int x = ed_long / 2; x < img.cols - ed_long / 2; x++) 
			if (p_viamask == NULL || p_viamask[x] == 0) {
				int sum0 = p_ig1[x] - p_ig1[x - ed_wide] - p_ig0[x] + p_ig0[x - ed_wide];
				int sum1 = p_ig1[x + 1 + ed_wide] - p_ig1[x + 1] - p_ig0[x + 1 + ed_wide] + p_ig0[x + 1];
				int g = sum1 - sum0;
				if (g > grad_low_l && g < grad_high_l && sum0 < gi_high_l && sum1 > gw_low_l && sum1 < gw_high_l) { //grad in range, gray in range
					int dw = ed_long * ed_wide * (p_iig1[x + 1 + ed_wide] - p_iig1[x + 1] - p_iig0[x + 1 + ed_wide] + p_iig0[x + 1]) - sum1 * sum1;
					if (dw < dw_high_l) { //variance in range
						p_grad[x] = g;
						p_edge[x] = DIR_RIGHT;
					}
				}
				else
				if (-g > grad_low_r && -g < grad_high_r && sum1 < gi_high_r && sum0 > gw_low_r && sum0 < gw_high_r) {
					int dw = ed_long * ed_wide * (p_iig1[x] - p_iig1[x - ed_wide] - p_iig0[x] + p_iig0[x - ed_wide]) - sum0 * sum0;
					if (dw < dw_high_r) {
						p_grad[x] = -g;
						p_edge[x] = DIR_LEFT;
					}
				}
			}
			else
				p_edge[x] = SEEM_CONFLICT;
		}
	}
	if (grad_low_u > 0 && grad_low_d > 0) {//following compute up down grad which satisfied grad, gray condition
		qInfo("edge_detect,ed_wide=%d,ed_long=%d,grad_low_u=%d,grad_high_u=%d,gi_high_u=%d,gw_low_u=%d,gw_high_u=%d,dw_high_u=%d", ed_wide, ed_long,
			grad_low_u, grad_high_u, gi_high_u, gw_low_u, gw_high_u, dw_high_u);
		qInfo("edge_detect,ed_guard=%d,grad_low_d=%d,grad_high_d=%d,gi_high_d=%d,gw_low_d=%d,gw_high_d=%d,dw_high_d=%d", ed_guard,
			grad_low_d, grad_high_d, gi_high_d, gw_low_d, gw_high_d, dw_high_d);
		if (grad_high_u <= grad_low_u || grad_high_d <= grad_low_d) {
			qCritical("Edge detect updown grad Error");
			return;
		}
		if (gi_high_u > gw_high_u || gw_low_u > gw_high_u || gi_high_d > gw_high_d || gw_low_d > gw_high_d) {
			qCritical("Edge detect updown gray Error");
			return;
		}
		for (int y = ed_long / 2; y < img.rows - ed_long / 2; y++) {
			int * p_ig0 = d.l[layer].ig.ptr<int>(y - ed_wide); //ed_wide*ed_long rect
			int * p_ig1 = d.l[layer].ig.ptr<int>(y);
			int * p_ig2 = d.l[layer].ig.ptr<int>(y + 1);
			int * p_ig3 = d.l[layer].ig.ptr<int>(y + 1 + ed_wide);
			int * p_iig0 = d.l[layer].iig.ptr<int>(y - ed_wide);
			int * p_iig1 = d.l[layer].iig.ptr<int>(y);
			int * p_iig2 = d.l[layer].iig.ptr<int>(y + 1);
			int * p_iig3 = d.l[layer].iig.ptr<int>(y + 1 + ed_wide);
			short * p_grad = grad.ptr<short>(y);
			unsigned char * p_edge = edge.ptr<unsigned char>(y);
			unsigned char * p_viamask = via_mask.empty() ? NULL : via_mask.ptr<unsigned char>(y);
			for (int x = ed_long / 2; x < img.cols - ed_long / 2; x++) 
			if (p_viamask == NULL || p_viamask[x] == 0) {
				int sum0 = p_ig1[x + ed_long - ed_long / 2] - p_ig1[x - ed_long / 2] - p_ig0[x + ed_long - ed_long / 2] + p_ig0[x - ed_long / 2];
				int sum1 = p_ig3[x + ed_long - ed_long / 2] - p_ig3[x - ed_long / 2] - p_ig2[x + ed_long - ed_long / 2] + p_ig2[x - ed_long / 2];
				int g = sum1 - sum0;
				if (g > grad_low_u && g < grad_high_u && sum0 < gi_high_u && sum1 > gw_low_u && sum1 < gw_high_u) { //grad in range, gray in range
					int dw = ed_long * ed_wide * (p_iig3[x + ed_long - ed_long / 2] - p_iig3[x - ed_long / 2] - 
						p_iig2[x + ed_long - ed_long / 2] + p_iig2[x - ed_long / 2]) - sum1 * sum1;
					if (dw < dw_high_u && g > p_grad[x]) {//variance in range
						p_grad[x] = g;
						p_edge[x] = DIR_DOWN;
					}
				}
				else
				if (-g > grad_low_d && -g < grad_high_d && sum1 < gi_high_d && sum0 > gw_low_d && sum0 < gw_high_d) {
					int dw = ed_long * ed_wide * (p_iig1[x + ed_long - ed_long / 2] - p_iig1[x - ed_long / 2] -
						p_iig0[x + ed_long - ed_long / 2] + p_iig0[x - ed_long / 2]) - sum0 * sum0;
					if (dw < dw_high_d && -g > p_grad[x]) {
						p_grad[x] = -g;
						p_edge[x] = DIR_UP;
					}
				}
			}
			else
				p_edge[x] = SEEM_CONFLICT;
		}
	}

	if (detect_opt & EDGE_DETECT_ERODE) {
#define ERODE(v, v1) if (v1==0xff) v1=8+v; else if (v1 & 8 && v1 != 8 + v) v1=SEEM_CONFLICT
		for (int y = 1; y < img.rows - 1; y++) {
			unsigned char * p_edge = edge.ptr<unsigned char>(y);
			unsigned char * p_edge_u = edge.ptr<unsigned char>(y - 1);
			unsigned char * p_edge_d = edge.ptr<unsigned char>(y + 1);
			for (int x = 1; x < img.cols - 1; x++) {
				if (p_edge[x] != 0xff) {
					switch (p_edge[x]) {
					case DIR_UP:
					case DIR_DOWN:
						ERODE(p_edge[x], p_edge[x - 1]);
						ERODE(p_edge[x], p_edge[x + 1]);
						break;
					case DIR_LEFT:
					case DIR_RIGHT:
						ERODE(p_edge[x], p_edge_u[x]);
						ERODE(p_edge[x], p_edge_d[x]);
						break;
					}
				}
				if (p_edge_u[x] & 8 && p_edge_u[x] < 16)
					p_edge_u[x] -= 8;
			}
			p_edge[0] = 0xff;
			p_edge[img.cols - 1] = 0xff;
		}
		unsigned char * p_edge_u = edge.ptr<unsigned char>(img.rows - 2);
		unsigned char * p_edge = edge.ptr<unsigned char>(img.rows-1);
		unsigned char * p_edge_d = edge.ptr<unsigned char>(0);
		for (int x = 1; x < img.cols - 1; x++) {
			p_edge[x] = 0xff;
			if (p_edge_u[x] & 8 && p_edge_u[x] < 16)
				p_edge_u[x] -= 8;
			p_edge_d[x] = 0xff;
		}
#undef ERODE
	}

	if (ed_guard > 1) {
#define TRANS(e0, e1, l0, l1) { if ((e1)==(e0)) l1=max(l1, (unsigned short) ((l0) + 1));}
#define CHECK(x, y, l1, l0, q) { \
		unsigned short * pl1 = l1; \
		if (*pl1 > 0 && *pl1 <= (l0)) {	\
			unsigned short old_l1 = *pl1; \
			*pl1 = (l0) + 1; \
			if (old_l1 < ed_guard - 1) \
				q.push_back(MAKE_PROB(*pl1, x, y)); \
		} }
		Mat elen(img.rows, img.cols, CV_16SC2);
		elen = Scalar::all(0);
		vector <unsigned long long> ext_ul, ext_dr;
		for (int y = 1; y < img.rows - 1; y++) {
			const unsigned char * p_edge = edge.ptr<unsigned char>(y);
			const unsigned char * p_edge_u = edge.ptr<unsigned char>(y - 1);
			const unsigned char * p_edge_d = edge.ptr<unsigned char>(y + 1);
			unsigned short * p_elen = elen.ptr<unsigned short>(y);
			unsigned short * p_elen_u = elen.ptr<unsigned short>(y - 1);
			unsigned short * p_elen_d = elen.ptr<unsigned short>(y + 1);
			for (int i = 1; i < img.cols - 1; i++) { // i is x
				unsigned short prev_elen_u = p_elen_u[i * 2 + 2];
				if (p_edge[i] == DIR_UP || p_edge[i] == DIR_DOWN) {
					if (p_elen[i * 2] == 0)
						p_elen[i * 2] = 1;
					TRANS(p_edge[i], p_edge_u[i + 1], p_elen[i * 2], p_elen_u[i * 2 + 2]);
					TRANS(p_edge[i], p_edge[i + 1], p_elen[i * 2], p_elen[i * 2 + 2]);									
					TRANS(p_edge[i], p_edge_d[i + 1], p_elen[i * 2], p_elen_d[i * 2 + 2]);
				}
				if (p_edge_u[i] == DIR_UP || p_edge_u[i] == DIR_DOWN) {
					TRANS(p_edge_u[i], p_edge_u[i + 1], p_elen_u[i * 2], p_elen_u[i * 2 + 2]);
					TRANS(p_edge_u[i], p_edge[i + 1], p_elen_u[i * 2], p_elen[i * 2 + 2]);
					TRANS(p_edge_u[i], p_edge_d[i + 1], p_elen_u[i * 2], p_elen_d[i * 2 + 2]);
				}
				if (p_edge_d[i] == DIR_UP || p_edge_d[i] == DIR_DOWN) {					
					TRANS(p_edge_d[i], p_edge_u[i + 1], p_elen_d[i * 2], p_elen_u[i * 2 + 2]);
					TRANS(p_edge_d[i], p_edge[i + 1], p_elen_d[i * 2], p_elen[i * 2 + 2]);
					TRANS(p_edge_d[i], p_edge_d[i + 1], p_elen_d[i * 2], p_elen_d[i * 2 + 2]);
				}
				if (p_elen_u[i * 2 + 2] > prev_elen_u && prev_elen_u < ed_guard - 1)
					ext_dr.push_back(MAKE_PROB(p_elen_u[i * 2 + 2], i + 1, y - 1));
			}

			for (int i = img.cols - 2; i > 0; i--) { //i is x
				unsigned short prev_elen_u = p_elen_u[i * 2 - 1];
				if (p_edge[i] == DIR_UP || p_edge[i] == DIR_DOWN) {
					if (p_elen[i * 2 + 1] == 0)
						p_elen[i * 2 + 1] = 1;
					TRANS(p_edge[i], p_edge_u[i - 1], p_elen[i * 2 + 1], p_elen_u[i * 2 - 1]);
					TRANS(p_edge[i], p_edge[i - 1], p_elen[i * 2 + 1], p_elen[i * 2 - 1]);					
					TRANS(p_edge[i], p_edge_d[i - 1], p_elen[i * 2 + 1], p_elen_d[i * 2 - 1]);
				}
				if (p_edge_u[i] == DIR_UP || p_edge_u[i] == DIR_DOWN) {
					TRANS(p_edge_u[i], p_edge_u[i - 1], p_elen_u[i * 2 + 1], p_elen_u[i * 2 - 1]);
					TRANS(p_edge_u[i], p_edge[i - 1], p_elen_u[i * 2 + 1], p_elen[i * 2 - 1]);
					TRANS(p_edge_u[i], p_edge_d[i - 1], p_elen_u[i * 2 + 1], p_elen_d[i * 2 - 1]);
				}
				if (p_edge_d[i] == DIR_UP || p_edge_d[i] == DIR_DOWN) {
					TRANS(p_edge_d[i], p_edge_u[i - 1], p_elen_d[i * 2 + 1], p_elen_u[i * 2 - 1]);
					TRANS(p_edge_d[i], p_edge[i - 1], p_elen_d[i * 2 + 1], p_elen[i * 2 - 1]);
					TRANS(p_edge_d[i], p_edge_d[i - 1], p_elen_d[i * 2 + 1], p_elen_d[i * 2 - 1]);
				}
				if (p_elen_u[i * 2 - 1] > prev_elen_u && prev_elen_u < ed_guard - 1)
					ext_ul.push_back(MAKE_PROB(p_elen_u[i * 2 - 1], i - 1, y - 1));
			}
		}
		while (!ext_dr.empty()) {
			unsigned long long temp = ext_dr.back();
			ext_dr.pop_back();
			int y = PROB_Y(temp);
			int x = PROB_X(temp);
			int l = PROB_S(temp);
			unsigned short cl = elen.at<unsigned short>(y, 2 * x);
			if (cl > l)
				continue;
			CV_Assert(cl == l);
			CHECK(x + 1, y, elen.ptr<unsigned short>(y, x + 1), l, ext_dr);
			CHECK(x + 1, y - 1, elen.ptr<unsigned short>(y - 1, x + 1), l, ext_dr);
			CHECK(x + 1, y + 1, elen.ptr<unsigned short>(y + 1, x + 1), l, ext_dr);
			if (y > 2)
				CHECK(x + 1, y - 2, elen.ptr<unsigned short>(y - 2, x + 1), l, ext_dr);
			if (y< img.rows - 2)
				CHECK(x + 1, y + 2, elen.ptr<unsigned short>(y + 2, x + 1), l, ext_dr);
		}
		while (!ext_ul.empty()) {
			unsigned long long temp = ext_ul.back();
			ext_ul.pop_back();
			int y = PROB_Y(temp);
			int x = PROB_X(temp);
			int l = PROB_S(temp);
			unsigned short cl = elen.at<unsigned short>(y, 2 * x + 1);
			if (cl > l)
				continue;
			CV_Assert(cl == l);
			CHECK(x - 1, y, elen.ptr<unsigned short>(y, x - 1) + 1, l, ext_ul);
			CHECK(x - 1, y - 1, elen.ptr<unsigned short>(y - 1, x - 1) + 1, l, ext_ul);
			CHECK(x - 1, y + 1, elen.ptr<unsigned short>(y + 1, x - 1) + 1, l, ext_ul);
			if (y > 2)
				CHECK(x - 1, y - 2, elen.ptr<unsigned short>(y - 2, x - 1) + 1, l, ext_ul);
			if (y< img.rows - 2)
				CHECK(x - 1, y + 2, elen.ptr<unsigned short>(y + 2, x - 1) + 1, l, ext_ul);
		}
		for (int y = 1; y < img.rows - 1; y++) {
			unsigned char * p_edge = edge.ptr<unsigned char>(y);
			unsigned short * p_elen = elen.ptr<unsigned short>(y);
			for (int x = 1; x < img.cols - 1; x++)
			if (p_edge[x] == DIR_UP || p_edge[x] == DIR_DOWN) {
				if (p_elen[x * 2] + p_elen[x * 2 + 1] - 1 <= ed_guard)
					p_edge[x] = SEEM_CONFLICT;
			}
		}
		elen = Scalar::all(0);
		vector<unsigned char> edge_buf(img.rows * 3, 0xff);
		vector<unsigned short> elen_buf(img.rows * 6, 0);
		unsigned char * p_edge = &edge_buf[img.rows];
		unsigned char * p_edge_u = &edge_buf[0];
		unsigned char * p_edge_d = &edge_buf[img.rows * 2];
		unsigned short * p_elen = &elen_buf[img.rows * 2];
		unsigned short * p_elen_u = &elen_buf[0];
		unsigned short * p_elen_d = &elen_buf[img.rows * 4];
		for (int x = 0; x <= img.cols; x++) {
			for (int y = 0; y < img.rows; y++) 
			if (x > 1) {
				elen.at<unsigned short>(y, 2 * x - 4) = p_elen_u[2 * y];
				elen.at<unsigned short>(y, 2 * x - 3) = p_elen_u[2 * y + 1];
			}
			swap(p_edge_u, p_edge);
			swap(p_edge, p_edge_d);
			swap(p_elen_u, p_elen);
			swap(p_elen, p_elen_d);
			if (x >= img.cols - 1)
				continue;
			for (int y = 0; y < img.rows; y++) {
				p_edge_d[y] = (x < img.cols - 1) ? edge.at<unsigned char>(y, x + 1) : 0xff;
				p_elen_d[y * 2] = 0;
				p_elen_d[y * 2 + 1] = 0;
			}
			if (x < 1)
				continue;
			for (int i = 1; i < img.rows - 1; i++) { // i is y
				unsigned short prev_elen_u = p_elen_u[i * 2 + 2];
				if (p_edge[i] == DIR_LEFT || p_edge[i] == DIR_RIGHT) {
					if (p_elen[i * 2] == 0)
						p_elen[i * 2] = 1;
					TRANS(p_edge[i], p_edge_u[i + 1], p_elen[i * 2], p_elen_u[i * 2 + 2]);
					TRANS(p_edge[i], p_edge[i + 1], p_elen[i * 2], p_elen[i * 2 + 2]);
					TRANS(p_edge[i], p_edge_d[i + 1], p_elen[i * 2], p_elen_d[i * 2 + 2]);
				}
				if (p_edge_u[i] == DIR_LEFT || p_edge_u[i] == DIR_RIGHT) {
					TRANS(p_edge_u[i], p_edge_u[i + 1], p_elen_u[i * 2], p_elen_u[i * 2 + 2]);
					TRANS(p_edge_u[i], p_edge[i + 1], p_elen_u[i * 2], p_elen[i * 2 + 2]);
					TRANS(p_edge_u[i], p_edge_d[i + 1], p_elen_u[i * 2], p_elen_d[i * 2 + 2]);
				}
				if (p_edge_d[i] == DIR_LEFT || p_edge_d[i] == DIR_RIGHT) {
					TRANS(p_edge_d[i], p_edge_u[i + 1], p_elen_d[i * 2], p_elen_u[i * 2 + 2]);
					TRANS(p_edge_d[i], p_edge[i + 1], p_elen_d[i * 2], p_elen[i * 2 + 2]);
					TRANS(p_edge_d[i], p_edge_d[i + 1], p_elen_d[i * 2], p_elen_d[i * 2 + 2]);
				}
				if (p_elen_u[i * 2 + 2] > prev_elen_u && prev_elen_u < ed_guard - 1)
					ext_dr.push_back(MAKE_PROB(p_elen_u[i * 2 + 2], x - 1, i + 1));
			}

			for (int i = img.rows - 2; i > 0; i--) { //i is y
				unsigned short prev_elen_u = p_elen_u[i * 2 - 1];
				if (p_edge[i] == DIR_LEFT || p_edge[i] == DIR_RIGHT) {
					if (p_elen[i * 2 + 1] == 0)
						p_elen[i * 2 + 1] = 1;
					TRANS(p_edge[i], p_edge_u[i - 1], p_elen[i * 2 + 1], p_elen_u[i * 2 - 1]);
					TRANS(p_edge[i], p_edge[i - 1], p_elen[i * 2 + 1], p_elen[i * 2 - 1]);
					TRANS(p_edge[i], p_edge_d[i - 1], p_elen[i * 2 + 1], p_elen_d[i * 2 - 1]);
				}
				if (p_edge_u[i] == DIR_LEFT || p_edge_u[i] == DIR_RIGHT) {
					TRANS(p_edge_u[i], p_edge_u[i - 1], p_elen_u[i * 2 + 1], p_elen_u[i * 2 - 1]);
					TRANS(p_edge_u[i], p_edge[i - 1], p_elen_u[i * 2 + 1], p_elen[i * 2 - 1]);
					TRANS(p_edge_u[i], p_edge_d[i - 1], p_elen_u[i * 2 + 1], p_elen_d[i * 2 - 1]);
				}
				if (p_edge_d[i] == DIR_LEFT || p_edge_d[i] == DIR_RIGHT) {
					TRANS(p_edge_d[i], p_edge_u[i - 1], p_elen_d[i * 2 + 1], p_elen_u[i * 2 - 1]);
					TRANS(p_edge_d[i], p_edge[i - 1], p_elen_d[i * 2 + 1], p_elen[i * 2 - 1]);
					TRANS(p_edge_d[i], p_edge_d[i - 1], p_elen_d[i * 2 + 1], p_elen_d[i * 2 - 1]);
				}
				if (p_elen_u[i * 2 - 1] > prev_elen_u && prev_elen_u < ed_guard - 1)
					ext_ul.push_back(MAKE_PROB(p_elen_u[i * 2 - 1], x - 1, i - 1));
			}
		}
		while (!ext_dr.empty()) {
			unsigned long long temp = ext_dr.back();
			ext_dr.pop_back();
			int y = PROB_Y(temp);
			int x = PROB_X(temp);
			int l = PROB_S(temp);
			unsigned short cl = elen.at<unsigned short>(y, 2 * x);
			if (cl > l)
				continue;
			CV_Assert(cl == l);
			CHECK(x, y + 1, elen.ptr<unsigned short>(y + 1, x), l, ext_dr);
			CHECK(x - 1, y + 1, elen.ptr<unsigned short>(y + 1, x - 1), l, ext_dr);
			CHECK(x + 1, y + 1, elen.ptr<unsigned short>(y + 1, x + 1), l, ext_dr);
			if (x > 2)
				CHECK(x - 2, y + 1, elen.ptr<unsigned short>(y + 1, x - 2), l, ext_dr);
			if (x< img.cols - 2)
				CHECK(x + 2, y + 1, elen.ptr<unsigned short>(y + 1, x + 2), l, ext_dr);
		}
		while (!ext_ul.empty()) {
			unsigned long long temp = ext_ul.back();
			ext_ul.pop_back();
			int y = PROB_Y(temp);
			int x = PROB_X(temp);
			int l = PROB_S(temp);
			unsigned short cl = elen.at<unsigned short>(y, 2 * x + 1);
			if (cl > l)
				continue;
			CV_Assert(cl == l);
			CHECK(x, y - 1, elen.ptr<unsigned short>(y - 1, x) + 1, l, ext_ul);
			CHECK(x - 1, y - 1, elen.ptr<unsigned short>(y - 1, x - 1) + 1, l, ext_ul);
			CHECK(x + 1, y - 1, elen.ptr<unsigned short>(y - 1, x + 1) + 1, l, ext_ul);
			if (x > 2)
				CHECK(x - 2, y - 1, elen.ptr<unsigned short>(y - 1, x - 2) + 1, l, ext_ul);
			if (x< img.cols - 2)
				CHECK(x + 2, y - 1, elen.ptr<unsigned short>(y - 1, x + 2) + 1, l, ext_ul);
		}
		for (int y = 1; y < img.rows - 1; y++) {
			unsigned char * p_edge = edge.ptr<unsigned char>(y);
			unsigned short * p_elen = elen.ptr<unsigned short>(y);
			for (int x = 1; x < img.cols - 1; x++) 
			if(p_edge[x] == DIR_LEFT || p_edge[x] == DIR_RIGHT)  {
				if (p_elen[x * 2] + p_elen[x * 2 + 1] - 1 <= ed_guard)
					p_edge[x] = SEEM_CONFLICT;
			}
		}
#undef TRANS
#undef CHECK
	}
	if (cpara.method & OPT_DEBUG_EN) {
		Mat debug_draw;
		cvtColor(img, debug_draw, CV_GRAY2BGR);
		CV_Assert(debug_draw.type() == CV_8UC3 && debug_draw.rows == edge.rows && debug_draw.cols == edge.cols);
		for (int y = 0; y < debug_draw.rows; y++) {
			unsigned char * p_draw = debug_draw.ptr<unsigned char>(y);
			unsigned char * p_edge = edge.ptr<unsigned char>(y);
			for (int x = 0; x < debug_draw.cols; x++) {
				switch (p_edge[x]) {
				case DIR_UP:
					p_draw[3 * x + 2] = 255; //draw skin red
					break;
				case DIR_DOWN:
					p_draw[3 * x + 1] = 255; //draw skin green
					break;
				case DIR_RIGHT:
					p_draw[3 * x] = 255; //draw skin blue
					break;
				case DIR_LEFT:
					p_draw[3 * x + 2] = 255; //draw skin yellow
					p_draw[3 * x + 1] = 255;
					break;
				case SEEM_CONFLICT:
					p_draw[3 * x + 2] = 255;
					p_draw[3 * x] = 255;
					break;
				}
			}
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_edgedet.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw.clone();
		}
	}
}

/*     31..24 23..16   15..8   7..0
opt0:		extend_guard extend_len enhance_opt
opt1:  ilr_max ilr_min wlr_min wlr_max
opt2:  iud_max iud_min wud_min wud_max
opt3:				   th_para1 th_para0
opt4:		  enhance_y enhance_x1 enhance_x0
extend_guard is to remove conflict edge
extend_len is to remove extend edge
enhance_opt is HAS_VIA_MASK
enhance include increase wire gray and reduce insu gray
enhance insu which satisfy ilr(insu lr width) within [ilr_min, ilr_max] or iud(insu ud width) within [iud_min, iud_max]
enhance wire which satisfy wlr(wire lr width) within [wlr_min, wlr_max] or wud(wire ud width) within [wud_min, wud_max]
method_opt
0: for remove via mask input
1: for edge detect input
2: for gray level turn_points inout
*/
static void image_enhance(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	int idx = cpara.method_opt & 0xf;
	int idx1 = cpara.method_opt >> 4 & 0xf;
	int idx2 = cpara.method_opt >> 8 & 0xf;
	int enhance_opt = cpara.opt0 & 0xff;
	int extend_len = cpara.opt0 >> 8 & 0xff;
	int extend_guard = cpara.opt0 >> 16 & 0xff;
	int ilr_max = cpara.opt1 >> 24 & 0xff;
	int ilr_min = cpara.opt1 >> 16 & 0xff;
	int wlr_min = cpara.opt1 >> 8 & 0xff;
	int wlr_max = cpara.opt1 & 0xff;
	int iud_max = cpara.opt2 >> 24 & 0xff;
	int iud_min = cpara.opt2 >> 16 & 0xff;
	int wud_min = cpara.opt2 >> 8 & 0xff;
	int wud_max = cpara.opt2 & 0xff;
	float th_para0 = (cpara.opt3 & 0xff) / 100.0;
	float th_para1 = (cpara.opt3 >> 8 & 0xff) / 10000.0;
	int enhance_x0 = cpara.opt4 & 0xff;
	int enhance_x1 = cpara.opt4 >> 8 & 0xff;
	int enhance_y = cpara.opt4 >> 16 & 0xff;
	Mat & edge = d.l[layer].v[idx1].d;
	
	Mat via_mask;
	if (enhance_opt & IMG_ENHANCE_HAS_VIA_MASK) {
		if (d.l[layer].v[idx].type != TYPE_REMOVE_VIA_MASK) {
			qCritical("image_enhance remove_mask[%d]=%d, error", idx, d.l[layer].v[idx].type);
			return;
		}
		via_mask = d.l[layer].v[idx].d;
		CV_Assert(via_mask.size() == img.size());
	}

	if (d.l[layer].v[idx1].type != TYPE_EDGE_MASK) {
		qCritical("image_enhance edge[%d]=%d, error", idx1, d.l[layer].v[idx1].type);
		return;
	}
	if (d.l[layer].v[idx2].type != TYPE_GRAY_LEVEL) {
		qCritical("image_enhance gray_lvl[%d]=%d, error", idx2, d.l[layer].v[idx2].type);
		return;
	}
	CV_Assert(edge.size() == img.size());
	qInfo("image_enhance extend_guard=%d, extend_len=%d", extend_guard, extend_len);

	//2 connect same edge

	//3 find insu and wire
	qInfo("image_enhance, ilr in [%d,%d], iud in [%d,%d]. wlr in [%d,%d], wud in [%d,%d]", ilr_min, ilr_max,
		iud_min, iud_max, wlr_min, wlr_max, wud_min, wud_max);
	int clr_max = max(wlr_max, ilr_max);
	int cud_max = max(wud_max, iud_max);
	for (int iter = 0; iter < 2; iter++) {
		if (wlr_max > 0 && ilr_max > 0)
		for (int y = 1; y < edge.rows - 1; y++) {
			unsigned char * p_edge = edge.ptr<unsigned char>(y);
			unsigned prev_edge = 0xff;
			bool conflict = false; //in conflict state, prev_edge=INSU or WIRE
			int prev_idx = 0, conflict_idx = 0;
			for (int x = 1; x < edge.cols - 1; x++)
			if (p_edge[x] != 0xff) {
				unsigned cur_edge = p_edge[x];
				switch (prev_edge) {
				case DIR_RIGHT:
				case SEEM_BE_WIRE:
					if (cur_edge == DIR_LEFT || cur_edge == SEEM_BE_WIRE) {
						if (x - prev_idx > 1 && (cur_edge == SEEM_BE_WIRE || prev_edge == SEEM_BE_WIRE || x - prev_idx >= wlr_min)
							&& x - prev_idx <= wlr_max) {
							for (int i = prev_idx + 1; i < x; i++) //mark SEEM_BE_WIRE
								p_edge[i] = SEEM_BE_WIRE;
						}
					}
					if (cur_edge == SEEM_BE_INSU) { //conflict
						if (prev_edge == DIR_RIGHT) {
							conflict = true;
							conflict_idx = prev_idx;
						}
						else { //prev_edge==SEEM_BE_WIRE
							if (conflict) {
								if (x - conflict_idx <= clr_max)
								for (int i = conflict_idx + 1; i < x; i++) { //mark CONFLICT
									CV_Assert(p_edge[i] == 0xff || p_edge[i] == SEEM_BE_INSU || p_edge[i] == SEEM_BE_WIRE);
									p_edge[i] = SEEM_CONFLICT;
								}
								conflict_idx = x;
							}
							else {
								conflict = true;
								conflict_idx = prev_idx;
							}
						}
					}
					if (cur_edge == DIR_RIGHT) {
						if (prev_edge == SEEM_BE_WIRE) { //conflict
							if (conflict) {
								if (x - conflict_idx <= clr_max)
								for (int i = conflict_idx + 1; i < x; i++) { //mark CONFLICT
									CV_Assert(p_edge[i] == 0xff || p_edge[i] == SEEM_BE_INSU || p_edge[i] == SEEM_BE_WIRE);
									p_edge[i] = SEEM_CONFLICT;
								}
							}
						}
					}
					break;

				case DIR_LEFT:
				case SEEM_BE_INSU:
					if (cur_edge == DIR_RIGHT || cur_edge == SEEM_BE_INSU) {
						if (x - prev_idx > 1 && (cur_edge == SEEM_BE_INSU || prev_edge == SEEM_BE_INSU || x - prev_idx >= ilr_min)
							&& x - prev_idx <= ilr_max) {
							for (int i = prev_idx + 1; i < x; i++) //mark SEEM_BE_INSU
								p_edge[i] = SEEM_BE_INSU;
						}
					}
					if (cur_edge == SEEM_BE_WIRE) { //conflict
						if (prev_edge == DIR_LEFT) {
							conflict = true;
							conflict_idx = prev_idx;
						}
						else { //prev_edge==SEEM_BE_INSU
							if (conflict) {
								if (x - conflict_idx <= clr_max)
								for (int i = conflict_idx + 1; i < x; i++) { //mark CONFLICT
									CV_Assert(p_edge[i] == 0xff || p_edge[i] == SEEM_BE_INSU || p_edge[i] == SEEM_BE_WIRE);
									p_edge[i] = SEEM_CONFLICT;
								}
								conflict_idx = x;
							}
							else {
								conflict = true;
								conflict_idx = prev_idx;
							}
						}
					}
					if (cur_edge == DIR_LEFT) {
						if (prev_edge == SEEM_BE_INSU) { //conflict
							if (conflict) {
								if (x - conflict_idx <= clr_max)
								for (int i = conflict_idx + 1; i < x; i++) {//mark CONFLICT
									CV_Assert(p_edge[i] == 0xff || p_edge[i] == SEEM_BE_INSU || p_edge[i] == SEEM_BE_WIRE);
									p_edge[i] = SEEM_CONFLICT;
								}
							}
						}
					}
					break;
				}
				if (cur_edge == DIR_LEFT || cur_edge == DIR_RIGHT || cur_edge == SEEM_BE_WIRE || cur_edge == SEEM_BE_INSU)
					prev_edge = cur_edge;
				else
					prev_edge = 0xff;
				if (cur_edge != SEEM_BE_WIRE && cur_edge != SEEM_BE_INSU)
					conflict = false;
				prev_idx = x;
			}
		}
		if (wud_max > 0 && iud_max > 0)
		for (int x = 1; x < edge.cols - 1; x++) {
			unsigned prev_edge = 0xff;
			bool conflict = false; //in conflict state, prev_edge=INSU or WIRE
			int prev_idx = 0, conflict_idx = 0;
			for (int y = 1; y < edge.rows - 1; y++)
			if (edge.at<unsigned char>(y, x) != 0xff) {
				unsigned cur_edge = edge.at<unsigned char>(y, x);
				switch (prev_edge) {
				case DIR_DOWN:
				case SEEM_BE_WIRE:
					if (cur_edge == DIR_UP || cur_edge == SEEM_BE_WIRE) {
						if (y - prev_idx > 1 && (cur_edge == SEEM_BE_WIRE || prev_edge == SEEM_BE_WIRE || y - prev_idx >= wud_min)
							&& y - prev_idx <= wud_max) {
							for (int i = prev_idx + 1; i < y; i++) //mark SEEM_BE_WIRE
								edge.at<unsigned char>(i, x) = SEEM_BE_WIRE;
						}
					}
					if (cur_edge == SEEM_BE_INSU) { //conflict
						if (prev_edge == DIR_DOWN) {
							conflict = true;
							conflict_idx = prev_idx;
						}
						else { //prev_edge==SEEM_BE_WIRE
							if (conflict) {
								if (y - conflict_idx <= cud_max)
								for (int i = conflict_idx + 1; i < y; i++) { //mark CONFLICT
									unsigned char e = edge.at<unsigned char>(i, x);
									CV_Assert(e == 0xff || e == SEEM_BE_INSU || e == SEEM_BE_WIRE);
									edge.at<unsigned char>(i, x) = SEEM_CONFLICT;
								}
								conflict_idx = y;
							}
							else {
								conflict = true;
								conflict_idx = prev_idx;
							}
						}
					}
					if (cur_edge == DIR_DOWN) {
						if (prev_edge == SEEM_BE_WIRE) { //conflict
							if (conflict) {
								if (y - conflict_idx <= cud_max)
								for (int i = conflict_idx + 1; i < y; i++) { //mark CONFLICT
									unsigned char e = edge.at<unsigned char>(i, x);
									CV_Assert(e == 0xff || e == SEEM_BE_INSU || e == SEEM_BE_WIRE);
									edge.at<unsigned char>(i, x) = SEEM_CONFLICT;
								}
							}
						}
					}
					break;

				case DIR_UP:
				case SEEM_BE_INSU:
					if (cur_edge == DIR_DOWN || cur_edge == SEEM_BE_INSU) {
						if (y - prev_idx > 1 && (cur_edge == SEEM_BE_INSU || prev_edge == SEEM_BE_INSU || y - prev_idx >= iud_min)
							&& y - prev_idx <= iud_max) {
							for (int i = prev_idx + 1; i < y; i++) //mark SEEM_BE_INSU
								edge.at<unsigned char>(i, x) = SEEM_BE_INSU;
						}
					}
					if (cur_edge == SEEM_BE_WIRE) {
						if (prev_edge == DIR_UP) {
							conflict = true;
							conflict_idx = prev_idx;
						}
						else { //prev_edge==SEEM_BE_INSU
							if (conflict) {
								if (y - conflict_idx <= cud_max)
								for (int i = conflict_idx + 1; i < y; i++) { //mark CONFLICT
									unsigned char e = edge.at<unsigned char>(i, x);
									CV_Assert(e == 0xff || e == SEEM_BE_INSU || e == SEEM_BE_WIRE);
									edge.at<unsigned char>(i, x) = SEEM_CONFLICT;
								}
								conflict_idx = y;
							}
							else {
								conflict = true;
								conflict_idx = prev_idx;
							}
						}
					}
					if (cur_edge == DIR_UP) {
						if (prev_edge == SEEM_BE_INSU) { //conflict
							if (conflict) {
								if (y - conflict_idx <= cud_max)
								for (int i = conflict_idx + 1; i < y; i++) { //mark CONFLICT
									unsigned char e = edge.at<unsigned char>(i, x);
									CV_Assert(e == 0xff || e == SEEM_BE_INSU || e == SEEM_BE_WIRE);
									edge.at<unsigned char>(i, x) = SEEM_CONFLICT;
								}
							}
						}
					}
					break;
				}
				if (cur_edge == DIR_UP || cur_edge == DIR_DOWN || cur_edge == SEEM_BE_WIRE || cur_edge == SEEM_BE_INSU)
					prev_edge = cur_edge;
				else
					prev_edge = 0xff;
				if (cur_edge != SEEM_BE_WIRE && cur_edge != SEEM_BE_INSU)
					conflict = false;
				prev_idx = y;
			}
		}
	}
	//4 calculate pdf and determine best Threshold
	vector<int> stat0(256, 0);
	vector<int> stat1(256, 0);
	for (int y = 0; y < edge.rows; y++) {
		unsigned char * p_vmsk = via_mask.empty() ? NULL : via_mask.ptr<unsigned char>(y);
		unsigned char * p_edge = edge.ptr<unsigned char>(y);
		unsigned char * p_img = img.ptr<unsigned char>(y);
		for (int x = 0; x < edge.cols; x++) 
		if (p_vmsk == NULL || p_vmsk[x]==0) {
			if (p_edge[x] == SEEM_BE_WIRE)
				stat1[p_img[x]]++;
			if (p_edge[x] == SEEM_BE_INSU)
				stat0[p_img[x]]++;
		}
	}
	
	int gv = find_index(d.l[layer].v[idx2].d, (int)GRAY_H1);
	gv = d.l[layer].v[idx2].d.at<int>(gv, 2);
	int tot_stat0 = 0, tot_stat1 = 0;
	float m0 = 0, l0 = 0;
	for (int i = 0; i < gv; i++) {
		tot_stat0 += stat0[i];
		tot_stat1 += stat1[i];
	}
	if (tot_stat0 < 300 || tot_stat1 < 300) {
		qWarning("image_enhance stat too small, tot_stat0=%d, tot_stat1=%d", tot_stat0, tot_stat1);
		return;
	}
	vector<float> f0(256, 0); //f0 is insu PDF
	vector<float> f1(256, 0); //f1 is wire PDF
	for (int i = 0; i < gv; i++) {
		f0[i] = (float) stat0[i] / tot_stat0;
		l0 += f0[i] * i; 
		f1[i] = (float) stat1[i] / tot_stat1;
		m0 += f1[i] * i;
	}
	
	for (int i = 1; i < 256; i++) {		
		f1[i] += f1[i - 1];
		f0[255 - i] += f0[256 - i];
	}
	CV_Assert(abs(f1[255] - 1) < 0.01 && abs(f0[0] - 1) < 0.01);
	float min_cost = 2;
	int gray_th;
	th_para0 = min(th_para0, 0.80f);
	th_para0 = max(th_para0, 0.20f);
	for (int i = 0; i < gv; i++) {
		float cost = f1[i] * th_para0 + f0[i] * (1 - th_para0) + th_para1 * fabs((m0+l0)/2-i);
		if (cost < min_cost) {
			gray_th = i;
			min_cost = cost;
		}
	}
	qInfo("image_enhance, best_th=%d, l0=%f, m0=%f min_cost=%f with th_para0=%f, th_para1=%f", gray_th, l0, m0, min_cost, th_para0, th_para1);
	if (gray_th < 10) {
		qWarning("gray_th too small");
		return;
	}
	//5 enhance image
	qInfo("image_enhance, original x0=%d, x1=%d, y=%d", enhance_x0, enhance_x1, enhance_y);
	if (enhance_x1 + enhance_y > gray_th) {
		float ratio = (float)gray_th / (enhance_x1 + enhance_y);
		enhance_x1 = enhance_x1 * ratio;
		enhance_y = enhance_y * ratio;
	}
	if (enhance_x1 + enhance_y > 255 - gray_th) {
		float ratio = (float)(255 - gray_th) / (enhance_x1 + enhance_y);
		enhance_x1 = enhance_x1 * ratio;
		enhance_y = enhance_y * ratio;
	}
	qInfo("image_enhance, after adjust x1=%d, y=%d, gm=%d, gi=%d", enhance_x1, enhance_y, 
		gray_th + enhance_x1 + enhance_y, gray_th - enhance_x1 - enhance_y);
	vector<int> lut1(256), lut0(256);
	for (int i = 0; i < 256; i++) {
		lut1[i] = i;
		lut0[i] = i;
	}
	CV_Assert(enhance_x1 + enhance_y <= gray_th && enhance_x1 + enhance_y <= 255 - gray_th);
	for (int i = 0; i < 256; i++) {
		if (i >= gray_th - enhance_x1 - enhance_y && i < gray_th + enhance_x0) {
			if (i < gray_th - enhance_x1)
				lut0[i] = gray_th - enhance_x1 - enhance_y;
			else
				lut0[i] = ((enhance_y + enhance_x0 + enhance_x1) * i - enhance_y* (enhance_x0 + gray_th)) / (enhance_x0 + enhance_x1);
		}
		if (i >= gray_th - enhance_x0 && i < gray_th + enhance_x1 + enhance_y) {
			if (i < gray_th + enhance_x1)
				lut1[i] = ((enhance_y + enhance_x0 + enhance_x1) * i - enhance_y* (gray_th - enhance_x0)) / (enhance_x0 + enhance_x1);
			else
				lut1[i] = gray_th + enhance_x1 + enhance_y;
		}
	}
	if (cpara.method & OPT_DEBUG_EN) {
		for (int i = 0; i < 20; i++)
			qDebug("lut0 %3d:%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d\n", i*10, lut0[i * 10], lut0[i * 10 + 1], lut0[i * 10 + 2],
				lut0[i * 10 + 3], lut0[i * 10 + 4], lut0[i * 10 + 5], lut0[i * 10 + 6], lut0[i * 10 + 7], lut0[i * 10 + 8], lut0[i * 10 +9]);
		for (int i = 0; i < 20; i++)
			qDebug("lut1 %3d:%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d\n", i*10, lut1[i * 10], lut1[i * 10 + 1], lut1[i * 10 + 2],
			lut1[i * 10 + 3], lut1[i * 10 + 4], lut1[i * 10 + 5], lut1[i * 10 + 6], lut1[i * 10 + 7], lut1[i * 10 + 8], lut1[i * 10 + 9]);
	}

	for (int y = 0; y < img.rows; y++) {
		unsigned char * p_img = img.ptr<unsigned char>(y);
		unsigned char * p_edge = edge.ptr<unsigned char>(y);
		for (int x = 0; x < img.cols; x++)
		switch (p_edge[x]) {
		case SEEM_BE_INSU:
			p_img[x] = lut0[p_img[x]];
			break;
		case SEEM_BE_WIRE:
			p_img[x] = lut1[p_img[x]];
			break;
		}
	}
	d.l[layer].ig_valid = false;

	Mat & m = d.l[layer].v[idx2].d;
	m.at<int>(6, 1) = max(gray_th + enhance_x1 + enhance_y, gv - 10);
	m.at<int>(6, 2) = lut1[m.at<int>(6, 1)];
	m.at<int>(5, 1) = gray_th + enhance_x1;
	m.at<int>(5, 2) = gray_th + enhance_x1 + enhance_y;
	m.at<int>(4, 1) = gray_th + enhance_x1 / 2;
	m.at<int>(4, 2) = gray_th + (enhance_x1 + enhance_y)/2;
	m.at<int>(3, 1) = gray_th - enhance_x1 / 2;
	m.at<int>(3, 2) = gray_th - (enhance_x1 - enhance_y)/2;
	m.at<int>(2, 1) = gray_th - enhance_x1;
	m.at<int>(2, 2) = gray_th - enhance_x1 - enhance_y;
	m.at<int>(1, 1) = min(gray_th - enhance_x1 - enhance_y, 10);
	m.at<int>(1, 2) = lut0[m.at<int>(1, 1)];

	if (cpara.method & OPT_DEBUG_EN) {
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = img.clone();
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_enhance.jpg", img);
		Mat debug_draw;
		cvtColor(img, debug_draw, CV_GRAY2BGR);
		CV_Assert(debug_draw.type() == CV_8UC3 && debug_draw.rows == edge.rows && debug_draw.cols == edge.cols);
		for (int y = 0; y < debug_draw.rows; y++) {
			unsigned char * p_draw = debug_draw.ptr<unsigned char>(y);
			unsigned char * p_edge = edge.ptr<unsigned char>(y);
			for (int x = 0; x < debug_draw.cols; x++) {
				switch (p_edge[x]) {
				case SEEM_BE_INSU:
					p_draw[3 * x + 2] += max(p_draw[3 * x + 2] >> 3, 10); //draw skin red
					break;
				case SEEM_BE_WIRE:
					p_draw[3 * x + 1] += max(p_draw[3 * x + 1] >> 3, 10); //draw skin green
					break;
				}
			}
		}
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			debug_idx = (debug_idx + 1) & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw.clone();
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_enhance_iw.jpg", debug_draw);
	}
	
}

/*     31..24 23..16   15..8   7..0
opt0:   inc1   inc0   w_long1 w_long0
opt1:up_prob search_opt  th    w_num
opt2: w_subtype w_pattern w_dir  w_type
opt3: w_subtype w_pattern w_dir  w_type
opt4: w_subtype w_pattern w_dir  w_type
opt5: w_subtype w_pattern w_dir  w_type
dir=0 is shuxian, 1 is hengxian.
long0 & long1 decides detect rect, inc0 & & inc1 reduce compute time
th is maximum deviation
up_prob 1 means update d.l[layer].prob, 2 means clear d.l[layer].prob, 4 means update with mask
3 means clear and then update, 7 means clear and then update with mask, 5 means not clear, update with mask
method_opt
0: for gray level turn_points  input
1: for shadow prob output
2: for remove via mask input
It assume wire is lighter than insu, it will update d.prob
*/
static void coarse_line_search(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	int idx = cpara.method_opt & 0xf;
	int idx1 = cpara.method_opt >> 4 & 0xf;
	int idx2 = cpara.method_opt >> 8 & 0xf;
	CV_Assert(img.type() == CV_8UC1);
	if (d.l[layer].v[idx].type != TYPE_GRAY_LEVEL) {
		qCritical("coarse_line_search gray level idx[%d]=%d, error", idx, d.l[layer].v[idx].type);
		return;
	}
	int w_long0 = cpara.opt0 & 0xff, w_long1 = cpara.opt0 >> 8 & 0xff;
	int w_inc0 = cpara.opt0 >> 16 & 0xff, w_inc1 = cpara.opt0 >> 24 & 0xff;
	int th = cpara.opt1 >> 8 & 0xff, search_opt = cpara.opt1 >> 16 & 0xff;
	int w_num = cpara.opt1 & 0xff;
	int update_prob = cpara.opt1 >> 24 & 0xff;
	qInfo("coarse_line_search, l=%d, tp_idx=%d, prob_idx=%d, w_long0=%d,w_long1=%d, w_inc0=%d,w_inc1=%d, w_num=%d, th=%d, search_opt=%d, up_prob=%d",
		layer, idx, idx1, w_long0, w_long1, w_inc0, w_inc1, w_num, th, search_opt, update_prob);

	Mat & via_mask = d.l[layer].v[idx2].d;
	if (update_prob & COARSE_LINE_UPDATE_WITH_MASK && d.l[layer].v[idx2].type != TYPE_REMOVE_VIA_MASK) {
		qCritical("coarse_line_search idx2[%d]=%d, error", idx2, d.l[layer].v[idx2].type);
		return;
		if (via_mask.type() != CV_8UC1) {
			qCritical("coarse_line_search,  via_mask.type(%d)!=%d", via_mask.type(), CV_8UC1);
			return;
		}
	}

	struct WireDetectInfo wpara_pack[] = {
		{ cpara.opt2 >> 24 & 0xff, cpara.opt2 >> 16 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt2 & 0xff, 0, 0, 0 },
		{ cpara.opt3 >> 24 & 0xff, cpara.opt3 >> 16 & 0xff, cpara.opt3 >> 8 & 0xff, cpara.opt3 & 0xff, 0, 0, 0 },
		{ cpara.opt4 >> 24 & 0xff, cpara.opt4 >> 16 & 0xff, cpara.opt4 >> 8 & 0xff, cpara.opt4 & 0xff, 0, 0, 0 },
		{ cpara.opt5 >> 24 & 0xff, cpara.opt5 >> 16 & 0xff, cpara.opt5 >> 8 & 0xff, cpara.opt5 & 0xff, 0, 0, 0 },
	};
	vector<WireDetectInfo> wpara;
	for (int i = 0; i < w_num; i++) {
		if (wpara_pack[i].w_dir >= 16)
			qWarning("wire %d bad dir=%d", i, wpara_pack[i].w_dir);
		for (int dir = 0; dir <= 3; dir++)
		if (wpara_pack[i].w_dir >> dir & 1) {
			wpara.push_back(wpara_pack[i]);
			wpara.back().w_dir = dir;
		}
	}
	vector<WireKeyPoint> wires[2];
	int w_guard[256];
	int x0 = w_long1 / 2, y0 = w_long0 / 2;
	for (int i = 0; i < (int)wpara.size(); i++) {
		VWParameter * vw = d.l[layer].vw.get_vw_para(wpara[i].w_pattern, wpara[i].w_type, wpara[i].w_subtype);
		if (vw == NULL) {
			qCritical("coarse_line_search invalid wire info pattern=%d, type=%d, subtype=%d", wpara[i].w_pattern, wpara[i].w_type, wpara[i].w_subtype);
			return;
		}
		bool finish = false;
		int j = 0;
		while (!finish) {
			if (vw->w.subtype < WIRE_SUBTYPE_MSBRANCH) {
				wpara[i].w_wide = (wpara[i].w_dir == 0) ? vw->w.w_wide : vw->w.w_high;
				wpara[i].i_wide = (wpara[i].w_dir == 0) ? vw->w.i_wide : vw->w.i_high;
				w_guard[wpara[i].w_type] = vw->w.guard;
				wpara[i].gamma = vw->w.arfactor / 100.0;
				finish = true;
			}
			else {
				wpara[i].w_wide = vw->w2.mwide_min + 3 * j;
				j++;
				wpara[i].w_type = d.l[layer].wts.add(vw->w2.type, vw->w2.subtype, vw->w2.pattern, wpara[i].w_wide);
				wpara[i].i_wide = vw->w2.i_wide;
				w_guard[wpara[i].w_type] = vw->w2.guard;
				wpara[i].gamma = vw->w2.arfactor / 100.0;
				finish = (wpara[i].w_wide >= vw->w2.mwide_max);
			}
			qInfo("%d:type=%d, dir=%d, pattern=%d, w_wide=%d, i_wide=%d, gamma=%f", i, wpara[i].w_type,
				wpara[i].w_dir, wpara[i].w_pattern, wpara[i].w_wide, wpara[i].i_wide, wpara[i].gamma);
			if (wpara[i].w_dir > 3) {
				qCritical("coarse_line_search w_dir(%d) error", wpara[i].w_dir);
				return;
			}

			if (wpara[i].w_wide / 2 + wpara[i].i_wide >= d.l[layer].compute_border) {
				qCritical("coarse_line_search: w_wide(%d) / 2 + i_wide(%d) >= compute_border(%d)",
					wpara[i].w_wide, wpara[i].i_wide, d.l[layer].compute_border);
				return;
			}

			WireKeyPoint wire;
			switch (wpara[i].w_dir) {
			case DIR_UP:
				wire.offset[0] = Point(-wpara[i].w_wide / 2 - wpara[i].i_wide, -w_long0 / 2);
				wire.offset[1] = Point(-wpara[i].w_wide / 2, -w_long0 / 2);
				wire.offset[2] = Point(wpara[i].w_wide - wpara[i].w_wide / 2, -w_long0 / 2);
				wire.offset[3] = Point(wpara[i].i_wide + wpara[i].w_wide - wpara[i].w_wide / 2, -w_long0 / 2);
				wire.offset[4] = Point(-wpara[i].w_wide / 2 - wpara[i].i_wide, w_long0 - w_long0 / 2);
				wire.offset[5] = Point(-wpara[i].w_wide / 2, w_long0 - w_long0 / 2);
				wire.offset[6] = Point(wpara[i].w_wide - wpara[i].w_wide / 2, w_long0 - w_long0 / 2);
				wire.offset[7] = Point(wpara[i].i_wide + wpara[i].w_wide - wpara[i].w_wide / 2, w_long0 - w_long0 / 2);
				wire.type = wpara[i].w_type;
				wire.a0 = 1.0 / ((wpara[i].i_wide * 2 + wpara[i].w_wide) * w_long0);
				wire.a1 = (float)wpara[i].w_wide / (wpara[i].i_wide * 2 + wpara[i].w_wide);
				wire.a2 = (float)wpara[i].i_wide / (wpara[i].i_wide * 2 + wpara[i].w_wide);
				wire.arf = pow(((wpara[i].i_wide * 2 + wpara[i].w_wide) * w_long0), 2 * wpara[i].gamma);
				wire.shape1 = BRICK_I_0;
				wire.shape2 = BRICK_II_0;
				wire.shape3 = BRICK_II_180;
				wire.shape4 = BRICK_III_0;
				wires[0].push_back(wire);
				x0 = max(x0, wpara[i].w_wide / 2 + wpara[i].i_wide + 1);
				break;
			case DIR_RIGHT:
				wire.offset[0] = Point(-w_long1 / 2, -wpara[i].w_wide / 2 - wpara[i].i_wide);
				wire.offset[1] = Point(-w_long1 / 2, -wpara[i].w_wide / 2);
				wire.offset[2] = Point(-w_long1 / 2, wpara[i].w_wide - wpara[i].w_wide / 2);
				wire.offset[3] = Point(-w_long1 / 2, wpara[i].i_wide + wpara[i].w_wide - wpara[i].w_wide / 2);
				wire.offset[4] = Point(w_long1 - w_long1 / 2, -wpara[i].w_wide / 2 - wpara[i].i_wide);
				wire.offset[5] = Point(w_long1 - w_long1 / 2, -wpara[i].w_wide / 2);
				wire.offset[6] = Point(w_long1 - w_long1 / 2, wpara[i].w_wide - wpara[i].w_wide / 2);
				wire.offset[7] = Point(w_long1 - w_long1 / 2, wpara[i].i_wide + wpara[i].w_wide - wpara[i].w_wide / 2);
				wire.type = wpara[i].w_type;
				wire.a0 = 1.0 / ((wpara[i].i_wide * 2 + wpara[i].w_wide) * w_long1);
				wire.a1 = (float)wpara[i].w_wide / (wpara[i].i_wide * 2 + wpara[i].w_wide);
				wire.a2 = (float)wpara[i].i_wide / (wpara[i].i_wide * 2 + wpara[i].w_wide);
				wire.arf = pow(((wpara[i].i_wide * 2 + wpara[i].w_wide) * w_long1), 2 * wpara[i].gamma);
				wire.shape1 = BRICK_I_90;
				wire.shape2 = BRICK_II_90;
				wire.shape3 = BRICK_II_270;
				wire.shape4 = BRICK_III_90;
				wires[1].push_back(wire);
				y0 = max(y0, wpara[i].w_wide / 2 + wpara[i].i_wide + 1);
				break;
			default:
				qCritical("bad dir %d", wpara[i].w_dir);
				break;
			}
		}
	}

	int gl = find_index(d.l[layer].v[idx].d, (int)GRAY_L0);
	gl = d.l[layer].v[idx].d.at<int>(gl, 2);
	int gm = find_index(d.l[layer].v[idx].d, (int)GRAY_M0);
	gm = d.l[layer].v[idx].d.at<int>(gm, 2);
	qInfo("coarse_line_search, gl=%d, gm=%d", gl, gm);

	if (search_opt & COARSE_LINE_SEARCH_CLEAR_COLOR) {
		clip_img(img, gl, gm, img);
		for (int i = 0; i < d.l[layer].v[idx].d.rows; i++) {
			int color = d.l[layer].v[idx].d.at<int>(i, 2);
			color = (color > gm) ? gm - gl : ((color < gl) ? 0 : color - gl);
			d.l[layer].v[idx].d.at<int>(i, 2) = color;
		}
		d.l[layer].ig_valid = false;
		gm = gm - gl;
		gl = 0;
	}

	d.l[layer].validate_ig();
	Mat & ig = d.l[layer].ig;
	Mat & iig = d.l[layer].iig;
	d.l[layer].v[idx1].type = TYPE_SHADOW_PROB;
	d.l[layer].v[idx1].d = d.l[layer].prob.clone();
	Mat & prob = d.l[layer].v[idx1].d;

	//clear prob
	if (update_prob & COARSE_LINE_CLEAR_PROB) {
		for (int y = 0; y < d.l[layer].prob.rows; y++) {
			unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y);
			if (update_prob & COARSE_LINE_UPDATE_WITH_MASK) {
				for (int x = 0; x < d.l[layer].prob.cols * 2; x++) {
					int x0 = PROB_X(p_prob[x]), y0 = PROB_Y(p_prob[x]);
					if (x0 < via_mask.cols && y0 < via_mask.rows && via_mask.at<unsigned char>(y0, x0))
						continue;
					p_prob[x] = 0xffffffffffffffffULL;
				}
			}
			else {
				for (int x = 0; x < d.l[layer].prob.cols * 2; x++)
					p_prob[x] = 0xffffffffffffffffULL;
			}
		}
	}
	//search BRICK_I_0 and BRICK_I_90
	for (int dir = 0; dir < 2; dir++) {
		vector<WireKeyPoint> & w = wires[dir];
		if (w.empty())
			continue;
		int dy = (dir == 0) ? w_inc0 : 1;
		int dx = (dir == 0) ? 1 : w_inc1;
		for (int y = y0; y < img.rows - y0; y += dy) {
			for (unsigned i = 0; i < w.size(); i++)
			for (int j = 0; j < 8; j++) {
				w[i].p_ig[j] = ig.ptr<int>(y + w[i].offset[j].y) + x0 + w[i].offset[j].x;
				w[i].p_iig[j] = iig.ptr<int>(y + w[i].offset[j].y) + x0 + w[i].offset[j].x;
				CV_Assert(y + w[i].offset[j].y >= 0 && y + w[i].offset[j].y < ig.rows && x0 + w[i].offset[j].x >= 0);
			}
			for (int x = x0; x < img.cols - x0; x += dx) {
				unsigned s0 = MAKE_S(0xffff, 0xff, w[0].shape4);;
				for (unsigned i = 0; i < w.size(); i++) {
					unsigned s1 = 0xffffffff;
					int sum0 = w[i].p_ig[5][0] + w[i].p_ig[0][0] - w[i].p_ig[1][0] - w[i].p_ig[4][0];
					int sum1 = w[i].p_ig[1][0] + w[i].p_ig[6][0] - w[i].p_ig[2][0] - w[i].p_ig[5][0];
					int sum2 = w[i].p_ig[2][0] + w[i].p_ig[7][0] - w[i].p_ig[3][0] - w[i].p_ig[6][0];
					int ssum0 = w[i].p_iig[5][0] + w[i].p_iig[0][0] - w[i].p_iig[1][0] - w[i].p_iig[4][0];
					int ssum1 = w[i].p_iig[1][0] + w[i].p_iig[6][0] - w[i].p_iig[2][0] - w[i].p_iig[5][0];
					int ssum2 = w[i].p_iig[2][0] + w[i].p_iig[7][0] - w[i].p_iig[3][0] - w[i].p_iig[6][0];
					float part0_0 = (ssum0 - gl * sum0 * 2) * w[i].a0 + gl * gl * w[i].a2;
					float part0_1 = (ssum0 - gm * sum0 * 2) * w[i].a0 + gm * gm * w[i].a2;
					float part1_0 = (ssum1 - gl * sum1 * 2) * w[i].a0 + gl * gl * w[i].a1;
					float part1_1 = (ssum1 - gm * sum1 * 2) * w[i].a0 + gm * gm * w[i].a1;
					float part2_0 = (ssum2 - gl * sum2 * 2) * w[i].a0 + gl * gl * w[i].a2;
					float part2_1 = (ssum2 - gm * sum2 * 2) * w[i].a0 + gm * gm * w[i].a2;
					unsigned score0 = part0_0 + part1_0 + part2_0;
					unsigned score1 = part0_0 + part1_1 + part2_0;
					unsigned score2 = part0_1 + part1_1 + part2_0;
					unsigned score3 = part0_0 + part1_1 + part2_1;
					unsigned score4 = part0_1 + part1_1 + part2_1;
					CV_Assert(score0 < 65536 && score1 < 65536 && score2 < 65536 && score3 < 65536);
					score0 = MAKE_S(score0, i, BRICK_NO_WIRE);
					score1 = MAKE_S(score1, i, w[i].shape1);
					score2 = MAKE_S(score2, i, w[i].shape2);
					score3 = MAKE_S(score3, i, w[i].shape3);
					score4 = MAKE_S(score4, i, w[i].shape4);
					s1 = min(min(min(score1, score0), min(score2, score3)), score4);
					if (S_SHAPE(s0) == BRICK_I_0 || S_SHAPE(s0) == BRICK_I_90) {
						if (S_SHAPE(s1) == BRICK_I_0 || S_SHAPE(s1) == BRICK_I_90) //two wire BRICK_I compare
							s0 = min(s0, s1);
					}
					else {
						if (S_SHAPE(s1) == BRICK_I_0 || S_SHAPE(s1) == BRICK_I_90) //s1 is BRICK_I, s0 is not BRICK_I
							s0 = s1;
						else
							s0 = min(s0, s1); //two non BRICK_I compare
					}
					for (int j = 0; j < 8; j++) {
						w[i].p_ig[j] += dx;
						w[i].p_iig[j] += dx;
					}
				}
				unsigned score = S_SCORE(s0);
				unsigned type = S_TYPE(s0);
				score = sqrt(score * w[type].arf); //TODO remove sqrt
				CV_Assert(score < 65536);
				score = (score < MIN_SCORE) ? MIN_SCORE : score;
				s0 = MAKE_S(score, w[type].type, S_SHAPE(s0));
				push_new_prob(prob, x, y, s0, d.l[layer].gs);
				unsigned long long * ps = get_prob(prob, x, y, d.l[layer].gs);
				if (PROB_SHAPE(ps[0]) != BRICK_I_0 && PROB_SHAPE(ps[0]) != BRICK_I_90) {
					if (PROB_SHAPE(ps[1]) == BRICK_I_0 || PROB_SHAPE(ps[1]) == BRICK_I_90) {
						swap(ps[0], ps[1]);
						SET_PROB_SCORE(ps[1], PROB_SCORE(ps[0]) + 1);
					}
					else {
						ps[1] = 0xffffffffffffffffULL;
					}
				}
				if ((s0 & 0xff) == BRICK_NO_WIRE && update_prob & COARSE_LINE_UPDATE_PROB) {
					if (!(update_prob & COARSE_LINE_UPDATE_WITH_MASK))
						push_new_prob(d.l[layer].prob, x, y, s0, d.l[layer].gs);
					else
					if (!via_mask.at<unsigned char>(y, x))
						push_new_prob(d.l[layer].prob, x, y, s0, d.l[layer].gs);
				}
			}
		}
	}
	Mat debug_draw;
	if (cpara.method & OPT_DEBUG_EN)
		debug_draw = img.clone();

	//search hot points, which is BRICK_I_0 or BRICK_I_90 and has minimum prob than nearby guard points
	for (int y = 0; y < prob.rows; y++) {
		unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
		for (int x = 0; x < prob.cols; x++)
		if ((PROB_SHAPE(p_prob[2 * x]) == BRICK_I_0 || PROB_SHAPE(p_prob[2 * x]) == BRICK_I_90)
			&& PROB_SCORE(p_prob[2 * x]) <= (unsigned)(th * th)) {
			bool pass = true;
			/*pass = true means prob BRICK_I_0 < all row BRICK_I_0, && < all nearby guard BRICK_I_90
			or prob BRICK_I_90< all column BRICK_I_90, && < all nearby guard BRICK_I_0
			*/
			unsigned long long prob0 = p_prob[2 * x];
			int cr = w_guard[PROB_TYPE(prob0)];
			int guard = (cr - 1) / d.l[layer].gs + 1;
			int y1 = max(0, y - guard), y2 = min(prob.rows - 1, y + guard);
			int x1 = max(0, x - guard), x2 = min(prob.cols - 1, x + guard);
			for (int yy = y1; yy <= y2; yy++)  {
				unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
				for (int xx = x1; xx <= x2; xx++) {
					if (p_prob2[2 * xx] < prob0 && PROB_SHAPE(p_prob2[2 * xx]) != PROB_SHAPE(prob0) && //this check make sure BRICK_I_0 < all nearby BRICK_I_90
						(PROB_SHAPE(p_prob2[2 * xx]) == BRICK_I_0 || PROB_SHAPE(p_prob2[2 * xx]) == BRICK_I_90)) { // and BRICK_I_90 < all nearby BRICK_I_0
						if (abs(PROB_X(p_prob2[2 * xx]) - PROB_X(prob0)) <= cr &&
							abs(PROB_Y(p_prob2[2 * xx]) - PROB_Y(prob0)) <= cr) {
							pass = false;
							yy = y2;
							break;
						}
					}
				}
			}

			if (PROB_SHAPE(prob0) == BRICK_I_0) {
				for (int xx = x1; xx <= x2; xx++)
				if (p_prob[2 * xx] < prob0 && PROB_SHAPE(p_prob[2 * xx]) == BRICK_I_0) {//this check make sure BRICK_I_0 <all row BRICK_I_0,
					if (abs(PROB_X(p_prob[2 * xx]) - PROB_X(prob0)) <= cr)
						pass = false;
				}
			}
			else { //PROB_SHAPE(prob0) == BRICK_I_90
				for (int yy = y1; yy <= y2; yy++) {
					unsigned long long prob1 = prob.at<unsigned long long>(yy, 2 * x);
					if (prob1 < prob0 && PROB_SHAPE(prob1) == BRICK_I_90) { //this check make sure BRICK_I_90< all column BRICK_I_90
						if (abs(PROB_Y(prob1) - PROB_Y(prob0)) <= cr)
							pass = false;
					}
				}
			}
			if (!pass)
				continue;

			if (update_prob & COARSE_LINE_UPDATE_PROB) {
				if (!(update_prob & COARSE_LINE_UPDATE_WITH_MASK))
					push_new_prob(d.l[layer].prob, p_prob[2 * x], d.l[layer].gs);
				else
				if (!via_mask.at<unsigned char>(y, x))
					push_new_prob(d.l[layer].prob, p_prob[2 * x], d.l[layer].gs);
			}
			if (cpara.method & OPT_DEBUG_EN)
			if (PROB_SHAPE(prob0) == BRICK_I_0)
				circle(debug_draw, Point(PROB_X(p_prob[2 * x]), PROB_Y(p_prob[2 * x])), 2, Scalar::all(255));
			else
				rectangle(debug_draw, Rect(PROB_X(p_prob[2 * x]) - 2, PROB_Y(p_prob[2 * x]) - 2, 5, 5), Scalar::all(255), 1);
		}
	}

	if (cpara.method & OPT_DEBUG_EN) {
		d.l[layer].check_prob();
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_coarse_line.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw;
		}
	}
}

/*
31..24 23..16   15..8   7..0
opt0:							vnum
opt1:			percent	wide subtype
opt2:			percent	wide subtype
opt3:			percent	wide subtype
opt4:			percent	wide subtype
opt5:			percent	wide subtype
percent is how much point for mask
wide is radius * sqrt(2)
method_opt
0: for via search mask output
*/
static void coarse_via_search_mask(PipeData & d, ProcessParameter & cpara)
{
	int idx = cpara.method_opt & 0xf;
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);

#define MAX_VIA_NUM 5
	int vnum = cpara.opt0 & 0xff;
	int v_subtype[MAX_VIA_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff };
	int v_wide[MAX_VIA_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt3 >> 8 & 0xff };
	int v_percent[MAX_VIA_NUM] = { cpara.opt1 >> 16 & 0xff, cpara.opt2 >> 16 & 0xff, cpara.opt3 >> 16 & 0xff };

	qInfo("coarse_via_search_mask, l=%d,vnum=%d", layer, vnum);
	if (vnum > MAX_VIA_NUM || vnum == 0) {
		qCritical("coarse_via_search_mask wrong parameter");
		return;
	}
	for (int i = 0; i < vnum; i++) {
		qInfo("%d: coarse_via_search_mask v_subtype=%d, wide=%d, percent=%d", i, v_subtype[i], v_wide[i], v_percent[i]);
		if (v_wide[i] / 2 >= d.l[layer].compute_border) {
			qCritical("coarse_via_search_mask wrong parameter");
			return;
		}
	}

	d.l[layer].v[idx].type = TYPE_VIA_MASK;
	Mat & mask = d.l[layer].v[idx].d;
	mask.create(img.rows, img.cols, CV_32S);
	mask = Scalar::all(0);
	d.l[layer].validate_ig();
	vector<vector<unsigned> >record(1025);
	for (int i = 0; i < vnum; i++) {
		int d1 = -v_wide[i] / 2;
		int d2 = v_wide[i] - v_wide[i] / 2;
		float area = 1.0f / (v_wide[i] * v_wide[i]);
		int total = 0;
		for (int y = d.l[layer].compute_border; y < img.rows - d.l[layer].compute_border; y++) {
			unsigned * p_ig1 = d.l[layer].ig.ptr<unsigned>(y + d1);
			unsigned * p_ig2 = d.l[layer].ig.ptr<unsigned>(y + d2);
			for (int x = d.l[layer].compute_border; x < img.cols - d.l[layer].compute_border; x++) {
				int sum = p_ig2[x + d2] + p_ig1[x + d1] - p_ig2[x + d1] - p_ig1[x + d2];
				int idx = sum * 4 * area;
				CV_Assert(idx >= 0 && idx <= 1024);
				record[idx].push_back(y << 16 | x);
				total++;
			}
		}

		total = total * v_percent[i] / 10000;
		for (int j = 1023; j > 0; j--) {
			for (int k = 0; k < record[j].size(); k++) {
				int y = record[j][k] >> 16;
				int x = record[j][k] & 0xffff;
				mask.at<int>(y, x) |= 1 << v_subtype[i];
			}
			total = total - (int)record[j].size();
			if (total < 0)
				break;
		}
		for (int j = 0; j < 1024; j++)
			record[j].clear();
	}

	if (cpara.method & OPT_DEBUG_EN) {
		Mat debug_draw = img.clone();
		for (int y = 0; y < img.rows; y++) {
			unsigned * p_mask = mask.ptr<unsigned>(y);
			unsigned char * p_debug = debug_draw.ptr<unsigned char>(y);
			for (int x = 0; x < img.cols; x++)
			if (p_mask[x])
				p_debug[x] = 255;
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_coarse_via_mask.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw;
		}
	}
#undef MAX_VIA_NUM
}

/*
31..24 23..16   15..8   7..0
opt0:				  update_fv vnum
opt1:	pattern th   subtype  type
opt2:	pattern th   subtype  type
opt3:	pattern th   subtype  type
opt4:	pattern th   subtype  type
opt5:	pattern th   subtype  type
update means update to viaset
via's guard + via's radius < w_long/2
method_opt
0: for gray level turn_points  input
1: for via search mask input
2: NA
3: for shadow prob input
*/
static void fine_via_search(PipeData & d, ProcessParameter & cpara)
{
	int idx = cpara.method_opt & 0xf;
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);
	if (d.l[layer].v[idx].type != TYPE_GRAY_LEVEL) {
		qCritical("fine_via_search gray level idx[%d]=%d, error", idx, d.l[layer].v[idx].type);
		return;
	}

	int glv[10];
	for (int i = 0; i < sizeof(glv) / sizeof(glv[0]); i++) {
		glv[i] = find_index(d.l[layer].v[idx].d, i);
		glv[i] = d.l[layer].v[idx].d.at<int>(glv[i], 2);
	}
	int idx1 = cpara.method_opt >> 4 & 0xf;
	if (d.l[layer].v[idx1].type != TYPE_VIA_MASK) {
		qCritical("fine_via_search search mask idx1[%d], error", idx1, d.l[layer].v[idx1].type);
		return;
	}
	Mat prob;
	int idx3 = cpara.method_opt >> 12 & 0xf;
	if (d.l[layer].v[idx3].type != TYPE_SHADOW_PROB) {
		prob = d.l[layer].prob.clone();
	}
	else
		prob = d.l[layer].v[idx3].d;
	Mat & mask = d.l[layer].v[idx1].d;
	int vnum = cpara.opt0 & 0xff;
	int update_fv = cpara.opt0 >> 8 & 0xff;
	qInfo("fine_via_search, l=%d, g_idx=%d, vmsk_idx=%d, prob_idx=%d, gl=%d, gm=%d, gh=%d, vnum=%d, update_fv=%d",
		layer, idx, idx1, idx3, glv[0], glv[1], glv[2], vnum, update_fv);

#define MAX_VIA_NUM 3	
	if (vnum > MAX_VIA_NUM || vnum == 0) {
		qCritical("fine_via_search wrong parameter");
		return;
	}
	int v_type[MAX_VIA_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff };
	int v_subtype[MAX_VIA_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt3 >> 8 & 0xff };
	int v_th[MAX_VIA_NUM] = { cpara.opt1 >> 16 & 0xff, cpara.opt2 >> 16 & 0xff, cpara.opt3 >> 16 & 0xff };
	int v_pattern[MAX_VIA_NUM] = { cpara.opt1 >> 24 & 0xff, cpara.opt2 >> 24 & 0xff, cpara.opt3 >> 24 & 0xff };
	int v_guard[MAX_VIA_NUM];
	int v_pair_d[MAX_VIA_NUM];
	QScopedPointer<ViaComputeScore> vcs[MAX_VIA_NUM];

	for (int i = 0; i < vnum; i++) {
		ViaParameter via_para;
		VWParameter * vw = d.l[layer].vw.get_vw_para(v_pattern[i], v_type[i], v_subtype[i]);
		if (vw == NULL) {
			qCritical("fine_via_search invalid via info %d, %d", v_type[i], v_subtype[i]);
			return;
		}
		via_para = vw->v;
		v_guard[i] = via_para.guard;
		v_pair_d[i] = via_para.pair_distance;
		if (via_para.gray0 < sizeof(glv) / sizeof(glv[0]))
			via_para.gray0 = glv[via_para.gray0];
		if (via_para.gray1 < sizeof(glv) / sizeof(glv[0]))
			via_para.gray1 = glv[via_para.gray1];
		if (via_para.gray2 < sizeof(glv) / sizeof(glv[0]))
			via_para.gray2 = glv[via_para.gray2];
		if (via_para.gray3 < sizeof(glv) / sizeof(glv[0]))
			via_para.gray3 = glv[via_para.gray3];
		qInfo("%d: fine_via_search type=%d, subtype=%d, pattern=%d, th=%d, g0=%d, g1=%d, g2=%d, g3=%d", i, v_type[i],
			v_subtype[i], v_pattern[i], v_th[i], via_para.gray0, via_para.gray1, via_para.gray2, via_para.gray3);
		vcs[i].reset(ViaComputeScore::create_via_compute_score(layer, via_para, d));
	}
	d.l[layer].validate_ig();

	for (int y = d.l[layer].compute_border; y < img.rows - d.l[layer].compute_border; y++) {
		unsigned * p_mask = mask.ptr<unsigned>(y);
		for (int x = d.l[layer].compute_border; x < img.cols - d.l[layer].compute_border; x++)
		if (p_mask[x]) {
			unsigned score = 0xffffffff;
			for (int i = 0; i < vnum; i++)
			if (!vcs[i].isNull() && (p_mask[x] >> v_subtype[i] & 1)) {
				unsigned s = vcs[i]->compute(x, y);
				s = MAKE_S(s, i, BRICK_VIA);
				score = min(score, s);
			}
			push_new_prob(prob, x, y, score, d.l[layer].gs);
		}
	}

	//choose local unique via
	vector<ViaInfo> & via_loc = d.l[layer].via_info;
	for (int y = 0; y < prob.rows; y++) {
		unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
		for (int x = 0; x < prob.cols; x++)
		if (PROB_SHAPE(p_prob[2 * x]) == BRICK_VIA || PROB_SHAPE(p_prob[2 * x + 1]) == BRICK_VIA) {
			unsigned long long prob0 = (PROB_SHAPE(p_prob[2 * x]) == BRICK_VIA) ? p_prob[2 * x] : p_prob[2 * x + 1];
			int gi = PROB_TYPE(prob0);
			bool pass = (PROB_SCORE(prob0) < v_th[gi] * v_th[gi]); //pass means it is lowest prob among nearby guard probs,pass=true => pass2=true
			bool pass2 = true; //pass2 means it is lowest prob among nearby guard via probs				
			int cr = v_guard[gi];
			int guard = (cr - 1) / d.l[layer].gs + 1;
			int y1 = max(y - guard, 0), y2 = min(y + guard, prob.rows - 1);
			int x1 = max(x - guard, 0), x2 = min(x + guard, prob.cols - 1);
			for (int yy = y1; yy <= y2; yy++) {
				unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
				for (int xx = 2 * x1; xx < 2 * x2 + 2; xx++) {
					if (p_prob2[xx] < prob0) {
						if (abs(PROB_X(p_prob2[xx]) - PROB_X(prob0)) <= cr &&
							abs(PROB_Y(p_prob2[xx]) - PROB_Y(prob0)) <= cr)
							pass = false;
					}
					if (PROB_SHAPE(p_prob2[xx]) == BRICK_VIA && p_prob2[xx] < prob0) {
						if (abs(PROB_X(p_prob2[xx]) - PROB_X(prob0)) <= cr &&
							abs(PROB_Y(p_prob2[xx]) - PROB_Y(prob0)) <= cr) {
							pass2 = false;
							yy = y2;
							break;
						}
					}
				}
			}
			if (pass) {
				unsigned long long score = prob0;
				SET_PROB_TYPE(score, v_type[gi]);
				push_new_prob(d.l[layer].prob, score, d.l[layer].gs);
				via_loc.push_back(ViaInfo(Point(PROB_X(prob0), PROB_Y(prob0)), v_type[gi], v_subtype[gi], v_pattern[gi], v_pair_d[gi], PROB_S(prob0)));
			}
		}
	}

	for (int i = 0; i < (int)via_loc.size(); i++) {
		for (int j = i - 1; j >= 0; j--) {
			if (via_loc[i].xy.y - via_loc[j].xy.y > via_loc[i].pair_d + d.l[layer].gs)
				break;
			if (via_loc[i].xy.x - via_loc[j].xy.x < via_loc[i].pair_d && via_loc[i].xy.x > via_loc[j].xy.x
				&& abs(via_loc[i].xy.y - via_loc[j].xy.y) < 3) {
				via_loc[i].via_adj_mask |= 1 << DIR_LEFT;
				via_loc[i].v_pair[DIR_LEFT] = j;
			}
			if (via_loc[i].xy.y - via_loc[j].xy.y < via_loc[i].pair_d && via_loc[i].xy.y > via_loc[j].xy.y
				&& abs(via_loc[i].xy.x - via_loc[j].xy.x) < 3) {
				via_loc[i].via_adj_mask |= 1 << DIR_UP;
				via_loc[i].v_pair[DIR_UP] = j;
			}
			if (via_loc[j].xy.x - via_loc[i].xy.x < via_loc[i].pair_d && via_loc[j].xy.x > via_loc[i].xy.x
				&& abs(via_loc[i].xy.y - via_loc[j].xy.y) < 3) {
				via_loc[i].via_adj_mask |= 1 << DIR_RIGHT;
				via_loc[i].v_pair[DIR_RIGHT] = j;
			}
		}
		for (int j = i + 1; j < (int)via_loc.size(); j++) {
			if (via_loc[j].xy.y - via_loc[i].xy.y > via_loc[i].pair_d + d.l[layer].gs)
				break;
			if (via_loc[i].xy.x - via_loc[j].xy.x < via_loc[i].pair_d && via_loc[i].xy.x > via_loc[j].xy.x
				&& abs(via_loc[i].xy.y - via_loc[j].xy.y) < 3) {
				via_loc[i].via_adj_mask |= 1 << DIR_LEFT;
				via_loc[i].v_pair[DIR_LEFT] = j;
			}
			if (via_loc[j].xy.x - via_loc[i].xy.x < via_loc[i].pair_d && via_loc[j].xy.x > via_loc[i].xy.x
				&& abs(via_loc[i].xy.y - via_loc[j].xy.y) < 3) {
				via_loc[i].via_adj_mask |= 1 << DIR_RIGHT;
				via_loc[i].v_pair[DIR_RIGHT] = j;
			}
			if (via_loc[j].xy.y - via_loc[i].xy.y < via_loc[i].pair_d && via_loc[j].xy.y > via_loc[i].xy.y
				&& abs(via_loc[i].xy.x - via_loc[j].xy.x) < 3) {
				via_loc[i].via_adj_mask |= 1 << DIR_DOWN;
				via_loc[i].v_pair[DIR_DOWN] = j;
			}
		}
	}
	Mat debug_draw;
	if (cpara.method & OPT_DEBUG_EN)
		debug_draw = img.clone();

	for (int i = 0; i < (int)via_loc.size(); i++) {
		if (cpara.method & OPT_DEBUG_EN)
			circle(debug_draw, via_loc[i].xy, 2, Scalar::all(0), 2);
	}
	if (cpara.method & OPT_DEBUG_EN) {
		d.l[layer].check_prob();
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_via.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw;
		}
	}
#undef MAX_VIA_NUM
}


/*
		31..24 23..16   15..8   7..0
opt0:	remove_opt default_dir check_len vnum
opt1:			pattern	subtype type
opt2:			pattern	subtype	type
opt3:			pattern	subtype	type
opt4:			pattern	subtype	type
opt5:			pattern	subtype	type
vnum is via number
default_dir is default dir when computing dir not deternmined
check_len is used for computing dir range
remove_opt is Remove via option, REMOVE_VIA_MASK means mark mask.
CLEAR_REMOVE_VIA_MASK mean clear it first.
REMOVE
method_opt
0: N/A
1: for mask output
2: N/A
*/
static void remove_via(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);

	int vnum = cpara.opt0 & 0xff;
	int check_len = cpara.opt0 >> 8 & 0xff;
	int default_dir = cpara.opt0 >> 16 & 0xff;
	int remove_opt = cpara.opt0 >> 24 & 0xff;
	qInfo("remove_via, l=%d check_len=%d, vnum=%d, default_dir=%d, cr_mask=%d",
		layer, check_len, vnum, default_dir, remove_opt);

	Mat mask;
	int idx1 = cpara.method_opt >> 4 & 0xf;
	if (remove_opt & REMOVE_VIA_MASK) {
		d.l[layer].v[idx1].type = TYPE_REMOVE_VIA_MASK;
		mask.create(img.rows, img.cols, CV_8U);
		if (remove_opt & CLEAR_REMOVE_VIA_MASK)
			mask = Scalar::all(0);
	}

#define MAX_VIA_NUM 3	
	if (vnum > MAX_VIA_NUM || vnum == 0) {
		qCritical("remove_via wrong vnum");
		return;
	}
	int v_type[MAX_VIA_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff };
	int v_subtype[MAX_VIA_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt3 >> 8 & 0xff };
	int v_pattern[MAX_VIA_NUM] = { cpara.opt1 >> 16 & 0xff, cpara.opt2 >> 16 & 0xff, cpara.opt3 >> 16 & 0xff };
	int v_rr[MAX_VIA_NUM];
	QScopedPointer<ViaRemove> vr[MAX_VIA_NUM];

	for (int i = 0; i < vnum; i++) {
		ViaParameter via_para;
		qInfo("remove_via type=%d, subtype=%d, pattern=%d", v_type[i], v_subtype[i], v_pattern[i]);
		VWParameter * vw = d.l[layer].vw.get_vw_para(v_pattern[i], v_type[i], v_subtype[i]);
		if (vw == NULL) {
			qCritical("remove_via invalid via info %d, %d, %d", v_type[i], v_subtype[i], v_pattern[i]);
			return;
		}
		via_para = vw->v;
		v_rr[i] = via_para.remove_rd;
		vr[i].reset(ViaRemove::create_via_remove(via_para, d.l[layer], remove_opt, check_len));
	}

	vector<ViaInfo> & via_info = d.l[layer].via_info;
	for (int i = 0; i < via_info.size(); i++) {
		int sel = -1;
		for (int j = 0; j < vnum; j++)
		if (v_type[j] == via_info[i].type && v_subtype[j] == via_info[i].subtype && v_pattern[j] == via_info[i].pattern && !vr[j].isNull()) {
			sel = j;
			break;
		}
		if (sel < 0)
			continue;
		int x0 = via_info[i].xy.x;
		int y0 = via_info[i].xy.y;
		//Following compute dir
		int dir = default_dir;
		if (dir < 2) {
			int cr = v_rr[sel] + check_len;
			int remove_check = (cr - 1) / d.l[layer].gs + 1;
			int y1 = max(y0 / d.l[layer].gs - remove_check, 0), y2 = min(y0 / d.l[layer].gs + remove_check, d.l[layer].prob.rows - 1);
			int x1 = max(x0 / d.l[layer].gs - remove_check, 0), x2 = min(x0 / d.l[layer].gs + remove_check, d.l[layer].prob.cols - 1);
			int i0 = 0, i90 = 0;
			for (int yy = y1; yy <= y2; yy++) {
				unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(yy);
				for (int xx = x1; xx <= x2; xx++) {
					if (PROB_SHAPE(p_prob[2 * xx]) == BRICK_I_0) {
						if (abs(PROB_X(p_prob[2 * xx]) - x0) <= cr &&
							abs(PROB_Y(p_prob[2 * xx]) - y0) <= cr)
							i0++;
					}
					if (PROB_SHAPE(p_prob[2 * xx]) == BRICK_I_90) {
						if (abs(PROB_X(p_prob[2 * xx]) - x0) <= cr &&
							abs(PROB_Y(p_prob[2 * xx]) - y0) <= cr)
							i90++;
					}
				}
			}
			dir = (i0 > i90) ? 0 : ((i90 == 0) ? default_dir : 1);
		}
		if (!(remove_opt & REMOVE_VIA_NOCHG_IMG))
			vr[sel]->remove(img, x0, y0, dir);
		if (remove_opt & REMOVE_VIA_MASK)
			vr[sel]->remove_mask(mask, x0, y0);
	}
	for (int i = 0; i < vnum; i++)
	if (!(remove_opt & REMOVE_VIA_NOCHG_IMG))
		vr[i]->finish(img);
#undef MAX_VIA_NUM
	if (cpara.method & OPT_DEBUG_EN) {
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_delvia.jpg", img);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = img.clone();
		}

		if (remove_opt & REMOVE_VIA_MASK) {
			Mat debug_draw = img.clone();
			for (int y = 0; y < mask.rows; y++) {
				unsigned char * p_mask = mask.ptr<unsigned char>(y);
				unsigned char * p_img = debug_draw.ptr<unsigned char>(y);
				for (int x = 0; x < mask.cols; x++)
				if (p_mask[x])
					p_img[x] = 0xff;
			}
			imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_delvia_mask.jpg", debug_draw);
		}
	}
	d.l[layer].ig_valid = false;
	if (remove_opt & REMOVE_VIA_MASK)
		d.l[layer].v[idx1].d = mask;
}

struct WireMaskInfo {
	int type_shape;
	int clong;
	int cwide;
	int subtype;
	int extend;
	int x1, y1, x2, y2, cx, cy;
	int mask;
};

/*
31..24 23..16   15..8   7..0
opt0:		          clear_mask wnum
opt1:	cwide	clong	 type	shape
opt2:					extend	subtype
opt3:	cwide	clong	 type	shape
opt4:					extend	subtype
opt5:	cwide	clong	 type	shape
opt6:					extend	subtype
cwide & clong is used for check connect
For BRICK_I_0, cwide is small (1 or 2), clong need to be bigger than coarse_line_search inc
method_opt
0: for search mask output
This function is called between coarse_line_search and fine_line_search
*/
static void hotpoint2fine_search_stmask(PipeData & d, ProcessParameter & cpara)
{
	int idx = cpara.method_opt & 0xf;
	int wnum = cpara.opt0 & 0xff;
	int layer = cpara.layer;

	Mat & mask = d.l[layer].v[idx].d;
	Mat & img = d.l[layer].img;
	d.l[layer].v[idx].type = TYPE_FINE_WIRE_MASK;
	int hotline_opt = cpara.opt0 >> 8 & 0xff;

	if (hotline_opt & HOTLINE_CLEAR_MASK) {
		mask.create(img.rows, img.cols, CV_32S);
		mask = Scalar::all(0);
	}
	if (mask.rows != img.rows || mask.cols != img.cols) {
		qCritical("hotpoint2fine_search_mask, mask.size(%d,%d)!=img.size(%d,%d)", mask.rows, mask.cols, img.rows, img.cols);
		return;
	}
	qInfo("hotpoint2fine_search_mask, l=%d, wnum=%d, hotline_opt=%d", layer, wnum, hotline_opt);
	if (wnum > 3) {
		qCritical("hotpoint2fine_search_mask wrong wnum");
		return;
	}
	struct WireMaskInfo wpara[] = {
		{ cpara.opt1 & 0xffff, cpara.opt1 >> 16 & 0xff, cpara.opt1 >> 24 & 0xff, cpara.opt2 & 0xff, cpara.opt2 >> 8 & 0xff },
		{ cpara.opt3 & 0xffff, cpara.opt3 >> 16 & 0xff, cpara.opt3 >> 24 & 0xff, cpara.opt4 & 0xff, cpara.opt4 >> 8 & 0xff },
		{ cpara.opt5 & 0xffff, cpara.opt5 >> 16 & 0xff, cpara.opt5 >> 24 & 0xff, cpara.opt6 & 0xff, cpara.opt6 >> 8 & 0xff }
	};

	vector <WireMaskInfo> w[2];
	for (int i = 0; i < wnum; i++) {
		qInfo("w%d:type=%d, shape=%d, clong=%d, cwide=%d, subtype=%d, extend=%d", i, wpara[i].type_shape >> 8, wpara[i].type_shape & 0xff,
			wpara[i].clong, wpara[i].cwide, wpara[i].subtype, wpara[i].extend);
		switch (wpara[i].type_shape & 0xff) {
		case  BRICK_I_0:
			wpara[i].cx = wpara[i].cwide;
			wpara[i].cy = wpara[i].clong;
			wpara[i].x2 = (wpara[i].cwide - 1) / d.l[layer].gs + 1;
			wpara[i].x1 = -wpara[i].x2;
			wpara[i].y1 = 1;
			wpara[i].y2 = (wpara[i].clong - 1) / d.l[layer].gs + 1;
			wpara[i].mask = 1 << (wpara[i].type_shape >> 8);
			w[0].push_back(wpara[i]);
			break;
		case BRICK_I_90:
			wpara[i].cx = wpara[i].clong;
			wpara[i].cy = wpara[i].cwide;
			wpara[i].x1 = 1;
			wpara[i].x2 = (wpara[i].clong - 1) / d.l[layer].gs + 1;
			wpara[i].y2 = (wpara[i].cwide - 1) / d.l[layer].gs + 1;
			wpara[i].y1 = -wpara[i].y2;
			wpara[i].mask = 1 << (wpara[i].type_shape >> 8);
			w[1].push_back(wpara[i]);
			break;
		default:
			qCritical("Error wrong shape");
			break;
		}
	}
	Mat & prob = d.l[layer].prob;
	Mat mark(prob.rows, prob.cols, CV_8UC1);
	mark = Scalar::all(0);
	for (int dir = 0; dir < 2; dir++)
	if (dir == 0) {
		if (w[dir].empty())
			continue;
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols; x++) {
				int i;
				for (i = 0; i < w[dir].size(); i++)
				if (PROB_TYPESHAPE(p_prob[2 * x]) == w[dir][i].type_shape && (mark.at<unsigned char>(y, x) & 1) == 0
					&& PROB_SCORE(p_prob[2 * x]) >= MIN_SCORE)
					break;
				if (i == w[dir].size())
					continue;
				int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]); //find hotline begining
				mark.at<unsigned char>(y, x) |= 1;
				mark_line(mask, Point(x0, y0), Point(x0, max(0, y0 - w[dir][i].extend)), w[dir][i].mask);
				bool check;
				do {
					int x1 = max(x0 / d.l[layer].gs + w[dir][i].x1, 0), x2 = min(x0 / d.l[layer].gs + w[dir][i].x2, prob.cols - 1);
					int y1 = max(y0 / d.l[layer].gs + w[dir][i].y1, 0), y2 = min(y0 / d.l[layer].gs + w[dir][i].y2, prob.rows - 1);
					check = false;
					for (int yy = y1; yy <= y2; yy++) { //search bottom for hotline
						unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
						for (int xx = x1; xx <= x2; xx++)
						if (PROB_TYPESHAPE(p_prob2[2 * xx]) == w[dir][i].type_shape) {
							if (abs(PROB_X(p_prob2[2 * xx]) - x0) <= w[dir][i].cx &&
								abs(PROB_Y(p_prob2[2 * xx]) - y0) <= w[dir][i].cy) {
								if (mark.at<unsigned char>(yy, xx) & 1)
									qWarning("hotpoint2fine_search_mask intersect dir=0 at (x=%d,y=%d) for (x0=%d,y0=%d), maybe cwide=%d too big",
									PROB_X(p_prob2[2 * xx]), PROB_Y(p_prob2[2 * xx]), x0, y0, w[dir][i].cx);

								mark.at<unsigned char>(yy, xx) |= 1;
								check = true;
								int nx0 = PROB_X(p_prob2[2 * xx]), ny0 = PROB_Y(p_prob2[2 * xx]);
								mark_line(mask, Point(x0, y0), Point(nx0, ny0), w[dir][i].mask);
								x0 = nx0, y0 = ny0;
								xx = x2, yy = y2;
							}
						}
					}
				} while (check);
				mark_line(mask, Point(x0, y0), Point(x0, min(mask.rows - 1, y0 + w[dir][i].extend)), w[dir][i].mask);
			}
		}
	}
	else {
		if (w[dir].empty())
			continue;
		for (int x = 0; x < prob.cols; x++) {
			for (int y = 0; y < prob.rows; y++) {
				unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
				int i;
				for (i = 0; i < w[dir].size(); i++)
				if (PROB_TYPESHAPE(p_prob[2 * x]) == w[dir][i].type_shape && (mark.at<unsigned char>(y, x) & 2) == 0
					&& PROB_SCORE(p_prob[2 * x]) >= MIN_SCORE)
					break;
				if (i == w[dir].size())
					continue;
				int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]); //find hotline begining
				mark.at<unsigned char>(y, x) |= 2;
				mark_line(mask, Point(x0, y0), Point(max(0, x0 - w[dir][i].extend), y0), w[dir][i].mask);
				bool check;
				do {
					int x1 = max(x0 / d.l[layer].gs + w[dir][i].x1, 0), x2 = min(x0 / d.l[layer].gs + w[dir][i].x2, prob.cols - 1);
					int y1 = max(y0 / d.l[layer].gs + w[dir][i].y1, 0), y2 = min(y0 / d.l[layer].gs + w[dir][i].y2, prob.rows - 1);
					check = false;
					for (int xx = x1; xx <= x2; xx++)
					for (int yy = y1; yy <= y2; yy++) {  //search right for hotline
						unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
						if (PROB_TYPESHAPE(p_prob2[2 * xx]) == w[dir][i].type_shape) {
							if (abs(PROB_X(p_prob2[2 * xx]) - x0) <= w[dir][i].cx &&
								abs(PROB_Y(p_prob2[2 * xx]) - y0) <= w[dir][i].cy) {
								if (mark.at<unsigned char>(yy, xx) & 2) {
									qWarning("hotpoint2fine_search_mask intersect at dir=1 (x=%d,y=%d) for (x0=%d,y0=%d), maybe cwide=%d too big",
										PROB_X(p_prob2[2 * xx]), PROB_Y(p_prob2[2 * xx]), x0, y0, w[dir][i].cy);
								}
								mark.at<unsigned char>(yy, xx) |= 2;
								check = true;
								int nx0 = PROB_X(p_prob2[2 * xx]), ny0 = PROB_Y(p_prob2[2 * xx]);
								mark_line(mask, Point(x0, y0), Point(nx0, ny0), w[dir][i].mask);
								x0 = nx0, y0 = ny0;
								xx = x2, yy = y2;
							}
						}
					}
				} while (check);
				mark_line(mask, Point(x0, y0), Point(min(mask.cols - 1, x0 + w[dir][i].extend), y0), w[dir][i].mask);
			}
		}
	}

	if (cpara.method & OPT_DEBUG_EN) {
		Mat debug_draw = img.clone();
		for (int y = 0; y < mask.rows; y++) {
			int * p_mask = mask.ptr<int>(y);
			unsigned char * p_img = debug_draw.ptr<unsigned char>(y);
			for (int x = 0; x < mask.cols; x++)
			if (p_mask[x])
				p_img[x] = 0xff;
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_hotline.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw;
		}
	}
}

/*
31..24  23..16   15..8  7..0
opt0:			search_opt      wnum
opt1:			subtype  type  pattern
opt2:			subtype  type  pattern
opt3:			subtype  type  pattern
opt4:			subtype  type  pattern
search_opt is
FINE_LINE_SEARCH_CLEAR_PROB    clear coarse prob to 0xffffff00ffffffff
FINE_LINE_SEARCH_CLEAR_COLOR   clip color to [0, gm - gi]
FINE_LINE_SEARCH_NO_VIA        no via Mat
FILE_LINE_SEARCH_CHECK_VIA_WIRE_CONNECT	check if via is connect to wire
wnum is wire type num
method_opt
0: for gray level turn_points  input
1: for search mask input
2: for remove via mask input
*/
static void fine_line_search(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);

	int idx = cpara.method_opt & 0xf;
	if (d.l[layer].v[idx].type != TYPE_GRAY_LEVEL) {
		qCritical("fine_line_search gray level idx[%d]=%d, error", idx, d.l[layer].v[idx].type);
		return;
	}
	int idx1 = cpara.method_opt >> 4 & 0xf;
	if (d.l[layer].v[idx1].type != TYPE_FINE_WIRE_MASK) {
		qCritical("fine_line_search mask idx1[%d]=%d, error", idx1, d.l[layer].v[idx1].type);
		return;
	}
	Mat mask = d.l[layer].v[idx1].d.clone();
	Mat via_mask;
	Mat already_search(mask.rows, mask.cols, CV_32SC1);
	already_search = Scalar::all(0);

	int search_opt = cpara.opt0 >> 16 & 0xff;
	if ((search_opt & FINE_LINE_SEARCH_NO_VIA) == 0) {
		int idx2 = cpara.method_opt >> 8 & 0xf;
		if (d.l[layer].v[idx2].type != TYPE_REMOVE_VIA_MASK) {
			qCritical("fine_line_search mask idx2[%d]=%d, error", idx2, d.l[layer].v[idx2].type);
			return;
		}
		via_mask = d.l[layer].v[idx2].d;
	}
	else {
		via_mask.create(mask.rows, mask.cols, CV_8UC1);
		via_mask = Scalar::all(0);
	}
	if (mask.rows != img.rows || mask.cols != img.cols) {
		qCritical("fine_line_search, mask.size(%d,%d)!=img.size(%d,%d)", mask.rows, mask.cols, img.rows, img.cols);
		return;
	}
	if (mask.rows != via_mask.rows || mask.cols != via_mask.cols) {
		qCritical("fine_line_search, mask.size(%d,%d)!=via_mask.size(%d,%d)", mask.rows, mask.cols, via_mask.rows, via_mask.cols);
		return;
	}
	if (mask.type() != CV_32SC1 || via_mask.type() != CV_8UC1) {
		qCritical("fine_line_search, mask.type(%d)!=%d, or via_mask.type(%d)!=%d", mask.type(), CV_32SC1, via_mask.type(), CV_8UC1);
		return;
	}
	int gi, gm;
	gi = find_index(d.l[layer].v[idx].d, (int)GRAY_L0);
	gi = d.l[layer].v[idx].d.at<int>(gi, 2);
	gm = find_index(d.l[layer].v[idx].d, (int)GRAY_M0);
	gm = d.l[layer].v[idx].d.at<int>(gm, 2);
	int wnum = cpara.opt0 & 0xff;

	qInfo("fine_line_search, l=%d, wnum=%d, gi=%d, gm=%d, cr_prob=%d", layer, wnum, gi, gm, search_opt);

	if (search_opt & FINE_LINE_SEARCH_CLEAR_PROB) { //clear prob if needed		
		if ((search_opt & FINE_LINE_SEARCH_NO_VIA) == 0) { //following fill via_info to prob
			vector<ViaInfo> & via_info = d.l[layer].via_info;
			for (int i = 0; i < (int)via_info.size(); i++) {
				int type = via_info[i].type;
				int x = via_info[i].xy.x;
				int y = via_info[i].xy.y;
				int y0 = y / d.l[layer].gs, x0 = x / d.l[layer].gs;
				unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y0, x0);
				//mark BRICK_VIA with via info, via prob may be cleared by remove_via & coarse_line_search				
				p_prob[0] = MAKE_PROB(MAKE_S(1, type, BRICK_VIA), x, y);
				p_prob[1] = p_prob[0];
			}
		}
		for (int y = 0; y < d.l[layer].prob.rows; y++) { //clear d.l[layer].prob
			unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y);
			for (int x = 0; x < d.l[layer].prob.cols * 2; x += 2)
			if (PROB_SCORE(p_prob[x]) >= MIN_SCORE) { //if it is not reserved, Mark as NO_WIRE
				p_prob[x] = 0xffffff00ffffffffULL;
				SET_PROB_X(p_prob[x], x / 2 * d.l[layer].gs + 1);
				SET_PROB_Y(p_prob[x], y*d.l[layer].gs + 1);
				p_prob[x + 1] = 0xffffff00ffffffffULL;
				SET_PROB_X(p_prob[x + 1], x / 2 * d.l[layer].gs + 1);
				SET_PROB_Y(p_prob[x + 1], y*d.l[layer].gs + 1);
			}
			else { //add reserved to via_mask
				Point p0(x / 2 * d.l[layer].gs, y * d.l[layer].gs);
				Point p1 = p0 + Point(d.l[layer].gs - 1, d.l[layer].gs - 1);
				mark_rect(via_mask, p0, p1, (char)1);
			}
		}
		//now d.l[layer].prob contain only 0xffffff00ffffffff or Reserved prob(<MIN_SCORE, normally it is via or via nearby)
		//Following mark prob within via_mask as Reserved if it is not reserved
		for (int y = d.l[layer].compute_border; y < img.rows - d.l[layer].compute_border; y++) {
			unsigned char * p_via_mask = via_mask.ptr<unsigned char>(y);
			for (int x = d.l[layer].compute_border; x < img.cols - d.l[layer].compute_border; x++)
			if (p_via_mask[x])
				//normally reserved score is less than 9, so we push 9 will not overwrite existing value
				push_new_prob(d.l[layer].prob, x, y, 0x0009ffff, d.l[layer].gs);
		}

	}

	if (search_opt & FINE_LINE_SEARCH_CLEAR_COLOR) {
		qInfo("fine_line_search, clear color, gi=%d, gm=%d", gi, gm);
		clip_img(img, gi, gm, img);
		/*
		for (int y = 0; y < img.rows; y++) {
		unsigned char * p_img = img.ptr<unsigned char>(y);
		for (int x = 0; x < img.cols; x++)
		p_img[x] = (p_img[x] > gm) ? gm - gi : ((p_img[x] < gi) ? 0 : p_img[x] - gi);
		}*/
		for (int i = 0; i < d.l[layer].v[idx].d.rows; i++) {
			int color = d.l[layer].v[idx].d.at<int>(i, 2);
			color = (color > gm) ? gm - gi : ((color < gi) ? 0 : color - gi);
			d.l[layer].v[idx].d.at<int>(i, 2) = color;
		}
		d.l[layer].ig_valid = false;
	}

#define MAX_WIRE_NUM 4
	if (wnum > MAX_WIRE_NUM) {
		qCritical("fine_line_search, wnum too big");
		return;
	}
	int w_type[MAX_WIRE_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt3 >> 8 & 0xff, cpara.opt4 >> 8 & 0xff };
	int w_subtype[MAX_WIRE_NUM] = { cpara.opt1 >> 16 & 0xff, cpara.opt2 >> 16 & 0xff, cpara.opt3 >> 16 & 0xff, cpara.opt4 >> 16 & 0xff };
	int w_pattern[MAX_WIRE_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff, cpara.opt4 & 0xff };
	int w_guard[MAX_WIRE_NUM];
	QScopedPointer<WireComputeScore> wcs[MAX_WIRE_NUM];

	for (int i = 0; i < wnum; i++) {
		WireParameter wire_para;
		VWParameter * vw = d.l[layer].vw.get_vw_para(w_pattern[i], w_type[i], w_subtype[i]);
		if (vw == NULL) {
			qCritical("fine_line_search invalid wire info pattern=%d, type=%d, subtype=%d", w_pattern[i], w_type[i], w_subtype[i]);
			return;
		}
		wire_para = vw->w;
		wire_para.gray_i = 0;
		wire_para.gray_w = gm - gi;
		wcs[i].reset(WireComputeScore::create_wire_compute_score(wire_para, d, layer));
		w_guard[i] = wire_para.guard;
	}

	Mat new_prob0(d.l[layer].prob.rows, d.l[layer].prob.cols, CV_8UC1);
	int mark_num = 100;
	int loop_num = 0;

	Mat prob1 = d.l[layer].prob.clone(); //it contain full brick prob
	while (mark_num != 0 && loop_num < 5) {
		new_prob0 = Scalar::all(0);
		//1 update prob1 with via_mask
		for (int y = d.l[layer].compute_border; y < img.rows - d.l[layer].compute_border; y++) {
			unsigned * p_mask = mask.ptr<unsigned>(y);
			unsigned char * p_via_mask = via_mask.ptr<unsigned char>(y);
			for (int x = d.l[layer].compute_border; x < img.cols - d.l[layer].compute_border; x++)
			if (p_mask[x] && !(p_via_mask[x] & 1)) {
				unsigned long long score4fake_via = 0xffffffffffffffff;
				//compute prob for each wire patten
				for (int i = 0; i < wnum; i++)
				if (!wcs[i].isNull() && (p_mask[x] >> w_type[i] & 1)) {
					PAIR_ULL score = wcs[i]->compute(x, y, img, d.l[layer].ig, d.l[layer].iig);
					SET_PROB_TYPE(score.first, i);
					SET_PROB_TYPE(score.second, i);
					if (PROB_SHAPE(score.first) != BRICK_FAKE_VIA)
						push_new_prob(prob1, score.first, d.l[layer].gs);
					else
						score4fake_via = min(score4fake_via, score.first);
					if (PROB_SHAPE(score.second) != BRICK_FAKE_VIA)
						push_new_prob(prob1, score.second, d.l[layer].gs);
				}
				//now check all BRICK_II, here BRICK_II BRICK_III is BRICK_FAKE_VIA
				unsigned long long * score_min = get_prob(prob1, x, y, d.l[layer].gs);
				if (PROB_SHAPE(score_min[0]) == BRICK_FAKE_VIA && score_min[0] > score4fake_via) //replace BRICK_II
					score_min[0] = score4fake_via;
				if (score_min[0] > score4fake_via && PROB_SHAPE(score_min[0]) != BRICK_FAKE_VIA
					&& PROB_TYPE(score_min[0]) == PROB_TYPE(score4fake_via)) {//BRICK_II can only take effect for same type BRICK_I
					push_new_prob(prob1, score4fake_via, d.l[layer].gs);
				}
				if (PROB_SHAPE(score_min[0]) == BRICK_FAKE_VIA && PROB_SHAPE(score_min[1]) != BRICK_FAKE_VIA //BRICK_II can only take effect for same type
					&& PROB_TYPE(score_min[0]) != PROB_TYPE(score_min[1])) {
					swap(score_min[0], score_min[1]);
					SET_PROB_SCORE(score_min[1], PROB_SCORE(score_min[0]) + 1);
				}
			}
		}

		//2 update d.l.prob by filtering prob1
		for (int y = 0; y < prob1.rows; y++) {
			unsigned long long * p_prob1 = prob1.ptr<unsigned long long>(y);
			unsigned long long * p_prob0 = d.l[layer].prob.ptr<unsigned long long>(y);
			unsigned char * p_new_prob0 = new_prob0.ptr<unsigned char>(y);
			for (int x = 0; x < prob1.cols * 2; x += 2) {
				int prob1_shape = PROB_SHAPE(p_prob1[x]);
				if (PROB_SCORE(p_prob0[x]) >= MIN_SCORE && PROB_S(p_prob1[x]) != 0xffffff00 &&
					prob1_shape != BRICK_HOLLOW && prob1_shape != BRICK_ONE_POINT && prob1_shape != BRICK_INVALID) { //following filter unique
					CV_Assert(PROB_SHAPE(p_prob1[x]) <= BRICK_IN_USE || PROB_SHAPE(p_prob1[x]) == BRICK_FAKE_VIA);
					bool pass = true;
					int idx = PROB_TYPE(p_prob1[x]);
					int cr = w_guard[idx];
					int guard = (cr - 1) / d.l[layer].gs + 1;
					int y1 = max(y - guard, 0), y2 = min(y + guard, prob1.rows - 1);
					int x1 = max(x / 2 - guard, 0), x2 = min(x / 2 + guard, prob1.cols - 1);
					//2.1 check unique for brick i, T, L, j, l; NO_WIRE; 
					if (prob1_shape != BRICK_I_0 && prob1_shape != BRICK_I_90 &&
						prob1_shape != BRICK_Z_0 && prob1_shape != BRICK_Z_90 &&
						prob1_shape != BRICK_FAKE_VIA) {
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW
								&& PROB_SHAPE(p_prob2[xx]) != BRICK_INVALID) { //Unique condition is it is minimal than nearyby brick
								if (PROB_SHAPE(p_prob2[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								int dx = PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x]);
								int dy = PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x]);
								if (abs(dx) <= cr && abs(dy) <= cr) {
									if (PROB_SHAPE(p_prob2[xx]) == BRICK_I_0 && abs(dx) <= 2) { //if BRICK_I_0 fits bypass
										if (dy < 0 && brick_conn.fit(DIR_UP, prob1_shape, BRICK_I_0) ||
											dy > 0 && brick_conn.fit(DIR_DOWN, prob1_shape, BRICK_I_0))
											continue;
									}
									if (PROB_SHAPE(p_prob2[xx]) == BRICK_I_90 && abs(dy) <= 2) { //if BRICK_I_90 fits bypass
										if (dx < 0 && brick_conn.fit(DIR_LEFT, prob1_shape, BRICK_I_90) ||
											dx > 0 && brick_conn.fit(DIR_RIGHT, prob1_shape, BRICK_I_90))
											continue;
									}
									if (PROB_SHAPE(p_prob2[xx]) == BRICK_Z_0 && abs(dx + dy) <= 2) { //if BRICK_Z_0 fits bypass
										if (dy < 0 && brick_conn.fit(DIR_UPRIGHT, prob1_shape, BRICK_Z_0) ||
											dy > 0 && brick_conn.fit(DIR_DOWNLEFT, prob1_shape, BRICK_Z_0))
											continue;
									}
									if (PROB_SHAPE(p_prob2[xx]) == BRICK_Z_90 && abs(dx - dy) <= 2) { //if BRICK_Z_90 fits bypass
										if (dy < 0 && brick_conn.fit(DIR_UPLEFT, prob1_shape, BRICK_Z_90) ||
											dy > 0 && brick_conn.fit(DIR_DOWNRIGHT, prob1_shape, BRICK_Z_90))
											continue;
									}
									//else it is not unique
									pass = false;
									yy = y2;
									break;
								}
							}
						}
					}
					//2.2 check unique for BRICK_I_0
					if (prob1_shape == BRICK_I_0) {
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW
								&& PROB_SHAPE(p_prob2[xx]) != BRICK_INVALID) { //Unique condition is it is minimal than nearyby non BRICK_I_O brick
								if (PROB_SHAPE(p_prob2[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								int dx = PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x]);
								int dy = PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x]);
								if (abs(dx) <= cr && abs(dy) <= cr) {
									if (yy == y && PROB_SHAPE(p_prob2[xx]) == BRICK_I_0) { //not allow two BRICK_I_0 exist at same row
										pass = false;
										yy = y2;
										break;
									}
									if (abs(dx) <= 2) { //if BRICK_I_0 fits bypass
										if (dy < 0 && brick_conn.fit(DIR_UP, prob1_shape, PROB_SHAPE(p_prob2[xx])) ||
											dy > 0 && brick_conn.fit(DIR_DOWN, prob1_shape, PROB_SHAPE(p_prob2[xx])))
											continue;
									}
									pass = false;
									yy = y2;
									break;
								}
							}
						}
					}
					//2.3 check unique for BRICK_I_90
					if (prob1_shape == BRICK_I_90) {
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW
								&& PROB_SHAPE(p_prob2[xx]) != BRICK_INVALID) { //Unique condition is it is minimal than nearyby non BRICK_I_9O brick
								if (PROB_SHAPE(p_prob2[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								int dx = PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x]);
								int dy = PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x]);
								if (abs(dx) <= cr && abs(dy) <= cr) {
									if (xx == x && PROB_SHAPE(p_prob2[xx]) == BRICK_I_90) { //not allow two BRICK_I_90 exist at same col
										pass = false;
										yy = y2;
										break;
									}
									if (abs(dy) <= 2) { //if BRICK_I_0 fits bypass
										if (dx < 0 && brick_conn.fit(DIR_LEFT, prob1_shape, PROB_SHAPE(p_prob2[xx])) ||
											dx > 0 && brick_conn.fit(DIR_RIGHT, prob1_shape, PROB_SHAPE(p_prob2[xx])))
											continue;
									}
									pass = false;
									yy = y2;
									break;
								}
							}
						}
					}
					//2.4 check unique for BRICK_Z_0
					if (prob1_shape == BRICK_Z_0) {
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW
								&& PROB_SHAPE(p_prob2[xx]) != BRICK_INVALID) { //Unique condition is it is minimal than nearyby non BRICK_I_9O brick
								if (PROB_SHAPE(p_prob2[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								int dx = PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x]);
								int dy = PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x]);
								if (abs(dx) <= cr && abs(dy) <= cr) {
									if (yy + xx == y + x && PROB_SHAPE(p_prob2[xx]) == BRICK_Z_0) { //not allow two BRICK_Z_0 exist at same /
										pass = false;
										yy = y2;
										break;
									}
									if (abs(dx + dy) <= 2) { //if BRICK_Z_0 fits bypass
										if (dy < 0 && brick_conn.fit(DIR_UPRIGHT, prob1_shape, PROB_SHAPE(p_prob2[xx])) ||
											dy > 0 && brick_conn.fit(DIR_DOWNLEFT, prob1_shape, PROB_SHAPE(p_prob2[xx])))
											continue;
									}
									pass = false;
									yy = y2;
									break;
								}
							}
						}
					}
					//2.5 check unique for BRICK_Z_90
					if (prob1_shape == BRICK_Z_90) {
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW
								&& PROB_SHAPE(p_prob2[xx]) != BRICK_INVALID) { //Unique condition is it is minimal than nearyby non BRICK_I_9O brick
								if (PROB_SHAPE(p_prob2[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								int dx = PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x]);
								int dy = PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x]);
								if (abs(dx) <= cr && abs(dy) <= cr) {
									if (yy - xx == y - x && PROB_SHAPE(p_prob2[xx]) == BRICK_Z_90) { //not allow two BRICK_Z_0 exist at same /
										pass = false;
										yy = y2;
										break;
									}
									if (abs(dx - dy) <= 2) { //if BRICK_Z_90 fits bypass
										if (dy < 0 && brick_conn.fit(DIR_UPLEFT, prob1_shape, PROB_SHAPE(p_prob2[xx])) ||
											dy > 0 && brick_conn.fit(DIR_DOWNRIGHT, prob1_shape, PROB_SHAPE(p_prob2[xx])))
											continue;
									}
									pass = false;
									yy = y2;
									break;
								}
							}
						}
					}
					//2.6 check unique for brick ii; FAKE_VIA;
					if (prob1_shape == BRICK_FAKE_VIA) {
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2) {
								if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW && PROB_SHAPE(p_prob2[xx]) != BRICK_INVALID ||
									PROB_SHAPE(p_prob2[xx]) <= BRICK_IN_USE && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]) && PROB_TYPE(p_prob2[xx]) != 0xff) { //Unique condition is it is minimal than nearyby brick
									if (abs(PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x])) <= cr &&
										abs(PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x])) <= cr) {
										pass = false;
										yy = y2;
										break;
									}
								}
							}
						}
					}

					if (pass) {//if unique, copy to d.l[layer].prob,
						p_new_prob0[x / 2] = (PROB_SHAPE(p_prob0[x]) != prob1_shape);
						p_prob0[x] = p_prob1[x];
					}
					else { //if it is not unique, mark self as BRICK_INVALID
						p_prob0[x] = p_prob1[x];
						SET_PROB_SHAPE(p_prob0[x], BRICK_INVALID);
					}
				}
				if (PROB_SCORE(p_prob0[x]) > MIN_SCORE) {
					if (PROB_SHAPE(p_prob1[x]) == BRICK_ONE_POINT || PROB_SHAPE(p_prob1[x]) == BRICK_INVALID) {
						//if it is OnePoint, mark as Invalid
						p_prob0[x] = p_prob1[x];
						SET_PROB_SHAPE(p_prob0[x], BRICK_INVALID);
					}
					if (PROB_SHAPE(p_prob1[x]) == BRICK_HOLLOW) {//if it is Hollow, mark it as NoWire
						p_prob0[x] = p_prob1[x];
						SET_PROB_SHAPE(p_prob0[x], BRICK_NO_WIRE);
					}
				}
			}
		}
		Mat & prob = d.l[layer].prob;
		already_search = already_search | mask;
		mask = Scalar::all(0);
		mark_num = 0;
		for (int y = 0; y < prob.rows; y++) {
			unsigned char * p_new_prob0 = new_prob0.ptr<unsigned char>(y);
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols; x++)
			if (p_new_prob0[x]) {
				unsigned long long prob0 = p_prob[2 * x];
				CV_Assert(PROB_TYPE(prob0) < MAX_WIRE_NUM);
				mark_num += wcs[PROB_TYPE(prob0)]->check_mark(x, y, d.l[layer].prob, mask, w_type[PROB_TYPE(prob0)], already_search);
			}
		}
		qDebug("new mark_num=%d", mark_num);
		loop_num++;
	}

	if (search_opt & FILE_LINE_SEARCH_CHECK_VIA_WIRE_CONNECT) {
		for (int i = 0; i < wnum; i++)
		if (!wcs[i].isNull()) {
			wcs[i]->check_via_wire_connect(via_mask, d.l[layer].prob, img, d.l[layer].ig);
		}
	}

	for (int y = 0; y < d.l[layer].prob.rows; y++) {
		unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y);
		for (int x = 0; x < d.l[layer].prob.cols * 2; x += 2) {
			unsigned long long prob0 = p_prob[x];
			if (PROB_S(prob0) != 0xffffff00 && PROB_SCORE(prob0) >= MIN_SCORE)
			if (PROB_SHAPE(prob0) <= BRICK_IN_USE || PROB_SHAPE(prob0) == BRICK_FAKE_VIA) {
				CV_Assert(PROB_TYPE(prob0) < MAX_WIRE_NUM);
				int idx = PROB_TYPE(prob0);
				int cr = w_guard[idx];
				int guard = (cr - 1) / d.l[layer].gs + 1;
				int y1 = max(y - guard, 0), y2 = min(y + guard, prob1.rows - 1);
				int x1 = max(x / 2 - guard, 0), x2 = min(x / 2 + guard, prob1.cols - 1);
				for (int yy = y1; yy <= y2; yy++) { //mark nearby NOWIRE brick BRICK_INVALID to avoid BRICK connect BRICK_NOWIRE
					unsigned long long * p_prob2 = d.l[layer].prob.ptr<unsigned long long>(yy);
					for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
					if (PROB_S(p_prob2[xx]) == 0xffffff00)
						SET_PROB_SHAPE(p_prob2[xx], BRICK_INVALID);
				}
				SET_PROB_TYPE(prob0, w_type[idx]);
				p_prob[x] = prob0;
			}
			if (PROB_SCORE(prob0) < MIN_SCORE && (PROB_SHAPE(prob0) <= BRICK_IN_USE || PROB_SHAPE(prob0) == BRICK_FAKE_VIA)) {
				int guard = 2;
				int y1 = max(y - guard, 0), y2 = min(y + guard, prob1.rows - 1);
				int x1 = max(x / 2 - guard, 0), x2 = min(x / 2 + guard, prob1.cols - 1);
				for (int yy = y1; yy <= y2; yy++) { //mark nearby NOWIRE brick BRICK_INVALID to avoid BRICK connect BRICK_NOWIRE
					unsigned long long * p_prob2 = d.l[layer].prob.ptr<unsigned long long>(yy);
					for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
					if (PROB_S(p_prob2[xx]) == 0xffffff00)
						SET_PROB_SHAPE(p_prob2[xx], BRICK_INVALID);
				}
			}
		}
	}

#undef MAX_WIRE_NUM

	if (cpara.method & OPT_DEBUG_EN) {
		d.l[layer].check_prob();
		Mat debug_draw;
		for (int debug_out = 0; debug_out < 2; debug_out++) {
			debug_draw = img.clone();
			for (int y = 0; y < prob1.rows; y++) {
				unsigned long long * p_prob = (debug_out == 0) ? prob1.ptr<unsigned long long>(y) : d.l[layer].prob.ptr<unsigned long long>(y);
				for (int x = 0; x < prob1.cols; x++) {
					int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]);
					int shape = PROB_SHAPE(p_prob[2 * x]);
					if (shape <= BRICK_IN_USE) {
						for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
						if (bricks[shape].a[i][j])
							debug_draw.at<unsigned char>(y0 + i - 1, x0 + j - 1) = 255;
					}
					else
					if (shape == BRICK_VIA || shape == BRICK_FAKE_VIA) {
						for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							debug_draw.at<unsigned char>(y0 + i - 1, x0 + j - 1) = 255;
					}
				}
			}
			if (debug_out == 0)
				imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_fine_line_search_prob1.jpg", debug_draw);
			else
				imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_fine_line_search_prob0.jpg", debug_draw);
		}
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw.clone();
		}
	}
}

struct SkinPoint {
	Point xy; //offset to branch
	int dir_mask;
	int len;
	int check;//check is true for skin point, false for bone and branch point
	SkinPoint(const Point & _xy, int _dir_mask, bool _check) {
		xy = _xy;
		dir_mask = _dir_mask;
		check = _check ? 1 : 0;
		len = abs(xy.x) + abs(xy.y) + ((xy.x == 0 || xy.y == 0) ? 0 : 2);
	}
};

class SearchWireBranch {
	int offset[4][4];
	vector<SkinPoint> skps;
	int prev_dir, prev_wide;
	const WireParameter2 * prev_wp;
	int th[4];
public:
	SearchWireBranch() {
		prev_wp = NULL;
	}

	/*
	Input pt1, start point
	Input pt2, end point
	Input dir, search dir
	Input wide, wire wide
	Input wp, wire parameter
	Input b_idx, branch idx
	Input ig
	output bm
	Return true if [pt1,pt2) are all wire else return false
	*/
	bool search_branch(Point pt1, Point pt2, int dir, int wide, const WireParameter2 * wp, int b_idx, const Mat & ig, BranchMark & bm) {
		int wide1 = max(5, wide - 2);
		if (prev_dir != dir || prev_wide != wide1 || prev_wp != wp) { //reinit skps and offset
			Point oxy[4][2];
			skps.clear();
			oxy[3][0] = Point(-wp->swide_min / 2, -wp->swide_min / 2); //oxy[1] is skin rec
			oxy[3][1] = Point(wp->swide_min - wp->swide_min / 2, wp->swide_min - wp->swide_min / 2);
			int bone_wide_2 = wide1 / 2;
			switch (dir) {
			case DIR_UP:
				oxy[0][0] = Point(-wide1 / 2, -wp->i_high - wp->swide_min / 2); //oxy[0] is wire
				oxy[0][1] = Point(0, -wp->swide_min / 2);
				oxy[1][0] = Point(0, -wp->i_high - wp->swide_min / 2);
				oxy[1][1] = Point(wide1 - wide1 / 2, -wp->swide_min / 2);
				oxy[2][0] = Point(-wide1 / 4, -wp->i_high - wp->swide_min / 2);
				oxy[2][1] = Point(wide1 / 2 - wide1 / 4, -wp->swide_min / 2);
				skps.push_back(SkinPoint(Point(0, 0), DIR_DOWN1_MASK, false));
				for (int i = 1; i <= bone_wide_2; i++) { //push heng line
					skps.push_back(SkinPoint(Point(i, 0), DIR_LEFT1_MASK | DIR_UPLEFT_MASK | DIR_DOWNLEFT_MASK, false));
					skps.push_back(SkinPoint(Point(-i, 0), DIR_RIGHT1_MASK | DIR_UPRIGHT_MASK | DIR_DOWNRIGHT_MASK, false));
				}
				skps.push_back(SkinPoint(Point(bone_wide_2 + 1, 0), DIR_LEFT1_MASK | DIR_UPLEFT_MASK | DIR_DOWNLEFT_MASK, true));
				skps.push_back(SkinPoint(Point(-bone_wide_2 - 1, 0), DIR_RIGHT1_MASK | DIR_UPRIGHT_MASK | DIR_DOWNRIGHT_MASK, true));
				break;
			case DIR_DOWN:
				oxy[0][0] = Point(-wide1 / 2, wp->swide_min / 2);
				oxy[0][1] = Point(0, wp->i_high + wp->swide_min / 2);
				oxy[1][0] = Point(0, wp->swide_min / 2);
				oxy[1][1] = Point(wide1 - wide1 / 2, wp->i_high + wp->swide_min / 2);
				oxy[2][0] = Point(-wide1 / 4, wp->swide_min / 2);
				oxy[2][1] = Point(wide1 / 2 - wide1 / 4, wp->i_high + wp->swide_min / 2);
				skps.push_back(SkinPoint(Point(0, 0), DIR_UP1_MASK, false));
				for (int i = 1; i <= bone_wide_2; i++) { //push heng line
					skps.push_back(SkinPoint(Point(i, 0), DIR_LEFT1_MASK | DIR_UPLEFT_MASK | DIR_DOWNLEFT_MASK, false));
					skps.push_back(SkinPoint(Point(-i, 0), DIR_RIGHT1_MASK | DIR_UPRIGHT_MASK | DIR_DOWNRIGHT_MASK, false));
				}
				skps.push_back(SkinPoint(Point(bone_wide_2 + 1, 0), DIR_LEFT1_MASK | DIR_UPLEFT_MASK | DIR_DOWNLEFT_MASK, true));
				skps.push_back(SkinPoint(Point(-bone_wide_2 - 1, 0), DIR_RIGHT1_MASK | DIR_UPRIGHT_MASK | DIR_DOWNRIGHT_MASK, true));
				break;
			case DIR_RIGHT:
				oxy[0][0] = Point(wp->swide_min / 2, -wide1 / 2);
				oxy[0][1] = Point(wp->i_high + wp->swide_min / 2, 0);
				oxy[1][0] = Point(wp->swide_min / 2, 0);
				oxy[1][1] = Point(wp->i_high + wp->swide_min / 2, wide1 - wide1 / 2);
				oxy[2][0] = Point(wp->swide_min / 2, -wide1 / 4);
				oxy[2][1] = Point(wp->i_high + wp->swide_min / 2, wide1 / 2 - wide1 / 4);
				skps.push_back(SkinPoint(Point(0, 0), DIR_LEFT1_MASK, false));
				for (int i = 1; i <= bone_wide_2; i++) { //push shu line
					skps.push_back(SkinPoint(Point(0, i), DIR_UP1_MASK | DIR_UPLEFT_MASK | DIR_UPRIGHT_MASK, false));
					skps.push_back(SkinPoint(Point(0, -i), DIR_DOWN1_MASK | DIR_DOWNLEFT_MASK | DIR_DOWNRIGHT_MASK, false));
				}
				skps.push_back(SkinPoint(Point(0, bone_wide_2 + 1), DIR_UP1_MASK | DIR_UPLEFT_MASK | DIR_UPRIGHT_MASK, true));
				skps.push_back(SkinPoint(Point(0, -bone_wide_2 - 1), DIR_DOWN1_MASK | DIR_DOWNLEFT_MASK | DIR_DOWNRIGHT_MASK, true));
				break;
			case DIR_LEFT:
				oxy[0][0] = Point(-wp->i_high - wp->swide_min / 2, -wide1 / 2);
				oxy[0][1] = Point(-wp->swide_min / 2, 0);
				oxy[1][0] = Point(-wp->i_high - wp->swide_min / 2, 0);
				oxy[1][1] = Point(-wp->swide_min / 2, wide1 - wide1 / 2);
				oxy[2][0] = Point(-wp->i_high - wp->swide_min / 2, -wide1 / 4);
				oxy[2][1] = Point(-wp->swide_min / 2, wide1 / 2 - wide1 / 4);
				skps.push_back(SkinPoint(Point(0, 0), DIR_RIGHT1_MASK, false));
				for (int i = 1; i <= bone_wide_2; i++) { //push shu line
					skps.push_back(SkinPoint(Point(0, i), DIR_UP1_MASK | DIR_UPLEFT_MASK | DIR_UPRIGHT_MASK, false));
					skps.push_back(SkinPoint(Point(0, -i), DIR_DOWN1_MASK | DIR_DOWNLEFT_MASK | DIR_DOWNRIGHT_MASK, false));
				}
				skps.push_back(SkinPoint(Point(0, bone_wide_2 + 1), DIR_UP1_MASK | DIR_UPLEFT_MASK | DIR_UPRIGHT_MASK, true));
				skps.push_back(SkinPoint(Point(0, -bone_wide_2 - 1), DIR_DOWN1_MASK | DIR_DOWNLEFT_MASK | DIR_DOWNRIGHT_MASK, true));
				break;
			}
			for (int i = 0; i < 4; i++) {
				offset[i][0] = (oxy[i][0].y * (int)ig.step.p[0] + oxy[i][0].x * (int)ig.step.p[1]) / sizeof(int);
				offset[i][1] = (oxy[i][0].y * (int)ig.step.p[0] + oxy[i][1].x * (int)ig.step.p[1]) / sizeof(int);
				offset[i][2] = (oxy[i][1].y * (int)ig.step.p[0] + oxy[i][0].x * (int)ig.step.p[1]) / sizeof(int);
				offset[i][3] = (oxy[i][1].y * (int)ig.step.p[0] + oxy[i][1].x * (int)ig.step.p[1]) / sizeof(int);
			}
			prev_dir = dir;
			prev_wide = wide1;
			prev_wp = wp;
			th[0] = wp->i_high * (wide1 / 2) * wp->gray_i + wp->i_high * (wide1 / 2) * (wp->gray_w - wp->gray_i) * (wp->mth_low / 100.0); //th0 is wire threshold
			th[1] = wp->i_high * (wide1 - wide1 / 2) * wp->gray_i + wp->i_high * (wide1 - wide1 / 2) * (wp->gray_w - wp->gray_i) * (wp->mth_low / 100.0); //th0 is wire threshold
			th[2] = th[0];
			th[3] = wp->swide_min * wp->swide_min* wp->gray_i + wp->swide_min * wp->swide_min *(wp->gray_w - wp->gray_i) * (wp->s_th / 100.0); //th1 is skin threshold 
		}

		vector<Point> pts;
		get_line_pts(pt1, pt2, pts);
		const Mat & mark = bm.get_mark();
		for (int i = 0; i < (int)pts.size(); i++) {
			const unsigned * p_ig = ig.ptr<unsigned>(pts[i].y, pts[i].x);
			int sum0 = 0, sum1 = 0, sum2 = 0;
			bool inside = pts[i].x > wp->i_high + wp->swide_min / 2 && pts[i].y > wp->i_high + wp->swide_min / 2 &&
				pts[i].x > wp->swide_min && pts[i].y > wp->swide_min && pts[i].x < ig.cols - wp->swide_min && pts[i].y < ig.rows - wp->swide_min &&
				pts[i].x < ig.cols - wp->i_high - wp->swide_min / 2 && pts[i].y < ig.rows - wp->i_high - wp->swide_min / 2;
			bool meet_parallel = false;
			if (inside) {
				sum0 = p_ig[offset[0][3]] + p_ig[offset[0][0]] - p_ig[offset[0][1]] - p_ig[offset[0][2]];
				sum1 = p_ig[offset[1][3]] + p_ig[offset[1][0]] - p_ig[offset[1][1]] - p_ig[offset[1][2]];
				sum2 = p_ig[offset[2][3]] + p_ig[offset[2][0]] - p_ig[offset[2][1]] - p_ig[offset[2][2]];
				const unsigned long long * p_mark = mark.ptr<unsigned long long>(pts[i].y, pts[i].x);
				if (p_mark[0] != 0xffffffffffffffffULL) { //meet another branch
					int ano_bra = MARK_BRANCH(p_mark[0]);
					int ano_dir = bm.bs[ano_bra].get_dir();
					if (ano_bra != b_idx && (dir == ano_dir || dir_1[dir] == ano_dir)) //parallel, push end
						meet_parallel = true;
				}
			}
			if (sum0 >= th[0] && sum1 >= th[1] && sum2 >= th[2] && inside && !meet_parallel) { //wire continue, push wire
				for (int j = 0; j < (int)skps.size(); j++) {
					Point skin_pt = skps[j].xy + pts[i];
					if (!skps[j].check) //push bone
						bm.push_mark(skin_pt, b_idx, skps[j].len, skps[j].dir_mask, 0, pts[i].x, pts[i].y, false);
					else {
						p_ig = ig.ptr<unsigned>(skin_pt.y, skin_pt.x);
						int sum3 = p_ig[offset[3][3]] + p_ig[offset[3][0]] - p_ig[offset[3][1]] - p_ig[offset[3][2]];
						if (sum3 >= th[3]) //push skin
							bm.push_mark(skin_pt, b_idx, skps[j].len, skps[j].dir_mask, 1, pts[i].x, pts[i].y, false);
					}
				}
				bm.bs[b_idx].push_xy(pts[i]);
			}
			else { //wire break, push wire end
				for (int k = 0; k < wp->swide_min / 2; k++) {
					Point extend_pt = Point(pts[i].x + dxy[dir][1] * k, pts[i].y + dxy[dir][0] * k);
					for (int j = 0; j < (int)skps.size(); j++) {
						Point skin_pt = extend_pt + skps[j].xy;
						if (!skps[j].check && k != wp->swide_min / 2 - 1) //push bone
							bm.push_mark(skin_pt, b_idx, skps[j].len, skps[j].dir_mask, 0, extend_pt.x, extend_pt.y, false);
						else {
							p_ig = ig.ptr<unsigned>(skin_pt.y, skin_pt.x);
							int sum3 = p_ig[offset[3][3]] + p_ig[offset[3][0]] - p_ig[offset[3][1]] - p_ig[offset[3][2]];
							if (sum3 >= th[3]) // push skin
								bm.push_mark(skin_pt, b_idx, skps[j].len, skps[j].dir_mask, 1, extend_pt.x, extend_pt.y, false);
						}
					}
					bm.bs[b_idx].push_xy(extend_pt);
				}
				return false;
			}
		}
		return true;
	}
};

class SearchViaBranch {
protected:
	const ViaParameter * prev_vp;
	vector<SkinPoint> skps;
	int offset[4];
	int prev_connect_rd, prev_swide_min;

public:
	SearchViaBranch() {
		prev_swide_min = -1;
		prev_vp = NULL;
	}

	/*
	Input: pt1 via center
	Input: vp
	Input: ig
	Inout: bm
	input: down_layer, 0 this layer, 1 downlayer
	*/
	int search_branch(Point pt1, const ViaParameter * vp, const Mat & ig, BranchMark & bm, int down_layer) {
		int th = vp->swide_min * vp->swide_min* vp->gray3 + vp->swide_min * vp->swide_min *(vp->gray0 - vp->gray3) * (vp->cgray_ratio / 100.0); //th1 is skin threshold 
		int bidx = bm.new_branch(vp->type, vp->swide_min, th, down_layer ? -2 : -1);
		if (prev_connect_rd != vp->connect_rd) {
			skps.clear();
			vector<int> flag(vp->connect_rd * 2 + 1, 0);
			for (int y = -vp->connect_rd; y <= 0; y++) {
				int x1 = sqrt(vp->connect_rd * vp->connect_rd - y * y);
				int dir0 = (y<0) ? DIR_DOWN1_MASK : ((y>0) ? DIR_UP1_MASK : 0);
				for (int x = -x1; x <= x1; x++) {
					bool check = (abs(x) == x1); //check is for skin point, true means x-boundary point
					if (!flag[vp->connect_rd + x]) { //check for skin point, true means y-boundary point
						flag[vp->connect_rd + x] = 1;
						check = true;
					}
					int dir = dir0 | ((x<0) ? DIR_RIGHT1_MASK : ((x>0) ? DIR_LEFT1_MASK : 0));
					skps.push_back(SkinPoint(Point(x, y), dir, check));
				}
			}
			fill(flag.begin(), flag.end(), 0);
			for (int y = vp->connect_rd; y > 0; y--) {
				int x1 = sqrt(vp->connect_rd * vp->connect_rd - y * y);
				int dir0 = (y<0) ? DIR_DOWN1_MASK : ((y>0) ? DIR_UP1_MASK : 0);
				for (int x = -x1; x <= x1; x++) {
					bool check = (abs(x) == x1); //check is for skin point
					if (!flag[vp->connect_rd + x]) {
						flag[vp->connect_rd + x] = 1;
						check = true;
					}
					int dir = dir0 | ((x<0) ? DIR_RIGHT1_MASK : ((x>0) ? DIR_LEFT1_MASK : 0));
					skps.push_back(SkinPoint(Point(x, y), dir, check));
				}
			}
			prev_connect_rd = vp->connect_rd;
		}
		if (prev_swide_min != vp->swide_min) {
			Point oxy[2];
			oxy[0] = Point(-vp->swide_min / 2, -vp->swide_min / 2);
			oxy[1] = Point(vp->swide_min - vp->swide_min / 2, vp->swide_min - vp->swide_min / 2);
			offset[0] = (oxy[0].y * (int)ig.step.p[0] + oxy[0].x * (int)ig.step.p[1]) / sizeof(int);
			offset[1] = (oxy[0].y * (int)ig.step.p[0] + oxy[1].x * (int)ig.step.p[1]) / sizeof(int);
			offset[2] = (oxy[1].y * (int)ig.step.p[0] + oxy[0].x * (int)ig.step.p[1]) / sizeof(int);
			offset[3] = (oxy[1].y * (int)ig.step.p[0] + oxy[1].x * (int)ig.step.p[1]) / sizeof(int);
			CV_Assert(abs(offset[0]) < 0x80000 && abs(offset[1]) < 0x80000 && abs(offset[2]) < 0x80000 && abs(offset[3]) < 0x80000);
			prev_swide_min = vp->swide_min;
		}
		bm.bs[bidx].push_xy(pt1);
		for (int j = 0; j < (int)skps.size(); j++) {
			Point skin_pt = skps[j].xy + pt1;
			if (skin_pt.x <= 0 || skin_pt.y <= 0 || skin_pt.x >= ig.cols - 1 || skin_pt.y >= ig.rows - 1)
				continue;
			if (!skps[j].check)  // push bone
				bm.push_mark(skin_pt, bidx, skps[j].len, skps[j].dir_mask, 0, pt1.x, pt1.y, true);

			else {
				if (skin_pt.x <= vp->swide_min / 2 || skin_pt.y <= vp->swide_min / 2 ||
					skin_pt.x >= ig.cols - vp->swide_min / 2 - 2 || skin_pt.y >= ig.rows - vp->swide_min / 2 - 2)
					continue;
				const unsigned * p_ig = ig.ptr<unsigned>(skin_pt.y, skin_pt.x);
				int sum = p_ig[offset[3]] + p_ig[offset[0]] - p_ig[offset[1]] - p_ig[offset[2]];
				if (sum >= th) //push skin
					bm.push_mark(skin_pt, bidx, skps[j].len, skps[j].dir_mask, 1, pt1.x, pt1.y, true);
			}
		}
		return bidx;
	}
};

static int search_wire_branch(Point ps, int dir, int wide, int bidx, const WireParameter2 * wp, WireMaskInfo & wmi, PipeDataPerLayer & d)
{
	if (ps.x <= wp->i_high + wp->swide_min / 2 || ps.y <= wp->i_high + wp->swide_min / 2 ||
		ps.x <= wp->swide_min || ps.y <= wp->swide_min ||
		ps.x <= wide / 2 + wp->swide_min / 2 || ps.y <= wide / 2 + wp->swide_min / 2 ||
		ps.x >= d.img.cols - wp->i_high - wp->swide_min / 2 || ps.y >= d.img.rows - wp->i_high - wp->swide_min / 2 ||
		ps.x >= d.img.cols - wp->swide_min || ps.y >= d.img.rows - wp->swide_min ||
		ps.x >= d.img.cols - wide / 2 - wp->swide_min / 2 || ps.y >= d.img.rows - wide / 2 - wp->swide_min / 2)
		return bidx;

	Mat & ig = d.ig;
	BranchMark & bm = d.bm;
	Mat & prob = d.prob;
	Point pt;
	SearchWireBranch sb;

	int th = wp->swide_min * wp->swide_min * wp->gray_i + wp->swide_min * wp->swide_min * (wp->gray_w - wp->gray_i) * (wp->s_th / 100.0);
	if (bidx<0)
		bidx = d.bm.new_branch(wmi.type_shape >> 8, wp->swide_min, th, dir);
	int oft[3][2]; //oft1 & oft2 is dir two side
	oft[0][0] = 0, oft[0][1] = 0;
	oft[1][0] = dxy[dir_2[dir]][0], oft[1][1] = dxy[dir_2[dir]][1];
	oft[2][0] = dxy[dir_1[dir_2[dir]]][0], oft[2][1] = dxy[dir_1[dir_2[dir]]][1];

	int y0 = ps.y / d.gs, x0 = ps.x / d.gs; //x0, y0 is look forward
	int xy = cnear[dir][0] * ps.y + cnear[dir][1] * ps.x;
	while (1) {
		int type, subtype, pattern, w1 = -1;
		while (1) {
			if (abs(x0*d.gs - ps.x) > wmi.clong || (abs(y0*d.gs - ps.y) > wmi.clong))
				break;
			y0 += dxy[dir][0];
			x0 += dxy[dir][1];
			if (x0 < 0 || y0 < 0 || x0 >= prob.cols || y0 >= prob.rows)
				break;
			for (int j = 0; j < 3; j++) {
				unsigned long long p0 = prob.at<unsigned long long>(y0 + oft[j][0], (x0 + oft[j][1]) * 2);
				if (PROB_SHAPE(p0) == (wmi.type_shape & 0xff)) {
					pt = Point(PROB_X(p0), PROB_Y(p0));
					if (abs(cnear[dir][0] * pt.y + cnear[dir][1] * pt.x - xy) <= wmi.cwide) { //find new anchor
						d.wts.get_all(PROB_TYPE(p0), type, subtype, pattern, w1);
						if (wp->type != type || wp->subtype != subtype || wp->pattern != pattern)
							qCritical("search_branch meet different type old(%d,%d,%d)!=new(%d,%d,%d)",
							wp->type, wp->subtype, wp->pattern, type, subtype, pattern);
						break;
					}
				}
			}
			if (w1 > 0)
				break;
		}
		if (w1 < 0) {//not found new anchor, 
			pt = ps + Point(d.gs * dxy[dir][1], d.gs * dxy[dir][0]);
		}
		if (!sb.search_branch(ps, pt, dir, (w1 > 0) ? min(wide, w1) : wide, wp, bidx, ig, bm))
			return bidx;
		if (w1 > 0) {//find new anchor
			y0 = pt.y / d.gs;
			x0 = pt.x / d.gs;
			xy = cnear[dir][0] * pt.y + cnear[dir][1] * pt.x;
			wide = w1;
		}
		ps = pt;
	}
}

/*
31..24 23..16   15..8   7..0
opt0:		             opt    wnum
opt1:	cwide	clong	 type	dir
opt2:	cwide	clong	 type	dir
opt3:	cwide	clong	 type	dir
opt4:	cwide	clong	 type	dir
opt5:	cwide	clong	 type	dir
opt6:
method_opt
0: for gray level turn_points  input
*/
static void hotpoint_search(PipeData & d, ProcessParameter & cpara)
{
	int idx = cpara.method_opt & 0xf;
	int wnum = cpara.opt0 & 0xff;
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	int hotpoint_opt = cpara.opt0 >> 8 & 0xff;

	qInfo("hotpoint_search, idx=%d, l=%d, wnum=%d, hotline_opt=%d", idx, layer, wnum, hotpoint_opt);

	if (d.l[layer].v[idx].type != TYPE_GRAY_LEVEL) {
		qCritical("hotpoint_search gray level idx[%d]=%d, error", idx, d.l[layer].v[idx].type);
		return;
	}
	if (d.l[layer].bm.get_mark().empty())
		d.l[layer].bm.clear_mark(img);
	int gl = find_index(d.l[layer].v[idx].d, (int)GRAY_L0);
	gl = d.l[layer].v[idx].d.at<int>(gl, 2);
	int gm = find_index(d.l[layer].v[idx].d, (int)GRAY_M0);
	gm = d.l[layer].v[idx].d.at<int>(gm, 2);
	qInfo("hotpoint_search, gl=%d, gm=%d", gl, gm);

	struct WireMaskInfo wpara[] = {
		{ cpara.opt1 & 0xffff, cpara.opt1 >> 16 & 0xff, cpara.opt1 >> 24 & 0xff },
		{ cpara.opt2 & 0xffff, cpara.opt2 >> 16 & 0xff, cpara.opt2 >> 24 & 0xff },
		{ cpara.opt3 & 0xffff, cpara.opt3 >> 16 & 0xff, cpara.opt3 >> 24 & 0xff },
		{ cpara.opt4 & 0xffff, cpara.opt4 >> 16 & 0xff, cpara.opt4 >> 24 & 0xff },
		{ cpara.opt5 & 0xffff, cpara.opt5 >> 16 & 0xff, cpara.opt5 >> 24 & 0xff }
	};
	vector <WireMaskInfo> w[2];
	for (int i = 0; i < wnum; i++) { //unpack WireMaskInfo dir
		qInfo("hotpoint_search w%d:type=%d, dir=%d, clong=%d, cwide=%d", i, wpara[i].type_shape >> 8, wpara[i].type_shape & 0xff,
			wpara[i].clong, wpara[i].cwide);
		int dir = wpara[i].type_shape & 0xff;
		if (dir & DIR_UP1_MASK) {
			wpara[i].type_shape &= 0xff00;
			wpara[i].type_shape |= BRICK_I_0;
			w[0].push_back(wpara[i]);
		}
		if (dir & DIR_RIGHT1_MASK) {
			wpara[i].type_shape &= 0xff00;
			wpara[i].type_shape |= BRICK_I_90;
			w[1].push_back(wpara[i]);
		}
	}

	Mat & prob = d.l[layer].prob;
	d.l[layer].validate_ig();
	Mat mark = d.l[layer].bm.get_mark();
	for (int dir = 0; dir < 2; dir++)
	if (dir == 0) {
		if (w[dir].empty())
			continue;
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols; x++) {
				if (PROB_SHAPE(p_prob[2 * x]) != BRICK_I_0)
					continue;
				int type_shape = d.l[layer].wts.get_type(PROB_TYPE(p_prob[2 * x])) << 8 | BRICK_I_0;
				int i;
				for (i = 0; i < w[dir].size(); i++)
				if (type_shape == w[dir][i].type_shape)
					break;
				if (i == w[dir].size())
					continue;
				WireMaskInfo & wsi = w[dir][i];
				int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]);
				//start to search hotpoint
				int type, subtype, pattern, wide;
				d.l[layer].wts.get_all(PROB_TYPE(p_prob[2 * x]), type, subtype, pattern, wide);
				if (subtype < WIRE_SUBTYPE_MSBRANCH) {
					qCritical("hotpoint_search found subtype=%d error", subtype);
					return;
				}
				WireParameter2 * wp = &(d.l[layer].vw.get_vw_para(pattern, type, subtype)->w2);
				CV_Assert(wp != NULL);
				wp->gray_i = gl;
				wp->gray_w = gm;
				int th0 = wide * wide * wp->gray_i + wide * wide * (wp->gray_w - wp->gray_i) * (wp->mth_high / 100.0);
				int offset[4];
				offset[0] = ((-wide / 2) * (int)d.l[layer].ig.step.p[0] + (-wide / 2)*(int)d.l[layer].ig.step.p[1]) / sizeof(int);
				offset[1] = ((-wide / 2) * (int)d.l[layer].ig.step.p[0] + (wide - wide / 2)*(int)d.l[layer].ig.step.p[1]) / sizeof(int);
				offset[2] = ((wide - wide / 2) * (int)d.l[layer].ig.step.p[0] + (-wide / 2)*(int)d.l[layer].ig.step.p[1]) / sizeof(int);
				offset[3] = ((wide - wide / 2) * (int)d.l[layer].ig.step.p[0] + (wide - wide / 2)*(int)d.l[layer].ig.step.p[1]) / sizeof(int);
				for (int yy = max(y0 - 6, d.l[layer].compute_border); yy <= min(y0 + 6, img.rows - d.l[layer].compute_border); yy++)
				if (mark.at<unsigned long long>(yy, x0) == 0xffffffffffffffffULL) {// not fill yet
					int * p_ig = d.l[layer].ig.ptr<int>(yy, x0);
					int sum = p_ig[offset[3]] + p_ig[offset[0]] - p_ig[offset[1]] - p_ig[offset[2]];
					if (sum >= th0) {
						int bidx = search_wire_branch(Point(x0, yy), DIR_UP, wide, -1, wp, wsi, d.l[layer]);
						search_wire_branch(Point(x0, yy), DIR_DOWN, wide, bidx, wp, wsi, d.l[layer]);
					}
				}
			}
		}
	}
	else {
		if (w[dir].empty())
			continue;
		for (int x = 0; x < prob.cols; x++) {
			for (int y = 0; y < prob.rows; y++) {
				unsigned long long p0 = prob.at<unsigned long long>(y, 2 * x);
				if (PROB_SHAPE(p0) != BRICK_I_90)
					continue;
				int type_shape = d.l[layer].wts.get_type(PROB_TYPE(p0)) << 8 | BRICK_I_90;
				int i;
				for (i = 0; i < w[dir].size(); i++)
				if (type_shape == w[dir][i].type_shape)
					break;
				if (i == w[dir].size())
					continue;
				WireMaskInfo & wsi = w[dir][i];
				int x0 = PROB_X(p0), y0 = PROB_Y(p0);
				//start to search hotpoint
				int type, subtype, pattern, wide;
				d.l[layer].wts.get_all(PROB_TYPE(p0), type, subtype, pattern, wide);
				if (subtype < WIRE_SUBTYPE_MSBRANCH) {
					qCritical("hotpoint_search found subtype=%d error", subtype);
					return;
				}
				WireParameter2 * wp = &(d.l[layer].vw.get_vw_para(pattern, type, subtype)->w2);
				CV_Assert(wp != NULL);
				wp->gray_i = gl;
				wp->gray_w = gm;
				int th0 = wide * wide * wp->gray_i + wide * wide * (wp->gray_w - wp->gray_i) * (wp->mth_high / 100.0);
				int offset[4];
				offset[0] = ((-wide / 2) * (int)d.l[layer].ig.step.p[0] + (-wide / 2)*(int)d.l[layer].ig.step.p[1]) / sizeof(int);
				offset[1] = ((-wide / 2) * (int)d.l[layer].ig.step.p[0] + (wide - wide / 2)*(int)d.l[layer].ig.step.p[1]) / sizeof(int);
				offset[2] = ((wide - wide / 2) * (int)d.l[layer].ig.step.p[0] + (-wide / 2)*(int)d.l[layer].ig.step.p[1]) / sizeof(int);
				offset[3] = ((wide - wide / 2) * (int)d.l[layer].ig.step.p[0] + (wide - wide / 2)*(int)d.l[layer].ig.step.p[1]) / sizeof(int);
				for (int xx = max(x0 - 6, d.l[layer].compute_border); xx <= min(x0 + 6, img.cols - d.l[layer].compute_border); xx++)
				if (mark.at<unsigned long long>(y0, xx) == 0xffffffffffffffffULL) {// not fill yet
					int * p_ig = d.l[layer].ig.ptr<int>(y0, xx);
					int sum = p_ig[offset[3]] + p_ig[offset[0]] - p_ig[offset[1]] - p_ig[offset[2]];
					if (sum >= th0) { //new branch
						int bidx = search_wire_branch(Point(xx, y0), DIR_LEFT, wide, -1, wp, wsi, d.l[layer]);
						search_wire_branch(Point(xx, y0), DIR_RIGHT, wide, bidx, wp, wsi, d.l[layer]);
					}
				}
			}
		}
	}

	if (cpara.method & OPT_DEBUG_EN) {
		Mat debug_draw;
		cvtColor(img, debug_draw, CV_GRAY2BGR);
		CV_Assert(debug_draw.type() == CV_8UC3);
		for (int y = 0; y < debug_draw.rows; y++) {
			unsigned char * p_draw = debug_draw.ptr<unsigned char>(y);
			unsigned long long  * p_mark = mark.ptr<unsigned long long>(y);
			for (int x = 0; x < debug_draw.cols; x++) {
				if (MARK_BRANCH(p_mark[x]) != INVALID_BRANCH)
				if (MARK_ISSKIN(p_mark[x]))
					p_draw[3 * x + 2] = 255; //draw skin red
				else
				if (MARK_LEN(p_mark[x]) == 0)
					p_draw[3 * x + 1] = 255; //draw bone green
			}
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_hotpoint.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw.clone();
		}
	}
}

/*
31..24 23..16   15..8   7..0
opt0:		             opt    layer_num
opt1:  connect_rd v_type cgray_ratio sw_min
connect_rd, v_type, cgray_ratio & sw_min will overwrite other layer ViaParameter
*/
static void assemble_via(PipeData & d, ProcessParameter & cpara)
{
	int layer[2] = { cpara.layer, cpara.layer - 1 };
	int assemble_opt = cpara.opt0 >> 8 & 0xff;
	int layer_num = cpara.opt0 & 0xff;
	bool allow_45 = (assemble_opt & ASSEMBLE_VIA_ALLOW_45) ? true : false;
	bool search_skin = (assemble_opt & ASSEMBLE_VIA_SEARCH_SKIN) ? true : false;
	SearchViaBranch sb;
	d.l[layer[0]].validate_ig();
	Mat & ig = d.l[layer[0]].ig;
	Mat & img = d.l[layer[0]].img;
	int idx = cpara.method_opt & 0xf;
	int sw_min = cpara.opt1 & 0xff;
	int c_ratio = cpara.opt1 >> 8 & 0xff;
	int v_type = cpara.opt1 >> 16 & 0xff;
	int connect_rd = cpara.opt1 >> 24 & 0xff;
	qInfo("assemble_via, l0=%d, l1=%d, layernum=%d,connect_rd=%d, sw_min=%d, c_ratio=%d, v_type=%d, allow45=%d",
		layer[0], layer[1], layer_num, connect_rd, sw_min, c_ratio, v_type, allow_45);

	if (d.l[layer[0]].v[idx].type != TYPE_GRAY_LEVEL) {
		qCritical("assemble_via gray level idx[%d]=%d, error", idx, d.l[layer[0]].v[idx].type);
		return;
	}
	if (layer_num > 2 || layer_num == 0) {
		qCritical("assemble_via layernum=%d, error", layer_num);
		return;
	}
	if (d.l[layer[0]].bm.get_mark().empty())
		d.l[layer[0]].bm.clear_mark(img);
	Mat mark = d.l[layer[0]].bm.get_mark();
	int gl = find_index(d.l[layer[0]].v[idx].d, (int)GRAY_L0);
	gl = d.l[layer[0]].v[idx].d.at<int>(gl, 2);
	int gm = find_index(d.l[layer[0]].v[idx].d, (int)GRAY_M0);
	gm = d.l[layer[0]].v[idx].d.at<int>(gm, 2);

	for (int l = 0; l < layer_num; l++) {
		ViaParameter * prev_vp = NULL;
		ViaParameter vp_copy; //vp_copy save real color
		if (layer[l] < 0)
			continue;
		for (int i = 0; i < d.l[layer[l]].via_info.size(); i++) {
			ViaInfo & v = d.l[layer[l]].via_info[i];
			ViaParameter * vp = &(d.l[layer[l]].vw.get_vw_para(v.pattern, v.type, v.subtype)->v);
			if (vp != prev_vp) {
				vp_copy = *vp;
				vp_copy.gray0 = gm;
				vp_copy.gray3 = gl;
				if (l == 1) { //replace parameter with new parameter
					vp_copy.type = v_type;
					vp_copy.cgray_ratio = c_ratio;
					vp_copy.swide_min = sw_min;
					vp_copy.connect_rd = connect_rd;
				}
				prev_vp = vp;
			}
			sb.search_branch(v.xy, &vp_copy, ig, d.l[layer[0]].bm, l);
		}
	}
	if (search_skin)
		d.l[layer[0]].bm.search_skin(ig, allow_45);
	if (cpara.method & OPT_DEBUG_EN) {
		Mat debug_draw;
		cvtColor(img, debug_draw, CV_GRAY2BGR);
		for (int y = 0; y < debug_draw.rows; y++) {
			unsigned char * p_draw = debug_draw.ptr<unsigned char>(y);
			unsigned long long  * p_mark = mark.ptr<unsigned long long>(y);
			for (int x = 0; x < debug_draw.cols; x++) {
				if (MARK_BRANCH(p_mark[x]) != INVALID_BRANCH)
				if (MARK_ISSKIN(p_mark[x]))
					p_draw[3 * x + 2] = 255; //draw skin red
				else
				if (MARK_LEN(p_mark[x]) == 0)
					p_draw[3 * x + 1] = 255; //draw bone green
			}
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer[0]) + "_assemble_via.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer[0]].v[debug_idx + 12].d = debug_draw.clone();
		}
	}
}

/*
31..24 23..16   15..8   7..0
opt0:		           opt
*/
static void assemble_branch(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	int assemble_opt = cpara.opt0 >> 8 & 0xff;
	bool allow_45 = (assemble_opt & ASSEMBLE_BRANCH_ALLOW_45) ? true : false;
	bool search_cross = (assemble_opt & ASSEMBLE_BRANCH_SEARCH_CROSS) ? true : false;
	bool remark_border = (assemble_opt & ASSEMBLE_BRANCH_NOT_REMARK_BORDER) ? false : true;
	Mat & img = d.l[layer].img;
	d.l[layer].validate_ig();

	qInfo("assemble_branch, l=%d, allow45=%d", layer, allow_45);

	if (search_cross)
		d.l[layer].bm.search_cross_points(allow_45, false, GUARD_FOR_JOINT, 20);
	d.l[layer].allow_45 = allow_45;
	d.l[layer].bm.draw_mark2(true, allow_45);
	d.l[layer].bm.self_check();

	if (remark_border) {
		d.l[layer].bm.remark_border_skin(d.l[layer].border_size * 2);
		d.l[layer].bm.search_skin(d.l[layer].ig, allow_45);
	}
	if (cpara.method & OPT_DEBUG_EN) {
		Mat mark2 = d.l[layer].bm.get_mark2();
		Mat mark = d.l[layer].bm.get_mark();
		Mat debug_draw;
		cvtColor(img, debug_draw, CV_GRAY2BGR);
		for (int y = 0; y < debug_draw.rows; y++) {
			unsigned char * p_draw = debug_draw.ptr<unsigned char>(y);
			unsigned * p_mark2 = mark2.ptr<unsigned>(y);
			unsigned long long  * p_mark = mark.ptr<unsigned long long>(y);
			for (int x = 0; x < debug_draw.cols; x++) {
				if (p_mark2[x]) {
					if (MARK2_LEN(p_mark2[x]) == 0)
						p_draw[3 * x + 1] = 255; //draw branch green
					if (MARK2_VIA(p_mark2[x]))
						p_draw[3 * x] = 255; //draw via blue
				}
				else {
					if (d.l[layer].bm.is_sub_branch(MARK_BRANCH(p_mark[x])))
						p_draw[3 * x + 2] = 255; //draw border red
				}

			}
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_assemble_branch.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw.clone();
		}
	}
}

/*
31..24 23..16   15..8   7..0
opt0:					filt_method process_method
for H_SHAPE_CHECK
opt1:						         clen
opt2:								 dlen
for VIA_CONNECT_CHECK
opt1:								 wlen
opt2:								 dlen_max
opt3:								 dlen_min
opt4:								 clen
opt5:								 check_opt
*/
static void shape_check(void * objs, int layer, int obj_type, ProcessParameter & cpara)
{
	if (layer != cpara.layer)
		return;
	BranchMark & bm = *((BranchMark *)objs);
	vector<BranchMeta> & sbs = bm.get_sbs();
	int filt_method = cpara.opt0 >> 8 & 0xff;
	switch (filt_method) {
	case H_SHAPE_CHECK:
		{
			int clen = cpara.opt1 & 0xff;
			int dlen = cpara.opt2 & 0xff;
			qInfo("hshape_check,cl=%d,dl=%d", clen, dlen);
			for (int i = 1; i < (int)sbs.size(); i++) {
				if (sbs[i].state != BranchMeta::READY_OUTPUT || sbs[i].dir < 0)
					continue;
				if (sbs[i].length() <= dlen && sbs[i].dc.size() >= 2) {
					int dir = sbs[i].dir;
					int c = cnear[dir][0] * sbs[i].bch.first.y + cnear[dir][1] * sbs[i].bch.first.x;
					int h_cnt = 0;
					for (int j = 0; j < (int)sbs[i].dc.size(); j++) {
						int k = sbs[i].dc[j];
						int c1 = cnear[dir][0] * sbs[k].bch.first.y + cnear[dir][1] * sbs[k].bch.first.x;
						int c2 = cnear[dir][0] * sbs[k].bch.second.y + cnear[dir][1] * sbs[k].bch.second.x;
						if (abs(c - c1) > clen && abs(c - c2) > clen)
							h_cnt++;
					}
					if (h_cnt >= 2)
						sbs[i].prob = 0.5;
				}
			}
			break;
		}
	case VIA_CONNECT_CHECK:
	{
#define GET_ROOT_PATH(x, xpath) { int r = x;  while (check_net[r] != r) { xpath.push_back(r); r = check_net[r]; } xpath.push_back(r); reverse(xpath.begin(), xpath.end());}

#define GET_P2P_PATH(xpath, ypath, zpath) { int t = 0; \
							  for (; t < xpath.size() && t < ypath.size() && xpath[t] == ypath[t]; t++); \
							  CV_Assert(t >= 1); \
							  for (int u = xpath.size() - 1; u >= t; u--) if (sbs[xpath[u]].dir >= 0) zpath.push_back(xpath[u]); \
							  for (int u = t - 1; u < ypath.size(); u++) if (sbs[ypath[u]].dir >= 0) zpath.push_back(ypath[u]); }
			int wlen = cpara.opt1 & 0xff;
			int dlen_max = cpara.opt2 & 0xff;
			int dlen_min = cpara.opt3 & 0xff;
			int olen = cpara.opt4 & 0xff;
			int check_opt = cpara.opt5 & 0xff;
			qInfo("hshape_check,wlen=%d,dmax=%d,dmin=%d,olen=%d,check_opt=%d", wlen, dlen_max, dlen_min, olen, check_opt);
			vector<int> check_net(sbs.size(), -1); //check_net save father node idx
			for (int i = 1; i < (int)sbs.size(); i++) {
				if (sbs[i].state == BranchMeta::ALEADY_OUTPUT || check_net[i]>0)
					continue;				
				vector <int> vias; //network vias
				vector <int> wires; //network wires
				vector <int> sq; //deep first search
				check_net[i] = i; //i is root node
				sq.push_back(i);
				while (!sq.empty()) {
					int b = sq.back(); //b is expand node
					sq.pop_back();
					if (sbs[b].dir >= 0)
						wires.push_back(b);
					else {
						if (sbs[b].final_out.size() >= 2)
							sbs[b].prob = 0.5;
						vias.push_back(b);
					}
					for (int j = 0; j < (int)sbs[b].dc.size(); j++) {
						int k = sbs[b].dc[j];
						if (k != check_net[b] && sbs[k].state != BranchMeta::ALEADY_OUTPUT) { //k is not b's father
							if (check_net[k] > 0 && check_opt & CHECK_LOOP) { //loop happen
								vector<int> k2root, b2root, path;
								GET_ROOT_PATH(k, k2root);
								GET_ROOT_PATH(b, b2root);
								GET_P2P_PATH(k2root, b2root, path); //path is loop
								if (path.size() < 3)
									continue;
								int min_len = 0x70000000, min_idx;
								for (int l = 0; l < (int)path.size(); l++)
								if (sbs[path[l]].length() < min_len) { //pick shortest wire and 
									min_len = sbs[path[l]].length();
									min_idx = path[l];
								}
								sbs[min_idx].prob = 0.5;
							}
							else
							if (check_net[k] < 0) {
								check_net[k] = b;
								sq.push_back(k);
							}
						}
					}
				}
				//check h shape
				if (vias.size() > 0 && wires.size() > 1)
				for (int j = 0; j < (int) wires.size(); j++) 
				if (sbs[wires[j]].length() >= wlen) {
#if 1
					BranchMeta & bm = sbs[wires[j]];
					if (bm.bch.first.y >= 10188 && bm.bch.first.y <= 10222 && bm.bch.second.y >= 10188 && bm.bch.second.y <= 10222
						&& bm.bch.first.x >= 34885)
						bm.bch.first.y = bm.bch.first.y * 2 - bm.bch.first.y;
#endif
					vector <int> j2root;
					GET_ROOT_PATH(wires[j], j2root);
					for (int k = j + 1; k < (int) wires.size(); k++)
					if (sbs[wires[k]].length() >= wlen) {
						int distance = sbs[wires[j]].intersect(sbs[wires[k]], dlen_max);
						if (distance == -1 || distance >=dlen_min) {
							//find the path from j to k
							vector<int> k2root, path;
							GET_ROOT_PATH(wires[k], k2root);
							GET_P2P_PATH(j2root, k2root, path);
							for (int l = 0; l < (int)path.size() - 1; l++) {
								Point pis;
								intersect_line(sbs[path[l]].bch.first, sbs[path[l]].dir, sbs[path[l+1]].bch.first, sbs[path[l+1]].dir, pis);
								for (int v = 0; v < (int)vias.size(); v++)
								if (sbs[vias[v]].dir == -1 && sbs[vias[v]].distance(pis)<=olen)
								if (sbs[path[l]].length() < sbs[path[l + 1]].length())
									sbs[path[l]].prob = 0.5;
								else
									sbs[path[l + 1]].prob = 0.5;
							}
						}
					}
				}
			}
		}
	}

}

#define OP_FILTER 0
#define OP_SHAPE_CHECK	1
struct ObjProcess {
	int method;
	ObjProcessFunc process_func;
} obj_process_array[] = {
	{ OP_SHAPE_CHECK, shape_check }
};
/*
31..24 23..16   15..8   7..0
opt0:							process_method
cwide, clong_heng clong_shu is for connectivity. Normally cwide <=3
method_opt
idx: via info
*/
static ObjProcessHook obj_process_translate(ProcessParameter & cpara)
{
	ObjProcessHook hook;
	hook.func = NULL;
	hook.cpara = cpara;
	int process_method = cpara.opt0 & 0xff;
	for (int i = 0; i < sizeof(obj_process_array) / sizeof(obj_process_array[0]); i++) {
		if (obj_process_array[i].method == process_method) {
			qInfo("hook obj process method %d, opt1=%d, opt2=%d", obj_process_array[i].method, cpara.opt1, cpara.opt2);
			hook.func = obj_process_array[i].process_func;
			break;
		}
	}
	return hook;
}

#define PP_SET_PARAM			0
#define PP_RGB2GRAY				1
#define PP_COMPUTE_MIN_STAT		2
#define PP_ADJUST_GRAY_LVL		3
#define PP_COARSE_LINE_SEARCH	4
#define PP_COARSE_VIA_MASK		5
#define PP_FINE_VIA_SEARCH		6
#define PP_REMOVE_VIA			7
#define PP_FINE_SEARCH_MASK		8
#define PP_FINE_LINE_SEARCH		9
#define PP_ASSEMBLE				10
#define PP_HOTPOINT_SEARCH		12
#define PP_ASSEMBLE_VIA			13
#define PP_ASSEMBLE_BRANCH		14
#define PP_EDGE_DETECT			15
#define PP_IMAGE_ENHANCE		16
#define PP_OBJ_PROCESS			254

typedef void(*ProcessFunc)(PipeData & d, ProcessParameter & cpara);
struct PipeProcess {
	int method;
	ProcessFunc process_func;
} pipe_process_array[] = {
	{ PP_SET_PARAM, set_pipeline_param },
	{ PP_RGB2GRAY, imgpp_RGB2gray },
	{ PP_COMPUTE_MIN_STAT, imgpp_compute_min_stat },
	{ PP_ADJUST_GRAY_LVL, imgpp_adjust_gray_lvl },
	{ PP_COARSE_LINE_SEARCH, coarse_line_search },
	{ PP_EDGE_DETECT, edge_detect },
	{ PP_IMAGE_ENHANCE, image_enhance },
	{ PP_COARSE_VIA_MASK, coarse_via_search_mask },
	{ PP_FINE_VIA_SEARCH, fine_via_search },
	{ PP_REMOVE_VIA, remove_via },
	{ PP_FINE_SEARCH_MASK, hotpoint2fine_search_stmask },
	{ PP_FINE_LINE_SEARCH, fine_line_search },
	{ PP_HOTPOINT_SEARCH, hotpoint_search },
	{ PP_ASSEMBLE_VIA, assemble_via },
	{ PP_ASSEMBLE_BRANCH, assemble_branch }
};

class ProcessTileData {
public:
	vector<ProcessFunc> * process_func;
	vector<ProcessParameter> * vwp;
	vector<ObjProcessHook> * p_obj_process;
	PipeData * d; //current pipe data
	PipeData * lpd; //left pipe data
	PipeData * upd; //upper pipe data
	QRect * sb;
	QPoint * sr_tl_pixel; //top left pixel for generate objects
	int merge_method;
	ProcessTileData() {
		process_func = NULL;
		vwp = NULL;
		d = NULL;
	}
	void release_temp_data() const {
		for (int l = 0; l < (int)d->l.size(); l++)
			d->l[l].release_temp_data();
	}
};

static void merge_tile_get_result(vector<MarkObj> & objs, const ProcessTileData & t)
{
	vector<ObjProcessHook> * p_obj_process = t.p_obj_process;
	int layer_num = (int)t.d->l.size();
	const QRect & sb = *(t.sb);
	const QPoint & sr_tl_pixel = *(t.sr_tl_pixel);
	PipeData & cpd = *(t.d);
	PipeData & lpd = *(t.lpd);
	PipeData & upd = *(t.upd);

	for (int l = 0; l < layer_num; l++) {
		cpd.l[l].bm.stitch_at(Point(cpd.l[l].img_pixel_x0 + sr_tl_pixel.x(), cpd.l[l].img_pixel_y0 + sr_tl_pixel.y()));
		int x_limit = (cpd.x0 == sb.right()) ? cpd.l[l].img_pixel_x0 + sr_tl_pixel.x() + cpd.l[l].raw_img.cols + 60 :
			cpd.l[l].img_pixel_x0 + sr_tl_pixel.x() + cpd.l[l].raw_img.cols - cpd.l[l].border_size * 2 - 20;
		int y_limit = (cpd.y0 == sb.bottom()) ? cpd.l[l].img_pixel_y0 + sr_tl_pixel.y() + cpd.l[l].raw_img.rows + 60 :
			cpd.l[l].img_pixel_y0 + sr_tl_pixel.y() + cpd.l[l].raw_img.rows - cpd.l[l].border_size * 2 - 20;
		/*
		if (cpd.x0 > sb.left())
		cpd.l[l].bm.merge(lpd.l[l].bm, false, cpd.l[l].guard_len.wire_connect, cpd.l[l].guard_len.via_connect,
		cpd.l[l].guard_len.wire_merge, cpd.l[l].guard_len.via_merge, cpd.l[l].img_pixel_x0 + sr_tl_pixel.x() - cpd.l[l].border_size,
		cpd.l[l].img_pixel_x0 + sr_tl_pixel.x() + cpd.l[l].border_size * 2, cpd.l[l].img_pixel_x0 + sr_tl_pixel.x(), y_limit, true);
		if (cpd.y0 > sb.top())
		cpd.l[l].bm.merge(upd.l[l].bm, true, cpd.l[l].guard_len.wire_connect, cpd.l[l].guard_len.via_connect,
		cpd.l[l].guard_len.wire_merge, cpd.l[l].guard_len.via_merge, cpd.l[l].img_pixel_y0 + sr_tl_pixel.y() - cpd.l[l].border_size,
		cpd.l[l].img_pixel_y0 + sr_tl_pixel.y() + cpd.l[l].border_size * 2, cpd.l[l].img_pixel_y0 + sr_tl_pixel.y(), x_limit, false);
		*/
		if (cpd.x0 > sb.left())
			cpd.l[l].bm.merge(lpd.l[l].bm, false, cpd.l[l].guard_len.wire_connect, cpd.l[l].guard_len.wire_merge, cpd.l[l].guard_len.via_merge,
			Rect(cpd.l[l].img_pixel_x0 + sr_tl_pixel.x(), cpd.l[l].img_pixel_y0 + sr_tl_pixel.y(),
			cpd.l[l].border_size * 2, cpd.l[l].raw_img.rows), cpd.l[l].img_pixel_x0 + sr_tl_pixel.x() - 20, y_limit);
		if (cpd.y0 > sb.top())
			cpd.l[l].bm.merge(upd.l[l].bm, true, cpd.l[l].guard_len.wire_connect, cpd.l[l].guard_len.wire_merge, cpd.l[l].guard_len.via_merge,
			Rect(cpd.l[l].img_pixel_x0 + sr_tl_pixel.x(), cpd.l[l].img_pixel_y0 + sr_tl_pixel.y(),
			cpd.l[l].raw_img.cols, cpd.l[l].border_size * 2), cpd.l[l].img_pixel_y0 + sr_tl_pixel.y() - 20, x_limit);


		qInfo("get result for l=%d, Rect(lt=%d,%d, rb=%d,%d, erb=%d,%d)", l, cpd.l[l].img_pixel_x0 + sr_tl_pixel.x(), cpd.l[l].img_pixel_y0 + sr_tl_pixel.y(),
			x_limit, y_limit, x_limit + cpd.l[l].border_size * 2, y_limit + cpd.l[l].border_size * 2);
		cpd.l[l].bm.get_result(x_limit, y_limit, cpd.l[l].guard_len.end_wire, l, objs, *p_obj_process);
	}
	t.release_temp_data();
}

//Process tile for all layer
static ProcessTileData process_tile(const ProcessTileData & t)
{
	vector<ProcessParameter> & vwp = *(t.vwp);
	vector<ProcessFunc> & process_func = *(t.process_func);
	PipeData & d = *(t.d);
	qInfo("Start process_tile (x=%d,y=%d), Rect(lt=%d,%d,rb=%d,%d)", t.d->x0, t.d->y0, t.d->l[0].img_pixel_x0, t.d->l[0].img_pixel_y0,
		t.d->l[0].img_pixel_x0 + t.d->l[0].raw_img.cols, t.d->l[0].img_pixel_y0 + t.d->l[0].raw_img.rows);
	for (int i = 0; i < vwp.size(); i++) {
		int method = vwp[i].method & 0xff;
		if (process_func[method] == NULL && method != PP_OBJ_PROCESS) {
			qCritical("process func method %d invalid", method);
			return t;
		}
		if (process_func[method] != NULL)
			process_func[method](d, vwp[i]);
	}
	return t;
}

VWExtractPipe::VWExtractPipe()
{
	private_data = NULL;
	prev_layer = -1;
}

VWExtractPipe::~VWExtractPipe()
{
	if (private_data) {
		PipeData * d = (PipeData *)private_data;
		delete d;
	}

}
int VWExtractPipe::set_train_param(int layer, int type, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, float pf1)
{
	ProcessParameter cpara;
	cpara.method = type >> 16 & 0xffff;
	cpara.method_opt = type & 0xffff;
	cpara.opt0 = pi1;
	cpara.opt1 = pi2;
	cpara.opt2 = pi3;
	cpara.opt3 = pi4;
	cpara.opt4 = pi5;
	cpara.opt5 = pi6;
	cpara.opt6 = pi7;
	cpara.opt_f0 = pf1;
	cpara.layer = (layer < 0) ? prev_layer : layer;
	if (cpara.layer < 0)
		qCritical("layer=%d < 0", layer);
	prev_layer = cpara.layer;
	vwp.push_back(cpara);
	return 0;
}

int VWExtractPipe::set_extract_param(int layer, int type, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, float pf1)
{
	return set_train_param(layer, type, pi1, pi2, pi3, pi4, pi5, pi6, pi7, pf1);
}

Mat VWExtractPipe::get_mark(int layer)
{
	if (private_data) {
		PipeData * d = (PipeData *)private_data;
		if (layer < d->l.size())
			return d->l[layer].v[12].d;
	}
	return Mat();
}

Mat VWExtractPipe::get_mark1(int layer)
{
	if (private_data) {
		PipeData * d = (PipeData *)private_data;
		if (layer < d->l.size())
			return d->l[layer].v[13].d;
	}
	return Mat();
}

Mat VWExtractPipe::get_mark2(int layer)
{
	if (private_data) {
		PipeData * d = (PipeData *)private_data;
		if (layer < d->l.size())
			return d->l[layer].v[14].d;
	}
	return Mat();
}

Mat VWExtractPipe::get_mark3(int layer)
{
	if (private_data) {
		PipeData * d = (PipeData *)private_data;
		if (layer < d->l.size())
			return d->l[layer].v[15].d;
	}
	return Mat();
}

void VWExtractPipe::get_feature(int layer, int x, int y, std::vector<float> &, std::vector<int> & f)
{
	if (private_data) {
		PipeData * d = (PipeData *)private_data;
		if (layer < d->l.size()) {
			f.resize(4);
			if (d->l[layer].prob.empty())
				return;
			unsigned long long * ret = get_prob(d->l[layer].prob, x, y, d->l[layer].gs);
			unsigned long long prob0 = ret[0];
			unsigned long long prob1 = ret[1];
			f[0] = PROB_S(prob0);
			f[1] = PROB_XY(prob0);
			f[2] = PROB_S(prob1);
			f[3] = PROB_XY(prob1);
		}
	}
}

int VWExtractPipe::extract(string file_name, QRect, vector<MarkObj> & obj_sets)
{
	vector<ObjProcessHook> obj_process;
	int im_dec_flag[100] = { 0 };
	for (int i = 0; i < vwp.size(); i++) {
		qInfo("extract vw%d:l=0x%x,m=0x%x,mo=0x%x,o0=0x%x,o1=0x%x,o2=0x%x,o3=0x%x,o4=0x%x,o5=0x%x,o6=0x%x,i8=0x%x,f0=%f",
			i, vwp[i].layer, vwp[i].method, vwp[i].method_opt, vwp[i].opt0, vwp[i].opt1, vwp[i].opt2,
			vwp[i].opt3, vwp[i].opt4, vwp[i].opt5, vwp[i].opt6, vwp[i].opt_f0);
		if ((vwp[i].method & 0xff) == PP_RGB2GRAY) {
			qInfo("Layer %d is color RGB", vwp[i].layer);
			im_dec_flag[vwp[i].layer] = 1;
		}
		if ((vwp[i].method & 0xff) == PP_OBJ_PROCESS)
			obj_process.push_back(obj_process_translate(vwp[i]));
	}
	QDir *qdir = new QDir;
	deldir("./DImg");
	bool exist = qdir->exists("./DImg");
	if (!exist) {
		bool ok = qdir->mkdir("./DImg");
		if (!ok)
			qCritical("mkdir failed");
	}
	delete qdir;
	PipeData * d;
	if (private_data) {
		d = (PipeData *)private_data;
		delete d;
	}
	d = new PipeData();
	private_data = d;

	vector<ProcessFunc> process_func(256, NULL);
	for (int i = 0; i < sizeof(pipe_process_array) / sizeof(pipe_process_array[0]); i++)
		process_func[pipe_process_array[i].method] = pipe_process_array[i].process_func;
	int layer_num = 0;
	for (int i = 0; i < vwp.size(); i++)
		layer_num = max(layer_num, vwp[i].layer);
	layer_num++;
	d->l.resize(layer_num);
	d->x0 = 0;
	d->y0 = 0;
	unsigned char ll = file_name[file_name.size() - 5];
	for (int i = 0; i < layer_num; i++) {
		file_name[file_name.size() - 5] = i + ll;
		Mat img = imread(file_name, im_dec_flag[i]);
		if (!img.empty())
			d->l[i].set_raw_img(img);
		else {
			qCritical("extract read layer %s, image file error", file_name.c_str());
			return -1;
		}
		qInfo("Load %s", file_name.c_str());
	}

	ProcessTileData ptd;
	ptd.d = d;
	ptd.process_func = &process_func;
	ptd.vwp = &vwp;
	process_tile(ptd);

#if 0
	for (int l = 0; l < layer_num; l++) {
		for (int j = 0; j < (int)d->l[l].via_info.size(); j++)
			d->l[l].fwv.push(d->l[l].via_info[j], 0, 0);
		for (int dir = 0; dir < 4; dir++) {
			for (int j = 0; j < d->l[l].lineset[dir].size(); j++) {
				WireLine & wl = d->l[l].lineset[dir][j];
				d->l[l].fwv.push(wl, 0, 0, dir);
			}
		}
		for (int i = 0; i < (int)obj_process.size(); i++)
			obj_process[i].func(d->l[l].fwv, l, OBJ_WIRE, obj_process[i].cpara);
	}

	for (int l = 0; l < layer_num; l++) {
		for (int dir = 0; dir < 4; dir++) {
			vector<FinalLine> & lines = d->l[l].fwv.lines[dir];
			for (vector<FinalLine>::const_iterator it = lines.begin(); it != lines.end(); it++) {
				Point p0 = it->point_at(0);
				for (int i = 1; i < (int)it->point_num(); i++) {
					MarkObj wire;
					Point p1 = it->point_at(i);
					wire.type = OBJ_LINE;
					wire.type2 = LINE_WIRE_AUTO_EXTRACT;
					wire.type3 = l;
					wire.state = 0;
					wire.prob = 1;
					wire.p0 = QPoint(p0.x, p0.y);
					wire.p1 = QPoint(p1.x, p1.y);
					obj_sets.push_back(wire);
					p0 = p1;
				}
			}
		}

		for (int j = 0; j < d->l[l].fwv.via_num(); j++) {
			QPoint point(d->l[l].fwv.via_at(j).x, d->l[l].fwv.via_at(j).y);
			MarkObj via;
			via.type = OBJ_POINT;
			via.type2 = POINT_VIA_AUTO_EXTRACT;
			via.type3 = l;
			via.state = 0;
			via.prob = 1;
			via.p0 = point;
			via.p1 = point;
			obj_sets.push_back(via);
		}
	}
#endif
	for (int i = 0; i < layer_num; i++) {
		d->l[i].bm.get_result(0x70000000, 0x70000000, 20, i, obj_sets, obj_process);
	}
	qInfo("Extract finished successfully");
	qDebug("*#*#DumpMessage#*#*");
	return 0;
}


int VWExtractPipe::extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets)
{
	int BORDER_SIZE = 16;
	int parallel_search = 0;
	int im_dec_flag[100] = { 0 };
	vector<ObjProcessHook> obj_process;
	QDir *qdir = new QDir;
	deldir("./DImg");
	bool exist = qdir->exists("./DImg");
	if (!exist) {
		bool ok = qdir->mkdir("./DImg");
		if (!ok)
			qCritical("mkdir failed");
	}
	delete qdir;
	for (int i = 0; i < vwp.size(); i++) {
#if !DEBUG_LEFT_TOP_IMG
		vwp[i].method &= ~OPT_DEBUG_EN;
#endif
		qInfo("extract vw%d:l=0x%x,m=0x%x,mo=0x%x,o0=0x%x,o1=0x%x,o2=0x%x,o3=0x%x,o4=0x%x,o5=0x%x,o6=0x%x,i8=0x%x,f0=%f",
			i, vwp[i].layer, vwp[i].method, vwp[i].method_opt, vwp[i].opt0, vwp[i].opt1, vwp[i].opt2,
			vwp[i].opt3, vwp[i].opt4, vwp[i].opt5, vwp[i].opt6, vwp[i].opt_f0);
		if ((vwp[i].opt0 >> 24) == 1 && (vwp[i].method & 0xff) == PP_SET_PARAM) {
			BORDER_SIZE = vwp[i].opt0 >> 16 & 0xff;
		}
		if ((vwp[i].method & 0xff) == PP_RGB2GRAY) {
			qInfo("Layer %d is color RGB", vwp[i].layer);
			im_dec_flag[vwp[i].layer] = 1;
		}
		if ((vwp[i].method & 0xff) == PP_OBJ_PROCESS)
			obj_process.push_back(obj_process_translate(vwp[i]));
	}

	//prepare process
	vector<ProcessFunc> process_func(256, NULL);
	for (int i = 0; i < sizeof(pipe_process_array) / sizeof(pipe_process_array[0]); i++)
		process_func[pipe_process_array[i].method] = pipe_process_array[i].process_func;
	int layer_num = 0;
	for (int i = 0; i < vwp.size(); i++)
		layer_num = max(layer_num, vwp[i].layer);
	layer_num++;

	int block_x, block_y;
	ic_layer[0]->getBlockNum(block_x, block_y);
	int block_width = ic_layer[0]->getBlockWidth();
	int scale = 32768 / block_width;
	vector<PipeData> diag_line[2];
	obj_sets.clear();
	for (int area_idx = 0; area_idx < area_.size(); area_idx++) {
		//extend area to sr, this is because extract can't process edge grid
		QRect sr = area_[area_idx].rect.marginsAdded(QMargins(BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE));
		parallel_search = area_[area_idx].option & OPT_PARALLEL_SEARCH;
		sr &= QRect(0, 0, block_x << 15, block_y << 15);
		if (sr.width() <= 0x8000 || sr.height() <= 0x8000) {
			qWarning("Picture(%d,%d) too small!", sr.width(), sr.height());
			continue;
		}

		QRect sb(QPoint((sr.left() + 0x4000) >> 15, (sr.top() + 0x4000) >> 15),
			QPoint((sr.right() - 0x4000) >> 15, (sr.bottom() - 0x4000) >> 15));
		CV_Assert(sb.right() >= sb.left() && sb.bottom() >= sb.top());
		for (int i = 0; i < 2; i++)
			diag_line[i].resize(max(sb.width() + 1, sb.height() + 1));

		int lx = sr.left() - (sb.left() << 15);
		int rx = sr.right() - ((sb.right() + 1) << 15);
		int ty = sr.top() - (sb.top() << 15);
		int by = sr.bottom() - ((sb.bottom() + 1) << 15);
		int cl = 1;
		int cl_num;
		QPoint sr_tl_pixel = sr.topLeft() / scale;
		qInfo("extract Rect, lt=(%d,%d), rb=(%d,%d), multithread=%d", sr_tl_pixel.x(), sr_tl_pixel.y(), sr.right() / scale, sr.bottom() / scale, parallel_search);
		for (int xay = sb.left() + sb.top(); xay <= sb.right() + sb.bottom(); xay++) {
			for (int i = 0; i < diag_line[cl].size(); i++) //release current diag_line memory
			for (int j = 0; j < diag_line[cl][i].l.size(); j++)
				diag_line[cl][i].l[j].release_temp_data();
			cl = 1 - cl;
			cl_num = 0;
			for (int i = 0; i < diag_line[cl].size(); i++) //clear current diag_line
				diag_line[cl][i].l.clear();
			//1 load image to diag_line
			for (int x0 = sb.left(); x0 <= sb.right(); x0++) {
				int y0 = xay - x0;
				if (sb.contains(x0, y0)) { //load image per Tile
					PipeData * d = &diag_line[cl][cl_num++];
					d->x0 = x0;
					d->y0 = y0;
					//1.1 compute load image source and destination
					int wide[3] = { 0 }, height[3] = { 0 }; //wide & height is image destination bound

					if (x0 == sb.left()) {
						if (lx < 0) {				//if left edge locates at less than half image picture, 
							wide[0] = -lx / scale;	//load one more image
							wide[1] = block_width;
						}
						else
							wide[1] = block_width - lx / scale;
					}
					if (x0 == sb.right()) {
						if (rx > 0) {				//if right edge locates at less than half image picture,  
							wide[1] = (wide[1] == 0) ? block_width : wide[1]; //load one more image
							wide[2] = rx / scale;
						}
						else
							wide[1] = block_width + rx / scale;
					}
					if (x0 != sb.left() && x0 != sb.right())
						wide[1] = block_width;

					if (y0 == sb.top()) {
						if (ty < 0) {				//if top edge locates at less than half image picture, 
							height[0] = -ty / scale;	//load one more image
							height[1] = block_width;
						}
						else
							height[1] = block_width - ty / scale;
					}
					if (y0 == sb.bottom()) {
						if (by > 0) {				//if bottom edge locates at less than half image picture, 
							height[1] = (height[1] == 0) ? block_width : height[1];	//load one more image
							height[2] = by / scale;
						}
						else
							height[1] = block_width + by / scale;
					}
					if (y0 != sb.top() && y0 != sb.bottom())
						height[1] = block_width;
					d->l.resize(layer_num);
					for (int l = 0; l < layer_num; l++) {
						int ewide = (x0 == sb.left()) ? 0 : BORDER_SIZE * 2;
						int ehight = (y0 == sb.top()) ? 0 : BORDER_SIZE * 2;
						Mat img(height[0] + height[1] + height[2] + ehight, wide[0] + wide[1] + wide[2] + ewide, im_dec_flag[l] ? CV_8UC3 : CV_8U);
						if (ewide == 0 && ehight == 0) {
							d->l[l].img_pixel_x0 = 0;
							d->l[l].img_pixel_y0 = 0;
						}
						if (ewide != 0) { // copy left image from old diag_line
							Mat * s_img;
							if (diag_line[1 - cl][cl_num - 1].y0 == y0) {
								s_img = &diag_line[1 - cl][cl_num - 1].l[l].raw_img;
								d->l[l].img_pixel_x0 = diag_line[1 - cl][cl_num - 1].l[l].img_pixel_x0 + s_img->cols
									- BORDER_SIZE * 2;
								d->l[l].img_pixel_y0 = diag_line[1 - cl][cl_num - 1].l[l].img_pixel_y0;
							}
							else {
								CV_Assert(diag_line[1 - cl][cl_num - 2].y0 == y0);
								s_img = &diag_line[1 - cl][cl_num - 2].l[l].raw_img;
								d->l[l].img_pixel_x0 = diag_line[1 - cl][cl_num - 2].l[l].img_pixel_x0 + s_img->cols
									- BORDER_SIZE * 2;
								d->l[l].img_pixel_y0 = diag_line[1 - cl][cl_num - 2].l[l].img_pixel_y0;
							}
							CV_Assert(s_img->rows == img.rows);
							(*s_img)(Rect(s_img->cols - BORDER_SIZE * 2, 0, BORDER_SIZE * 2, img.rows)).copyTo(img(Rect(0, 0, BORDER_SIZE * 2, img.rows)));
						}

						if (ehight != 0){  // copy upper image from old diag_line
							Mat * s_img;
							if (diag_line[1 - cl][cl_num - 1].x0 == x0) {
								s_img = &diag_line[1 - cl][cl_num - 1].l[l].raw_img;
								if (ewide != 0)
									CV_Assert(d->l[l].img_pixel_x0 == diag_line[1 - cl][cl_num - 1].l[l].img_pixel_x0 &&
									d->l[l].img_pixel_y0 == diag_line[1 - cl][cl_num - 1].l[l].img_pixel_y0 + s_img->rows
									- BORDER_SIZE * 2);
								d->l[l].img_pixel_x0 = diag_line[1 - cl][cl_num - 1].l[l].img_pixel_x0;
								d->l[l].img_pixel_y0 = diag_line[1 - cl][cl_num - 1].l[l].img_pixel_y0 + s_img->rows
									- BORDER_SIZE * 2;
							}
							else {
								CV_Assert(diag_line[1 - cl][cl_num].x0 == x0);
								s_img = &diag_line[1 - cl][cl_num].l[l].raw_img;
								if (ewide != 0)
									CV_Assert(d->l[l].img_pixel_x0 == diag_line[1 - cl][cl_num].l[l].img_pixel_x0 &&
									d->l[l].img_pixel_y0 == diag_line[1 - cl][cl_num].l[l].img_pixel_y0 + s_img->rows
									- BORDER_SIZE * 2);
								d->l[l].img_pixel_x0 = diag_line[1 - cl][cl_num].l[l].img_pixel_x0;
								d->l[l].img_pixel_y0 = diag_line[1 - cl][cl_num].l[l].img_pixel_y0 + s_img->rows
									- BORDER_SIZE * 2;
							}
							CV_Assert(s_img->cols == img.cols);
							(*s_img)(Rect(0, s_img->rows - BORDER_SIZE * 2, img.cols, BORDER_SIZE * 2)).copyTo(img(Rect(0, 0, img.cols, BORDER_SIZE * 2)));
						}

						//When extract width(height) > 2 * image width
						//if loading image is not at the edge, load 1*1 image; if at the edge, load 1*1 or 1*2;
						//if at the corner, load 1*1 or 1*2, or 2*2.
						//When extract width(height) < 2 * image width
						//if loading image is at the edge, load 1*1, 1*2 or 1*3 image, if at the corner, load 1*1 or 1*2 or 2*2 or 2*3 or 3*3 
						for (int y = y0 - 1, img_y = ehight; y <= y0 + 1; y++) {
							int dy = y - y0;
							for (int x = x0 - 1, img_x = ewide; x <= x0 + 1; x++) {
								int dx = x - x0;
								if (wide[dx + 1] != 0 && height[dy + 1] != 0) {
									vector<uchar> encode_img;
									if (ic_layer[l]->getRawImgByIdx(encode_img, x, y, 0, 0, false) != 0) {
										qCritical("load image error at l=%d, (%d,%d)", l, x, y);
										return -1;
									}
									Mat image = imdecode(Mat(encode_img), im_dec_flag[l]);
									if (image.rows != image.cols) {
										qCritical("load image rows!=cols at l=%d, (%d,%d)", l, x, y);
										return -1;
									}
									QRect s(0, 0, image.cols, image.rows);
									if (dx == -1)
										s.setLeft(image.cols - wide[0]);
									if (dx == 1)
										s.setRight(wide[2] - 1);
									if (dx == 0) {
										if (wide[0] == 0 && x0 == sb.left())
											s.setLeft(image.cols - wide[1]);
										if (wide[2] == 0 && x0 == sb.right())
											s.setRight(wide[1] - 1);
									}
									if (dy == -1)
										s.setTop(image.rows - height[0]);
									if (dy == 1)
										s.setBottom(height[2] - 1);
									if (dy == 0) {
										if (height[0] == 0 && y0 == sb.top())
											s.setTop(image.rows - height[1]);
										if (height[2] == 0 && y0 == sb.bottom())
											s.setBottom(height[1] - 1);
									}
									CV_Assert(s.width() == wide[dx + 1] && s.height() == height[dy + 1]);
									if (l == 0)
										qDebug("load image%d_%d, (x=%d,y=%d),(w=%d,h=%d) to (x=%d,y=%d)", y, x, s.left(), s.top(), s.width(), s.height(), img_x, img_y);
									image(Rect(s.left(), s.top(), s.width(), s.height())).copyTo(img(Rect(img_x, img_y, wide[dx + 1], height[dy + 1])));
								}
								img_x += wide[dx + 1];
							}
							img_y += height[dy + 1];
						}
						d->l[l].set_raw_img(img);
					}
				}
			}
			//2 now diag_line is loaded, process it
			vector<ProcessTileData> ptd_sets(cl_num);
			for (int i = 0; i < cl_num; i++) {
				ptd_sets[i].process_func = &process_func;
				ptd_sets[i].vwp = &vwp;
				ptd_sets[i].d = &diag_line[cl][i];
				ptd_sets[i].lpd = &diag_line[1 - cl][i];
				ptd_sets[i].upd = &diag_line[1 - cl][i];
				ptd_sets[i].merge_method = 1;
				ptd_sets[i].sb = &sb;
				ptd_sets[i].sr_tl_pixel = &sr_tl_pixel;
				ptd_sets[i].p_obj_process = &obj_process;
				if (ptd_sets[i].d->y0 > sb.top() && ptd_sets[i].d->x0 != ptd_sets[i].upd->x0)
					ptd_sets[i].upd = &diag_line[1 - cl][i + 1];
				if (ptd_sets[i].d->x0 > sb.left() && ptd_sets[i].d->y0 != ptd_sets[i].lpd->y0)
					ptd_sets[i].lpd = &diag_line[1 - cl][i - 1];
				if (ptd_sets[i].d->x0 > sb.left())
					CV_Assert(ptd_sets[i].d->y0 == ptd_sets[i].lpd->y0);
				if (ptd_sets[i].d->y0 > sb.top())
					CV_Assert(ptd_sets[i].d->x0 == ptd_sets[i].upd->x0);
#if DEBUG_LEFT_TOP_IMG
				if (xay == sb.left() + sb.top()) {
					for (int l = 0; l < layer_num; l++) {
						char file_name[50];
						sprintf(file_name, "./DImg/sc_M%d.jpg", l);
						imwrite(file_name, diag_line[cl][0].l[l].raw_img);
					}
				}
#endif
			}
			if (parallel_search) {
				//each thread process all layer on same tile
				vector<MarkObj> temp_vec;
				temp_vec.swap(QtConcurrent::blockingMappedReduced<vector<MarkObj>, vector<ProcessTileData> >(ptd_sets, process_tile, merge_tile_get_result,
					QtConcurrent::OrderedReduce | QtConcurrent::SequentialReduce));
				obj_sets.insert(obj_sets.end(), temp_vec.begin(), temp_vec.end());
			}
			else
			for (int i = 0; i < ptd_sets.size(); i++) {
				process_tile(ptd_sets[i]);
				//3 output result to obj_sets
				merge_tile_get_result(obj_sets, ptd_sets[i]);
			}

#if DEBUG_LEFT_TOP_IMG
			for (int i = 0; i < vwp.size(); i++)
				vwp[i].method &= ~OPT_DEBUG_EN;
#endif
		}
	}

#if SAVE_RST_TO_FILE
	FILE * fp;
	fp = fopen("result.txt", "w");
	for (int i = 0; i < obj_sets.size(); i++) {
		unsigned t = obj_sets[i].type;
		if (t == OBJ_POINT) {
			fprintf(fp, "via, l=%d, x=%d, y=%d\n", obj_sets[i].type3, obj_sets[i].p0.x(), obj_sets[i].p0.y());
		}
		else {
			fprintf(fp, "wire, l=%d, (x=%d,y=%d)->(x=%d,y=%d)\n", obj_sets[i].type3, obj_sets[i].p0.x(), obj_sets[i].p0.y(),
				obj_sets[i].p1.x(), obj_sets[i].p1.y());
		}
		continue;
	}
	fclose(fp);
#endif	

	for (int i = 0; i < obj_sets.size(); i++) {
		obj_sets[i].p0 = obj_sets[i].p0 * scale;
		obj_sets[i].p1 = obj_sets[i].p1 * scale;
	}
	qInfo("Extract finished successfully");
	qDebug("*#*#DumpMessage#*#*");
	return 0;
}

#include "vwextract3.h"

VWExtract * VWExtract::create_extract(int method)
{
	switch (method) {
	case 0:
		return new VWExtractPipe;
	case 1:
		return new VWExtractAnt;
	}
	return NULL;
}
