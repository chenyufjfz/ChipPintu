#include "vwextract.h"
#include <algorithm>
#include <functional>
#include <list>
#include <queue>  
#include <cfloat>
#include <set>
#include <stdio.h>
#include <QtConcurrent>
#include <QScopedPointer>
using namespace std;


//WEIGHT_FB is for find grid look forward & backward
#define WEIGHT_FB 3
//VIA_MAX_BIAS_PIXEL is max bias between via and grid, if exceed, warning message is generated
#define VIA_MAX_BIAS_PIXEL	3
//GRID_MAX_BIAS_PIXEL_FOR_LAYER is for grid finding, use up_layer or down_layer grid line as reference
#define GRID_MAX_BIAS_PIXEL_FOR_LAYER	2
//GRID_MAX_BIAS_PIXEL_FOR_LAYER is for grid finding, use left or up tile grid line as reference
#define GRID_MAX_BIAS_PIXEL_FOR_TILE	2
//GRID_COPY is grid info copy from left or up tile 
#define GRID_COPY			3
//PROB2_INT is for probability interger conver, should >=1000
#define PROB2_INT			10000
//BRICK_CHOOSE_NUM is How many brick candidate use
#define BRICK_CHOOSE_NUM	5
//CONFLICT_SEARCH_DEPTH is for fine adjust search length
#define CONFLICT_SEARCH_DEPTH	8
//EXTEND_VIA_FACTOR is for via & up_layer wire overlap
#define EXTEND_VIA_FACTOR	0.7
//PARALLEL if for Multithread vwextract
#define PARALLEL 1

#define BRICK_NO_WIRE		0
#define BRICK_i_0			1
#define BRICK_i_90			2
#define BRICK_i_180			3
#define BRICK_i_270			4
#define BRICK_I_0			5
#define BRICK_I_90			6
#define BRICK_L_0			7
#define BRICK_L_90			8
#define BRICK_L_180			9
#define BRICK_L_270			10
#define BRICK_T_0			11
#define BRICK_T_90			12
#define BRICK_T_180			13
#define BRICK_T_270			14
#define BRICK_X_0			15
#define BRICK_INVALID		255

#define DIR_UP				0
#define DIR_RIGHT			1
#define DIR_DOWN			2
#define DIR_LEFT			3

#define DIR_UP1_MASK		(1 << (DIR_UP * 2))
#define DIR_RIGHT1_MASK		(1 << (DIR_RIGHT * 2))
#define DIR_DOWN1_MASK		(1 << (DIR_DOWN * 2))
#define DIR_LEFT1_MASK		(1 << (DIR_LEFT * 2))
//following are used for half grid
#define DIR_UP2_MASK		(2 << (DIR_UP * 2))
#define DIR_RIGHT2_MASK		(2 << (DIR_RIGHT * 2))
#define DIR_DOWN2_MASK		(2 << (DIR_DOWN * 2))
#define DIR_LEFT2_MASK		(2 << (DIR_LEFT * 2))
#define DIR_UP3_MASK		(3 << (DIR_UP * 2))
#define DIR_RIGHT3_MASK		(3 << (DIR_RIGHT * 2))
#define DIR_DOWN3_MASK		(3 << (DIR_DOWN * 2))
#define DIR_LEFT3_MASK		(3 << (DIR_LEFT * 2))

#define VIA_MASK			(1 << 8)
#define UNSURE_SHIFT		9
#define UNSURE_MASK			(7 << UNSURE_SHIFT)
#define SAVE_RST_TO_FILE	1

#define SGN(x) (((x)>0) ? 1 : (((x)==0) ? 0 : -1))
#ifndef QT_DEBUG
#undef CV_Assert
#define CV_Assert(x) do {if (!(x)) {qFatal("Wrong at %s, %d", __FILE__, __LINE__);}} while(0)
#endif

enum {
	IGNORE_ALL_RULE,
	OBEY_RULE_WEAK,
	OBEY_RULE_STRONG,
	DO_FIND_TUNE
};

struct LayerParam{
	int wire_wd; //wire width
	int via_rd0; //outter via radius
	int via_rd1; //middle via radius
	int via_rd2; //inter via radius
	int via_method; //via extract method
	int grid_wd; //grid width	
	float param1; //via th, close to 1, higher threshold
	float param2; //wire th, close to 1, higher threshold
	float param3; //via_cred vs wire_cred, if via_cred> wire_cred, >1; else <1
	float param4; //via density
	unsigned long long rule; //rule affect bbfm
	unsigned long long warning_rule; //rule affect bbfm
};

//dxy[][0] is for y, dxy[][1] is for x
static const int dxy[4][2] = {
		{ -1, 0 }, //up
		{ 0, 1 }, //right
		{ 1, 0 }, //down
		{ 0, -1 } //left
};
static const int dxy8[8][2] = {
		{ -1, 0 }, //up
		{ -1, 1 }, //up right
		{ 0, 1 }, //right
		{ 1, 1 }, //down right
		{ 1, 0 }, //down
		{ 1, -1 }, //down left
		{ 0, -1 }, //left
		{ -1, -1 } //up left
};
static struct Brick {
	float priori_prob;
	int a[3][3];
	int shape;
} bricks[] = {
		{ 1.0f, {
				{ 0, 0, 0 },
				{ 0, 0, 0 },
				{ 0, 0, 0 } },
				0
		},

		{ 0.9f, {
				{ 0, 1, 0 },
				{ 0, 1, 0 },
				{ 0, 0, 0 } },
				DIR_UP1_MASK
		},

		{ 0.9f, {
				{ 0, 0, 0 },
				{ 0, 1, 1 },
				{ 0, 0, 0 } },
				DIR_RIGHT1_MASK
		},

		{ 0.9f, {
				{ 0, 0, 0 },
				{ 0, 1, 0 },
				{ 0, 1, 0 } },
				DIR_DOWN1_MASK
		},

		{ 0.9f, {
				{ 0, 0, 0 },
				{ 1, 1, 0 },
				{ 0, 0, 0 } },
				DIR_LEFT1_MASK
		},
		{ 1.0f, {
				{ 0, 1, 0 },
				{ 0, 1, 0 },
				{ 0, 1, 0 } },
				DIR_UP1_MASK | DIR_DOWN1_MASK
		},

		{ 1.0f, {
				{ 0, 0, 0 },
				{ 1, 1, 1 },
				{ 0, 0, 0 } },
				DIR_LEFT1_MASK | DIR_RIGHT1_MASK
		},

		{ 0.8f, {
				{ 0, 1, 0 },
				{ 0, 1, 1 },
				{ 0, 0, 0 } },
				DIR_UP1_MASK | DIR_RIGHT1_MASK
		},

		{ 0.8f, {
				{ 0, 0, 0 },
				{ 0, 1, 1 },
				{ 0, 1, 0 } },
				DIR_RIGHT1_MASK | DIR_DOWN1_MASK
		},

		{ 0.8f, {
				{ 0, 0, 0 },
				{ 1, 1, 0 },
				{ 0, 1, 0 } },
				DIR_DOWN1_MASK | DIR_LEFT1_MASK
		},

		{ 0.8f, {
				{ 0, 1, 0 },
				{ 1, 1, 0 },
				{ 0, 0, 0 } },
				DIR_UP1_MASK | DIR_LEFT1_MASK
		},

		{ 0.7f, {
				{ 0, 1, 0 },
				{ 0, 1, 1 },
				{ 0, 1, 0 } },
				DIR_UP1_MASK | DIR_RIGHT1_MASK | DIR_DOWN1_MASK
		},

		{ 0.7f, {
				{ 0, 0, 0 },
				{ 1, 1, 1 },
				{ 0, 1, 0 } },
				DIR_RIGHT1_MASK | DIR_DOWN1_MASK | DIR_LEFT1_MASK
		},

		{ 0.7f, {
				{ 0, 1, 0 },
				{ 1, 1, 0 },
				{ 0, 1, 0 } },
				DIR_UP1_MASK | DIR_LEFT1_MASK | DIR_DOWN1_MASK
		},

		{ 0.7f, {
				{ 0, 1, 0 },
				{ 1, 1, 1 },
				{ 0, 0, 0 } },
				DIR_UP1_MASK | DIR_LEFT1_MASK | DIR_RIGHT1_MASK
		},

		{ 0.5f, {
				{ 0, 1, 0 },
				{ 1, 1, 1 },
				{ 0, 1, 0 } },
				DIR_UP1_MASK | DIR_DOWN1_MASK | DIR_LEFT1_MASK | DIR_RIGHT1_MASK
		}
};

struct GridInfo {
	int x, y;
	int state; //only vaild for wire layer, 1 means fit adj brick, 0 means unfit adj brick
	float prob; //brick choose error probability, bigger more error, only vaild for wire layer
	vector<unsigned> brick_order; //prob & brick number, see MAKE_BRICK_ORDER, brick_order stores preprocess prob from image
	vector<int> brick_weight; //post process prob, see post_process_wire_via_prob
	int brick_choose; //brick currently chosed
	int brick_prefer; //brick preliminary election for fine adjust
	unsigned long long abm; //adj brick-allow mask
	GridInfo()
	{
		brick_choose = BRICK_INVALID;
		brick_prefer = BRICK_INVALID;
		abm = 0xffffffffffffffff;
	}
};

struct LayerBrickRuleMask {
	unsigned long long rule;
	bool via_extend_ovlap;
	unsigned long long wwfm[64][4]; //wire wire fit mask
	unsigned long long vwvfm[2][2]; //via(low_layer - wire -via(high_layer) fit mask, 
	unsigned long long fit_mask; //wire available mask
	void config(unsigned long long _rule);
};

struct LayerGridInfo {
	vector<GridInfo> grid_infos;
	int grid_rows, grid_cols; //equal to gl_y.size()-2, gl_x.size()-2
	GridInfo & at(int y, int x) {
		CV_Assert(y >= 1 && y <= grid_rows && x >= 1 && x <= grid_cols);
		return grid_infos.at((y - 1)*grid_cols + (x - 1));
	}
	const GridInfo & at(int y, int x) const {
		CV_Assert(y >= 1 && y <= grid_rows && x >= 1 && x <= grid_cols);
		return grid_infos.at((y - 1)*grid_cols + (x - 1));
	}
};

struct LayerTileData {
	Mat mark; //In nondebug mode, mark is for inout right image, In debug mode, mark is for show_mark
	Mat mark1; //In nondebug mode, mark1 is for inout down image, In debug mode, mark1 is for show_debug
	Mat img; //Input raw image to ProcessTile, after process, img is release to NULL
	Mat conet;
	vector<int> gl_x, gl_y;
};

struct TileData {
	int valid_grid_x0, valid_grid_y0; // it is conet grid zuobiao
	int img_grid_x0, img_grid_y0; // it is load image grid zuobiao for gl_x[0], gl_y[0]
	int img_pixel_x0, img_pixel_y0; // it is load image pixel zuobiao for gl_x[0], gl_y[0], so gl_y pixel = img_pixel_y0 + ts[idx].d[l].gl_y[i] + 2
	int img_cols, img_rows;
	int tile_x, tile_y;
	vector<LayerTileData> d; //image_num
	vector<LayerGridInfo> lg; // 2 * image_num-1
};

struct ProcessTileData {
	const TileData * ut;
	const TileData * lt;
	TileData * tt;
	vector<LayerBrickRuleMask> * lbrm;
	vector<LayerBrickRuleMask> * warn_lbrm;
	vector<LayerParam> * lpm;
	bool debug;
	int up_down_layer, left_right_layer;
};

class VWExtractStat : public VWExtract {
protected:
	/*ts is only used in extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets); not used in
	extract(vector<ICLayerWr *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets);*/
	vector<TileData> ts;
	vector<LayerParam> lpm;

public:
	int set_train_param(int layer, int width, int r, int rule_low, int warning_rule_low, int grid_width, int _param1, int _param2, int _param3, float _param4) {
		if (layer == 0)
			lpm.clear();
		if (layer > (int)lpm.size())
			return -1;
		if (layer == (int)lpm.size())
			lpm.push_back(LayerParam());
		lpm[layer].wire_wd = width;
		lpm[layer].via_rd0 = r & 0xff;
		lpm[layer].via_rd1 = (r >> 8) & 0xff;
		lpm[layer].via_rd2 = (r >> 16) & 0xff;
		lpm[layer].via_method = (r >> 24) & 0xff;
		lpm[layer].rule = rule_low;
		lpm[layer].warning_rule = warning_rule_low;
		lpm[layer].grid_wd = grid_width;
		lpm[layer].param1 = _param1 / 100.0;
		lpm[layer].param2 = _param2 / 100.0;
		lpm[layer].param3 = _param3 / 100.0;
		lpm[layer].param4 = _param4;
		return 0;
	}
	int set_extract_param(int layer, int width, int r, int rule_low, int warning_rule_low, int grid_width, int _param1, int _param2, int _param3, float _param4) {
		return set_train_param(layer, width, r, rule_low, warning_rule_low, grid_width, _param1, _param2, _param3, _param4);
	}
	Mat get_mark(int layer);
	Mat get_mark1(int layer);
	Mat get_mark2(int layer);
	Mat get_mark3(int layer);
	int train(string, const std::vector<MarkObj> &) { return 0; }
	int extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	int train(vector<ICLayerWrInterface *> &, const std::vector<MarkObj> &) { return 0; }
	int extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets);
	void get_feature(int, int, int, vector<float> &, std::vector<int> &) {}
};
/*
Use kmean 2 julei
in: bins, stat histogram
in: ratio, hign bin number ratio
out: th, julei center
out: var_win, julei window
return: <1, result is bad, more > 1, more good
*/
static float cal_threshold(const vector<unsigned> & bins, vector<float> & th, vector<float> & var_win, float ratio = 0)
{
	unsigned total1 = 0, sep, total2 = 0, old_sep = 0xffffffff;
	double avg = 0;
	if (ratio == 0) {
		for (sep = 0; sep < bins.size(); sep++) {
			total1 += bins[sep];
			avg += bins[sep] * sep;
		}
		sep = avg / total1;
	}
	else {
		for (sep = 0; sep < bins.size(); sep++)
			total1 += bins[sep];
		total1 = total1 * ratio;
		for (sep = 0; sep < bins.size(); sep++) {
			total2 += bins[sep];
			if (total2 >= total1)
				break;
		}
	}

	CV_Assert(sep < bins.size());
	double m1_l, m1_r, d_l, d_r;
	while (sep != old_sep) {
		old_sep = sep;
		m1_l = 0, m1_r = 0, d_l = 0, d_r = 0;
		total1 = 0, total2 = 0;
		for (unsigned j = 0; j < sep; j++) {
			m1_l += bins[j] * j;
			d_l += bins[j] * j * j;
			total1 += bins[j];
		}
		for (unsigned j = sep; j < bins.size(); j++) {
			m1_r += bins[j] * j;
			d_r += bins[j] * j * j;
			total2 += bins[j];
		}
		m1_l = m1_l / total1;
		d_l = d_l / total1;
		m1_r = m1_r / total2;
		d_r = d_r / total2;
		sep = (m1_l + m1_r) / 2;
	}
	th.clear();
	th.push_back(m1_l);
	th.push_back(m1_r);
	CV_Assert(d_l >= m1_l * m1_l && d_r >= m1_r*m1_r);
	var_win.clear();
	var_win.push_back(sqrt(d_l - m1_l * m1_l));
	var_win.push_back(sqrt(d_r - m1_r * m1_r));
	if (m1_r - m1_l < var_win[0] + var_win[1])
		qWarning("Bad, threshold difference %f is smaller than deviation %f", m1_r - m1_l, var_win[0] + var_win[1]);
	return (m1_r - m1_l) / (var_win[0] + var_win[1]);
}

bool intersect(const QLine &line, const QRect &rect, QLine &inter)
{
	QPoint p1(line.p1());
	QPoint p2(line.p2());
	if (max(p1.x(), p2.x()) <= rect.left() || min(p1.x(), p2.x()) >= rect.right() ||
		max(p1.y(), p2.y()) <= rect.top() || min(p1.y(), p2.y()) >= rect.bottom())
		return false;
	if (p1.x() > rect.right())
		p1.setX(rect.right());
	if (p1.x() < rect.left())
		p1.setX(rect.left());
	if (p1.y() > rect.bottom())
		p1.setY(rect.bottom());
	if (p1.y() < rect.top())
		p1.setY(rect.top());
	if (p2.x() > rect.right())
		p2.setX(rect.right());
	if (p2.x() < rect.left())
		p2.setX(rect.left());
	if (p2.y() > rect.bottom())
		p2.setY(rect.bottom());
	if (p2.y() < rect.top())
		p2.setY(rect.top());
	inter.setPoints(p1, p2);
	return true;
}

Rect toRect(QRect &r)
{
	return Rect(r.x(), r.y(), r.width(), r.height());
}

QRect toQRect(Rect &r)
{
	return QRect(r.x, r.y, r.width, r.height);
}

static void fill_circle(Mat & mark, int x0, int y0, int r, unsigned char v, const QRect & rect)
{
	CV_Assert(mark.type() == CV_8UC1);

	for (int y = max(rect.top(), y0 - r); y <= min(y0 + r, rect.bottom()); y++) {
		int dx = (int)sqrt((float)r*r - (y - y0) *(y - y0));
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), x0 - dx); x <= min(x0 + dx, rect.right()); x++)
			p_mark[x] = v;
	}
}

static void fill_circle_check(Mat & mark, int x0, int y0, int r, unsigned char v, QRect & rect, unsigned long long forbid_mark)
{
	CV_Assert(mark.type() == CV_8UC1);

	for (int y = max(rect.top(), y0 - r); y <= min(y0 + r, rect.bottom()); y++) {
		int dx = (int)sqrt((float)r*r - (y - y0) *(y - y0));
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), x0 - dx); x <= min(x0 + dx, rect.right()); x++)
			if (!((forbid_mark >> p_mark[x]) & 1))
				p_mark[x] = v;
	}
}

static void fill_rect(Mat & mark, QPoint lt, QPoint rb, unsigned char v, int eu, int ed, int el, int er, QRect & rect)
{
	for (int y = max(rect.top(), lt.y() - eu); y <= min(rect.bottom(), rb.y() + ed); y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), lt.x() - el); x <= min(rect.right(), rb.x() + er); x++)
			p_mark[x] = v;
	}
}

static void fill_rect_check(Mat & mark, QPoint lt, QPoint rb, unsigned char v, int eu, int ed, int el, int er,
	QRect & rect, unsigned long long forbid_mark)
{
	for (int y = max(rect.top(), lt.y() - eu); y <= min(rect.bottom(), rb.y() + ed); y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), lt.x() - el); x <= min(rect.right(), rb.x() + er); x++)
			if (!((forbid_mark >> p_mark[x]) & 1))
				p_mark[x] = v;
	}
}

/*
Return Vec[0], Vec[1] is up down grad
Vec[2], Vec[3] is left right grad
Vec[4] is via grad
*/
static Vec<float, 5> feature_extract_5(const unsigned char * a, int lsize, int w1, int w2, int r, vector<int> dx)
{
	int s[6] = { 0 };
	int mmin[4] = { 0xfffffff };
	int mmax[4] = { 0 };

	CV_Assert(w1 >= w2);
	const unsigned char * b;
	for (int y = 0, i = 0; y < 4; y++) {
		int w = (y == 0 || y == 3) ? w2 : w1;
		b = a + lsize * (y - 1) - w / 2;
		if (y == 2)
			i++;
		for (int x = 0; x < w; x++) {
			s[i] += b[x];
			mmin[i] = min(mmin[i], (int)b[x]);
			mmax[i] = max(mmax[i], (int)b[x]);
		}
	}

	b = a - lsize * (w1 / 2);
	for (int y = 0; y < w1; y++, b += lsize) {
		s[2] += b[0];
		mmin[2] = min(mmin[2], (int)b[0]);
		mmax[2] = max(mmax[2], (int)b[0]);

		s[3] += b[1];
		mmin[3] = min(mmin[3], (int)b[1]);
		mmax[3] = max(mmax[3], (int)b[1]);
	}

	b = a - lsize * (w2 / 2) - 1;
	for (int y = 0; y < w2; y++, b += lsize) {
		s[2] += b[0];
		mmin[2] = min(mmin[2], (int)b[0]);
		mmax[2] = max(mmax[2], (int)b[0]);

		s[3] += b[3];
		mmin[3] = min(mmin[3], (int)b[3]);
		mmax[3] = max(mmax[3], (int)b[3]);
	}
	for (int y = -r, i = 0; y <= r; y++, i++) {
		//int dx = (int)sqrt((float)r*r - y *y);
		b = a + lsize * y;
		for (int x = -dx[i]; x <= dx[i]; x++)
			s[4] += b[x];
	}

	for (int y = -r - 1; y <= r + 1; y++) {
		b = a + lsize * y;
		for (int x = -r - 1; x <= r + 1; x++)
			s[5] += b[x];
	}

	for (int i = 0; i < 4; i++)
		s[i] = s[i] - mmin[i] - mmax[i];
#if 0	
	return Vec<float, 5>((float)log((s[1] + 0.001f) / (s[0] + 0.001f)) * 10,
		s[1] - s[0],
		(float)log((s[3] + 0.001f) / (s[2] + 0.001f)) * 10,
		s[3] - s[2],
		s[4] / (s[4] + 2 * (s[5] - s[4]) + 0.001f));
#else
	float d10 = s[1] - s[0];
	float d32 = s[3] - s[2];
	float r10 = log((s[1] + 0.001f) / (s[0] + 0.001f)) * 10;
	float r32 = log((s[3] + 0.001f) / (s[2] + 0.001f)) * 10;

	return Vec<float, 5>(r10 + SGN(r10) * fabs(r32) / 2,
		d10 + SGN(d10) * fabs(d32) / 2,
		r32 + SGN(r32) * fabs(r10) / 2,
		d32 + SGN(d32) * fabs(d10) / 2,
		s[4] / (s[4] + 2 * (s[5] - s[4]) + 0.001f));
#endif
}
//gl_edge[2 * i]+1 to gl_edge[2 * i + 1] is same wire or same insu
void compute_grid_edge(const vector<int> & gl, int wire_wd, vector<int> & gl_edge)
{
	gl_edge.resize(gl.size() * 2);

	for (unsigned i = 0; i < gl.size(); i++) {
		gl_edge[2 * i] = gl[i] - wire_wd / 2; //gl_edge[2 * i] is insu
		gl_edge[2 * i + 1] = gl_edge[2 * i] + wire_wd; //gl_edge[2 * i + 1] is wire
	}
}
/*
In img: raw image
In lpm.wire_wd: metal width
lpm.grid_wd: grid width
lpm.param2, wire_cred, wire reliablity
In gl_x: up-down grid line
In gl_y: left-right grid line
In mask: available brick mask
out prob: brick prob for each grid, 3-dim Mat (gl_y.size() * gl_x.size() * Brick_NUM),
more near to 1 means bigger probability for the brick,
more near to 0 means less probability for the brick.
Features and Restriction
1 Assume wire is lighter than insu.
2 Assume at least 3 insu grid within 3*3
*/
static void compute_grid_prob(const Mat & img, struct LayerParam & lpm, const vector<int> & gl_x, const vector<int> & gl_y,
	unsigned long long mask, Mat & prob, Mat & mark, bool detect_dummy_fill = false)
{
	int wire_wd = lpm.wire_wd, grid_wd = lpm.grid_wd, wire_cred = lpm.param2;
	CV_Assert(grid_wd > wire_wd && wire_cred <= 1 && wire_cred >= 0 && gl_x[0] >= wire_wd / 2 && gl_y[0] >= wire_wd / 2);
	CV_Assert(gl_x[gl_x.size() - 1] <= img.cols - wire_wd + wire_wd / 2 - 2);
	CV_Assert(gl_y[gl_y.size() - 1] <= img.rows - wire_wd + wire_wd / 2 - 2);
	CV_Assert(mark.empty() || mark.type() == CV_8UC1 && mark.size == img.size);

	Mat ig, mlb; //maximum like brick
	integral(img, ig, CV_32S);
	int sz[] = { (int)gl_y.size(), (int)gl_x.size(), (int) sizeof(bricks) / sizeof(bricks[0]) };
	prob.create(3, sz, CV_32F);
	prob = Scalar(0);
	mlb.create((int)gl_y.size(), (int)gl_x.size(), CV_8UC1);

	//1 compute each grid average gray
	vector<int> glx_edge, gly_edge;
	vector<unsigned> bins(512, 0);
	compute_grid_edge(gl_x, wire_wd, glx_edge);
	compute_grid_edge(gl_y, wire_wd, gly_edge);
	Mat gray((int)gl_y.size() * 2 - 1, (int)gl_x.size() * 2 - 1, CV_32FC1);

	for (int y = 0; y < gray.rows; y++)
		for (int x = 0; x < gray.cols; x++) {
		//(gly_edge[y]+1, gly_edge[x]+1) (gly_edge[y+1],gly_edge[x+1]) 
		float sum = ig.at<int>(gly_edge[y] + 1, glx_edge[x] + 1) + ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x + 1] + 1) -
			ig.at<int>(gly_edge[y] + 1, glx_edge[x + 1] + 1) - ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x] + 1);
		float avg = sum / ((gly_edge[y + 1] - gly_edge[y]) * (glx_edge[x + 1] - glx_edge[x]));
		gray.at<float>(y, x) = avg;
		bins[avg * 2]++;
		}

	//2 Auto compute gray level, th[0] is insu gray, th[1] is wire gray
	vector<float> th, var_win;
	cal_threshold(bins, th, var_win);
	th[0] = th[0] / 2, th[1] = th[1] / 2;
	var_win[0] = var_win[0] / 2, var_win[1] = var_win[1] / 2;
	float wg_th = th[1] - th[0] + (var_win[0] + var_win[1]) * wire_cred;
	qDebug("Grid gray low (a=%f,w=%f), high (a=%f,w=%f), choose wire gray th=%f",
		th[0], var_win[0], th[1], var_win[1], wg_th);

#define PUSH_T_MIN(t, m, s, r) do \
		{   if (m > t) \
				{ r = s; s = m; m = t; } \
				else \
			if (s > t) \
							{r = s; s = t;} \
						else \
				if (r > t) \
					r = t; \
		} while (0)

	//3 compute brick prob for each grid	
	for (int y = 1; y < (int)gl_y.size() - 1; y++)
		for (int x = 1; x < (int)gl_x.size() - 1; x++) {
		float m = 100000, s = 100000, t0 = 100000;
		int x0 = 2 * x, y0 = 2 * y;
		//3.1 find 3rd minimum gray to determine bottom gray for insu
		for (int yy = y0 - 1; yy <= y0 + 1; yy++)
			for (int xx = x0 - 1; xx <= x0 + 1; xx++) {
			float t = gray.at<float>(yy, xx);
			//Here assume at least 3 insu grid within 3*3
			PUSH_T_MIN(t, m, s, t0);
			}
		double alpha[3][3] = { 0 }; //alpha store p(grid_is_wire | img), near to 1 like wire, near to 0 like insu
		//3.2 Do point by point gray compare to compute grid wire probability
		for (int yy = y0 - 1; yy <= y0 + 1; yy++)
			for (int xx = x0 - 1; xx <= x0 + 1; xx++) {
			float sw = 0;
			for (int y1 = gly_edge[yy] + 1; y1 <= gly_edge[yy + 1]; y1++) {
				const unsigned char * p_img = img.ptr<unsigned char>(y1);
				for (int x1 = glx_edge[xx] + 1; x1 <= glx_edge[xx + 1]; x1++) {
					int gray = p_img[x1];
					if (gray >= t0 + wg_th)
						sw += 1;
					else
						if (gray > t0)
							sw += (float)(gray - t0) / wg_th;
				}
			}
			alpha[yy - y0 + 1][xx - x0 + 1] = (double)sw / ((gly_edge[yy + 1] - gly_edge[yy]) * (glx_edge[xx + 1] - glx_edge[xx]));
			}
		//Do some post process to integrate grad
		alpha[0][0] = alpha[0][0] / 1.05;
		alpha[0][2] = alpha[0][2] / 1.05;
		alpha[2][0] = alpha[2][0] / 1.05;
		alpha[2][2] = alpha[2][2] / 1.05;
		if (!mark.empty()) {
			for (int yy = 0; yy < 3; yy++)
				for (int xx = 0; xx < 3; xx++) {
				CV_Assert(alpha[yy][xx] <= 1 && alpha[yy][xx] >= 0);
				unsigned char color = alpha[yy][xx] * 255;
				mark(Rect(gl_x[x] + xx * 3 - 4, gl_y[y] + yy * 3 - 4, 3, 3)) = color;
				}
		}
		//3.3 Compute grad			
		float glr[3][2], gud[2][3];
		for (int yy = 0; yy < 3; yy++)
			for (int xx = 0; xx < 2; xx++) {
			glr[yy][xx] = (gray.at<float>(yy + y0 - 1, xx + x0) - gray.at<float>(yy + y0 - 1, xx + x0 - 1)) / wg_th;
			if (abs(glr[yy][xx]) > 1)
				glr[yy][xx] = SGN(glr[yy][xx]);
			}


		for (int yy = 0; yy < 2; yy++)
			for (int xx = 0; xx < 3; xx++) {
			gud[yy][xx] = (gray.at<float>(yy + y0, xx + x0 - 1) - gray.at<float>(yy + y0 - 1, xx + x0 - 1)) / wg_th;
			if (abs(gud[yy][xx]) > 1)
				gud[yy][xx] = SGN(gud[yy][xx]);
			}


		//3.4 Compute each brick likelihood probability
		vector<double> brick_prob(sizeof(bricks) / sizeof(bricks[0])); // store p(grid | brick)
		double normal = 0; //store p(grid) =Sum{p(grid | brick) * p(brick)}
		for (unsigned i = 0; i < brick_prob.size(); i++)
			if (mask & (1ULL << i)) {
			double eng = FLT_MIN; //energy needed to transform the brick to grid image
			for (int yy = 0; yy < 3; yy++)
				for (int xx = 0; xx < 3; xx++) {
				double t = abs(alpha[yy][xx] - bricks[i].a[yy][xx]);
				eng += t;
				}
			for (int yy = 0; yy < 3; yy++)
				for (int xx = 0; xx < 2; xx++) {
				double t = abs(glr[yy][xx] - (bricks[i].a[yy][xx + 1] - bricks[i].a[yy][xx]));
				eng += t;
				}

			for (int yy = 0; yy < 2; yy++)
				for (int xx = 0; xx < 3; xx++) {
				double t = abs(gud[yy][xx] - (bricks[i].a[yy + 1][xx] - bricks[i].a[yy][xx]));
				eng += t;
				}

			brick_prob[i] = 1 / eng; //if engrgy big, prob is small
			normal += brick_prob[i] * bricks[i].priori_prob;
			}
			else
				brick_prob[i] = -1;
		CV_Assert(normal != 0);
		float * p_prob = prob.ptr<float>(y, x); //prob store p(brick |grid)
		unsigned char * p_mlb = mlb.ptr<unsigned char>(y, x);
		float max_prob = 0;
		//3.5 Compute posteriori with BAYES method
		for (unsigned i = 0; i < brick_prob.size(); i++)
			if (mask & (1ULL << i)) {
			p_prob[i] = brick_prob[i] * bricks[i].priori_prob / normal; //p(grid | brick) * p(brick) / p(grid)
			if (p_prob[i] > max_prob) {
				max_prob = p_prob[i];
				p_mlb[0] = i;
			}
			}
			else
				p_prob[i] = 0;
		}
	//Find dummy filling 
	if (detect_dummy_fill) {
#define EDGE_DETECT_SIZE 3
		Mat e0((int)gl_x.size(), img.rows, CV_32FC1); //up_down grad
		Mat e1((int)gl_y.size(), img.cols, CV_32FC1); //left_right grad
		Mat a0((int)gl_y.size(), (int)gl_x.size(), CV_32FC1);
		Mat a1((int)gl_y.size(), (int)gl_x.size(), CV_32FC1);
		//compute up_down and left_right grad
		for (int x = 1; x < (int)gl_x.size() - 1; x++) {
			int x0 = (gl_x[x] + gl_x[x - 1]) / 2;
			int x1 = (gl_x[x] + gl_x[x + 1]) / 2;
			float area = (x1 - x0) * EDGE_DETECT_SIZE;
			for (int y = EDGE_DETECT_SIZE; y <= img.rows - EDGE_DETECT_SIZE; y++)
				//Rect((y,x0),(y+2,x1-1)) - Rect((y-3,x0), (y-1,x1-1))
				e0.at<float>(x, y) = (ig.at<int>(y + EDGE_DETECT_SIZE, x1) - ig.at<int>(y + EDGE_DETECT_SIZE, x0) +
				2 * (ig.at<int>(y, x0) - ig.at<int>(y, x1)) +
				ig.at<int>(y - EDGE_DETECT_SIZE, x1) - ig.at<int>(y - EDGE_DETECT_SIZE, x0)) / area;
		}
		for (int y = 1; y < (int)gl_y.size() - 1; y++) {
			int y0 = (gl_y[y] + gl_y[y - 1]) / 2;
			int y1 = (gl_y[y] + gl_y[y + 1]) / 2;
			float area = (y1 - y0) * EDGE_DETECT_SIZE;
			for (int x = EDGE_DETECT_SIZE; x <= img.cols - EDGE_DETECT_SIZE; x++)
				//Rect((y0,x),(y1-1,x+2)) - Rect((y0,x-3),(y1-1,x-1))
				e1.at<float>(y, x) = (ig.at<int>(y1, x + EDGE_DETECT_SIZE) - ig.at<int>(y0, x + EDGE_DETECT_SIZE) +
				2 * (ig.at<int>(y0, x) - ig.at<int>(y1, x)) +
				ig.at<int>(y1, x - EDGE_DETECT_SIZE) - ig.at<int>(y0, x - EDGE_DETECT_SIZE)) / area;
		}
		a0 = Scalar(0);
		a1 = Scalar(0);
		for (int y = 1; y < (int)gl_y.size() - 1; y++)
			for (int x = 1; x < (int)gl_x.size() - 1; x++) {
				unsigned char cb = mlb.at<unsigned char>(y, x);
				int max_loc, d, min_loc, search_end;
				float maxe, min_e, *pg;
				pg = e1.ptr<float>(y);
				maxe = 0;
				for (int i = gl_x[x] - wire_wd / 2 - 1; i <= gl_x[x] + wire_wd / 2 + 1; i++)
					if (abs(pg[i]) > maxe) { //find max grad for edge detect
					maxe = abs(pg[i]);
					max_loc = i;
					}
				d = (pg[max_loc] > 0) ? 1 : -1;
				if (maxe > wg_th / 2 && (x + d == 0 || x + d == gl_x.size() - 2 ||
					mlb.at<unsigned char>(y, x + d) == BRICK_NO_WIRE)) { //If detect edge, and adj grid is BRICK_NO_WIRE
					search_end = (pg[max_loc] > 0) ? min(gl_x[x] + grid_wd * 2, img.cols - EDGE_DETECT_SIZE) : max(gl_x[x] - grid_wd * 2, EDGE_DETECT_SIZE);
					min_e = 100000 * d; //search another edge
					min_loc = -1;
					for (int i = max_loc, j = 0; i != search_end; i += d, j++)
						if ((pg[max_loc] + pg[i]) * d < min_e * d && j >= EDGE_DETECT_SIZE * 2) { //find max inverse grad as another edge
						min_e = pg[max_loc] + pg[i];
						min_loc = i;
						}
					float alpha = pg[max_loc] * min_e / (wg_th * wg_th);
					if (alpha > 0.43f && abs(pg[min_loc]) < wg_th / 2 && maxe > 3 * abs(pg[min_loc]) && min_loc >= 0) { //0.43f wg_th/2, 3 * pg is my guess number
						float p0 = prob.at<float>(y, x, BRICK_NO_WIRE);							//If another edge is smooth enough, found dummy filling
						float p1 = (x + d == 0 || x + d == gl_x.size() - 2) ? prob.at<float>(y, x, cb) + 0.02f : 
							max(prob.at<float>(y, x, cb) + 0.02f, prob.at<float>(y, x + d, BRICK_NO_WIRE)); //max BRICK_NO_WIRE probability is adj BRICK_NO_WIRE's probability					
						alpha = min(alpha *1.5f, 1.0f);
						a1.at<float>(y, x) = alpha * p1 + (1 - alpha) *p0; //increase BRICK_NO_WIRE probability.
					}
				}

				pg = e0.ptr<float>(x);
				maxe = 0;
				for (int i = gl_y[y] - wire_wd / 2 - 1; i <= gl_y[y] + wire_wd / 2 + 1; i++)
					if (abs(pg[i]) > maxe) { //find max grad for edge detect
					maxe = abs(pg[i]);
					max_loc = i;
					}
				d = (pg[max_loc] > 0) ? 1 : -1;
				if (maxe > wg_th / 2 && (y + d == 0 || y + d == gl_y.size() - 2 ||
					mlb.at<unsigned char>(y + d, x) == BRICK_NO_WIRE)) { //If detect edge, and adj grid is BRICK_NO_WIRE
					search_end = (pg[max_loc] > 0) ? min(gl_y[y] + grid_wd * 2, img.rows - EDGE_DETECT_SIZE) : max(gl_y[y] - grid_wd * 2, EDGE_DETECT_SIZE);
					min_e = 100000 * d; //search another edge
					min_loc = -1;
					for (int i = max_loc, j = 0; i != search_end; i += d, j++)
						if ((pg[max_loc] + pg[i]) * d < min_e * d && j >= EDGE_DETECT_SIZE * 2) { //find max inverse grad as another edge
						min_e = pg[max_loc] + pg[i];
						min_loc = i;
						}
					float alpha = pg[max_loc] * min_e / (wg_th * wg_th); //min_e bigger -> abs(pg[min_loc]) smaller
					if (alpha > 0.43f && abs(pg[min_loc]) < wg_th / 2 && maxe > 3 * abs(pg[min_loc]) && min_loc >= 0) { //0.43f wg_th/2, 3 * pg is my guess number							
						float p0 = prob.at<float>(y, x, BRICK_NO_WIRE);						//If another edge is smooth enough, found dummy filling
						float p1 = (y + d == 0 || y + d == gl_y.size() - 2) ? prob.at<float>(y, x, cb) + 0.002f :
							max(prob.at<float>(y, x, cb) + 0.002f, prob.at<float>(y + d, x, BRICK_NO_WIRE)); //max BRICK_NO_WIRE probability is adj BRICK_NO_WIRE's probability					
						alpha = min(alpha *1.5f, 1.0f);
						a0.at<float>(y, x) = alpha * p1 + (1 - alpha) *p0; //increase BRICK_NO_WIRE probability.
					}
				}
			}
		//increase BRICK_NO_WIRE probability for dummy filling
		for (int y = 1; y < (int)gl_y.size() - 1; y++)
			for (int x = 1; x < (int)gl_x.size() - 1; x++) {
			unsigned char cb = mlb.at<unsigned char>(y, x);
			switch (cb) {
			case BRICK_I_0:
				if (a1.at<float>(y, x) > prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a1.at<float>(y, x);
				break;
			case BRICK_I_90:
				if (a0.at<float>(y, x) > prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a0.at<float>(y, x);
				break;
			case BRICK_L_0:
				if (a0.at<float>(y, x + 1) > prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a0.at<float>(y, x + 1);
				if (a1.at<float>(y - 1, x) > prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a1.at<float>(y - 1, x);
				break;
			case BRICK_L_90:
				if (a0.at<float>(y, x + 1) >  prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a0.at<float>(y, x + 1);
				if (a1.at<float>(y + 1, x) > prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a1.at<float>(y + 1, x);
				break;
			case BRICK_L_180:
				if (a0.at<float>(y, x - 1) >  prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a0.at<float>(y, x - 1);
				if (a1.at<float>(y + 1, x) > prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a1.at<float>(y + 1, x);
				break;
			case BRICK_L_270:
				if (a0.at<float>(y, x - 1) >  prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a0.at<float>(y, x - 1);
				if (a1.at<float>(y - 1, x) > prob.at<float>(y, x, BRICK_NO_WIRE))
					prob.at<float>(y, x, BRICK_NO_WIRE) = a1.at<float>(y - 1, x);
				break;
			}
			}
	}
#undef PUSH_T_MIN
#undef EDGE_DETECT_SIZE
}

/*
inout: img input: image raw data
output: image data which remove via
input: mark, via is 2 others is 0
input: w insulator width, used to compute gray of points which is deleted via
*/
static void remove_via_line(const vector<unsigned char> & mark, vector<unsigned char> & img, int w)
{
	CV_Assert(img.size() == mark.size());

	for (int i = w; i < (int)img.size(); i++) {
		if (mark[i] == 2) {
			int len = 0;
			bool tail_good;
			do {
				CV_Assert(mark[i + len] == 2);
				for (; i + len < (int)img.size() && mark[i + len] == 2; len++);
				if (i + len + w >= (int)img.size())
					return;
				tail_good = true;
				for (int j = len + i + 1; j < len + i + w; j++)
					if (mark[j] == 2) {
					tail_good = false;
					len = j - i;
					break;
					}
			} while (!tail_good);

			CV_Assert(mark[i + len] != 2);
			int a = 0, b = 0;
			for (int j = 1; j <= w; j++) {
				a += img[i - j];
				b += img[i + len + j - 1];
			}
			a = a / w;
			b = b / w;
			for (int j = 0; j < len / 2; j++)
				img[i + j] = a;
			for (int j = len / 2 + 1; j < len; j++)
				img[i + j] = b;
			img[i + len / 2] = (a + b) / 2;
			i += len;
		}
	}
}

/*
input mark, via is 2 others is 0
inout img, input: image raw data
output: image data which remove via
in wire_up_down, 0 is shuxian rule, 1 is hengxian rule.
input: w insulator width, used to compute gray of points which is deleted via
*/
static void remove_via(const Mat & mark, Mat & img, int wire_up_down, int w)
{
	CV_Assert(wire_up_down < 2 && mark.rows == img.rows && mark.cols == img.cols);
	CV_Assert(mark.type() == CV_8UC1 && img.type() == CV_8UC1);

	if (wire_up_down == 0) {
		vector<unsigned char> img_line(img.rows);
		vector<unsigned char> mark_line(img.rows);
		for (int x = 0; x < mark.cols; x++) {
			for (int y = 0; y < mark.rows; y++) {
				img_line[y] = img.at<unsigned char>(y, x);
				mark_line[y] = (unsigned char)mark.at<unsigned char>(y, x);
			}
			remove_via_line(mark_line, img_line, w);
			for (int y = 0; y < mark.rows; y++)
				img.at<unsigned char>(y, x) = img_line[y];
		}
	}
	else {
		vector<unsigned char> img_line(img.cols);
		vector<unsigned char> mark_line(img.cols);
		for (int y = 0; y < mark.rows; y++) {
			unsigned char * p_img = img.ptr<unsigned char>(y);
			const unsigned char * p_mark = mark.ptr<unsigned char>(y);
			for (int x = 0; x < mark.cols; x++) {
				img_line[x] = p_img[x];
				mark_line[x] = p_mark[x];
			}
			remove_via_line(mark_line, img_line, w);
			for (int x = 0; x < mark.cols; x++)
				p_img[x] = img_line[x];
		}
	}
}

/*
Compute feature in rect [-r, r] * [-r, r]
In: a, image pointer
In: lsize, image.col
In: r, Via radius
Out: feature
*/
static void feature_extract_via(const unsigned char * a, int lsize, int r, float * feature)
{
	for (int i = 0; i < 4; i++) {
		int x0, y0, x1, y1, s0 = 0, s1 = 0;
		switch (i) {
		case 0:
			x0 = -r;
			x1 = -r / 3;
			y0 = -r;
			y1 = -r / 3;
			break;
		case 1:
			x0 = r / 3;
			x1 = r;
			y0 = -r;
			y1 = -r / 3;
			break;
		case 2:
			x0 = r / 3;
			x1 = r;
			y0 = r / 3;
			y1 = r;
			break;
		case 3:
			x0 = -r;
			x1 = -r / 3;
			y0 = r / 3;
			y1 = r;
			break;
		}
		const unsigned char * center = a + lsize*((y0 + y1) / 2) + (x0 + x1) / 2;
		switch (i) {
		case 0:
			s1 += center[0] + center[lsize - 1] + center[1 - lsize];
			break;
		case 1:
			s1 += center[0] + center[lsize + 1] + center[-1 - lsize];
			break;
		case 2:
			s1 += center[0] + center[lsize - 1] + center[1 - lsize];
			break;
		case 3:
			s1 += center[0] + center[lsize + 1] + center[-1 - lsize];
			break;
		}
		for (int y = y0; y <= y1; y++) {
			const unsigned char * b = a + lsize * y;
			for (int x = x0; x <= x1; x++) {
				s0 += b[x];
				if ((i == 0) && (y + x > y1 + x0) || (i == 2) && (y + x < y1 + x0) ||
					(i == 1) && (x - y < x0 - y0) || (i == 3) && (x - y > x0 - y0))
					s1 += b[x];
			}
		}
		feature[i] = (float)s1 / max((float)s0 - s1, 0.000001f);
	}
}

/*
In:img
In: r, Via radius
Out: ia, line integrate
Out: stat, gray stat for each 32*32 grid
Out: dx, circle x
*/
static void feature_extract_via2_prepare(const Mat & img, int r, int r1, Mat & ia, Mat & stat, vector<int> & dx, vector<int> & dx1)
{
	CV_Assert(img.type() == CV_8UC1);
	int sz[] = { img.rows / 32 + 1, img.cols / 32 + 1, 128 };
	stat.create(3, sz, CV_32S);
	stat = Scalar(0);
	ia.create(img.rows, img.cols + 1, CV_32SC1);

	for (int y = 0; y < img.rows; y++) {
		const unsigned char * p_img = img.ptr<unsigned char>(y);
		int * p_ia = ia.ptr<int>(y);
		p_ia[0] = 0;
		for (int x = 0; x < img.cols; x++) {
			p_ia[x + 1] = p_ia[x] + p_img[x];
			int * p_stat = stat.ptr<int>(y >> 5, x >> 5);
			if (p_img[x] < 254)
				p_stat[p_img[x] / 2]++;
			else
				p_stat[126]++;
			p_stat[127]++;
		}
	}

	dx.resize(r + 1);
	for (int i = 0; i <= r; i++)
		dx[i] = sqrt(r * r - i * i);
	dx1.resize(r1 + 1);
	for (int i = 0; i <= r1; i++)
		dx1[i] = sqrt(r1 * r1 - i * i);
}

/*
In, x0, y0
In, ia, feature_extract_via2_prepare's output
In, stat, feature_extract_via2_prepare's output
In, r, via radius
In, dx, feature_extract_via2_prepare's output
Out feature,
feature_extract_via2 suppose via is lighter than insu, it compute two concentric circles gray
*/
static void feature_extract_via2(int x0, int y0, const Mat & ia, const Mat & stat, int r, int r1,
	const vector<int> & dx, const vector<int> & dx1, float feature[], int & last_xx, int &last_yy, int &last_gray)
{
	CV_Assert((int)dx.size() == r + 1 && (int)dx1.size() == r1 + 1);
	int s = 0, n = 0, gray, s1 = 0, n1 = 0;
	for (int y = -r; y <= r; y++) {
		int x = dx[abs(y)];
		s += ia.at<int>(y + y0, x + x0 + 1) - ia.at<int>(y + y0, x0 - x);
		n += x * 2 + 1;
	}
	for (int y = -r1; y <= r1; y++) {
		int x = dx1[abs(y)];
		s1 += ia.at<int>(y + y0, x + x0 + 1) - ia.at<int>(y + y0, x0 - x);
		n1 += x * 2 + 1;
	}
	int yy = max((y0 >> 5) - 1, 0);
	int xx = max((x0 >> 5) - 1, 0);
	yy = min(yy, stat.size[0] - 3);
	xx = min(xx, stat.size[1] - 3);
	vector<int> sum(stat.size[2], 0);
	//Compute insu gray
	if (xx != last_xx || yy != last_yy) {
		for (int y = yy; y <= yy + 2; y++)
			for (int x = xx; x <= xx + 2; x++) {
			const int * p_stat = stat.ptr<int>(y, x);
			for (int i = 0; i < stat.size[2]; i++)
				sum[i] += p_stat[i];
			}
		int th = sum.back() / 10; //suppose insu occupy at least 1/10 area
		for (int i = 0; i < (int)sum.size(); i++) {
			th -= sum[i];
			if (th < 0) {
				gray = i;
				break;
			}
		}
		gray = max(1, gray);
		last_xx = xx;
		last_yy = yy;
		last_gray = gray;
	}
	else
		gray = last_gray;
	feature[0] = (float)s / (n * gray);
	feature[1] = (float)s1 / (n1 * gray);
}


class FeatureExtractVia {
protected:
	int r0, r1, r2;
public:
	static FeatureExtractVia * create_feaext_via(int method);
	virtual void prepare(Mat & img, int r0, int r1, int r2) = 0;
	virtual unsigned compute_feature(int x, int y) = 0;
	virtual ~FeatureExtractVia() {
	}
};

//The via r0 is nearly same as wire's width
class BigViaExtract : public FeatureExtractVia {
protected:
	Mat img;
public:
	void prepare(Mat & _img, int _r0, int _r1, int _r2) {
		img = _img;
		r0 = _r0;
		r1 = _r1;
		r2 = _r2;
	}
	unsigned compute_feature(int x, int y) {
		float feature[4];
		double v;
		unsigned c;
		feature_extract_via(img.ptr<unsigned char>(y) + x, (int)img.step[0], r0, feature);
		v = (double)feature[0] * feature[1] * feature[2] * feature[3]; //Normally, Insu and wire = 1, and via >1
		v = max(v, 0.001);
		v = min(v, 1000.0);
		c = 0x40000000 + log(v) * 0x8000000;  //-7< log(v) <7, so c>0 & c<0x78000000, insu and wire, c=0x40000000
		return c;
	}
};

//The via has two circle
class TwoLightCircleViaExtract : public FeatureExtractVia {
protected:
	Mat img, ia, stat;
	vector<int>  dx, dx1;
	int last_xx, last_yy, last_gray;
public:
	void prepare(Mat & _img, int _r0, int _r1, int _r2) {
		img = _img;
		r0 = _r0;
		r1 = _r1;
		r2 = _r2;
		last_xx = -1; 
		last_yy = -1; 
		last_gray = 0;
		feature_extract_via2_prepare(img, r0, r1, ia, stat, dx, dx1);
	}
	unsigned compute_feature(int x, int y) {
		float feature[2];
		float v;
		feature_extract_via2(x, y, ia, stat, r0, r1, dx, dx1, feature, last_xx, last_yy, last_gray); //Normally, Insu and wire = 1, and via >1
		v = feature[0] * feature[1];
		v = max(v, 0.001f);
		v = min(v, 1000.0f);
		unsigned c = 0x40000000 + log(v) * 0x8000000; //-7< log(v) <7, so c>0 & c<0x78000000, insu and wire, c=0x40000000
		return c;
	}
};

FeatureExtractVia * FeatureExtractVia::create_feaext_via(int method)
{
	switch (method) {
	case 0:
		return new BigViaExtract;
	case 1:
		return new TwoLightCircleViaExtract;
	default:
		qCritical("Invalid via feature extract method %d", method);
		return NULL;
	}
}
/*
Input: weight, 
Input: w
Input: grid
Output: grid_line
it use dynamic programming, and make sure abs(grid_line[i+1] - grid_line[i] - grid) <= 1
*/
static double find_grid_line(const vector<double> & weight, int w, int grid, vector<int> & grid_line)
{
	CV_Assert(grid > WEIGHT_FB + 1);
	grid_line.clear();
	vector<double> weight_f[WEIGHT_FB], weight_b[WEIGHT_FB];

	//accumulate WEIGHT_FB forward and backward
	for (int i = 0; i < WEIGHT_FB; i++) {
		weight_f[i].resize(weight.size());
		weight_b[i].resize(weight.size());
		for (int j = 0; j < (int)weight.size(); j++) {
			int jf0 = min((int)weight.size() - 1, j + grid);
			int jf1 = min((int)weight.size() - 1, j + grid + 1);
			int jf2 = min((int)weight.size() - 1, j + grid - 1);
			int jb0 = max(0, j - grid);
			int jb1 = max(0, j - grid - 1);
			int jb2 = max(0, j - grid + 1);
			if (i == 0) {
				weight_f[0][j] = weight[j] + max(max(weight[jf0], weight[jf1]), weight[jf2]);
				weight_b[0][j] = weight[j] + max(max(weight[jb0], weight[jb1]), weight[jb2]);
			}
			else {
				weight_f[i][j] = weight[j] + max(max(weight_f[i - 1][jf0], weight_f[i - 1][jf1]), weight_f[i - 1][jf2]);
				weight_b[i][j] = weight[j] + max(max(weight_b[i - 1][jb0], weight_b[i - 1][jb1]), weight_b[i - 1][jb2]);
			}
		}
	}
	//choose max as baseline, this is important
	int base_line;
	double max_base = -100000000;
	for (int i = grid * 2; i < weight.size() - grid * 2; i++)
		if (max_base < weight_f[WEIGHT_FB - 1][i] + weight_b[WEIGHT_FB - 1][i] - weight[i]) {
		max_base = weight_f[WEIGHT_FB - 1][i] + weight_b[WEIGHT_FB - 1][i] - weight[i];
		base_line = i;
		}
	grid_line.push_back(base_line + w / 2);
	qDebug("base=%d", base_line + w / 2);
	//from base, search forward and backward
	for (int i = base_line; i > 0;) {
		int ib0 = max(0, i - grid);
		int ib1 = max(0, i - grid - 1);
		int ib2 = max(0, i - grid + 1);
		i = (weight_b[WEIGHT_FB - 1][ib0] > weight_b[WEIGHT_FB - 1][ib1]) ? ib0 : ib1;
		i = (weight_b[WEIGHT_FB - 1][i] > weight_b[WEIGHT_FB - 1][ib2]) ? i : ib2;
		if (i != 0) //be careful to change this
			grid_line.push_back(i + w / 2);
	}
	reverse(grid_line.begin(), grid_line.end());
	for (int i = base_line; i < (int)weight.size() - 1;) {
		int if0 = min((int)weight.size() - 1, i + grid);
		int if1 = min((int)weight.size() - 1, i + grid + 1);
		int if2 = min((int)weight.size() - 1, i + grid - 1);
		i = (weight_f[WEIGHT_FB - 1][if0] > weight_f[WEIGHT_FB - 1][if1]) ? if0 : if1;
		i = (weight_f[WEIGHT_FB - 1][i] > weight_f[WEIGHT_FB - 1][if2]) ? i : if2;
		if (i < (int)weight.size() - 1) //be careful to change this
			grid_line.push_back(i + w / 2);
	}
	return max_base;
}

struct FindViaGridLineReq {
	int dx, dy; //allow max deviation
	bool find_minor_gl, find_via, gl_x_valid, gl_y_valid, allow_new_x, allow_new_y;
	int x_align, y_align; //0, +1, -1
	FindViaGridLineReq(int _dx, int _dy, bool _find_minor_gl, bool _find_via, bool _gl_x_valid, bool _gl_y_valid,
		bool _allow_new_x, bool _allow_new_y) {
		dx = _dx;
		dy = _dy;
		find_minor_gl = _find_minor_gl;
		find_via = _find_via;
		gl_x_valid = _gl_x_valid;
		gl_y_valid = _gl_y_valid;
		allow_new_x = _allow_new_x;
		allow_new_y = _allow_new_y;
		x_align = 0;
		y_align = 0;
	}
};
/*
In lpm: wire_wd: metal width, grid_wd: grid width,  via_rd: via radius
Inout td:  in img, input image,
	out gl_x, gl_y,  grid line
Inout req: in dx, dy, find_minor_gl, find_via, gl_x_valid, gl_y_valid, allow_new_x, allow_new_y
	out x_align, y_align
out via_prob: via probability gl_y.size() * gl_x.size(), 1 is via, 0 is not via
return: direction, 0 means up-down grid line
It require wire is darker than via; and insu is darker than wire
*/
static int find_via_grid_line(struct LayerParam & lpm, struct LayerTileData & td, FindViaGridLineReq & req,
	Mat & via_prob)
{
	Mat img = td.img;
	int wire_wd = lpm.wire_wd, grid_wd = lpm.grid_wd, via_rd = lpm.via_rd0;
	float via_cred = lpm.param1;	

	if (!(via_cred >= 0 && via_cred <= 1 && grid_wd > wire_wd && grid_wd > via_rd)) {
		qCritical("invalid parameter via_cred=%f, grid=%d, wird_wd=%d, via_rd=%d", via_cred, grid_wd, wire_wd, via_rd);
		return -1;
	}

	QScopedPointer<FeatureExtractVia> extract_via(FeatureExtractVia::create_feaext_via(lpm.via_method));
	if (extract_via.isNull())
		return -2;
	extract_via->prepare(img, lpm.via_rd0, lpm.via_rd1, lpm.via_rd2);

	if (req.find_via)
		req.find_minor_gl = true;

	Mat ig;
	integral(img, ig, CV_32S);
	//0 each point left right and up down grad
	Mat grad0(img.rows, img.cols - 1, CV_32SC1);
	Mat grad1(img.rows - 1, img.cols, CV_32SC1);
	int dw = grid_wd / 2;
	vector<unsigned> stat(dw * 255, 0);

	for (int y = 0; y < img.rows; y++) {
		int * p_grad0 = grad0.ptr<int>(y);
		int y1 = max(0, y - dw / 2);
		if (y1 + dw >img.rows)
			y1 = img.rows - dw;
		const int * p_ig0 = ig.ptr<int>(y1);
		const int * p_ig1 = ig.ptr<int>(y1 + dw);
		for (int x = 1; x < img.cols; x++) { //grad0[y,x] = point[y,x+1] gray - point[y,x] gray
			int e = p_ig1[x + 1] - p_ig0[x + 1] - 2 * (p_ig1[x] - p_ig0[x]) + p_ig1[x - 1] - p_ig0[x - 1];
			p_grad0[x - 1] = e;
			stat[abs(e)]++;
		}
	}
	for (int y = 1; y < img.rows; y++) {
		int * p_grad1 = grad1.ptr<int>(y - 1);
		const int * p_ig0 = ig.ptr<int>(y - 1);
		const int * p_ig1 = ig.ptr<int>(y);
		const int * p_ig2 = ig.ptr<int>(y + 1); //grad1[y,x] = point[y+1,x] gray - point[y,x] gray
		for (int x = 0; x < img.cols; x++) {
			int x1 = max(0, x - dw / 2);
			if (x1 + dw >img.cols)
				x1 = img.cols - dw;
			int e = p_ig2[x1 + dw] - p_ig2[x1] - 2 * (p_ig1[x1 + dw] - p_ig1[x1]) + p_ig0[x1 + dw] - p_ig0[x1];
			p_grad1[x] = e;
			stat[abs(e)]++;
		}
	}

	//1 Do auto threshold and compute each line weight	
	vector<float> th, var_win;
	cal_threshold(stat, th, var_win);
	qDebug("grad low(a=%f,w=%f), grad high(a=%f,w=%f)", th[0], var_win[0], th[1], var_win[1]);
	double d0 = 0, d1 = 0;
	vector<float> lweight0, lweight1; //lweight means how much point is edge point, lweight0 for up-down, lweight1 for left-right
	for (int x = 0; x < img.cols - 1; x++) { //lweight0.size() == img.cols-1
		float s = 0;
		for (int y = 0; y < img.rows; y++) {
			int g = grad0.at<int>(y, x);
			float t = abs(g);
			if (t >= th[1])		// if exceed th, 
				s += SGN(g);	// make it saturate to +-1
			else
				if (t > th[0])	// else, make it (-1, +1)
					s += SGN(g) * (t - th[0]) / (th[1] - th[0]);
		}
		lweight0.push_back(s); //lweight0[i] = Image.col[i+1] - Image.col[i]
		d0 += abs(s);
	}
	for (int y = 0; y < img.rows - 1; y++) { //lweight1.size() == img.rows-1
		float s = 0;
		for (int x = 0; x < img.cols; x++) {
			int g = grad1.at<int>(y, x);
			float t = abs(g);
			if (t >= th[1])
				s += SGN(g);
			else
				if (t > th[0])
					s += SGN(g) * (t - th[0]) / (th[1] - th[0]);
		}
		lweight1.push_back(s); //lweight0[i] = Image.col[i+1] - Image.col[i]
		d1 += abs(s);
	}
	//2 Compute main grid line and compute via feature for all main grid line points
	vector<unsigned long long> caps; //center axis points set

	if (d0 > d1) {
		if (req.dx != 0 || !req.gl_x_valid) { //2.1 compute main grid line
			vector<double> weight;
			for (int x = 0; x < lweight0.size() - wire_wd; x++) //weight.size() == img.cols - wire_wd-1
				weight.push_back(lweight0[x] - lweight0[x + wire_wd]);
			if (req.dx < lpm.grid_wd / 2 && req.gl_x_valid) { //use reference gl_x for compute
				vector<int> gl_x;
				for (int i = 0; i <= td.gl_x.size(); i++) {
					int min_x = (i == 0) ? 0 : td.gl_x[i - 1] + req.dx + 1 - wire_wd / 2;
					int max_x = (i >= (int)td.gl_x.size()) ? (int)weight.size() - 1 : td.gl_x[i] - req.dx - 1 - wire_wd / 2;
					for (int x = min_x; x <= max_x; x++) //kill location far away from gl_x
						weight[x] = -1000000;
				}
				find_grid_line(weight, wire_wd, grid_wd, gl_x);
				req.x_align = 0;
				if (abs(td.gl_x[0] - gl_x[0]) > req.dx) {
					if (req.allow_new_x)
						req.x_align = (gl_x[0] > td.gl_x[0]) ? -1 : 1;
					else
						if (gl_x[0] > td.gl_x[0])
							gl_x.insert(gl_x.begin(), wire_wd / 2);
						else
							gl_x.erase(gl_x.begin());
				}

				if (abs(td.gl_x.back() - gl_x.back()) > req.dx && !req.allow_new_x) {
					if (gl_x.back() < td.gl_x.back())
						gl_x.push_back((int)weight.size() - 1 + wire_wd / 2);
					else
						gl_x.pop_back();
				}
				CV_Assert(td.gl_x.size() == gl_x.size() || req.allow_new_x);
				for (int i = 1; i < min(gl_x.size(), td.gl_x.size()) - 1; i++)
					CV_Assert(abs(td.gl_x[i] - gl_x[i + req.x_align]) <= req.dx);
				td.gl_x.swap(gl_x);
			}
			else //don't user reference
				find_grid_line(weight, wire_wd, grid_wd, td.gl_x);
		}
		if (!req.find_minor_gl)
			return (d0 > d1) ? 0 : 1;
		for (int x = 1; x < td.gl_x.size() - 1; x++) { //2.2 compute via feature for main grid line points
			int gl = 0;
			for (int y = via_rd + 1; y < img.rows - via_rd - 1; y++) {
				if (req.gl_y_valid && y < td.gl_y[gl] - req.dy)
					continue;
				if (req.gl_y_valid && y >= td.gl_y[gl] + req.dy)
					gl++;
				unsigned long long c;
				c = extract_via->compute_feature(td.gl_x[x], y);
				c = (c << 32) | (y << 16) | td.gl_x[x];
				caps.push_back(c);
				if (req.gl_y_valid && gl >= td.gl_y.size())
					break;
			}
		}
	}
	else {
		if (req.dy != 0 || !req.gl_y_valid) {
			vector<double> weight;
			for (int y = 0; y < lweight1.size() - wire_wd; y++) //weight.size() == img.rows - wire_wd-1
				weight.push_back(lweight1[y] - lweight1[y + wire_wd]);
			if (req.dy < lpm.grid_wd / 2 && req.gl_y_valid) { //use reference gl_y for compute
				vector<int> gl_y;
				for (int i = 0; i <= td.gl_y.size(); i++) {
					int min_y = (i == 0) ? 0 : td.gl_y[i - 1] + req.dy + 1 - wire_wd / 2;
					int max_y = (i >= (int)td.gl_y.size()) ? (int)weight.size() - 1 : td.gl_y[i] - req.dy - 1 - wire_wd / 2;
					for (int y = min_y; y <= max_y; y++)
						weight[y] = -1000000; //kill location far away from gl_x
				}
				find_grid_line(weight, wire_wd, grid_wd, gl_y);
				req.y_align = 0;
				if (abs(td.gl_y[0] - gl_y[0]) > req.dy) {
					if (req.allow_new_y)
						req.y_align = (gl_y[0] > td.gl_y[0]) ? -1 : 1;
					else
						if (gl_y[0] > td.gl_y[0])
							gl_y.insert(gl_y.begin(), wire_wd / 2);
						else
							gl_y.erase(gl_y.begin());
				}
				if (abs(td.gl_y.back() - gl_y.back()) > req.dy && !req.allow_new_y) {
					if (gl_y.back() < td.gl_y.back())
						gl_y.push_back((int)weight.size() - 1 + wire_wd / 2);
					else
						gl_y.pop_back();
				}
				CV_Assert(td.gl_y.size() == gl_y.size() || req.allow_new_y);
				for (int i = 1; i < min(td.gl_y.size(), gl_y.size()) - 1; i++)
					CV_Assert(abs(td.gl_y[i] - gl_y[i + req.y_align]) <= req.dy);
				td.gl_y.swap(gl_y);
			}
			else //don't user reference
				find_grid_line(weight, wire_wd, grid_wd, td.gl_y);
		}
		if (!req.find_minor_gl)
			return (d0 > d1) ? 0 : 1;
		for (int y = 1; y < td.gl_y.size() - 1; y++) {
			int gl = 0;
			for (int x = via_rd + 1; x < img.cols - via_rd - 1; x++) {
				if (req.gl_x_valid && x < td.gl_x[gl] - req.dx)
					continue;
				if (req.gl_x_valid && x >= td.gl_x[gl] + req.dx)
					gl++;
				
				unsigned long long c;
				
				c = extract_via->compute_feature(x, td.gl_y[y]);
				c = (c << 32) | (td.gl_y[y] << 16) | x;	
				caps.push_back(c);
				if (req.gl_x_valid && gl >= td.gl_x.size())
					break;
			}
		}
	}
	//3 Compute Via threshold
	sort(caps.begin(), caps.end(), greater<unsigned long long>());
	vector<unsigned> bins(2048, 0);
	for (int i = 0; i < caps.size() / 6; i++)
		bins[caps[i] >> 52]++;
	float beta = 0.8f;
	while (cal_threshold(bins, th, var_win, beta) < 1 && beta < 0.99) //TODO the beta setting has bug
		beta += 0.005f;

	int th0 = 0, th1 = 0;
	beta = 1.01f;
	while (th1 - th0 < 32 * 0x100000) {
		beta = beta - 0.01f;
		th1 = (th[1] - var_win[1] * beta + var_win[1] * (via_cred - 0.5)) * 0x100000;
		th0 = (th[0] + var_win[0] * beta + var_win[0] * (via_cred - 0.5)) * 0x100000;
	}	//Normally beta = 1, when quit
	CV_Assert(th1>th0);
	int th2 = th1 * 0.7 + th0 * 0.3;
	qDebug("Via extract: wire&insu (a=%f,w=%f), via (a=%f,w=%f), Obvious via=%f, Obviouse wire insu=%f, Coarse via th=%f",
		(th[0] - 1024) / 128, var_win[0] / 128, (th[1] - 1024) / 128, var_win[1] / 128, ((double)th1 - 0x40000000) / 0x8000000,
		((double)th0 - 0x40000000) / 0x8000000, ((double)th2 - 0x40000000) / 0x8000000);

	//4 Find coarse Via point based on th2
	vector<QPoint> via;
	int distance_th = (grid_wd - 2) * (grid_wd - 2);
	for (int i = 0; i < caps.size(); i++) {
		unsigned v = caps[i] >> 32;
		if ((int)v < th2)
			break;
		int y = (caps[i] >> 16) & 0xffff;
		int x = caps[i] & 0xffff;
		bool already_add = false;
		for (unsigned j = 0; j < via.size(); j++)
			if (((via[j].x() - x) * (via[j].x() - x) + (via[j].y() - y) * (via[j].y() - y)) <= distance_th)
				already_add = true;
		if (!already_add)
			via.push_back(QPoint(x, y));
	}
	//5 Compute minor grid line
	double vs = 3 * wire_wd;
	if (d0 > d1) {
		vector<double> weight;
		for (int y = 0; y < lweight1.size() - wire_wd; y++)
			weight.push_back(abs(lweight1[y] - lweight1[y + wire_wd]));
		for (int i = 0; i < via.size(); i++) {
			if (via[i].y() >= wire_wd / 2 && via[i].y() < weight.size() + wire_wd / 2)
				weight[via[i].y() - wire_wd / 2] += vs;
			if (via[i].y() >= wire_wd / 2 + 1 && via[i].y() < weight.size() + wire_wd / 2 + 1)
				weight[via[i].y() - wire_wd / 2 - 1] += vs*0.7;
			if (via[i].y() >= wire_wd / 2 + 2 && via[i].y() < weight.size() + wire_wd / 2 + 2)
				weight[via[i].y() - wire_wd / 2 - 2] += vs*0.3;
			if (via[i].y() >= wire_wd / 2 - 1 && via[i].y() < weight.size() + wire_wd / 2 - 1)
				weight[via[i].y() - wire_wd / 2 + 1] += vs*0.7;
			if (via[i].y() >= wire_wd / 2 - 2 && via[i].y() < weight.size() + wire_wd / 2 - 2)
				weight[via[i].y() - wire_wd / 2 + 2] += vs*0.3;
		}
		if (req.dy < lpm.grid_wd / 2 && req.gl_y_valid) { //use reference gl_y for compute
			vector<int> gl_y;
			for (int i = 0; i <= td.gl_y.size(); i++) {
				int min_y = (i == 0) ? 0 : td.gl_y[i - 1] + req.dy + 1 - wire_wd / 2;
				int max_y = (i >= (int)td.gl_y.size()) ? (int)weight.size() - 1 : td.gl_y[i] - req.dy - 1 - wire_wd / 2;
				for (int y = min_y; y <= max_y; y++)
					weight[y] = -1000000;
			}
			find_grid_line(weight, wire_wd, grid_wd, gl_y);
			req.y_align = 0;
			if (abs(td.gl_y[0] - gl_y[0]) > req.dy) {
				if (req.allow_new_y)
					req.y_align = (gl_y[0] > td.gl_y[0]) ? -1 : 1;
				else
					if (gl_y[0] > td.gl_y[0])
						gl_y.insert(gl_y.begin(), wire_wd / 2);
					else
						gl_y.erase(gl_y.begin());
			}
			if (abs(td.gl_y.back() - gl_y.back()) > req.dy && !req.allow_new_y) {
				if (gl_y.back() < td.gl_y.back())
					gl_y.push_back((int)weight.size() - 1 + wire_wd / 2);
				else
					gl_y.pop_back();
			}
			if (!req.allow_new_y)
				CV_Assert(td.gl_y.size() == gl_y.size());
			for (int i = 1; i < min(gl_y.size(), td.gl_y.size()) - 1; i++)
				CV_Assert(abs(td.gl_y[i] - gl_y[i + req.y_align]) <= req.dy);
			td.gl_y.swap(gl_y);
		}
		else //don't user reference
			find_grid_line(weight, wire_wd, grid_wd, td.gl_y);
	}
	else {
		vector<double> weight;
		for (int x = 0; x < lweight0.size() - wire_wd; x++)
			weight.push_back(abs(lweight0[x] - lweight0[x + wire_wd]));
		for (int i = 0; i < via.size(); i++) {
			if (via[i].x() >= wire_wd / 2 && via[i].x() < weight.size() + wire_wd / 2)
				weight[via[i].x() - wire_wd / 2] += vs;
			if (via[i].x() >= wire_wd / 2 + 1 && via[i].x() < weight.size() + wire_wd / 2 + 1)
				weight[via[i].x() - wire_wd / 2 - 1] += vs*0.7;
			if (via[i].x() >= wire_wd / 2 + 2 && via[i].x() < weight.size() + wire_wd / 2 + 2)
				weight[via[i].x() - wire_wd / 2 - 2] += vs*0.3;
			if (via[i].x() >= wire_wd / 2 - 1 && via[i].x() < weight.size() + wire_wd / 2 - 1)
				weight[via[i].x() - wire_wd / 2 + 1] += vs*0.7;
			if (via[i].x() >= wire_wd / 2 - 2 && via[i].x() < weight.size() + wire_wd / 2 - 2)
				weight[via[i].x() - wire_wd / 2 + 2] += vs*0.3;
		}
		if (req.dx < lpm.grid_wd / 2 && req.gl_x_valid) { //use reference gl_x for compute
			vector<int> gl_x;
			for (int i = 0; i <= td.gl_x.size(); i++) {
				int min_x = (i == 0) ? 0 : td.gl_x[i - 1] + req.dx + 1 - wire_wd / 2;
				int max_x = (i >= (int)td.gl_x.size()) ? (int)weight.size() - 1 : td.gl_x[i] - req.dx - 1 - wire_wd / 2;
				for (int x = min_x; x <= max_x; x++)
					weight[x] = -1000000;
			}
			find_grid_line(weight, wire_wd, grid_wd, gl_x);
			req.x_align = 0;
			if (abs(td.gl_x[0] - gl_x[0]) > req.dx) {
				if (req.allow_new_x)
					req.x_align = (gl_x[0] > td.gl_x[0]) ? -1 : 1;
				else
					if (gl_x[0] > td.gl_x[0])
						gl_x.insert(gl_x.begin(), wire_wd / 2);
					else
						gl_x.erase(gl_x.begin());
			}

			if (abs(td.gl_x.back() - gl_x.back()) > req.dx && !req.allow_new_x) {
				if (gl_x.back() < td.gl_x.back())
					gl_x.push_back((int)weight.size() - 1 + wire_wd / 2);
				else
					gl_x.pop_back();
			}
			if (!req.allow_new_x)
				CV_Assert(td.gl_x.size() == gl_x.size());
			for (int i = 1; i < min(gl_x.size(), td.gl_x.size()) - 1; i++)
				CV_Assert(abs(td.gl_x[i] - gl_x[i + req.x_align]) <= req.dx);
			td.gl_x.swap(gl_x);
		}
		else //don't user reference
			find_grid_line(weight, wire_wd, grid_wd, td.gl_x);
	}
	qDebug("Grid y lines=%d, grid x lines=%d", td.gl_y.size(), td.gl_x.size());
	//6 compute via prob
	if (!req.find_via)
		return (d0 > d1) ? 0 : 1;
	int stat_obvious_via = 0, stat_possible_via = 0;
	via_prob.create((int)td.gl_y.size(), (int)td.gl_x.size(), CV_32FC1);
	via_prob = Scalar(0);
	for (int i = 0; i < caps.size(); i++) {
		int v = caps[i] >> 32;
		if (v < th0)
			break;
		int y = (caps[i] >> 16) & 0xffff;
		int x = caps[i] & 0xffff;
		vector<int>::iterator iter;
		iter = lower_bound(td.gl_y.begin(), td.gl_y.end(), y); //found point (x,y)'s grid
		if (iter != td.gl_y.begin())
			iter--;
		int yy = (int *)&(*iter) - (int *)&(*td.gl_y.begin());
		if (yy < td.gl_y.size() - 1)
			if (abs(td.gl_y[yy] - y) > abs(td.gl_y[yy + 1] - y))
				yy++;

		iter = lower_bound(td.gl_x.begin(), td.gl_x.end(), x);
		if (iter != td.gl_x.begin())
			iter--;
		int xx = (int *)&(*iter) - (int *)&(*td.gl_x.begin());
		if (xx < td.gl_x.size() - 1)
			if (abs(td.gl_x[xx] - x) > abs(td.gl_x[xx + 1] - x))
				xx++;

		if (via_prob.at<float>(yy, xx) == 0 && yy > 0 && yy < td.gl_y.size() - 1 && xx>0 && xx < td.gl_x.size() - 1) {
			if (abs(y - td.gl_y[yy]) > VIA_MAX_BIAS_PIXEL && v > th2) {
				qWarning("Found via(x=%d,y=%d) not in grid, near y=%d", x, y, td.gl_y[yy]);
				continue;
			}
			if (abs(x - td.gl_x[xx]) > VIA_MAX_BIAS_PIXEL && v > th2) {
				qWarning("Found via(x=%d,y=%d) not in grid, near x=%d", x, y, td.gl_x[xx]);
				continue;
			}
			via_prob.at<float>(yy, xx) = min((double)(v - th0) / (th1 - th0), 1.0);
			if (v > th2)
				stat_obvious_via++;
			else
				stat_possible_via++;
		}
	}
	qDebug("obvious via num=%d, possible via num =%d", stat_obvious_via, stat_possible_via);
	return (d0 > d1) ? 0 : 1;
}

//only used for metal1 via extract
static void find_via_grid_line2(struct LayerParam & lpm, struct LayerTileData & td, FindViaGridLineReq & req,
	Mat & via_prob)
{
	Mat img = td.img;
	int grid_wd = lpm.grid_wd, via_rd = lpm.via_rd0;
	float via_cred = lpm.param1;

	if (!(via_cred >= 0 && via_cred <= 1 && grid_wd > via_rd)) {
		qCritical("invalid parameter via_cred=%f, grid=%d, via_rd=%d", via_cred, grid_wd, via_rd);
		return;
	}
	CV_Assert(req.find_via && req.gl_x_valid && req.gl_y_valid && !req.allow_new_x && !req.allow_new_y);

	QScopedPointer<FeatureExtractVia> extract_via(FeatureExtractVia::create_feaext_via(lpm.via_method));
	extract_via->prepare(img, lpm.via_rd0, lpm.via_rd1, lpm.via_rd2);
	if (extract_via.isNull())
		return;

	//1 compute via feature for all grid line points
	vector<unsigned long long> caps;
	for (int yy = 1; yy < td.gl_y.size() - 1; yy++)
		for (int xx = 1; xx < td.gl_x.size() - 1; xx++) {
		int y0 = td.gl_y[yy];
		int x0 = td.gl_x[xx];
		for (int y = y0 - req.dy; y <= y0 + req.dy; y++)
			for (int x = x0 - req.dx; x <= x0 + req.dx; x++) {
				unsigned long long c = extract_via->compute_feature(x, y);
				c = (c << 32) | (y << 16) | x;
				caps.push_back(c);
			}
		}

	//2 Compute Via threshold
	sort(caps.begin(), caps.end(), greater<unsigned long long>());
	vector<unsigned> bins(2048, 0);
	for (int i = 0; i < caps.size() / 8; i++)
		bins[caps[i] >> 52]++;
	float beta = 0.75;
	vector<float> th, var_win;
	while (cal_threshold(bins, th, var_win, beta) < 1 && beta < 0.99)
		beta += 0.005f;

	int th0 = 0, th1 = 0;
	beta = 1.01f;
	while (th1 - th0 < 32 * 0x100000) {
		beta = beta - 0.01f;
		th1 = (th[1] - var_win[1] * beta + var_win[1] * (via_cred - 0.5)) * 0x100000;
		th0 = (th[0] + var_win[0] * beta + var_win[0] * (via_cred - 0.5)) * 0x100000;
	}	//Normally beta = 1, when quit
	CV_Assert(th1>th0);
	int th2 = th1 * 0.7 + th0 * 0.3;
	qDebug("L0 Via extract: wire&insu (a=%f,w=%f), via (a=%f,w=%f), Obvious via=%f, Obviouse wire insu=%f, Coarse via th=%f",
		(th[0] - 1024) / 128, var_win[0] / 128, (th[1] - 1024) / 128, var_win[1] / 128, ((double)th1 - 0x40000000) / 0x8000000,
		((double)th0 - 0x40000000) / 0x8000000, ((double)th2 - 0x40000000) / 0x8000000);

	//3 Find coarse Via point based on th2
	vector<QPoint> via;
	int distance_th = (grid_wd - 2) * (grid_wd - 2);
	for (int i = 0; i < caps.size(); i++) {
		int v = caps[i] >> 32;
		if (v < th2)
			break;
		int y = (caps[i] >> 16) & 0xffff;
		int x = caps[i] & 0xffff;
		bool already_add = false;
		for (unsigned j = 0; j < via.size(); j++)
			if (((via[j].x() - x) * (via[j].x() - x) + (via[j].y() - y) * (via[j].y() - y)) <= distance_th)
				already_add = true;
		if (!already_add)
			via.push_back(QPoint(x, y));
	}
	//4 Compute grid line
	vector<double> weight;
	vector<int> gl_y, gl_x;
	weight.assign(img.rows, 0);
	for (int i = 0; i < via.size(); i++) {
		weight[via[i].y()] += 1;
		weight[via[i].y() - 1] += 0.7;
		weight[via[i].y() - 2] += 0.3;
		weight[via[i].y() + 1] += 0.7;
		weight[via[i].y() + 2] += 0.3;
	}
	for (int i = 0; i <= td.gl_y.size(); i++) {
		int min_y = (i == 0) ? 0 : td.gl_y[i - 1] + req.dy + 1;
		int max_y = (i >= (int)td.gl_y.size()) ? (int)weight.size() - 1 : td.gl_y[i] - req.dy - 1;
		for (int y = min_y; y <= max_y; y++)
			weight[y] = -1000000;
	}
	find_grid_line(weight, 0, grid_wd, gl_y);
	if (abs(td.gl_y[0] - gl_y[0]) > req.dy) {
		if (gl_y[0] > td.gl_y[0])
			gl_y.insert(gl_y.begin(), 0);
		else
			gl_y.erase(gl_y.begin());
	}
	if (abs(td.gl_y.back() - gl_y.back()) > req.dy) {
		if (gl_y.back() < td.gl_y.back())
			gl_y.push_back((int)weight.size() - 1);
		else
			gl_y.pop_back();
	}
	CV_Assert(td.gl_y.size() == gl_y.size());
	for (int i = 1; i < min(gl_y.size(), td.gl_y.size()) - 1; i++)
		CV_Assert(abs(td.gl_y[i] - gl_y[i]) <= req.dy);
	td.gl_y.swap(gl_y);

	weight.assign(img.cols, 0);
	for (int i = 0; i < via.size(); i++) {
		weight[via[i].x()] += 1;
		weight[via[i].x() - 1] += 0.7;
		weight[via[i].x() - 2] += 0.3;
		weight[via[i].x() + 1] += 0.7;
		weight[via[i].x() + 2] += 0.3;
	}
	for (int i = 0; i <= td.gl_x.size(); i++) {
		int min_x = (i == 0) ? 0 : td.gl_x[i - 1] + req.dx + 1;
		int max_x = (i >= (int)td.gl_x.size()) ? (int)weight.size() - 1 : td.gl_x[i] - req.dx - 1;
		for (int x = min_x; x <= max_x; x++)
			weight[x] = -1000000;
	}
	find_grid_line(weight, 0, grid_wd, gl_x);
	if (abs(td.gl_x[0] - gl_x[0]) > req.dx) {
		if (gl_x[0] > td.gl_x[0])
			gl_x.insert(gl_x.begin(), 0);
		else
			gl_x.erase(gl_x.begin());
	}
	if (abs(td.gl_x.back() - gl_x.back()) > req.dx) {
		if (gl_x.back() < td.gl_x.back())
			gl_x.push_back((int)weight.size() - 1);
		else
			gl_x.pop_back();
	}
	CV_Assert(td.gl_x.size() == gl_x.size());
	for (int i = 1; i < min(gl_x.size(), td.gl_x.size()) - 1; i++)
		CV_Assert(abs(td.gl_x[i] - gl_x[i]) <= req.dx);
	td.gl_x.swap(gl_x);
	qDebug("L0 Grid y lines=%d, grid x lines=%d", td.gl_y.size(), td.gl_x.size());
	int stat_obvious_via = 0, stat_possible_via = 0;
	via_prob.create((int)td.gl_y.size(), (int)td.gl_x.size(), CV_32FC1);
	via_prob = Scalar(0);
	for (int i = 0; i < caps.size(); i++) {
		int v = caps[i] >> 32;
		if (v < th0)
			break;
		int y = (caps[i] >> 16) & 0xffff;
		int x = caps[i] & 0xffff;
		vector<int>::iterator iter;
		iter = lower_bound(td.gl_y.begin(), td.gl_y.end(), y); //found point (x,y)'s grid
		if (iter != td.gl_y.begin())
			iter--;
		int yy = (int *)&(*iter) - (int *)&(*td.gl_y.begin());
		if (yy < td.gl_y.size() - 1)
			if (abs(td.gl_y[yy] - y) > abs(td.gl_y[yy + 1] - y))
				yy++;

		iter = lower_bound(td.gl_x.begin(), td.gl_x.end(), x);
		if (iter != td.gl_x.begin())
			iter--;
		int xx = (int *)&(*iter) - (int *)&(*td.gl_x.begin());
		if (xx < td.gl_x.size() - 1)
			if (abs(td.gl_x[xx] - x) > abs(td.gl_x[xx + 1] - x))
				xx++;
		if (via_prob.at<float>(yy, xx) == 0 && yy > 0 && yy < td.gl_y.size() - 1 && xx>0 && xx < td.gl_x.size() - 1) {
			if (abs(y - td.gl_y[yy]) > VIA_MAX_BIAS_PIXEL && v > th2) {
				qWarning("L0 Found via(x=%d,y=%d) not in grid, near y=%d", x, y, td.gl_y[yy]);
				continue;
			}
			if (abs(x - td.gl_x[xx]) > VIA_MAX_BIAS_PIXEL && v > th2) {
				qWarning("L0 Found via(x=%d,y=%d) not in grid, near x=%d", x, y, td.gl_x[xx]);
				continue;
			}
			via_prob.at<float>(yy, xx) = min((double)(v - th0) / (th1 - th0), 1.0);
			if (v > th2)
				stat_obvious_via++;
			else
				stat_possible_via++;
		}
	}
	qDebug("L0 obvious via num=%d, possible via num =%d", stat_obvious_via, stat_possible_via);
}

/*
In: slg
In: srect
In: d, destionation point refer to srect.topleft()
Out: dlg
*/
static void copy_grid_info(const LayerGridInfo & slg, LayerGridInfo & dlg, const QRect srect, const QPoint d)
{
	CV_Assert(d.x() >= 1 && d.y() >= 1);
	for (int y = 1; y <= slg.grid_rows; y++)
		if (y >= srect.top() && y <= srect.bottom())
			for (int x = 1; x <= slg.grid_cols; x++)
				if (x >= srect.left() && x <= srect.right()) {
					QPoint t(x - srect.left() + d.x(), y - srect.top() + d.y()); //destionation point refer to srect.topleft()
					if (t.y() >= 1 && t.y() <= dlg.grid_rows && t.x() >= 1 && t.x() <= dlg.grid_cols) {
						dlg.at(t.y(), t.x()) = slg.at(y, x);
						dlg.at(t.y(), t.x()).x = t.x();
						dlg.at(t.y(), t.x()).y = t.y();
					}
				}
}

#define MAKE_BRICK_ORDER(t, b) ((t) << 8 | (b))
#define PROB(bo) ((bo) >> 8)
#define BRICK(bo) ((bo) & 0xff)
/*In: prob brick prob for each grid, 3 - dim Mat
more near to 1 means bigger probability for the brick,
more near to 0 means less probability for the brick.
In: layer
In: gl_x, gl_y, used for mark debug
InOut: grid_infos, sorted brick prob order, push (prob.rows-2) * (prob.cols-2) grid_infos
	out grid_info.brick_order
Out: mark used for debug
*/
static void post_process_grid_prob(const Mat & prob, LayerBrickRuleMask & lbrm, vector<int> & gl_x, vector<int> & gl_y,
	LayerGridInfo & g, QPoint cp, Mat & mark)
{
	CV_Assert(mark.empty() || mark.type() == CV_8UC1 && mark.rows > gl_y[gl_y.size() - 1] && mark.cols > gl_x[gl_x.size() - 1]);
	CV_Assert(prob.type() == CV_32F && prob.dims == 3 && prob.size[0] == gl_y.size() && prob.size[1] == gl_x.size());
	CV_Assert(cp.y() >= 0 && cp.x() >= 0);

	vector<unsigned> brick_order;
	for (int y = 1; y < prob.size[0] - 1; y++)
		for (int x = 1; x < prob.size[1] - 1; x++) {
		const float * p_prob = prob.ptr<float>(y, x);
		GridInfo grid_info;
		brick_order.clear();
		for (int i = 0; i < prob.size[2]; i++)
			if (p_prob[i] > 0 && (lbrm.fit_mask & 1ULL << i)) {
			int t = p_prob[i] * PROB2_INT;
			brick_order.push_back(MAKE_BRICK_ORDER(t, i));
			}
		CV_Assert(brick_order.size()>0);
		//1 Sort brick order
		sort(brick_order.begin(), brick_order.end(), greater<unsigned int>());
		//2 drop low probability brick
		bool empty_exist = false;
		for (int i = 0; i < min(BRICK_CHOOSE_NUM, (int)brick_order.size()); i++) {
			grid_info.brick_order.push_back(brick_order[i]);
			if (BRICK(brick_order[i]) == BRICK_NO_WIRE)
				empty_exist = true;
		}

		if (!empty_exist)
			grid_info.brick_order.push_back(MAKE_BRICK_ORDER(1, BRICK_NO_WIRE)); //insert BRICK_NO_WIRE

		if (!mark.empty())
			for (int ord = 0; ord < 2; ord++) {
			int x1 = (ord == 0) ? (gl_x[x] + gl_x[x + 1]) / 2 : gl_x[x];
			int y1 = (ord == 0) ? gl_y[y] : (gl_y[y] + gl_y[y + 1]) / 2;
			int color = (unsigned)PROB(grid_info.brick_order[ord]) * 255 / PROB2_INT;
			int b = BRICK(grid_info.brick_order[ord]);
			for (int yy = 0; yy < 3; yy++)
				for (int xx = 0; xx < 3; xx++)
					if (bricks[b].a[yy][xx])
						mark.at<unsigned char>(y1 - 1 + yy, x1 - 1 + xx) = color;
			}
		g.at(y - 1 + cp.y(), x - 1 + cp.x()).brick_order.swap(grid_info.brick_order);
		g.at(y - 1 + cp.y(), x - 1 + cp.x()).x = x - 1 + cp.x();
		g.at(y - 1 + cp.y(), x - 1 + cp.x()).y = y - 1 + cp.y();
		}
}

/*In: prob via prob for each grid, 2 - dim Mat, more near to 0, more like wire&insu, more near to 1, more like via
In: gl_x, gl_y, used for mark debug
InOut: grid_infos, sorted brick prob order, push (prob.rows-2) * (prob.cols-2) grid_infos
Out: mark used for debug
*/
static void post_process_via_prob(const Mat & prob, float beta, vector<int> & gl_x, vector<int> & gl_y,
	LayerGridInfo & g, QPoint cp, Mat & mark)
{
	CV_Assert(mark.empty() || mark.type() == CV_8UC1 && mark.rows > gl_y[gl_y.size() - 1] && mark.cols > gl_x[gl_x.size() - 1]);
	CV_Assert(prob.type() == CV_32F && prob.dims == 2 && prob.rows == (int)gl_y.size() && prob.cols == (int)gl_x.size());
	CV_Assert(beta >= 0.5 && beta <= 8 && cp.y() >= 0 && cp.x() >= 0);

	for (int y = 1; y < prob.rows - 1; y++) {
		const float * p_prob = prob.ptr<float>(y);
		for (int x = 1; x < prob.cols - 1; x++) {
			GridInfo grid_info;
			unsigned t1 = p_prob[x] * PROB2_INT * beta;
			unsigned t0 = (1 - p_prob[x]) * PROB2_INT * beta;
			if (t0 > t1) {
				grid_info.brick_order.push_back(MAKE_BRICK_ORDER(t0, 0));
				grid_info.brick_order.push_back(MAKE_BRICK_ORDER(t1, 1));
			}
			else {
				grid_info.brick_order.push_back(MAKE_BRICK_ORDER(t1, 1));
				grid_info.brick_order.push_back(MAKE_BRICK_ORDER(t0, 0));
			}
			if (!mark.empty()) {
				int x1 = (gl_x[x] + gl_x[x + 1]) / 2;
				int y1 = (gl_y[y] + gl_y[y + 1]) / 2;
				int color = (p_prob[x] > 0.5) ? (2 * p_prob[x] - 1) * 255 : 0;
				for (int yy = 0; yy < 3; yy++)
					for (int xx = 0; xx < 3; xx++)
						if (xx == 1 || yy == 1)
							mark.at<unsigned char>(y1 - 1 + yy, x1 - 1 + xx) = color;
			}
			g.at(y - 1 + cp.y(), x - 1 + cp.x()).brick_order.swap(grid_info.brick_order);
			g.at(y - 1 + cp.y(), x - 1 + cp.x()).x = x - 1 + cp.x();
			g.at(y - 1 + cp.y(), x - 1 + cp.x()).y = y - 1 + cp.y();
		}
	}
}

#define BRICK_PREFER(l, gi) lg[l].grid_infos[gi].brick_prefer
#define BRICK_CHOOSE(l, gi) lg[l].grid_infos[gi].brick_choose
#define BRICK_ORDER(l, gi, i) lg[l].grid_infos[gi].brick_order[i]
#define BRICK_WEIGHT(l, gi, i) lg[l].grid_infos[gi].brick_weight[i]
#define BRICK_STATE(l, gi) lg[l].grid_infos[gi].state
#define BRICK_PROB(l, gi) lg[l].grid_infos[gi].prob
#define VIA_BRICK_WEIGHT(l, gi, b) ((BRICK(BRICK_ORDER(l,gi,0)) ==b) ? BRICK_WEIGHT(l, gi,0) : BRICK_WEIGHT(l, gi,1))
#define ABM(l, gi) lg[l].grid_infos[gi].abm

/*
In: lbrm, rule for brick fit, 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
Inout: lg, in brick_order(pre process) and out brick_choose(post process), 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
It will do following post_process
For wire layer
* if bottom via layer grid is via, increase brick-i
* decrease brick-i to kill wire break, if up & bottom via layer is no via
For via layer
* if both up and down wire layer conflict with current via layer, decrease prob
*/
static void post_process_wire_via_prob(const vector<LayerBrickRuleMask> & lbrm, vector<LayerGridInfo> & lg)
{
	static int via_extend_improve[][3] = {
			{ BRICK_I_0, BRICK_i_0, BRICK_i_180 }, //BRICK_I_0 may increase probability of BRICK_i_0 and BRICK_i_180
			{ BRICK_I_90, BRICK_i_90, BRICK_i_270 },
			{ BRICK_L_0, BRICK_i_0, BRICK_i_90 },
			{ BRICK_L_90, BRICK_i_90, BRICK_i_180 },
			{ BRICK_L_180, BRICK_i_180, BRICK_i_270 },
			{ BRICK_L_270, BRICK_i_270, BRICK_i_0 }
	};
	CV_Assert(lbrm.size() == lg.size());
	//1 Post process wire prob
	for (unsigned l = 1; l < lg.size(); l += 2) {
		CV_Assert(lg[l].grid_cols == lg[l - 1].grid_cols && lg[l].grid_cols == lg[l + 1].grid_cols &&
			lg[l].grid_rows == lg[l - 1].grid_rows && lg[l].grid_rows == lg[l + 1].grid_rows);
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
			int x = lg[l].grid_infos[gi].x;
			int y = lg[l].grid_infos[gi].y;
			if (lg[l].grid_infos[gi].brick_order.empty() || BRICK_CHOOSE(l, gi) != BRICK_INVALID)
				continue;

			unsigned l1 = l - 1, l2 = l + 1;
			CV_Assert(lg[l1].grid_infos[gi].x == x && lg[l1].grid_infos[gi].y == y &&
				lg[l2].grid_infos[gi].x == x && lg[l2].grid_infos[gi].y == y);

			int tot = 0;
			if (lbrm[l].via_extend_ovlap && BRICK(BRICK_ORDER(l1, gi, 0)) == 1) { //if bottom layer grid is via, increase brick-i
				int i1 = -1, i2 = -1;
				for (unsigned i = 0; i < sizeof(via_extend_improve) / sizeof(via_extend_improve[0]); i++) {
					if (via_extend_improve[i][0] == BRICK(BRICK_ORDER(l, gi, 0))) {
						i1 = via_extend_improve[i][1];
						i2 = via_extend_improve[i][2];
					}
				}
				if (i1 >= 0) { //Let BRICK_i increase probability, mainly increase brick-i for wire end
					float factor = EXTEND_VIA_FACTOR * PROB(BRICK_ORDER(l1, gi, 0)) * 2 / (PROB(BRICK_ORDER(l1, gi, 0)) + PROB(BRICK_ORDER(l1, gi, 1)))
						- EXTEND_VIA_FACTOR;
					CV_Assert(factor >= 0);
					for (unsigned i = 0; i <lg[l].grid_infos[gi].brick_order.size(); i++)
						if (BRICK(BRICK_ORDER(l, gi, i)) == i1 || BRICK(BRICK_ORDER(l, gi, i)) == i2) {
						int prob = factor * PROB(BRICK_ORDER(l, gi, 0)) + (1 - factor) * PROB(BRICK_ORDER(l, gi, i));
						BRICK_ORDER(l, gi, i) = MAKE_BRICK_ORDER(prob, BRICK(BRICK_ORDER(l, gi, i)));
						}
				}
			}
			for (unsigned i = 0; i < lg[l].grid_infos[gi].brick_order.size(); i++)
				tot += PROB(BRICK_ORDER(l, gi, i));

			lg[l].grid_infos[gi].brick_weight.resize(lg[l].grid_infos[gi].brick_order.size());
			for (unsigned i = 0; i < lg[l].grid_infos[gi].brick_order.size(); i++) {
				BRICK_ORDER(l, gi, i) = MAKE_BRICK_ORDER(PROB(BRICK_ORDER(l, gi, i)) * PROB2_INT / tot,
					BRICK(BRICK_ORDER(l, gi, i))); //Normalize
				BRICK_WEIGHT(l, gi, i) = PROB(BRICK_ORDER(l, gi, i));
			}
			bool renorm = false;
			int via_punish = 0.1 * min(PROB(BRICK_ORDER(l1, gi, 0)) - PROB(BRICK_ORDER(l1, gi, 1)),
				PROB(BRICK_ORDER(l2, gi, 0)) - PROB(BRICK_ORDER(l2, gi, 1))); //mainly for up & bottom layer no via, punish brick-i to kill wire break
			tot = 0;
			for (unsigned i = 0; i < lg[l].grid_infos[gi].brick_order.size(); i++) {
				if (!(lbrm[l].vwvfm[BRICK(BRICK_ORDER(l1, gi, 0))][BRICK(BRICK_ORDER(l2, gi, 0))] &
					1ULL << BRICK(BRICK_ORDER(l, gi, i)))) {
					renorm = true;
					BRICK_WEIGHT(l, gi, i) = max(BRICK_WEIGHT(l, gi, i) - via_punish, 0);
				}
				tot += BRICK_WEIGHT(l, gi, i);
			}
			if (renorm) {
				for (unsigned i = 0; i < lg[l].grid_infos[gi].brick_weight.size(); i++)
					BRICK_WEIGHT(l, gi, i) = BRICK_WEIGHT(l, gi, i) * PROB2_INT / tot;
			}
		}
	}
	//2 post process via prob
	for (unsigned l = 0; l < lg.size(); l += 2) {
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
			if (lg[l].grid_infos[gi].brick_order.empty() || BRICK_CHOOSE(l, gi) != BRICK_INVALID)
				continue;
			lg[l].grid_infos[gi].brick_weight.resize(2);
			BRICK_WEIGHT(l, gi, 0) = PROB(BRICK_ORDER(l, gi, 0));
			BRICK_WEIGHT(l, gi, 1) = PROB(BRICK_ORDER(l, gi, 1));
			bool check = false;
			int m = 0;
			for (int up = 0; up < 2; up++) {
				int l1, l2;
				if (up == 0) {
					l1 = l - 1;
					l2 = l - 2;
				}
				else {
					l1 = l + 1;
					l2 = l + 2;
				}
				if (l2 < 0 || l2 >(int) lg.size())
					continue;

				if ((int) l < l2) {
					if (lbrm[l1].vwvfm[BRICK(BRICK_ORDER(l, gi, 0))][BRICK(BRICK_ORDER(l2, gi, 0))] &
						1ULL << BRICK(BRICK_ORDER(l1, gi, 0)))
						check = true;
				}
				else
					if (lbrm[l1].vwvfm[BRICK(BRICK_ORDER(l2, gi, 0))][BRICK(BRICK_ORDER(l, gi, 0))] &
						1ULL << BRICK(BRICK_ORDER(l1, gi, 0)))
						check = true;

				m += min(PROB(BRICK_ORDER(l1, gi, 0)), PROB(BRICK_ORDER(l2, gi, 0)));
			}
			if (!check) { //if both up and down wire layer conflict with current via layer, decrease prob
				int tot = BRICK_WEIGHT(l, gi, 0) + BRICK_WEIGHT(l, gi, 1);
				BRICK_WEIGHT(l, gi, 0) = max(BRICK_WEIGHT(l, gi, 0) - m, 0);
				BRICK_WEIGHT(l, gi, 1) = tot - BRICK_WEIGHT(l, gi, 0);
			}
		}
	}
}

#define MAKE_LOC(l, gi) ((l) << 24 | (gi))
#define _LAYER(x) ((x) >> 24)
#define _GI(x) ((x) & 0xffffff)

struct FineAdjustAnswer {
	int best_score;
	vector<unsigned> best_path;
	vector<int> brick;
};
/*
In: lbrm, rule for brick fit, 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
Inout: lg, in brick_order and out brick_choose, 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
In: path, store real point
In: expand, store virtual point
*/
static void fine_adjust(int score, FineAdjustAnswer & best, int max_path, const vector<LayerBrickRuleMask> & lbrm, vector<LayerGridInfo> & lg, vector<unsigned> & path, vector <unsigned> expand)
{
	int x, y;
	unsigned loc, l, gi;
	if (score < best.best_score || path.size() + expand.size() > max_path)
		return;

	//1 Check all unfit nearby brick of path.back, push the unfit grid to expand queue (generate new virtual point)
	if (!path.empty()) {
		loc = path.back();
		l = _LAYER(loc);
		gi = _GI(loc);
		int cb = BRICK_PREFER(l, gi);
		CV_Assert(cb != BRICK_INVALID);
		x = lg[l].grid_infos[gi].x;
		y = lg[l].grid_infos[gi].y;
		if (l % 2 == 1) { // is wire layer
			for (int dir = 0; dir <= 3; dir++) {
				int y1 = y + dxy[dir][0];
				int x1 = x + dxy[dir][1];
				int gi1 = gi + lg[l].grid_cols * dxy[dir][0] + dxy[dir][1];
				if (y1 > 0 && y1 <= lg[l].grid_rows && x1 > 0 && x1 <= lg[l].grid_cols) {
					CV_Assert(lg[l].grid_infos[gi1].x == x1 && lg[l].grid_infos[gi1].y == y1);
					unsigned long long abm = lbrm[l].wwfm[cb][dir];
					if (BRICK_PREFER(l, gi1) != BRICK_INVALID)
						CV_Assert(abm & 1ULL << BRICK_PREFER(l, gi1)); //make sure all path brick is satisfied abm
					else {
						if (!(abm & 1ULL << BRICK_CHOOSE(l, gi1)) || BRICK_STATE(l, gi1) == 0) //found nearby unfit brick, push to expand 
							expand.push_back(MAKE_LOC(l, gi1));
					}
				}
			}
			CV_Assert(lg[l - 1].grid_infos[gi].x == x && lg[l - 1].grid_infos[gi].y == y &&
				lg[l + 1].grid_infos[gi].x == x && lg[l + 1].grid_infos[gi].y == y);
			int bl1 = (BRICK_PREFER(l - 1, gi) == BRICK_INVALID) ? BRICK_CHOOSE(l - 1, gi) : BRICK_PREFER(l - 1, gi);
			int bl2 = (BRICK_PREFER(l + 1, gi) == BRICK_INVALID) ? BRICK_CHOOSE(l + 1, gi) : BRICK_PREFER(l + 1, gi);
			if (!(lbrm[l].vwvfm[bl1][bl2] & 1ULL << cb)) { //found unfit via, push to expand
				CV_Assert(BRICK_PREFER(l - 1, gi) == BRICK_INVALID || BRICK_PREFER(l + 1, gi) == BRICK_INVALID);
				expand.push_back(MAKE_LOC(l - 1, gi));
			}
		}
		else { //is via layer
			bool up_check = true, down_check = true;
			if (l > 0) {
				int bl1 = (BRICK_PREFER(l - 1, gi) == BRICK_INVALID) ? BRICK_CHOOSE(l - 1, gi) : BRICK_PREFER(l - 1, gi);
				int bl2 = (BRICK_PREFER(l - 2, gi) == BRICK_INVALID) ? BRICK_CHOOSE(l - 2, gi) : BRICK_PREFER(l - 2, gi);
				if (!(lbrm[l - 1].vwvfm[bl2][cb] & 1ULL << bl1))
					down_check = false;
			}
			if (l < lg.size() - 1) {
				int bl1 = (BRICK_PREFER(l + 1, gi) == BRICK_INVALID) ? BRICK_CHOOSE(l + 1, gi) : BRICK_PREFER(l + 1, gi);
				int bl2 = (BRICK_PREFER(l + 2, gi) == BRICK_INVALID) ? BRICK_CHOOSE(l + 2, gi) : BRICK_PREFER(l + 2, gi);
				if (!(lbrm[l + 1].vwvfm[cb][bl2] & 1ULL << bl1))
					up_check = false;
			}
			if (!up_check && !down_check) //TODO may neglect potential best answer
				return;
			if (!up_check || !down_check) {
				int ll[2];
				if (!up_check) {
					ll[0] = l + 1;
					ll[1] = l + 2;
				}
				else {
					ll[0] = l - 1;
					ll[1] = l - 2;
				}
				if (BRICK_PREFER(ll[0], gi) != BRICK_INVALID)
					return;

				if (BRICK_PREFER(ll[1], gi) != BRICK_INVALID || BRICK_STATE(ll[0], gi) == 0)
					expand.push_back(MAKE_LOC(ll[0], gi));
				else
					for (int i = 1; i >= 0; i--)	{
					CV_Assert(lg[ll[i]].grid_infos[gi].x == x && lg[ll[i]].grid_infos[gi].y == y);
					path.push_back(MAKE_LOC(ll[i], gi));
					BRICK_PREFER(ll[i], gi) = BRICK_CHOOSE(ll[i], gi);
					fine_adjust(score, best, max_path, lbrm, lg, path, expand);
					path.pop_back();
					BRICK_PREFER(ll[i], gi) = BRICK_INVALID;
					}
			}
		}
	}
	bool expand_empty = true;
	while (!expand.empty()) {
		loc = expand.back();
		l = _LAYER(loc);
		gi = _GI(loc);
		expand.pop_back();
		if (BRICK_PREFER(l, gi) == BRICK_INVALID) { //check if it is still virtual point (not turn to real point)
			expand_empty = false; //if it is virtual point, break 
			break;
		}
	}
	if (path.size() + expand.size() >= max_path)
		return;

	if (expand_empty) { //if no virtual point
		if (best.best_score < score) {//Now find one solution
			best.best_score = score;
			best.best_path = path;
			best.brick.resize(path.size());
			for (unsigned i = 0; i < path.size(); i++) {
				unsigned loc = path[i];
				unsigned l = _LAYER(loc);
				unsigned gi = _GI(loc);
				best.brick[i] = BRICK_PREFER(l, gi);
			}
		}
		return;
	}
	//2 calculate abm, and choose two brick with biggest probability (Turn virtual point to real point) 
	//TODO: need to choose 3 brick?	
	x = lg[l].grid_infos[gi].x;
	y = lg[l].grid_infos[gi].y;
	if (l % 2 == 1) { // is wire layer
		CV_Assert(BRICK_PREFER(l, gi) == BRICK_INVALID);
		unsigned long long abm = ABM(l, gi); //following compute abm of nearby brick prefer
		for (int dir = 0; dir <= 3; dir++) {
			int y1 = y + dxy[dir][0];
			int x1 = x + dxy[dir][1];
			int gi1 = gi + lg[l].grid_cols * dxy[dir][0] + dxy[dir][1];
			if (y1 > 0 && y1 <= lg[l].grid_rows && x1 > 0 && x1 <= lg[l].grid_cols)
				if (BRICK_PREFER(l, gi1) != BRICK_INVALID) {
				CV_Assert(lg[l].grid_infos[gi1].x == x1 && lg[l].grid_infos[gi1].y == y1);
				abm &= lbrm[l].wwfm[BRICK_PREFER(l, gi1)][(dir + 2) & 3];
				}
		}
		CV_Assert(lg[l - 1].grid_infos[gi].x == x && lg[l - 1].grid_infos[gi].y == y &&
			lg[l + 1].grid_infos[gi].x == x && lg[l + 1].grid_infos[gi].y == y);
		if (BRICK_PREFER(l - 1, gi) != BRICK_INVALID && BRICK_PREFER(l + 1, gi) != BRICK_INVALID)
			abm &= lbrm[l].vwvfm[BRICK_PREFER(l - 1, gi)][BRICK_PREFER(l + 1, gi)];

		int max = -PROB2_INT, submax = -PROB2_INT;
		int max_b = BRICK_INVALID, submax_b = BRICK_INVALID;
		int cur_prob = -1, max_prob = -1, submax_prob;
		//Choose two brick with biggest prob 
		for (unsigned i = 0; i < lg[l].grid_infos[gi].brick_weight.size(); i++) {
			if (BRICK(BRICK_ORDER(l, gi, i)) == BRICK_CHOOSE(l, gi))
				cur_prob = PROB(BRICK_ORDER(l, gi, i));
			if (abm & 1ULL << BRICK(BRICK_ORDER(l, gi, i))) {
				if (BRICK_WEIGHT(l, gi, i) > max) {
					submax = max;
					submax_b = max_b;
					submax_prob = max_prob;
					max = BRICK_WEIGHT(l, gi, i);
					max_b = BRICK(BRICK_ORDER(l, gi, i));
					max_prob = PROB(BRICK_ORDER(l, gi, i));
				}
				else
					if (BRICK_WEIGHT(l, gi, i)  > submax) {
					submax = BRICK_WEIGHT(l, gi, i);
					submax_b = BRICK(BRICK_ORDER(l, gi, i));
					submax_prob = PROB(BRICK_ORDER(l, gi, i));
					}
			}
		}

		//3 try two brick
		path.push_back(loc);
		if (max_b != BRICK_INVALID) {
			BRICK_PREFER(l, gi) = max_b;
			fine_adjust(score + max_prob - cur_prob, best, max_path, lbrm, lg, path, expand);
		}
		if (submax_b != BRICK_INVALID) {
			BRICK_PREFER(l, gi) = submax_b;
			fine_adjust(score + submax_prob - cur_prob, best, max_path, lbrm, lg, path, expand);
		}
		path.pop_back();
		BRICK_PREFER(l, gi) = BRICK_INVALID;
	}
	else { //is via layer
		//3 two choice is to reverse bottom via or up via, try it
		int cur_prob, reverse_prob, l1 = l, l2 = l + 2;
		int l1_prob, l2_prob;
		if (BRICK_CHOOSE(l1, gi) == BRICK(BRICK_ORDER(l1, gi, 0)))
			l1_prob = PROB(BRICK_ORDER(l1, gi, 0));
		else
			l1_prob = PROB(BRICK_ORDER(l1, gi, 1));
		if (BRICK_CHOOSE(l2, gi) == BRICK(BRICK_ORDER(l2, gi, 0)))
			l2_prob = PROB(BRICK_ORDER(l2, gi, 0));
		else
			l2_prob = PROB(BRICK_ORDER(l2, gi, 1));
		if (l1_prob > l2_prob)
			std::swap(l1, l2);
		if (BRICK_PREFER(l1, gi) == BRICK_INVALID) {
			path.push_back(MAKE_LOC(l1, gi));
			BRICK_PREFER(l1, gi) = 1 - BRICK_CHOOSE(l1, gi);
			if (BRICK_CHOOSE(l1, gi) == BRICK(BRICK_ORDER(l1, gi, 0))) {
				cur_prob = PROB(BRICK_ORDER(l1, gi, 0));
				reverse_prob = PROB(BRICK_ORDER(l1, gi, 1));
			}
			else {
				cur_prob = PROB(BRICK_ORDER(l1, gi, 1));
				reverse_prob = PROB(BRICK_ORDER(l1, gi, 0));
			}
			fine_adjust(score + reverse_prob - cur_prob, best, max_path, lbrm, lg, path, expand);
			path.pop_back();
			BRICK_PREFER(l1, gi) = BRICK_INVALID;
		}
		if (BRICK_PREFER(l2, gi) == BRICK_INVALID) {
			path.push_back(MAKE_LOC(l2, gi));
			BRICK_PREFER(l2, gi) = 1 - BRICK_CHOOSE(l2, gi);
			if (BRICK_CHOOSE(l2, gi) == BRICK(BRICK_ORDER(l2, gi, 0))) {
				cur_prob = PROB(BRICK_ORDER(l2, gi, 0));
				reverse_prob = PROB(BRICK_ORDER(l2, gi, 1));
			}
			else {
				cur_prob = PROB(BRICK_ORDER(l2, gi, 1));
				reverse_prob = PROB(BRICK_ORDER(l2, gi, 0));
			}
			fine_adjust(score + reverse_prob - cur_prob, best, max_path, lbrm, lg, path, expand);
			path.pop_back();
			BRICK_PREFER(l2, gi) = BRICK_INVALID;
		}
	}
}
/*
In: l, layer
In: gi, grid_infos's index
In: lbrm, rule for brick fit, 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
Inout: lg, in brick_order and out brick_choose, 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
out: abm
Return: 0xffffffff means good, else means 1st conflict (l,gi)
*/
static unsigned check_nearby_unfit(int l, int gi, const vector<LayerBrickRuleMask> & lbrm, vector<LayerGridInfo> & lg, unsigned long long & abm, bool check_adj_via = false)
{
	CV_Assert((l & 1) == 1);

	int x = lg[l].grid_infos[gi].x;
	int y = lg[l].grid_infos[gi].y;
	abm = ABM(l, gi);
	unsigned lgi = 0xffffffff;
	if (BRICK_CHOOSE(l + 1, gi) == 0) //no via, no check_adj_via
		check_adj_via = false;
	for (int dir = 0; dir <= 3; dir++) {
		int y1 = y + dxy[dir][0];
		int x1 = x + dxy[dir][1];
		int gi1 = gi + lg[l].grid_cols * dxy[dir][0] + dxy[dir][1];
		if (y1 > 0 && y1 <= lg[l].grid_rows && x1 > 0 && x1 <= lg[l].grid_cols && !lg[l].grid_infos[gi1].brick_order.empty()) {
			CV_Assert(lg[l].grid_infos[gi1].x == x1 && lg[l].grid_infos[gi1].y == y1);
			abm &= lbrm[l].wwfm[BRICK_CHOOSE(l, gi1)][(dir + 2) & 3];
			if (!(abm & 1ULL << BRICK_CHOOSE(l, gi)) && lgi == 0xffffffff) // check unfit
				lgi = MAKE_LOC(l, gi1);
			if (check_adj_via && BRICK_CHOOSE(l + 1, gi1)==1 && (bricks[BRICK_CHOOSE(l, gi)].shape & 3 << (dir*2)))
				lgi = MAKE_LOC(l, gi1);
		}
	}
	CV_Assert(lg[l - 1].grid_infos[gi].x == x && lg[l - 1].grid_infos[gi].y == y &&
		lg[l + 1].grid_infos[gi].x == x && lg[l + 1].grid_infos[gi].y == y);
	abm &= lbrm[l].vwvfm[BRICK_CHOOSE(l - 1, gi)][BRICK_CHOOSE(l + 1, gi)];
	if (!(abm & 1ULL << BRICK_CHOOSE(l, gi)) && lgi == 0xffffffff) {
		lgi = MAKE_LOC(l - 1, gi); //Match with fine_adjust, so fine_adjust will check l-1 and l+1.	
	}
	return lgi;
}
/*
In: l, layer, must be wire layer
Inout: lg, in brick_order and out brick_choose, 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
In: ar, area rect
Remove adj wire connection, if adj grid both have via.
*/
static int remove_adj_vias(int l, vector<LayerGridInfo> & lg, const QRect & ar)
{
	int remove_num = 0;
	CV_Assert(l % 2 == 1);
	for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++)
		if (BRICK_CHOOSE(l + 1, gi) == 1 && !lg[l + 1].grid_infos[gi].brick_order.empty()) { // have via, check adj
		int x = lg[l].grid_infos[gi].x;
		int y = lg[l].grid_infos[gi].y;
		if (ar.contains(x, y, true)) {
			int shape = bricks[BRICK_CHOOSE(l, gi)].shape;
			for (int dir = 0; dir <= 3; dir++)
				if (shape & 3 << (dir*2)) { //adj connected, check if it have via
				int y1 = y + dxy[dir][0];
				int x1 = x + dxy[dir][1];
				int gi1 = gi + lg[l].grid_cols * dxy[dir][0] + dxy[dir][1];
				if (ar.contains(x1, y1, true) && y1 > 0 && y1 <= lg[l].grid_rows && x1 > 0 && x1 <= lg[l].grid_cols && BRICK_CHOOSE(l + 1, gi1) == 1)
					shape &= ~(3 << (dir*2)); //remove adj connection
				}
			if (shape != bricks[BRICK_CHOOSE(l, gi)].shape) {
				for (int i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++)
					if (bricks[i].shape == shape) {
					BRICK_CHOOSE(l, gi) = i;
					remove_num++;
					}
			}
		}
		}
	return remove_num;
}

/*
In: lbrm, rule for brick fit, 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
In: lg.grid_infos.brick_order, 2*img_layer-1
0,2,4 for via;
1,3,5 for wire,
out: lg.grid_infos.brick_choose, 
In: ar, only assemble grid's brick truly inside ar (not in edge)
In: method, IGNORE_ALL_RULE or OBEY_RULE_WEAK or OBEY_RULE_STRONG
First do coarse assemble, choose max brick_order or brick_weight
Then 
*/
static void try_assemble_multilayer(const vector<LayerBrickRuleMask> & lbrm, vector<LayerGridInfo> & lg, const QRect & ar, int method)
{
	CV_Assert(lbrm.size() == lg.size());
	int layer_num = (int)lg.size();

	CV_Assert(layer_num % 2 == 1);
	//choose max brick_order
	if (method == IGNORE_ALL_RULE) {
		for (int l = 0; l < layer_num; l++)
			for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
			int x = lg[l].grid_infos[gi].x;
			int y = lg[l].grid_infos[gi].y;
			if (ar.contains(x, y, true) && BRICK_CHOOSE(l, gi) == BRICK_INVALID) {
				for (int j = 0; j < lg[l].grid_infos[gi].brick_order.size(); j++)
					if (ABM(l, gi) & 1ULL << BRICK(BRICK_ORDER(l, gi, j))) {
					BRICK_CHOOSE(l, gi) = BRICK(BRICK_ORDER(l, gi, j));
					break;
					}
			}
			}
		return;
	}

	//choose max brick_weight
	for (int l = 0; l < layer_num; l++)
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
			int x = lg[l].grid_infos[gi].x;
			int y = lg[l].grid_infos[gi].y;
			if (ar.contains(x, y, true) && BRICK_CHOOSE(l, gi) == BRICK_INVALID) {
				int m = 0;
				for (int j = 0; j < lg[l].grid_infos[gi].brick_weight.size(); j++)
					if ((ABM(l, gi) & 1ULL << BRICK(BRICK_ORDER(l, gi, j))) &&
						BRICK_WEIGHT(l, gi, j) >m) {
					BRICK_CHOOSE(l, gi) = BRICK(BRICK_ORDER(l, gi, j));
					m = BRICK_WEIGHT(l, gi, j);
					}
			}
		}
	if (method == OBEY_RULE_WEAK)
		return;

	set<unsigned long> unfit_set;
	int bc_strong = 0;
	long long bc_score = 0;
	/*
	#define MAKE_UNFIT(lgi0, lgi1) ((lgi0) < (lgi1) ? (unsigned long long) (lgi0) << 32 | (lgi1) : (unsigned long long) (lgi1) << 32 | (lgi0))
	#define _LGI1(uf) (uf & 0xffffffff)
	#define _LGI0(uf) (uf >> 32)*/

	for (int l = 1; l < layer_num; l += 2) //only check wire layer
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
		int x = lg[l].grid_infos[gi].x;
		int y = lg[l].grid_infos[gi].y;
		if (!ar.contains(x, y, true) || lg[l].grid_infos[gi].brick_order.empty()) {
			BRICK_STATE(l, gi) = 1;
			BRICK_PROB(l, gi) = 0;
			continue;
		}
		unsigned long long abm;
		unsigned lgi = check_nearby_unfit(l, gi, lbrm, lg, abm);

		if (lgi != 0xffffffff) { 
			BRICK_STATE(l, gi) = 0; //unfit nearby brick, push to unfit_set
			BRICK_PROB(l, gi) = 1;
			unfit_set.insert(MAKE_LOC(l, gi));
		}
		else {
			BRICK_STATE(l, gi) = 1;
			BRICK_PROB(l, gi) = 0;
		}
		}
	//following is fine adjust
	while (!unfit_set.empty()) {
		unsigned lgi0 = *unfit_set.begin();
		unfit_set.erase(unfit_set.begin());
		vector<unsigned> search_unfit_queue;
		vector<unsigned> nearby_unfit;
		search_unfit_queue.push_back(lgi0);

		//Put all lgi0 nearby unfit grid to nearby_unfit, and delete them from unfit_set
		while (!search_unfit_queue.empty()) {
			lgi0 = search_unfit_queue.back();
			search_unfit_queue.pop_back();
			nearby_unfit.push_back(lgi0);
			int l = _LAYER(lgi0);
			int gi = _GI(lgi0);
			CV_Assert(BRICK_STATE(l, gi) == 0);
			for (int dir = 0; dir <= 3; dir++) {
				int gi1 = gi + lg[l].grid_cols * dxy[dir][0] + dxy[dir][1];
				if (unfit_set.find(MAKE_LOC(l, gi1)) != unfit_set.end()) {
					unfit_set.erase(MAKE_LOC(l, gi1));
					search_unfit_queue.push_back(MAKE_LOC(l, gi1));
				}
			}
		}
		//Do fine_adjust for grid in nearby_unfit
		FineAdjustAnswer best;
		vector<unsigned> path, expand;
		best.best_score = -100 * PROB2_INT;
		int depth = (int) nearby_unfit.size() + 2;

		if (depth <= CONFLICT_SEARCH_DEPTH) {
			while (!nearby_unfit.empty()) {
				lgi0 = nearby_unfit.back();
				nearby_unfit.pop_back();
				expand.push_back(lgi0);
				fine_adjust(0, best, depth, lbrm, lg, path, expand);
				CV_Assert(path.empty());
				expand.pop_back();
			}
		}

		if (best.best_score > -100 * PROB2_INT) {
			for (unsigned i = 0; i < best.best_path.size(); i++) {
				unsigned loc = best.best_path[i];
				unsigned l = _LAYER(loc);
				unsigned gi = _GI(loc);
				CV_Assert(BRICK_PREFER(l, gi) == BRICK_INVALID);
				BRICK_CHOOSE(l, gi) = best.brick[i];
				if (l % 2 == 1) {
					BRICK_STATE(l, gi) = 1;
					BRICK_PROB(l, gi) = best.best_score > 0 ? 0 : (float)-best.best_score / PROB2_INT;
				}
			}
			bc_strong++;
			bc_score += best.best_score;
		}
	}
	qDebug("brick change=%d, brick score change = %d", bc_strong, bc_score);
	int remove_num = 0;
	for (int l = 1; l < layer_num; l += 2)
		if (lbrm[l].rule & RULE_NO_ADJ_VIA_CONN)
			remove_num += remove_adj_vias(l, lg, ar);
	qDebug("adj via remove_num=%d", remove_num);
}

/*
In: lg.grid_infos.brick_order, 
0,2,4 for via;
1,3,5 for wire,
	inout lg.grid_infos.brick_choose, if it is not on boundary, it should be BRICK_INVALID
In: lbrm, (2*img_layer-1), it represents rules for neighbor wire & wire, via & wire & via
In: warn_lbrm, same as lbrm
Out: td.d.conet brick shape
assemble_grid_multilayer neglects all non BRICK_INVALID brick, so normally boundary brick is initialed from adj tile,
And assign BRICK_INVALID in current tile as initial value.
*/
static void assemble_grid_multilayer(vector<LayerGridInfo> & lg, const vector<LayerBrickRuleMask> & lbrm,
	const vector<LayerBrickRuleMask> & warn_lbrm, TileData & td, const QRect & ar, int method)
{

	CV_Assert(lg.size() % 2 == 1 && lg.size() == lbrm.size());
	//init brick mask as layer rule 
	for (int l = 0; l < lg.size(); l++) {
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
			int x = lg[l].grid_infos[gi].x;
			int y = lg[l].grid_infos[gi].y;
			if (ar.contains(x, y, true))
				ABM(l, gi) = lbrm[l].fit_mask;
		}
	}
	//compute adj brick mask from boundary brick
	for (int l = 0; l < lg.size(); l++)
		if (l % 2 == 1) //is wire layer
			for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
				if (BRICK_CHOOSE(l, gi) == BRICK_INVALID)
					continue;
				int x = lg[l].grid_infos[gi].x;
				int y = lg[l].grid_infos[gi].y;
				if (((y == ar.top() || y == ar.bottom()) && x > ar.left() && x < ar.right()) ||
					((x == ar.left() || x == ar.right()) && y > ar.top() && y < ar.bottom())) //if it's boundary brick
					for (int dir = 0; dir <= 3; dir++) {
						int y1 = y + dxy[dir][0];
						int x1 = x + dxy[dir][1];
						int gi1 = gi + lg[l].grid_cols * dxy[dir][0] + dxy[dir][1];
						if (ar.contains(x1, y1, true)) {
							CV_Assert(lg[l].grid_infos[gi1].x == x1 && lg[l].grid_infos[gi1].y == y1);
							ABM(l, gi1) &= lbrm[l].wwfm[BRICK_CHOOSE(l, gi)][dir];
						}
					}
			}
	post_process_wire_via_prob(lbrm, lg);
	try_assemble_multilayer(lbrm, lg, ar, method);

	//Pack grid_info to conet
	for (int l = 0; l < lg.size(); l++) {
		int gi = 0;
		if (l == 0 || l % 2 == 1)
			td.d[(l + 1) / 2].conet.create(lg[l].grid_rows, lg[l].grid_cols, CV_32SC1);
		for (int y = 0; y < lg[l].grid_rows; y++) {
			int * p_con = td.d[(l + 1) / 2].conet.ptr<int>(y);
			for (int x = 0; x < lg[l].grid_cols; x++, gi++) {
				if (BRICK_CHOOSE(l, gi) == BRICK_INVALID) {
					p_con[x] = 0;
					continue;
				}
				CV_Assert(lg[l].grid_infos[gi].y == y + 1 && lg[l].grid_infos[gi].x == x + 1);
				if (l == 0)
					p_con[x] = BRICK_CHOOSE(l, gi) ? VIA_MASK : 0;
				else
					if (l % 2 == 0) //via layer
						p_con[x] |= BRICK_CHOOSE(l, gi) ? VIA_MASK : 0;
					else
						p_con[x] = bricks[BRICK_CHOOSE(l, gi)].shape;
				if (l % 2 == 1) { //wire layer
					if (BRICK_STATE(l, gi) == 0)
						p_con[x] |= UNSURE_MASK;
					else {
						unsigned long long abm;
						if (check_nearby_unfit(l, gi, warn_lbrm, lg, abm, warn_lbrm[l].rule & RULE_NO_ADJ_VIA_CONN) != 0xffffffff)
							p_con[x] |= UNSURE_MASK;
						else {
							int unsure = BRICK_PROB(l, gi) * 10;
							unsure = min(6, unsure);
							p_con[x] |= unsure << UNSURE_SHIFT;
						}
					}
				}
			}
		}
	}
}

/*
in: connet, each grid metal connection, up 1, right 2, down 4, left 8, via 16, unsure, xxx00000
in: gl_x, grid line x
in: gl_y, grid line y
out: obj_sets
*/
void grid2_wire_obj(const Mat & conet, int layer, const vector<int> & gl_x, const vector<int> & gl_y, vector<MarkObj> & obj_sets)
{
	CV_Assert(conet.rows + 2 == gl_y.size() && conet.cols + 2 == gl_x.size() && conet.type() == CV_32SC1);
	MarkObj wire;
	wire.type = OBJ_LINE;
	wire.type2 = LINE_WIRE_AUTO_EXTRACT;
	wire.type3 = layer;
	wire.state = 0;
	wire.prob = 1;

	for (int y = 0; y < conet.rows; y++) {
		int state = 0;
		wire.p0.setY(gl_y[y + 1]);
		wire.p1.setY(gl_y[y + 1]);
		const int * p_conet = conet.ptr<int>(y);
		for (int x = 0; x < conet.cols; x++) //find left-right line
			if (p_conet[x] & DIR_RIGHT1_MASK) {
				CV_Assert(p_conet[x] & UNSURE_MASK || x + 2 >= conet.cols || p_conet[x + 1] & DIR_LEFT1_MASK);
				if (state == 0)
					wire.p0.setX(gl_x[x + 1]);
				state = 1;
			}
			else {
				CV_Assert(p_conet[x] & UNSURE_MASK || x + 2 >= conet.cols || !(p_conet[x + 1] & DIR_LEFT1_MASK));
				if (state == 1) {
					wire.p1.setX(gl_x[x + 1]);
					obj_sets.push_back(wire);
				}
				state = 0;
			}
			if (state == 1) {
				wire.p1.setX(gl_x[conet.cols]);
				obj_sets.push_back(wire);
			}
	}
	for (int x = 0; x < conet.cols; x++) {
		int state = 0;
		wire.p0.setX(gl_x[x + 1]);
		wire.p1.setX(gl_x[x + 1]);
		for (int y = 0; y < conet.rows; y++) //find up-down line
			if (conet.at<int>(y, x) & DIR_DOWN1_MASK) {
				CV_Assert(conet.at<int>(y, x) & UNSURE_MASK || y + 2 >= conet.rows || conet.at<int>(y + 1, x) & DIR_UP1_MASK);
				if (state == 0)
					wire.p0.setY(gl_y[y + 1]);
				state = 1;
			}
			else {
				CV_Assert(conet.at<int>(y, x) & UNSURE_MASK || y + 2 >= conet.rows || !(conet.at<int>(y + 1, x) & DIR_UP1_MASK));
				if (state == 1) {
					wire.p1.setY(gl_y[y + 1]);
					obj_sets.push_back(wire);
				}
				state = 0;
			}
			if (state == 1) {
				wire.p1.setY(gl_y[conet.rows]);
				obj_sets.push_back(wire);
			}
	}
}


/*
in: connet, each grid metal connection, up 1, right 2, down 4, left 8, via 16, unsure, xxx00000
in: gl_x, grid line x
in: gl_y, grid line y
out: obj_sets
*/
void grid2_via_obj(const Mat & conet, int layer, const vector<int> & gl_x, const vector<int> & gl_y, vector<MarkObj> & obj_sets)
{
	CV_Assert(conet.rows + 2 == gl_y.size() && conet.cols + 2 == gl_x.size() && conet.type() == CV_32SC1);
	MarkObj via;
	via.type = OBJ_POINT;
	via.type2 = POINT_VIA_AUTO_EXTRACT;
	via.type3 = layer;
	via.state = 0;
	via.prob = 1;

	for (int y = 0; y < conet.rows; y++) {
		const int * p_conet = conet.ptr<int>(y);
		for (int x = 0; x < conet.cols; x++) {
			if (p_conet[x] & VIA_MASK) {
				via.p0 = QPoint(gl_x[x + 1], gl_y[y + 1]); //via p0 and p1 are same
				via.p1 = via.p0;
				obj_sets.push_back(via);
			}
		}
	}
}

/*
in: connet, each grid metal connection, up 1, right 2, down 4, left 8, via 16, unsure, xxx00000
in: gl_x, grid line x
in: gl_y, grid line y
out: obj_sets
*/
void grid2_check_obj(Mat & conet, int layer, const vector<int> & gl_x, const vector<int> & gl_y, vector<MarkObj> & obj_sets)
{
	CV_Assert(conet.rows + 2 == gl_y.size() && conet.cols + 2 == gl_x.size() && conet.type() == CV_32SC1);
	MarkObj check;
	check.type = OBJ_AREA;
	check.type2 = AREA_CHECK_ERR;
	check.type3 = layer;
	check.state = 0;

	for (int y = 0; y < conet.rows; y++) {
		int * p_conet = conet.ptr<int>(y);
		for (int x = 0; x < conet.cols; x++)
			if (p_conet[x] & UNSURE_MASK) {
			check.prob = (p_conet[x] >> UNSURE_SHIFT & 7) / 7.0f;
			vector<unsigned> search_unsure_queue;
			//merge nearby unsure grid, and form a unsure check rect
			search_unsure_queue.push_back(y << 16 | x);
			int xmin = x, xmax = x, ymin = y, ymax = y;
			p_conet[x] &= ~UNSURE_MASK;
			while (!search_unsure_queue.empty()) {
				int x0 = search_unsure_queue.back();
				int y0 = x0 >> 16;
				x0 = x0 & 0xffff;
				search_unsure_queue.pop_back();
				for (int dir = 0; dir <= 3; dir++) {
					int y1 = y0 + dxy[dir][0];
					int x1 = x0 + dxy[dir][1];
					if (y1 >= 0 && y1 < conet.rows && x1 >= 0 && x1 < conet.cols &&
						conet.at<int>(y1, x1) & UNSURE_MASK) {
						search_unsure_queue.push_back(y1 << 16 | x1);
						xmin = min(x1, xmin);
						ymin = min(y1, ymin);
						xmax = max(x1, xmax);
						ymax = max(y1, ymax);
						conet.at<int>(y1, x1) &= ~UNSURE_MASK;
					}

				}
			}
			check.p0 = QPoint((gl_x[xmin] + gl_x[xmin + 1]) / 2, (gl_y[ymin] + gl_y[ymin + 1]) / 2);
			check.p1 = QPoint((gl_x[xmax + 2] + gl_x[xmax + 1]) / 2, (gl_y[ymax + 2] + gl_y[ymax + 1]) / 2);
			obj_sets.push_back(check);
			}
	}
}

unsigned long long config_fit_mask(unsigned long long rule, unsigned long long bbfm[][4], unsigned long long vbvfm[][2])
{
	unsigned long long fit_mask = 0;
	bool del_xia = false, del_cao = false, del_bian = false, del_qi = false;

	for (unsigned i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++) {
		bbfm[i][0] = 0;
		bbfm[i][1] = 0;
		bbfm[i][2] = 0;
		bbfm[i][3] = 0;
		fit_mask = fit_mask << 1 | 1;
	}

	if (rule & RULE_NO_X_POINT)
		fit_mask &= ~(1ULL << BRICK_X_0);
	if (rule & RULE_NO_T_POINT) {
		fit_mask &= ~(1ULL << BRICK_X_0);
		fit_mask &= ~(1ULL << BRICK_T_0);
		fit_mask &= ~(1ULL << BRICK_T_90);
		fit_mask &= ~(1ULL << BRICK_T_180);
		fit_mask &= ~(1ULL << BRICK_T_270);
	}

	static int dxy[4][2] = {
			{ 0, 1 },
			{ 1, 2 },
			{ 2, 1 },
			{ 1, 0 }
	};

#define ENABLE_BRICK_CONN(b0, b1, dir) do { \
	bbfm[b0][dir] |= 1ULL << b1; \
	bbfm[b1][(dir + 2) % 4] |= 1ULL << b0; } while(0)

#define DISABLE_BRICK_CONN(b0, b1, dir) do { \
	CV_Assert(bbfm[b0][dir] & 1ULL << b1); \
	bbfm[b0][dir] &= ~(1ULL << b1); \
	bbfm[b1][(dir + 2) % 4] &= ~(1ULL << b0); } while (0)

	for (unsigned i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++)
		if (fit_mask & 1ULL << i) {
		for (unsigned j = 0; j <= i; j++)
			if (fit_mask & 1ULL << j) {
			for (int dir = 0; dir <= 3; dir++) {
				int dir_1 = (dir + 2) % 4;
				if (bricks[i].a[dxy[dir][0]][dxy[dir][1]] == bricks[j].a[dxy[dir_1][0]][dxy[dir_1][1]])
					ENABLE_BRICK_CONN(i, j, dir);
			}
			}
		}

	if (rule & RULE_NO_UCONN) {
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_L_90, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_L_180, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_L_270, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_L_0, DIR_LEFT);
		rule |= RULE_NO_hCONN | RULE_NO_FCONN;
	}

	if (rule & RULE_NO_hCONN && !(rule & RULE_NO_T_POINT)) {
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_T_180, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_T_270, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_T_0, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_T_90, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_T_90, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_T_180, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_T_270, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_T_0, DIR_LEFT);
		del_xia = true;
		rule |= RULE_NO_HCONN;
		del_qi = true;
	}

	if (rule & RULE_NO_FCONN && !(rule & RULE_NO_T_POINT)) {
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_T_0, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_T_90, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_T_180, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_T_270, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_T_270, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_T_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_T_90, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_T_180, DIR_UP);
		del_xia = true;
		del_cao = true;
		del_qi = true;
	}

	if (rule & RULE_NO_fCONN && !(rule & RULE_NO_T_POINT)) {
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_T_180, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_T_270, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_T_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_T_90, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_T_90, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_T_180, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_T_270, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_T_0, DIR_UP);
		del_xia = true;
		del_bian = true;
		del_qi = true;
	}

	if (rule & RULE_NO_TT_CONN && !(rule & RULE_NO_T_POINT)) {
		rule |= RULE_NO_HCONN;
		del_xia = true;
		del_cao = true;
		del_bian = true;
	}

	if (rule & RULE_NO_HCONN && !(rule & RULE_NO_T_POINT)) {
		DISABLE_BRICK_CONN(BRICK_T_0, BRICK_T_180, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_T_90, BRICK_T_270, DIR_DOWN);
		rule |= RULE_NO_XT_CONN;
	}

	if (del_xia && !(rule & RULE_NO_T_POINT)) {
		DISABLE_BRICK_CONN(BRICK_T_0, BRICK_T_270, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_T_90, BRICK_T_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_T_180, BRICK_T_90, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_T_270, BRICK_T_180, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_T_0, BRICK_T_90, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_T_90, BRICK_T_180, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_T_180, BRICK_T_270, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_T_270, BRICK_T_0, DIR_UP);
		rule |= RULE_NO_XT_CONN;
	}

	if (del_cao && !(rule & RULE_NO_T_POINT)) {
		DISABLE_BRICK_CONN(BRICK_T_0, BRICK_T_0, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_T_90, BRICK_T_90, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_T_180, BRICK_T_180, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_T_270, BRICK_T_270, DIR_LEFT);
		rule |= RULE_NO_XT_CONN;
	}

	if (del_bian && !(rule & RULE_NO_T_POINT)) {
		DISABLE_BRICK_CONN(BRICK_T_0, BRICK_T_180, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_T_90, BRICK_T_270, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_T_180, BRICK_T_0, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_T_270, BRICK_T_90, DIR_RIGHT);
		rule |= RULE_NO_XT_CONN;
	}

	if (del_qi && !(rule & RULE_NO_X_POINT)) {
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_X_0, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_X_0, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_X_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_X_0, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_L_0, BRICK_X_0, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_L_90, BRICK_X_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_L_180, BRICK_X_0, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_L_270, BRICK_X_0, DIR_UP);
		rule |= RULE_NO_XT_CONN;
	}

	if (rule & RULE_NO_XT_CONN && !(rule & RULE_NO_T_POINT) && !(rule & RULE_NO_X_POINT)) {
		DISABLE_BRICK_CONN(BRICK_T_0, BRICK_X_0, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_T_90, BRICK_X_0, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_T_180, BRICK_X_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_T_270, BRICK_X_0, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_T_0, BRICK_X_0, DIR_RIGHT);
		DISABLE_BRICK_CONN(BRICK_T_90, BRICK_X_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_T_180, BRICK_X_0, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_T_270, BRICK_X_0, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_T_0, BRICK_X_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_T_90, BRICK_X_0, DIR_LEFT);
		DISABLE_BRICK_CONN(BRICK_T_180, BRICK_X_0, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_T_270, BRICK_X_0, DIR_RIGHT);
		rule |= RULE_NO_XX_CONN;
	}

	if (rule & RULE_NO_XX_CONN && !(rule & RULE_NO_X_POINT)) {
		DISABLE_BRICK_CONN(BRICK_X_0, BRICK_X_0, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_X_0, BRICK_X_0, DIR_RIGHT);
	}

	for (unsigned i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++)
		for (unsigned j = 0; j <= i; j++)
			for (int dir = 0; dir <= 3; dir++)  {
		int dir_1 = (dir + 2) % 4;
		CV_Assert(!(bbfm[i][dir] & 1ULL << j) == !(bbfm[j][dir_1] & 1ULL << i));
			}

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			vbvfm[i][j] = fit_mask;

	if (rule & RULE_END_WITH_VIA) {//wire end has up or bottom via, up or bottom via have wire
		vbvfm[0][0] &= ~(1ULL << BRICK_i_0) & ~(1ULL << BRICK_i_90) & ~(1ULL << BRICK_i_180) & ~(1ULL << BRICK_i_270);
		vbvfm[0][1] &= ~(1ULL << BRICK_NO_WIRE);
		vbvfm[1][0] &= ~(1ULL << BRICK_NO_WIRE);
	}

	if (rule & RULE_VIA_NO_LCONN) {
		vbvfm[0][1] &= ~(1ULL << BRICK_L_0) & ~(1ULL << BRICK_L_90) & ~(1ULL << BRICK_L_180) & ~(1ULL << BRICK_L_270);
		//Since fine_adjust limit, following rule can't be add. TODO modify fine_adjust
		//vbvfm[1][1] &= ~(1ULL << BRICK_L_0) & ~(1ULL << BRICK_L_90) & ~(1ULL << BRICK_L_180) & ~(1ULL << BRICK_L_270);
	}

	return fit_mask;
#undef ENABLE_BRICK_CONN
#undef DISABLE_BRICK_CONN	
}

void LayerBrickRuleMask::config(unsigned long long _rule)
{
	rule = _rule;
	fit_mask = config_fit_mask(rule, wwfm, vwvfm);
	via_extend_ovlap = rule & RULE_EXTEND_VIA_OVERLAP;
}

static void process_tile(ProcessTileData & t3)
{
	CV_Assert(t3.lbrm->size() == t3.lpm->size() * 2 - 1 && t3.tt->d.size() == t3.lpm->size());
	Mat prob;
	vector<LayerParam> & lpm = *t3.lpm;
	vector<LayerBrickRuleMask> & lbrm = *t3.lbrm;
	vector<LayerBrickRuleMask> & warn_lbrm = *t3.warn_lbrm;
	QPoint compute_g0;
	t3.tt->lg.resize(lbrm.size());

	if (t3.ut == NULL && t3.lt == NULL) { //top-left corner
		FindViaGridLineReq find_req(0, 0, false, false, false, false, true, true);
		int ud0 = find_via_grid_line(lpm[t3.up_down_layer], t3.tt->d[t3.up_down_layer], find_req, prob);
		int ud1 = find_via_grid_line(lpm[t3.left_right_layer], t3.tt->d[t3.left_right_layer], find_req, prob);
		CV_Assert(ud0 != ud1);
		if (ud1 == 0)
			std::swap(t3.up_down_layer, t3.left_right_layer);
		t3.tt->img_cols = t3.tt->d[0].img.cols;
		t3.tt->img_rows = t3.tt->d[0].img.rows;
		t3.tt->img_grid_x0 = 0;
		t3.tt->img_grid_y0 = 0;
		t3.tt->img_pixel_x0 = 0;
		t3.tt->img_pixel_y0 = 0;
		t3.tt->valid_grid_x0 = 1;
		t3.tt->valid_grid_y0 = 1;
		t3.tt->tile_x = 0;
		t3.tt->tile_y = 0;
		compute_g0 = QPoint(0, 0);
#if 0
		char file_name[100];
		for (int l = 0; l < (int)lpm.size(); l++) {
			sprintf(file_name, "layer_M%d.jpg", l);
			imwrite(file_name, t3.tt->d[l].img);
		}
#endif
	}

	if (t3.ut == NULL && t3.lt != NULL) { // top edge
		FindViaGridLineReq find_req(0, 0, false, false, false, false, true, true);
		//copy image
		for (int l = 0; l < (int)lpm.size(); l++) {
			CV_Assert(t3.lt->d[l].mark.rows == t3.tt->d[l].img.rows && t3.tt->d[l].img.type() == CV_8UC1);
			Mat img(t3.lt->d[l].mark.rows, t3.lt->d[l].mark.cols + t3.tt->d[l].img.cols, CV_8UC1);
			t3.lt->d[l].mark.copyTo(img(Rect(0, 0, t3.lt->d[l].mark.cols, t3.lt->d[l].mark.rows)));
			t3.tt->d[l].img.copyTo(img(Rect(t3.lt->d[l].mark.cols, 0, t3.tt->d[l].img.cols, t3.tt->d[l].img.rows)));
			t3.tt->d[l].img = img;
			if (l != 0)
				CV_Assert(t3.tt->d[l].img.size() == t3.tt->d[l - 1].img.size());
		}
		//find grid line
		int ud0 = find_via_grid_line(lpm[t3.up_down_layer], t3.tt->d[t3.up_down_layer], find_req, prob);
		t3.tt->d[t3.left_right_layer].gl_y = t3.lt->d[t3.left_right_layer].gl_y; //use left gl_y for reference
		FindViaGridLineReq find_req1(0, GRID_MAX_BIAS_PIXEL_FOR_TILE, false, false, false, true, true, true);
		int ud1 = find_via_grid_line(lpm[t3.left_right_layer], t3.tt->d[t3.left_right_layer], find_req1, prob);
		CV_Assert(ud0 == 0 && ud1 == 1);
		if (find_req1.y_align != 0)
			qDebug("detect left_right align!=0, y_align=%d", find_req1.y_align);
		t3.tt->img_cols = t3.tt->d[0].img.cols;
		t3.tt->img_rows = t3.tt->d[0].img.rows;
		t3.tt->img_grid_x0 = t3.lt->img_grid_x0 + (int)t3.lt->d[t3.up_down_layer].gl_x.size() - 2;
		t3.tt->img_grid_y0 = t3.lt->img_grid_y0 - find_req1.y_align;
		t3.tt->img_pixel_x0 = t3.lt->img_pixel_x0 + t3.lt->img_cols - t3.lt->d[0].mark.cols;
		t3.tt->img_pixel_y0 = 0;
		CV_Assert(t3.tt->d[t3.up_down_layer].gl_x[0] < lpm[t3.up_down_layer].grid_wd);
		CV_Assert(abs(t3.tt->img_pixel_x0 + t3.tt->d[t3.up_down_layer].gl_x[0] - t3.lt->img_pixel_x0 -
			t3.lt->d[t3.up_down_layer].gl_x[t3.lt->d[t3.up_down_layer].gl_x.size() - 2]) < 6); //tt->gl_x[0] align with lt->gl_x[n-2]
		t3.tt->valid_grid_x0 = t3.tt->img_grid_x0 - GRID_COPY + 2;
		t3.tt->valid_grid_y0 = t3.tt->img_grid_y0 + 1;
		t3.tt->tile_x = t3.lt->tile_x + 1;
		t3.tt->tile_y = t3.lt->tile_y;
		compute_g0 = QPoint(1, 0);
	}

	if (t3.ut != NULL && t3.lt == NULL) { //left edge
		FindViaGridLineReq find_req(0, 0, false, false, false, false, true, true);
		//copy image
		for (int l = 0; l < (int)lpm.size(); l++) {
			CV_Assert(t3.ut->d[l].mark1.cols == t3.tt->d[l].img.cols && t3.tt->d[l].img.type() == CV_8UC1);
			Mat img(t3.ut->d[l].mark1.rows + t3.tt->d[l].img.rows, t3.ut->d[l].mark1.cols, CV_8UC1);
			t3.ut->d[l].mark1.copyTo(img(Rect(0, 0, t3.ut->d[l].mark1.cols, t3.ut->d[l].mark1.rows)));
			t3.tt->d[l].img.copyTo(img(Rect(0, t3.ut->d[l].mark1.rows, t3.tt->d[l].img.cols, t3.tt->d[l].img.rows)));
			t3.tt->d[l].img = img;
			if (l != 0)
				CV_Assert(t3.tt->d[l].img.size() == t3.tt->d[l - 1].img.size());
		}
		//find grid line
		int ud1 = find_via_grid_line(lpm[t3.left_right_layer], t3.tt->d[t3.left_right_layer], find_req, prob);
		t3.tt->d[t3.up_down_layer].gl_x = t3.ut->d[t3.up_down_layer].gl_x; //use top gl_x for reference
		FindViaGridLineReq find_req0(GRID_MAX_BIAS_PIXEL_FOR_TILE, 0, false, false, true, false, true, true);
		int ud0 = find_via_grid_line(lpm[t3.up_down_layer], t3.tt->d[t3.up_down_layer], find_req0, prob);
		CV_Assert(ud0 == 0 && ud1 == 1);
		if (find_req0.x_align != 0)
			qDebug("detect up_down align!=0, x_align=%d", find_req0.x_align);
		t3.tt->img_cols = t3.tt->d[0].img.cols;
		t3.tt->img_rows = t3.tt->d[0].img.rows;
		t3.tt->img_grid_y0 = t3.ut->img_grid_y0 + (int)t3.ut->d[t3.left_right_layer].gl_y.size() - 2;
		t3.tt->img_grid_x0 = t3.ut->img_grid_x0 - find_req0.x_align;
		t3.tt->img_pixel_x0 = 0;
		t3.tt->img_pixel_y0 = t3.ut->img_pixel_y0 + t3.ut->img_rows - t3.ut->d[0].mark1.rows;
		CV_Assert(t3.tt->d[t3.left_right_layer].gl_y[0] < lpm[t3.left_right_layer].grid_wd);
		CV_Assert(abs(t3.tt->img_pixel_y0 + t3.tt->d[t3.left_right_layer].gl_y[0] - t3.ut->img_pixel_y0 -
			t3.ut->d[t3.left_right_layer].gl_y[t3.ut->d[t3.left_right_layer].gl_y.size() - 2]) < 6);//tt->gl_y[0] align with lt->gl_y[n-2]
		t3.tt->valid_grid_x0 = t3.tt->img_grid_x0 + 1;
		t3.tt->valid_grid_y0 = t3.tt->img_grid_y0 - GRID_COPY + 2;
		t3.tt->tile_x = t3.ut->tile_x;
		t3.tt->tile_y = t3.ut->tile_y + 1;
		compute_g0 = QPoint(0, 1);
	}

	if (t3.ut != NULL && t3.lt != NULL) {
		//copy image
		for (int l = 0; l < (int)lpm.size(); l++) {
			Mat img(t3.ut->d[l].mark1.rows + t3.tt->d[l].img.rows, t3.lt->d[l].mark.cols + t3.tt->d[l].img.cols, CV_8UC1);
			CV_Assert(abs(t3.lt->d[l].mark.rows - img.rows) <= 4 && abs(t3.ut->d[l].mark1.cols - img.cols) <= 4);
			Mat right_img = (t3.lt->d[l].mark.rows > img.rows) ?
				t3.lt->d[l].mark(Rect(0, t3.lt->d[l].mark.rows - img.rows, t3.lt->d[l].mark.cols, img.rows)) :
				t3.lt->d[l].mark;
			right_img.copyTo(img(Rect(0, img.rows - right_img.rows, right_img.cols, right_img.rows)));
			Mat top_img = (t3.ut->d[l].mark1.cols > img.cols) ?
				t3.ut->d[l].mark1(Rect(t3.ut->d[l].mark1.cols - img.cols, 0, img.cols, t3.ut->d[l].mark1.rows)) :
				t3.ut->d[l].mark1;
			top_img.copyTo(img(Rect(img.cols - top_img.cols, 0, top_img.cols, top_img.rows)));
			t3.tt->d[l].img.copyTo(img(Rect(t3.lt->d[l].mark.cols, t3.ut->d[l].mark1.rows, t3.tt->d[l].img.cols, t3.tt->d[l].img.rows)));
			t3.tt->d[l].img = img;
			if (l != 0)
				CV_Assert(t3.tt->d[l].img.size() == t3.tt->d[l - 1].img.size());
		}
		//find grid line
		t3.tt->img_cols = t3.tt->d[0].img.cols;
		t3.tt->img_rows = t3.tt->d[0].img.rows;
		t3.tt->img_pixel_x0 = t3.lt->img_pixel_x0 + t3.lt->img_cols - t3.lt->d[0].mark.cols;
		t3.tt->img_pixel_y0 = t3.ut->img_pixel_y0 + t3.ut->img_rows - t3.ut->d[0].mark1.rows;
		FindViaGridLineReq find_req(GRID_MAX_BIAS_PIXEL_FOR_TILE, 0, false, false, true, false, true, true);
		t3.tt->d[t3.up_down_layer].gl_x = t3.ut->d[t3.up_down_layer].gl_x; //use top gl_x for reference
		for (int i = 0; i < (int)t3.tt->d[t3.up_down_layer].gl_x.size(); i++)
			t3.tt->d[t3.up_down_layer].gl_x[i] += t3.ut->img_pixel_x0 - t3.tt->img_pixel_x0;
		int ud0 = find_via_grid_line(lpm[t3.up_down_layer], t3.tt->d[t3.up_down_layer], find_req, prob);
		FindViaGridLineReq find_req0(0, GRID_MAX_BIAS_PIXEL_FOR_TILE, false, false, false, true, true, true);
		t3.tt->d[t3.left_right_layer].gl_y = t3.lt->d[t3.left_right_layer].gl_y; //use left gl_y for reference
		for (int i = 0; i < (int)t3.tt->d[t3.left_right_layer].gl_y.size(); i++)
			t3.tt->d[t3.left_right_layer].gl_y[i] += t3.lt->img_pixel_y0 - t3.tt->img_pixel_y0;
		int ud1 = find_via_grid_line(lpm[t3.left_right_layer], t3.tt->d[t3.left_right_layer], find_req0, prob);
		CV_Assert(ud0 == 0 && ud1 == 1);		
		t3.tt->img_grid_x0 = t3.lt->img_grid_x0 + (int)t3.lt->d[0].gl_x.size() - 2;
		t3.tt->img_grid_y0 = t3.ut->img_grid_y0 + (int)t3.ut->d[0].gl_y.size() - 2;		
		t3.tt->valid_grid_x0 = t3.tt->img_grid_x0 - GRID_COPY + 2;
		t3.tt->valid_grid_y0 = t3.tt->img_grid_y0 - GRID_COPY + 2;
		int up_dx = 10000, left_dy = 10000;
		int up_dy = abs(t3.tt->img_pixel_y0 + t3.tt->d[t3.left_right_layer].gl_y[0] - t3.ut->img_pixel_y0 -
			t3.ut->d[t3.left_right_layer].gl_y[t3.ut->d[t3.left_right_layer].gl_y.size() - 2]); //this gl_y[0] is same as up gl_y[n-2]
		int left_dx = abs(t3.tt->img_pixel_x0 + t3.tt->d[t3.up_down_layer].gl_x[0] - t3.lt->img_pixel_x0 -
			t3.lt->d[t3.up_down_layer].gl_x[t3.lt->d[t3.up_down_layer].gl_x.size() - 2]); //this gl_x[0] is same as left gl_x[n-2]
		for (int idx = -1; idx <= 1; idx++) {
			int max_dx = 0, max_dy = 0;
			for (int i = 1; i < t3.tt->d[t3.up_down_layer].gl_x.size() - 3; i++) {
				int dx = abs(t3.tt->img_pixel_x0 + t3.tt->d[t3.up_down_layer].gl_x[i] - t3.ut->img_pixel_x0 -
					t3.ut->d[t3.up_down_layer].gl_x[i + idx]);
				max_dx = max(max_dx, dx);
			}
			up_dx = min(max_dx, up_dx);
			for (int i = 1; i < t3.tt->d[t3.left_right_layer].gl_y.size() - 3; i++) {
				int dy = abs(t3.tt->img_pixel_y0 + t3.tt->d[t3.left_right_layer].gl_y[i] - t3.lt->img_pixel_y0 -
					t3.lt->d[t3.left_right_layer].gl_y[i + idx]);
				max_dy = max(max_dy, dy);
			}
			left_dy = min(max_dy, left_dy);
		}
		qDebug("Tile up_dx = %d, up_dy = %d; left_dx =%d, left_dy = %d.", up_dx, up_dy, left_dx, left_dy);
		CV_Assert(t3.tt->d[t3.up_down_layer].gl_x[0] < lpm[t3.up_down_layer].grid_wd);
		CV_Assert(t3.tt->d[t3.left_right_layer].gl_y[0] < lpm[t3.left_right_layer].grid_wd);
		CV_Assert(left_dx < 6 && left_dy < 6);//tt->gl_x[0] align with lt->gl_x[n-2]
		CV_Assert(up_dx < 6 && up_dy < 6);//tt->gl_y[0] align with lt->gl_y[n-2]
		CV_Assert(abs(t3.tt->img_grid_x0 - t3.ut->img_grid_x0) <= 1 && abs(t3.tt->img_grid_y0 - t3.lt->img_grid_y0) <= 1);		
		t3.tt->tile_x = t3.ut->tile_x;
		t3.tt->tile_y = t3.lt->tile_y;
		CV_Assert(t3.tt->tile_x == t3.lt->tile_x + 1 && t3.tt->tile_y == t3.ut->tile_y + 1);
		compute_g0 = QPoint(1, 1);
	}

	for (int l = 0; l < lpm.size(); l++) {
		//other layer isn't allow to generate new gl_x, gl_y
		FindViaGridLineReq find_req(GRID_MAX_BIAS_PIXEL_FOR_LAYER, GRID_MAX_BIAS_PIXEL_FOR_LAYER, true, true, true, true, false, false);
		if (t3.debug) {
			t3.tt->d[l].mark1.create(t3.tt->d[l].img.rows, t3.tt->d[l].img.cols, CV_8UC1);
			t3.tt->d[l].mark1 = Scalar(0);
		}
		t3.tt->d[l].mark.create(t3.tt->d[l].img.rows, t3.tt->d[l].img.cols, CV_8UC1);
		t3.tt->d[l].mark = M_UNKNOW;
		if (l != t3.up_down_layer)
			t3.tt->d[l].gl_x = t3.tt->d[t3.up_down_layer].gl_x;
		if (l != t3.left_right_layer)
			t3.tt->d[l].gl_y = t3.tt->d[t3.left_right_layer].gl_y;
		int up_down;
		if (l == 0)
			find_via_grid_line2(lpm[l], t3.tt->d[l], find_req, prob);
		else
			up_down = find_via_grid_line(lpm[l], t3.tt->d[l], find_req, prob);
		t3.tt->lg[2 * l].grid_rows = (int)t3.tt->d[l].gl_y.size() - 2 + ((t3.ut != NULL) ? GRID_COPY : 0);
		t3.tt->lg[2 * l].grid_cols = (int)t3.tt->d[l].gl_x.size() - 2 + ((t3.lt != NULL) ? GRID_COPY : 0);
		t3.tt->lg[2 * l].grid_infos.resize(t3.tt->lg[2 * l].grid_rows * t3.tt->lg[2 * l].grid_cols);
		//Copy GridInfo
		QPoint pic_g0(1, 1);
		if (t3.lt != NULL) {
			copy_grid_info(t3.lt->lg[2 * l], t3.tt->lg[2 * l],
				QRect(t3.lt->lg[2 * l].grid_cols - GRID_COPY + 1, 1 + t3.tt->img_grid_y0 - t3.lt->img_grid_y0, GRID_COPY, t3.lt->lg[2 * l].grid_rows),
				QPoint(1, 1));
			pic_g0.setX(GRID_COPY + 1);
		}
		if (t3.ut != NULL) {
			copy_grid_info(t3.ut->lg[2 * l], t3.tt->lg[2 * l],
				QRect(1 + t3.tt->img_grid_x0 - t3.ut->img_grid_x0, t3.ut->lg[2 * l].grid_rows - GRID_COPY + 1, t3.ut->lg[2 * l].grid_cols, GRID_COPY),
				QPoint(1, 1));
			pic_g0.setY(GRID_COPY + 1);
		}
		post_process_via_prob(prob, lpm[l].param3, t3.tt->d[l].gl_x, t3.tt->d[l].gl_y,
			t3.tt->lg[2 * l], pic_g0, t3.tt->d[l].mark1); //0, 2, 4 for via
		if (l != 0) {
			//TODO: suppose >0.6 is via, more precise? 
#if 1
			vector<QPoint> vias;
			for (int y = 0; y < prob.rows; y++)
				for (int x = 0; x < prob.cols; x++)
					if (prob.at<float>(y, x)>0.6f)
						vias.push_back(QPoint(t3.tt->d[l].gl_x[x], t3.tt->d[l].gl_y[y]));
#endif

			for (int i = 0; i < vias.size(); i++)
				fill_circle(t3.tt->d[l].mark, vias[i].x(), vias[i].y(), lpm[l].via_rd0 + 2, M_V,
				QRect(0, 0, t3.tt->d[l].img.cols, t3.tt->d[l].img.rows));
			Mat img = t3.tt->d[l].img.clone();
			remove_via(t3.tt->d[l].mark, img, up_down, lpm[l].grid_wd - lpm[l].wire_wd);

			//compute grid prob		
			compute_grid_prob(img, lpm[l], t3.tt->d[l].gl_x,
				t3.tt->d[l].gl_y, lbrm[2 * l - 1].fit_mask, prob, t3.tt->d[l].mark1, !(lbrm[2 * l - 1].rule & RULE_NO_DUMY_FILLING));
			t3.tt->lg[2 * l - 1].grid_rows = (int)t3.tt->d[l].gl_y.size() - 2 + ((t3.ut != NULL) ? GRID_COPY : 0);
			t3.tt->lg[2 * l - 1].grid_cols = (int)t3.tt->d[l].gl_x.size() - 2 + ((t3.lt != NULL) ? GRID_COPY : 0);
			t3.tt->lg[2 * l - 1].grid_infos.resize(t3.tt->lg[2 * l - 1].grid_rows * t3.tt->lg[2 * l - 1].grid_cols);
			//Copy GridInfo
			if (t3.lt != NULL)
				copy_grid_info(t3.lt->lg[2 * l - 1], t3.tt->lg[2 * l - 1],
				QRect(t3.lt->lg[2 * l - 1].grid_cols - GRID_COPY + 1, 1 + t3.tt->img_grid_y0 - t3.lt->img_grid_y0, GRID_COPY, t3.lt->lg[2 * l - 1].grid_rows),
				QPoint(1, 1));

			if (t3.ut != NULL)
				copy_grid_info(t3.ut->lg[2 * l - 1], t3.tt->lg[2 * l - 1],
				QRect(1 + t3.tt->img_grid_x0 - t3.ut->img_grid_x0, t3.ut->lg[2 * l].grid_rows - GRID_COPY + 1, t3.ut->lg[2 * l].grid_cols, GRID_COPY),
				QPoint(1, 1));

			post_process_grid_prob(prob, lbrm[2 * l - 1], t3.tt->d[l].gl_x, t3.tt->d[l].gl_y,
				t3.tt->lg[2 * l - 1], pic_g0, t3.tt->d[l].mark1); //1,3, 5 for wire
		}
	}
	if (!t3.debug) {
		//save right and bottom image
		for (unsigned l = 0; l < lpm.size(); l++)
			if (l != 0)
				CV_Assert(t3.tt->d[l].img.rows == t3.tt->d[l - 1].img.rows && t3.tt->d[l].img.cols == t3.tt->d[l - 1].img.cols);

		int offset = lpm[t3.up_down_layer].wire_wd / 2 + GRID_MAX_BIAS_PIXEL_FOR_LAYER + 1;
		Rect right_rect(t3.tt->d[t3.up_down_layer].gl_x[t3.tt->d[t3.up_down_layer].gl_x.size() - 2] - offset, 0,
			t3.tt->d[t3.up_down_layer].img.cols - t3.tt->d[t3.up_down_layer].gl_x[t3.tt->d[t3.up_down_layer].gl_x.size() - 2] + offset,
			t3.tt->d[t3.up_down_layer].img.rows);
		offset = lpm[t3.left_right_layer].wire_wd / 2 + GRID_MAX_BIAS_PIXEL_FOR_LAYER + 1;
		Rect bot_rect(0, t3.tt->d[t3.left_right_layer].gl_y[t3.tt->d[t3.left_right_layer].gl_y.size() - 2] - offset,
			t3.tt->d[t3.left_right_layer].img.cols, t3.tt->d[t3.left_right_layer].img.rows - t3.tt->d[t3.left_right_layer].gl_y[t3.tt->d[t3.left_right_layer].gl_y.size() - 2] + offset);
		for (unsigned l = 0; l < lpm.size(); l++) {
			t3.tt->d[l].mark.release();
			t3.tt->d[l].img(right_rect).copyTo(t3.tt->d[l].mark);
			t3.tt->d[l].img(bot_rect).copyTo(t3.tt->d[l].mark1);
			t3.tt->d[l].img.release();
		}
	}

	assemble_grid_multilayer(t3.tt->lg, lbrm, warn_lbrm, *(t3.tt), QRect(compute_g0, QPoint(0x70000000, 0x70000000)), OBEY_RULE_STRONG);
	for (int l = 0; l < lpm.size(); l++) {
		t3.tt->d[l].conet = t3.tt->d[l].conet(Rect(compute_g0.x(), compute_g0.y(),
			t3.tt->d[l].conet.cols - compute_g0.x(), t3.tt->d[l].conet.rows - compute_g0.y()));
	}
}


Mat VWExtractStat::get_mark(int layer) {
	if (ts.empty())
		return Mat();
	CV_Assert(layer < lpm.size());
	return ts[0].d[layer].mark;
}
Mat VWExtractStat::get_mark1(int layer) {
	if (ts.empty())
		return Mat();
	CV_Assert(layer < lpm.size());
	return ts[0].d[layer].mark1;
}
Mat VWExtractStat::get_mark2(int) {
	return Mat();
}
Mat VWExtractStat::get_mark3(int) {
	return Mat();
}

int VWExtractStat::extract(string file_name, QRect, std::vector<MarkObj> & obj_sets)
{
	vector<LayerBrickRuleMask> lbrm(lpm.size() * 2 - 1);
	vector<LayerBrickRuleMask> warn_lbrm(lpm.size() * 2 - 1);
	for (int l = 0; l < lpm.size(); l++) {
		int lbrm_idx = (l == 0) ? 0 : 2 * l - 1;
		lbrm[lbrm_idx].config(lpm[l].rule);
		warn_lbrm[lbrm_idx].config(lpm[l].warning_rule | lpm[l].rule);
		lbrm[lbrm_idx + 1].fit_mask = 3; //let via_layer fit_mask =3
		warn_lbrm[lbrm_idx + 1].fit_mask = 3;
	}	
	ts.resize(1);
	ts[0].d.resize(lpm.size());
	unsigned char ll = file_name[file_name.size() - 5];
	for (int l = 0; l < lpm.size(); l++) {
		file_name[file_name.size() - 5] = ll++;
		ts[0].d[l].img = imread(file_name, 0);
		CV_Assert(ts[0].d[l].img.type() == CV_8UC1 && !ts[0].d[l].img.empty());
	}

	ProcessTileData ptd;
	ptd.debug = true;
	ptd.left_right_layer = 1;
	ptd.up_down_layer = 2;
	ptd.lbrm = &lbrm;
	ptd.warn_lbrm = &warn_lbrm;
	ptd.lpm = &lpm;
	ptd.lt = NULL;
	ptd.ut = NULL;
	ptd.tt = &ts[0];

	process_tile(ptd);
	ts[0].lg.clear();
	obj_sets.clear();
	for (int l = 0; l < lpm.size(); l++) {
		if (l > 0) {
			grid2_wire_obj(ts[0].d[l].conet, l, ts[0].d[l].gl_x, ts[0].d[l].gl_y, obj_sets);
			grid2_check_obj(ts[0].d[l].conet, l, ts[0].d[l].gl_x, ts[0].d[l].gl_y, obj_sets);
		}
		grid2_via_obj(ts[0].d[l].conet, l, ts[0].d[l].gl_x, ts[0].d[l].gl_y, obj_sets);
		qInfo("l=%d, gl_x0=%d, gl_y0=%d", l, ts[0].d[l].gl_x[0], ts[0].d[l].gl_y[0]);
	}
	ts.clear();
	return 0;
}

int VWExtractStat::extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets)
{
	vector<LayerBrickRuleMask> lbrm(lpm.size() * 2 - 1);
	vector<LayerBrickRuleMask> warn_lbrm(lpm.size() * 2 - 1);

	for (int l = 0; l < lpm.size(); l++) {
		int lbrm_idx = (l == 0) ? 0 : 2 * l - 1;
		lbrm[lbrm_idx].config(lpm[l].rule);
		warn_lbrm[lbrm_idx].config(lpm[l].warning_rule | lpm[l].rule);
		lbrm[lbrm_idx + 1].fit_mask = 3;
		warn_lbrm[lbrm_idx + 1].fit_mask = 3;
		qInfo("layer%d: w=%d,g=%d,vr=%d,rule=%llx,wrule=%llx,via_th=%f,wire_th=%f,vw_ratio=%f", l, lpm[l].wire_wd, lpm[l].grid_wd,
			lpm[l].via_rd0, lpm[l].rule, lpm[l].warning_rule, lpm[l].param1, lpm[l].param2, lpm[l].param3);
	}
#if SAVE_RST_TO_FILE
	FILE * fp;
	fp = fopen("result.txt", "w");
#endif
	int block_x, block_y;
	ic_layer[0]->getBlockNum(block_x, block_y);
	int block_width = ic_layer[0]->getBlockWidth();
	int scale = 32768 / block_width;
	vector <ProcessTileData> ptd_sets;
	obj_sets.clear();
	vector<TileData> ts;
	for (int area_idx = 0; area_idx < area_.size(); area_idx++) {
		//extend area to sr, this is because extract can't process edge grid
		QRect sr = area_[area_idx].rect.marginsAdded(QMargins(lpm[1].grid_wd * scale, lpm[1].grid_wd *scale, lpm[1].grid_wd*scale, lpm[1].grid_wd*scale));
		sr &= QRect(0, 0, block_x << 15, block_y << 15);
		if (sr.width() <= 0x8000 || sr.height() <= 0x8000) {
			qWarning("Picture(%d,%d) too small!", sr.width(), sr.height());
			continue;
		}
		
		QRect sb(QPoint((sr.left() + 0x4000) >> 15, (sr.top() + 0x4000) >> 15),
			QPoint((sr.right() - 0x4000) >> 15, (sr.bottom() - 0x4000) >> 15));
		CV_Assert(sb.right() >= sb.left() && sb.bottom() >= sb.top());
		ts.resize(sb.width() * sb.height());
		int lx = sr.left() - (sb.left() << 15);
		int rx = sr.right() - ((sb.right() + 1) << 15);
		int ty = sr.top() - (sb.top() << 15);
		int by = sr.bottom() - ((sb.bottom() + 1) << 15);
		int up_down_layer = 1, left_right_layer = 2;
		for (int xay = sb.left() + sb.top(); xay <= sb.right() + sb.bottom(); xay++) {
			ptd_sets.clear();
			for (int x0 = sb.left(); x0 <= sb.right(); x0++) {
				int y0 = xay - x0;
				//1 load image
				if (sb.contains(x0, y0)) {
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

					ts[(y0 - sb.top()) * sb.width() + x0 - sb.left()].d.resize(lpm.size());
					TileData * tt = &ts[(y0 - sb.top()) * sb.width() + x0 - sb.left()];
					for (int l = 0; l < lpm.size(); l++) {
						tt->d[l].img.create(height[0] + height[1] + height[2], wide[0] + wide[1] + wide[2], CV_8U);
						//When extract width(height) > 2 * image width
						//if loading image is not at the edge, load 1*1 image; if at the edge, load 1*1 or 1*2;
						//if at the corner, load 1*1 or 1*2, or 2*2.
						//When extract width(height) < 2 * image width
						//if loading image is at the edge, load 1*1, 1*2 or 1*3 image, if at the corner, load 1*1 or 1*2 or 2*2 or 2*3 or 3*3 
						for (int y = y0 - 1, img_y = 0; y <= y0 + 1; y++) {
							int dy = y - y0;
							for (int x = x0 - 1, img_x = 0; x <= x0 + 1; x++) {
								int dx = x - x0;
								if (wide[dx + 1] != 0 && height[dy + 1] != 0) {
									vector<uchar> encode_img;
									if (ic_layer[l]->getRawImgByIdx(encode_img, x, y, 0, 0, false) != 0) {
										qCritical("load image error at l=%d, (%d,%d)", l, x, y);
										return -1;
									}
									Mat image = imdecode(Mat(encode_img), 0);
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
										qDebug("load image%d_%d, (%d,%d),(%d,%d)", y, x, s.left(), s.top(), s.width(), s.height());
									image(Rect(s.left(), s.top(), s.width(), s.height())).copyTo(tt->d[l].img(Rect(img_x, img_y, wide[dx + 1], height[dy + 1])));
								}
								img_x += wide[dx + 1];
							}
							img_y += height[dy + 1];
						}
					}
					//2 prepare ProcessTileData
					ProcessTileData ptd;
					ptd.debug = false;
					ptd.lbrm = &lbrm;
					ptd.warn_lbrm = &warn_lbrm;
					ptd.lpm = &lpm;
					ptd.up_down_layer = up_down_layer;
					ptd.left_right_layer = left_right_layer;
					ptd.tt = tt;
					ptd.lt = (x0 == sb.left()) ? NULL : &ts[(y0 - sb.top()) * sb.width() + x0 - sb.left() - 1];
					ptd.ut = (y0 == sb.top()) ? NULL : &ts[(y0 - sb.top() - 1) * sb.width() + x0 - sb.left()];
					ptd_sets.push_back(ptd);
				}
			}
			//3 ProcessTileData
#if 1
			if (ptd_sets.size() == 1) {
				char file_name[100];
				for (int l = 0; l < (int)lpm.size(); l++) {
					sprintf(file_name, "layer_M%d.jpg", l);
					imwrite(file_name, ptd_sets[0].tt->d[l].img);
				}
			}
#endif
#if PARALLEL 
			QtConcurrent::blockingMap<vector <ProcessTileData> >(ptd_sets, process_tile);
#else
			for (int i = 0; i < ptd_sets.size(); i++) 
				process_tile(ptd_sets[i]);
			
#endif
			if (xay == sb.left() + sb.top()) {
				up_down_layer = ptd_sets[0].up_down_layer;
				left_right_layer = ptd_sets[0].left_right_layer;
			}
			//4 release previous lg memory
			for (int x0 = sb.left(); x0 <= sb.right(); x0++) {
				int y0 = xay - 1 - x0;
				if (sb.contains(x0, y0)) {
					ts[(y0 - sb.top()) * sb.width() + x0 - sb.left()].lg.clear();
					for (int l = 0; l < lpm.size(); l++) {
						ts[(y0 - sb.top()) * sb.width() + x0 - sb.left()].d[l].mark.release();
						ts[(y0 - sb.top()) * sb.width() + x0 - sb.left()].d[l].mark1.release();
					}
				}
			}
		}
		//5 compute global gl_x, gl_y and assemble global conet
		for (int l = 0; l < lpm.size(); l++) {
			vector<int> gl_x, gl_y;
			for (int x = sb.left(); x <= sb.right(); x++) {
				int idx = x - sb.left();
				int end = (x != sb.right()) ? (int) ts[idx].d[l].gl_x.size() - 2 : (int) ts[idx].d[l].gl_x.size();
				for (int i = 0; i < end; i++)
					gl_x.push_back(sr.left() / scale + ts[idx].img_pixel_x0 + ts[idx].d[l].gl_x[i] + 2); //TODO: Check why +2?
			}

			for (int y = sb.top(); y <= sb.bottom(); y++) {
				int idx = (y - sb.top()) * sb.width();
				int end = (y != sb.bottom()) ? (int) ts[idx].d[l].gl_y.size() - 2 : (int) ts[idx].d[l].gl_y.size();
				for (int i = 0; i < end; i++)
					gl_y.push_back(sr.top() / scale + ts[idx].img_pixel_y0 + ts[idx].d[l].gl_y[i] + 2); //TODO: check why + 2
			}
			Mat conet((int)gl_y.size() - 2, (int)gl_x.size() - 2, CV_32SC1);
			conet = Scalar(0);
			Rect cr(0, 0, conet.cols, conet.rows);
			for (int i = 0; i < ts.size(); i++) {
				int x = ts[i].valid_grid_x0 - 1;
				int y = ts[i].valid_grid_y0 - 1;
				Rect sr(x, y, ts[i].d[l].conet.cols, ts[i].d[l].conet.rows);
				Rect dr = sr & cr;
				ts[i].d[l].conet(Rect(dr.x - sr.x, dr.y - sr.y, dr.width, dr.height)).copyTo(conet(dr));
			}
			qDebug("Layer%d, total Grid x=%d, y=%d", l, conet.cols, conet.rows);
			if (l > 0) {
				grid2_wire_obj(conet, l, gl_x, gl_y, obj_sets);
				grid2_check_obj(conet, l, gl_x, gl_y, obj_sets);
			}
			grid2_via_obj(conet, l, gl_x, gl_y, obj_sets);
		}
		for (int i = 0; i < obj_sets.size(); i++) {
			obj_sets[i].p0 *= scale;
			obj_sets[i].p1 *= scale;
		}
#if SAVE_RST_TO_FILE		
		for (int i = 0; i < obj_sets.size(); i++) {
			unsigned t = obj_sets[i].type;
			t = (t << 8) | obj_sets[i].type2;
			t = (t << 8) | obj_sets[i].type3;
			fprintf(fp, "t=%d, (%d,%d)->(%d,%d)\n", t, obj_sets[i].p0.x(), obj_sets[i].p0.y(),
				obj_sets[i].p1.x(), obj_sets[i].p1.y());
		}
#endif
		ts.clear();
	}
#if SAVE_RST_TO_FILE
	fclose(fp);
#endif
	return 0;
}
