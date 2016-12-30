#include "vwextract.h"
#include <algorithm>
#include <functional>
#include <list>
#include <queue>  
#include <cfloat>
#include <set>
#include <stdio.h>
using namespace std;
#define SGN(x) (((x)>0) ? 1 : (((x)==0) ? 0 : -1))
#define WEIGHT_FB 3
#define VIA_MAX_BIAS_PIXEL	3
#define GRID_MAX_BIAS_PIXEL_FOR_LAYER	2
#define GRID_MAX_BIAS_PIXEL_FOR_TILE	2
#define GRID_COPY			3
#define PROB2_INT			10000
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

#define DIR_UP_MASK			(1 << DIR_UP)
#define DIR_RIGHT_MASK		(1 << DIR_RIGHT)
#define DIR_DOWN_MASK		(1 << DIR_DOWN)
#define DIR_LEFT_MASK		(1 << DIR_LEFT)
#define VIA_MASK			(1 << 5)

#define SAVE_RST_TO_FILE	1
#ifndef QT_DEBUG
#undef CV_Assert
#define CV_Assert(x) do {if (!(x)) {qFatal("Wrong at %s, %d", __FILE__, __LINE__);}} while(0)
#endif
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
				DIR_UP_MASK
		},

		{ 0.9f, {
				{ 0, 0, 0 },
				{ 0, 1, 1 },
				{ 0, 0, 0 } },
				DIR_RIGHT_MASK
		},

		{ 0.9f, {
				{ 0, 0, 0 },
				{ 0, 1, 0 },
				{ 0, 1, 0 } },
				DIR_DOWN_MASK
		},

		{ 0.9f, {
				{ 0, 0, 0 },
				{ 1, 1, 0 },
				{ 0, 0, 0 } },
				DIR_LEFT_MASK
		},
		{ 1.0f, {
				{ 0, 1, 0 },
				{ 0, 1, 0 },
				{ 0, 1, 0 } },
				DIR_UP_MASK | DIR_DOWN_MASK
		},

		{ 1.0f, {
				{ 0, 0, 0 },
				{ 1, 1, 1 },
				{ 0, 0, 0 } },
				DIR_LEFT_MASK | DIR_RIGHT_MASK
		},

		{ 0.8f, {
				{ 0, 1, 0 },
				{ 0, 1, 1 },
				{ 0, 0, 0 } },
				DIR_UP_MASK | DIR_RIGHT_MASK
		},

		{ 0.8f, {
				{ 0, 0, 0 },
				{ 0, 1, 1 },
				{ 0, 1, 0 } },
				DIR_RIGHT_MASK | DIR_DOWN_MASK
		},

		{ 0.8f, {
				{ 0, 0, 0 },
				{ 1, 1, 0 },
				{ 0, 1, 0 } },
				DIR_DOWN_MASK | DIR_LEFT_MASK
		},

		{ 0.8f, {
				{ 0, 1, 0 },
				{ 1, 1, 0 },
				{ 0, 0, 0 } },
				DIR_UP_MASK | DIR_LEFT_MASK
		},

		{ 0.7f, {
				{ 0, 1, 0 },
				{ 0, 1, 1 },
				{ 0, 1, 0 } },
				DIR_UP_MASK | DIR_RIGHT_MASK | DIR_DOWN_MASK
		},

		{ 0.7f, {
				{ 0, 0, 0 },
				{ 1, 1, 1 },
				{ 0, 1, 0 } },
				DIR_RIGHT_MASK | DIR_DOWN_MASK | DIR_LEFT_MASK
		},

		{ 0.7f, {
				{ 0, 1, 0 },
				{ 1, 1, 0 },
				{ 0, 1, 0 } },
				DIR_UP_MASK | DIR_LEFT_MASK | DIR_DOWN_MASK
		},

		{ 0.7f, {
				{ 0, 1, 0 },
				{ 1, 1, 1 },
				{ 0, 0, 0 } },
				DIR_UP_MASK | DIR_LEFT_MASK | DIR_RIGHT_MASK
		},

		{ 0.5f, {
				{ 0, 1, 0 },
				{ 1, 1, 1 },
				{ 0, 1, 0 } },
				DIR_UP_MASK | DIR_DOWN_MASK | DIR_LEFT_MASK | DIR_RIGHT_MASK
		}	
};

struct GridInfo {
	int x, y;
	vector<unsigned> brick_order; //first is prob, second is brick no
	int brick_choose; //brick no chosed
	int po; //prefer order, as brick_order's index
	int abm; //available brick mask
	GridInfo()
	{
		brick_choose = BRICK_INVALID;
	}
};

struct LayerBrickRuleMask {
	unsigned long long wwfm[64][4]; //wire wire fit mask
	unsigned long long vwvfm[2][2]; //via - wire -via fit mask
	unsigned long long fit_mask; //wire available mask
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
	int img_pixel_x0, img_pixel_y0; // it is load image pixel zuobiao for gl_x[0], gl_y[0]
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
	vector<LayerParam> * lpm;
	bool debug;
	int up_down_layer, left_right_layer;
};

class VWExtractStat : public VWExtract {
protected:
	vector<TileData> ts;
public:
	Mat get_mark(int layer);
	Mat get_mark1(int layer);
	Mat get_mark2(int layer);
	Mat get_mark3(int layer);
	int train(string, const std::vector<MarkObj> &) { return 0; }
	int extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
    int train(vector<ICLayerWr *> &, const std::vector<MarkObj> &) { return 0; }
    int extract(vector<ICLayerWr *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets);
	void get_feature(int, int, int, vector<float> &) {}
};
/*
Use kmean 2 julei
in: bins, stat histogram 
out: th, julei center
out: var_win, julei window
return: <1, result is bad, more > 1, more good
*/
static float cal_threshold(const vector<unsigned> & bins, vector<float> & th, vector<float> & var_win, float ratio=0)
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
		int dx = (int) sqrt((float) r*r - (y - y0) *(y - y0));
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
		mmin[2] = min(mmin[2], (int) b[0]);
		mmax[2] = max(mmax[2], (int) b[0]);
		
		s[3] += b[1];
		mmin[3] = min(mmin[3], (int) b[1]);
		mmax[3] = max(mmax[3], (int) b[1]);
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
	for (int y = -r, i=0; y <= r; y++, i++) {
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
	return Vec<float, 5>((float) log((s[1] + 0.001f) / (s[0] + 0.001f)) * 10,
		s[1] - s[0],
		(float) log((s[3] + 0.001f) / (s[2] + 0.001f)) * 10,
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
In wire_wd: metal width
In grid_wd: grid width
In wire_cred: wire reliablity
In gl_x: up-down grid line
In gl_y: left-right grid line
In mask: available brick mask
out prob: brick prob for each grid, 3-dim Mat (gl_y.size() * gl_x.size() * Brick_NUM),
		  more near to 1 means bigger probability for the brick,
		  more near to 0 means less probability for the brick.
Features and Restriction
1 Assume wire is lighter than wire. 
2 Assume at least 3 insu grid within 3*3
*/
static void compute_grid_prob(const Mat & img, struct LayerParam & lpm,
	const vector<int> & gl_x, const vector<int> & gl_y, unsigned long long mask, Mat & prob, Mat & mark)
{
	int wire_wd = lpm.wire_wd, grid_wd = lpm.grid_wd, wire_cred = lpm.param2;
	CV_Assert(grid_wd > wire_wd && wire_cred <= 1 && wire_cred >= 0 && gl_x[0] >= wire_wd / 2 && gl_y[0] >= wire_wd / 2);
	CV_Assert(gl_x[gl_x.size() - 1] <= img.cols - wire_wd + wire_wd / 2 - 2);
	CV_Assert(gl_y[gl_y.size() - 1] <= img.rows - wire_wd + wire_wd / 2 - 2);
	CV_Assert(mark.empty() || mark.type() == CV_8UC1 && mark.size == img.size);

	Mat ig;
	integral(img, ig, CV_32S);
	int sz[] = { (int)gl_y.size(), (int)gl_x.size(), (int) sizeof(bricks) / sizeof(bricks[0])};
	prob.create(3, sz, CV_32F);
	prob = Scalar(0);
	//qDebug("prob size0=%d, step0=%d, size1=%d, step1=%d", prob.size[0], prob.step.p[0], prob.size[1], prob.step.p[1]);

	//1 compute each grid average gray
	vector<int> glx_edge, gly_edge;
	vector<unsigned> bins(512, 0);	
	compute_grid_edge(gl_x, wire_wd, glx_edge);
	compute_grid_edge(gl_y, wire_wd, gly_edge);	
	Mat gray((int) gl_y.size() * 2 - 1, (int) gl_x.size() * 2 - 1, CV_32FC1);

	for (int y = 0; y < gray.rows; y++)
		for (int x = 0; x < gray.cols; x++) {
			//(gly_edge[y]+1, gly_edge[x]+1) (gly_edge[y+1],gly_edge[x+1]) 
			float sum = ig.at<int>(gly_edge[y] + 1, glx_edge[x] + 1) + ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x + 1] + 1) -
				ig.at<int>(gly_edge[y] + 1, glx_edge[x + 1] + 1) - ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x] + 1);
			float avg = sum / ((gly_edge[y + 1] - gly_edge[y]) * (glx_edge[x + 1] - glx_edge[x]));
			gray.at<float>(y, x) = avg;
			bins[avg * 2]++;
		}

	//2 Auto compute gray level
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
	for (int y = 1; y < gl_y.size() - 1; y++)
		for (int x = 1; x < gl_x.size() - 1; x++) {
			float m = 100000, s=100000, t0=100000;
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
					alpha[yy - y0 + 1][xx - x0 + 1] = (double) sw / ((gly_edge[yy + 1] - gly_edge[yy]) * (glx_edge[xx + 1] - glx_edge[xx]));
				}
			//Do some post process to integrate grad
			alpha[0][0] = alpha[0][0] / 1.05;
			alpha[0][2] = alpha[0][2] / 1.05;
			alpha[2][0] = alpha[2][0] / 1.05;
			alpha[2][2] = alpha[2][2] / 1.05;
			if (!mark.empty()) {
				for (int yy = 0; yy < 3; yy++)
					for (int xx = 0; xx < 3; xx++) {
					CV_Assert(alpha[yy][xx] <= 1 && alpha[yy][xx]>=0);
					unsigned char color = alpha[yy][xx] * 255;
					mark(Rect(gl_x[x] + xx * 3 - 4, gl_y[y] + yy * 3 - 4, 3, 3)) = color;
					}
			}
			//3.3 Compute grad
			Vec<float, 4> e[3][3];

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
				if (mask & (1ULL <<i)) {
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
			//3.5 Compute posteriori with BAYES method
			for (unsigned i = 0; i < brick_prob.size(); i++)
				if (mask & (1ULL << i))
					p_prob[i] = brick_prob[i] * bricks[i].priori_prob / normal; //p(grid | brick) * p(brick) / p(grid)
				else
					p_prob[i] = -1;
		}
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

	for (int i = w; i < img.size(); i++) {		
		if (mark[i] == 2) {	
			int len = 0;
			bool tail_good;
			do {
				CV_Assert(mark[i + len] == 2);
				for (; i + len < img.size() && mark[i + len] == 2; len++);
				if (i + len + w >= img.size())
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
				mark_line[y] = (unsigned char) mark.at<unsigned char>(y, x);
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
Out: stat, 
Out: dx, circle x
*/
static void feature_extract_via2_prepare(const Mat & img, int r, int r1, Mat & ia, Mat & stat, vector<int> & dx, vector<int> & dx1)
{
	CV_Assert(img.type() == CV_8UC1);
	int sz[] = { img.rows / 32 + 1, img.cols / 32 + 1, 128};
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
Out feature
*/
static void feature_extract_via2(int x0, int y0, const Mat & ia, const Mat & stat, int r, int r1, 
	const vector<int> & dx, const vector<int> & dx1, float feature[])
{
	CV_Assert(dx.size() == r + 1 && dx1.size() == r1 + 1);
	static int last_xx = -1, last_yy = -1, last_gray = 0;
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
	if (xx != last_xx || yy != last_yy) {
		for (int y = yy; y <= yy + 2; y++)
			for (int x = xx; x <= xx + 2; x++) {
			const int * p_stat = stat.ptr<int>(y, x);
			for (int i = 0; i < stat.size[2]; i++)
				sum[i] += p_stat[i];
			}
		int th = sum.back() / 10;
		for (int i = 0; i < sum.size(); i++) {
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

static double find_grid_line(const vector<double> & weight, int w, int grid, vector<int> & grid_line)
{
	CV_Assert(grid > WEIGHT_FB + 1);
	grid_line.clear();
	vector<double> weight_f[WEIGHT_FB], weight_b[WEIGHT_FB];

	for (int i = 0; i < WEIGHT_FB; i++) {
		weight_f[i].resize(weight.size());
		weight_b[i].resize(weight.size());
		for (int j = 0; j < weight.size(); j++) {
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

	int base_line;
	double max_base = -100000000;
	for (int i = grid * 2; i < weight.size() - grid * 2; i++)
		if (max_base < weight_f[WEIGHT_FB - 1][i] + weight_b[WEIGHT_FB - 1][i] - weight[i]) {
		max_base = weight_f[WEIGHT_FB - 1][i] + weight_b[WEIGHT_FB - 1][i] - weight[i];
		base_line = i;
		}
	grid_line.push_back(base_line + w / 2);
	qDebug("base=%d", base_line + w / 2);
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
	int x_align, y_align;
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
	int wire_wd = lpm.wire_wd, grid_wd = lpm.grid_wd, via_rd = lpm.via_rd;
	float via_cred = lpm.param1;
	
	CV_Assert(via_cred >= 0 && via_cred <= 1 && grid_wd > wire_wd && grid_wd > via_rd);
	if (req.find_via)
		req.find_minor_gl = true;

	Mat ig;
	integral(img, ig, CV_32S);
	//0 each point left right and up down grad
	Mat grad0(img.rows, img.cols-1, CV_32SC1);
	Mat grad1(img.rows-1, img.cols, CV_32SC1);	
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
					int max_x = (i >= (int) td.gl_x.size()) ? (int) weight.size() - 1 : td.gl_x[i] - req.dx - 1 - wire_wd / 2;
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
						gl_x.push_back((int) weight.size() - 1 + wire_wd / 2);
					else
						gl_x.pop_back();
				}					
				CV_Assert(td.gl_x.size() == gl_x.size() || req.allow_new_x);
				for (int i = 1; i < min(gl_x.size(), td.gl_x.size()) - 1; i++)
					CV_Assert(abs(td.gl_x[i] - gl_x[i + req.x_align]) <= req.dx);
				td.gl_x.swap(gl_x);
			} else //don't user reference
				find_grid_line(weight, wire_wd, grid_wd, td.gl_x);
		}
		if (!req.find_minor_gl)
			return (d0 > d1) ? 0 : 1;
		for (int x = 1; x < td.gl_x.size() -1; x++) { //2.2 compute via feature for main grid line points
			int gl = 0;
			for (int y = via_rd + 1; y < img.rows - via_rd - 1; y++) {
				if (req.gl_y_valid && y < td.gl_y[gl] - req.dy)
					continue;
				if (req.gl_y_valid && y >= td.gl_y[gl] + req.dy)
					gl++;				
				float feature[4];
				double v;
				unsigned long long c;
				feature_extract_via(img.ptr<unsigned char>(y) + td.gl_x[x], (int)img.step[0], via_rd, feature);
				v = (double)feature[0] * feature[1] * feature[2] * feature[3]; //Normally, Insu and wire = 1, and via >1
				v = max(v, 0.001);
				v = min(v, 1000.0);
				c = 0x40000000 + log(v) * 0x8000000;  //-7< log(v) <7, so c>0 & c<0x80000000, insu and wire, c=0x40000000
				c = (c << 32) | (y << 16) | td.gl_x[x];
				caps.push_back(c);
				if (req.gl_y_valid && gl >= td.gl_y.size())
					break;
			}
		}		
	} else {
		if (req.dy != 0 || !req.gl_y_valid) {
			vector<double> weight;
			for (int y = 0; y < lweight1.size() - wire_wd; y++) //weight.size() == img.rows - wire_wd-1
				weight.push_back(lweight1[y] - lweight1[y + wire_wd]);
			if (req.dy < lpm.grid_wd / 2 && req.gl_y_valid) { //use reference gl_y for compute
				vector<int> gl_y;
				for (int i = 0; i <= td.gl_y.size(); i++) {
					int min_y = (i == 0) ? 0 : td.gl_y[i - 1] + req.dy + 1 - wire_wd / 2;
					int max_y = (i >= (int) td.gl_y.size()) ? (int) weight.size() - 1 : td.gl_y[i] - req.dy - 1 - wire_wd / 2;
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
						gl_y.push_back((int) weight.size() - 1 + wire_wd / 2);
					else
						gl_y.pop_back();
				}
				CV_Assert(td.gl_y.size() == gl_y.size() || req.allow_new_y);
				for (int i = 1; i < min(td.gl_y.size(), gl_y.size()) - 1; i++)
					CV_Assert(abs(td.gl_y[i] - gl_y[i + req.y_align]) <= req.dy);				
				td.gl_y.swap(gl_y);
			} else //don't user reference
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
				float feature[4];
				double v;
				unsigned long long c;
				feature_extract_via(img.ptr<unsigned char>(td.gl_y[y]) + x, (int) img.step[0], via_rd, feature);
				v = (double)feature[0] * feature[1] * feature[2] * feature[3]; //Normally, Insu and wire = 1, and via >1
				v = max(v, 0.001);
				v = min(v, 1000.0);
				c = 0x40000000 + log(v) * 0x8000000;  //-7< log(v) <7, so c>0 & c<0x78000000, insu and wire, c=0x40000000
				c = (c << 32) | (td.gl_y[y] << 16) | x;
				caps.push_back(c);
				if (req.gl_x_valid && gl >= td.gl_x.size())
					break;
			}
		}
	}
	//3 Compute Via threshold
	sort(caps.begin(), caps.end(), greater<unsigned long long>());
	vector<unsigned> bins(2048,0);
	for (int i = 0; i < caps.size() / 6; i++)
		bins[caps[i] >> 52]++;
	float beta = 0.8f;
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
	qDebug("Via extract: wire&insu (a=%f,w=%f), via (a=%f,w=%f), Obvious via=%f, Obviouse wire insu=%f, Coarse via th=%f", 
		(th[0] - 1024) / 128, var_win[0] / 128, (th[1] - 1024) / 128, var_win[1] / 128, ((double)th1 - 0x40000000) / 0x8000000,
		 ((double)th0 - 0x40000000) / 0x8000000, ((double)th2 - 0x40000000) / 0x8000000);

	//4 Find coarse Via point based on th2
	vector<QPoint> via;
	int distance_th = (grid_wd -2) * (grid_wd-2);
	for (int i = 0; i < caps.size(); i++) {
		unsigned v = caps[i] >> 32;
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
				int max_y = (i >= (int) td.gl_y.size()) ? (int) weight.size() - 1 : td.gl_y[i] - req.dy - 1 - wire_wd / 2;
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
					gl_y.push_back((int) weight.size() - 1 + wire_wd / 2);
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
				int max_x = (i >= (int) td.gl_x.size()) ? (int) weight.size() - 1 : td.gl_x[i] - req.dx - 1 - wire_wd / 2;
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
					gl_x.push_back((int) weight.size() - 1 + wire_wd / 2);
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
	via_prob.create((int) td.gl_y.size(), (int) td.gl_x.size(), CV_32FC1);
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

static void find_via_grid_line2(struct LayerParam & lpm, struct LayerTileData & td, FindViaGridLineReq & req,
	Mat & via_prob)
{
	Mat img = td.img;
	int grid_wd = lpm.grid_wd, via_rd = lpm.via_rd, r1 = lpm.wire_wd;
	float via_cred = lpm.param1;

	CV_Assert(via_cred >= 0 && via_cred <= 1 && grid_wd > via_rd);
	CV_Assert(req.find_via && req.gl_x_valid && req.gl_y_valid && !req.allow_new_x && !req.allow_new_y);
	Mat ia, st;
	vector<int> dx, dx1;
	feature_extract_via2_prepare(img, via_rd, r1, ia, st, dx, dx1);
	//1 compute via feature for all grid line points
	vector<unsigned long long> caps;
	for (int yy = 1; yy < td.gl_y.size() - 1; yy++) 
		for (int xx = 1; xx < td.gl_x.size() - 1; xx++) {
			int y0 = td.gl_y[yy];
			int x0 = td.gl_x[xx];
			for (int y = y0 - req.dy; y <= y0 + req.dy; y++)
				for (int x = x0 - req.dx; x <= x0 + req.dx; x++) {	
					float feature[2];
					float v;
					feature_extract_via2(x, y, ia, st, via_rd, r1, dx, dx1, feature); //Normally, Insu and wire = 1, and via >1
					v = feature[0] * feature[1];
					v = max(v, 0.001f);
					v = min(v, 1000.0f);
					unsigned long long c = 0x40000000 + log(v) * 0x8000000; //-7< log(v) <7, so c>0 & c<0x78000000, insu and wire, c=0x40000000
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
		beta += 0.005;

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

#define MAKE_BRICK_ORDER(t, b) ((t) << 8 | b)
#define PROB(bo) ((bo) >> 8)
#define BRICK(bo) ((bo) & 0xff)
/*In: prob brick prob for each grid, 3 - dim Mat
		more near to 1 means bigger probability for the brick,
		more near to 0 means less probability for the brick.
  In: layer
  In: gl_x, gl_y, used for mark debug
  InOut: grid_infos, sorted brick prob order, push (prob.rows-2) * (prob.cols-2) grid_infos
  Out: mark used for debug
*/
static void post_process_grid_prob(const Mat & prob, vector<int> & gl_x, vector<int> & gl_y,
	LayerGridInfo & g, QPoint cp, Mat & mark)
{
	CV_Assert(mark.empty() || mark.type() == CV_8UC1 && mark.rows > gl_y[gl_y.size()-1] && mark.cols > gl_x[gl_x.size()-1]);
	CV_Assert(prob.type() == CV_32F && prob.dims==3 && prob.size[0] == gl_y.size() && prob.size[1] == gl_x.size());
	CV_Assert(cp.y() >= 0 && cp.x() >= 0);

	for (int y = 1; y < prob.size[0] - 1; y++)
		for (int x = 1; x < prob.size[1] - 1; x++) {
			const float * p_prob = prob.ptr<float>(y, x);
			GridInfo grid_info;
			grid_info.x = x;
			grid_info.y = y;
			for (int i = 0; i < prob.size[2]; i++)
				if (p_prob[i] >= 0) {
					int t = p_prob[i] * PROB2_INT;
					grid_info.brick_order.push_back(MAKE_BRICK_ORDER(t, i));
				}
			CV_Assert(grid_info.brick_order.size()>0);
			sort(grid_info.brick_order.begin(), grid_info.brick_order.end());
			reverse(grid_info.brick_order.begin(), grid_info.brick_order.end());			
			
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
			g.at(y - 1 + cp.y(), x - 1 + cp.x()) = grid_info;
			g.at(y - 1 + cp.y(), x - 1 + cp.x()).x = x - 1 + cp.x();
			g.at(y - 1 + cp.y(), x - 1 + cp.x()).y = y - 1 + cp.y();
		}
}

/*
In: slg
In: srect
In: d
Out: dlg
*/
static void copy_grid_info(const LayerGridInfo & slg, LayerGridInfo & dlg, const QRect srect, const QPoint d)
{
	CV_Assert(d.x() >= 1 && d.y() >= 1);
	for (int y = 1; y <= slg.grid_rows; y++)
		if (y >= srect.top() && y <= srect.bottom())
		for (int x = 1; x <= slg.grid_cols; x++)
			if (x >= srect.left() && x <= srect.right()) {
				QPoint t(x - srect.left() + d.x(), y - srect.top() + d.y());
				if (t.y() >= 1 && t.y() <= dlg.grid_rows && t.x() >= 1 && t.x() <= dlg.grid_cols) {
					dlg.at(t.y(), t.x()) = slg.at(y, x);
					dlg.at(t.y(), t.x()).x = t.x();
					dlg.at(t.y(), t.x()).y = t.y();
				}
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
	CV_Assert(prob.type() == CV_32F && prob.dims == 2 && prob.rows == gl_y.size() && prob.cols == gl_x.size());
	CV_Assert(beta >= 0.5 && beta <= 8 && cp.y() >= 0 && cp.x() >= 0);
	
	for (int y = 1; y < prob.rows - 1; y++) {
		const float * p_prob = prob.ptr<float>(y);
		for (int x = 1; x < prob.cols - 1; x++) {
			GridInfo grid_info;
			grid_info.x = x;
			grid_info.y = y;
			grid_info.brick_choose = BRICK_INVALID;
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
			g.at(y - 1 + cp.y(), x - 1 + cp.x()) = grid_info;
			g.at(y - 1 + cp.y(), x - 1 + cp.x()).x = x - 1 + cp.x();
			g.at(y - 1 + cp.y(), x - 1 + cp.x()).y = y - 1 + cp.y();
		}
	}
}


#define BRICK_PREFER(l, gi) BRICK(lg[l].grid_infos[gi].brick_order[lg[l].grid_infos[gi].po])
#define BRICK_CHOOSE(l, gi) lg[l].grid_infos[gi].brick_choose
#define ABM(l, gi) lg[l].grid_infos[gi].abm

/*
In: lbrm, rule for brick fit, 2*img_layer-1
		0,2,4 for via;
		1,3,5 for wire,
Inout: lg, in brick_order and out brick_choose, 2*img_layer-1
       0,2,4 for via; 
	   1,3,5 for wire, 
*/
static void coarse_assemble_multilayer(vector<LayerBrickRuleMask> & lbrm, vector<LayerGridInfo> & lg, const QRect & ar, int ignore_all_rule)
{
	CV_Assert(lbrm.size() == lg.size());
	int layer_num = (int)lg.size();
	static int dxy[4][2] = {
			{ -1, 0 },
			{ 0, 1 },
			{ 1, 0 },
			{ 0, -1 }
	};
	CV_Assert(layer_num % 2 == 1);

	for (int l = 0; l < layer_num; l++)
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {			
			int x = lg[l].grid_infos[gi].x;
			int y = lg[l].grid_infos[gi].y;
			if (ar.contains(x, y, true)) {
				BRICK_CHOOSE(l, gi) = BRICK_INVALID;
				for (int j = 0; j < lg[l].grid_infos[gi].brick_order.size(); j++)
					if (lg[l].grid_infos[gi].abm & 1 << BRICK(lg[l].grid_infos[gi].brick_order[j])) {
					BRICK_CHOOSE(l, gi) = BRICK(lg[l].grid_infos[gi].brick_order[j]);
					break;
					}
			}				
		}
	if (ignore_all_rule)
		return;

	for (int l = 0; l < layer_num; l++)
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
			int x = lg[l].grid_infos[gi].x;
			int y = lg[l].grid_infos[gi].y;
			if (!ar.contains(x, y, true))
				continue;
			if (l % 2 == 0) { //is via layer
				if (l > 0) {
					CV_Assert(lg[l - 1].grid_infos[gi].x == x && lg[l - 1].grid_infos[gi].y == y);
					CV_Assert(lg[l - 2].grid_infos[gi].x == x && lg[l - 2].grid_infos[gi].y == y);
					for (int v = 0; v <= 1; v++)
						if (!(lbrm[l - 1].vwvfm[v][BRICK_CHOOSE(l - 2, gi)] & 1ULL << BRICK_CHOOSE(l - 1, gi)))
							ABM(l, gi) &= ~(1ULL << v);					
				}
				if (l + 1 < layer_num) {
					CV_Assert(lg[l + 1].grid_infos[gi].x == x && lg[l + 1].grid_infos[gi].y == y);
					CV_Assert(lg[l + 2].grid_infos[gi].x == x && lg[l + 2].grid_infos[gi].y == y);
					for (int v = 0; v <= 1; v++)
						if (!(lbrm[l + 1].vwvfm[v][BRICK_CHOOSE(l + 2, gi)] & 1ULL << BRICK_CHOOSE(l + 1, gi)))
							ABM(l, gi) &= ~(1ULL << v);
				}
			} 
			else { //is wire layer
				for (int dir = 0; dir <= 3; dir++) {
					int y1 = y + dxy[dir][0];
					int x1 = x + dxy[dir][1];
					int gi1 = gi + lg[l].grid_cols * dxy[dir][0] + dxy[dir][1];
					if (y1 > 0 && y1 < lg[l].grid_rows && x1 > 0 && x1 < lg[l].grid_cols) {
						CV_Assert(lg[l].grid_infos[gi1].x == x1 && lg[l].grid_infos[gi1].y == y1);
						ABM(l, gi) &= lbrm[l].wwfm[BRICK_CHOOSE(l, gi1)][(dir + 2) & 3];
					}
				}
				CV_Assert(lg[l - 1].grid_infos[gi].x == x && lg[l - 1].grid_infos[gi].y == y && 
					lg[l + 1].grid_infos[gi].x == x && lg[l + 1].grid_infos[gi].y == y);
				ABM(l, gi) &= lbrm[l].vwvfm[BRICK_CHOOSE(l - 1, gi)][BRICK_CHOOSE(l + 1, gi)];
			}				
		}
}

/*
In: lg.brick_order
    0,2,4 for via;
	1,3,5 for wire, 
In: lbrm, (2*img_layer-1), it represents rules for neighbor wire & wire, via & wire & via
InOut: td,(img_layer) in td.gl_x, td.gl_y, out  brick shape
*/
static void assemble_grid_multilayer(vector<LayerGridInfo> & lg, vector<LayerBrickRuleMask> & lbrm,
	TileData & td, const QRect & ar, int ignore_all_rule = 1)
{
	static int dxy[4][2] = {
			{ -1, 0 },
			{ 0, 1 },
			{ 1, 0 },
			{ 0, -1 }
	};

	CV_Assert(lg.size() % 2 == 1 && lg.size()==lbrm.size());	
	
	for (int l = 0; l < lg.size(); l++) {
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
			int x = lg[l].grid_infos[gi].x;
			int y = lg[l].grid_infos[gi].y;
			if (ar.contains(x, y, true))
				lg[l].grid_infos[gi].abm = lbrm[l].fit_mask;
		}				
	}
	for (int l = 0; l < lg.size(); l++)
		if (l % 2 == 1) //is wire layer
		for (unsigned gi = 0; gi < lg[l].grid_infos.size(); gi++) {
			if (BRICK_CHOOSE(l, gi) == BRICK_INVALID)
				continue;
			int x = lg[l].grid_infos[gi].x;
			int y = lg[l].grid_infos[gi].y;			
			if (((y == ar.top() || y == ar.bottom()) && x > ar.left() && x < ar.right()) ||
				((x == ar.left() || x == ar.right()) && y > ar.top() && y < ar.bottom()))
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
	coarse_assemble_multilayer(lbrm, lg, ar, ignore_all_rule);

	for (int i = 0; i < lg.size(); i++) {
		int gi = 0;
		if (i==0 || i%2==1)
			td.d[(i + 1) / 2].conet.create(lg[i].grid_rows, lg[i].grid_cols, CV_8UC1);
		for (int y = 0; y < lg[i].grid_rows; y++) {			
			unsigned char * p_con = td.d[(i + 1) / 2].conet.ptr<unsigned char>(y);
			for (int x = 0; x < lg[i].grid_cols; x++, gi++) {
				if (BRICK_CHOOSE(i, gi) == BRICK_INVALID) {
					p_con[x] = 0;
					continue;
				}
				CV_Assert(lg[i].grid_infos[gi].y == y + 1 && lg[i].grid_infos[gi].x == x + 1);				
				if (i == 0)
					p_con[x] = BRICK_CHOOSE(i, gi) ? VIA_MASK : 0;
				else
					if (i % 2==0)
						p_con[x] |= BRICK_CHOOSE(i, gi) ? VIA_MASK : 0;
					else
						p_con[x] = bricks[BRICK_CHOOSE(i, gi)].shape;
			}				
		}
	}
}

/*
in: connet, each grid metal connection, up 1, right 2, down 4, left 8
in: gl_x, grid line x
in: gl_y, grid line y
out: obj_sets
*/
void grid2_wire_obj(const Mat & conet, int layer, const vector<int> & gl_x, const vector<int> & gl_y, vector<MarkObj> & obj_sets)
{
	CV_Assert(conet.rows + 2 == gl_y.size() && conet.cols + 2 == gl_x.size() && conet.type()==CV_8UC1);
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
		const unsigned char * p_conet = conet.ptr<unsigned char>(y);
		for (int x = 0; x < conet.cols; x++) 
			if (p_conet[x] & DIR_RIGHT_MASK) {
				if (state == 0)
					wire.p0.setX(gl_x[x + 1]);
				state = 1;
			}
			else {
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
		for (int y = 0; y < conet.rows; y++)
			if (conet.at<unsigned char>(y, x) & DIR_DOWN_MASK) {
				if (state == 0)
					wire.p0.setY(gl_y[y + 1]);
				state = 1;
			}
			else {
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
in: connet, each grid metal connection, up 1, right 2, down 4, left 8
in: gl_x, grid line x
in: gl_y, grid line y
out: obj_sets
*/
void grid2_via_obj(const Mat & conet, int layer, const vector<int> & gl_x, const vector<int> & gl_y, vector<MarkObj> & obj_sets)
{
	CV_Assert(conet.rows + 2 == gl_y.size() && conet.cols + 2 == gl_x.size() && conet.type() == CV_8UC1);
	MarkObj via;
	via.type = OBJ_POINT;
	via.type2 = POINT_VIA_AUTO_EXTRACT;
	via.type3 = layer;
	via.state = 0;	
	via.prob = 1;

	for (int y = 0; y < conet.rows; y++) {
		const unsigned char * p_conet = conet.ptr<unsigned char>(y);
		for (int x = 0; x < conet.cols; x++) {
			if (p_conet[x] & VIA_MASK) {
				via.p0 = QPoint(gl_x[x + 1], gl_y[y + 1]);
				via.p1 = via.p0;
				obj_sets.push_back(via);
			}
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
	
	if (rule & RULE_END_WITH_VIA) {
		vbvfm[0][0] &= ~(1ULL << BRICK_i_0) & ~(1ULL << BRICK_i_90) & ~(1ULL << BRICK_i_180) & ~(1ULL << BRICK_i_270);
		vbvfm[0][1] &= ~(1ULL << BRICK_NO_WIRE);
		vbvfm[1][0] &= ~(1ULL << BRICK_NO_WIRE);
	}	

	return fit_mask;
#undef ENABLE_BRICK_CONN
#undef DISABLE_BRICK_CONN	
}

void process_tile(ProcessTileData & t3)
{
	CV_Assert(t3.lbrm->size() == t3.lpm->size() * 2 - 1 && t3.tt->d.size() == t3.lpm->size());
	Mat prob;
	vector<LayerParam> & lpm = *t3.lpm;
	vector<LayerBrickRuleMask> & lbrm = *t3.lbrm;
	QPoint compute_g0;
	t3.tt->lg.resize(lbrm.size());

	if (t3.ut == NULL && t3.lt == NULL) {
		FindViaGridLineReq find_req(0, 0, false, false, false, false, true, true);
		int ud0 = find_via_grid_line(lpm[t3.up_down_layer], t3.tt->d[t3.up_down_layer],	find_req, prob);
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

	if (t3.ut == NULL && t3.lt != NULL) {
		FindViaGridLineReq find_req(0, 0, false, false, false, false, true, true);
		//copy image
		for (int l = 0; l < (int) lpm.size(); l++) {
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
		t3.tt->d[t3.left_right_layer].gl_y = t3.lt->d[t3.left_right_layer].gl_y;	
		FindViaGridLineReq find_req1(0, GRID_MAX_BIAS_PIXEL_FOR_TILE, false, false, false, true, true, true);
		int ud1 = find_via_grid_line(lpm[t3.left_right_layer], t3.tt->d[t3.left_right_layer], find_req1, prob);
		CV_Assert(ud0 == 0 && ud1 == 1);		
		if (find_req1.y_align != 0)
			qDebug("detect left_right align!=0, y_align=%d", find_req1.y_align);
		t3.tt->img_cols = t3.tt->d[0].img.cols;
		t3.tt->img_rows = t3.tt->d[0].img.rows;
		t3.tt->img_grid_x0 = t3.lt->img_grid_x0 + (int) t3.lt->d[t3.up_down_layer].gl_x.size() - 2;
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

	if (t3.ut != NULL && t3.lt == NULL) {
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
		t3.tt->d[t3.up_down_layer].gl_x = t3.ut->d[t3.up_down_layer].gl_x;	
		FindViaGridLineReq find_req0(GRID_MAX_BIAS_PIXEL_FOR_TILE, 0, false, false, true, false, true, true);
		int ud0 = find_via_grid_line(lpm[t3.up_down_layer], t3.tt->d[t3.up_down_layer], find_req0, prob);
		CV_Assert(ud0 == 0 && ud1 == 1);		
		if (find_req0.x_align != 0)
			qDebug("detect up_down align!=0, x_align=%d", find_req0.x_align);
		t3.tt->img_cols = t3.tt->d[0].img.cols;
		t3.tt->img_rows = t3.tt->d[0].img.rows;
		t3.tt->img_grid_y0 = t3.ut->img_grid_y0 + (int) t3.ut->d[t3.left_right_layer].gl_y.size() - 2;
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
		FindViaGridLineReq find_req(0, 0, false, false, false, false, true, true);
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
		int ud0 = find_via_grid_line(lpm[t3.up_down_layer], t3.tt->d[t3.up_down_layer], find_req, prob);
		int ud1 = find_via_grid_line(lpm[t3.left_right_layer], t3.tt->d[t3.left_right_layer], find_req, prob);
		CV_Assert(ud0 == 0 && ud1 == 1);		
		t3.tt->img_cols = t3.tt->d[0].img.cols;
		t3.tt->img_rows = t3.tt->d[0].img.rows;
		t3.tt->img_grid_x0 = t3.lt->img_grid_x0 + (int) t3.lt->d[0].gl_x.size() - 2;
		t3.tt->img_grid_y0 = t3.ut->img_grid_y0 + (int) t3.ut->d[0].gl_y.size() - 2;
		t3.tt->img_pixel_x0 = t3.lt->img_pixel_x0 + t3.lt->img_cols - t3.lt->d[0].mark.cols;
		t3.tt->img_pixel_y0 = t3.ut->img_pixel_y0 + t3.ut->img_rows - t3.ut->d[0].mark1.rows;		
		t3.tt->valid_grid_x0 = t3.tt->img_grid_x0 - GRID_COPY + 2;
		t3.tt->valid_grid_y0 = t3.tt->img_grid_y0 - GRID_COPY + 2;
		CV_Assert(t3.tt->d[t3.up_down_layer].gl_x[0] < lpm[t3.up_down_layer].grid_wd);
		CV_Assert(t3.tt->d[t3.left_right_layer].gl_y[0] < lpm[t3.left_right_layer].grid_wd);
		CV_Assert(abs(t3.tt->img_pixel_x0 + t3.tt->d[t3.up_down_layer].gl_x[0] - t3.lt->img_pixel_x0 -
			t3.lt->d[t3.up_down_layer].gl_x[t3.lt->d[t3.up_down_layer].gl_x.size() - 2]) < 6);//tt->gl_x[0] align with lt->gl_x[n-2]
		CV_Assert(abs(t3.tt->img_pixel_y0 + t3.tt->d[t3.left_right_layer].gl_y[0] - t3.ut->img_pixel_y0 -
			t3.ut->d[t3.left_right_layer].gl_y[t3.ut->d[t3.left_right_layer].gl_y.size() - 2]) < 6);//tt->gl_y[0] align with lt->gl_y[n-2]
		CV_Assert(abs(t3.tt->img_grid_x0 - t3.ut->img_grid_x0) <= 1 && abs(t3.tt->img_grid_y0 - t3.lt->img_grid_y0) <= 1);
		t3.tt->tile_x = t3.ut->tile_x;
		t3.tt->tile_y = t3.lt->tile_y;
		CV_Assert(t3.tt->tile_x == t3.lt->tile_x + 1 && t3.tt->tile_y == t3.ut->tile_y + 1);
		compute_g0 = QPoint(1, 1);
	}
		
	for (int l = 0; l < lpm.size(); l++) {
		FindViaGridLineReq find_req(GRID_MAX_BIAS_PIXEL_FOR_LAYER, GRID_MAX_BIAS_PIXEL_FOR_LAYER, true, true, true, true, false, false);
		if (t3.debug) {
			t3.tt->d[l].mark1.create(t3.tt->d[l].img.rows, t3.tt->d[l].img.cols, CV_8UC1);
			t3.tt->d[l].mark1 = Scalar(0);
		}
		t3.tt->d[l].mark.create(t3.tt->d[l].img.rows, t3.tt->d[l].img.cols, CV_8UC1);
		t3.tt->d[l].mark = M_UNKNOW;
		if (l != t3.up_down_layer)
			t3.tt->d[l].gl_x = t3.tt->d[t3.up_down_layer].gl_x;
		if (l!=t3.left_right_layer)
			t3.tt->d[l].gl_y = t3.tt->d[t3.left_right_layer].gl_y;
		int up_down;
		if (l==0)
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
				fill_circle(t3.tt->d[l].mark, vias[i].x(), vias[i].y(), lpm[l].via_rd + 2, M_V,
				QRect(0, 0, t3.tt->d[l].img.cols, t3.tt->d[l].img.rows));
			Mat img = t3.tt->d[l].img.clone();
			remove_via(t3.tt->d[l].mark, img, up_down, lpm[l].grid_wd - lpm[l].wire_wd);

			//compute grid prob		
			compute_grid_prob(img, lpm[l], t3.tt->d[l].gl_x,
				t3.tt->d[l].gl_y, lbrm[2 * l - 1].fit_mask, prob, t3.tt->d[l].mark1);
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

			post_process_grid_prob(prob, t3.tt->d[l].gl_x, t3.tt->d[l].gl_y,
				t3.tt->lg[2 * l - 1], pic_g0, t3.tt->d[l].mark1); //1,3, 5 for wire
		}		 
	}
	if (!t3.debug) {
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
	
	assemble_grid_multilayer(t3.tt->lg, lbrm, *(t3.tt), QRect(compute_g0, QPoint(0x70000000, 0x70000000)), 1);
	for (int l = 0; l < lpm.size(); l++) {
		t3.tt->d[l].conet = t3.tt->d[l].conet(Rect(compute_g0.x(), compute_g0.y(),
			t3.tt->d[l].conet.cols - compute_g0.x(), t3.tt->d[l].conet.rows - compute_g0.y()));
	}
}

VWExtract * VWExtract::create_extract(int)
{
	return new VWExtractStat;
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
Mat VWExtractStat::get_mark2(int ) {
	return Mat();
}
Mat VWExtractStat::get_mark3(int ) {
	return Mat();
}

int VWExtractStat::extract(string file_name, QRect, std::vector<MarkObj> & obj_sets)
{
	vector<LayerBrickRuleMask> lbrm(lpm.size() * 2 - 1);
	for (int l = 0; l < lpm.size(); l++) {
		int lbrm_idx = (l == 0) ? 0 : 2 * l - 1;
		lbrm[lbrm_idx].fit_mask = config_fit_mask(lpm[l].rule, lbrm[lbrm_idx].wwfm, lbrm[lbrm_idx].vwvfm);
		lbrm[lbrm_idx + 1].fit_mask = 3;
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
	ptd.lpm = &lpm;
	ptd.lt = NULL;
	ptd.ut = NULL;
	ptd.tt = &ts[0];
	
	process_tile(ptd);
	ts[0].lg.clear();
	obj_sets.clear();
	for (int l = 0; l < lpm.size(); l++) {
		if (l>0)
			grid2_wire_obj(ts[0].d[l].conet, l, ts[0].d[l].gl_x, ts[0].d[l].gl_y, obj_sets);
		grid2_via_obj(ts[0].d[l].conet, l, ts[0].d[l].gl_x, ts[0].d[l].gl_y, obj_sets);
		qInfo("l=%d, gl_x0=%d, gl_y0=%d", l, ts[0].d[l].gl_x[0],ts[0].d[l].gl_y[0]);
	}
	return 0;
}

int VWExtractStat::extract(vector<ICLayerWr *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets)
{
	vector<LayerBrickRuleMask> lbrm(lpm.size() * 2 - 1);
	for (int l = 0; l < lpm.size(); l++) {
		int lbrm_idx = (l == 0) ? 0 : 2 * l - 1;
		lbrm[lbrm_idx].fit_mask = config_fit_mask(lpm[l].rule, lbrm[lbrm_idx].wwfm, lbrm[lbrm_idx].vwvfm);
		lbrm[lbrm_idx + 1].fit_mask = 3;
	}
#if SAVE_RST_TO_FILE
	FILE * fp;
	fp = fopen("result.txt", "w");
#endif
	int block_x, block_y;
    ic_layer[0]->getBlockNum(block_x, block_y);
    int scale = 32768 / ic_layer[0]->getBlockWidth();
	vector <ProcessTileData> ptd_sets;
	obj_sets.clear();
	for (int area_idx = 0; area_idx < area_.size(); area_idx++) {
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
		int up_down_layer=1, left_right_layer=2;
		for (int xay = sb.left() + sb.top(); xay <= sb.right() + sb.bottom(); xay++) {
			ptd_sets.clear();
			for (int x0 = sb.left(); x0 <= sb.right(); x0++) {
				int y0 = xay - x0;
				//1 load image
				if (sb.contains(x0, y0)) {		
					//1.1 compute load image source and destination
					int wide[3] = { 0 }, height[3] = { 0 }; //wide & height is image destination bound
					
					if (x0 == sb.left()) {
						if (lx < 0) {
							wide[0] = -lx / scale;
                            wide[1] = ic_layer[0]->getBlockWidth();
						} else
                            wide[1] = ic_layer[0]->getBlockWidth() - lx / scale;
					} 
					if (x0 == sb.right()) {
						if (rx > 0) {
                            wide[1] = (wide[1] == 0) ? ic_layer[0]->getBlockWidth() : wide[1];
							wide[2] = rx / scale;
						} else
                            wide[1] = ic_layer[0]->getBlockWidth() + rx / scale;
					} 
					if (x0 != sb.left() && x0 != sb.right())
                        wide[1] = ic_layer[0]->getBlockWidth();

					if (y0 == sb.top()) {
						if (ty < 0) {
							height[0] = -ty / scale;
                            height[1] = ic_layer[0]->getBlockWidth();
						} else
                            height[1] = ic_layer[0]->getBlockWidth() - ty / scale;
					} 
					if (y0 == sb.bottom()) {
						if (by > 0) {
                            height[1] = (height[1] == 0) ? ic_layer[0]->getBlockWidth() : height[1];
							height[2] = by / scale;
						} else
                            height[1] = ic_layer[0]->getBlockWidth() + by / scale;
					}
					if (y0 != sb.top() && y0 != sb.bottom())
                        height[1] = ic_layer[0]->getBlockWidth();

					ts[(y0 - sb.top()) * sb.width() + x0 - sb.left()].d.resize(lpm.size());
					TileData * tt = &ts[(y0 - sb.top()) * sb.width() + x0 - sb.left()];
					for (int l = 0; l < lpm.size(); l++) {
						tt->d[l].img.create(height[0] + height[1] + height[2], wide[0] + wide[1] + wide[2], CV_8U);
						for (int y = y0 - 1, img_y=0; y <= y0 + 1; y++) {
							int dy = y - y0;
							for (int x = x0 - 1, img_x = 0; x <= x0 + 1; x++) {
								int dx = x - x0;
								if (wide[dx + 1] != 0 && height[dy + 1] != 0) {
									vector<uchar> encode_img;
                                    if (ic_layer[l]->getRawImgByIdx(encode_img, x, y, 0, 0) != 0) {
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
			for (int i = 0; i < ptd_sets.size(); i++)
				process_tile(ptd_sets[i]);
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
		for (int l = 0; l < lpm.size(); l++) {			
			vector<int> gl_x, gl_y;
			for (int x = sb.left(); x <= sb.right(); x++) {
				int idx = x - sb.left();
				int end = (x != sb.right()) ? ts[idx].d[l].gl_x.size() - 2 : ts[idx].d[l].gl_x.size();
				for (int i = 0; i < end; i++)
					gl_x.push_back(sr.left() / scale + ts[idx].img_pixel_x0 + ts[idx].d[l].gl_x[i]);
			}
			
			for (int y = sb.top(); y <= sb.bottom(); y++) {
				int idx = (y - sb.top()) * sb.width();
				int end = (y != sb.bottom()) ? ts[idx].d[l].gl_y.size() - 2 : ts[idx].d[l].gl_y.size();
				for (int i = 0; i < end; i++)
					gl_y.push_back(sr.top() / scale + ts[idx].img_pixel_y0 + ts[idx].d[l].gl_y[i]);
			}
			Mat conet((int) gl_y.size() - 2, (int) gl_x.size() - 2, CV_8UC1);
			conet = 0;
			Rect cr(0, 0, conet.cols, conet.rows);
			for (int i = 0; i < ts.size(); i++) {
				int x = ts[i].valid_grid_x0 - 1;
				int y = ts[i].valid_grid_y0 - 1;
				Rect sr(x, y, ts[i].d[l].conet.cols, ts[i].d[l].conet.rows);
				Rect dr = sr & cr;
				ts[i].d[l].conet(Rect(dr.x - sr.x, dr.y - sr.y, dr.width, dr.height)).copyTo(conet(dr));
			}

			if (l>0)
				grid2_wire_obj(conet, l, gl_x, gl_y, obj_sets);
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
