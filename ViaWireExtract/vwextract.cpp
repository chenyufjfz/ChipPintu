#include "vwextract.h"
#include <algorithm>
#include <functional>
#include <list>
#include <queue>  
#include <cfloat>
#include <set>
using namespace std;
#define SGN(x) (((x)>0) ? 1 : (((x)==0) ? 0 : -1))
#define WEIGHT_FB 3

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

		{ 0.3f, {
				{ 0, 1, 0 },
				{ 1, 1, 1 },
				{ 0, 1, 0 } },
				DIR_UP_MASK | DIR_DOWN_MASK | DIR_LEFT_MASK | DIR_RIGHT_MASK
		}	
};

struct GridInfo {
	int x, y;
	vector<pair<int, int>> brick_order;
	int brick_choose;
	int po; //prefer order
	int in_low_queue;
};
/*
Use kmean 2 julei
in: bins, stat histogram 
out: th, julei center
out: var_win, julei window
*/
static void cal_threshold(vector<unsigned> bins, vector<float> & th, vector<float> & var_win)
{
	unsigned total1 = 0, sep, total2, old_sep = 0xffffffff;
	double avg = 0;
	for (sep = 0; sep < bins.size(); sep++) {
		total1 += bins[sep];
		avg += bins[sep] * sep;
	}
	sep = avg / total1;

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

static void fill_circle(Mat & mark, int x0, int y0, int r, unsigned char v, QRect & rect)
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
In ig: integral image
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
static void compute_grid_prob(const Mat & img, const Mat & ig, int wire_wd, int grid_wd, float wire_cred,
	const vector<int> & gl_x, const vector<int> & gl_y, unsigned long long mask, Mat & prob, Mat & mark)
{
	CV_Assert(ig.type() == CV_32SC1 && img.type() == CV_8UC1 && img.rows + 1 == ig.rows && img.cols + 1 == ig.cols);
	CV_Assert(grid_wd > wire_wd && wire_cred <= 1 && wire_cred >= 0 && gl_x[0] >= wire_wd / 2 + 1 && gl_y[0] >= wire_wd / 2 + 1);
	CV_Assert(gl_x[gl_x.size() - 1] <= img.cols - wire_wd + wire_wd / 2 - 3);
	CV_Assert(gl_x[gl_y.size() - 1] <= img.rows - wire_wd + wire_wd / 2 - 3);
	CV_Assert(mark.empty() || mark.type() == CV_8UC1 && mark.size == img.size);

	int sz[] = { (int)gl_y.size(), (int)gl_x.size(), (int) sizeof(bricks) / sizeof(bricks[0])};
	prob.create(3, sz, CV_32F);
	prob = Scalar(0);
	qDebug("prob size0=%d, step0=%d, size1=%d, step1=%d", prob.size[0], prob.step.p[0], prob.size[1], prob.step.p[1]);

	//1 compute each grid average gray
	vector<int> glx_edge, gly_edge;
	vector<unsigned> bins(512, 0);	
	compute_grid_edge(gl_x, wire_wd, glx_edge);
	compute_grid_edge(gl_y, wire_wd, gly_edge);	
	Mat gray((int) gl_y.size() * 2 - 1, (int) gl_x.size() * 2 - 1, CV_32FC1);
#if 0
	Mat edge_gray((int)gl_y.size() * 2 - 1, (int)gl_x.size() * 2 - 1, CV_32FC4);
	vector<unsigned> grad_bins(512, 0);
#endif
	for (int y = 0; y < gray.rows; y++)
		for (int x = 0; x < gray.cols; x++) {
			//(gly_edge[y]+1, gly_edge[x]+1) (gly_edge[y+1],gly_edge[x+1]) 
			float sum = ig.at<int>(gly_edge[y] + 1, glx_edge[x] + 1) + ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x + 1] + 1) -
				ig.at<int>(gly_edge[y] + 1, glx_edge[x + 1] + 1) - ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x] + 1);
			float avg = sum / ((gly_edge[y + 1] - gly_edge[y]) * (glx_edge[x + 1] - glx_edge[x]));
			gray.at<float>(y, x) = avg;
			bins[avg * 2]++;
#if 0
			//(gly_edge[y]+1, gly_edge[x]+1) (gly_edge[y]+2,gly_edge[x+1])
			float e_up = ig.at<int>(gly_edge[y] + 1, glx_edge[x] + 1) + ig.at<int>(gly_edge[y] + 3, glx_edge[x + 1] + 1) -
				ig.at<int>(gly_edge[y] + 1, glx_edge[x + 1] + 1) - ig.at<int>(gly_edge[y] + 3, glx_edge[x] + 1);
			//(gly_edge[y+1]-1, gly_edge[x]+1) (gly_edge[y+1],gly_edge[x+1])
			float e_dn = ig.at<int>(gly_edge[y + 1] - 1, glx_edge[x] + 1) + ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x + 1] + 1) -
				ig.at<int>(gly_edge[y + 1] - 1, glx_edge[x + 1] + 1) - ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x] + 1);
			//(gly_edge[y]+1, gly_edge[x]+1) (gly_edge[y+1], gly_edge[x]+2)
			float e_lt = ig.at<int>(gly_edge[y] + 1, glx_edge[x] + 1) + ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x] + 3) -
				ig.at<int>(gly_edge[y] + 1, glx_edge[x] + 3) - ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x] + 1);
			//(gly_edge[y]+1, gly_edge[x+1]-1) (gly_edge[y+1],gly_edge[x+1]) 
			float e_rt = ig.at<int>(gly_edge[y] + 1, gly_edge[x + 1] - 1) + ig.at<int>(gly_edge[y + 1] + 1, gly_edge[x + 1] + 1) -
				ig.at<int>(gly_edge[y] + 1, gly_edge[x + 1] + 1) - ig.at<int>(gly_edge[y + 1] + 1, gly_edge[x + 1] - 1);			
			e_up = e_up / (glx_edge[x + 1] - glx_edge[x]) / 2;
			e_rt = e_rt / (gly_edge[y + 1] - gly_edge[y]) / 2;
			e_dn = e_dn / (glx_edge[x + 1] - glx_edge[x]) / 2;
			e_lt = e_lt / (gly_edge[y + 1] - gly_edge[y]) / 2;
			edge_gray.at< Vec<float, 4> >(y, x) = Vec<float, 4>(e_up, e_rt, e_dn, e_lt);
			if (y > 0)
				grad_bins[abs(e_up - edge_gray.at<Vec<float, 4> >(y - 1, x)[2]) * 2]++;
			if (x > 0)
				grad_bins[abs(e_lt - edge_gray.at<Vec<float, 4> >(y, x - 1)[1]) * 2]++;
#endif	
		}

	//2 Auto compute gray level
	vector<float> th, var_win;
	cal_threshold(bins, th, var_win);
	th[0] = th[0] / 2, th[1] = th[1] / 2;
	var_win[0] = var_win[0] / 2, var_win[1] = var_win[1] / 2;
	float wg_th = th[1] - th[0] + (var_win[0] + var_win[1]) * wire_cred;
	qDebug("Grid gray low (a=%f,w=%f), high (a=%f,w=%f), choose wire gray th=%f",
		th[0], var_win[0], th[1], var_win[1], wg_th);
#if 0
	cal_threshold(grad_bins, th, var_win);
	th[0] = th[0] / 2, th[1] = th[1] / 2;
	var_win[0] = var_win[0] / 2, var_win[1] = var_win[1] / 2;
	float grad_th = th[1] - th[0] + (var_win[0] + var_win[1]) * wire_cred;
	qDebug("Grad low (a=%f,w=%f), high (a=%f,w=%f), choose grad th=%f",
		th[0], var_win[0], th[1], var_win[1], grad_th);
#endif

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
#if 0
							t = sqrt(t * t * t);
							eng += (yy == 1 || xx == 1) ? t : t / 2; //add t/2 is floor noise adjust, if floor noise is bigger, prob is more similar
#else
							eng += t;
#endif
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
				if (i + len >= img.size())
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
		for (int y = y0; y <= y1; y++) {
			const unsigned char * b = a + lsize * y;
			for (int x = x0; x <= x1; x++) {
				s0 += b[x];
				if ((i == 0) && (y + x > y1 + x0) || (i == 2) && (y + x < y1 + x0) ||
					(i == 1) && (x - y < x0 - y0) || (i == 3) && (x - y > x0 - y0))
					s1 += b[x];
			}
		}
		feature[i] = (float)s1 / s0;
	}
	
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
/*
In img: raw image
In ig: integral image
In wire_wd: metal width
In grid_wd: grid width
In via_rd: via radius
Out via: via's x,y
Out gl_x: up-down grid line
Out gl_y; left-right grid line
return: direction
Itrequire insulator is darker than wire, if insulator is lighter than wire,do following change
   - weight.push_back(lweight0[x] - lweight0[x + wire_wd]); 
   - weight.push_back(lweight1[y] - lweight1[y + wire_wd]);
   + weight.push_back(abs(lweight0[x] - lweight0[x + wire_wd]));
   + weight.push_back(abs(lweight1[y] - lweight1[y + wire_wd]));
*/
static int find_via_grid_line(const Mat & img, const Mat & ig, int wire_wd, int grid_wd, int via_rd, float via_cred,
	vector<QPoint> & via, vector<int> & gl_x, vector<int> & gl_y)
{
	CV_Assert(ig.type() == CV_32SC1 && img.rows + 1 == ig.rows && img.cols + 1 == ig.cols && ig.rows < 65536 && ig.cols <65536);
	CV_Assert(via_cred > 0 && via_cred <= 1 && via_cred >= 0 && grid_wd > wire_wd);
	
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
	double d0 = 0, d1 = 0;
	vector<float> th, var_win;
	cal_threshold(stat, th, var_win);
	qDebug("grad low(a=%f,w=%f), grad high(a=%f,w=%f)", th[0], var_win[0], th[1], var_win[1]);
	vector<float> lweight0, lweight1;
	vector<double> weight;	
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
	double vs;
	if (d0 > d1) {
		for (int x = 0; x < lweight0.size() - wire_wd; x++) //weight.size() == img.cols - wire_wd-1
			weight.push_back(lweight0[x] - lweight0[x + wire_wd]); //Warning: need abs?
		vs = find_grid_line(weight, wire_wd, grid_wd, gl_x);
		vs = vs / ((2 * WEIGHT_FB + 1) * img.rows);		
		for (int x = 0; x < gl_x.size(); x++) {
			for (int y = via_rd + 1; y < img.rows - via_rd - 1; y++) {
				float feature[4];
				double v;
				unsigned long long c;
				feature_extract_via(img.ptr<unsigned char>(y) +gl_x[x], (int)img.step[0], via_rd, feature);
				v = (double)feature[0] * feature[1] * feature[2] * feature[3];
				c = v * 0x20000000;
				c = (c << 32) | (y << 16) | gl_x[x];
				caps.push_back(c);
			}
		}		
	} else {
		for (int y = 0; y < lweight1.size() - wire_wd; y++)
			weight.push_back(lweight1[y] - lweight1[y + wire_wd]); //Warning: need abs?
		vs = find_grid_line(weight, wire_wd, grid_wd, gl_y);
		vs = vs / ((2 * WEIGHT_FB + 1) * img.cols);
		for (int y = 0; y < gl_y.size(); y++) {
			for (int x = via_rd + 1; x < img.cols - via_rd - 1; x++) {
				float feature[4];
				double v;
				unsigned long long c;
				feature_extract_via(img.ptr<unsigned char>(gl_y[y]) + x, (int) img.step[0], via_rd, feature);
				v = (double)feature[0] * feature[1] * feature[2] * feature[3];
				c = v * 0x20000000;
				c = (c << 32) | (gl_y[y] << 16) | x;
				caps.push_back(c);
			}
		}
	}
	//3 Compute Via threshold
	sort(caps.begin(), caps.end(), greater<unsigned long long>());
	unsigned th0, th1, th2;
	th0 = caps[0] >> 32;
	th1 = caps[caps.size() / via_rd] >> 32;
	th2 = (unsigned)((0.6 * via_cred + 0.2) * th0 + (0.8 - 0.6 * via_cred) * th1);
	qDebug("via via_th high=%f, low=%f, choose th=%f", (double)th0 / 0x2000000, 
		(double)th1 / 0x2000000, (double)th2 / 0x2000000);
	vs = vs * 3 * wire_wd;
	//4 Compute Via prob based on th2
	via.clear();
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
	weight.clear();		
	if (d0 > d1) {
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
		find_grid_line(weight, wire_wd, grid_wd, gl_y);
		for (int i = 0; i < via.size(); i++) {
			bool found = false;
			for (int y = 0; y < gl_y.size(); y++) {
				if (abs(via[i].y() - gl_y[y]) <= grid_wd / 4) {
					found = true;
					break;
				}
				if (gl_y[y] > via[i].y())
					break;
			}
			if (!found) {
				qDebug("remove via(%d,%d) not in grid", via[i].x(), via[i].y());
				via.erase(via.begin() + i);
			}
		}
	}
	else {
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
		find_grid_line(weight, wire_wd, grid_wd, gl_x);
		for (int i = 0; i < via.size(); i++) {
			bool found = false;
			for (int x = 0; x < gl_x.size(); x++) {
				if (abs(via[i].x() - gl_x[x]) <= grid_wd / 4) {
					found = true;
					break;
				}
				if (gl_x[x] > via[i].x())
					break;
			}
			if (!found) {
				qDebug("remove via(%d,%d) not in grid", via[i].x(), via[i].y());
				via.erase(via.begin() + i);
			}
		}
	}

	return (d0 > d1) ? 0 : 1;
} 
/*In: prob brick prob for each grid, 3 - dim Mat
		more near to 1 means bigger probability for the brick,
		more near to 0 means less probability for the brick.
  In: gl_x, gl_y, used for mark debug
  Out: grid_infos, sorted brick prob order
  Out: mark used for debug
*/
void post_process_grid_prob(const Mat & prob, vector<int> & gl_x, vector<int> & gl_y, vector<GridInfo> & grid_infos, Mat & mark)
{
	CV_Assert(mark.empty() || mark.type() == CV_8UC1 && mark.rows > gl_y[gl_y.size()-1] && mark.cols > gl_x[gl_x.size()-1]);
	grid_infos.clear();
	for (int y = 1; y < prob.size[0] - 1; y++)
		for (int x = 1; x < prob.size[1] - 1; x++) {
			const float * p_prob = prob.ptr<float>(y, x);
			GridInfo grid_info;
			grid_info.x = x;
			grid_info.y = y;
			for (int i = 0; i < prob.size[2]; i++)
				if (p_prob[i] >= 0) {
					int t = p_prob[i] * 0xF00000;
					grid_info.brick_order.push_back(make_pair(t, i));
				}
			CV_Assert(grid_info.brick_order.size()>0);
			sort(grid_info.brick_order.begin(), grid_info.brick_order.end());
			reverse(grid_info.brick_order.begin(), grid_info.brick_order.end());			
			
			if (!mark.empty())
				for (int ord = 0; ord < 2; ord++) {
					int x1 = (ord == 0) ? (gl_x[x] + gl_x[x + 1]) / 2 : gl_x[x];
					int y1 = (ord == 0) ? gl_y[y] : (gl_y[y] + gl_y[y + 1]) / 2;
					int color = (long long)grid_info.brick_order[ord].first * 255 / 0xF00000;
					int b = grid_info.brick_order[ord].second;
					for (int yy = 0; yy < 3; yy++)
						for (int xx = 0; xx < 3; xx++)
							if (bricks[b].a[yy][xx])
								mark.at<unsigned char>(y1 - 1 + yy, x1 - 1 + xx) = color;
				}
			grid_infos.push_back(grid_info);
		}
}

/*
In: fit_mask, available brick mask for all grid
In: bfm, neighbour brick fit mask
InOut: abm, available brick mask for each grid
Inout: grid_infos, get brick prob order and store grid choose brick
*/
void coarse_assemble(unsigned long long fit_mask, unsigned long long bfm[][4], vector< vector<int> > & abm,
	vector<GridInfo> & grid_infos)
{
	//low_queue means at least two choice, high_queue means at least only 1 choice
	set<unsigned long long, greater<unsigned long long>> high_queue, low_queue;
	vector<unsigned long long> n010;
	static int dxy[4][2] = {
			{ -1, 0 },
			{ 0, 1 },
			{ 1, 0 },
			{ 0, -1 }
	};

	for (int i = 0; i < 64; i++)
		if (fit_mask & 1ULL << i)
			n010.push_back(1ULL << i);

#define DECODE_XYB(t, g, x, y, b)  do { \
			g = t & 0xffffffff; \
			x = grid_infos[g].x; \
			y = grid_infos[g].y; \
			b = (t >> 32) & 0xff; \
			} while(0)

#define ENCODE_PXYB(t, p, xy, b) do { \
			t = p << 8 | b; \
			t = t << 32 | xy; \
			} while(0)

#define CHECK_ONLY1(n, only1) do { \
			only1=false; \
			for (int i=0; i < n010.size(); i++) \
				if (n == n010[i]) \
					only1=true; \
			} while(0)

#define XY2GI(x, y) ((y-1) * ((int) abm[0].size() - 2) + x-1)

#define BRICK_PREFER(gi) grid_infos[gi].brick_order[grid_infos[gi].po].second

#define ENQUEUE2(gi, queue) do { \
			unsigned long long t; \
			ENCODE_PXYB(t, grid_infos[gi].brick_order[grid_infos[gi].po].first, gi, BRICK_PREFER(gi)); \
			bool success = queue.insert(t).second; \
			CV_Assert(success); \
			} while (0)

#define DEQUEUE2(gi, queue) do { \
			unsigned long long t; \
			ENCODE_PXYB(t, grid_infos[gi].brick_order[grid_infos[gi].po].first, gi, BRICK_PREFER(gi)); \
			int success = (int) queue.erase(t); \
			CV_Assert(success==1); \
			} while (0)

#define DEQUEUE(gi) do { \
		if (grid_infos[gi].in_low_queue==0) \
			DEQUEUE2(gi, low_queue); \
				else if (grid_infos[gi].in_low_queue==1) \
			DEQUEUE2(gi, high_queue); \
		} while (0)

#define ENQUEUE(gi) do { \
		if (grid_infos[gi].in_low_queue==0) \
			ENQUEUE2(gi, low_queue); \
				else if (grid_infos[gi].in_low_queue==1) \
			ENQUEUE2(gi, high_queue); \
			} while (0)

	for (unsigned i = 0; i < grid_infos.size(); i++) {
		grid_infos[i].brick_choose = BRICK_INVALID;
		grid_infos[i].po = 0;
		grid_infos[i].in_low_queue = 0;
		ENQUEUE(i);
	}

	bool only1_choice;
	double lost_score = 0;
	int drop_num = 0;
	while (!high_queue.empty() || !low_queue.empty()) {
		//First pick high queue brick, Then pick low queue
		set<unsigned long long, greater<unsigned long long>> &queue = high_queue.empty() ? low_queue : high_queue;
		int gi, x, y, b;
		unsigned long long t = *(queue.begin());
		queue.erase(queue.begin());
		DECODE_XYB(t, gi, x, y, b);
		CV_Assert(BRICK_PREFER(gi) == b && XY2GI(x, y) == gi);
		grid_infos[gi].brick_choose = b;
#if 1
		continue;
#endif
		lost_score += grid_infos[gi].brick_order[0].first - grid_infos[gi].brick_order[b].first;
		for (int dir = 0; dir <= 3; dir++) {
			int y1 = y + dxy[dir][0];
			int x1 = x + dxy[dir][1];
			int gi1 = XY2GI(x1, y1);
			if (y1 > 0 && y1 < abm.size() - 1 && x1>0 && x1 < abm[0].size() - 1 && grid_infos[gi1].in_low_queue < 2) {
				abm[y1][x1] &= bfm[b][dir];
				if (!(abm[y1][x1] & 1 << BRICK_PREFER(gi1))) { //Neighbour prefer not fit, Need to rechoose prefer and reenqueue
					CV_Assert(grid_infos[gi1].brick_choose == BRICK_INVALID);
					DEQUEUE(gi1);
					grid_infos[gi1].po = BRICK_INVALID;
					for (int i = 0; i < grid_infos[gi1].brick_order.size(); i++)
						if (abm[y1][x1] & 1 << grid_infos[gi1].brick_order[i].second) {
						grid_infos[gi1].po = i;
						break;
						}
					CHECK_ONLY1(abm[y1][x1], only1_choice);
					grid_infos[gi1].in_low_queue = (only1_choice) ? 1 : 0;
					if (grid_infos[gi1].po == BRICK_INVALID) {
						grid_infos[gi1].in_low_queue = 2;
						drop_num++;
					}						
					ENQUEUE(gi1);
				}
			}
		}
	}

	qDebug("drop %d grid, lost score=%f.", drop_num, lost_score / 0xF00000);

#undef ENCODE_PXYB
#undef DECODE_XYB
#undef CHECK_ONLY1
#undef XY2GI
}

/*
In: prob brick prob for each grid, 3 - dim Mat (gl_y.size() * gl_x.size() * Brick_NUM)
	more near to 1 means bigger probability for the brick,
	more near to 0 means less probability for the brick.
In: fit_mask available brick mask
In: bfm, neighbour brick fit mask
In: gl_x, gl_y, used for mark debug
Out: conet
Out: mark, used for debug
*/
void assemble_grid(const Mat & prob, unsigned long long fit_mask, unsigned long long bfm[][4],
	vector<int> & gl_x, vector<int> & gl_y, Mat & conet, Mat & mark)
{
	vector<GridInfo> grid_infos; 
	vector<unsigned long long> n010;
	vector< vector<int> > abm; //available brick mask

	post_process_grid_prob(prob, gl_x, gl_y, grid_infos, mark);

	abm.resize(prob.size[0]);
	for (int i = 0; i < abm.size(); i++)
		abm[i].assign(prob.size[1], fit_mask);

	coarse_assemble(fit_mask, bfm, abm, grid_infos);

	conet.create(prob.size[0], prob.size[1], CV_32SC1);
	conet = 0;
	for (int i = 0; i < grid_infos.size(); i++)
		if (grid_infos[i].brick_choose != BRICK_INVALID) {
			CV_Assert(fit_mask & 1ULL << grid_infos[i].brick_choose);
			conet.at<int>(grid_infos[i].y, grid_infos[i].x) = bricks[grid_infos[i].brick_choose].shape;
		}
		
}

/*
in: connet, each grid metal connection, up 1, right 2, down 4, left 8
in: gl_x, grid line x
in: gl_y, grid line y
out: obj_sets
*/
void grid2_wire_obj(const Mat & conet, const vector<int> & gl_x, const vector<int> & gl_y, vector<MarkObj> & obj_sets)
{
	CV_Assert(conet.rows == gl_y.size() && conet.cols == gl_x.size() && conet.type()==CV_32SC1);
	MarkObj wire;
	wire.type = OBJ_LINE;
	wire.type2 = 0;
	wire.state = 0;

	for (int y = 0; y < conet.rows; y++) {
		int state = 0;
		wire.p0.setY(gl_y[y]);
		wire.p1.setY(gl_y[y]);
		const int * p_conet = conet.ptr<int>(y);
		for (int x = 0; x < conet.cols; x++) 
			if (p_conet[x] & DIR_RIGHT_MASK) {
				if (state == 0) 
					wire.p0.setX(gl_x[x]);
				state = 1;
			}
			else {
				if (state == 1) {
					wire.p1.setX(gl_x[x]);
					obj_sets.push_back(wire);
				}
				state = 0;
			}		
	}
	for (int x = 0; x < conet.cols; x++) {
		int state = 0;
		wire.p0.setX(gl_x[x]);
		wire.p1.setX(gl_x[x]);
		for (int y = 0; y < conet.rows; y++)
			if (conet.at<int>(y, x) & DIR_DOWN_MASK) {
				if (state == 0)
					wire.p0.setY(gl_y[y]);
				state = 1;
			}
			else {
				if (state == 1) {
					wire.p1.setY(gl_y[y]);
					obj_sets.push_back(wire);
				}
				state = 0;
			}
	}
}

VWExtract::VWExtract()
{
	
}

unsigned long long VWExtractStat::config_fit_mask(unsigned long long rule)
{
	unsigned long long fit_mask = 0;
	bool del_xia = false, del_cao = false, del_bian = false, del_qi = false;

	for (unsigned i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++) {
		bfm[i][0] = 0;
		bfm[i][1] = 0;
		bfm[i][2] = 0;
		bfm[i][3] = 0;
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
	bfm[b0][dir] |= 1ULL << b1; \
	bfm[b1][(dir + 2) % 4] |= 1ULL << b0; } while(0)

#define DISABLE_BRICK_CONN(b0, b1, dir) do { \
	CV_Assert(bfm[b0][dir] & 1ULL << b1); \
	bfm[b0][dir] &= ~(1ULL << b1); \
	bfm[b1][(dir + 2) % 4] &= ~(1ULL << b0); } while (0)

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

	if (rule & RULE_MINIUM_3_POINT) {
		DISABLE_BRICK_CONN(BRICK_i_0, BRICK_i_180, DIR_UP);
		DISABLE_BRICK_CONN(BRICK_i_90, BRICK_i_270, DIR_RIGHT);
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
		DISABLE_BRICK_CONN(BRICK_X_0, BRICK_X_0, DIR_DOWN);
		DISABLE_BRICK_CONN(BRICK_X_0, BRICK_X_0, DIR_LEFT);
	}

	for (unsigned i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++)		
		for (unsigned j = 0; j <= i; j++) 
			for (int dir = 0; dir <= 3; dir++)  {
				int dir_1 = (dir + 2) % 4; 
				CV_Assert(!(bfm[i][dir] & 1ULL << j) == !(bfm[j][dir_1] & 1ULL << i));
			}
			
	return fit_mask;
#undef ENABLE_BRICK_CONN
#undef DISABLE_BRICK_CONN	
}

int VWExtractStat::extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets)
{
	Mat img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);
	//feature extract
	mark.create(img.rows, img.cols, CV_8UC1);
	mark1.create(img.rows, img.cols, CV_8UC1);
	mark2.create(img.rows, img.cols, CV_8UC1);
	mark3.create(img.rows, img.cols, CV_8UC1);
	mark = M_UNKNOW;
	mark1 = 0;
	mark2 = 0;
	mark3 = 0;

	unsigned long long fit_mask = config_fit_mask(rule);

	//find grid base line
	Mat ig;
	vector<QPoint> vias;
	vector<int> gl_x, gl_y;
	integral(img, ig, CV_32S);
	int up_down = find_via_grid_line(img, ig, wire_wd, grid_wd, via_rd, param1, vias, gl_x, gl_y);

	//remove via	
	MarkObj via;
	via.type = OBJ_POINT;
	via.type2 = POINT_NORMAL_VIA0;
	via.state = 0;
	
	for (int i = 0; i < vias.size(); i++) {
		via.p0 = vias[i];
		via.p1 = vias[i];
		obj_sets.push_back(via);
		fill_circle(mark, vias[i].x(), vias[i].y(), via_rd + 2, M_V, QRect(0, 0, img.cols, img.rows));
	}
	remove_via(mark, img, up_down, grid_wd - wire_wd);

	//compute grid prob
	Mat grid_prob, conet;
	integral(img, ig, CV_32S);
	compute_grid_prob(img, ig, wire_wd, grid_wd, param2, gl_x, gl_y, fit_mask, grid_prob, mark3);
	
	//transfer grid prob to wire
	assemble_grid(grid_prob, fit_mask, bfm, gl_x, gl_y, conet, mark2);
	grid2_wire_obj(conet, gl_x, gl_y, obj_sets);
		
	return 0;
}

void VWExtractStat::get_feature(int x, int y, vector<float> & feature)
{
	if (img.empty() || y<20 || y>img.rows - 20 || x<20 || x>img.cols - 20) {
		feature.clear();
		return;
	}

	Vec<float, 5> feature_vec;
	vector<int> xlimit;
	for (int yy = -via_rd; yy <= via_rd; yy++)
		xlimit.push_back((int)sqrt((float)via_rd*via_rd - yy *yy));
	feature_vec = feature_extract_5(img.ptr<unsigned char>(y) +x, (int)img.step[0],
		wire_wd + 2, wire_wd - 2, via_rd, xlimit);
	feature.resize(5);
	for (int i = 0; i < 5; i++)
		feature[i] = feature_vec[i];
}
