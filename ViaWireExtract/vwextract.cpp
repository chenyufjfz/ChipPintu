#include "vwextract.h"
#include <algorithm>
#include <functional>
#include <list>
#include <queue>  
#include <cfloat>
using namespace std;
#define SGN(x) (((x)>0) ? 1 : (((x)==0) ? 0 : -1))
#define WEIGHT_FB 3

/*
Use 2 julei
*/
void cal_threshold(vector<unsigned> bins, vector<float> & th)
{
	unsigned total1 = 0, sep, total2, old_sep = 0xffffffff;
	double avg = 0;
	for (sep = 0; sep < bins.size(); sep++) {
		total1 += bins[sep];
		avg += bins[sep] * sep;
	}
	sep = avg / total1;

	CV_Assert(sep < bins.size());
	double m1_l = 0, m1_r = 0;
	while (sep != old_sep) {
		old_sep = sep;
		m1_l = 0, m1_r = 0;
		total1 = 0, total2 = 0;
		for (unsigned j = 0; j < sep; j++) {
			m1_l += bins[j] * j;
			total1 += bins[j];
		}
		for (unsigned j = sep; j < bins.size(); j++) {
			m1_r += bins[j] * j;
			total2 += bins[j];
		}
		m1_l = m1_l / total1;
		m1_r = m1_r / total2;
		sep = (m1_l + m1_r) / 2;
	}
	th.push_back(m1_l);
	th.push_back(m1_r);
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

void fill_circle(Mat & mark, int x0, int y0, int r, unsigned char v, QRect & rect)
{
	CV_Assert(mark.type() == CV_8UC1);

	for (int y = max(rect.top(), y0 - r); y <= min(y0 + r, rect.bottom()); y++) {
		int dx = (int) sqrt((float) r*r - (y - y0) *(y - y0));
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), x0 - dx); x <= min(x0 + dx, rect.right()); x++)			
			p_mark[x] = v;
	}
}

void fill_circle_check(Mat & mark, int x0, int y0, int r, unsigned char v, QRect & rect, unsigned long long forbid_mark)
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

void fill_rect(Mat & mark, QPoint lt, QPoint rb, unsigned char v, int eu, int ed, int el, int er, QRect & rect)
{
	for (int y = max(rect.top(), lt.y() - eu); y <= min(rect.bottom(), rb.y() + ed); y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), lt.x() - el); x <= min(rect.right(), rb.x() + er); x++)
			p_mark[x] = v;
	}
}

void fill_rect_check(Mat & mark, QPoint lt, QPoint rb, unsigned char v, int eu, int ed, int el, int er, 
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
Vec<float, 5> feature_extract_5(const unsigned char * a, int lsize, int w1, int w2, int r, vector<int> dx)
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
In cwin_radius: assume within cwin_radius lighting is same
In wire_cred: wire reliablity
In: light_consist global vs local gray compare, more near to 1 means more use global, 
In gl_x: up-down grid line
In gl_y; left-right grid line
out prob: more near to 1 means more like metal, more near to 0 means more like insu
Features and Restriction
1 Doesn't care insulator is darker than wire or lighter than wire. 
2 Assume at least 5 grid wire
*/
void compute_grid_prob(const Mat & img, const Mat & ig, int wire_wd, int grid_wd, int cwin_radius, float wire_cred,
	float light_consist, const vector<int> & gl_x, const vector<int> & gl_y, Mat & mark, Mat & prob)
{
	CV_Assert(ig.type() == CV_32SC1 && img.type() == CV_8UC1 && img.rows + 1 == ig.rows && img.cols + 1 == ig.cols);
	CV_Assert(cwin_radius > 3 && cwin_radius< gl_y.size() && cwin_radius < gl_x.size() && light_consist <= 1 && light_consist >= 0);
	CV_Assert(grid_wd > wire_wd && wire_cred <= 1 && wire_cred >= 0 && gl_x[0] >= wire_wd / 2 + 1 && gl_y[0] >= wire_wd / 2 + 1);
	CV_Assert(gl_x[gl_x.size() - 1] <= img.cols - wire_wd + wire_wd / 2 - 3);
	CV_Assert(gl_x[gl_y.size() - 1] <= img.rows - wire_wd + wire_wd / 2 - 3);
	CV_Assert(!mark.data || mark.size() == img.size() && mark.type() == CV_8UC1);

	prob.create((int) gl_y.size() * 2 - 1, (int) gl_x.size() * 2 - 1, CV_32FC1);
	int dw = (grid_wd + 1) / 2;
	Mat caps_y(img.rows, (int) gl_x.size(), CV_32SC2); //center axis point set y	
	Mat caps_x((int) gl_y.size(), img.cols, CV_32SC2); //center axis point set x
	vector<int> dwire, dinsu;
	double sum_dwire = 0, sum_dinsu = 0;
	//1 Compute up down wire edge gradient
	for (int y = 0; y < img.rows; y++) {		
		if (y < dw / 2 || y >= img.rows - dw / 2 - 1) {
			for (int x = 0; x < gl_x.size(); x++) 
				caps_y.at<Vec2i>(y, x) = Vec2i(0, 0);
		} 
		else {
			int y0 = y - dw / 2;
			const int * p_ig0 = ig.ptr<int>(y0);
			const int * p_ig1 = ig.ptr<int>(y0 + dw);
			for (int x = 0; x < gl_x.size(); x++) {
				int x0 = gl_x[x] - wire_wd / 2 + 1;	//line gl_x[x] - wire_wd / 2 is insu
				//line (x0,x0+1) - (x0-2,x0-1) Right_wire-left_insu
				int e1 = p_ig1[x0 + 2] - p_ig0[x0 + 2] - 2 * (p_ig1[x0] - p_ig0[x0]) + p_ig1[x0 - 2] - p_ig0[x0 - 2];
				//int e1 = ig.at<int>(y0+dw,x0+2) - ig.at<int>(y0,x0+2) - 2 * (ig.at<int>(y0+dw,x0) -ig.at<int>(y0,x0)) + (ig.at<int>(y0+dw,x0-2) - ig.at<int>(y0,x0-2));
				x0 += wire_wd; //line gl_x[x] - wire_wd / 2 + wire_wd  is wire
				//Left_wire-right_insu
				int e2 = -(p_ig1[x0 + 2] - p_ig0[x0 + 2] - 2 * (p_ig1[x0] - p_ig0[x0]) + p_ig1[x0 - 2] - p_ig0[x0 - 2]);
				dwire.push_back(abs(e1 + e2));
				sum_dwire += abs(e1 + e2);
				caps_y.at<Vec2i>(y, x) = Vec2i(e1, e2);
			}
		}
	}
	//2 Compute left right wire edge gradient
	for (int y = 0; y < gl_y.size(); y++) {
		int y0 = gl_y[y] - wire_wd / 2 + 1; //line gl_y[y] - wire_wd / 2 is insu
		const int * p_ig0 = ig.ptr<int>(y0 - 2);
		const int * p_ig1 = ig.ptr<int>(y0);
		const int * p_ig2 = ig.ptr<int>(y0 + 2); //line (y0,y0+1) - (y0-2,y0-1)
		y0 += wire_wd;
		const int * p_ig3 = ig.ptr<int>(y0 - 2);
		const int * p_ig4 = ig.ptr<int>(y0);
		const int * p_ig5 = ig.ptr<int>(y0 + 2);
		for (int x = 0; x < dw / 2; x++)
			caps_x.at<Vec2i>(y, x) = Vec2i(0, 0);
		for (int x = img.cols - dw / 2 - 1; x < img.cols; x++)
			caps_x.at<Vec2i>(y, x) = Vec2i(0, 0);
		for (int x = dw / 2; x < img.cols - dw / 2 - 1; x++) {
			int x0 = x - dw / 2;
			int e1 = p_ig2[x0 + dw] - p_ig2[x0] - 2 * (p_ig1[x0 + dw] - p_ig1[x0]) + p_ig0[x0 + dw] - p_ig0[x0];//down_wire-up_insu
			//int e1 = ig.at<int>(y0+2,x0+dw) - ig.at<int>(y0+2,x0) - 2 * (ig.at<int>(y0,x0+dw) -ig.at<int>(y0,x0)) +	(ig.at<int>(y0-2,x0+dw) - ig.at<int>(y0-2,x0));			
			int e2 = -(p_ig5[x0 + dw] - p_ig5[x0] - 2 * (p_ig4[x0 + dw] - p_ig4[x0]) + p_ig3[x0 + dw] - p_ig3[x0]);//up_wire-down_insu
			dwire.push_back(abs(e1 + e2));
			sum_dwire += abs(e1 + e2);
			caps_x.at<Vec2i>(y, x) = Vec2i(e1, e2);			
		}
	}
	//3 compute insu gradient
	for (int y = 2; y < gl_y.size() - 1; y++) {
		int y0 = (gl_y[y - 1] + gl_y[y]) / 2;
		for (int x = 2; x < gl_x.size() - 1; x++) {
			int x0 = (gl_x[x - 1] + gl_x[x]) / 2;
			int e1 = ig.at<int>(y0 + dw, x0 + 1) - ig.at<int>(y0 - dw, x0 + 1) -
				2 * (ig.at<int>(y0 + dw, x0) - ig.at<int>(y0 - dw, x0)) +
				(ig.at<int>(y0 + dw, x0 - 1) - ig.at<int>(y0 - dw, x0 - 1));
			dinsu.push_back(abs(e1 * 2));
			int e2 = ig.at<int>(y0 + 1, x0 + dw) - ig.at<int>(y0 + 1, x0 - dw) -
				2 * (ig.at<int>(y0, x0 + dw) - ig.at<int>(y0, x0 - dw)) +
				(ig.at<int>(y0 - 1, x0 + dw) - ig.at<int>(y0 - 1, x0 - dw));
			dinsu.push_back(abs(e2 * 2));
			sum_dinsu += abs(e1 * 2) + abs(e2 * 2);
		}
	}
	if (sum_dwire / dwire.size() < sum_dinsu / dinsu.size()) {
		prob = 0;
		qCritical("Algorithm error, dwire %f < dinsu %f!", sum_dwire / dwire.size(), sum_dinsu / dinsu.size());
		return;
	}
	//4 compute threshold for wire extract, th2 is wire grad threshold, th1 is insu threshold, Assume at least 5 grid wire 
	sort(dwire.begin(), dwire.end(), greater<int>());
	sort(dinsu.begin(), dinsu.end(), greater<int>());
	int th0, th1, th2;
	th0 = dwire[5 * grid_wd];
	th1 = dinsu[dinsu.size() / 2];
	if (th0 < th1) {
		prob = 0;
		qCritical("Algorithm error, th0 %d < th1 %d!", th0, th1);
		return;
	}
	th2 = (0.4 * wire_cred + 0.3) * th0 + (0.7 - 0.4 * wire_cred) * th1;
	th1 = th1 / 2;
	qDebug("wire grad high=%d, low =%d, choose th=%d!", th0, th1, th2);

	//5 compute initial prob0 based on th1 & th2, double edge sum > th1, single edge > th0
	Mat prob0((int) gl_y.size() * 2 - 1, (int) gl_x.size() * 2 - 1, CV_32FC1); // prob0 at [-1, 1]
	prob0 = 0;
	vector<int> glx_edge, gly_edge;
	compute_grid_edge(gl_x, wire_wd, glx_edge);
	compute_grid_edge(gl_y, wire_wd, gly_edge);

	for (int x = 0; x < gl_x.size(); x++) {
		int yy = 0, count1 = 0, count2 = 0;
		for (int y = gly_edge[0]; y < img.rows; y++) {
			if (y > gly_edge[yy + 1]) {
				if (count1 != 0) {
					float delta = (float)count1 * 0.5f / count2;
					prob0.at<float>(yy, x * 2) += 2 * delta;
					if (x > 0)
						prob0.at<float>(yy, x * 2 - 1) -= delta;
					if (x < gl_x.size() - 1)
						prob0.at<float>(yy, x * 2 + 1) -= delta;
				}
				yy++;
				if (yy >= gl_y.size() * 2 - 1)
					break;
				count1 = 0;
				count2 = 0;
			}
			Vec2i e = caps_y.at<Vec2i>(y, x);
			count2++;
			if (abs(e[1]) > th1 && abs(e[0]) > th1 && abs(e[0] + e[1]) > th2) {//double edge sum > th1, single edge > th0
				count1++;
				if (mark.data) {
					mark.at<unsigned char>(y, gl_x[x] - 1) = M_W;
					mark.at<unsigned char>(y, gl_x[x] + 1) = M_W;
				}					
			}
		}
	}
	
	for (int y = 0; y < gl_y.size(); y++) {
		int xx = 0, count1 = 0, count2 = 0;
		for (int x = glx_edge[0]; x < img.cols; x++) {
			if (x > glx_edge[xx + 1]) {	
				if (count1 != 0) {
					float delta = (float)count1 * 0.5f / count2;
					prob0.at<float>(y * 2, xx) += 2 * delta;
					if (y > 0)
						prob0.at<float>(y * 2 - 1, xx) -= delta;
					if (y < gl_y.size() - 1)
						prob0.at<float>(y * 2 + 1, xx) -= delta;
				}
				xx++;
				if (xx >= gl_x.size() * 2 - 1)
					break;
				count1 = 0;
				count2 = 0;				
			}
			Vec2i e = caps_x.at<Vec2i>(y, x);
			count2++;
			if (abs(e[1]) > th1 && abs(e[0]) > th1 && abs(e[0] + e[1]) > th2) {//double edge sum > th1, single edge > th0
				count1++;
				if (mark.data) {
					mark.at<unsigned char>(gl_y[y] - 1, x) = M_W;
					mark.at<unsigned char>(gl_y[y] + 1, x) = M_W;
				}
			}
		}
	}
	//6 compute each grid average gray
	Mat gray((int) gl_y.size() * 2 - 1, (int) gl_x.size() * 2 - 1, CV_32FC1);
	for (int y = 0; y < gray.rows; y++)
		for (int x = 0; x < gray.cols; x++) {
			//(gly_edge[y]+1, gly_edge[x]+1) (gly_edge[y+1],gly_edge[x+1]) 
			float sum = ig.at<int>(gly_edge[y] + 1, glx_edge[x] + 1) + ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x + 1] + 1) -
				ig.at<int>(gly_edge[y] + 1, glx_edge[x + 1] + 1) - ig.at<int>(gly_edge[y + 1] + 1, glx_edge[x] + 1);
			gray.at<float>(y, x) = sum / ((gly_edge[y + 1] - gly_edge[y]) * (glx_edge[x + 1] - glx_edge[x]));
		}
	//7 compute local filter window
	Mat fwin(cwin_radius * 2 + 1, cwin_radius * 2 + 1, CV_32FC1);
	for (int y = 0; y < fwin.rows; y++)
		for (int x = 0; x < fwin.cols; x++)
			fwin.at<float>(y, x) = 1 / sqrt(1.0 + (y - cwin_radius) * (y - cwin_radius) + (x - cwin_radius) * (x - cwin_radius));

	Mat prob1 = prob0.clone();	
	prob = -32768;
	vector<unsigned long long> sch_odr;	
	while (1) {		
		//8 compute whole image wire and insu gray
		float sw0 = 0, sw1 = 0, si0 = 0, si1 = 0;
		float fw0 = FLT_MIN, fw1 = FLT_MIN, fi0 = FLT_MIN, fi1 = FLT_MIN;
		int cw0 = 0, cw1 = 0, ci0 = 0, ci1 = 0;
		for (int y = 0; y < prob0.rows; y++) {
			float * p_prob = prob.ptr<float>(y);
			float * p_prob0 = prob0.ptr<float>(y);
			float * p_gray = gray.ptr<float>(y);
			for (int x = 0; x < prob0.cols; x++) {
				if (p_prob[x] != -32768) {
					if (p_prob[x] > 0) {
						cw1++;
						fw1 += p_prob[x];
						sw1 += p_prob[x] * p_gray[x];
					}
					else {
						ci1++;
						fi1 += -p_prob[x];
						si1 += -p_prob[x] * p_gray[x];
					}
				}
				else {
					if (p_prob0[x] > 0) {
						cw0++;
						fw0 += p_prob0[x];
						sw0 += p_prob0[x] * p_gray[x];
					}
					else {
						ci0++;
						fi0 += -p_prob0[x];
						si0 += -p_prob0[x] * p_gray[x];
					}
				}
			}
		}
		float gray_w_global = (sw0 / fw0*cw0 + sw1 / fw1*cw1) / (cw0 + cw1);
		float gray_i_global = (si0 / fi0*ci0 + si1 / fi1*ci1) / (ci0 + ci1);
		qDebug("Whole image gray, wire=%f, insu=%f", gray_w_global, gray_i_global);
		if (abs(gray_w_global - gray_i_global) < 5) {
			qCritical("Error, whole image wire and insu are at same gray");
			prob = 0;
			return;
		}

		sch_odr.clear();	
		//9 Since computed prob will feedforward to uncomputed prob, so schedule order for probability computing, 
		for (int y = 0; y < prob0.rows; y++) {
			float * p_prob = prob.ptr<float>(y);
			for (int x = 0; x < prob0.cols; x++) {
				if (p_prob[x] == -32768) {
					int area = 0;
					float sum = 0;
					for (int y1 = max(0, y - cwin_radius); y1 <= min(y + cwin_radius, prob0.rows - 1); y1++) {
						float * p_prob0 = prob0.ptr<float>(y1);
						float * p_fwin = fwin.ptr<float>(y1 - y + cwin_radius) - x + cwin_radius;
						for (int x1 = max(0, x - cwin_radius); x1 <= min(x + cwin_radius, prob0.cols - 1); x1++) {
							area++;
							sum += p_fwin[x1] * fabs(p_prob0[x1]);
						}
					}
					if (sum > 0.000001) {
						unsigned long long s = sum * 0x2000000 / area;
						sch_odr.push_back((s << 32) | (y << 16) | x);
					}
				}
			}
		}
		if (sch_odr.empty())
			break;
		sort(sch_odr.begin(), sch_odr.end(), greater<unsigned long long>());
		//10 Compute prob based on local and global gray
		for (int i = 0; i < sch_odr.size(); i++) {
			int y = (sch_odr[i] >> 16) & 0xffff;
			int x = sch_odr[i] & 0xffff;
			CV_Assert(prob.at<float>(y, x) == -32768);
			//compute local wire and insu gray
			sw0 = 0, sw1 = 0, si0 = 0, si1 = 0;
			fw0 = FLT_MIN, fw1 = FLT_MIN, fi0 = FLT_MIN, fi1 = FLT_MIN;
			cw0 = 0, cw1 = 0, ci0 = 0, ci1 = 0;			
			for (int y1 = max(0, y - cwin_radius); y1 <= min(y + cwin_radius, prob0.rows - 1); y1++) {
				float * p_prob0 = prob0.ptr<float>(y1);
				float * p_prob = prob.ptr<float>(y1);
				float * p_fwin = fwin.ptr<float>(y1 - y + cwin_radius) - x + cwin_radius;
				float * p_gray = gray.ptr<float>(y1);
				for (int x1 = max(0, x - cwin_radius); x1 <= min(x + cwin_radius, prob0.cols - 1); x1++)
					if (p_prob[x1] != -32768) {
						if (p_prob[x1] > 0) {
							cw1++;
							float fwp = p_prob[x1] * p_fwin[x1];
							fw1 += fwp;
							sw1 += fwp * p_gray[x1];
						} else {
							ci1++;
							float fwp = -p_prob[x1] * p_fwin[x1];
							fi1 += fwp;
							si1 += fwp * p_gray[x1];
						}
					}
					else {
						if (p_prob0[x1] > 0) {
							cw0++;
							float fwp = p_prob0[x1] * p_fwin[x1];
							fw0 += fwp;
							sw0 += fwp * p_gray[x1];
						} else {
							ci0++;
							float fwp = -p_prob0[x1] * p_fwin[x1];
							fi0 += fwp;
							si0 += fwp * p_gray[x1];
						}
					}
			}			

			if (gray_w_global < gray_i_global && 
				(sw0 > 0 && sw0 / fw0 > si0 / fi0 || sw1> 0 && sw1 / fw1 > si1 / fi1))
				qCritical("Error, (%d, %d) wire is lighter than insu.", y, x);
			if (gray_w_global > gray_i_global &&
				(sw0 > 0 && sw0 / fw0 < si0 / fi0 || sw1> 0 && sw1 / fw1 < si1 / fi1))
				qCritical("Error, (%d, %d) wire is darker than insu.", y, x);
			//mix local and global gray
			float cg = (2 * cwin_radius + 1) * cwin_radius * (0.1 + 0.9 * light_consist);
			float gray_w = (sw0 / fw0*cw0 + sw1 / fw1*cw1 + gray_w_global * cg) / (cw0 + cw1 + cg);
			float gray_i = (si0 / fi0*ci0 + si1 / fi1*ci1 + gray_i_global * cg) / (ci0 + ci1 + cg);
			float sw = 0;
			if (gray_w == gray_i) {
				qCritical("Error, (%d,%d) wire and insu are at same gray", y, x);
				prob.at<float>(y, x) = 0;
				continue;
			}
			//compute prob
			if (gray_w > gray_i) {				
				for (int y1 = gly_edge[y] + 1; y1 <= gly_edge[y + 1]; y1++) {
					const unsigned char * p_img = img.ptr<unsigned char>(y1);
					for (int x1 = glx_edge[x] + 1; x1 <= glx_edge[x + 1]; x1++) {
						int gray = p_img[x1];
						if (gray >= gray_w)
							sw += 1;
						else
							if (gray > gray_i)
								sw += (float)(gray - gray_i) / (gray_w - gray_i);
					}
				}
			} else {				
				for (int y1 = gly_edge[y] + 1; y1 <= gly_edge[y + 1]; y1++) {
					const unsigned char * p_img = img.ptr<unsigned char>(y1);
					for (int x1 = glx_edge[x] + 1; x1 <= glx_edge[x + 1]; x1++) {
						int gray = p_img[x1];
						if (gray <= gray_w)
							sw += 1;
						else
							if (gray < gray_i)
								sw += (float)(gray_i - gray) / (gray_i - gray_w);
					}
				}
			}	
			float alpha = sw / ((gly_edge[y + 1] - gly_edge[y]) * (glx_edge[x + 1] - glx_edge[x]));
			prob.at<float>(y, x) = (2 * alpha - 1);
			prob1.at<float>(y, x) += (2 * alpha - 1) * 2 / grid_wd;
		}
		prob1.copyTo(prob0);
	}

	for (int y = 0; y < prob.rows; y++) {
		float * p_prob = prob.ptr<float>(y);
		for (int x = 0; x < prob.cols; x++) {
			CV_Assert(abs(p_prob[x]) <= 1);
			p_prob[x] = (p_prob[x] + 1) / 2;
		}
	}
}

/*
inout: img input: image raw data
		   output: image data which remove via
input: mark, via is 2 others is 0
input: w insulator width, used to compute gray of points which is deleted via
*/
void remove_via_line(const vector<unsigned char> & mark, vector<unsigned char> & img, int w)
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
void remove_via(const Mat & mark, Mat & img, int wire_up_down, int w)
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
void feature_extract_via(const unsigned char * a, int lsize, int r, float * feature)
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

double find_grid_line(const vector<double> & weight, int w, int grid, vector<int> & grid_line)
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
int find_via_grid_line(const Mat & img, const Mat & ig, int wire_wd, int grid_wd, int via_rd, float via_cred,
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
	vector<float> th;
	cal_threshold(stat, th);
	qDebug("grad low=%f, grad high=%f", th[0], th[1]);
	vector<float> lweight0, lweight1;
	vector<double> weight;	
	for (int x = 0; x < img.cols - 1; x++) { //lweight0.size() == img.cols-1
		float s = 0;
		for (int y = 0; y < img.rows; y++) {
			int g = grad0.at<int>(y, x);
			float t = abs(g);			
			if (t >= th[1])
				s += SGN(g);
			else
				if (t > th[0])
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

/*
in: alpha better<=0.5, grid_prob prize parameter, more big, more prize
inout: grid_prob more near to 1 means more like metal, more near to 0 means more like insu
*/
void post_process_grid_prob(float alpha, Mat & grid_prob)
{
	CV_Assert(grid_prob.type() == CV_32FC1 && grid_prob.cols % 2 == 1 && grid_prob.rows % 2 == 1 && alpha <= 0.7f);

#define PUSH_T_MAX(t, m, s) { if (m < t) { s = m; m = t; } else	if (s < t) s = t; }

    for (int y = 0; y < grid_prob.rows; y += 2) {
        float * p_grid_prob = grid_prob.ptr<float>(y);
        for (int x = 0; x < grid_prob.cols; x += 2) {
            float mmax = -10000000;
            float submax = -10000000;
            float t;

            if (x>0)
                mmax = min(p_grid_prob[x - 2], p_grid_prob[x - 1]);
            if (x + 2 < grid_prob.cols) {
                t = min(p_grid_prob[x + 2], p_grid_prob[x + 1]);
                PUSH_T_MAX(t, mmax, submax);
            }
            if (y > 0) {
                t = min(grid_prob.at<float>(y - 2, x), grid_prob.at<float>(y - 1, x));
                PUSH_T_MAX(t, mmax, submax);
            }
            if (y + 2 < grid_prob.rows) {
                t = min(grid_prob.at<float>(y + 2, x), grid_prob.at<float>(y + 1, x));
                PUSH_T_MAX(t, mmax, submax);
            }
            if (mmax + submax > 2 * p_grid_prob[x] && submax > 0.6f) //feed forward p_grid_prob[x]
                p_grid_prob[x] = p_grid_prob[x] * (1 - alpha) + alpha * (mmax + submax) / 2;

        }
    }

#undef PUSH_T_MAX
}

/*
in: grid_prob, more near to 1 means more like metal, more near to 0 means more like insu
in: rule
out: connet, each grid metal connection, up 1, right 2, down 4, left 8
*/
void assemble_grid(const Mat & grid_prob, unsigned long long rule, Mat & conet)
{
	CV_Assert(grid_prob.type() == CV_32FC1 && grid_prob.cols % 2 == 1 && grid_prob.rows % 2 == 1);
	conet.create(grid_prob.rows / 2 + 1, grid_prob.cols / 2 + 1, CV_32SC1);
	conet = 0;

	vector <unsigned long long> edge;

	for (int y = 0; y < grid_prob.rows; y++) {
		const float * p_grid_prob_1 = (y >= 1) ? grid_prob.ptr<float>(y - 1) : NULL;
		const float * p_grid_prob = grid_prob.ptr<float>(y);
		const float * p_grid_prob_a1 = (y + 1 < grid_prob.rows) ? grid_prob.ptr<float>(y + 1) : NULL;
        for (int x = (y + 1) & 1; x < grid_prob.cols; x += 2) {
			float t;
            if (y & 1) {
				t = min(p_grid_prob[x], p_grid_prob_1[x]);
				t = min(p_grid_prob_a1[x], t);
				CV_Assert(t <= 1);
				if (t > 0.5f) {
					unsigned long long ti = t * 1000000;
					ti = (ti << 32) | (y << 16) | x;
					edge.push_back(ti);
				}
			}
			else {
				t = min(p_grid_prob[x], p_grid_prob[x - 1]);
				t = min(p_grid_prob[x + 1], t);
				CV_Assert(t <= 1);
				if (t > 0.5f) {
					unsigned long long ti = t * 1000000;
					ti = (ti << 32) | (y << 16) | x;
					edge.push_back(ti);
				}					
			}
		}
	}
	sort(edge.begin(), edge.end(), greater<unsigned long long>());

	Mat belong(grid_prob.rows / 2 + 1, grid_prob.cols / 2 + 1, CV_32SC1);
	vector<int> path;
	if (rule & RULE_TREE)
		for (int y = 0; y < belong.rows; y++)
			for (int x = 0; x < belong.cols; x++)
				belong.at<int>(y, x) = (y << 16) | x;

	for (int i = 0; i < edge.size(); i++) {
		int y = (edge[i] >> 16) & 0x7fff;
		int x = edge[i] & 0x7fff;		
		int y0 = y / 2;
        int y1 = (y & 1) ? y / 2 + 1 : y / 2;
		int x0 = x / 2;
        int x1 = (x & 1) ? x / 2 + 1 : x / 2;
        int a0 = (y & 1) ? 4 : 2;
        int a1 = (y & 1) ? 1 : 8;
		int e0 = conet.at<int>(y0, x0);
		int e1 = conet.at<int>(y1, x1);

        CV_Assert((y & 1) != (x & 1) && (e0 & a0)==0 && (e1 & a1)==0);

		if ((rule & RULE_TREE) && belong.at<int>(y0, x0) == belong.at<int>(y1, x1))
			continue;
		if ((rule & RULE_NO_4CONN) && (e0 + a0 == 15 || e1 + a1 == 15))
			continue;
		if ((rule & RULE_NO_3CONN_PAIR) && (e0 == 5 && e1 == 5 || e0 == 10 && e1 == 10))
			continue;

        if (rule & RULE_TREE) {
			int t;
			if (belong.at<int>(y0, x0) < belong.at<int>(y1, x1)) {
				t = belong.at<int>(y0, x0);
				path.push_back((y1 << 16) | x1);
			}
			else {
				t = belong.at<int>(y1, x1);
				path.push_back((y0 << 16) | x0);
			}
			while (!path.empty()) {
				int yy = path.back() >> 16;
				int xx = path.back() & 0x7fff;
				int e = conet.at<int>(yy, xx);
				path.pop_back();
				belong.at<int>(yy, xx) = t;
				if ((e & 1) && belong.at<int>(yy - 1, xx) != t)
					path.push_back(((yy - 1) << 16) | xx);
				if ((e & 2) && belong.at<int>(yy, xx + 1) != t)
					path.push_back((yy << 16) | (xx + 1));
				if ((e & 4) && belong.at<int>(yy + 1, xx) != t)
					path.push_back(((yy + 1) << 16) | xx);
				if ((e & 8) && belong.at<int>(yy, xx - 1) != t)
					path.push_back((yy << 16) | (xx - 1));
			}
		}

		conet.at<int>(y0, x0) = e0 + a0;
		conet.at<int>(y1, x1) = e1 + a1;
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
			if (p_conet[x] & 2) {
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
			if (conet.at<int>(y, x) & 4) {
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
	compute_grid_prob(img, ig, wire_wd, grid_wd, iter_num, param2, param3, gl_x, gl_y, mark, grid_prob);
	post_process_grid_prob(0.5, grid_prob);

	//transfer grid prob to wire
	assemble_grid(grid_prob, RULE_TREE | RULE_NO_4CONN | RULE_NO_3CONN_PAIR, conet);
	grid2_wire_obj(conet, gl_x, gl_y, obj_sets);

	//store grid prob to mark3
	vector<int> glx_edge, gly_edge;
	compute_grid_edge(gl_x, wire_wd, glx_edge);
	compute_grid_edge(gl_y, wire_wd, gly_edge);
	for (int y = gly_edge[0], yy=0; y < mark3.rows; y+=2) {
		unsigned char * p_mark3 = mark3.ptr<unsigned char>(y);
		float * p_prob = grid_prob.ptr<float>(yy);
		if (y > gly_edge[yy + 1]) {
			yy++;
			if (yy + 1 == gly_edge.size())
				break;
		}				
		for (int x = glx_edge[0], xx=0; x < mark3.cols; x += 2) {
			if (x > glx_edge[xx + 1]) {
				xx++;
				if (xx + 1 == glx_edge.size())
					break;
			}
			p_mark3[x] = 255 * p_prob[xx];
		}
	}
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
