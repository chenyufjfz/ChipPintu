#include <QtConcurrent>
#include "vwextract.h"
#include <QDir>
#include <complex>
#define SAVE_RST_TO_FILE	1
#ifdef Q_OS_WIN
#ifdef QT_DEBUG
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif

#define WEAK_EDGE_LEN		6
#define STRONG_EDGE_WEIGHT  13
#define GOOD_EDGE_TH		95
#define GOOD_WIRE_TH		95
//CANNY_EDGE_TH1 should > CANNY_EDGE_TH2
#define CANNY_EDGE_TH1      70
#define CANNY_EDGE_TH2      50

#define MARK_EDGE			1
#define MARK_STRONG_WIRE	2
#define MARK_WEAK_WIRE		4
#define MARK_STRONG_INSU	8
#define MARK_WEAK_INSU		16
#define MARK_VIA			32
#define MARK_BORDER			128

#define MARK_210_GRID       4
#define GRID_EDGE_TH		39


typedef unsigned long long LINE_TYPE;
class DxyDegree {
public:
	int deg[8][8];
	DxyDegree()
	{
		for (int i = 0; i < 8; i++) {
			Complex<double> a(dxy[i][1], dxy[i][0]);
			for (int j = 0; j < 8; j++) {
				Complex<double> b(dxy[j][1], dxy[j][0]);
				b = b / a;
				double s = atan2(b.im, b.re) / 3.14159 * 180;
				s = s / 45;
				deg[i][j] = (s > 0) ? (int)(s + 0.1) : (int)(s - 0.1);
				if (deg[i][j] < 0)
					deg[i][j] += 8;
			}
		}
	}

	int operator()(int d0, int d1) {
		if (d0<0 || d0 > 7 || d1 < 0 || d1 > 7)
			return -1;
		return deg[d0][d1];
	}
} dxy_degree;

struct EdgeLine {
	int weight, id, weight2; //id may be len or wire id
	Point p0, p1;
	uchar d0, d1; //d0 is grad dir, d1 is edge dir, they are orthogo

	EdgeLine(int x0, int y0, int x1, int y1, int w, uchar _d0, uchar _d1) {
		p0 = Point(x0, y0);
		p1 = Point(x1, y1);
		d0 = _d0;
		d1 = _d1;
		weight2 = w;
		CV_Assert(d1 == DIR_DOWN || d1 == DIR_RIGHT || d1 == DIR_DOWNRIGHT || d1 == DIR_DOWNLEFT);
		weight = max(y1 - y0, x1 - x0);
		id = weight + 1;
		CV_Assert(weight > 0 && y1 >= y0);
	}

	Range get_range(int dir) {
		int a0, a1;
		switch (dir) {
		case DIR_UP:
		case DIR_DOWN:
			a0 = p0.x;
			a1 = p1.x;
			break;
		case DIR_LEFT:
		case DIR_RIGHT:
			a0 = p0.y;
			a1 = p1.y;
			break;
		case DIR_UPLEFT:
		case DIR_DOWNRIGHT:
			a0 = p0.y - p0.x;
			a1 = p1.y - p1.x;
			break;
		case DIR_UPRIGHT:
		case DIR_DOWNLEFT:
			a0 = p0.x + p0.y;
			a1 = p1.x + p1.y;
			break;
		default:
			CV_Assert(0);
		}
		return Range(min(a0, a1), max(a0, a1));
	}
	/*
	Input o, another edgeline
	Input dir, project dir
	inout check_wire, As input,if <0, not care, else it is my d0. As output, 1 means wire, 0 means insu
	Return negative if project interact, positive if project doesn't interact
	*/
	int project_distance(EdgeLine & o, int dir, int & check_wire) {
		Range my, your;
		int a0, a1;
		my = get_range(dir);
		your = o.get_range(dir);
		if (check_wire >= 0) {
			switch (check_wire) {
			case DIR_UP:
			case DIR_LEFT:
			case DIR_UPLEFT:
			case DIR_UPRIGHT:
				check_wire = -1;
				break;
			case DIR_DOWN:
			case DIR_RIGHT:
			case DIR_DOWNLEFT:
			case DIR_DOWNRIGHT:
				check_wire = 1;
				break;
			default:
				CV_Assert(0);
			}
			if (check_wire * (my.start - your.start) > 0)
				check_wire = 1;
			else
				check_wire = 0;
		}
		a0 = max(my.start, your.start);
		a1 = min(my.end, your.end);
		if (a0 > a1)
			return a0 - a1; //doesn't intersect
		else
			return a0 - a1 - 1;	//intersect
	}

	void cut(EdgeLine & o, int dir) {
		Range my, your;
		int a0, a1;
		my = get_range(dir);
		your = o.get_range(dir);
		if (my.start >= your.start && my.start <= your.end) {
			my.start = your.end + 1;
			switch (dir) {
			case DIR_UP:
			case DIR_DOWN:
				p0.x = my.start;
				break;
			case DIR_LEFT:
			case DIR_RIGHT:
				p0.y = my.start;
				break;
			case DIR_UPLEFT:
			case DIR_DOWNRIGHT:
				a0 = p0.x + p0.y;
				p0.y = (a0 + my.start) / 2;
				p0.x = a0 - p0.y;
				break;
			case DIR_UPRIGHT:
			case DIR_DOWNLEFT:
				a0 = p0.x - p0.y;
				p0.x = (a0 + my.start) / 2;
				p0.y = p0.x - a0;
				break;
			default:
				CV_Assert(0);
			}
		}
		if (my.end >= your.start && my.end <= your.end) {
			my.end = your.start - 1;
			switch (dir) {
			case DIR_UP:
			case DIR_DOWN:
				p1.x = my.end;
				break;
			case DIR_LEFT:
			case DIR_RIGHT:
				p1.y = my.end;
				break;
			case DIR_UPLEFT:
			case DIR_DOWNRIGHT:
				a0 = p1.x + p1.y;
				p1.y = (a0 + my.end) / 2;
				p1.x = a0 - p1.y;
				break;
			case DIR_UPRIGHT:
			case DIR_DOWNLEFT:
				a0 = p1.x - p1.y;
				p1.x = (a0 + my.end) / 2;
				p1.y = p1.x - a0;
				break;
			default:
				CV_Assert(0);
			}
		}
		if (p1.y == p0.y)
			id = p1.x - p0.x;
		else
			id = p1.y - p0.y;
		if (id > 0)
			id++;
	}

	/*
	inout o
	input wire, wire.start means minimum wire, wire.end means max sweet wire length
	input insu_min, min insu length
	*/
	void interact(EdgeLine & o, Range wire, int insu_min) {
		int s = dxy_degree(d0, o.d0);
		int bonus = 0, d, c, dd, dd1;
		int not_check_wire = -1;
		int is_wire = d0, is_wire1 = o.d0;
		switch (s) {
		case 0:
			if (project_distance(o, d1, not_check_wire) >= wire.start + insu_min) //not intersect, don't punish
				return;
			bonus = project_distance(o, d0, not_check_wire);
			if (bonus > 0) //not intersect, don't punish
				return;
			if (-bonus == id || -bonus == o.id) //inclusive
				bonus = -1000000;
			break;
		case 1:
		case 7:
			return;
			if (project_distance(o, d1, not_check_wire) >= wire.start + insu_min ||
				project_distance(o, o.d1, not_check_wire) >= wire.start + insu_min)
				return;
			dd = project_distance(o, d0, not_check_wire);
			dd1 = project_distance(o, o.d0, not_check_wire);
			bonus = min(dd, dd1);
			if (bonus > 0) //not intersect, don't punish
				return;
			break;
		case 4:
			d = project_distance(o, d1, is_wire);
			if (d > wire.end)
				return;
			bonus = project_distance(o, d0, not_check_wire);
			if (bonus > 0) //not intersect, don't do anything
				return;
			if (is_wire) {
				if (d >= wire.start) //sweet len, give bonus; else wire too narrow, punish
					bonus = -bonus;
				else
					if (-bonus == id || -bonus == o.id) //wire too narrow inclusive
						bonus = -1000000;
			}
			else {
				if (d >= insu_min) //insu wide big enough, do nothing; else insu too narrow, punish
					return;
				else
					if (-bonus == id || -bonus == o.id) //insu too narrow inclusive
						bonus = -1000000;
			}
			break;
		case 2:
		case 6:
			return;
			d = project_distance(o, d1, not_check_wire);
			c = project_distance(o, o.d1, not_check_wire);
			if (c < 0 && d < insu_min || d < 0 && c < insu_min)
				bonus = -wire.start;
			else
				return;
			break;
		case 3:
		case 5:
			return;
			d = project_distance(o, d1, is_wire);
			c = project_distance(o, o.d1, is_wire1);
			dd = project_distance(o, d0, not_check_wire);
			dd1 = project_distance(o, o.d0, not_check_wire);
			if (dd >= insu_min || dd1 >= insu_min)
				return;
			if (dd < 0 && dd1 < 0) { //project intersect				
				if (d < c) {
					d = c;
					is_wire1 = is_wire;
				} //now d is distance
				bonus = max(dd, dd1); //bonus is overlap
				if (is_wire1) {
					if (d >= wire.end) //too far away, do nothing
						return;
					if (d >= wire.start) //sweet len, give bonus; else do nothing
						bonus = min(-bonus, wire.size());
					else
						return; //how about d < insu_min
				}
				else {
					if (d >= insu_min)
						return;
					bonus = max(bonus, -insu_min);
				}
			}
			else
				return; //how about dd or dd1 < insu_min
			break;
		default:
			CV_Assert(0);
		}
		//reduce both weight and o.weight
		if (bonus > 0) {
			weight += bonus;
			o.weight += bonus;
		}
		else {
			if (id > o.id || id == o.id && weight2 >= o.weight2) {
				o.cut(*this, d0);
				o.weight = o.id - 1;
			}
			else {
				this->cut(o, d0);
				weight = id - 1;
			}
		}
	}
};

bool greaterEdgeLine(const EdgeLine & a, const EdgeLine & b) { return a.weight > b.weight; }

/*
input ic_layer
input scale, 0,1,2,3...
input rect, non-scale pixel unit
Return raw image
*/
Mat prepare_raw_img(ICLayerWrInterface * ic_layer, int scale, QRect rect)
{
	Mat img((rect.height() >> scale), (rect.width() >> scale), CV_8UC1);
	int xl = (rect.left() / ic_layer->getBlockWidth() >> scale) << scale; //xl is picture idx x
	int yt = (rect.top() / ic_layer->getBlockWidth() >> scale) << scale;
	int xr = (rect.right() / ic_layer->getBlockWidth() >> scale) << scale;
	int yb = (rect.bottom() / ic_layer->getBlockWidth() >> scale) << scale;
	bool right_hit = false, bottom_hit = false;
	for (int y = yt; y <= yb; y += 1 << scale) //both y & x is picture idx
		for (int x = xl; x <= xr; x += 1 << scale) {
			vector<uchar> encode_img;
			qDebug("load img from database scale=%d, x=%d,y=%d", scale, x, y);
			if (ic_layer->getRawImgByIdx(encode_img, x, y, scale, 0, true) != 0) {
				qCritical("load image error at s=%d, (%d,%d)", scale, x, y);
				return Mat();
			}
			Mat raw_img = imdecode(Mat(encode_img), 0);

			//now raw_img is loaded, copy it to img;
			QRect raw_img_rect(x *ic_layer->getBlockWidth(), y *ic_layer->getBlockWidth(), ic_layer->getBlockWidth() << scale, ic_layer->getBlockWidth() << scale);
			QRect overlap_rect = raw_img_rect & rect; //overlap rect is the copy rect, unit is pixel
			QRect src = overlap_rect.translated(-raw_img_rect.topLeft());
			QRect tgt = overlap_rect.translated(-rect.topLeft());
			CV_Assert((tgt.left() >> scale) + (tgt.width() >> scale) <= img.cols &&
				(tgt.top() >> scale) + (tgt.height() >> scale) <= img.rows);
			CV_Assert((src.left() >> scale) + (src.width() >> scale) <= raw_img.cols &&
				(src.top() >> scale) + (src.height() >> scale) <= raw_img.rows);
			if ((tgt.left() >> scale) + (tgt.width() >> scale) == img.cols)
				right_hit = true;
			if ((tgt.top() >> scale) + (tgt.height() >> scale) == img.rows)
				bottom_hit = true;
			if ((src.width() >> scale) == 0 || (src.height() >> scale) == 0)
				continue;
			CV_Assert(raw_img.type() == img.type());
			raw_img(Rect(src.left() >> scale, src.top() >> scale, src.width() >> scale, src.height() >> scale)).copyTo(
				img(Rect(tgt.left() >> scale, tgt.top() >> scale, tgt.width() >> scale, tgt.height() >> scale)));
		}
	CV_Assert(right_hit && bottom_hit);
	return img;
}

struct ProcessData {
	int x0, y0;
	int img_pixel_x0, img_pixel_y0; //imgae left-top pixel
	int layer;
	Mat raw_img;
	Mat * via_mark_debug; //only for debug
	Mat * edge_mark_debug; //only for debug
	Mat * edge_mark_debug1; //only for debug
	Mat * edge_mark_debug2; //only for debug
	Mat * edge_mark_debug3;
	vector<ElementObj *> eo;
	QPolygon * poly;
	int ref_cnt;
	ProcessData() {
		ref_cnt = 0;
		x0 = -10000;
		y0 = -10000;
		img_pixel_x0 = -10000;
		img_pixel_y0 = -10000;
		layer = -1;
		via_mark_debug = NULL;
		edge_mark_debug = NULL;
		edge_mark_debug1 = NULL;
		edge_mark_debug2 = NULL;
		edge_mark_debug3 = NULL;
	}
};

struct ProcessImageData {
	VWfeature * vwf;
	ProcessData * lpd;
	ProcessData * upd;
	ProcessData * cpd;
	Range wire_sweet_len;
	int insu_min;
	bool multi_thread;
};

static void prob_color(const Mat & prob, Mat & debug_mark, float th = 0.8)
{
	CV_Assert(prob.type() == CV_8UC4 && debug_mark.type() == CV_8UC3);
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		Vec3b * p_debugm = debug_mark.ptr<Vec3b>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++)
			if (p_prob[x][POINT_DIR] & 8) {//Via
				p_debugm[x][0] = 255;
				p_debugm[x][1] = 255;
				p_debugm[x][2] = 255;
			}
			else
			{
				if (p_prob[x][POINT_IS_INSU] > POINT_TOT_PROB * th)
					p_debugm[x][POINT_IS_INSU] += 20;
				else
					if (p_prob[x][POINT_IS_WIRE] > POINT_TOT_PROB * th)
						p_debugm[x][POINT_IS_WIRE] += 20;
					else
						if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] > POINT_TOT_PROB * th)
							p_debugm[x][POINT_IS_EDGE_WIRE_INSU] += 20;
			}
	}
}

static void mark_color(const Mat & mark, Mat & debug_mark)
{
	CV_Assert(mark.type() == CV_8UC1 && debug_mark.type() == CV_8UC3);
	for (int y = EDGE_JUDGE_BORDER; y < mark.rows - EDGE_JUDGE_BORDER; y++) {
		const uchar * pm = mark.ptr<uchar>(y);
		Vec3b * p_debugm = debug_mark.ptr<Vec3b>(y);
		for (int x = EDGE_JUDGE_BORDER; x < mark.cols - EDGE_JUDGE_BORDER; x++)
			if (pm[x] & MARK_VIA) {//Via
				p_debugm[x][0] = 255;
				p_debugm[x][1] = 255;
				p_debugm[x][2] = 255;
			}
			else
			{
				if (pm[x] & MARK_STRONG_INSU)
					p_debugm[x][POINT_IS_INSU] += 20;
				else
					if (pm[x] & MARK_STRONG_WIRE)
						p_debugm[x][POINT_IS_WIRE] += 20;
					else
						if (pm[x] & MARK_EDGE)
							p_debugm[x][POINT_IS_EDGE_WIRE_INSU] += 20;
			}
	}
}

static void mark_210(const Mat & mark, Mat & debug_mark)
{
	CV_Assert(mark.type() == CV_8UC1 && debug_mark.type() == CV_8UC3);
	for (int y = 0; y < mark.rows; y++) {
		const uchar * pm = mark.ptr<uchar>(y);
		Vec3b * p_debugm = debug_mark.ptr<Vec3b>(y);
		for (int x = 0; x < mark.cols; x++)
			if (pm[x] > 10) {//wrong
				p_debugm[x][0] = 255;
				p_debugm[x][1] = 255;
				p_debugm[x][2] = 255;
			}
			else
			{
				switch (pm[x]) {
				case 0:
					p_debugm[x][POINT_IS_INSU] += 20;
					break;
				case 10:
					p_debugm[x][POINT_IS_WIRE] = (p_debugm[x][POINT_IS_WIRE] < 235) ? p_debugm[x][POINT_IS_WIRE] + 20 : 255;
					break;
				case 5:
					p_debugm[x][POINT_IS_EDGE_WIRE_INSU] = (p_debugm[x][POINT_IS_EDGE_WIRE_INSU] < 235) ? p_debugm[x][POINT_IS_EDGE_WIRE_INSU] + 20 : 255;
					break;
				case 3:
				case 4:
					p_debugm[x][POINT_IS_EDGE_WIRE_INSU] = (p_debugm[x][POINT_IS_EDGE_WIRE_INSU] < 235) ? p_debugm[x][POINT_IS_EDGE_WIRE_INSU] + 20 : 255;
					p_debugm[x][POINT_IS_INSU] += 20;
					break;
				case 6:
				case 7:
					p_debugm[x][POINT_IS_EDGE_WIRE_INSU] = (p_debugm[x][POINT_IS_EDGE_WIRE_INSU] < 235) ? p_debugm[x][POINT_IS_EDGE_WIRE_INSU] + 20 : 255;
					p_debugm[x][POINT_IS_WIRE] = (p_debugm[x][POINT_IS_WIRE] < 235) ? p_debugm[x][POINT_IS_WIRE] + 20 : 255;
					break;
				}		
						
			}
	}
}
/*
Input prob
Output rst
do 1*5 filter
*/
static void filter_prob(const Mat & prob, Mat & rst)
{
	CV_Assert(prob.type() == CV_8UC4);
	int shift[8][4];
	for (int dir = 0; dir < 8; dir++) {
		int dir2 = dir_2[dir];
		int dir3 = dir_1[dir_2[dir]];
		shift[dir][0] = (dxy[dir2][0] * (int)prob.step.p[0] + dxy[dir2][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
		shift[dir][1] = (dxy[dir2][0] * 2 * (int)prob.step.p[0] + dxy[dir2][1] * 2 * (int)prob.step.p[1]) / sizeof(Vec4b);
		shift[dir][2] = (dxy[dir3][0] * (int)prob.step.p[0] + dxy[dir3][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
		shift[dir][3] = (dxy[dir3][0] * 2 * (int)prob.step.p[0] + dxy[dir3][1] * 2 * (int)prob.step.p[1]) / sizeof(Vec4b);
	}
	rst.create(prob.rows, prob.cols, prob.type());
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const Vec4b * p_prob_1 = prob.ptr<Vec4b>(y - 1);
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		const Vec4b * p_prob1 = prob.ptr<Vec4b>(y + 1);
		Vec4b * p_rst = rst.ptr<Vec4b>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			int iu = p_prob[x][POINT_IS_INSU];
			int wi = p_prob[x][POINT_IS_WIRE];
			int ed = p_prob[x][POINT_IS_EDGE_WIRE_INSU];
			int dir = p_prob[x][POINT_DIR];
			if (dir & 8) {//Via case, not change
				p_rst[x] = p_prob[x];
				continue;
			}
			/*
			bool y_in_range = (y > EDGE_JUDGE_BORDER && y < prob.rows - EDGE_JUDGE_BORDER - 1);
			bool x_in_range = (x > EDGE_JUDGE_BORDER && x < prob.cols - EDGE_JUDGE_BORDER - 1);
			int wi_num, ed_num;
			if (y_in_range && x_in_range) { //normal case
			iu += p_prob[x - 1][POINT_IS_INSU] + p_prob[x + 1][POINT_IS_INSU] +
			p_prob_1[x][POINT_IS_INSU] + p_prob_1[x - 1][POINT_IS_INSU] + p_prob_1[x + 1][POINT_IS_INSU] +
			p_prob1[x][POINT_IS_INSU] + p_prob1[x - 1][POINT_IS_INSU] + p_prob1[x + 1][POINT_IS_INSU];
			wi += p_prob[x - 1][POINT_IS_WIRE] + p_prob[x + 1][POINT_IS_WIRE] +
			p_prob_1[x][POINT_IS_WIRE] + p_prob_1[x - 1][POINT_IS_WIRE] + p_prob_1[x + 1][POINT_IS_WIRE] +
			p_prob1[x][POINT_IS_WIRE] + p_prob1[x - 1][POINT_IS_WIRE] + p_prob1[x + 1][POINT_IS_WIRE];
			wi_num = 9;
			}
			if (y_in_range && !x_in_range) { //x out of range
			iu += (x <= EDGE_JUDGE_BORDER) ? p_prob[x + 1][POINT_IS_INSU] + p_prob_1[x][POINT_IS_INSU] +
			p_prob_1[x + 1][POINT_IS_INSU] + p_prob1[x][POINT_IS_INSU] + p_prob1[x + 1][POINT_IS_INSU] :
			p_prob[x - 1][POINT_IS_INSU] + p_prob_1[x][POINT_IS_INSU] +
			p_prob_1[x - 1][POINT_IS_INSU] + p_prob1[x][POINT_IS_INSU] + p_prob1[x - 1][POINT_IS_INSU];
			wi += (x <= EDGE_JUDGE_BORDER) ? p_prob[x + 1][POINT_IS_WIRE] + p_prob_1[x][POINT_IS_WIRE] +
			p_prob_1[x + 1][POINT_IS_WIRE] + p_prob1[x][POINT_IS_WIRE] + p_prob1[x + 1][POINT_IS_WIRE] :
			p_prob[x - 1][POINT_IS_WIRE] + p_prob_1[x][POINT_IS_WIRE] +
			p_prob_1[x - 1][POINT_IS_WIRE] + p_prob1[x][POINT_IS_WIRE] + p_prob1[x - 1][POINT_IS_WIRE];
			wi_num = 6;
			}
			if (!y_in_range && x_in_range) { //y out of range
			iu += (y <= EDGE_JUDGE_BORDER) ? p_prob[x - 1][POINT_IS_INSU] + p_prob[x + 1][POINT_IS_INSU] +
			p_prob1[x][POINT_IS_INSU] + p_prob1[x - 1][POINT_IS_INSU] + p_prob1[x + 1][POINT_IS_INSU] :
			p_prob[x - 1][POINT_IS_INSU] + p_prob[x + 1][POINT_IS_INSU] +
			p_prob_1[x][POINT_IS_INSU] + p_prob_1[x - 1][POINT_IS_INSU] + p_prob_1[x + 1][POINT_IS_INSU];
			wi += (y <= EDGE_JUDGE_BORDER) ? p_prob[x - 1][POINT_IS_WIRE] + p_prob[x + 1][POINT_IS_WIRE] +
			p_prob1[x][POINT_IS_WIRE] + p_prob1[x - 1][POINT_IS_WIRE] + p_prob1[x + 1][POINT_IS_WIRE] :
			p_prob[x - 1][POINT_IS_WIRE] + p_prob[x + 1][POINT_IS_WIRE] +
			p_prob_1[x][POINT_IS_WIRE] + p_prob_1[x - 1][POINT_IS_WIRE] + p_prob_1[x + 1][POINT_IS_WIRE];
			wi_num = 6;
			}
			if (!y_in_range && !x_in_range) {
			iu += (y <= EDGE_JUDGE_BORDER) ? ((x <= EDGE_JUDGE_BORDER) ?
			p_prob[x + 1][POINT_IS_INSU] + p_prob1[x][POINT_IS_INSU] + p_prob1[x + 1][POINT_IS_INSU] :
			p_prob[x - 1][POINT_IS_INSU] + p_prob1[x][POINT_IS_INSU] + p_prob1[x - 1][POINT_IS_INSU])
			: ((x <= EDGE_JUDGE_BORDER) ?
			p_prob[x + 1][POINT_IS_INSU] + p_prob_1[x][POINT_IS_INSU] + p_prob_1[x + 1][POINT_IS_INSU] :
			p_prob[x - 1][POINT_IS_INSU] + p_prob_1[x][POINT_IS_INSU] + p_prob_1[x - 1][POINT_IS_INSU]);
			wi += (y <= EDGE_JUDGE_BORDER) ? ((x <= EDGE_JUDGE_BORDER) ?
			p_prob[x + 1][POINT_IS_WIRE] + p_prob1[x][POINT_IS_WIRE] + p_prob1[x + 1][POINT_IS_WIRE] :
			p_prob[x - 1][POINT_IS_WIRE] + p_prob1[x][POINT_IS_WIRE] + p_prob1[x - 1][POINT_IS_WIRE])
			: ((x <= EDGE_JUDGE_BORDER) ?
			p_prob[x + 1][POINT_IS_WIRE] + p_prob_1[x][POINT_IS_WIRE] + p_prob_1[x + 1][POINT_IS_WIRE] :
			p_prob[x - 1][POINT_IS_WIRE] + p_prob_1[x][POINT_IS_WIRE] + p_prob_1[x - 1][POINT_IS_WIRE]);
			wi_num = 4;
			}*/
			if (y > EDGE_JUDGE_BORDER + 1 && y < prob.rows - EDGE_JUDGE_BORDER - 2 &&
				x > EDGE_JUDGE_BORDER + 1 && x < prob.cols - EDGE_JUDGE_BORDER - 2) {
				ed += (p_prob[x + shift[dir][0]][POINT_DIR] == dir) ? p_prob[x + shift[dir][0]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][0]][POINT_IS_EDGE_WIRE_INSU] / 2;
				ed += (p_prob[x + shift[dir][1]][POINT_DIR] == dir) ? p_prob[x + shift[dir][1]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][1]][POINT_IS_EDGE_WIRE_INSU] / 2;
				ed += (p_prob[x + shift[dir][2]][POINT_DIR] == dir) ? p_prob[x + shift[dir][2]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][2]][POINT_IS_EDGE_WIRE_INSU] / 2;
				ed += (p_prob[x + shift[dir][3]][POINT_DIR] == dir) ? p_prob[x + shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] / 2;
				iu += p_prob[x + shift[dir][0]][POINT_IS_INSU] + p_prob[x + shift[dir][1]][POINT_IS_INSU] +
					p_prob[x + shift[dir][2]][POINT_IS_INSU] + p_prob[x + shift[dir][3]][POINT_IS_INSU];
				wi += p_prob[x + shift[dir][0]][POINT_IS_WIRE] + p_prob[x + shift[dir][1]][POINT_IS_WIRE] +
					p_prob[x + shift[dir][2]][POINT_IS_WIRE] + p_prob[x + shift[dir][3]][POINT_IS_WIRE];

			}
			else {
				int dir2 = dir_2[dir];
				int dir3 = dir_1[dir_2[dir]];
#define IN_RANGE(x, y) (x >= EDGE_JUDGE_BORDER && x < prob.cols - EDGE_JUDGE_BORDER && y >=EDGE_JUDGE_BORDER && y < prob.rows - EDGE_JUDGE_BORDER)
				if (IN_RANGE(x + dxy[dir2][1], y + dxy[dir2][0])) {
					ed += (p_prob[x + shift[dir][0]][POINT_DIR] == dir) ? p_prob[x + shift[dir][0]][POINT_IS_EDGE_WIRE_INSU] :
						p_prob[x + shift[dir][0]][POINT_IS_EDGE_WIRE_INSU] / 2;
					iu += p_prob[x + shift[dir][0]][POINT_IS_INSU];
					wi += p_prob[x + shift[dir][0]][POINT_IS_WIRE];
				}
				if (IN_RANGE(x + dxy[dir2][1] * 2, y + dxy[dir2][0] * 2)) {
					ed += (p_prob[x + shift[dir][1]][POINT_DIR] == dir) ? p_prob[x + shift[dir][1]][POINT_IS_EDGE_WIRE_INSU] :
						p_prob[x + shift[dir][1]][POINT_IS_EDGE_WIRE_INSU] / 2;
					iu += p_prob[x + shift[dir][1]][POINT_IS_INSU];
					wi += p_prob[x + shift[dir][1]][POINT_IS_WIRE];
				}
				if (IN_RANGE(x + dxy[dir3][1], y + dxy[dir3][0])) {
					ed += (p_prob[x + shift[dir][2]][POINT_DIR] == dir) ? p_prob[x + shift[dir][2]][POINT_IS_EDGE_WIRE_INSU] :
						p_prob[x + shift[dir][2]][POINT_IS_EDGE_WIRE_INSU] / 2;
					iu += p_prob[x + shift[dir][2]][POINT_IS_INSU];
					wi += p_prob[x + shift[dir][2]][POINT_IS_WIRE];
				}
				if (IN_RANGE(x + dxy[dir3][1] * 2, y + dxy[dir3][0] * 2)) {
					ed += (p_prob[x + shift[dir][3]][POINT_DIR] == dir) ? p_prob[x + shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] :
						p_prob[x + shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] / 2;
					iu += p_prob[x + shift[dir][3]][POINT_IS_INSU];
					wi += p_prob[x + shift[dir][3]][POINT_IS_WIRE];
				}
#undef IN_RANGE
			}
			p_rst[x][POINT_IS_INSU] = iu * POINT_TOT_PROB / (iu + wi + ed);
			p_rst[x][POINT_IS_WIRE] = wi * POINT_TOT_PROB / (iu + wi + ed);
			p_rst[x][POINT_IS_EDGE_WIRE_INSU] = POINT_TOT_PROB - p_rst[x][POINT_IS_WIRE] - p_rst[x][POINT_IS_INSU];
			p_rst[x][POINT_DIR] = dir;
		}
	}
}

/*inout prob
input sweet_len, sweet_len range
input insu_min, min insu distance
output edges,
output mark
weak edge length must >=WEAK_EDGE_LEN
strong edge length either >= STRONG_EDGE_WEIGHT or (length >= STRONG_EDGE_WEIGHT & it has opposite edge)
if -+ +-edge distance < wire_sweet_len.end and > wire_swee_len.start, make wire band
if +- -=edge distance < insu_min * 2 + wire_sweet_len.start, make insu band
output mark2, dir and edge factor
*/
static void mark_strong_edge(Mat & prob, Range wire_sweet_len, int insu_min, vector<EdgeLine> & edges, Mat & mark, Mat & mark2)
{
	CV_Assert(prob.type() == CV_8UC4);
	int shift[8][5];//shift[dir][0] shift[4] is edge dir, shift[1] is insu dir, shift[2] is wire dir, shift[3] is grad dir
	for (int dir = 0; dir < 8; dir++) {
		int dir2 = dir_2[dir];
		int dir3 = dir_1[dir];
		shift[dir][0] = (dxy[dir2][0] * (int)prob.step.p[0] + dxy[dir2][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
		if (shift[dir][0] < 0)
			shift[dir][4] = dir2;
		else
			shift[dir][4] = dir_1[dir2];

		shift[dir][0] = -abs(shift[dir][0]);
		shift[dir][1] = (dxy[dir][0] * (int)prob.step.p[0] + dxy[dir][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
		shift[dir][2] = (dxy[dir3][0] * (int)prob.step.p[0] + dxy[dir3][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
		shift[dir][3] = (dxy[dir][0] * (int)prob.step.p[0] + dxy[dir][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	}
	shift[DIR_UPRIGHT][1] = (dxy[DIR_RIGHT][0] * (int)prob.step.p[0] + dxy[DIR_RIGHT][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_UPRIGHT][2] = (dxy[DIR_DOWN][0] * (int)prob.step.p[0] + dxy[DIR_DOWN][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_DOWNRIGHT][1] = (dxy[DIR_RIGHT][0] * (int)prob.step.p[0] + dxy[DIR_RIGHT][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_DOWNRIGHT][2] = (dxy[DIR_UP][0] * (int)prob.step.p[0] + dxy[DIR_UP][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_DOWNLEFT][1] = (dxy[DIR_LEFT][0] * (int)prob.step.p[0] + dxy[DIR_LEFT][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_DOWNLEFT][2] = (dxy[DIR_UP][0] * (int)prob.step.p[0] + dxy[DIR_UP][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_UPLEFT][1] = (dxy[DIR_LEFT][0] * (int)prob.step.p[0] + dxy[DIR_LEFT][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_UPLEFT][2] = (dxy[DIR_DOWN][0] * (int)prob.step.p[0] + dxy[DIR_DOWN][1] * (int)prob.step.p[1]) / sizeof(Vec4b);

	int th1 = POINT_TOT_PROB * CANNY_EDGE_TH1 / 100;
	int th2 = POINT_TOT_PROB * CANNY_EDGE_TH2 / 100;
	mark2.create(prob.rows, prob.cols, CV_8U);
	mark2 = 8;
	//1 find local maximum point 
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		uchar * pm = mark2.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] < th2 || p_prob[x][POINT_DIR] & 8) //edge factor is low or via case
				continue; //edge factor is 8 if < th2
			else {
				pm[x] = 16 | p_prob[x][POINT_DIR]; //default edge factor is 16, > th2
				if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] == POINT_TOT_PROB) //edge factor is 128 if max strong
					pm[x] = 128 | p_prob[x][POINT_DIR];
				else {
					int dir = p_prob[x][POINT_DIR] & 7;
					const Vec4b * p1 = p_prob + x + shift[dir][3];
					uchar edge_grad = p_prob[x][POINT_IS_EDGE_WIRE_INSU];
					if (p1[0][POINT_IS_EDGE_WIRE_INSU] > edge_grad ||
						p1[shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] > edge_grad) //local compare cut
						continue;
					p1 = p_prob + x - shift[dir][3];
					if (p1[0][POINT_IS_EDGE_WIRE_INSU] <= edge_grad &&
						p1[-shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] <= edge_grad) {//local compare cut
						pm[x] = (p_prob[x][POINT_IS_EDGE_WIRE_INSU] > th1) ? 64 : 32;//edge factor is strong, 64, else 32							 
						pm[x] |= p_prob[x][POINT_DIR];
					}
				}
			}
		}
	}

	//2 local maximum point make up edge line
	vector<EdgeLine> edges_dir[8];
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const uchar * pm = mark2.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++)
			if (pm[x] & 0xc0) { //edge factor > CANNY_EDGE_TH1
				int dir = pm[x] & 7;
				int s = shift[dir][0];
				if ((pm[x + s] & 0x1f) == dir && pm[x + s] & 0xc0) //edge line already found
					continue;
				int l = 0;
				int w = 0;
				bool already_found = false;
				while ((pm[x + s] & 0x1f) == dir ||
					(pm[x + s + shift[dir][1]] & 0x1f) == dir ||
					(pm[x + s + shift[dir][2]] & 0x1f) == dir) {
					if ((pm[x + s] & 7) != dir)
						break;
					if (pm[x + s] & 0x80)
						w += 100;
					else
						if (pm[x + s] & 0x40)
							w += 80;
						else
							if (pm[x + s] & 0x20)
								w += 60;
							else
								if (pm[x + s] & 0x10)
									w += 30;
					s += shift[dir][0];
					l++;
					if ((pm[x + s] & 0x1f) == dir && pm[x + s] & 0xc0) {
						already_found = true;
						break;
					}
				}
				if (already_found)
					continue;
				s = -shift[dir][0];
				int r = 0;
				while ((pm[x + s] & 0x1f) == dir ||
					(pm[x + s + shift[dir][1]] & 0x1f) == dir ||
					(pm[x + s + shift[dir][2]] & 0x1f) == dir) {
					if ((pm[x + s] & 7) != dir)
						break;
					if (pm[x + s] & 0x80)
						w += 100;
					else
						if (pm[x + s] & 0x40)
							w += 80;
						else
							if (pm[x + s] & 0x20)
								w += 60;
							else
								if (pm[x + s] & 0x10)
									w += 30;
					s -= shift[dir][0];
					r++;
				}
				if (l + r + 1 < WEAK_EDGE_LEN) //weak edge line, not added
					continue;
				int x0 = dxy[shift[dir][4]][1] * l + x;
				int y0 = dxy[shift[dir][4]][0] * l + y;
				int x1 = -dxy[shift[dir][4]][1] * r + x;
				int y1 = -dxy[shift[dir][4]][0] * r + y;
				CV_Assert(x0 > 0 && y0 > 0 && x1 > 0 && y1 > 0 && x0 + 1 < prob.cols && y0 + 1 < prob.rows && x1 + 1 < prob.cols && y1 + 1 < prob.rows);
				edges_dir[dir].push_back(EdgeLine(x0, y0, x1, y1, w / (l + r), dir, dir_1[shift[dir][4]]));
			}
	}

	//3 sort
	int total_len = 0;
	for (int dir = 0; dir < 8; dir++)
		for (int i = 0; i < edges_dir[dir].size(); i++) {
			for (int j = i + 1; j < edges_dir[dir].size(); j++) {
				if (edges_dir[dir][i].weight < 0)
					break;
				if (edges_dir[dir][j].weight < 0)
					continue;
				edges_dir[dir][i].interact(edges_dir[dir][j], wire_sweet_len, insu_min);
				if (edges_dir[dir][i].weight >= 0)
				CV_Assert(edges_dir[dir][i].p0.x > 0 && edges_dir[dir][i].p0.y > 0 && edges_dir[dir][i].p1.x > 0 && edges_dir[dir][i].p1.y > 0
					&& edges_dir[dir][i].p0.x < prob.cols && edges_dir[dir][i].p0.y < prob.rows && edges_dir[dir][i].p1.x < prob.cols && edges_dir[dir][i].p1.y < prob.rows);
				if (edges_dir[dir][j].weight >= 0)
				CV_Assert(edges_dir[dir][j].p0.x > 0 && edges_dir[dir][j].p0.y > 0 && edges_dir[dir][j].p1.x > 0 && edges_dir[dir][j].p1.y > 0
					&& edges_dir[dir][j].p0.x < prob.cols && edges_dir[dir][j].p0.y < prob.rows && edges_dir[dir][j].p1.x < prob.cols && edges_dir[dir][j].p1.y < prob.rows);
			}
			if (edges_dir[dir][i].weight > 0) {
				edges.push_back(edges_dir[dir][i]);
				total_len += edges.back().id;
			}
		}
	/*
	for (int dir = 0; dir < 8; dir++)
		for (int i = 0; i < edges_dir[dir].size(); i++) {
			if (edges_dir[dir][i].weight < 0)
				continue;
			for (int j = 0; j < edges_dir[dir_1[dir]].size(); j++) {
				if (edges_dir[dir][i].weight < 0)
					break;
				if (edges_dir[dir_1[dir]][j].weight < 0)
					continue;
				edges_dir[dir][i].interact(edges_dir[dir_1[dir]][j], wire_sweet_len, insu_min);
			}
			if (edges_dir[dir][i].weight > 0) {
				edges.push_back(edges_dir[dir][i]);
				total_len += edges.back().id;
			}
		}*/
	qInfo("Edge num =%d, len=%d", (int)edges.size(), total_len);

	//4 draw strong edge in draft
	Mat m1(prob.rows, prob.cols, CV_8U);
	m1 = 0;
	cv::rectangle(m1, Point(0, 0), Point(EDGE_JUDGE_BORDER - 1, m1.rows - 1), Scalar::all(MARK_BORDER), CV_FILLED);
	cv::rectangle(m1, Point(0, 0), Point(m1.cols - 1, EDGE_JUDGE_BORDER - 1), Scalar::all(MARK_BORDER), CV_FILLED);
	cv::rectangle(m1, Point(m1.cols - EDGE_JUDGE_BORDER, 0), Point(m1.cols - 1, m1.rows - 1), Scalar::all(MARK_BORDER), CV_FILLED);
	cv::rectangle(m1, Point(0, m1.rows - EDGE_JUDGE_BORDER), Point(m1.cols - 1, m1.rows - 1), Scalar::all(MARK_BORDER), CV_FILLED);
	for (auto & e : edges) {
		if (e.weight < STRONG_EDGE_WEIGHT)
			continue;
		vector<Point> pts;
		get_line_pts(e.p0, e.p1, pts);
		pts.push_back(e.p1);
		int dir = e.d0;
		for (auto & pt : pts) {
			uchar * pm1 = m1.ptr<uchar>(pt.y, pt.x);
			pm1[0] |= MARK_EDGE; //mark edge
			for (int i = 1, j = shift[dir][1]; i <= insu_min; i++, j += shift[dir][1]) {
				if (pm1[j] & MARK_BORDER)
					break;
				pm1[j] |= MARK_WEAK_INSU; //weak insu
			}
			for (int i = 1, j = shift[dir][2]; i <= wire_sweet_len.start; i++, j += shift[dir][2]) {
				if (pm1[j] & MARK_BORDER)
					break;
				pm1[j] |= MARK_WEAK_WIRE; //weak wire
			}
		}
	}

	//5 draw strong edge, 128 is border, 1 is edge, 2 is strong wire, 4 is weak wire, 8 is strong insu, 16 is weak insu
	mark.create(prob.rows, prob.cols, CV_8U);
	mark = 0;
	cv::rectangle(mark, Point(0, 0), Point(EDGE_JUDGE_BORDER - 1, mark.rows - 1), Scalar::all(MARK_BORDER), CV_FILLED);
	cv::rectangle(mark, Point(0, 0), Point(mark.cols - 1, EDGE_JUDGE_BORDER - 1), Scalar::all(MARK_BORDER), CV_FILLED);
	cv::rectangle(mark, Point(mark.cols - EDGE_JUDGE_BORDER, 0), Point(mark.cols - 1, mark.rows - 1), Scalar::all(MARK_BORDER), CV_FILLED);
	cv::rectangle(mark, Point(0, mark.rows - EDGE_JUDGE_BORDER), Point(mark.cols - 1, mark.rows - 1), Scalar::all(MARK_BORDER), CV_FILLED);
	for (auto & e : edges) {
		if (e.weight < STRONG_EDGE_WEIGHT)
			continue;
		vector<Point> pts;
		get_line_pts(e.p0, e.p1, pts);
		pts.push_back(e.p1);
		//5.1 check every point in e
		vector<int> check_pt; //if point ok, check_pt=1
		int dir = e.d0;
		for (auto & pt : pts) {
			const uchar * pm1 = m1.ptr<uchar>(pt.y, pt.x);
			if (pm1[0] != MARK_EDGE) {
				check_pt.push_back(0);
				continue;
			}
			bool check_pass = true; //default pass
			for (int i = 1, j = shift[dir][1]; i <= insu_min; i++, j += shift[dir][1]) {
				if (pm1[j] & MARK_BORDER)
					break;
				if (pm1[j] != MARK_WEAK_INSU) { //edge's insu meet wire
					check_pass = false;
					break;
				}
			}
			if (!check_pass) {
				check_pt.push_back(0);
				continue;
			}
			for (int i = 1, j = shift[dir][2]; i <= wire_sweet_len.start; i++, j += shift[dir][2]) {
				if (pm1[j] & MARK_BORDER)
					break;
				if (pm1[j] != MARK_WEAK_WIRE) { //edge's wire meet insu
					check_pass = false;
					break;
				}
			}
			if (!check_pass)
				check_pt.push_back(0);
			else
				check_pt.push_back(1);
		}
		CV_Assert(check_pt.size() == pts.size());

		for (int i = 0; i < (int) pts.size(); i++) {
			Point pt = pts[i];
			if (!check_pt[i])
				continue;
			uchar * pm = mark.ptr<uchar>(pt.y, pt.x);
			pm[0] = MARK_EDGE; //mark edge
			int insu_len = 0; //strong insu length
			for (int i = 1, j = shift[dir][1]; i < insu_min * 2 + wire_sweet_len.start; i++, j += shift[dir][1]) {
				if (pm[j] & MARK_STRONG_INSU) //meet strong insu
					insu_len = -i; //mark it negative
				if (pm[j] & MARK_EDGE) { //meet edge
					if (insu_len == 1 - i) //meet strong insu, then meet edge, perfect, find insu band
						insu_len = i - 1;
					break;
				}
				if (pm[j] & (MARK_BORDER | MARK_STRONG_WIRE | MARK_WEAK_WIRE)) { //meet wire or border
					insu_len = 0;
					break;
				}
			}
			pm[shift[dir][1]] = MARK_EDGE; //mark 1 strong insu
			insu_len = max(3, insu_len);
			for (int i = 2, j = shift[dir][1] * 2; i < insu_len; i++, j += shift[dir][1])
				pm[j] |= MARK_STRONG_INSU; //mark strong insu with insu_len

			int wire_len = 0; //strong wire length
			for (int i = 1, j = shift[dir][2]; i < wire_sweet_len.end; i++, j += shift[dir][2]) {
				if (pm[j] & MARK_STRONG_WIRE) //meet strong wire
					wire_len = -i; //mark it negative
				if (pm[j] & MARK_EDGE) { //meet edge
					if (wire_len == 1 - i) //meet strong wire, then meet edge, perfect, find wire band
						wire_len = i - 1;
					break;
				}
				if (pm[j] & (MARK_BORDER | MARK_WEAK_INSU | MARK_STRONG_INSU)) { //meet insu or border
					wire_len = 0;
					break;
				}
			}
			pm[shift[dir][2]] = MARK_EDGE; //wire
			wire_len = max(3, wire_len);
			for (int i = 2, j = shift[dir][2] * 2; i < wire_len; i++, j += shift[dir][2])
				pm[j] |= MARK_STRONG_WIRE; //mark strong wire with wire_len

		}
	}

	//5 change prob
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		Vec4b * p_prob = prob.ptr<Vec4b>(y);
		uchar * pm = mark.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			if (p_prob[x][POINT_DIR] & 8)  {//via case
				pm[x] |= MARK_VIA;
				continue;
			}
			if (pm[x] & MARK_EDGE) {
				p_prob[x][POINT_IS_EDGE_WIRE_INSU] = POINT_TOT_PROB;
				p_prob[x][POINT_IS_INSU] = 0;
				p_prob[x][POINT_IS_WIRE] = 0;
			}
			else
				if (pm[x] & MARK_STRONG_WIRE) {
					p_prob[x][POINT_IS_EDGE_WIRE_INSU] = 0;
					p_prob[x][POINT_IS_INSU] = 0;
					p_prob[x][POINT_IS_WIRE] = POINT_TOT_PROB;
				}
				else
					if (pm[x] & MARK_STRONG_INSU) {
						p_prob[x][POINT_IS_EDGE_WIRE_INSU] = 0;
						p_prob[x][POINT_IS_INSU] = POINT_TOT_PROB;
						p_prob[x][POINT_IS_WIRE] = 0;
					}
		}
	}
}

static int enhance_coef[5][5] = {
	{ 16, 24, 32, 24, 16 },
	{ 24, 48, 64, 48, 24 },
	{ 32, 64, 96, 64, 32 },
	{ 24, 48, 64, 48, 24 },
	{ 16, 24, 32, 24, 16 },
};
/*
Input prob
Output rst
Input iter_num
*/
static void self_enhance(const Mat & prob, Mat & rst, int iter_num)
{
	Mat m[2];
	CV_Assert(prob.type() == CV_8UC4);
	m[0].create(prob.rows, prob.cols, CV_8SC4);
	m[1].create(prob.rows, prob.cols, CV_8SC4);
	int half = POINT_TOT_PROB * 0.5;
	int th = POINT_TOT_PROB * GOOD_WIRE_TH / 100;
	int th2 = POINT_TOT_PROB * GOOD_EDGE_TH / 100;
	for (int y = 0; y < prob.rows; y++) {
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		Vec<char, 4> * pm = m[0].ptr<Vec<char, 4> >(y);
		if (y < EDGE_JUDGE_BORDER || y >= prob.rows - EDGE_JUDGE_BORDER) {
			for (int x = 0; x < prob.cols; x++) {
				pm[x][POINT_IS_INSU] = half;
				pm[x][POINT_IS_WIRE] = half;
				pm[x][POINT_IS_EDGE_WIRE_INSU] = 0;
				pm[x][POINT_DIR] = 64; //can't be affected by others, can't affect others
			}
			continue;
		}
		for (int x = 0; x < prob.cols; x++) {
			if (x < EDGE_JUDGE_BORDER || x >= prob.cols - EDGE_JUDGE_BORDER) {
				pm[x][POINT_IS_INSU] = half;
				pm[x][POINT_IS_WIRE] = half;
				pm[x][POINT_IS_EDGE_WIRE_INSU] = 0;
				pm[x][POINT_DIR] = 64; //can't be affected by others, can't affect others
				continue;
			}
			if (p_prob[x][POINT_DIR] & 8) {//can't be affected by others, via case
				pm[x][POINT_IS_INSU] = half;
				pm[x][POINT_IS_WIRE] = half;
				pm[x][POINT_IS_EDGE_WIRE_INSU] = 0;
				pm[x][POINT_DIR] = p_prob[x][POINT_DIR] | 64;
				continue;
			}
			int iu = p_prob[x][POINT_IS_INSU];
			int wi = p_prob[x][POINT_IS_WIRE];

			pm[x][POINT_IS_EDGE_WIRE_INSU] = 0;
			pm[x][POINT_DIR] = 0; //can be affected by others, but can't affect others
			pm[x][POINT_IS_INSU] = iu;
			pm[x][POINT_IS_WIRE] = wi;
			if (iu > half) {
				pm[x][POINT_IS_EDGE_WIRE_INSU] = min(POINT_TOT_PROB + wi - iu * 2, 0);
				if (pm[x][POINT_IS_EDGE_WIRE_INSU] < 0)
					pm[x][POINT_DIR] = 32; //16 or 32 can affect others
				if (iu > th)
					pm[x][POINT_DIR] |= 64; //can't be affected by others
			}
			else
				if (wi > half) {
					pm[x][POINT_IS_EDGE_WIRE_INSU] = max(wi * 2 - iu - POINT_TOT_PROB, 0);
					if (pm[x][POINT_IS_EDGE_WIRE_INSU] > 0)
						pm[x][POINT_DIR] = 32; //16 or 32 can affect others
					if (wi > th)
						pm[x][POINT_DIR] |= 64; //64 can't be affected by others
				}
				else
					if (iu + wi < half) {
						pm[x][POINT_IS_EDGE_WIRE_INSU] = max(POINT_TOT_PROB - wi * 2 - iu * 2, 0);
						if (pm[x][POINT_IS_EDGE_WIRE_INSU] > 0)
							pm[x][POINT_DIR] = 16 | p_prob[x][POINT_DIR]; //16 or 32 can affect others
						if (iu + wi < POINT_TOT_PROB - th2) //strong edge
							pm[x][POINT_DIR] |= 128; //128 affect factor can't be affected by others
					}
		}
	}
#define SIGN(x) ((x>0)? 1 : ((x==0) ? 0 : -1))
	for (int i = 0; i < iter_num; i++)
		for (int y = 0; y < prob.rows; y++) {
			int i2 = i % 2;
			Vec<char, 4> * pm0[5];
			pm0[0] = (y >= 2) ? m[i2].ptr<Vec<char, 4> >(y - 2) : NULL;
			pm0[1] = (y >= 1) ? m[i2].ptr<Vec<char, 4> >(y - 1) : NULL;
			pm0[2] = m[i2].ptr<Vec<char, 4> >(y);
			pm0[3] = (y + 1 < prob.rows) ? m[i2].ptr<Vec<char, 4> >(y + 1) : NULL;
			pm0[4] = (y + 2 < prob.rows) ? m[i2].ptr<Vec<char, 4> >(y + 2) : NULL;
			Vec<char, 4> * pm1 = m[(i + 1) % 2].ptr<Vec<char, 4> >(y);
			for (int x = 0; x < prob.cols; x++)
				if (pm0[2][x][POINT_DIR] & 64) //can't be affected by others
					pm1[x] = pm0[2][x];
				else { //can be affected by others
					int a = 0;
					for (int yy = 0; yy < 5; yy++)
						for (int xx = -2; xx <= 2; xx++) {
							int dir = pm0[yy][x + xx][POINT_DIR] & 0x3f;
							if (dir == 0 || dir & 8) //no affect or via case
								continue;
							else
								if (dir == 32) //affect by insu or wire
									a += enhance_coef[yy][xx + 2] * pm0[yy][x + xx][POINT_IS_EDGE_WIRE_INSU];
								else { //affect by edge
									CV_Assert(dir >= 16 && dir < 24);
									dir -= 16;
									int s = dxy[dir][0] * (2 - yy) - dxy[dir][1] * xx; //vector inner product
									a += SIGN(s) * enhance_coef[yy][xx + 2] * pm0[yy][x + xx][POINT_IS_EDGE_WIRE_INSU];
								}
						}
					a = a >> 7;
					int iu = pm0[2][x][POINT_IS_INSU];
					int wi = pm0[2][x][POINT_IS_WIRE];
					if (a > 0) {
						wi += a;
						wi = wi * POINT_TOT_PROB / (wi + iu);
						iu = POINT_TOT_PROB - wi;
					}
					else {
						iu -= a;
						wi = (wi + iu == 0) ? POINT_TOT_PROB / 2 : wi * POINT_TOT_PROB / (wi + iu);
						iu = POINT_TOT_PROB - wi;
					}
					pm1[x][POINT_IS_EDGE_WIRE_INSU] = 0;
					pm1[x][POINT_DIR] = 0; //can be affected by others, but can't affect others
					pm1[x][POINT_IS_INSU] = iu;
					pm1[x][POINT_IS_WIRE] = wi;
					if (pm0[2][x][POINT_DIR] & 128) { //strong edge
						pm1[x][POINT_IS_EDGE_WIRE_INSU] = pm0[2][x][POINT_IS_EDGE_WIRE_INSU]; //don't change affect factor
						pm1[x][POINT_DIR] = pm0[2][x][POINT_DIR];
					}
					else {
						if (pm0[2][x][POINT_DIR] & 16) { //weak edge
							pm1[x][POINT_IS_EDGE_WIRE_INSU] = max(pm0[2][x][POINT_IS_EDGE_WIRE_INSU] - 20, 0); //reduce affect factor
							pm1[x][POINT_DIR] = pm0[2][x][POINT_DIR];
						}
						else
							if (iu > half) {
								pm1[x][POINT_IS_EDGE_WIRE_INSU] = min(POINT_TOT_PROB + wi - iu * 2, 0);
								if (pm1[x][POINT_IS_EDGE_WIRE_INSU] < 0)
									pm1[x][POINT_DIR] = 32; //16 or 32 can affect others
								if (iu > th)
									pm1[x][POINT_DIR] |= 64; //can't be affected by others
							}
							else
								if (wi > half) {
									pm1[x][POINT_IS_EDGE_WIRE_INSU] = max(wi * 2 - iu - POINT_TOT_PROB, 0);
									if (pm1[x][POINT_IS_EDGE_WIRE_INSU] > 0)
										pm1[x][POINT_DIR] = 32; //16 or 32 can affect others
									if (wi > th)
										pm1[x][POINT_DIR] |= 64; //64 can't be affected by others
								}
					}
				}
		}
	rst.create(prob.rows, prob.cols, CV_8UC4);
	for (int y = 0; y < rst.rows; y++) {
		Vec<char, 4> * pm = m[iter_num % 2].ptr<Vec<char, 4> >(y);
		Vec4b * prst = rst.ptr<Vec4b>(y);
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		for (int x = 0; x < rst.cols; x++) {
			prst[x][POINT_IS_INSU] = pm[x][POINT_IS_INSU];
			prst[x][POINT_IS_WIRE] = pm[x][POINT_IS_WIRE];
			prst[x][POINT_IS_EDGE_WIRE_INSU] = p_prob[x][POINT_IS_EDGE_WIRE_INSU];
			prst[x][POINT_DIR] = p_prob[x][POINT_DIR];
		}
	}
}

/*
input img
input mark
output m01
mark wire as 2, insu as 0, edge as 1
*/
void mark_wvi210(const Mat & img, const Mat & mark, Mat & c210)
{
	static const short not_sure_th[] = {1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
	CV_Assert(img.size() == mark.size() && img.type() == mark.type());
	c210.create(img.size(), CV_8U);
	//1 compute global gray stat for wire and insu
	unsigned wire_stat[256] = { 0 };
	unsigned insu_stat[256] = { 0 };

	for (int y = EDGE_JUDGE_BORDER; y < img.rows - EDGE_JUDGE_BORDER; y++) {
		const uchar * pm = mark.ptr<uchar>(y);
		const uchar * pimg = img.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < img.cols - EDGE_JUDGE_BORDER; x++)
			if (pm[x] & MARK_STRONG_WIRE)
				wire_stat[pimg[x]]++;
			else
				if (pm[x] & MARK_STRONG_INSU)
					insu_stat[pimg[x]]++;
	}

	//2 compute global th
	int total_w = 0, total_i = 0, wire_th = 0, insu_th = 0;
	float th;
	for (int i = 0; i < 256; i++) {
		total_w += wire_stat[i];
		total_i += insu_stat[i];
	}
	for (int i = 0; i < 256; i++) {
		wire_th += wire_stat[i];
		if (wire_th > total_w / 10) { //low 10%
			wire_th = i;
			break;
		}
	}
	for (int i = 255; i >= 0; i--) {
		insu_th += insu_stat[i];
		if (insu_th > total_i / 10) { //high 10%
			insu_th = i;
			break;
		}
	}
	th = (wire_th + insu_th) * 0.5;
	qInfo("strongwire num=%d, stronginsu num=%d, wire_th=%d, insu_th=%d, th=%f", total_w, total_i, wire_th, insu_th, th);

	//3 compute grid gray stat for wire and insu
	int sizes[] = { (img.rows + MARK_210_GRID - 1) / MARK_210_GRID, (img.cols + MARK_210_GRID - 1) / MARK_210_GRID, 16 }; //15 level
	Mat statw_g(3, sizes, CV_8U), stati_g(3, sizes, CV_8U); //0..14 for gray level, 15 for total
	sizes[0]++;
	sizes[1]++;
	Mat statw_integrate(3, sizes, CV_16U), stati_integrate(3, sizes, CV_16U); //integrate of statw_g
	statw_g = Scalar::all(0);
	stati_g = Scalar::all(0);

	for (int y = EDGE_JUDGE_BORDER; y < img.rows - EDGE_JUDGE_BORDER; y++) {
		const uchar * pm = mark.ptr<uchar>(y);
		const uchar * pimg = img.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < img.cols - EDGE_JUDGE_BORDER; x++)
			if (pm[x] & MARK_STRONG_WIRE || pm[x] & MARK_STRONG_INSU) {
				int z = pimg[x];
				z = (z - th) / 2 + 7;
				if (z < 0)
					z = 0;
				else
					if (z > 14)
						z = 14;
				if (pm[x] & MARK_STRONG_WIRE) {
					statw_g.at<uchar>(y / MARK_210_GRID, x / MARK_210_GRID, z)++;
					statw_g.at<uchar>(y / MARK_210_GRID, x / MARK_210_GRID, 15)++;
				}
				else {
					stati_g.at<uchar>(y / MARK_210_GRID, x / MARK_210_GRID, z)++;
					stati_g.at<uchar>(y / MARK_210_GRID, x / MARK_210_GRID, 15)++;
				}
			}
	}
	//3.1 compute integrate
	for (int y = 0; y < sizes[0]; y++) {
		ushort isum[16] = { 0 };
		ushort wsum[16] = { 0 };
		ushort * pstatw_ig = statw_integrate.ptr<ushort>(y, 0);
		ushort * pstati_ig = stati_integrate.ptr<ushort>(y, 0);
		if (y == 0) { //ig[0][...] = 0
			for (int x = 0; x < sizes[1]; x++) {
				pstatw_ig = statw_integrate.ptr<ushort>(y, x);
				pstati_ig = stati_integrate.ptr<ushort>(y, x);
				for (int z = 0; z < 16; z++) {
					pstatw_ig[z] = 0;
					pstati_ig[z] = 0;
				}
			}
			continue;
		}
		for (int z = 0; z < 16; z++) { //ig[y][0] = 0;
			pstatw_ig[z] = 0;
			pstati_ig[z] = 0;
		}
		for (int x = 0; x < sizes[1] - 1; x++) {
			pstatw_ig = statw_integrate.ptr<ushort>(y, x + 1);
			ushort * pstatw_ig_1 = statw_integrate.ptr<ushort>(y - 1, x + 1);
			pstati_ig = stati_integrate.ptr<ushort>(y, x + 1);
			ushort * pstati_ig_1 = stati_integrate.ptr<ushort>(y - 1, x + 1);
			uchar * pstatw = statw_g.ptr<uchar>(y - 1, x);
			uchar * pstati = stati_g.ptr<uchar>(y - 1, x);
			for (int z = 0; z < 16; z++) { //wsum+=stat[y-1][x], ig[y][x+1] = ig[y-1][x+1] + wsum
				wsum[z] += pstatw[z];
				isum[z] += pstati[z];
				pstatw_ig[z] = pstatw_ig_1[z] + wsum[z];
				pstati_ig[z] = pstati_ig_1[z] + isum[z];
			}
#if 1
			ushort tw = 0, ti = 0;
			for (int z = 0; z < 15; z++) {
				tw += pstatw_ig[z];
				ti += pstati_ig[z];
			}
			CV_Assert(tw == pstatw_ig[15] && ti == pstati_ig[15]);
#endif
		}
	}
	CV_Assert(statw_integrate.at<ushort>(sizes[0] - 1, sizes[1] - 1, 15) == (total_w & 0xffff) &&
		stati_integrate.at<ushort>(sizes[0] - 1, sizes[1] - 1, 15) == (total_i & 0xffff));

	//4 compute grid th
	Mat insu_grid(sizes[0] - 1, sizes[1] - 1, CV_8U);
	Mat wire_grid(sizes[0] - 1, sizes[1] - 1, CV_8U);
	for (int y = 0; y < sizes[0] - 1; y++) {
		int prevw_size = 3, previ_size = 3;
		ushort statw[16], stati[16];
		for (int x = 0; x < sizes[1] - 1; x++) {
			int s;
			float wire_gray = 0, insu_gray = 0;
			for (s = max(prevw_size - 1, 2); s < 20; s++) {
				int y0 = max(y - s, 0);
				int x0 = max(x - s, 0);
				int y1 = min(y + s + 1, sizes[0] - 1);
				int x1 = min(x + s + 1, sizes[1] - 1);
				ushort numw = statw_integrate.at<ushort>(y1, x1, 15) + statw_integrate.at<ushort>(y0, x0, 15) -
					statw_integrate.at<ushort>(y1, x0, 15) - statw_integrate.at<ushort>(y0, x1, 15);
				if (numw > GRID_EDGE_TH) {
					ushort remain = numw / 4;
					for (int z = 0; z < 15; z++) {
						statw[z] = statw_integrate.at<ushort>(y1, x1, z) + statw_integrate.at<ushort>(y0, x0, z) -
							statw_integrate.at<ushort>(y1, x0, z) - statw_integrate.at<ushort>(y0, x1, z);
						wire_gray += z * min(remain, statw[z]);
						if (remain <= statw[z])
							break;
						remain -= statw[z];
					}
					statw[15] = numw;
					wire_gray = wire_gray * 4 / numw;
					wire_gray = (wire_gray - 7) * 2 + th;
					prevw_size = s;
					break;
				}
			}
			for (s = max(previ_size - 1, 2); s < 20; s++) {
				int y0 = max(y - s, 0);
				int x0 = max(x - s, 0);
				int y1 = min(y + s + 1, sizes[0] - 1);
				int x1 = min(x + s + 1, sizes[1] - 1);
				ushort numi = stati_integrate.at<ushort>(y1, x1, 15) + stati_integrate.at<ushort>(y0, x0, 15) -
					stati_integrate.at<ushort>(y1, x0, 15) - stati_integrate.at<ushort>(y0, x1, 15);
				if (numi > GRID_EDGE_TH) {
					ushort remain = numi / 4;
					for (int z = 14; z >= 0; z--) {
						stati[z] = stati_integrate.at<ushort>(y1, x1, z) + stati_integrate.at<ushort>(y0, x0, z) -
							stati_integrate.at<ushort>(y1, x0, z) - stati_integrate.at<ushort>(y0, x1, z);
						insu_gray += z * min(remain, stati[z]);
						if (remain <= stati[z])
							break;
						remain -= stati[z];
					}
					stati[15] = numi;
					insu_gray = insu_gray * 4 / numi;
					insu_gray = (insu_gray - 7) * 2 + th;
					previ_size = s;
					break;
				}
			}
			if (insu_gray > 0)
				insu_grid.at<uchar>(y, x) = insu_gray;
			else
				insu_grid.at<uchar>(y, x) = th;
			if (wire_gray > 0)
				wire_grid.at<uchar>(y, x) = wire_gray;
			else
				wire_grid.at<uchar>(y, x) = th;
		}
	}

	//5 judge 0,1,2
	Mat insu_grid_th, wire_grid_th;
	resize(insu_grid, insu_grid_th, Size(insu_grid.cols * MARK_210_GRID, insu_grid.rows * MARK_210_GRID));
	resize(wire_grid, wire_grid_th, Size(wire_grid.cols * MARK_210_GRID, wire_grid.rows * MARK_210_GRID));
	for (int y = 0; y < img.rows; y++) {
		const uchar * p_insu_th = insu_grid_th.ptr<uchar>(y);
		const uchar * p_wire_th = wire_grid_th.ptr<uchar>(y);
		const uchar * pm = mark.ptr<uchar>(y);
		const uchar * pimg = img.ptr<uchar>(y);
		uchar * pc = c210.ptr<uchar>(y);
		for (int x = 0; x < img.cols; x++) {
			if (pm[x] & MARK_EDGE)
				pc[x] = 5; //EDGE
			else {
				unsigned char th = (p_insu_th[x] + p_wire_th[x]) / 2;
				if (p_insu_th[x] + 1 >= p_wire_th[x]) {
					short a = p_insu_th[x] + 1 - p_wire_th[x];
					CV_Assert(a < 30);
					if (pimg[x] > p_insu_th[x] + not_sure_th[a])
						pc[x] = (pm[x] & MARK_VIA) ? 7 : 10; //Wire
					else
						if (pimg[x] + not_sure_th[a] < p_wire_th[x])
							pc[x] = (pm[x] & MARK_VIA) ? 3 : 0; //insu
						else
							pc[x] = (pimg[x] > th) ? 6 : 4;
							
				}
				else
					pc[x] = (pm[x] & MARK_VIA) ? (pimg[x] > th ? 7 : 3) : (pimg[x] > th ? 10 : 0);
			}
		}
	}
}

ProcessImageData process_img(const ProcessImageData & pi)
{
	qInfo("process_img (%d,%d,%d)", pi.cpd->x0, pi.cpd->y0, pi.cpd->layer);

	if (!pi.cpd->raw_img.empty()) {
		Mat empty;
		Mat via_mark;
		Mat prob, prob1;
		pi.vwf->via_search(pi.cpd->raw_img, via_mark, (pi.cpd->via_mark_debug == NULL ? empty : *(pi.cpd->via_mark_debug)), pi.cpd->eo, pi.multi_thread);
		pi.vwf->edge_search(pi.cpd->raw_img, prob, via_mark, (pi.cpd->edge_mark_debug == NULL ? empty : pi.cpd->edge_mark_debug[0]), pi.multi_thread);
		if (!prob.empty()) {
			prob_color(prob, (pi.cpd->edge_mark_debug == NULL ? empty : pi.cpd->edge_mark_debug[0]), 0.55);
			filter_prob(prob, prob1);
			prob_color(prob1, (pi.cpd->edge_mark_debug1 == NULL ? empty : pi.cpd->edge_mark_debug1[0]));
			vector<EdgeLine> edges;
			Mat mark, mark2, c210;
			mark_strong_edge(prob1, pi.wire_sweet_len, pi.insu_min, edges, mark, mark2);
			//prob_color(prob1, (pi.cpd->edge_mark_debug2 == NULL ? empty : pi.cpd->edge_mark_debug2[0]), 0.98);
			//self_enhance(prob1, prob, 3);	
			mark_color(mark, (pi.cpd->edge_mark_debug2 == NULL ? empty : pi.cpd->edge_mark_debug2[0]));
			mark_wvi210(pi.cpd->raw_img, mark, c210);
			mark_210(c210, (pi.cpd->edge_mark_debug3 == NULL ? empty : pi.cpd->edge_mark_debug3[0]));
		}
		QPoint tl(pi.cpd->img_pixel_x0, pi.cpd->img_pixel_y0);
		for (auto & o : pi.cpd->eo) { //change local to global
			o->type3 = pi.cpd->layer;
			o->p0 = o->p0 + tl;
		}
	}
	qInfo("process_img (%d,%d,%d) o=%d", pi.cpd->x0, pi.cpd->y0, pi.cpd->layer, pi.cpd->eo.size());
	return pi;
}

void merge_process_data(ProcessData * pd0, ProcessData * pd1)
{
	qInfo("merge_process_data (%d,%d,%d) o=%d and (%d,%d,%d) o=%d", pd0->x0, pd0->y0, pd0->layer, pd0->eo.size(),
		pd1->x0, pd1->y0, pd1->layer, pd1->eo.size());
	for (int i = 0; i < (int)pd0->eo.size(); i++)
		if (pd0->eo[i]->type == OBJ_POINT) {
			QPoint loc = pd0->eo[i]->p0;
			int layer = pd0->eo[i]->type3;
			for (int j = 0; j < (int)pd1->eo.size(); j++)
				if (pd1->eo[j]->type == OBJ_POINT && pd1->eo[j]->type3 == layer && pd0->eo[i] != pd1->eo[j]) {
					QPoint diff = loc - pd1->eo[j]->p0;
					if (abs(diff.x()) <= 2 && abs(diff.y()) <= 2) { //pd0 and pd1 point to same via, merge
						if (pd0->eo[i]->un.attach >= pd1->eo[j]->un.attach) {
							CV_Assert(pd1->eo[j]->un.attach == 1);
							pd0->eo[i]->un.attach++;
							delete pd1->eo[j];
							pd1->eo[j] = pd0->eo[i];
						}
						else {
							pd1->eo[j]->un.attach++;
							delete pd0->eo[i];
							pd0->eo[i] = pd1->eo[j];
						}
						break;
					}
				}
		}
		else {

		}
}

void get_result(ProcessData * pd, vector<ElementObj *> & objs)
{
	qInfo("get_result (%d,%d,%d) w=%d,h=%d,o=%d", pd->x0, pd->y0, pd->layer,
		pd->raw_img.cols, pd->raw_img.rows, pd->eo.size());
	for (auto & e : pd->eo)
		if (e->type == OBJ_POINT) {
			if (e->un.attach == 1) {
				if (!pd->poly)
					objs.push_back(e);
				else
					if (pd->poly->containsPoint(e->p0, Qt::OddEvenFill))
						objs.push_back(e);
			}
			else
				e->un.attach--;
		}
}

void merge_img_result(vector<ElementObj *> & objs, const ProcessImageData & t)
{
	if (t.lpd)
		merge_process_data(t.lpd, t.cpd);
	if (t.upd)
		merge_process_data(t.upd, t.cpd);
	if (t.lpd) {
		if (t.lpd->ref_cnt-- == 1)
			get_result(t.lpd, objs);
	}
	if (t.upd) {
		if (t.upd->ref_cnt-- == 1)
			get_result(t.upd, objs);
	}
}

void process_imgs(vector<ProcessImageData> & pis, vector<ElementObj *> & obj_sets, bool parallel) {
	if (parallel) {
		//each thread process all layer on same tile
		vector<ElementObj *> temp_vec;
		temp_vec = QtConcurrent::blockingMappedReduced<vector<ElementObj *>, vector<ProcessImageData> >(pis, process_img, merge_img_result,
			QtConcurrent::OrderedReduce | QtConcurrent::SequentialReduce);
		obj_sets.insert(obj_sets.end(), temp_vec.begin(), temp_vec.end());
	}
	else
		for (int i = 0; i < pis.size(); i++) {
			process_img(pis[i]);
			//3 output result to obj_sets
			merge_img_result(obj_sets, pis[i]);
		}
}

VWExtractML::VWExtractML()
{
	layer_min = 0;
	layer_max = 0;
	via_diameter_min = 8;
	via_diameter_max = 9;
	insu_min = 3;
	wire_min = 7;
	train_cmd = 0;
}

/*    31..24 23..16 15..8 7..0
type
d            clear dmax  dmin
clear
*/
int VWExtractML::set_train_param(int layer, int type, int d, int, int, int, int, int, int, float)
{
	layer_min = layer & 0xff;
	layer_max = layer >> 8 & 0xff;
	train_cmd = layer >> 16 & 0xff;
	if ((type & 0xff) == OBJ_POINT && ((type >> 8 & 0xff) == POINT_NORMAL_VIA0 || (type >> 8 & 0xff) == POINT_NO_VIA)) {
		via_diameter_min = d & 0xff;
		via_diameter_max = d >> 8 & 0xff;
		return 0;
	}
	if ((type & 0xff) == OBJ_POINT && ((type >> 8 & 0xff) == POINT_WIRE_INSU || (type >> 8 & 0xff) == POINT_WIRE
		|| (type >> 8 & 0xff) == POINT_INSU || (type >> 8 & 0xff) == POINT_WIRE_INSU_V)
		|| (type >> 8 & 0xff) == POINT_WIRE_V || (type >> 8 & 0xff) == POINT_INSU_V) {
		insu_min = d & 0xff;
		wire_min = d >> 8 & 0xff;
	}
	return 0;
}

int VWExtractML::set_extract_param(int layer, int, int, int, int, int, int, int, int, float)
{
	layer_min = layer & 0xff;
	layer_max = layer >> 8 & 0xff;
	layer_max = max(layer_max, layer_min);
	if (layer_max >= (int)vwf.size()) {
		vwf.resize(layer_max + 1);
		via_mark.resize(layer_max + 1);
		edge_mark.resize(layer_max + 1);
		edge_mark1.resize(layer_max + 1);
		edge_mark2.resize(layer_max + 1);
		edge_mark3.resize(layer_max + 1);
	}
	return 0;
}

Mat VWExtractML::get_mark(int layer)
{
	if (layer < edge_mark.size())
		return edge_mark[layer];
	return Mat();
}

Mat VWExtractML::get_mark1(int layer)
{
	if (layer < edge_mark1.size())
		return edge_mark1[layer];
	return Mat();
}

Mat VWExtractML::get_mark2(int layer)
{
	if (layer < edge_mark2.size())
		return edge_mark2[layer];
	return Mat();
}

Mat VWExtractML::get_mark3(int layer)
{
	if (layer < edge_mark3.size())
		return edge_mark3[layer];
	return Mat();
}

int VWExtractML::train(string img_name, vector<MarkObj> & obj_sets)
{
	if (train_cmd != TRAIN_CMD_GET) {
		layer_min = 200, layer_max = 0;
		for (auto & m : obj_sets) {
			layer_min = (layer_min > m.type3) ? m.type3 : layer_min;
			layer_max = (layer_max < m.type3) ? m.type3 : layer_max;
		}
	}
	else
		layer_max = layer_min;
	if (layer_max >= (int)vwf.size()) {
		vwf.resize(layer_max + 1);
		via_mark.resize(layer_max + 1);
		edge_mark.resize(layer_max + 1);
		edge_mark1.resize(layer_max + 1);
		edge_mark2.resize(layer_max + 1);
		edge_mark3.resize(layer_max + 1);
	}
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		string file_name(img_name);
		file_name[file_name.length() - 5] = current_layer + '0';
		Mat img = imread(file_name, 0);
		if (via_mark[current_layer].empty()) {
			via_mark[current_layer].create(img.rows, img.cols, CV_8U);
			via_mark[current_layer] = Scalar::all(0);
		}
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		int temp1, temp2;
		vwf[current_layer].read_file(project_path, current_layer, temp1, temp2);
		if (train_cmd == TRAIN_CMD_GET) {
			vwf[current_layer].get_features(obj_sets);
			for (auto & o : obj_sets)
				o.type3 = current_layer;
			return 0;
		}
		else
			for (auto & m : obj_sets)
				if (train_cmd == TRAIN_CMD_INSERT) {
					if (m.type == OBJ_POINT && m.type3 == current_layer) {
						if (m.type2 == POINT_NO_VIA || m.type2 == POINT_NORMAL_VIA0) { // add via or no via
							Point range;
							int label = (m.type2 == POINT_NO_VIA) ? 0 : VIA_IS_VIA;
							vector<Point> vs;
							label |= VIA_FEATURE;
							Point loc = Point(m.p0.x(), m.p0.y());
							bool ret = vwf[current_layer].add_feature(img, loc, loc, range,
								via_diameter_min, via_diameter_max, label, &vs);
							if (ret) { //add success
								m.p0 = QPoint(loc.x, loc.y); //return location
								m.p1 = QPoint(range.x, range.y); //return diameter
								for (auto p : vs)
									via_mark[current_layer].at<uchar>(p) = 255;
							}
							else
								m.p1 = QPoint(0, 0); //add fail
						}
						if (m.type2 == POINT_WIRE_INSU || m.type2 == POINT_WIRE || m.type2 == POINT_INSU ||
							m.type2 == POINT_WIRE_INSU_V || m.type2 == POINT_WIRE_V || m.type2 == POINT_INSU_V) {
							Point range = (m.type2 == POINT_WIRE_INSU || m.type2 == POINT_WIRE_INSU_V) ?
								Point(2, 2) : Point(1, 1);
							Point loc = Point(m.p0.x(), m.p0.y());
							int label = EDGE_FEATURE;
							label |= (m.type2 == POINT_WIRE_INSU_V) ? EDGE_IS_WIRE_INSU | EDGE_NEAR_VIA :
								(m.type2 == POINT_WIRE_V) ? EDGE_IS_WIRE | EDGE_NEAR_VIA :
								(m.type2 == POINT_INSU_V) ? EDGE_IS_INSU | EDGE_NEAR_VIA :
								(m.type2 == POINT_WIRE_INSU) ? EDGE_IS_WIRE_INSU :
								(m.type2 == POINT_WIRE) ? EDGE_IS_WIRE : EDGE_IS_INSU;

							bool ret = vwf[current_layer].add_feature(img, loc, loc, range,
								via_diameter_min, via_diameter_max, label, NULL);
							if (ret) { //add success
								m.p0 = QPoint(loc.x, loc.y); //return location
								m.p1 = QPoint(range.x, range.y); //return diameter
							}
							else
								m.p1 = QPoint(0, 0); //add fail
						}
					}
				}
				else
					if (train_cmd == TRAIN_CMD_DELETE) {
						if (m.type == OBJ_POINT && m.type3 == current_layer) { //delete via or no via
							Point loc = Point(m.p0.x(), m.p0.y());
							if (m.type2 == POINT_NO_VIA || m.type2 == POINT_NORMAL_VIA0) // del via or no via
								vwf[current_layer].del_feature(loc, via_diameter_max);
							if (m.type2 == POINT_WIRE_INSU || m.type2 == POINT_WIRE || m.type2 == POINT_INSU ||
								m.type2 == POINT_WIRE_INSU_V || m.type2 == POINT_WIRE_V || m.type2 == POINT_INSU_V)
								vwf[current_layer].del_feature(loc, 2);
						}
					}
		vwf[current_layer].write_file(project_path, current_layer, insu_min, wire_min);
	}
	return 0;
}

int VWExtractML::extract(string img_name, QRect rect, vector<MarkObj> & obj_sets)
{
	vector<ProcessImageData> pis;
	ProcessData ed[100];
	obj_sets.clear();
	if (layer_max >= (int)vwf.size()) {
		vwf.resize(layer_max + 1);
		via_mark.resize(layer_max + 1);
		edge_mark.resize(layer_max + 1);
		edge_mark1.resize(layer_max + 1);
		edge_mark2.resize(layer_max + 1);
		edge_mark3.resize(layer_max + 1);
	}
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		string file_name(img_name);
		file_name[file_name.length() - 5] = current_layer + '0';
		Mat img = imread(file_name, 0);
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		if (!vwf[current_layer].read_file(project_path, current_layer, insu_min, wire_min))
			return -1;
		ProcessImageData pi;
		ed[current_layer].x0 = 0;
		ed[current_layer].y0 = 0;
		ed[current_layer].img_pixel_x0 = 0;
		ed[current_layer].img_pixel_y0 = 0;
		ed[current_layer].layer = current_layer;
		ed[current_layer].raw_img = img;
		ed[current_layer].poly = NULL;
		via_mark[current_layer] = img.clone();
		cvtColor(img, edge_mark[current_layer], CV_GRAY2BGR);
		edge_mark1[current_layer] = edge_mark[current_layer].clone();
		edge_mark2[current_layer] = edge_mark[current_layer].clone();
		edge_mark3[current_layer] = edge_mark[current_layer].clone();
		ed[current_layer].via_mark_debug = &(via_mark[current_layer]);
		ed[current_layer].edge_mark_debug = &(edge_mark[current_layer]);
		ed[current_layer].edge_mark_debug1 = &(edge_mark1[current_layer]);
		ed[current_layer].edge_mark_debug2 = &(edge_mark2[current_layer]);
		ed[current_layer].edge_mark_debug3 = &(edge_mark3[current_layer]);
		pi.vwf = &vwf[current_layer];
		pi.lpd = NULL;
		pi.upd = NULL;
		pi.cpd = &ed[current_layer];
		pi.multi_thread = false;
		pi.insu_min = insu_min;
		pi.wire_sweet_len = Range(wire_min, wire_min * 2 + insu_min);
		pis.push_back(pi);
	}
	vector<ElementObj *> es;
	process_imgs(pis, es, false);
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++)
		get_result(&ed[current_layer], es);
	convert_element_obj(es, obj_sets, 1);
	return 0;
}

#define PREPARE_IMG_RADIUS 60

int VWExtractML::train(vector<ICLayerWrInterface *> & ics, vector<MarkObj> & obj_sets)
{
	if (train_cmd != TRAIN_CMD_GET) {
		layer_min = 200, layer_max = 0;
		for (auto & m : obj_sets) {
			layer_min = (layer_min > m.type3) ? m.type3 : layer_min;
			layer_max = (layer_max < m.type3) ? m.type3 : layer_max;
		}
	}
	else
		layer_max = layer_min;
	int block_width = ics[0]->getBlockWidth();
	int scale = 32768 / block_width;
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		if (current_layer >= (int)vwf.size())
			vwf.resize(current_layer + 1);
		string img_name = ics[current_layer]->get_file_name();
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		int temp1, temp2;
		vwf[current_layer].read_file(project_path, current_layer, temp1, temp2);
		if (train_cmd == TRAIN_CMD_GET) {
			vwf[current_layer].get_features(obj_sets);
			for (auto & o : obj_sets)
				o.type3 = current_layer;
		}
		else
			for (auto & m : obj_sets)
				if (train_cmd == TRAIN_CMD_INSERT) {
					if (m.type == OBJ_POINT && m.type3 == current_layer) {
						Point range;
						int label = (m.type2 == POINT_NO_VIA) ? 0 : VIA_IS_VIA;
						label |= VIA_FEATURE;
						Point c = Point(m.p0.x() / scale, m.p0.y() / scale);
						Point loc(PREPARE_IMG_RADIUS, PREPARE_IMG_RADIUS);
						Mat img = prepare_raw_img(ics[current_layer], 0,
							QRect((c - loc).x, (c - loc).y, PREPARE_IMG_RADIUS * 2 + 1, PREPARE_IMG_RADIUS * 2 + 1));
						bool ret = vwf[current_layer].add_feature(img, loc, c, range,
							via_diameter_min, via_diameter_max, label, NULL);
						if (ret) {
							m.p0 = QPoint(c.x * scale, c.y * scale);
							m.p1 = QPoint(range.x, range.y);
						}
						else
							m.p1 = QPoint(0, 0);
					}
				}
				else
					if (train_cmd == TRAIN_CMD_DELETE) {
						if (m.type == OBJ_POINT && m.type3 == current_layer) {
							Point c = Point(m.p0.x() / scale, m.p0.y() / scale);
							vwf[current_layer].del_feature(c, via_diameter_max);
						}
					}
		if (train_cmd == TRAIN_CMD_DELETE)
			obj_sets.clear();
		vwf[current_layer].write_file(project_path, current_layer, insu_min, wire_min);
	}
	return 0;
}

struct SearchAreaPoly {
	QPolygon poly;
	int option;
	SearchAreaPoly(QPolygon p, int o) {
		poly = p;
		option = o;
	}
};
int VWExtractML::extract(vector<ICLayerWrInterface *> & ics, const vector<SearchArea> & area_rect, vector<MarkObj> & obj_sets)
{
	QDir *qdir = new QDir;
	deldir("./DImg");
	bool exist = qdir->exists("./DImg");
	if (!exist) {
		bool ok = qdir->mkdir("./DImg");
		if (!ok)
			qCritical("mkdir failed");
	}
	delete qdir;
	int BORDER_SIZE = 0;
	int insu_mins[50] = { 0 };
	int wire_mins[50] = { 0 };
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		if (current_layer >= (int)vwf.size())
			vwf.resize(current_layer + 1);
		string img_name = ics[current_layer]->get_file_name();
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer, insu_mins[current_layer], wire_mins[current_layer]);
		BORDER_SIZE = max(BORDER_SIZE, vwf[current_layer].get_max_d());
	}
	BORDER_SIZE += 8;

	vector<SearchAreaPoly> area_;
	QPolygon area_poly;
	int option;
	for (auto & ar : area_rect) {
		if (ar.option & OPT_POLYGON_SEARCH) {
			area_poly.push_back(ar.rect.topLeft());
			area_poly.push_back(QPoint(ar.rect.width(), ar.rect.height()));
			option = ar.option;
		}
		else {
			if (area_poly.size() > 2) {
				area_.push_back(SearchAreaPoly(area_poly, option));
				for (int i = 0; i < area_poly.size(); i += 2)
					qInfo("Poly (%x,%x) (%x,%x)", area_poly[i].x(), area_poly[i].y(), area_poly[i + 1].x(), area_poly[i + 1].y());
				area_poly.clear();
			}
			area_.push_back(SearchAreaPoly(QPolygon(ar.rect), ar.option));
		}
	}
	if (area_poly.size() > 2) {
		area_.push_back(SearchAreaPoly(area_poly, option));
		for (int i = 0; i < area_poly.size(); i += 2)
			qInfo("Poly (%x,%x) (%x,%x)", area_poly[i].x(), area_poly[i].y(), area_poly[i + 1].x(), area_poly[i + 1].y());
		area_poly.clear();
	}
	int block_x, block_y;
	ics[0]->getBlockNum(block_x, block_y);
	int block_width = ics[0]->getBlockWidth();
	int scale = 32768 / block_width;
	obj_sets.clear();

	for (int area_idx = 0; area_idx < area_.size(); area_idx++) {
		int extend = BORDER_SIZE * scale;
		int parallel_search = area_[area_idx].option & OPT_PARALLEL_SEARCH;
		QRect sr = area_[area_idx].poly.boundingRect().marginsAdded(QMargins(extend, extend, extend, extend));
		QPolygon cur_poly;
		for (int i = 0; i < area_[area_idx].poly.size(); i++)
			cur_poly.push_back(QPoint(area_[area_idx].poly[i].x() / scale, area_[area_idx].poly[i].y() / scale));

		sr &= QRect(0, 0, block_x << 15, block_y << 15);
		if (sr.width() <= 0x10000 || sr.height() <= 0x10000) {
			vector<ProcessImageData> pis;
			ProcessData ed[100];
			for (int l = layer_min; l <= layer_max; l++) {
				if (!vwf[l].via_valid(false)) {
					qWarning("skip layer %d, its feature is invalid", l);
					continue;
				}
				ProcessImageData pi;
				ed[l].x0 = 0;
				ed[l].y0 = 0;
				ed[l].img_pixel_x0 = sr.left() / scale;
				ed[l].img_pixel_y0 = sr.top() / scale;
				ed[l].layer = l;
				QRect r(sr.left() / scale, sr.top() / scale, sr.width() / scale, sr.height() / scale);
				ed[l].raw_img = prepare_raw_img(ics[l], 0, r);
				pi.vwf = &vwf[l];
				pi.lpd = NULL;
				pi.upd = NULL;
				pi.cpd = &ed[l];
				pi.multi_thread = false;
				pis.push_back(pi);
			}
			vector<ElementObj *> es;
			process_imgs(pis, es, false);
			for (int l = layer_min; l <= layer_max; l++)
				get_result(&ed[l], es);
			convert_element_obj(es, obj_sets, scale);
		}
		else {
			//following same as vwextract2
			QRect sb(QPoint((sr.left() + 0x4000) >> 15, (sr.top() + 0x4000) >> 15),
				QPoint((sr.right() - 0x4000) >> 15, (sr.bottom() - 0x4000) >> 15));
			CV_Assert(sb.right() >= sb.left() && sb.bottom() >= sb.top());
			vector<vector<ProcessData> > diag_line[2];
			for (int i = 0; i < 2; i++) {
				diag_line[i].resize(layer_max + 1);
				for (int j = 0; j < (int)diag_line[i].size(); j++)
					diag_line[i][j].resize(max(sb.width() + 1, sb.height() + 1));
			}
			int lx = sr.left() - (sb.left() << 15); //do 4dec/5inc
			int rx = sr.right() - ((sb.right() + 1) << 15);
			int ty = sr.top() - (sb.top() << 15);
			int by = sr.bottom() - ((sb.bottom() + 1) << 15);
			int cl = 1;
			int cl_num;
			QPoint sr_tl_pixel = sr.topLeft() / scale;
			qInfo("extract Rect, lt=(%d,%d), rb=(%d,%d), multithread=%d", sr_tl_pixel.x(), sr_tl_pixel.y(), sr.right() / scale, sr.bottom() / scale, parallel_search);
			for (int xay = sb.left() + sb.top(); xay <= sb.right() + sb.bottom(); xay++) {
				/*Scan from top left to bottom right. One loop process one xie line /,
				For xie line /, process each ProcessData concurrenty*/
				cl = 1 - cl;
				cl_num = 0;
				for (int i = 0; i < (int)diag_line[cl].size(); i++) //release current diag_line memory
					for (int j = 0; j < (int)diag_line[cl][i].size(); j++) {
						CV_Assert(diag_line[cl][i][j].ref_cnt == 0);
						diag_line[cl][i][j].eo.clear();
					}
				//1 load image to diag_line
				for (int x0 = sb.left(); x0 <= sb.right(); x0++) {
					int y0 = xay - x0;
					if (sb.contains(x0, y0)) { //load image per Tile
						bool in_area = true;
						bool need_check_inpoly = false;
						cl_num++;
						QPoint p0(x0 << 15, y0 << 15);
						QPoint p1((x0 << 15) + 0x4000, (y0 << 15) + 0x4000);
						QPoint p2((x0 << 15) + 0x8000 + BORDER_SIZE * scale, (y0 << 15) + 0x8000 + BORDER_SIZE * scale);
						bool in0 = area_[area_idx].poly.containsPoint(p0, Qt::OddEvenFill);
						bool in1 = area_[area_idx].poly.containsPoint(p1, Qt::OddEvenFill);
						bool in2 = area_[area_idx].poly.containsPoint(p2, Qt::OddEvenFill);
						if (!in0 && !in1 &&	!in2 && !(x0 == sb.left() && y0 == sb.top()))
							in_area = false;
						if (!in0 || !in1 || !in2)
							need_check_inpoly = true;
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
						int ewide = (x0 == sb.left()) ? 0 : BORDER_SIZE * 2;
						int ehight = (y0 == sb.top()) ? 0 : BORDER_SIZE * 2;
						for (int l = layer_min; l <= layer_max; l++) {
							Mat img(height[0] + height[1] + height[2] + ehight, wide[0] + wide[1] + wide[2] + ewide, CV_8U);
							ProcessData * d = &diag_line[cl][l][cl_num - 1];
							d->layer = l;
							d->x0 = x0;
							d->y0 = y0;
							d->poly = need_check_inpoly ? &cur_poly : NULL;
							if (ewide == 0 && ehight == 0) {
								d->img_pixel_x0 = sr_tl_pixel.x();
								d->img_pixel_y0 = sr_tl_pixel.y();
							}
							if (ewide != 0) { // copy left image from old diag_line
								Mat * s_img;
								if (diag_line[1 - cl][l][cl_num - 1].y0 == y0) {
									s_img = &diag_line[1 - cl][l][cl_num - 1].raw_img;
									d->img_pixel_x0 = diag_line[1 - cl][l][cl_num - 1].img_pixel_x0 + s_img->cols
										- BORDER_SIZE * 2;
									d->img_pixel_y0 = diag_line[1 - cl][l][cl_num - 1].img_pixel_y0;
								}
								else {
									CV_Assert(diag_line[1 - cl][l][cl_num - 2].y0 == y0);
									s_img = &diag_line[1 - cl][l][cl_num - 2].raw_img;
									d->img_pixel_x0 = diag_line[1 - cl][l][cl_num - 2].img_pixel_x0 + s_img->cols
										- BORDER_SIZE * 2;
									d->img_pixel_y0 = diag_line[1 - cl][l][cl_num - 2].img_pixel_y0;
								}
								if (in_area) {
									if (!s_img->empty()) {
										CV_Assert(s_img->rows == img.rows);
										(*s_img)(Rect(s_img->cols - BORDER_SIZE * 2, 0, BORDER_SIZE * 2, img.rows)).copyTo(img(Rect(0, 0, BORDER_SIZE * 2, img.rows)));
									}
									else
										img(Rect(0, 0, BORDER_SIZE * 2, img.rows)) = Scalar::all(0);
								}
							}

							if (ehight != 0) {  // copy upper image from old diag_line
								Mat * s_img;
								if (diag_line[1 - cl][l][cl_num - 1].x0 == x0) {
									s_img = &diag_line[1 - cl][l][cl_num - 1].raw_img;
									if (ewide != 0)
										CV_Assert(d->img_pixel_x0 == diag_line[1 - cl][l][cl_num - 1].img_pixel_x0 &&
										d->img_pixel_y0 == diag_line[1 - cl][l][cl_num - 1].img_pixel_y0 + s_img->rows
										- BORDER_SIZE * 2);
									d->img_pixel_x0 = diag_line[1 - cl][l][cl_num - 1].img_pixel_x0;
									d->img_pixel_y0 = diag_line[1 - cl][l][cl_num - 1].img_pixel_y0 + s_img->rows
										- BORDER_SIZE * 2;
								}
								else {
									CV_Assert(diag_line[1 - cl][l][cl_num].x0 == x0);
									s_img = &diag_line[1 - cl][l][cl_num].raw_img;
									if (ewide != 0)
										CV_Assert(d->img_pixel_x0 == diag_line[1 - cl][l][cl_num].img_pixel_x0 &&
										d->img_pixel_y0 == diag_line[1 - cl][l][cl_num].img_pixel_y0 + s_img->rows
										- BORDER_SIZE * 2);
									d->img_pixel_x0 = diag_line[1 - cl][l][cl_num].img_pixel_x0;
									d->img_pixel_y0 = diag_line[1 - cl][l][cl_num].img_pixel_y0 + s_img->rows
										- BORDER_SIZE * 2;
								}
								if (in_area) {
									if (!s_img->empty()) {
										CV_Assert(s_img->cols == img.cols);
										(*s_img)(Rect(0, s_img->rows - BORDER_SIZE * 2, img.cols, BORDER_SIZE * 2)).copyTo(img(Rect(0, 0, img.cols, BORDER_SIZE * 2)));
									}
									else
										img(Rect(0, 0, img.cols, BORDER_SIZE * 2)) = Scalar::all(0);
								}
							}
							d->raw_img.release();
							//When extract width(height) > 2 * image width
							//if loading image is not at the edge, load 1*1 image; if at the edge, load 1*1 or 1*2;
							//if at the corner, load 1*1 or 1*2, or 2*2.
							//When extract width(height) < 2 * image width
							//if loading image is at the edge, load 1*1, 1*2 or 1*3 image, if at the corner, load 1*1 or 1*2 or 2*2 or 2*3 or 3*3 
							if (in_area)
								for (int y = y0 - 1, img_y = ehight; y <= y0 + 1; y++) {
									int dy = y - y0;
									for (int x = x0 - 1, img_x = ewide; x <= x0 + 1; x++) {
										int dx = x - x0;
										if (wide[dx + 1] != 0 && height[dy + 1] != 0) {
											vector<uchar> encode_img;
											if (ics[l]->getRawImgByIdx(encode_img, x, y, 0, 0, false) != 0) {
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
												qDebug("load image%d_%d, (x=%d,y=%d),(w=%d,h=%d) to (x=%d,y=%d)", y, x, s.left(), s.top(), s.width(), s.height(), img_x, img_y);
											image(Rect(s.left(), s.top(), s.width(), s.height())).copyTo(img(Rect(img_x, img_y, wide[dx + 1], height[dy + 1])));
										}
										img_x += wide[dx + 1];
									}
									img_y += height[dy + 1];
								}
							d->raw_img = img;
						}
					}
				}
				//2 now diag_line is loaded, process it
				vector<ProcessImageData> pis;
				for (int i = 0; i < cl_num; i++)
					for (int l = layer_min; l <= layer_max; l++) {
						ProcessImageData pi;
						pi.vwf = &vwf[l];
						pi.cpd = &diag_line[cl][l][i];
						pi.lpd = &diag_line[1 - cl][l][i];
						pi.upd = &diag_line[1 - cl][l][i];
						pi.multi_thread = (pi.cpd->x0 == sb.left() && pi.cpd->y0 == sb.top()) ? false : parallel_search;
						if (pi.cpd->y0 > sb.top() && pi.cpd->x0 != pi.upd->x0)
							pi.upd = &diag_line[1 - cl][l][i + 1];
						if (pi.cpd->x0 > sb.left() && pi.cpd->y0 != pi.lpd->y0)
							pi.lpd = &diag_line[1 - cl][l][i - 1];
						if (pi.cpd->x0 > sb.left()) {
							CV_Assert(pi.cpd->y0 == pi.lpd->y0 && pi.cpd->layer == pi.lpd->layer);
							pi.lpd->ref_cnt++;
						}
						else
							pi.lpd = NULL;
						if (pi.cpd->y0 > sb.top()) {
							CV_Assert(pi.cpd->x0 == pi.upd->x0 && pi.cpd->layer == pi.upd->layer);
							pi.upd->ref_cnt++;
						}
						else
							pi.upd = NULL;
						pis.push_back(pi);
					}
				vector<ElementObj *> es;
				process_imgs(pis, es, parallel_search);
				convert_element_obj(es, obj_sets, scale);
			}
			vector<ElementObj *> es;
			for (int l = layer_min; l <= layer_max; l++)
				get_result(&diag_line[cl][l][0], es);
			convert_element_obj(es, obj_sets, scale);
		}
	}
#if SAVE_RST_TO_FILE
	save_rst_to_file(obj_sets, scale);
#endif
	qInfo("VWExtractML Extract finished successfully");
	qDebug("*#*#DumpMessage#*#*");
	return 0;
}
