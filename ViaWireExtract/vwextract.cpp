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
	int weight;
	Point p0, p1;
	uchar d0, d1; //d0 is grad dir, d1 is edge dir, they are orthogo

	EdgeLine(int x0, int y0, int x1, int y1, uchar _d0, uchar _d1) {
		p0 = Point(x0, y0);
		p1 = Point(x1, y1);
		d0 = _d0;
		d1 = _d1;
		CV_Assert(d1 == DIR_DOWN || d1 == DIR_RIGHT || d1 == DIR_DOWNRIGHT || d1 == DIR_DOWNLEFT);
		weight = max(y1 - y0, x1 - x0);
		CV_Assert(weight > 0 && y1 >= y0);
	}

	/*
	Input o, another edgeline
	Input dir, project dir
	inout check_wire, As input,if <0, not care, else it is my d0. As output, 1 means wire, 0 means insu
	Return negative if project interact, positive if project doesn't interact
	*/
	int project_distance(EdgeLine & o, int dir, int & check_wire) {
		Range my, your;
		int a0, a1, b0, b1;
		switch (dir) {
		case DIR_UP:
		case DIR_DOWN:
			a0 = p0.x;
			a1 = p1.x;
			b0 = o.p0.x;
			b1 = o.p1.x;
			break;
		case DIR_LEFT:
		case DIR_RIGHT:
			a0 = p0.y;
			a1 = p1.y;
			b0 = o.p0.y;
			b1 = o.p1.y;
			break;
		case DIR_UPLEFT:
		case DIR_DOWNRIGHT:
			a0 = p0.x - p0.y;
			a1 = p1.x - p1.y;
			b0 = o.p0.x - o.p0.y;
			b1 = o.p1.x - o.p1.y;
			break;
		case DIR_UPRIGHT:
		case DIR_DOWNLEFT:
			a0 = p0.x + p0.y;
			a1 = p1.x + p1.y;
			b0 = o.p0.x + o.p0.y;
			b1 = o.p1.x + o.p1.y;
			break;
		default:
			CV_Assert(0);
		}
		
		my = Range(min(a0, a1), max(a0, a1));
		your = Range(min(b0, b1), max(b0, b1));
		if (check_wire >= 0) {
			switch (check_wire) {
			case DIR_UP:
			case DIR_LEFT:
			case DIR_UPLEFT:
			case DIR_UPRIGHT:
				check_wire = -1;
			case DIR_DOWN:
			case DIR_RIGHT:
			case DIR_DOWNLEFT:
			case DIR_DOWNRIGHT:
				check_wire = 1;
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

	/*
	inout o
	input wire, wire.start means minimum wire, wire.end means max sweet wire length
	*/
	void interact(EdgeLine & o, Range wire, int insu_min) {
		int s = dxy_degree(d0, o.d0);
		int bonus = 0, d, d1, dd, dd1;
		int not_check_wire = -1;
		int is_wire = d0, is_wire1 = o.d0;
		switch (s) {
		case 0:
			if (project_distance(o, d1, not_check_wire) >= wire.start + insu_min)
				return;
			bonus = project_distance(o, d0, not_check_wire);
			if (bonus > 0) //not intersect, don't punish
				return;
			//bonus < 0, punish
			break;
		case 1:
		case 7:
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
			}
			else {
				if (d >= insu_min) //insu wide big enough, do nothing; else insu too narrow, punish
					return;
			}
			break;
		case 2:
		case 6:
			d = project_distance(o, d1, not_check_wire);
			d1 = project_distance(o, o.d1, not_check_wire);
			if (d1 < 0 && d < insu_min || d < 0 && d1 < insu_min) 
				bonus = -min(weight, o.weight);
			else
				return;
			break;
		case 3:
		case 5:
			d = project_distance(o, d1, is_wire);
			d1 = project_distance(o, o.d1, is_wire1);
			dd = project_distance(o, d0, not_check_wire);
			dd1 = project_distance(o, o.d0, not_check_wire);
			if (dd >= insu_min || dd1 >= insu_min)
				return;
			if (dd < 0 && dd1 < 0) { //project intersect				
				if (d1 < d) {
					d1 = d;
					is_wire1 = is_wire;
				} //now d1 is distance
				bonus = max(dd, dd1); //bonus is overlap
				if (is_wire1) {
					if (d1 >= wire.end) //too far away, do nothing
						return;
					if (d1 >= wire.start) //sweet len, give bonus; else do nothing
						bonus = min(-bonus, wire.size());
					else
						return; //how about d1 < insu_min
				}
				else {
					if (d1 >= insu_min)
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
		if (weight > o.weight) {
			weight += bonus / 2;
			o.weight += bonus;
		}
		else {
			weight += bonus;
			o.weight += bonus / 2;
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
	}
};

struct ProcessImageData {
	VWfeature * vwf;
	ProcessData * lpd;
	ProcessData * upd;
	ProcessData * cpd;
	bool multi_thread;
};

static void prob_color(const Mat & prob, Mat & debug_mark)
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
			if (p_prob[x][POINT_IS_INSU] > POINT_TOT_PROB * 0.8)
				p_debugm[x][POINT_IS_INSU] += 20;
			else
			if (p_prob[x][POINT_IS_WIRE] > POINT_TOT_PROB * 0.8)
				p_debugm[x][POINT_IS_WIRE] += 20;
			else
			if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] > POINT_TOT_PROB * 0.8)
				p_debugm[x][POINT_IS_EDGE_WIRE_INSU] += 20;
		}
	}
}
/*
Input prob
Output rst
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
			}
			if (y > EDGE_JUDGE_BORDER + 1 && y < prob.rows - EDGE_JUDGE_BORDER - 2 &&
				x > EDGE_JUDGE_BORDER + 1 && x < prob.cols - EDGE_JUDGE_BORDER - 2) {
				ed_num = 5;
				ed += (p_prob[x + shift[dir][0]][POINT_DIR] == dir) ? p_prob[x + shift[dir][0]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][0]][POINT_IS_EDGE_WIRE_INSU] / 2;
				ed += (p_prob[x + shift[dir][1]][POINT_DIR] == dir) ? p_prob[x + shift[dir][1]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][1]][POINT_IS_EDGE_WIRE_INSU] / 2;
				ed += (p_prob[x + shift[dir][2]][POINT_DIR] == dir) ? p_prob[x + shift[dir][2]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][2]][POINT_IS_EDGE_WIRE_INSU] / 2;
				ed += (p_prob[x + shift[dir][3]][POINT_DIR] == dir) ? p_prob[x + shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] / 2;
			}
			else {
				int dir2 = dir_2[dir];
				int dir3 = dir_1[dir_2[dir]];
				ed_num = 1;
#define IN_RANGE(x, y) (x >= EDGE_JUDGE_BORDER && x < prob.cols - EDGE_JUDGE_BORDER && y >=EDGE_JUDGE_BORDER && y < prob.rows - EDGE_JUDGE_BORDER)
				if (IN_RANGE(x + dxy[dir2][1], y + dxy[dir2][0])) {
					ed += (p_prob[x + shift[dir][0]][POINT_DIR] == dir) ? p_prob[x + shift[dir][0]][POINT_IS_EDGE_WIRE_INSU] :
						p_prob[x + shift[dir][0]][POINT_IS_EDGE_WIRE_INSU] / 2;
					ed_num++;
				}
				if (IN_RANGE(x + dxy[dir2][1] * 2, y + dxy[dir2][0] * 2)) {
					ed += (p_prob[x + shift[dir][1]][POINT_DIR] == dir) ? p_prob[x + shift[dir][1]][POINT_IS_EDGE_WIRE_INSU] :
						p_prob[x + shift[dir][1]][POINT_IS_EDGE_WIRE_INSU] / 2;
					ed_num++;
				}
				if (IN_RANGE(x + dxy[dir3][1], y + dxy[dir3][0])) {
					ed += (p_prob[x + shift[dir][2]][POINT_DIR] == dir) ? p_prob[x + shift[dir][2]][POINT_IS_EDGE_WIRE_INSU] :
						p_prob[x + shift[dir][2]][POINT_IS_EDGE_WIRE_INSU] / 2;
					ed_num++;
				}
				if (IN_RANGE(x + dxy[dir3][1] * 2, y + dxy[dir3][0] * 2)) {
					ed += (p_prob[x + shift[dir][3]][POINT_DIR] == dir) ? p_prob[x + shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] :
					p_prob[x + shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] / 2;
					ed_num++;
				}
#undef IN_RANGE
			}
			p_rst[x][POINT_IS_INSU] = iu * ed_num * POINT_TOT_PROB / ((iu + wi) * ed_num + ed * wi_num);
			p_rst[x][POINT_IS_WIRE] = wi * ed_num * POINT_TOT_PROB / ((iu + wi) * ed_num + ed * wi_num);
			p_rst[x][POINT_IS_EDGE_WIRE_INSU] = POINT_TOT_PROB - p_rst[x][POINT_IS_WIRE] - p_rst[x][POINT_IS_INSU];
			p_rst[x][POINT_DIR] = dir;
		}
	}
}

/*inout prob
input sweet_len, sweet_len range
output edges, 
*/
static void mark_strong_edge(Mat & prob, Range wire_sweet_len, int insu_min, vector<EdgeLine> & edges)
{
	CV_Assert(prob.type() == CV_8UC4);
	int shift[8][5];
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
	shift[DIR_UPRIGHT][1] = (dxy[DIR_LEFT][0] * (int)prob.step.p[0] + dxy[DIR_LEFT][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_UPRIGHT][2] = (dxy[DIR_DOWN][0] * (int)prob.step.p[0] + dxy[DIR_DOWN][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_DOWNRIGHT][1] = (dxy[DIR_LEFT][0] * (int)prob.step.p[0] + dxy[DIR_LEFT][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_DOWNRIGHT][2] = (dxy[DIR_UP][0] * (int)prob.step.p[0] + dxy[DIR_UP][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_DOWNLEFT][1] = (dxy[DIR_RIGHT][0] * (int)prob.step.p[0] + dxy[DIR_RIGHT][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_DOWNLEFT][2] = (dxy[DIR_UP][0] * (int)prob.step.p[0] + dxy[DIR_UP][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_UPLEFT][1] = (dxy[DIR_RIGHT][0] * (int)prob.step.p[0] + dxy[DIR_RIGHT][1] * (int)prob.step.p[1]) / sizeof(Vec4b);
	shift[DIR_UPLEFT][2] = (dxy[DIR_DOWN][0] * (int)prob.step.p[0] + dxy[DIR_DOWN][1] * (int)prob.step.p[1]) / sizeof(Vec4b);

	int th = POINT_TOT_PROB * 0.6;
	Mat m(prob.rows, prob.cols, CV_8U);
	m = 0;
	//1 find local maximum point 
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		uchar * pm = m.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] < th || p_prob[x][POINT_DIR] & 8) //edge factor is low or via case
				continue;
			else 
			if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] == POINT_TOT_PROB) //edge factor is max strong
				pm[x] = 16 | p_prob[x][POINT_DIR];
			else {
				int dir = p_prob[x][POINT_DIR] & 7;
				const Vec4b * p1 = p_prob + x + shift[dir][3];
				uchar edge_grad = p_prob[x][POINT_IS_EDGE_WIRE_INSU];
				if (p1[0][POINT_IS_EDGE_WIRE_INSU] > edge_grad ||
					p1[shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] > edge_grad) //local compare cut
					continue;
				p1 = p_prob + x - shift[dir][3];
				if (p1[0][POINT_IS_EDGE_WIRE_INSU] <= edge_grad &&
					p1[-shift[dir][3]][POINT_IS_EDGE_WIRE_INSU] <= edge_grad) //local compare cut
					pm[x] = 16 | p_prob[x][POINT_DIR]; //edge factor is strong enough
			}
		}
	}

	//local maximum point make up edge line
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const uchar * pm = m.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++)
		if (pm[x]) {
			int dir = pm[x] & 7;
			if (pm[x + shift[dir][0]] == pm[x]) //edge line already found
				continue;
			int s = shift[dir][0];
			int l = 0;
			bool already_found = false;
			while (pm[x + s + shift[dir][1]] == pm[x] || pm[x + s + shift[dir][2]] == pm[x]) {
				s += shift[dir][0];
				l++;
				if (pm[x + s] == pm[x]) {
					already_found = true;
					break;
				}
			}
			if (already_found)
				continue;
			s = -shift[dir][0];
			int r = 0;
			while (pm[x + s] == pm[x] || pm[x + s + shift[dir][1]] == pm[x] || pm[x + s + shift[dir][2]] == pm[x]) {
				s -= shift[dir][0];
				r++;
			}
			if (l + r + 1 < WEAK_EDGE_LEN) //weak edge line, not added
				continue;
			int x0 = dxy[shift[dir][4]][0] * l + x;
			int y0 = dxy[shift[dir][4]][1] * l + y;
			int x1 = -dxy[shift[dir][4]][0] * r + x;
			int y1 = -dxy[shift[dir][4]][1] * r + y;
			edges.push_back(EdgeLine(x0, y0, x1, y1, dir, dir_1[shift[dir][4]]));
		}
	}

	for (int i = 0; i < edges.size(); i++)
	for (int j = i + 1; j < edges.size(); j++)
		edges[i].interact(edges[j], wire_sweet_len, insu_min);
	sort(edges.begin(), edges.end(), greaterEdgeLine);
	
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
	int th = POINT_TOT_PROB * 0.95;
	int th2 = POINT_TOT_PROB * 0.85;
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
			filter_prob(prob, prob1);
			prob_color(prob1, (pi.cpd->edge_mark_debug1 == NULL ? empty : pi.cpd->edge_mark_debug1[0]));
			self_enhance(prob1, prob, 3);
			prob_color(prob, (pi.cpd->edge_mark_debug2 == NULL ? empty : pi.cpd->edge_mark_debug2[0]));
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
	for (int i = 0; i < (int) pd0->eo.size(); i++)
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
	layer_min = -1;
	layer_max = -1;
}

/*    31..24 23..16 15..8 7..0
type
d            clear dmax  dmin
clear
*/
int VWExtractML::set_train_param(int type, int d, int, int, int, int, int, int, int, float)
{
	if ((type & 0xff) == OBJ_POINT) {
		via_diameter_min = d & 0xff;
		via_diameter_max = d >> 8 & 0xff;
		del = d >> 16 & 0xff;
	}
	return 0;
}

int VWExtractML::set_extract_param(int layer, int , int, int, int, int, int, int, int, float)
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
	}
	return 0;
}

Mat VWExtractML::get_mark(int layer)
{
	if (layer < via_mark.size())
		return via_mark[layer];
	return Mat();
}

Mat VWExtractML::get_mark1(int layer)
{
	if (layer < edge_mark.size())
		return edge_mark[layer];
	return Mat();
}

Mat VWExtractML::get_mark2(int layer)
{
	if (layer < edge_mark1.size())
		return edge_mark1[layer];
	return Mat();
}

Mat VWExtractML::get_mark3(int layer)
{
	if (layer < edge_mark2.size())
		return edge_mark2[layer];
	return Mat();
}

int VWExtractML::train(string img_name, vector<MarkObj> & obj_sets)
{
	layer_min = 200, layer_max = 0;
	for (auto & m : obj_sets) {
		layer_min = (layer_min > m.type3) ? m.type3 : layer_min;
		layer_max = (layer_max < m.type3) ? m.type3 : layer_max;
	}
	if (layer_max >= (int)vwf.size()) {
		vwf.resize(layer_max + 1);
		via_mark.resize(layer_max + 1);
		edge_mark.resize(layer_max + 1);
		edge_mark1.resize(layer_max + 1);
		edge_mark2.resize(layer_max + 1);
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
		vwf[current_layer].read_file(project_path, current_layer);
		for (auto & m : obj_sets) 
		if (!del) {
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
				if (m.type2 == POINT_WIRE_INSU || m.type2 == POINT_WIRE || m.type2 == POINT_INSU) {
					Point range = m.type2 == POINT_WIRE_INSU ? Point(2, 2) : Point(1, 1);
					Point loc = Point(m.p0.x(), m.p0.y());
					int label = EDGE_FEATURE;
					label |= (m.type2 == POINT_WIRE_INSU) ? EDGE_IS_WIRE_INSU :
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
			if (m.type == OBJ_POINT && m.type3 == current_layer) { //delete via or no via
				Point loc = Point(m.p0.x(), m.p0.y());
				if (m.type2 == POINT_NO_VIA || m.type2 == POINT_NORMAL_VIA0) // del via or no via
					vwf[current_layer].del_feature(loc, via_diameter_max);
				if (m.type2 == POINT_WIRE_INSU || m.type2 == POINT_WIRE || m.type2 == POINT_INSU)
					vwf[current_layer].del_feature(loc, 2);
			}
		vwf[current_layer].write_file(project_path, current_layer);
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
	}
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		string file_name(img_name);
		file_name[file_name.length() - 5] = current_layer + '0';
		Mat img = imread(file_name, 0);
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		if (!vwf[current_layer].read_file(project_path, current_layer))
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
		ed[current_layer].via_mark_debug = &(via_mark[current_layer]);
		ed[current_layer].edge_mark_debug = &(edge_mark[current_layer]);
		ed[current_layer].edge_mark_debug1 = &(edge_mark1[current_layer]);
		ed[current_layer].edge_mark_debug2 = &(edge_mark2[current_layer]);
		pi.vwf = &vwf[current_layer];
		pi.lpd = NULL;
		pi.upd = NULL;
		pi.cpd = &ed[current_layer];
		pi.multi_thread = false;
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
	layer_min = 200, layer_max = 0;
	for (auto & m : obj_sets) {
		layer_min = (layer_min > m.type3) ? m.type3 : layer_min;
		layer_max = (layer_max < m.type3) ? m.type3 : layer_max;
	}
	int block_width = ics[0]->getBlockWidth();
	int scale = 32768 / block_width;
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		if (current_layer >= (int)vwf.size())
			vwf.resize(current_layer + 1);
		string img_name = ics[current_layer]->get_file_name();
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		
		vwf[current_layer].read_file(project_path, current_layer);
		for (auto & m : obj_sets) 
		if (!del) {				
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
		else {
			if (m.type == OBJ_POINT && m.type3 == current_layer) {
				Point c = Point(m.p0.x() / scale, m.p0.y() / scale);
				vwf[current_layer].del_feature(c, via_diameter_max);
			}
		}
		if (del)
			obj_sets.clear();
		vwf[current_layer].write_file(project_path, current_layer);			
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
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		if (current_layer >= (int)vwf.size())
			vwf.resize(current_layer + 1);
		string img_name = ics[current_layer]->get_file_name();
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer);
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
					qInfo("Poly (%x,%x) (%x,%x)", area_poly[i].x(), area_poly[i].y(), area_poly[i+1].x(), area_poly[i+1].y());
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
				for (int i = 0; i < (int) diag_line[cl].size(); i++) //release current diag_line memory
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
