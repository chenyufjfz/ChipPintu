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
#define ATOM_EDGE_LEN		5
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
#define EDGE_WIRE_TH		50
#define EDGE_INSU_TH		30
#define VIA_REGION_RADIUS   5
#define VIA_BROKE_INSU_LEN	3
#define VIA_CONNECT_METHOD  2

#define COLOR_WIRE			10
#define COLOR_INSU			0
#define COLOR_VIA_WIRE		9
#define COLOR_VIA_INSU		1
#define COLOR_WIRE_PP		8
#define COLOR_WIRE_UNSURE	7
#define COLOR_INSU_UNSURE	3
#define COLOR_INSU_PP		2
#define COLOR_JUDGE			5
#define COLOR_CUT_WIRE      2
#define COLOR_CONNECT_JUDGE 7.5

#define ENCLOSE_UNSURE_TH   13

//CCL_GRID_SIZE must be 2
#define CCL_GRID_SIZE		2
#define CCL_CORNER_FACTOR   0.85
#define CCL_VIA_FACTOR		0.9
#define ADJ_CONNECT_SCORE	100
#define CCL_BORDER_REGION	0xfffffffe
#define CCL_INSU_REGION		0xffffffff
#define CCL_EDGE_MASK0		0x00000000
#define CCL_EDGE_MASK1		0x00004000
#define CCL_EDGE_MASK2		0x40000000
#define CCL_EDGE_MASK3		0x40004000
#define CCL_REGION_MASK		0x3fff3fff
#define SELF_CHECK_MASK		7

#define CGRID_UD_WIRE			1
#define CGRID_LR_WIRE			2
#define CGRID_VIA_WIRE			4
#define CGRID_VIA_CENTER		8
#define CGRID_VIA_REGION		16

#define CCL_MERGE_AREA			100
#define CCL_CUT_AREA			52
#define CCL_CUT_LEN				50

//VALID_EDGE_LEN2 can only be 4 6 8 10
#define VALID_EDGE_LEN2			6
#define XIE_RATIO				0.7f
#define NEAR1_COVER				0.25f
#define MAX_TURN_LEN			12
#define ERASE_EDGE_LEN			12
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
		int a0;
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

typedef unsigned long long RegionID;
#define REGION_EDGE_ATOM_GRID		1
#define REGION_EDGE_TILE_GRID		2
#define REGION_EDGE_TURN_POINT		3
#define REGION_EDGE_CONNECT			4
#define LOCAL_REGION				0
class RegionEdge;

class Region {
public:
	RegionID region_id;
	set<RegionEdge *> edges;
	void release();
	Region(RegionID id) {
		region_id = id;
	}
	Region() {
		region_id = (RegionID)0;
	}
	~Region() {
		release();
	}
	bool has_tile();
};

class RegionEdge {
public:
	Region * region;
	RegionEdge * next;
	RegionEdge * prev;
	int type;
	//Return delete which region
	Region * link(RegionEdge * another) {
		Region * ret;
		
		if (another->region == NULL && this->region != NULL) {			
			RegionEdge * e = another;
			do {
				region->edges.insert(e);
				e->region = region;
				e = e->next;
			} while (e != another);
			ret = NULL;
		}
		else
			if (another->region != NULL && this->region == NULL) {
				RegionEdge * e = this;
				do {
					another->region->edges.insert(e);
					e->region = another->region;					
					e = e->next;
				} while (e != this);
				ret = NULL;
			}
			else
				if (another->region != NULL && this->region != NULL) {
					if (another->region == region)
						ret = NULL;
					else 
					if (another->region->edges.size() > region->edges.size()) {
						ret = region;
						for (auto & e : ret->edges)
							e->region = another->region;
						another->region->edges.insert(ret->edges.begin(), ret->edges.end()); //Warning: use merge can be faster
						ret->edges.clear();
					}
					else  {
						ret = another->region;
						for (auto & e : ret->edges)
							e->region = region;
						region->edges.insert(ret->edges.begin(), ret->edges.end()); //use merge can be faster
						ret->edges.clear();
					}
				}
				else
					ret = NULL;
		another->prev->next = next;
		next->prev = another->prev;
		next = another;
		another->prev = this;
		return ret;
	}
	void unlink() {
		prev->next = next;
		next->prev = prev;
		next = this;
		prev = this;
		region->edges.erase(this);
	}
	/*unlink [this, et] from original region */
	void unlink(RegionEdge * et) {
		prev->next = et->next;
		et->next->prev = prev;
		prev = et;
		et->next = this;
		RegionEdge * e = this;
		Region * r = region;
		do {
			r->edges.erase(e);
			e->region = NULL;
			e = e->next;
		} while (e != this);
	}
};

class RegionEdgeAtom : public RegionEdge { //right hand is region
public:
	vector<Point> pts;
	bool is_global;
	RegionEdgeAtom() {
		next = prev = this;
		region = NULL;
		is_global = false;
		type = REGION_EDGE_ATOM_GRID;
	}
};
class RegionEdgeTile : public RegionEdge { //right hand is region
public:
	Point pt1, pt2;
	int dir;
	RegionEdgeTile * other;
	RegionEdgeTile() {
		next = prev = this;
		region = NULL;
		type = REGION_EDGE_TILE_GRID;
		other = NULL;
	}
};
RegionEdgeTile endtile;

bool Region::has_tile()
{
	for (auto & e : edges)
		if (e->type == REGION_EDGE_TILE_GRID)
			return true;
	return false;
}

class RegionEdgeTurnPoint : public RegionEdge {
public:
	vector<Point> pts;
	vector<uchar> dir_margin; //low 3 bit is dit, 3..5 is len, 6,7 is margin
	RegionEdgeTurnPoint() {
		next = prev = this;
		region = NULL;
		type = REGION_EDGE_TURN_POINT;
	}
};
class RegionEdgeConnect : public RegionEdge {
public:
	vector<Point> pts;
	int start_dir;
	int start_len;
	RegionEdgeConnect() {
		next = prev = this;
		region = NULL;
		type = REGION_EDGE_CONNECT;
	}
};
void Region::release()
{
	for (auto &e : edges) {
		if (e->type == REGION_EDGE_ATOM_GRID)
			delete (RegionEdgeAtom *)e;
		if (e->type == REGION_EDGE_TILE_GRID)
			delete (RegionEdgeTile *)e;

	}
	edges.clear();
}
class RegionSet {
public:
	map<RegionID, Region *> regions;
};
bool greaterEdgeLine(const EdgeLine & a, const EdgeLine & b) { return a.weight > b.weight; }

/*
input ic_layer
input scale, 0,1,2,3...
input rect, non-scale pixel unit
Return raw image
*/
static Mat prepare_raw_img(ICLayerWrInterface * ic_layer, int scale, QRect rect)
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
	Mat adjscore, cgrid;
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

class ProcessImageData {
public:
	VWfeature * vwf;
	ProcessData * lpd;
	ProcessData * upd;
	ProcessData * cpd;
	RegionSet * global_rs;
	RegionSet * global_out;
	Range wire_sweet_len_x, wire_sweet_len_y;
	int via_diameter;
	int insu_min;
	int border_size;
	bool multi_thread;
	ProcessImageData() {
		border_size = 0;
		via_diameter = 0;
		vwf = NULL;
		lpd = upd = cpd = NULL;
	}
};

static void draw_prob(const Mat & prob, Mat & debug_mark, float th = 0.8)
{
	CV_Assert(prob.type() == CV_8UC4 && debug_mark.type() == CV_8UC3);
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		Vec3b * p_debugm = debug_mark.ptr<Vec3b>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++)
			if (p_prob[x][POINT_DIR] & POINT_VIA_CENTER) {//Via center
				p_debugm[x][0] = 0;
				p_debugm[x][1] = 255;
				p_debugm[x][2] = 255;
			}
			else
			if (p_prob[x][POINT_DIR] & POINT_VIA_REGION) {//Via
				p_debugm[x][0] = 255;
				p_debugm[x][1] = 255;
				p_debugm[x][2] = 255;
			}
			else
			{
				if (p_prob[x][POINT_IS_INSU] > POINT_TOT_PROB * th)
					p_debugm[x][POINT_IS_INSU] = p_debugm[x][POINT_IS_INSU] < 235 ? p_debugm[x][POINT_IS_INSU] + 20 : 255;
				else
					if (p_prob[x][POINT_IS_WIRE] > POINT_TOT_PROB * th)
						p_debugm[x][POINT_IS_WIRE] = p_debugm[x][POINT_IS_WIRE] < 255 ? p_debugm[x][POINT_IS_WIRE] + 20 : 255;
					else
						if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] > POINT_TOT_PROB * th)
							p_debugm[x][POINT_IS_EDGE_WIRE_INSU] = p_debugm[x][POINT_IS_EDGE_WIRE_INSU] < 255 ? p_debugm[x][POINT_IS_EDGE_WIRE_INSU] + 20 : 255;
			}
	}
}

static void draw_mark(const Mat & mark, Mat & debug_mark)
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
					p_debugm[x][POINT_IS_INSU] = p_debugm[x][POINT_IS_INSU] < 235 ? p_debugm[x][POINT_IS_INSU] + 20 : 255;
				else
					if (pm[x] & MARK_STRONG_WIRE)
						p_debugm[x][POINT_IS_WIRE] = p_debugm[x][POINT_IS_WIRE] < 255 ? p_debugm[x][POINT_IS_WIRE] + 20 : 255;
					else
						if (pm[x] & MARK_EDGE)
							p_debugm[x][POINT_IS_EDGE_WIRE_INSU] = p_debugm[x][POINT_IS_EDGE_WIRE_INSU] < 255 ? p_debugm[x][POINT_IS_EDGE_WIRE_INSU] + 20 : 255;
			}
	}
}

static void draw_210(const Mat & mark, Mat & debug_mark)
{
	CV_Assert(mark.type() == CV_8UC1 && debug_mark.type() == CV_8UC3);
	for (int y = 0; y < mark.rows; y++) {
		const uchar * pm = mark.ptr<uchar>(y);
		Vec3b * p_debugm = debug_mark.ptr<Vec3b>(y);
		for (int x = 0; x < mark.cols; x++)
			if (pm[x] > 10) {//wrong
				CV_Assert(0);
			}
			else
			{
				switch (pm[x]) {
				case COLOR_INSU:
				case COLOR_INSU_PP:
					p_debugm[x][POINT_IS_INSU] += 20;
					break;
				case COLOR_INSU_UNSURE:
					p_debugm[x][POINT_IS_INSU] += 50;
					break;
				case COLOR_WIRE:
				case COLOR_WIRE_PP:
					p_debugm[x][POINT_IS_WIRE] = (p_debugm[x][POINT_IS_WIRE] < 235) ? p_debugm[x][POINT_IS_WIRE] + 20 : 255;
					break;
				case COLOR_WIRE_UNSURE:
					p_debugm[x][POINT_IS_WIRE] = (p_debugm[x][POINT_IS_WIRE] < 205) ? p_debugm[x][POINT_IS_WIRE] + 50 : 255;
					break;
				case COLOR_VIA_INSU:
					p_debugm[x][POINT_IS_INSU] = 255;
					break;
				case COLOR_VIA_WIRE:
					p_debugm[x][POINT_IS_WIRE] = 255;
					break;
				case COLOR_JUDGE:				
					p_debugm[x][POINT_IS_EDGE_WIRE_INSU] = (p_debugm[x][POINT_IS_EDGE_WIRE_INSU] < 235) ? p_debugm[x][POINT_IS_EDGE_WIRE_INSU] + 20 : 255;
					break;
				}		
						
			}
	}
}

static void draw_ccl(const Mat & ccl, Mat & debug_mark)
{
	CV_Assert(ccl.type() == CV_32SC1 && debug_mark.type() == CV_8UC3);
	for (int y = 0; y < ccl.rows; y++) {
		const int * p_ccl = ccl.ptr<int>(y);
		Vec3b * p_debugm = debug_mark.ptr<Vec3b>(y);
		for (int x = 0; x < ccl.cols; x++) 
		if (p_ccl[x] & CCL_EDGE_MASK3) {
			int c = p_ccl[x];
			c = c ^ 0xffffffff;
			p_debugm[x][0] = (c & 0xf) << 4 | (c & 0xf0) >> 4;
			p_debugm[x][1] = (c & 0xf0000) >> 12;
			p_debugm[x][1] |= (c & 0xf00) >> 8;
			p_debugm[x][2] = (c & 0xf000000) >> 24 | (c & 0xf00000) >> 16;
		}
	}
}

static void draw_region_edge(RegionSet & rs, Mat & debug_mark, Point offset)
{
	static int m[][2] = { 
		{ -1, 0 },
		{ 0, 0 },
		{ 0, -1 },
		{ -1, -1 } };
	for (auto &rs_iter : rs.regions) {
		Region * r = rs_iter.second;
		int c = r->region_id & 0xffffffff;
#if 1
		if (c == 0x01830371) {
			c = c * 2 - c;
		}
#endif
		vector<Point> edge_pts;
		for (auto & e : r->edges) {
#if CCL_GRID_SIZE == 2			
			switch (e->type) {
			case REGION_EDGE_ATOM_GRID:
			{
				RegionEdgeAtom * ea = (RegionEdgeAtom *)e;
				for (int i = 0; i < (int)ea->pts.size() - 1; i++) {
					int dir = get_pts_dir(ea->pts[i], ea->pts[i + 1]);
					CV_Assert(dir < 4);
					Point pt1, pt2;
					pt1 = ea->pts[i] * CCL_GRID_SIZE + Point(m[dir][1], m[dir][0]);;
					pt2 = pt1 + Point(dxy[dir][1], dxy[dir][0]);
					if (edge_pts.empty() || edge_pts.back() != pt1)
						edge_pts.push_back(pt1);
					edge_pts.push_back(pt2);
				}
				break;
			}
			case REGION_EDGE_TILE_GRID:
			{
				RegionEdgeTile * et = (RegionEdgeTile *)e;
				int dir = get_pts_dir(et->pt1, et->pt2);
				Point pt1, pt2;
				pt1 = et->pt1 * CCL_GRID_SIZE + Point(m[dir][1], m[dir][0]);
				pt2 = et->pt2 * CCL_GRID_SIZE + Point(m[dir][1], m[dir][0]);
				vector<Point> pts;
				get_line_pts(pt1, pt2, pts);
				if (edge_pts.empty() || edge_pts.back() != pt1)
					edge_pts.insert(edge_pts.end(), pts.begin(), pts.end());
				else
					edge_pts.insert(edge_pts.end(), pts.begin() + 1, pts.end());
				break;
			}
			case REGION_EDGE_TURN_POINT:
			{
				RegionEdgeTurnPoint * etp = (RegionEdgeTurnPoint *)e;
				int edge_size = (etp->next == etp) ? (int)etp->pts.size() + 1 : (int)etp->pts.size();
				for (int i = 0; i + 1 < edge_size; i++) {
					Point pt2 = (i + 1 == etp->pts.size()) ? etp->pts[0] : etp->pts[i + 1];
					int dir = get_pts_dir(etp->pts[i], pt2);
					CV_Assert(dir < 8);
					vector<Point> pts;					
					get_line_pts(etp->pts[i], pt2, pts);
					if (edge_pts.empty() || edge_pts.back() != etp->pts[i])
						edge_pts.insert(edge_pts.end(), pts.begin(), pts.end());
					else
						edge_pts.insert(edge_pts.end(), pts.begin() + 1, pts.end());
				}				
				break;
			}
			}
#else
#error unsupport CCL_GRID_SIZE
#endif	
		}
		c = c ^ 0xffffffff;
		for (auto & pt : edge_pts) {
			pt += offset;
			Vec3b * p_debugm = debug_mark.ptr<Vec3b>(pt.y, pt.x);
			p_debugm[0][0] = (c & 0xf) << 4 | (c & 0xf0) >> 4;
			p_debugm[0][1] = (c & 0xf0000) >> 12;
			p_debugm[0][1] |= (c & 0xf00) >> 8;
			p_debugm[0][2] = (c & 0xf000000) >> 24 | (c & 0xf00000) >> 16;
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
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		Vec4b * p_rst = rst.ptr<Vec4b>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			int iu = p_prob[x][POINT_IS_INSU];
			int wi = p_prob[x][POINT_IS_WIRE];
			int ed = p_prob[x][POINT_IS_EDGE_WIRE_INSU];
			int dir = p_prob[x][POINT_DIR];
			if (dir & POINT_VIA_REGION) {//Via case, not change
				p_rst[x] = p_prob[x];
				continue;
			}
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
static void mark_strong_edge(const Mat & prob, Range wire_sweet_len, int insu_min, vector<EdgeLine> & edges, Mat & mark, Mat & mark2)
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
	cv::rectangle(mark2, Point(0, 0), Point(EDGE_JUDGE_BORDER - 1, mark2.rows - 1), Scalar::all(8), CV_FILLED);
	cv::rectangle(mark2, Point(0, 0), Point(mark2.cols - 1, EDGE_JUDGE_BORDER - 1), Scalar::all(8), CV_FILLED);
	cv::rectangle(mark2, Point(mark2.cols - EDGE_JUDGE_BORDER, 0), Point(mark2.cols - 1, mark2.rows - 1), Scalar::all(8), CV_FILLED);
	cv::rectangle(mark2, Point(0, mark2.rows - EDGE_JUDGE_BORDER), Point(mark2.cols - 1, mark2.rows - 1), Scalar::all(8), CV_FILLED);
	//1 find local maximum point 
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		uchar * pm = mark2.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] < th2 || p_prob[x][POINT_DIR] & POINT_VIA_REGION) {//edge factor is low or via case
				pm[x] = 8 | p_prob[x][POINT_DIR]; //edge factor is 8 if < th2
				continue; 
			}
			else {
				pm[x] = 16 | p_prob[x][POINT_DIR]; //default edge factor is 16, [th2,100)
				if (p_prob[x][POINT_IS_EDGE_WIRE_INSU] == POINT_TOT_PROB) //edge factor is 128 if prob=100
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
						pm[x] = (p_prob[x][POINT_IS_EDGE_WIRE_INSU] > th1) ? 64 : 32;//edge factor is 64 if prob in (th1,100), else 32 prob in [th2, th1]							 
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
			if (edges_dir[dir][i].id >= WEAK_EDGE_LEN) { //long enough
				edges.push_back(edges_dir[dir][i]);
				total_len += edges.back().id;
			}
		}
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
			wire_len = max(4, wire_len);
			for (int i = 2, j = shift[dir][2] * 2; i < wire_len; i++, j += shift[dir][2])
				pm[j] |= MARK_STRONG_WIRE; //mark strong wire with wire_len

		}
	}

	//5 change prob
	for (int y = EDGE_JUDGE_BORDER; y < prob.rows - EDGE_JUDGE_BORDER; y++) {
		const Vec4b * p_prob = prob.ptr<Vec4b>(y);
		uchar * pm = mark.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			if (p_prob[x][POINT_DIR] & POINT_VIA_REGION)  {//via case
				pm[x] = MARK_VIA;
				continue;
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
			if (p_prob[x][POINT_DIR] & POINT_VIA_REGION) {//can't be affected by others, via case
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
mark wire as 10, insu as 0
*/
static void mark_wvi210(const Mat & img, const Mat & mark, Mat & c210, int via_cut_len, const ProcessImageData & pi)
{
	static const short not_sure_th0[] = { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
		6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 
		6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 66, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, };
	static const short not_sure_th1[] = { 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
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
	float gray_unit, gray_unit_1;
	if (insu_th - wire_th >= 15 || wire_th - insu_th > 82) {
		gray_unit = 5;
		gray_unit_1 = 0.2;
	}
	else
		if (insu_th - wire_th >= 13 || wire_th - insu_th > 75) {
			gray_unit = 4.5;
			gray_unit_1 = 0.222222;
		}
		else
		if (insu_th - wire_th >= 10 || wire_th - insu_th > 67) {
			gray_unit = 4;
			gray_unit_1 = 0.25;
		} 
		else
			if (insu_th - wire_th >= 7 || wire_th - insu_th > 60) {
				gray_unit = 3.5;
				gray_unit_1 = 0.2857;
			}
			else
			if (insu_th - wire_th >= 5 || wire_th - insu_th > 52) {
				gray_unit = 3;
				gray_unit_1 = 0.3333333;
			}
			else 
				if (insu_th - wire_th >= 2 || wire_th - insu_th > 45) {
					gray_unit = 2.5;
					gray_unit_1 = 0.4;
				}
				else {
					gray_unit = 2;
					gray_unit_1 = 0.5;
				}


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
				z = (z - th) * gray_unit_1 + 7;
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
				if (numw > EDGE_WIRE_TH) { //enough wire
					ushort remain = numw / 4; //compute low 25% avg gray
					for (int z = 0; z < 15; z++) {
						statw[z] = statw_integrate.at<ushort>(y1, x1, z) + statw_integrate.at<ushort>(y0, x0, z) -
							statw_integrate.at<ushort>(y1, x0, z) - statw_integrate.at<ushort>(y0, x1, z);
						wire_gray += z * min(remain, statw[z]);
						if (remain <= statw[z])
							break;
						remain -= statw[z];
					}
					statw[15] = numw;
					remain = numw / 4;
					wire_gray = wire_gray / remain; //compute avg
					wire_gray = (wire_gray - 7) * gray_unit + th;
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
				if (numi > EDGE_INSU_TH) { //enough insu
					ushort remain = numi / 4; //compute high 25% avg gray
					for (int z = 14; z >= 0; z--) {
						stati[z] = stati_integrate.at<ushort>(y1, x1, z) + stati_integrate.at<ushort>(y0, x0, z) -
							stati_integrate.at<ushort>(y1, x0, z) - stati_integrate.at<ushort>(y0, x1, z);
						insu_gray += z * min(remain, stati[z]);
						if (remain <= stati[z])
							break;
						remain -= stati[z];
					}
					stati[15] = numi;
					remain = numi / 4;
					insu_gray = insu_gray / remain; //compute avg
					insu_gray = (insu_gray - 7) * gray_unit + th;
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

	//5 judge 0,1
	Mat insu_grid_th, wire_grid_th;
	//expand to same size as image
	resize(insu_grid, insu_grid_th, Size(insu_grid.cols * MARK_210_GRID, insu_grid.rows * MARK_210_GRID));
	resize(wire_grid, wire_grid_th, Size(wire_grid.cols * MARK_210_GRID, wire_grid.rows * MARK_210_GRID));
	for (int y = 0; y < img.rows; y++) {
		const uchar * p_insu_th = insu_grid_th.ptr<uchar>(y);
		const uchar * p_wire_th = wire_grid_th.ptr<uchar>(y);
		const uchar * pimg = img.ptr<uchar>(y);
		uchar * pc = c210.ptr<uchar>(y);
		for (int x = 0; x < img.cols; x++) {
			unsigned char th = (p_insu_th[x] + p_wire_th[x]) / 2;
			if (p_insu_th[x] + 1 >= p_wire_th[x]) { //unsure region
				short a = p_insu_th[x] + 1 - p_wire_th[x];
				CV_Assert(a < sizeof(not_sure_th0) / sizeof(not_sure_th0[0]));
				if (pimg[x] >= p_insu_th[x] + not_sure_th0[a])
					pc[x] = COLOR_WIRE; //Wire
				else
					if (pimg[x] + not_sure_th0[a] <= p_wire_th[x])
						pc[x] = COLOR_INSU; //insu
					else
						pc[x] = (pimg[x] > th) ? COLOR_WIRE_UNSURE : COLOR_INSU_UNSURE;
							
			}
			else {
				short a = p_wire_th[x] - p_insu_th[x] - 1;
				CV_Assert(a < sizeof(not_sure_th1) / sizeof(not_sure_th1[0]));
				if (pimg[x] >= p_wire_th[x] + not_sure_th1[a])
					pc[x] = COLOR_WIRE; //Wire
				else
					if (pimg[x] + not_sure_th1[a] <= p_insu_th[x])
						pc[x] = COLOR_INSU; //insu
					else						
						pc[x] = (pimg[x] > th ? COLOR_WIRE_UNSURE : COLOR_INSU_UNSURE);
			}			
		}
	}

	//6 judge via
	//init circles
	int d = pi.vwf->get_max_d();
	int via_extend_len = via_cut_len + 2;
	vector<vector<Point> > circles;
	for (int i = 0; i <= VIA_REGION_RADIUS * 2; i += 2) {
		float r = (d + i) * 0.5;
		vector<Point> c;
		for (float sita = 0; sita < M_PI * 2; sita += 0.05) {
			Point p0(r * cos(sita), r * sin(sita));
			if (c.empty() || p0 != c.back())
				c.push_back(p0);
		}
		for (int j = 0; j < via_extend_len; j++)
			c.push_back(c[j]);
		circles.push_back(c); //now push nearby circle
		for (int j = 0; j < c.size(); j++)
			c[j] += Point(-1, 0);
		circles.push_back(c);
		for (int j = 0; j < c.size(); j++)
			c[j] += Point(2, 0);
		circles.push_back(c);
		for (int j = 0; j < c.size(); j++)
			c[j] += Point(-1, -1);
		circles.push_back(c);
		for (int j = 0; j < c.size(); j++)
			c[j] += Point(0, 2);
		circles.push_back(c);
	}
	//init via_locs
	vector<Point> via_locs;
	for (int i = 0; i < 3; i++) {
		ProcessData * ppd = (i == 0) ? pi.lpd : (i == 1) ? pi.upd : pi.cpd;
		QPoint tl = QPoint(pi.cpd->img_pixel_x0, pi.cpd->img_pixel_y0);
		if (ppd) {
			for (auto & o : ppd->eo) {
				if (o->type == OBJ_POINT && o->type2 == POINT_VIA_AUTO_EXTRACT){
					QPoint oo = (i == 2) ? o->p0 : o->p0 - tl;
					if (oo.x() >= 0 && oo.y() >= 0)					
						via_locs.push_back(Point(oo.x(), oo.y()));
				}
			}
		}
	}

	//7cut via
	for (auto & v : via_locs) {
		int th = 255; //th is via region threshold
		vector<vector<int> > cgs;
		//first compute th
		for (auto & circle : circles) {
			int th_high = 0, th_low = 0, num = 0;
			//1 compute avg wire th_high and avg insu th_low
			for (int i = 0; i < (int)circle.size() - via_cut_len * 2 - 1; i++) {
				Point ci = v + circle[i];
				if (ci.y < 0 || ci.x < 0 || ci.y >= img.rows || ci.x >= img.cols)
					continue;
				
				th_high += wire_grid_th.at<uchar>(ci);
				th_low += insu_grid_th.at<uchar>(ci);
				num++;
			}
			CV_Assert(num != 0);
			th_high /= num;
			th_low /= num;
			//2 get circle gray
			vector<int> cg; //cg is circle gray
			for (int i = 0; i < (int)circle.size(); i++) {
				Point ci = v + circle[i];
				if (ci.y < 0 || ci.x < 0 || ci.y >= img.rows || ci.x >= img.cols)
					cg.push_back(-1); //out of bound
				else
					cg.push_back(img.at<uchar>(ci));
			}
			CV_Assert(circle.size() == cg.size());
			cgs.push_back(cg);
			//3 find th_high, satisfy consistent Point(> th_high) len >= via_cut_len
			Range c0(-1000, -1000); //c0 is longest consistent Point qujian (>th_high)
			th = min(th, th_high + 10); //init water mark, now th_high is watermark
			for (th_high = th; th_high > th_low; th_high--) { //watermark drop down to satisfy Point(> th_high) len >= via_cut_len
				Range c1(-1000, -1000); //c1.start and c1.end point > th_high, point in [start, end] may have (VIA_BROKE_INSU_LEN -1) < th_high
				int n1 = 0; //n1 is consistent wire num
				for (int i = 0; i < (int)cg.size(); i++) { //search c0 clockwise
					if (cg[i] >= th_high) {
						if (c1.start < 0)
							c1.start = i;
						c1.end = i;
						if (n1 < VIA_BROKE_INSU_LEN)
							n1++;
					}
					else {
						if (n1 > 0) {
							n1--;
							if (n1 == 0) { //wire broken
								if (c1.size() > c0.size())
									c0 = c1;
								c1 = Range(-1000, -1000);
							}
						}
					}
				}
				if (c1.size() > c0.size())
					c0 = c1;
				c1 = Range(-1000, -1000);
				n1 = 0;
				for (int i = (int)cg.size() - 1; i >= 0; i--) { //search c0 anticlockwise
					if (cg[i] >= th_high) {
						if (c1.end < 0)
							c1.end = i;
						c1.start = i;
						if (n1 < VIA_BROKE_INSU_LEN)
							n1++;
					}
					else {
						if (n1 > 0) {
							n1--;
							if (n1 == 0) {
								if (c1.size() > c0.size())
									c0 = c1;
								c1 = Range(-1000, -1000);
							}
						}
					}
				}
				if (c1.size() > c0.size())
					c0 = c1;
				if (c0.size() >= via_cut_len -1)
					break;
			}
			if (c0.size() >= via_cut_len - 1) { //consistent Point(> th_high) len >=via_cut_len
				int th0 = 0, th1 = 0, n0 = 0; //th1 is c0 avg gray, th0 is point (<th_high) avg gray
				for (int i = c0.start; i <= c0.end; i++)
					if (cg[i] >= 0) {
						th1 += cg[i];
						n0++;
					}					
				th1 /= n0;
				n0 = 0;
				for (int i = 0; i < (int)cg.size(); i++)
					if (cg[i] >= 0 && cg[i] < th_high) {
						th0 += cg[i];
						n0++;
					}
				if (n0 >= via_cut_len) {
					th0 /= n0;
					th = min(th, th_high);
					th = min(th, (th0 + th1) / 2);
				}
			}
		}
		//now th is decided, cut via
#if VIA_CONNECT_METHOD == 1
		CV_Assert(cgs.size() == circles.size());
		for (int c = 0; c < (int) cgs.size(); c++) {
			vector<int> & cg = cgs[c];
			vector<Point> & circle = circles[c];
			CV_Assert(cg.size() == circle.size());
			//same as above
			Range c1(-1000, -1000); //c1.start and c1.end point > th_high, point in [start, end] may have (VIA_BROKE_INSU_LEN -1) < th_high
			int n1 = 0;
			vector<int> check(cg.size(), 0);
			for (int i = 0; i < (int)cg.size(); i++) {
				if (cg[i] >= th) {
					if (c1.start < 0)
						c1.start = i;
					c1.end = i;
					if (n1 < VIA_BROKE_INSU_LEN)
						n1++;
				}
				else {
					if (n1 > 0) {
						n1--;
						if (n1 == 0) {
							if (c1.size() >= via_cut_len - 1)
								for (int j = c1.start; j <= c1.end; j++)
									if (cg[j] >= 0) //cg[j]<0 out of boundary
										check[j] = 1;
							c1 = Range(-1000, -1000);
						}
					}
				}
			}
			if (c1.size() >= via_cut_len - 1)
				for (int j = c1.start; j <= c1.end; j++)
					if (cg[j] >= 0)
						check[j] = 1;
			c1 = Range(-1000, -1000);
			n1 = 0;
			for (int i = (int)cg.size() - 1; i >= 0; i--) {
				if (cg[i] >= th) {
					if (c1.end < 0)
						c1.end = i;
					c1.start = i;
					if (n1 < VIA_BROKE_INSU_LEN)
						n1++;
				}
				else {
					if (n1 > 0) {
						n1--;
						if (n1 == 0) {
							if (c1.size() > via_cut_len - 1)
								for (int j = c1.start; j <= c1.end; j++)
									if (cg[j] >= 0)
										check[j] = 1;
							c1 = Range(-1000, -1000);
						}
					}
				}
			}
			if (c1.size() >= via_cut_len - 1)
				for (int j = c1.start; j <= c1.end; j++)
					if (cg[j] >= 0)
						check[j] = 1;
			for (int j = 0; j < via_extend_len; j++)
				check[j] |= check[j + check.size() - via_extend_len]; //via_extend_len is duplicated check, roll it to front
			for (int i = 0; i < (int)check.size() - via_extend_len; i++)
				if (cg[i] >= 0 && c210.at<uchar>(v + circle[i]) != COLOR_VIA_INSU) //in bound
					c210.at<uchar>(v + circle[i]) = check[i] ? COLOR_VIA_WIRE : COLOR_VIA_INSU;
		}
#else
		for (int y = -d / 2 -VIA_REGION_RADIUS; y <= d / 2 + VIA_REGION_RADIUS; y++) {
			if (v.y + y < 0 || v.y + y >= img.rows)
				continue;
			const uchar * pimg = img.ptr<uchar>(v.y + y);
			for (int x = -d / 2 -VIA_REGION_RADIUS; x <= d / 2 + VIA_REGION_RADIUS; x++) {
				int distance = y*y + x*x;
				if (distance >(d / 2 + VIA_REGION_RADIUS) * (d / 2 + VIA_REGION_RADIUS) || distance <= d * d / 4 || v.x + x < 0 || v.x + x >= img.cols)
					continue;
				c210.at<uchar>(v.y + y, v.x + x) = (pimg[v.x + x] >= th) ? COLOR_VIA_WIRE : COLOR_VIA_INSU;
			}
		}
#endif
	}
}

/*
Inout c210
Check WIRE_UNSURE and INSU_UNSURE surround border
*/
static void process_unsure(const Mat & prob, Mat & c210)
{
	CV_Assert(c210.type() == CV_8UC1 && prob.type() == CV_8UC4);
	CV_Assert(c210.rows == prob.rows && c210.cols == prob.cols);
	for (int y = 0; y < c210.rows; y++) {
		uchar * pc = c210.ptr<uchar>(y);
		for (int x = 0; x < c210.cols; x++) 
		if (pc[x] == COLOR_WIRE_UNSURE || pc[x] == COLOR_INSU_UNSURE) {
			int nw = 0, ni = 0;
			pc[x] |= 0x80; //mark it visited
			vector<Point> unsure_queue; //broard first search
			int head = 0;
			unsure_queue.push_back(Point(x, y));
			while (head < (int) unsure_queue.size()) {
				Point o = unsure_queue[head++];
				for (int dir = 0; dir <= 3; dir++) {
					Point o1 = o + Point(dxy[dir][1], dxy[dir][0]);
					if (o1.x < 0 || o1.y < 0 || o1.x >= c210.cols || o1.y >= c210.rows)
						continue;
					uchar * pc1 = c210.ptr<uchar>(o1.y, o1.x);	
					const Vec4b * p_prob = prob.ptr<Vec4b>(o1.y, o1.x); 

					if (p_prob[0][POINT_IS_EDGE_WIRE_INSU] < POINT_TOT_PROB / 3 && //check if meet edge
						(pc1[0] == COLOR_WIRE_UNSURE || pc1[0] == COLOR_INSU_UNSURE)) { //new unsure, push queue
						unsure_queue.push_back(o1);
						pc1[0] |= 0x80; //mark it visited
					}
					else {
						uchar c1 = pc1[0] & 0x7f;
						if (c1 == COLOR_WIRE || c1 == COLOR_INSU || c1 == COLOR_VIA_INSU || c1 == COLOR_VIA_WIRE) { //meet border
							if (c1 > COLOR_JUDGE)
								nw++; //enclosed wire ++
							else
								ni++; //enclosed insu ++
						}
					}
				}
			}
			uchar new_uc = 0;
			if (nw > ENCLOSE_UNSURE_TH * ni)
				new_uc = 0x80 | COLOR_WIRE_PP;
			if (ni > ENCLOSE_UNSURE_TH * nw)
				new_uc = 0x80 | COLOR_INSU_PP;
			if (new_uc)
			for (int i = 0; i < (int)unsure_queue.size(); i++)
				c210.at<uchar>(unsure_queue[i]) = new_uc;
		}
	}

	for (int y = 0; y < c210.rows; y++) {
		uchar * pc = c210.ptr<uchar>(y);
		for (int x = 0; x < c210.cols; x++)
			pc[x] &= 0x7f; //clear mark
	}
}

/*
Input m
output ud, lr, ul, ur, ud is used to colume line sum, lr is used to row line sum
*/
static void integrate_line(const Mat & m, Mat & ud, Mat & lr, Mat & ul, Mat & ur)
{
	CV_Assert(m.type() == CV_8UC1);
	ud.create(m.rows + 1, m.cols, CV_16UC1);
	lr.create(m.rows, m.cols + 1, CV_16UC1);
	ul.create(m.rows + 1, m.cols + 1, CV_16UC1);
	ur.create(m.rows + 1, m.cols + 1, CV_16UC1);

	for (int y = 0; y <= m.rows; y++) {
		ushort * pud = ud.ptr<ushort>(y);
		ushort * pul = ul.ptr<ushort>(y);
		ushort * pur = ur.ptr<ushort>(y);
		if (y == 0) {
			for (int x = 0; x <= m.cols; x++) {
				pud[x] = 0;
				pul[x] = 0;
				pur[x] = 0;
			}
			continue;
		}
		ushort * plr = lr.ptr<ushort>(y - 1);
		ushort * pud1 = ud.ptr<ushort>(y - 1);
		ushort * pul1 = ul.ptr<ushort>(y - 1);
		ushort * pur1 = ur.ptr<ushort>(y - 1);
		const uchar * pc = m.ptr<uchar>(y - 1);
		plr[0] = 0;
		pul[0] = 0;
		pur[m.cols] = 0;
		for (int x = 1; x <= m.cols; x++) {
			plr[x] = plr[x - 1] + pc[x - 1];
			pud[x - 1] = pud1[x - 1] + pc[x - 1];
			pul[x] = pul1[x - 1] + pc[x - 1];
			pur[x - 1] = pur1[x] + pc[x - 1];
		}
	}
}

/*
Input cgrid, adj, adjscore, this is result of compute_adj
Output ccl, ccl region
*/
static void do_ccl(const Mat & cgrid, const Mat & adjscore, Mat & ccl, int merge_area_th, int cut_area_th)
{
	CV_Assert(cgrid.rows == adjscore.rows && cgrid.cols == adjscore.cols);
	merge_area_th /= CCL_GRID_SIZE * CCL_GRID_SIZE;
	cut_area_th /= CCL_GRID_SIZE * CCL_GRID_SIZE;
	
	//now do ccl 1st scan
	ccl.create(cgrid.rows, cgrid.cols, CV_32SC1);
	for (int y = 0; y < ccl.rows; y++) {
		const Vec2b * padjs = adjscore.ptr<Vec2b>(y);
		const uchar * pcg = cgrid.ptr<uchar>(y);
		unsigned * p_ccl = ccl.ptr<unsigned>(y);
		unsigned * p_ccl_1 = y > 0 ? ccl.ptr<unsigned>(y - 1) : NULL;
		unsigned fa, fa0, fa1;
		for (int x = 0; x < ccl.cols; x++) {
			uchar adj = 0;
			if (y!=0 && padjs[x][0] >= ADJ_CONNECT_SCORE)
				adj |= 2;
			if (x!=0 && padjs[x][1] >= ADJ_CONNECT_SCORE)
				adj |= 1;
			if (pcg[x] == 0)
				p_ccl[x] = CCL_INSU_REGION;
			else
				switch (adj) {
				case 0: //solo point
					p_ccl[x] = y << 16 | x;
					break;
				case 1: //left adj
					p_ccl[x] = p_ccl[x - 1];
					break;
				case 2: //up adj
					p_ccl[x] = p_ccl_1[x];
					break;
				case 3: //both left and up adj
					fa = p_ccl[x - 1]; //find left root
					fa0 = ccl.at<int>(fa >> 16, fa & 0xffff);
					while (fa0 != fa) {
						fa = fa0;
						fa0 = ccl.at<int>(fa >> 16, fa & 0xffff);
					}
					fa1 = p_ccl_1[x]; //find up root
					fa0 = ccl.at<int>(fa1 >> 16, fa1 & 0xffff);
					while (fa0 != fa1) {
						fa1 = fa0;
						fa0 = ccl.at<int>(fa1 >> 16, fa1 & 0xffff);
					}
					p_ccl[x] = min(fa, fa1);
					if (fa < fa1) //left ccl < up ccl
						ccl.at<int>(fa1 >> 16, fa1 & 0xffff) = fa;
					if (fa1 < fa) {//up ccl < left ccl
						fa = y << 16 | (x - 1);
						fa0 = p_ccl[x - 1];
						while (fa0 != fa) {
							ccl.at<int>(fa >> 16, fa & 0xffff) = fa1;
							fa = fa0;
							fa0 = ccl.at<int>(fa >> 16, fa & 0xffff);
						}
						ccl.at<int>(fa >> 16, fa & 0xffff) = fa1;
					}
					break;
			}
		}
	}
	//ccl 2nd scan
	Mat area(cgrid.rows, cgrid.cols, CV_32SC1); //region area
	vector<unsigned> region;
	for (int y = 0; y < ccl.rows; y++) {
		int * p_area = area.ptr<int>(y);
		unsigned * p_ccl = ccl.ptr<unsigned>(y);
		for (int x = 0; x < ccl.cols; x++) {
			unsigned fa = p_ccl[x];
			if (fa == CCL_INSU_REGION) { //insu
				p_area[x] = 1000000;
				continue;
			}
			else
				p_area[x] = 0;
			fa = ccl.at<int>(fa >> 16, fa & 0xffff);
			if ((fa >> 16) == y && (fa & 0xffff) == x)
				region.push_back(fa);
			p_ccl[x] = fa;
			fa = ccl.at<int>(fa >> 16, fa & 0xffff);
			CV_Assert(p_ccl[x] == fa);
			area.at<int>(fa >> 16, fa & 0xffff)++;
		}
	}

	if (merge_area_th > 0) {
		for (auto r : region)
			if (area.at<int>(r >> 16, r & 0xffff) < merge_area_th) {
				vector<Point> region_queue; //broad first search
				vector<pair<int, int> > adj_queue; //adj_queue.first is adj region, adj_queue.second is adjscore
				int head = 0;
				region_queue.push_back(Point(r & 0xffff, r >> 16));
				ccl.at<int>(region_queue.back()) = CCL_INSU_REGION;
				bool is_via_region = false;
				while (head < (int)region_queue.size()) {
					Point o = region_queue[head++];
					if (cgrid.at<uchar>(o) & CGRID_VIA_CENTER) {
						is_via_region = true;
						break;
					}
					for (int dir = 0; dir <= 3; dir++) {
						Point o1 = o + Point(dxy[dir][1], dxy[dir][0]);
						if (o1.x < 0 || o1.y < 0 || o1.x >= ccl.cols || o1.y >= ccl.rows)
							continue;
						int * pc = ccl.ptr<int>(o1.y, o1.x);
						if (pc[0] == CCL_INSU_REGION)
							continue;
						else
							if (pc[0] == r) {
								region_queue.push_back(o1);
								pc[0] = CCL_INSU_REGION;
							}
							else {
								switch (dir) {
								case DIR_UP:
									adj_queue.push_back(make_pair(pc[0], adjscore.at<Vec2b>(o)[0]));
									break;
								case DIR_LEFT:
									adj_queue.push_back(make_pair(pc[0], adjscore.at<Vec2b>(o)[1]));
									break;
								case DIR_DOWN:
									adj_queue.push_back(make_pair(pc[0], adjscore.at<Vec2b>(o1)[0]));
									break;
								case DIR_RIGHT:
									adj_queue.push_back(make_pair(pc[0], adjscore.at<Vec2b>(o1)[1]));
									break;
								}
							}
					}
				}
				int max_adj = 0, belong = CCL_INSU_REGION;
				for (auto & a : adj_queue)
					if (max_adj < a.second) { //pick biggest score adj region
						max_adj = a.second;
						belong = a.first;
					}
				if (max_adj > 0 && !is_via_region) { //merge to belong
					area.at<int>(belong >> 16, belong & 0xffff) += (int)region_queue.size();
					for (auto & o : region_queue)
						ccl.at<int>(o) = belong;
				}
				else {
					if (region_queue.size() > cut_area_th)
						for (auto & o : region_queue)
							ccl.at<int>(o) = r; //recover it to r
				}
			}
	}

}

/*
Input c210
input prob, used to judge via center
output cgrid, c210 2*2 grid
output adjscore, cgrid adj score, 0 for upscore, 1 for leftscore
input min_wire_len_x, for up_adj compute
input min_wire_len_y, for left_adj compute
*/
static void compute_adj(const Mat & c210, const Mat & prob, Mat & cgrid, Mat & adjscore, int min_wire_len_x, int min_wire_len_y)
{
	CV_Assert(c210.type() == CV_8UC1 && prob.type() == CV_8UC4);
	CV_Assert(c210.rows == prob.rows && c210.cols == prob.cols);
	CV_Assert(c210.rows % CCL_GRID_SIZE == 0 && c210.cols % CCL_GRID_SIZE == 0);
	Mat ud, lr, ul, ur;
	cgrid.create(c210.rows / CCL_GRID_SIZE, c210.cols / CCL_GRID_SIZE, CV_8UC1); //2*2 grid for wire and insu
	//cgrid[y,x]=0, means insu, =1 means up-down wire, =2 means left-right wire
	Mat clr_sum(cgrid.rows + 1, cgrid.cols + 1, CV_32SC1);
	Mat cud_sum(cgrid.rows + 1, cgrid.cols + 1, CV_32SC1);

	integrate_line(c210, ud, lr, ul, ur);
	for (int y = 0; y < cgrid.rows; y++) {
		uchar * pcg = cgrid.ptr<uchar>(y);
#if CCL_GRID_SIZE == 2
		const uchar * pc0 = c210.ptr<uchar>(CCL_GRID_SIZE * y);
		const Vec4b * pprob = prob.ptr<Vec4b>(CCL_GRID_SIZE * y);
		int y0 = CCL_GRID_SIZE * y + 1;
		const uchar * pc1 = (y0 < c210.rows) ? c210.ptr<uchar>(y0) : pc0;
		const Vec4b * pprob1 = (y0 < c210.rows) ? prob.ptr<Vec4b>(y0) : pprob;
		const ushort * pud0 =(y0 > 100) ? ud.ptr<ushort>(y0 - 100) : ud.ptr<ushort>(0); //up 100 line
		const ushort * pud1 =ud.ptr<ushort>(y0);
		const ushort * pud2 = (y0+ 100 < ud.rows) ? ud.ptr<ushort>(y0 + 100) : ud.ptr<ushort>(ud.rows - 1); //down 100 line
		const ushort * plr0 = lr.ptr<ushort>(y0 - 1);
		const ushort * plr1 = (y0 < c210.rows) ? lr.ptr<ushort>(y0) : plr0;
#else
#error unsupport CCL_GRID_SIZE
#endif
		for (int x = 0; x < cgrid.cols; x++) {
			int sum_grid; //judge grid is insu or wire
			ushort sum_ud = 0, sum_lr = 0; //judge grid dir is heng or shu
#if CCL_GRID_SIZE == 2
			int x0 = CCL_GRID_SIZE * x + 1;
			int is_via_center = 0;
			int is_via_region = 0;
			if (x0 < c210.cols) {
				sum_grid = pc0[x0 - 1] + pc0[x0] + pc1[x0 - 1] + pc1[x0];
				sum_ud = max(pud1[x0] - pud0[x0] + pud1[x0 - 1] - pud0[x0 - 1], pud2[x0] - pud1[x0] + pud2[x0 - 1] - pud1[x0 - 1]);//max(up, down)
				int m = pprob[x0 - 1][POINT_DIR] | pprob[x0][POINT_DIR] | pprob1[x0 - 1][POINT_DIR] | pprob1[x0][POINT_DIR];
				is_via_center = m & POINT_VIA_CENTER;
				is_via_region = m & POINT_VIA_REGION;
			}
			else {
				sum_grid = (pc0[x0 - 1] + pc1[x0 - 1]) *CCL_GRID_SIZE;
				int m = pprob[x0 - 1][POINT_DIR] | pprob1[x0 - 1][POINT_DIR];
				is_via_center = m & POINT_VIA_CENTER;
				is_via_region = m & POINT_VIA_REGION;
				sum_ud = max(pud1[x0 - 1] - pud0[x0 - 1], pud2[x0 - 1] - pud1[x0 - 1]);//max(up, down)
				sum_ud *= CCL_GRID_SIZE;
			}
			sum_lr = (x0 + 100 < lr.cols) ? plr0[x0 + 100] - plr0[x0] + plr1[x0 + 100] - plr1[x0] : plr0[lr.cols - 1] - plr0[x0] + plr1[lr.cols - 1] - plr1[x0];
			sum_lr = max(sum_lr, (ushort)((x0 >= 100) ? plr0[x0] - plr0[x0 - 100] + plr1[x0] - plr1[x0 - 100] : plr0[x0] + plr1[x0])); //max(right, left)
#else
#error unsupport CCL_GRID_SIZE
#endif
			if (is_via_center)
				pcg[x] = CGRID_VIA_CENTER | CGRID_VIA_REGION | CGRID_VIA_WIRE;
			else
			if (sum_grid >= COLOR_VIA_WIRE * 2) { //bigger than wire threshold
				if (pc0[x0 - 1] == COLOR_VIA_WIRE || pc1[x0 - 1] == COLOR_VIA_WIRE ||
					x0 < c210.cols && (pc0[x0] == COLOR_VIA_WIRE || pc1[x0] == COLOR_VIA_WIRE))
					pcg[x] = CGRID_VIA_WIRE; //via
				else
					if (is_via_region)
						pcg[x] = CGRID_VIA_REGION | CGRID_VIA_WIRE;
				else
					pcg[x] = (sum_lr > sum_ud) ? CGRID_LR_WIRE : CGRID_UD_WIRE; //2 means left-right wire, 1 means up-down wire
			}
			else
				pcg[x] = 0; //0 means grid is insu
		}
	}

	//do integral for cgrid left-right and up-down
	for (int y = 0; y < clr_sum.rows; y++) {
		int * p_clr = clr_sum.ptr<int>(y);
		int * p_cud = cud_sum.ptr<int>(y);
		if (y == 0)
			for (int x = 0; x < clr_sum.cols; x++) {
				p_clr[x] = 0;
				p_cud[x] = 0;
			}
		else {
			int * p_clr_1 = clr_sum.ptr<int>(y - 1);
			int * p_cud_1 = cud_sum.ptr<int>(y - 1);
			const unsigned char * pcg = cgrid.ptr<const unsigned char>(y - 1);
			p_clr[0] = 0;
			p_cud[0] = 0;
			unsigned lsum = 0, usum = 0;
			for (int x = 0; x < cgrid.cols; x++) {
				uchar c = pcg[x];
				if (c == 2)
					lsum++;
				if (c == 1)
					usum++;
				p_clr[x + 1] = p_clr_1[x + 1] + lsum;
				p_cud[x + 1] = p_cud_1[x + 1] + usum;
			}
		}
	}

	adjscore.create(cgrid.rows, cgrid.cols, CV_8UC2);
	for (int y = 0; y < adjscore.rows; y++) {
		int y0 = CCL_GRID_SIZE * y + 1;
		Vec2b * padjs = adjscore.ptr<Vec2b>(y);
		uchar * pcg = cgrid.ptr<uchar>(y);
		uchar * pcg_1 = y > 0 ? cgrid.ptr<uchar>(y - 1) : NULL;
#if CCL_GRID_SIZE == 2
		const ushort * plr0 = (y0 > 2) ? lr.ptr<ushort>(y0 - 2) : NULL;
		const ushort * plr1 = lr.ptr<ushort>(y0 - 1);		
#else
#error unsupport CCL_GRID_SIZE
#endif
		for (int x = 0; x < adjscore.cols; x++) {
			int up_adj, left_adj;
			int x0 = CCL_GRID_SIZE * x + 1;
			if (y == 0)
				up_adj = 0;
			else
				if (pcg[x] == 0 && pcg_1[x] == 0) //both are insu
					up_adj = ADJ_CONNECT_SCORE;
				else
					if (pcg[x] == 0 && pcg_1[x] != 0 || pcg[x] != 0 && pcg_1[x] == 0)
						up_adj = 0;
					else { //both are wire
#if CCL_GRID_SIZE == 2
						int slr0 = 0, slr1 = 0, sul0 = 0, sul1 = 0, sur0 = 0, sur1 = 0, s0, s1;
						if (x0 >= min_wire_len_x) {
							slr0 = plr0[x0] - plr0[x0 - min_wire_len_x];
							slr1 = plr1[x0] - plr1[x0 - min_wire_len_x];
						}
						if (x0 + min_wire_len_x < lr.cols) {
							s0 = plr0[x0 + min_wire_len_x] - plr0[x0];
							s1 = plr1[x0 + min_wire_len_x] - plr1[x0];
							if (s0 + s1 > slr0 + slr1) { //slip left-right to get max connect wire
								slr0 = s0; 
								slr1 = s1;
							}
						}
						if (x0 > min_wire_len_x / 2 && x0 + min_wire_len_x / 2 < lr.cols) {
							s0 = plr0[x0 + min_wire_len_x / 2] - plr0[x0 + min_wire_len_x / 2 - min_wire_len_x];
							s1 = plr1[x0 + min_wire_len_x / 2] - plr1[x0 + min_wire_len_x / 2 - min_wire_len_x];
							if (s0 + s1 > slr0 + slr1) { //slip left-right to get max connect wire
								slr0 = s0;
								slr1 = s1;
							}
						}
						if (x0 > min_wire_len_x && y0 > min_wire_len_x) {
							sul0 = ul.at<ushort>(y0 - 1, x0) - ul.at<ushort>(y0 - 1 - min_wire_len_x, x0 - min_wire_len_x);
							sul1 = ul.at<ushort>(y0 - 1, x0 - 1) - ul.at<ushort>(y0 - 1 - min_wire_len_x, x0 - min_wire_len_x - 1);
						}
						if (x0 + min_wire_len_x < ul.cols && y0 + min_wire_len_x <= ul.rows) {
							s0 = ul.at<ushort>(y0 - 1 + min_wire_len_x, x0 + min_wire_len_x) - ul.at<ushort>(y0 - 1, x0);
							s1 = ul.at<ushort>(y0 - 1 + min_wire_len_x, x0 + min_wire_len_x - 1) - ul.at<ushort>(y0 - 1, x0 - 1);
							if (s0 + s1 > sul0 + sul1) {
								sul0 = s0;
								sul1 = s1;
							}
						}
						if (x0 > min_wire_len_x / 2 + 1 && x0 + min_wire_len_x / 2 < ul.cols && y0 > min_wire_len_x / 2 + 1 && y0 + min_wire_len_x / 2 <= ul.rows) {
							s0 = ul.at<ushort>(y0 - 1 + min_wire_len_x / 2, x0 + min_wire_len_x / 2) - ul.at<ushort>(y0 - 1 + min_wire_len_x / 2 - min_wire_len_x, x0 + min_wire_len_x / 2 - min_wire_len_x);
							s1 = ul.at<ushort>(y0 - 1 + min_wire_len_x / 2, x0 + min_wire_len_x / 2 - 1) - ul.at<ushort>(y0 - 1 + min_wire_len_x / 2 - min_wire_len_x, x0 + min_wire_len_x / 2 - min_wire_len_x - 1);
							if (s0 + s1 > sul0 + sul1) {
								sul0 = s0;
								sul1 = s1;
							}
						}
						if (x0 + min_wire_len_x < ur.cols && y0 > min_wire_len_x) {
							sur0 = ur.at<ushort>(y0 - 1, x0) - ur.at<ushort>(y0 - 1 - min_wire_len_x, x0 + min_wire_len_x);
							sur1 = ur.at<ushort>(y0 - 1, x0 - 1) - ur.at<ushort>(y0 - 1 - min_wire_len_x, x0 + min_wire_len_x - 1);
						}
						if (x0 > min_wire_len_x && y0 + min_wire_len_x <= ur.rows) {
							s0 = ur.at<ushort>(y0 - 1 + min_wire_len_x, x0 - min_wire_len_x) - ur.at<ushort>(y0 - 1, x0);
							s1 = ur.at<ushort>(y0 - 1 + min_wire_len_x, x0 - min_wire_len_x - 1) - ur.at<ushort>(y0 - 1, x0 - 1);
							if (s0 + s1 > sur0 + sur1) {
								sur0 = s0;
								sur1 = s1;
							}
						}
						if (x0 > min_wire_len_x / 2 + 1 && x0 + min_wire_len_x / 2 < ur.cols && y0 > min_wire_len_x / 2 + 1 && y0 + min_wire_len_x / 2 <= ur.rows) {
							s0 = ur.at<ushort>(y0 - 1 + min_wire_len_x / 2, x0 + min_wire_len_x / 2 - min_wire_len_x) - ur.at<ushort>(y0 - 1 + min_wire_len_x / 2 - min_wire_len_x, x0 + min_wire_len_x / 2);
							s1 = ur.at<ushort>(y0 - 1 + min_wire_len_x / 2, x0 + min_wire_len_x / 2 - min_wire_len_x - 1) - ur.at<ushort>(y0 - 1 + min_wire_len_x / 2 - min_wire_len_x, x0 + min_wire_len_x / 2 - 1);
							if (s0 + s1 > sur0 + sur1) {
								sur0 = s0;
								sur1 = s1;
							}
						}
						CV_Assert(slr0 >=0 && slr1 >=0 && sul0 >=0 && sul1 >=0 && sur0 >=0 && sur1 >=0);
						int s = min(min(slr0, slr1), min(sul0, sul1));
						s = min(s, min(sur0, sur1));
						float factor = 1;
						if ((pcg[x] & CGRID_VIA_WIRE) == CGRID_VIA_WIRE || (pcg_1[x] & CGRID_VIA_WIRE) == CGRID_VIA_WIRE) { //via case
							if ((pcg[x] & CGRID_VIA_CENTER) == CGRID_VIA_CENTER || (pcg_1[x] & CGRID_VIA_CENTER) == CGRID_VIA_CENTER)
								factor = 0.5;  //weak connection is ok
							else {
								for (int i = 0; i < 2; i++) {
									int y1 = (i==0) ? y - min_wire_len_x * 4 : y;
									int y2 = y1 + min_wire_len_x * 4;
									int x1 = x - min_wire_len_x / 2;
									int x2 = x1 + min_wire_len_x;
									if (y1 < 0) {
										y2 -= y1;
										y1 = 0;
									}
									if (x1 < 0) {
										x2 -= x1;
										x1 = 0;
									}
									if (y2 > adjscore.rows) {
										y1 -= y2 - adjscore.rows;
										y2 = adjscore.rows;
									}
									if (x2 > adjscore.cols) {
										x1 -= x2 - adjscore.cols;
										x2 = adjscore.cols;
									}
									float f = cud_sum.at<int>(y2, x2) - cud_sum.at<int>(y2, x1) - cud_sum.at<int>(y1, x2) + cud_sum.at<int>(y1, x1);
									f /= min_wire_len_x * 3 * min_wire_len_x;
									f = min(f, 1.0f);
									f = 1 - f * 0.5; //0 -> 1, 1 -> 0.5, longer wire means lower threshold
									factor = min(factor, f);
								}
							}
						}
						else
						if (pcg[x] == CGRID_LR_WIRE && pcg_1[x] == CGRID_LR_WIRE) //both are left-right wire
							factor = 0.5; //weak connection is ok
						else
							if (pcg[x] == CGRID_UD_WIRE && pcg_1[x] == CGRID_UD_WIRE) { //both are up-down wire
								int y1 = y - min_wire_len_x * 3; // 6 * 1, updown rect
								int y2 = y1 + min_wire_len_x * 6;
								int x1 = x - min_wire_len_x / 2;
								int x2 = x1 + min_wire_len_x;
								if (y1 < 0) {
									y2 -= y1;
									y1 = 0;
								}
								if (x1 < 0) {
									x2 -= x1;
									x1 = 0;
								}
								if (y2 > adjscore.rows) {
									y1 -= y2 - adjscore.rows;
									y2 = adjscore.rows;
								}
								if (x2 > adjscore.cols) {
									x1 -= x2 - adjscore.cols;
									x2 = adjscore.cols;
								}
								factor = cud_sum.at<int>(y2, x2) - cud_sum.at<int>(y2, x1) - cud_sum.at<int>(y1, x2) + cud_sum.at<int>(y1, x1);
								factor /= min_wire_len_x * 4 * min_wire_len_x; //factor is updown wire is long enough, longer is 1, shorter is 0
								factor = min(factor, 1.0f);
								factor = 1 - factor * 0.5; //0 -> 1, 1 -> 0.5, longer wire means lower threshold
							}
							else
								if (pcg[x] == CGRID_LR_WIRE && pcg_1[x] == CGRID_UD_WIRE || pcg[x] == CGRID_UD_WIRE && pcg_1[x] == CGRID_LR_WIRE) //left-right wire meet updown wire
									factor = CCL_CORNER_FACTOR; //need strong connection
								else
									CV_Assert(0);
						CV_Assert(factor <= 1.0);
						up_adj = (s < COLOR_CONNECT_JUDGE * min_wire_len_x * factor) ? ADJ_CONNECT_SCORE * s / (COLOR_CONNECT_JUDGE * min_wire_len_x) : ADJ_CONNECT_SCORE;
#else
#error unsupport CCL_GRID_SIZE
#endif					
					}

			if (x == 0)
				left_adj = 0;
			else
				if (pcg[x] == 0 && pcg[x - 1] == 0)
					left_adj = ADJ_CONNECT_SCORE;
				else
					if (pcg[x] == 0 && pcg[x - 1] != 0 || pcg[x] != 0 && pcg[x - 1] == 0)
						left_adj = 0;
					else {
#if CCL_GRID_SIZE == 2
						int sud0 = 0, sud1 = 0, sul0 = 0, sul1 = 0, sur0 = 0, sur1 = 0, s0, s1;
						if (y0 >= min_wire_len_y) {
							sud0 = ud.at<ushort>(y0, x0 - 1) - ud.at<ushort>(y0 - min_wire_len_y, x0 - 1);
							sud1 = ud.at<ushort>(y0, x0 - 2) - ud.at<ushort>(y0 - min_wire_len_y, x0 - 2);
						}
						if (y0 + min_wire_len_y < ud.rows) {
							s0 = ud.at<ushort>(y0 + min_wire_len_y, x0 - 1) - ud.at<ushort>(y0, x0 - 1);
							s1 = ud.at<ushort>(y0 + min_wire_len_y, x0 - 2) - ud.at<ushort>(y0, x0 - 2);
							if (s0 + s1 > sud0 + sud1) { //slip up-down to get max connect wire
								sud0 = s0;
								sud1 = s1;
							}
						}
						if (y0 > min_wire_len_y / 2 && y0 + min_wire_len_y / 2 < ud.rows) {
							s0 = ud.at<ushort>(y0 + min_wire_len_y / 2, x0 - 1) - ud.at<ushort>(y0 + min_wire_len_y / 2 - min_wire_len_y, x0 - 1);
							s1 = ud.at<ushort>(y0 + min_wire_len_y / 2, x0 - 2) - ud.at<ushort>(y0 + min_wire_len_y / 2 - min_wire_len_y, x0 - 2);
							if (s0 + s1 > sud0 + sud1) { //slip up-down to get max connect wire
								sud0 = s0;
								sud1 = s1;
							}
						}
						if (x0 > min_wire_len_y && y0 >= min_wire_len_y) {
							sul0 = ul.at<ushort>(y0, x0) - ul.at<ushort>(y0 - min_wire_len_y, x0 - min_wire_len_y);
							sul1 = ul.at<ushort>(y0, x0 - 1) - ul.at<ushort>(y0 - min_wire_len_y, x0 - min_wire_len_y - 1);
						}
						if (x0 + min_wire_len_y < ul.cols && y0 + min_wire_len_y < ul.rows) {
							s0 = ul.at<ushort>(y0 + min_wire_len_y, x0 + min_wire_len_y) - ul.at<ushort>(y0, x0);
							s1 = ul.at<ushort>(y0 + min_wire_len_y, x0 + min_wire_len_y - 1) - ul.at<ushort>(y0, x0 - 1);
							if (s0 + s1 > sul0 + sul1) { 
								sul0 = s0;
								sul1 = s1;
							}							
						}
						if (x0 > min_wire_len_y / 2 + 1 && x0 + min_wire_len_y / 2 < ul.cols && y0 > min_wire_len_y / 2 && y0 + min_wire_len_y / 2 < ul.rows) {
							s0 = ul.at<ushort>(y0 + min_wire_len_y / 2, x0 + min_wire_len_y / 2) - ul.at<ushort>(y0 + min_wire_len_y / 2 - min_wire_len_y, x0 + min_wire_len_y / 2 - min_wire_len_y);
							s1 = ul.at<ushort>(y0 + min_wire_len_y / 2, x0 + min_wire_len_y / 2 - 1) - ul.at<ushort>(y0 + min_wire_len_y / 2 - min_wire_len_y, x0 + min_wire_len_y / 2 - min_wire_len_y - 1);
							if (s0 + s1 > sul0 + sul1) { 
								sul0 = s0;
								sul1 = s1;
							}
						}
						if (x0 + min_wire_len_y < ur.cols && y0 >= min_wire_len_y) {
							sur0 = ur.at<ushort>(y0, x0) - ur.at<ushort>(y0 - min_wire_len_y, x0 + min_wire_len_y);
							sur1 = ur.at<ushort>(y0, x0 - 1) - ur.at<ushort>(y0 - min_wire_len_y, x0 + min_wire_len_y - 1);
						}
						if (x0 > min_wire_len_y && y0 + min_wire_len_y < ur.rows) {
							s0 = ur.at<ushort>(y0 + min_wire_len_y, x0 - min_wire_len_y) - ur.at<ushort>(y0, x0);
							s1 = ur.at<ushort>(y0 + min_wire_len_y, x0 - min_wire_len_y - 1) - ur.at<ushort>(y0, x0 - 1);
							if (s0 + s1 > sur0 + sur1) {
								sur0 = s0;
								sur1 = s1;
							}
						}
						if (x0 > min_wire_len_y / 2 + 1 && x0 + min_wire_len_y / 2 < ur.cols && y0 > min_wire_len_y / 2 && y0 + min_wire_len_y / 2 < ur.rows) {
							s0 = ur.at<ushort>(y0 + min_wire_len_y / 2, x0 + min_wire_len_y / 2 - min_wire_len_y) - ur.at<ushort>(y0 + min_wire_len_y / 2 - min_wire_len_y, x0 + min_wire_len_y / 2);
							s1 = ur.at<ushort>(y0 + min_wire_len_y / 2, x0 + min_wire_len_y / 2 - min_wire_len_y - 1) - ur.at<ushort>(y0 + min_wire_len_y / 2 - min_wire_len_y, x0 + min_wire_len_y / 2 - 1);
							if (s0 + s1 > sur0 + sur1) {
								sur0 = s0;
								sur1 = s1;
							}
						}
						CV_Assert(sud0 >=0 && sud1 >=0 && sul0 >=0 && sul1 >=0 && sur0 >=0 && sur1 >=0);
						int s = min(min(sud0, sud1), min(sul0, sul1));
						s = min(s, min(sur0, sur1));
						float factor = 1;
						if ((pcg[x] & CGRID_VIA_WIRE) == CGRID_VIA_WIRE || (pcg[x - 1] & CGRID_VIA_WIRE) == CGRID_VIA_WIRE) { //via case
							if ((pcg[x] & CGRID_VIA_CENTER) == CGRID_VIA_CENTER || (pcg[x - 1] & CGRID_VIA_CENTER) == CGRID_VIA_CENTER)
								factor = 0.5;  //weak connection is ok
							else {
								for (int i = 0; i < 2; i++) {
									int y1 = y - min_wire_len_y / 2;
									int y2 = y1 + min_wire_len_y;
									int x1 = (i==0) ? x - min_wire_len_y * 4 : x;
									int x2 = x1 + min_wire_len_y * 4;
									if (y1 < 0) {
										y2 -= y1;
										y1 = 0;
									}
									if (x1 < 0) {
										x2 -= x1;
										x1 = 0;
									}
									if (y2 > adjscore.rows) {
										y1 -= y2 - adjscore.rows;
										y2 = adjscore.rows;
									}
									if (x2 > adjscore.cols) {
										x1 -= x2 - adjscore.cols;
										x2 = adjscore.cols;
									}
									float f = clr_sum.at<int>(y2, x2) - clr_sum.at<int>(y2, x1) - clr_sum.at<int>(y1, x2) + clr_sum.at<int>(y1, x1);
									f /= min_wire_len_y * 3 * min_wire_len_y;
									f = min(f, 1.0f);
									f = 1 - f * 0.5; //0 -> 1, 1 -> 0.5, longer wire means lower threshold
									factor = min(factor, f);
								}
							}
						}
						else
						if (pcg[x] == CGRID_UD_WIRE && pcg[x - 1] == CGRID_UD_WIRE) //both are up-down wire
							factor = 0.5;
						else
							if (pcg[x] == CGRID_LR_WIRE && pcg[x - 1] == CGRID_LR_WIRE) { //both are left-right wire
								int y1 = y - min_wire_len_y / 2; // 1 * 6, left-right rect
								int y2 = y1 + min_wire_len_y;
								int x1 = x - min_wire_len_y * 3;
								int x2 = x1 + min_wire_len_y * 6;
								if (y1 < 0) {
									y2 -= y1;
									y1 = 0;
								}
								if (x1 < 0) {
									x2 -= x1;
									x1 = 0;
								}
								if (y2 > adjscore.rows) {
									y1 -= y2 - adjscore.rows;
									y2 = adjscore.rows;
								}
								if (x2 > adjscore.cols) {
									x1 -= x2 - adjscore.cols;
									x2 = adjscore.cols;
								}
								factor = clr_sum.at<int>(y2, x2) - clr_sum.at<int>(y2, x1) - clr_sum.at<int>(y1, x2) + clr_sum.at<int>(y1, x1);
								factor /= min_wire_len_y * 4 * min_wire_len_y;
								factor = min(factor, 1.0f);
								factor = 1 - factor * 0.5; //0 -> 1, 1 -> 0.5
							}
							else 
								if (pcg[x] == CGRID_LR_WIRE && pcg[x - 1] == CGRID_UD_WIRE || pcg[x] == CGRID_UD_WIRE && pcg[x - 1] == CGRID_LR_WIRE)
									factor = CCL_CORNER_FACTOR; //need strong connection
								else
									CV_Assert(0);
						CV_Assert(factor <= 1.0);
						left_adj = (s < COLOR_CONNECT_JUDGE * min_wire_len_y * factor) ? ADJ_CONNECT_SCORE * s / (COLOR_CONNECT_JUDGE * min_wire_len_y) : ADJ_CONNECT_SCORE;
#else
#error unsupport CCL_GRID_SIZE
#endif
					}
			padjs[x][0] = up_adj;
			padjs[x][1] = left_adj;
		}
	}
}

/*
Input ccl_in
Output ccl_out
Add mask 1,2,3
*/
void expand_ccl(const Mat & ccl_in, Mat & ccl_out)
{
	Mat ccl = ccl_in.clone();
	vector<Point> queue; //queue for Mask2 Point
	for (int y = 0; y < ccl.rows; y++) {
		const unsigned * p_cclin = ccl_in.ptr<unsigned>(y);
		const unsigned * p_cclin_1 = y > 0 ? ccl_in.ptr<unsigned>(y - 1) : p_cclin;
		const unsigned * p_cclin1 = y + 1 < ccl.rows ? ccl_in.ptr<unsigned>(y + 1) : p_cclin;
		unsigned * p_ccl = ccl.ptr<unsigned>(y);
		unsigned * p_ccl_1 = (y > 0) ? ccl.ptr<unsigned>(y - 1) : NULL;
		unsigned * p_ccl1 = (y + 1 < ccl.rows) ? ccl.ptr<unsigned>(y + 1) : NULL;
		for (int x = 0; x < ccl.cols; x++)
			if (p_cclin[x] != CCL_INSU_REGION) {
				bool change_to_mask2 = false;
				if (p_cclin1[x] == CCL_INSU_REGION) {
					p_ccl1[x] = p_cclin[x] | CCL_EDGE_MASK2;
					queue.push_back(Point(x, y + 1));
				}
				else
					if (p_cclin1[x] != p_cclin[x]) 
						change_to_mask2 = true;
				if (p_cclin_1[x] == CCL_INSU_REGION) {
					p_ccl_1[x] = p_cclin[x] | CCL_EDGE_MASK2;
					queue.push_back(Point(x, y - 1));
				}
				else
					if (p_cclin_1[x] != p_cclin[x])
						change_to_mask2 = true;
				if (x > 0 && p_cclin[x - 1] == CCL_INSU_REGION) {
					p_ccl[x - 1] = p_cclin[x] | CCL_EDGE_MASK2;
					queue.push_back(Point(x - 1, y));
				}
				else
					if (x > 0 && p_cclin[x - 1] != p_cclin[x]) 
						change_to_mask2 = true;
				if (x + 1 < ccl.cols && p_cclin[x + 1] == CCL_INSU_REGION) {
					p_ccl[x + 1] = p_cclin[x] | CCL_EDGE_MASK2;
					queue.push_back(Point(x + 1, y));
				}
				else
					if (x + 1 < ccl.cols && p_cclin[x + 1] != p_cclin[x])						
						change_to_mask2 = true;
				if (change_to_mask2) {
					p_ccl[x] |= CCL_EDGE_MASK2;
					queue.push_back(Point(x, y));
				}
			}
	}

	for (int i = 0; i < (int)queue.size(); i++) {
		unsigned r = ccl.at<int>(queue[i]) & CCL_REGION_MASK;
		for (int dir = 0; dir <= 3; dir++) {
			int x1 = queue[i].x + dxy[dir][1];
			int y1 = queue[i].y + dxy[dir][0];
			if (x1 < 0 || y1 < 0 || x1 >= ccl.cols || y1 >= ccl.rows)
				continue;
			unsigned r1 = (unsigned) ccl.at<int>(y1, x1);
			if (r1 == CCL_INSU_REGION)
				ccl.at<int>(y1, x1) = r | CCL_EDGE_MASK3; //1 not possible near 3
			else
				if ((r1 & CCL_REGION_MASK) == r) { //0 not possible near 2
					if ((r1 & CCL_EDGE_MASK3) == CCL_EDGE_MASK0)
					ccl.at<int>(y1, x1) |= CCL_EDGE_MASK1;
				}
				else
					if ((r1 & CCL_EDGE_MASK3) < CCL_EDGE_MASK2) { //2 not possible near other region 0,1
						ccl.at<int>(y1, x1) = (r1 & CCL_REGION_MASK) | CCL_EDGE_MASK2;
						queue.push_back(Point(x1, y1));
					}
		}
	}

#if SELF_CHECK_MASK & 1
	for (int y = 0; y < ccl.rows; y++) {
		unsigned * p_ccl = ccl.ptr<unsigned>(y);
		for (int x = 0; x < ccl.cols; x++) 
		if ((p_ccl[x] & CCL_EDGE_MASK3) < CCL_EDGE_MASK3) {
			bool check = true;
			int r = p_ccl[x] & CCL_REGION_MASK;
			for (int dir = 0; dir <= 3; dir++) {
				int x1 = x + dxy[dir][1];
				int y1 = y + dxy[dir][0];
				if (x1 < 0 || y1 < 0 || x1 >= ccl.cols || y1 >= ccl.rows)
					continue;
				int r1 = ccl.at<int>(y1, x1);
				switch (p_ccl[x] & CCL_EDGE_MASK3) {
				case CCL_EDGE_MASK0:
					if (r1 != (r | CCL_EDGE_MASK1) && r1 != (r | CCL_EDGE_MASK0)) //0 can only near 0, 1
						check = false;
					break;
				case CCL_EDGE_MASK1:
					if (r1 != (r | CCL_EDGE_MASK1) && r1 != (r | CCL_EDGE_MASK0) && r1 != (r | CCL_EDGE_MASK2)) //1 can only near 0,1,2
						check = false;
					break;
				case CCL_EDGE_MASK2:
					if ((r1 & CCL_EDGE_MASK3) == CCL_EDGE_MASK0) //2 can only near 1,3
						check = false;
				}
			}
			if (!check) {
				for (int y1 = max(y - 2, 0); y1 <= min(y + 2, ccl.rows - 1); y1++) {
					char s[200];
					int j = 0;
					for (int x1 = max(x - 2, 0); x1 <= min(x + 2, ccl.cols - 1); x1++)
						j += sprintf(s + j, "%8x ", ccl_in.at<int>(y1, x1));
					j += sprintf(s + j, "        ");
					for (int x1 = max(x - 2, 0); x1 <= min(x + 2, ccl.cols - 1); x1++)
						j += sprintf(s + j, "%8x ", ccl.at<int>(y1, x1));
					qWarning(s);
				}
				CV_Assert(0);
			}
		}
	}
#endif

	ccl_out = ccl;
}

/*
Input ccl
Input ea
Input pt_idx
Input dir
*/
static void line_range(const Mat & ccl, RegionEdgeAtom * ea, int pt_idx, int dir)
{
	CV_Assert(pt_idx < ea->pts.size() && dir < 8 && ea->type == REGION_EDGE_ATOM_GRID);
	int best_cover[3] = { pt_idx, pt_idx, pt_idx };
	int max_cover[3] = { pt_idx, pt_idx, pt_idx };
	int err[3];
	int acc_err[3] = { 0, 0, 0 };
	Point pt[3] = { ea->pts[pt_idx] * CCL_GRID_SIZE, ea->pts[pt_idx] * CCL_GRID_SIZE, ea->pts[pt_idx] * CCL_GRID_SIZE };

	for (int i = pt_idx + 1; i < ea->pts.size(); i++) {

	}
}
static const int move_decision[4][3] = { //it is offset between line crosspoint location and ccl grid location 
	//1st choice, 2nd choice
	{ DIR_UP, DIR_UPLEFT, 8 }, //8 means offset is 0
	{ 8, DIR_UP, DIR_LEFT },
	{ DIR_LEFT, 8, DIR_UPLEFT },
	{ DIR_UPLEFT, DIR_LEFT, DIR_UP }
};
/*
inout ccl
output rs, contain atom edge and tile edge
input border size
*/
void mark_atom_edge(Mat & ccl, RegionSet & rs, int border, int cut_len)
{
	CV_Assert(border >= 1);
	cut_len = cut_len / CCL_GRID_SIZE;
	expand_ccl(ccl, ccl);
	Mat visit(ccl.rows, ccl.cols, CV_8UC1);
	visit = 0;
	//now mark between 1 and 2
	for (int y = border; y < ccl.rows - border; y++) {
		const unsigned * p_ccl = ccl.ptr<unsigned>(y);
		uchar * pv = visit.ptr<uchar>(y);
		for (int x = border; x < ccl.cols - border; x++)
		if ((p_ccl[x] & CCL_EDGE_MASK3) == CCL_EDGE_MASK1 && pv[x] == 0) { //border may be 0,1,2,3; 0,1,2; 1,2,3; 1,2; 
			unsigned r = p_ccl[x] & CCL_REGION_MASK;
			Point org(-1, -1), cur(-1, -1);
			int m_dir = -1;
			for (int dir = 0; dir <= 3; dir++) {
				int x1 = x + dxy[dir][1];
				int y1 = y + dxy[dir][0];
				unsigned r1 = (unsigned)ccl.at<int>(y1, x1);
				if (r1 == (r | CCL_EDGE_MASK2)) { //search edge 2
					switch (dir) {
					case DIR_UP:
						org = Point(x, y);
						m_dir = DIR_RIGHT;
						break;
					case DIR_RIGHT:
						org = Point(x + 1, y);
						m_dir = DIR_DOWN;
						break;
					case DIR_DOWN:
						org = Point(x + 1, y + 1);
						m_dir = DIR_LEFT;
						break;
					case DIR_LEFT:
						org = Point(x, y + 1);
						m_dir = DIR_UP;
						break;
					}
					break;
				}
			}
			CV_Assert(m_dir >= 0); //1 must have 2 nearby
			pv[x] = 1;
			cur = org;
			RegionEdgeAtom * cur_atom_edge = new RegionEdgeAtom(); //create new atom edge
			RegionEdgeAtom * start_atom_edge = cur_atom_edge;
			auto it = rs.regions.find(r);
			if (it == rs.regions.end()) {
				Region * new_region = new Region(r);
				rs.regions[r] = new_region;
				it = rs.regions.find(r);
			}
			cur_atom_edge->region = it->second;
			it->second->edges.insert(cur_atom_edge);
			do {
				cur_atom_edge->pts.push_back(cur);
				cur += Point(dxy[m_dir][1], dxy[m_dir][0]);
				if (cur.x < border || cur.y < border || cur.x > ccl.cols - border || cur.y > ccl.rows - border) { //out of Tile
					Point l3 = cur;
					if (move_decision[m_dir][2] < 8)
						l3 += Point(dxy[move_decision[m_dir][2]][1], dxy[move_decision[m_dir][2]][0]);
					unsigned r3 = (unsigned)ccl.at<int>(l3);
					CV_Assert(r3 == (r | CCL_EDGE_MASK1));
					RegionEdge * prev_edge = cur_atom_edge;
					cur -= Point(dxy[m_dir][1], dxy[m_dir][0]); //back to border
					CV_Assert(cur.x == border || cur.y == border || cur.x == ccl.cols - border || cur.y == ccl.rows - border);
					for (int tile_border = 0; tile_border <= 3; tile_border++) {
						RegionEdgeTile * cur_tile_edge = new RegionEdgeTile();						
						cur_tile_edge->pt1 = cur; //tile edge's pt1 overlap with prev atomedge's back
						prev_edge->link(cur_tile_edge);
						m_dir = dir_2[m_dir];
						do {
							cur += Point(dxy[m_dir][1], dxy[m_dir][0]);							
							l3 += Point(dxy[m_dir][1], dxy[m_dir][0]);
							r3 = (unsigned)ccl.at<int>(l3);
							CV_Assert((r3 & CCL_REGION_MASK) == r && (r3 & CCL_EDGE_MASK3) != CCL_EDGE_MASK3);
							if (cur.x < border || cur.y < border || cur.x > ccl.cols - border || cur.y > ccl.rows - border)
								break;
						} while (r3 != (r | CCL_EDGE_MASK2));
						if (cur.x < border || cur.y < border || cur.x > ccl.cols - border || cur.y > ccl.rows - border) { //out of Tile
							l3 = cur;
							if (move_decision[m_dir][2] < 8)
								l3 += Point(dxy[move_decision[m_dir][2]][1], dxy[move_decision[m_dir][2]][0]);
							r3 = (unsigned)ccl.at<int>(l3);
							cur -= Point(dxy[m_dir][1], dxy[m_dir][0]); //back to corner
							CV_Assert(cur.x == border && cur.y == border ||
								cur.x == border && cur.y == ccl.rows - border || 
								cur.x == ccl.cols - border && cur.y == border || 
								cur.x == ccl.cols - border && cur.y == ccl.rows - border);
							if (r3 != (r | CCL_EDGE_MASK2)) {
								if (cur_tile_edge->pt1 == cur) {
									cur_tile_edge->region->edges.erase(cur_tile_edge);
									delete cur_tile_edge;
								}
								else {
									cur_tile_edge->pt2 = cur;
									if (cur.x == border && cur_tile_edge->pt1.x == border)
										cur_tile_edge->dir = DIR_LEFT;
									else
										if (cur.x == ccl.cols - border && cur_tile_edge->pt1.x == ccl.cols - border)
											cur_tile_edge->dir = DIR_RIGHT;
										else
											if (cur.y == border && cur_tile_edge->pt1.y == border)
												cur_tile_edge->dir = DIR_UP;
											else
												if (cur.y == ccl.rows - border && cur_tile_edge->pt1.y == ccl.rows - border)
													cur_tile_edge->dir = DIR_DOWN;
												else
													CV_Assert(0);
									prev_edge = cur_tile_edge;
								}
								continue; //need to check next border 
							}
							else
								m_dir = dir_1[m_dir];							
						}
						else
							m_dir = dir_2[m_dir];
						cur_atom_edge = new RegionEdgeAtom();
						cur_tile_edge->link(cur_atom_edge);
						cur_tile_edge->pt2 = cur;
						if (cur.x == border && cur_tile_edge->pt1.x == border)
							cur_tile_edge->dir = DIR_LEFT;
						else
							if (cur.x == ccl.cols - border && cur_tile_edge->pt1.x == ccl.cols - border)
								cur_tile_edge->dir = DIR_RIGHT;
							else
								if (cur.y == border && cur_tile_edge->pt1.y == border)
									cur_tile_edge->dir = DIR_UP;
								else
									if (cur.y == ccl.rows - border && cur_tile_edge->pt1.y == ccl.rows - border)
										cur_tile_edge->dir = DIR_DOWN;
									else
										CV_Assert(0);
						break;
					}
				}
				Point l1 = cur;
				if (move_decision[m_dir][0] < 8)
					l1 += Point(dxy[move_decision[m_dir][0]][1], dxy[move_decision[m_dir][0]][0]);
				unsigned r1 = (unsigned) ccl.at<int>(l1);
				if (r1 == (r | CCL_EDGE_MASK2))		//_ 2 x
					m_dir = dir_2[m_dir];			//  1 2(r1)
				else {
					CV_Assert((r1 & CCL_EDGE_MASK3) < CCL_EDGE_MASK2 && (r1 & CCL_REGION_MASK) == r);
					Point l2 = cur;
					if (move_decision[m_dir][1] < 8)
						l2 += Point(dxy[move_decision[m_dir][1]][1], dxy[move_decision[m_dir][1]][0]);
					unsigned r2 = (unsigned)ccl.at<int>(l2);
					if (r2 == (r | CCL_EDGE_MASK2)) {
						CV_Assert((r1 & CCL_EDGE_MASK3) == CCL_EDGE_MASK1); //_ 2 2(r2)
						visit.at<uchar>(l1) = 1;							//  1 1(r1)
						m_dir = m_dir; //not change
					}
					else {
						CV_Assert(r2 == (r | CCL_EDGE_MASK1));	//_ 2 1(r2)
						visit.at<uchar>(l1) = 1;				//  1 1(r1)
						visit.at<uchar>(l2) = 1;
						m_dir = dir_1[dir_2[m_dir]];
					}
				}
			}while (cur != org);
			
			if (start_atom_edge != cur_atom_edge) {
				start_atom_edge->pts.insert(start_atom_edge->pts.begin(), cur_atom_edge->pts.begin(), cur_atom_edge->pts.end());
				cur_atom_edge->unlink();
				start_atom_edge->region->edges.erase(cur_atom_edge);
				delete cur_atom_edge;
			}
			else {
				CV_Assert(cur_atom_edge->next == cur_atom_edge && cur_atom_edge->prev == cur_atom_edge);
				if (cur_atom_edge->pts.size() < cut_len) {
					cur_atom_edge->region->edges.erase(cur_atom_edge);
					delete cur_atom_edge;
				}
			}
			
		}
	}
}

/*
Input ccl
inout rs
*/
struct PointEdgeLen {
	uchar dir0, dir1, cover, in_via;
	float len0;	
	float len1;
	float score;
	pair<Point, Point> p0; //it is | - [start, end] point
	pair<Point, Point> p1; //it is / \ [start, end] point
	PointEdgeLen() {
		len0 = -1;
		len1 = -1;
	}
};
struct RangeF {
	float start, end;
	RangeF() {
		start = 0;
		end = 0;
	}
	RangeF(float _start, float _end) {
		start = _start;
		end = _end;
	}
};
struct TurnPoint {
	RangeF cover; //cover.[Begin, end) 's cover =1, if tp[i].end == tp[i+1].start, means connected
	Point p1, p2; //It is pixel location, not need * CCL_GRID_SIZE
	int dir;
};

bool great_edgelen(const PointEdgeLen * a, const PointEdgeLen * b) {
	return a->score > b->score;
}

/*
Input ccl
Inout region_map
Input p0, p1
Input r
Return true if line [p0, p1] is not in other region
*/
static bool check_region(const Mat & region_map, Point p0, Point p1, vector <Point> & opts, int r0) {
	if (p0 == p1)
		return true;
	opts.clear();
	vector <Point> pts;
	get_line_pts(p0, p1, pts);
	pts.push_back(p1);
	bool check = true;
	for (int i = 0; i < pts.size(); i+=2) {
		Point p(pts[i].x / CCL_GRID_SIZE, pts[i].y / CCL_GRID_SIZE);
		if (p.x < 0 || p.y < 0 || p.x >= region_map.cols || p.y >= region_map.rows)
			continue;
		int r = region_map.at<int>(p);
		if (r != 0 && r != r0)
			check = false;
		if (!check)
			break;
		opts.push_back(p);
	}
	return check;
}
/*
Input cgrid, used to check via
Input rs, region atom edge set
Inout global_rs, global reserved edge set
Out global_out, global output edge set
Input org, left-up gloabl pixel location
Input pre_region
Input border
*/
static void process_atom_edge(const Mat & cgrid, RegionSet & rs, RegionSet * global_rs, RegionSet * global_out, Point org, RegionID pre_region, int border, int gen_border)
{
#define HAVE_CONNECT  1
#define NEED_CONNECT  2
	Mat region_map(cgrid.rows + 1, cgrid.cols + 1, CV_32SC1);
	region_map = 0;
	set<RegionEdge *> local_atom_edges; //contain atom edge in rx
	set<RegionEdge *> local_tile_edges; //contain tile edge in rx
	vector<pair<RegionID, RegionEdgeTurnPoint *> > local_turn_edges;
	Rect border_rect(border - 1, border - 1, cgrid.cols * CCL_GRID_SIZE - border + 1, cgrid.rows * CCL_GRID_SIZE - border + 1);
	for (auto &rs_iter : rs.regions) { //loop every region
		Region * r = rs_iter.second;
		r->region_id |= pre_region << 32;
		for (auto & e : r->edges)
			if (e->type == REGION_EDGE_ATOM_GRID)
				local_atom_edges.insert(e);
			else
				if (e->type == REGION_EDGE_TILE_GRID) {
					local_tile_edges.insert(e);
					RegionEdgeTile * e_tile = (RegionEdgeTile *)e;
					if (e_tile->other == NULL && (gen_border & 1 << e_tile->dir))
						e_tile->other = &endtile;
				}
				else
					CV_Assert(0);
	}
	while (!local_atom_edges.empty()) { //loop every edge
#if CCL_GRID_SIZE == 2
		//1 search head and tail
		RegionEdgeAtom * e_atom = (RegionEdgeAtom *)(*local_atom_edges.begin());
		RegionID global_region_id = e_atom->region->region_id;
		int region_id = global_region_id & CCL_REGION_MASK;
		RegionEdge * eh = e_atom; //search head
		RegionEdge * et;
		vector<RegionEdge *> edge_queue;
		int head_type = 0, tail_type = 0;				
		edge_queue.push_back(eh);
		while (1) { //search tile
			if (eh->prev == e_atom) {//circle back
				CV_Assert(head_type == 0);
				break;
			}
			if (eh->prev->type == REGION_EDGE_ATOM_GRID) {
				eh = eh->prev;						
				edge_queue.insert(edge_queue.begin(), eh);
				continue;
			}
			if (eh->prev->type == REGION_EDGE_TILE_GRID) {
				RegionEdgeTile * tt = (RegionEdgeTile *)eh->prev;
				if (tt->other == NULL) {
					head_type = NEED_CONNECT;
					break;
				}
				if (tt->other == &endtile) { //reach end
					eh = tt;								
					edge_queue.insert(edge_queue.begin(), eh);
				}
				else {
					Region * ret = tt->other->prev->link(eh); //link to neighbour tile's region
					if (ret) {
						global_rs->regions.erase(ret->region_id);
						delete ret;
					}
				}
				continue;
			}
			if (eh->prev->type == REGION_EDGE_CONNECT) { //reach another side turn edge
				eh = eh->prev;
				CV_Assert((eh->region->region_id >> 32) != LOCAL_REGION);
				edge_queue.insert(edge_queue.begin(), eh);
				head_type = HAVE_CONNECT;
				break;
			}
			CV_Assert(0);
		}
		if (head_type) {
			et = eh; //search tail
			while (1) { //search tile
				CV_Assert(et->next != e_atom);
				if (et->next->type == REGION_EDGE_ATOM_GRID) {
					et = et->next;								
					edge_queue.push_back(et);
					continue;
				}
				if (et->next->type == REGION_EDGE_TILE_GRID) {
					RegionEdgeTile * tt = (RegionEdgeTile *)et->next;
					if (tt->other == NULL) {
						tail_type = NEED_CONNECT;
						break;
					}
					if (tt->other == &endtile) { //reach end
						et = tt;									
						edge_queue.push_back(et);
					}
					else {
						Region * ret = et->link(tt->other->next); //link to neighbour tile's region
						if (ret) {
							global_rs->regions.erase(ret->region_id);
							delete ret;
						}
					}
					continue;
				}
				if (et->next->type == REGION_EDGE_CONNECT) {  //reach another side turn edge
					et = et->next;
					CV_Assert((eh->region->region_id >> 32) != LOCAL_REGION);								
					edge_queue.push_back(et);
					tail_type = HAVE_CONNECT;
					break;
				}
				CV_Assert(0);
			}
		}
		vector<Point> pts;
		for (auto & e : edge_queue) {
			vector<Point> * ppts;
			vector<Point> temp;
			bool is_global = false;
			if (e->type == REGION_EDGE_ATOM_GRID) {
				RegionEdgeAtom * ea = (RegionEdgeAtom *)e;
				ppts = &(ea->pts);
				is_global = ea->is_global;							
			}
			if (e->type == REGION_EDGE_CONNECT) {
				RegionEdgeConnect * ec = (RegionEdgeConnect *)e;
				is_global = true;
				ppts = &(ec->pts);
			}
			if (e->type == REGION_EDGE_TILE_GRID) {
				RegionEdgeTile * etile = (RegionEdgeTile *)e;
				get_line_pts(etile->pt1, etile->pt2, temp);
				temp.push_back(etile->pt2);
				ppts = &temp;
			}
			if (is_global) { //change global to local
				temp = *ppts;
				for (auto & pt : temp)
					pt -= org;
				ppts = &temp;
			}
			if (pts.empty()) {
				pts.insert(pts.end(), ppts->begin(), ppts->end());
				continue;
			}
			bool finish = false;
			while (!finish && !pts.empty()) { //search ppts->at(i) same as pts.back(), so pts.back() and ppts->at(i+1) distance is 1.
				for (int i = 0; i + 1 < (int)ppts->size() && i < 20; i++)
					if (pts.back() == ppts->at(i)) {
						finish = true;
						pts.insert(pts.end(), ppts->begin() + i + 1, ppts->end());
						break;
					}
				if (!finish)
					pts.pop_back();
			}
			CV_Assert(finish);
		}
		if (head_type == 0 && pts.size() > 1 && pts.back() == pts[0])
			pts.pop_back(); //remove loop circle same point
		
		//now pts is made completely
		for (int i = 1; i < pts.size(); i++) //debug check
			if (abs(pts[i].x - pts[i - 1].x) + abs(pts[i].y - pts[i - 1].y) != 1)
				qFatal("p[i]=(%d,%d), p[i-1]=(%d,%d)", pts[i].x, pts[i].y, pts[i - 1].x, pts[i - 1].y);
#if 1
		if (pts[0] == Point(942, 408))
			pts[0].x = 942;
		if (pts[0] == Point(142, 207))
			pts[0].x = 142;
#endif
		vector<TurnPoint> tp;
		if (head_type == HAVE_CONNECT) {
			TurnPoint s;
			RegionEdgeConnect *ec = (RegionEdgeConnect *)eh;
			s.cover = RangeF(-1, 0);
			s.dir = ec->start_dir;
			s.p2 = ec->pts[0] - org; //this is another tile's tail
			s.p1 = s.p2 - ec->start_len * Point(dxy[s.dir][1], dxy[s.dir][0]);
			vector<Point> pts0;
			if (check_region(region_map, s.p1, s.p2, pts0, region_id))
				for (auto & p : pts0)
					region_map.at<int>(p) = region_id; //mark region map with region_id
			tp.push_back(s);
		}
		if (tail_type == HAVE_CONNECT) {
			TurnPoint s;
			RegionEdgeConnect *ec = (RegionEdgeConnect *)eh;
			s.cover = RangeF(pts.size() - 1, pts.size());
			s.dir = ec->start_dir;
			s.p1 = ec->pts.back() - org; //this is another tile's head
			s.p2 = s.p1 + ec->start_len * Point(dxy[s.dir][1], dxy[s.dir][0]);
			vector<Point> pts0;
			if (check_region(region_map, s.p1, s.p2, pts0, region_id))
				for (auto & p : pts0)
					region_map.at<int>(p) = region_id; //mark region map with region_id
			tp.push_back(s);
		}
		RegionEdge * nh;
		if (head_type == NEED_CONNECT && tail_type == NEED_CONNECT && pts.size() < border) {
			RegionEdgeAtom * na = new RegionEdgeAtom();
			na->is_global = true;
			for (int i = 0; i < pts.size(); i++)
				na->pts.push_back(pts[i] + org);
			qDebug("Push Atom edge to global (%d,%d)",na->pts[0].x, na->pts[0].y);
			nh = na;
		}
		else {
			int pts_len = pts.size();
			vector<PointEdgeLen> el;
			if (head_type == 0) { //double pts			
				pts.resize(pts_len * 2 - 1);
				el.resize(pts_len);
				for (int i = 0; i < pts_len - 1; i++)
					pts[pts_len + i] = pts[i];
			}
			else
				el.resize(pts_len - 1);
			//2 compute max line len for each point, long line line can form tp				
			vector<PointEdgeLen *> pel(el.size());
			for (int i = 0; i < (int)el.size(); i++)
				el[i].dir0 = get_pts_dir(pts[i], pts[i + 1]);
			for (int i = 0; i < (int)el.size(); i++) 	//compute max extend - | line for every point
			if (el[i].len0 <= 0) {	
				int dir = el[i].dir0;
				Point dv(dxy[dir][1], dxy[dir][0]); //dir vector
				Point dh(dxy[dir_2[dir]][1], dxy[dir_2[dir]][0]);							
				int l0_cover = 0, l1_cover = 0, l_1_cover = 0; //max cover pts idx
				Point p0 = pts[i], p1 = pts[i] + dh, p_1 = pts[i] - dh; //distance dh.dot(p1-p0)=1, dh.dot(p_1-p0)=-1
				Point q0, q1, q_1;
				int out0 = 0, out1 = 0, out_1 = 0; //out point num
				for (int j = 0; i + j < (int)pts.size() && j < pts_len / 2; j++) {
					int h_distance = dh.dot(pts[i + j] - p0);//pt2line_distance(pts[i + j], pts[i], dir);
					int v_distance = dv.dot(pts[i + j] - p0);
					bool finish = true;
					if (out0 < ERASE_EDGE_LEN) {						
						int r = region_map.at<int>(p0 + v_distance * dv);
						if (r != 0 && r != region_id)
							out0 = ERASE_EDGE_LEN;
						else
							if (abs(h_distance) <= 1) {	//within distance, clear out0, extend cover				
								out0 = 0;
								l0_cover = j;
								q0 = p0 + v_distance * dv;
							}
							else
								out0++; //out distance, increate out num
						finish = false;
					}
					if (out1 < ERASE_EDGE_LEN) {
						int r = region_map.at<int>(p1 + v_distance * dv);
						if (r != 0 && r != region_id)
							out1 = ERASE_EDGE_LEN;
						else
							if (abs(h_distance - 1) <= 1) {
								out1 = 0;
								l1_cover = j;
								q1 = p1 + v_distance * dv;
							}
							else
								out1++;
						finish = false;
					}
					if (out_1 < ERASE_EDGE_LEN) {
						int r = region_map.at<int>(p_1 + v_distance * dv);
						if (r != 0 && r != region_id)
							out_1 = ERASE_EDGE_LEN;
						else
							if (abs(h_distance + 1) <= 1) {
								out_1 = 0;
								l_1_cover = j;
								q_1 = p_1 + v_distance * dv;
							}
							else
								out_1++;
						finish = false;
					}					
					if (finish)
						break;
				}
				if (l0_cover >= l1_cover && l0_cover >= l_1_cover) {
					el[i].len0 = l0_cover;
					el[i].p0 = make_pair(p0, q0);
				}
				else
					if (l1_cover > l_1_cover) {
						el[i].len0 = l1_cover - NEAR1_COVER;
						el[i].p0 = make_pair(p1, q1);
					}
					else {
						el[i].len0 = l_1_cover - NEAR1_COVER;
						el[i].p0 = make_pair(p_1, q_1);
					}
					for (int j = 1; j < el[i].len0; j++) {
						int idx = (i + j < el.size()) ? i + j : i + j - el.size();
						if (el[idx].dir0 == el[i].dir0) {
							int h_distance = dh.dot(pts[i + j] - p0);//pt2line_distance(pts[i + j], pts[i], dir);
							if (h_distance == 0) {
								el[idx].len0 = el[i].len0 - j;
								el[idx].p0 = make_pair(el[i].p0.first + dv*j, el[i].p0.second);
							}
						}
					}
			}			

			for (int i = 0; i < (int)el.size(); i++) {	//compute max extend / \ line for every point
				int k = 0;
				while (i + k < (int)el.size() - 1) {
					int j = min(VALID_EDGE_LEN2, (int)pts.size() - i - k - 1) / 2 * 2;
					bool check = false;
					for (; j > 0; j -= 2)
						if (abs(pts[i + j + k].x - pts[i].x) == (j + k) / 2 && abs(pts[i + j + k].y - pts[i].y) == (j + k) / 2) {
							check = true;
							break;
						}
					if (k > 0) { //check region map
						int dir = get_pts_dir(pts[i], pts[i + k]);
						Point dv(dxy[dir][1], dxy[dir][0]);
						for (int l = k / 2; l <= (k + j) / 2; l++) {
							int r0 = region_map.at<int>(pts[i] + dv * l);
							if (r0 != 0 && r0 != region_id)
								check = false;
						}							
					}					
					if (!check)
						break;
					k += j;
				}
				CV_Assert(k % 2 == 0);
				el[i].len1 = k;
				el[i].dir1 = get_pts_dir(pts[i], pts[i + k]);
				el[i].score = max(el[i].len0, el[i].len1 * XIE_RATIO);
				el[i].p1 = make_pair(pts[i], pts[i + k]);
				el[i].cover = 0;
				el[i].in_via = (cgrid.at<uchar>(pts[i]) & CGRID_VIA_REGION) ? 1 : 0;
				pel[i] = &el[i];
				CV_Assert(el[i].dir1 >= 0);
			}
			sort(pel.begin(), pel.end(), great_edgelen);

			for (int edge_len = MAX_TURN_LEN; edge_len >= 3; edge_len-=3) { //gap_len < 3 is too short line
				//3 find new tp longer than edge_len				
				for (int i = 0; i < pel.size(); i++) {
					if (pel[i]->cover) //already cover
						continue;
					if (abs(pel[i]->score - max(pel[i]->len0, pel[i]->len1 * XIE_RATIO)) > 0.01f) { //len0 or len1 updated
						pel[i]->score = max(pel[i]->len0, pel[i]->len1 * XIE_RATIO);
						continue;
					}
					if (pel[i]->score < edge_len)
						break;					
					float idx = pel[i] - &el[0];
					if ((int)idx + 1 < (int) el.size() && el[idx + 1].cover || head_type == 0 && (int)idx + 1 ==(int) el.size() && el[0].cover)
						continue;
					float idx_end;
					vector<Point> pts0;
					TurnPoint p;
					int len;
					if (pel[i]->len1 * XIE_RATIO <= pel[i]->len0) {
						p.dir = pel[i]->dir0;
						p.p1 = pel[i]->p0.first * CCL_GRID_SIZE;
						p.p2 = pel[i]->p0.second * CCL_GRID_SIZE;
						idx_end = idx + pel[i]->len0;
						len = pel[i]->len0 + 0.5f;
						if (idx_end - (int)idx_end > 0.01)
							idx += NEAR1_COVER;
						get_line_pts(pel[i]->p0.first, pel[i]->p0.second, pts0);
						pts0.push_back(pel[i]->p0.second);					
					}
					else {
						p.dir = pel[i]->dir1;
						p.p1 = pel[i]->p1.first * CCL_GRID_SIZE;
						p.p2 = pel[i]->p1.second * CCL_GRID_SIZE;
						idx_end = idx + pel[i]->len1;
						len = pel[i]->len1 + 0.5f;
						get_line_pts(pel[i]->p1.first, pel[i]->p1.second, pts0);
						pts0.push_back(pel[i]->p1.second);
					}
					for (auto & p0 : pts0)
						region_map.at<int>(p0) = region_id; //mark region map with region_id

					for (int j = 0; j < len; j++) {
						int k = idx - j - 1;
						if (k < 0) {
							if (head_type == 0)
								k += (int)el.size();
							else
								break;
						}
						el[k].len0 = min(el[k].len0, j + 1 - NEAR1_COVER);
						el[k].len1 = min(el[k].len1, j + 1 - NEAR1_COVER);
					}
						
					if (idx_end < pts_len) {
						p.cover = RangeF(idx, idx_end);
						for (int j = idx + 1 - 0.01f; j < idx_end; j++)
							el[j].cover = 1;			
						for (int j = 0; j <= (int)tp.size(); j++) {
							if (j == (int)tp.size()) {
								if (edge_len <= 3 && !tp.empty() && tail_type == NEED_CONNECT && tp.back().cover.end + border > pts.size()) { //tp.back already satisfy 
									RegionEdgeTile * etile = (RegionEdgeTile *)et; //don't need to replace tp.back because edge_line too short
									if (etile->dir == DIR_RIGHT && tp.back().dir != DIR_LEFT && tp.back().dir != DIR_UPLEFT && tp.back().dir != DIR_DOWNLEFT) {
										CV_Assert((tp.back().p1.x / CCL_GRID_SIZE + border >= cgrid.cols || tp.back().p2.x / CCL_GRID_SIZE + border >= cgrid.cols)
											&& (pts[idx].x + border >= cgrid.cols || pts[idx_end].x + border >= cgrid.cols));
										break;
									}
									if (etile->dir == DIR_DOWN && tp.back().dir != DIR_UP && tp.back().dir != DIR_UPLEFT && tp.back().dir != DIR_UPRIGHT) {
										CV_Assert((tp.back().p1.y / CCL_GRID_SIZE + border >= cgrid.rows || tp.back().p2.y / CCL_GRID_SIZE + border >= cgrid.rows)
											&& (pts[idx].y + border >= cgrid.rows || pts[idx_end].y + border >= cgrid.rows));
										break;
									}
								}
								tp.push_back(p);
								break;
							}
							CV_Assert(tp[j].cover.start != idx);
							if (tp[j].cover.start > idx) {
								if (j == 0 && edge_len <= 3 && head_type == NEED_CONNECT && tp[0].cover.start < border) { //tp.front already satisfy
									RegionEdgeTile * etile = (RegionEdgeTile *)eh;
									if (etile->dir == DIR_RIGHT && tp[0].dir != DIR_RIGHT && tp[0].dir != DIR_UPRIGHT && tp[0].dir != DIR_DOWNRIGHT) {
										CV_Assert((tp[0].p1.x / CCL_GRID_SIZE + border >= cgrid.cols || tp[0].p2.x / CCL_GRID_SIZE + border >= cgrid.cols) //tp.back already satisfy 
											&& (pts[idx].x + border >= cgrid.cols || pts[idx_end].x + border >= cgrid.cols)); //don't need tp.back edge_line too short
										break;
									}
									if (etile->dir == DIR_DOWN && tp[0].dir != DIR_DOWN && tp[0].dir != DIR_DOWNRIGHT && tp[0].dir != DIR_DOWNLEFT) {
										CV_Assert((tp[0].p1.y / CCL_GRID_SIZE + border >= cgrid.rows || tp[0].p2.y / CCL_GRID_SIZE + border >= cgrid.rows) //tp.back already satisfy 
											&& (pts[idx].y + border >= cgrid.rows || pts[idx_end].y + border >= cgrid.rows)); //don't need tp.back edge_line too short
										break;
									}
								}
								if (tp[j].cover.end <= p.cover.end) //p.cover include tp[j].cover									
									tp[j] = p;								
								else
									tp.insert(tp.begin() + j, p);
								break;
							}
						}
					}
					else { //tp covers loop end and begin, seperate tp to two tp
						CV_Assert(head_type == 0 && idx < pts_len);
						if (tp.empty() || tp.front().cover.start != -1) {
							p.cover = RangeF(-1, idx_end - pts_len); //front tp start with -1
							if (!tp.empty() && tp.back().cover.start >= idx) //p.cover include tp.back.cover								
								tp.pop_back();							
							for (int j = idx + 1 - 0.01f; j < idx_end; j++) {
								if (j < el.size())
									el[j].cover = 1;
								else
									el[j - el.size()].cover = 1;
							}								
							if (!tp.empty() && tp[0].cover.end <= p.cover.end) //p.cover include tp[0].cover								
								tp[0] = p;							
							else
								tp.insert(tp.begin(), p);
							p.cover = RangeF(idx, pts_len); //back tp end with pts_len
							tp.push_back(p);
						}
					}	//end if (idx_end < pts_len) 				
				} //end for pel
				sort(pel.begin(), pel.end(), great_edgelen);
				//4 With new tp created in 3, direct connect tp
				for (int i = 0; i + 1 < (int)((head_type == 0) ? tp.size() + 1 : tp.size()); i++) {
					bool direct_connect = false; //direct_connect=true means may connect two tp directly				
					TurnPoint &tpi1 = (i + 1 == (int)tp.size()) ? tp[0] : tp[i + 1];
					float i1cover = tpi1.cover.start; //tp.cover.start < pts_len and tp.cover.end <= pts_len
					float i1coverend = tpi1.cover.end;
					CV_Assert(i1cover < pts_len);
					if (head_type == 0 && i + 1 == (int)tp.size())
						if (i1cover == -1) //tp covers loop end and begin, don't connect
							continue;
						else {
							i1cover += pts_len;
							i1coverend += pts_len;
						}
					if (i1cover == tp[i].cover.end) //already connected
						continue;
					if (i1coverend <= tp[i].cover.end) {
						tp.erase(tp.begin() + i + 1);
						i--;
						continue;
					}
					if (i1cover - tp[i].cover.end <= MAX_TURN_LEN) //it tp's distance is short enough, dirrect connect, i1cover may < tp[i].cover.end 
						direct_connect = true;
					else {
						int len = 0;
						for (int j = tp[i].cover.end; j <= i1cover; j++)
							if (j < (int)el.size() && el[j].in_via || j >= (int)el.size() && el[j - el.size()].in_via) //skip via point
								len++;
						if (i1cover - tp[i].cover.end <= MAX_TURN_LEN + len) //if points are via between tp, dirrect connect
							direct_connect = true;
					}
					bool connected = false; //connected=true means connected finally
					if (direct_connect) { //connect two tp
						Point di(dxy[tp[i].dir][1], dxy[tp[i].dir][0]);
						Point di1(dxy[tpi1.dir][1], dxy[tpi1.dir][0]);						
						vector<Point> pts0;
						if (di.dot(di1) >= 0 && tp[i].dir != tpi1.dir) {
							Point pis;
							bool intersected = intersect_line(tp[i].p2, tp[i].dir, tpi1.p1, tpi1.dir, pis);
							vector<Point> pts1, pts2;
							if (check_region(region_map, tp[i].p2, pis, pts1, region_id) &&
								check_region(region_map, tpi1.p1, pis, pts2, region_id)) { //region ok, can connect with intersect point
								CV_Assert(pis.inside(border_rect));
								pts0.swap(pts1);								//connect  |
								pts0.insert(pts0.end(), pts2.begin(), pts2.end()); //      |
								tp[i].p2 = pis;									//   ______|
								tpi1.p1 = pis;
								connected = true;
							}
						}
						else
							if (tp[i].dir == tpi1.dir) {
								Point p2 = tpi1.p1, p1 = tp[i].p2;
								switch (tp[i].dir) { //conserved method is to narrow edge
								case DIR_UP:
									if (p1.x > p2.x)
										p1.y = p2.y;
									else
										p2.y = p1.y;
									break;
								case DIR_DOWN:
									if (p1.x > p2.x)
										p2.y = p1.y;
									else
										p1.y = p2.y;
									break;
								case DIR_RIGHT:
									if (p1.y > p2.y)
										p1.x = p2.x;
									else
										p2.x = p1.x;
									break;
								case DIR_LEFT:
									if (p1.y > p2.y)
										p2.x = p1.x;
									else
										p1.x = p2.x;
									break;
								case DIR_UPRIGHT:
									if (p1.y + p1.x > p2.y + p2.x) {
										p1.x = p1.y + p1.x - p2.y;
										p1.y = p2.y;
									}
									else {
										p2.x = p2.y + p2.x - p1.y;
										p2.y = p1.y;
									}
									break;
								case DIR_DOWNLEFT:
									if (p1.y + p1.x > p2.y + p2.x) {
										p2.x = p2.y + p2.x - p1.y;
										p2.y = p1.y;
									}
									else {
										p1.x = p1.y + p1.x - p2.y;
										p1.y = p2.y;
									}
									break;
								case DIR_UPLEFT:
									if (p1.y - p1.x > p2.y - p2.x) {
										p2.y = p2.y - p2.x + p1.x;
										p2.x = p1.x;
									}
									else {
										p1.y = p1.y - p1.x + p2.x;
										p1.x = p2.x;
									}
									break;
								case DIR_DOWNRIGHT:
									if (p1.y - p1.x > p2.y - p2.x) {
										p1.y = p1.y - p1.x + p2.x;
										p1.x = p2.x;
									}
									else {
										p2.y = p2.y - p2.x + p1.x;
										p2.x = p1.x;
									}
									break;
								}
								vector<Point> pts1, pts2, pts3;
#if 0
								if (tpi1.p1.x >= 356 && tpi1.p1.x <= 362 && tpi1.p1.y >= 838 && tpi1.p1.y <= 856) {
									p1.x = p1.x * 2 - p1.x;
								}
#endif
								if (check_region(region_map, tp[i].p2, p1, pts1, region_id) &&
									check_region(region_map, tpi1.p1, p2, pts3, region_id) &&
									check_region(region_map, p1, p2, pts2, region_id)) { //region ok, can connect
									CV_Assert(get_pts_dir(p1, p2) >= 0 && p1.inside(border_rect) && p2.inside(border_rect));
									pts0.swap(pts1);								//connect   _____
									pts0.insert(pts0.end(), pts2.begin(), pts2.end()); //______|
									pts0.insert(pts0.end(), pts3.begin(), pts3.end());
									tp[i].p2 = p1;
									tpi1.p1 = p2;									
									connected = true;
								}
							}
							else {
								Point p0 = pts[i1cover] * CCL_GRID_SIZE; //p0 is farest point
								int m = -10000000;
								Point dvec(dxy[tp[i].dir][1], dxy[tp[i].dir][0]);
								for (int j = tp[i].cover.end; j <= i1cover; j++) {
									int d = pts[j].dot(dvec);
									if (d > m) {
										m = d;
										p0 = pts[j] * CCL_GRID_SIZE;
									}
								}
								for (int j = 3; j >= 0; j--) { //reduce distance and try
									Point pp0 = Point((p0.x * j + tp[i].p2.x * (3 - j)) / 3, (p0.y * j + tp[i].p2.y * (3 - j)) / 3);
									Point p1, p2;
									vector<Point> pts1, pts2, pts3;
									intersect_line(tp[i].p2, tp[i].dir, pp0, dir_2[tp[i].dir], p1);
									intersect_line(tpi1.p1, tpi1.dir, pp0, dir_2[tp[i].dir], p2);
									if (check_region(region_map, tp[i].p2, p1, pts1, region_id) &&
										check_region(region_map, tpi1.p1, p2, pts3, region_id) &&
										check_region(region_map, p1, p2, pts2, region_id)) { //region ok, can connect)
										CV_Assert(get_pts_dir(p1, p2) >= 0 && p1.inside(border_rect) && p2.inside(border_rect));
										pts0.swap(pts1);							//connect  ______
										pts0.insert(pts0.end(), pts2.begin(), pts2.end());	// ______|
										pts0.insert(pts0.end(), pts3.begin(), pts3.end());
										tp[i].p2 = p1;
										tpi1.p1 = p2;										
										connected = true;
										break;
									}
								}
							}
							if (connected) {
								for (int j = tp[i].cover.end; j <= i1cover; j++)
									if (j < (int)el.size())
										el[j].cover = 1;
									else
										el[j - el.size()].cover = 1;
								if (i1cover < pts_len) {
									tp[i].cover.end = (int) (tpi1.cover.start + tp[i].cover.end) / 2 + 0.5f;
									tpi1.cover.start = tp[i].cover.end;
								}
								else { //for loop case
									CV_Assert(head_type == 0);
									tp[i].cover.end = pts_len - 0.5f;
									tpi1.cover.start = -0.5f;
								}
								for (auto & p : pts0)
									region_map.at<int>(p) = region_id; //mark region map with region_id
							}
					}					
				} //end for connection
			}

			//now tp is ready change tp to RegionEdgeTurnPoint
			RegionEdgeTurnPoint *ntp = new RegionEdgeTurnPoint();
			for (int i = 0; i < tp.size(); i++) {
				CV_Assert(get_pts_dir(tp[i].p2, tp[i].p1) >= 0 && tp[i].p2.inside(border_rect) && tp[i].p1.inside(border_rect));
				if (i == 0) {
					if (head_type == HAVE_CONNECT) 
						ntp->pts.push_back(tp[i].p2);
					else
						if (head_type == NEED_CONNECT) {
							ntp->pts.push_back(tp[i].p1);
							ntp->pts.push_back(tp[i].p2);
						}
						else {
							if (tp[i].cover.start == -1) //tp cross loop
								ntp->pts.push_back(tp[i].p2);
							else {
								for (int j = 0; j < tp[i].cover.start; j++) {
									ntp->pts.push_back(pts[j] * CCL_GRID_SIZE);
									region_map.at<int>(pts[j]) = region_id;
								}
								ntp->pts.push_back(tp[i].p1);
								ntp->pts.push_back(tp[i].p2);
							}
						}
				}
				else {
					for (int j = tp[i - 1].cover.end + 0.51f; j < tp[i].cover.start; j++) {
						ntp->pts.push_back(pts[j] * CCL_GRID_SIZE);
						region_map.at<int>(pts[j]) = region_id;
					}
					if (i + 1 == tp.size()) {
						if (tail_type == HAVE_CONNECT)
							ntp->pts.push_back(tp[i].p1);
						else
							if (tail_type == NEED_CONNECT) {
								ntp->pts.push_back(tp[i].p1);
								ntp->pts.push_back(tp[i].p2);
							}
							else {								
								if (tp[0].cover.start == -1)//tp cross loop
									ntp->pts.push_back(tp[i].p1);
								else {
									ntp->pts.push_back(tp[i].p1);
									ntp->pts.push_back(tp[i].p2);
									for (int j = tp[i].cover.end + 0.51f; j < pts_len; j++) {
										ntp->pts.push_back(pts[j] * CCL_GRID_SIZE);
										region_map.at<int>(pts[j]) = region_id;
									}
								}
							}
					}
					else {						
						ntp->pts.push_back(tp[i].p1);
						ntp->pts.push_back(tp[i].p2);
					}
				}
			}
			for (int i = 1; i < (int)ntp->pts.size(); i++) {				
				if (ntp->pts[i] == ntp->pts[i - 1]) {
					ntp->pts.erase(ntp->pts.begin() + i); //erase same
					i--;
					continue;
				}
				CV_Assert(get_pts_dir(ntp->pts[i - 1], ntp->pts[i]) >= 0 && ntp->pts[i - 1].inside(border_rect) && ntp->pts[i].inside(border_rect));
			}
			if (head_type == 0) {
				if (ntp->pts[0] == ntp->pts.back())
					ntp->pts.pop_back();
				else
					CV_Assert(get_pts_dir(ntp->pts[0], ntp->pts.back()) >= 0);
			}
			for (int i = 1; i + 1 < (int)ntp->pts.size(); i++) {
				if (get_pts_dir(ntp->pts[i - 1], ntp->pts[i]) == get_pts_dir(ntp->pts[i], ntp->pts[i + 1])) {
					ntp->pts.erase(ntp->pts.begin() + i);
					i--;
					continue;
				}
			}
			local_turn_edges.push_back(make_pair(region_id, ntp));
			//Replace [eh,et] with ntp
			nh = ntp;
			if (head_type == NEED_CONNECT) { //connect ntp to head
				RegionEdgeConnect * nc = new RegionEdgeConnect();
				for (int j = 0; j <= tp[0].cover.start; j++) {
					nc->pts.push_back(pts[j] * CCL_GRID_SIZE + org);
					region_map.at<int>(pts[j]) = region_id;
				}
				if (nc->pts.back() != ntp->pts[0] + org)
					nc->pts.push_back(ntp->pts[0] + org);
				CV_Assert(!nc->pts.empty());
				nc->start_dir = tp[0].dir;
				nc->start_len = min(abs(tp[0].p2.x - tp[0].p1.x) + abs(tp[0].p2.y - tp[0].p1.y), 5);
				nh = nc;
				nc->link(ntp);
			}
			if (tail_type == NEED_CONNECT) { //connect ntp to tail
				RegionEdgeConnect * nc = new RegionEdgeConnect();
				for (int j = tp.back().cover.end; j < pts_len; j++) {
					nc->pts.push_back(pts[j] * CCL_GRID_SIZE + org);
					region_map.at<int>(pts[j]) = region_id;
				}
				if (nc->pts.front() != ntp->pts.back() + org)
					nc->pts.insert(nc->pts.begin(), ntp->pts.back() + org);
				CV_Assert(!nc->pts.empty());
				nc->start_dir = tp.back().dir;
				nc->start_len = min(abs(tp.back().p2.x - tp.back().p1.x) + abs(tp.back().p2.y - tp.back().p1.y), 5);
				ntp->link(nc);
			}
		} //else end for head_type == NEED_CONNECT & taile_type==NEED_CONNECT
		//now nh is new edge in rs
		if (head_type) {
			RegionEdge * nt = eh->prev;  //replace existing region [eh, et] with nh
			eh->unlink(et);		//delete [eh,et]		
			Region * ret = nt->link(nh); //link nh
			CV_Assert(ret == NULL);
			auto it = global_rs->regions.find(nh->region->region_id);
			if (it == global_rs->regions.end())
				global_rs->regions[nh->region->region_id] = nh->region;
			else
				CV_Assert(it->second == nh->region);
		}
		else { //head_type ==0 && tail_type==0, directly output	
			auto it = global_out->regions.find(global_region_id);
			Region * new_region;
			if (it == global_out->regions.end()) {
				new_region = new Region(global_region_id); // add new region to output				
				global_out->regions[global_region_id] = new_region;
			}
			else 
				new_region = it->second;			
			RegionEdge * nt = nh;
			do {
				nt->region = new_region;
				new_region->edges.insert(nt);
				nt = nt->next;
			} while (nt != nh);
		}
		et = eh; //erase [eh, et] in local_atom_edges
		do {
			RegionEdge * e = et->next;
			if (et->type == REGION_EDGE_ATOM_GRID)
				local_atom_edges.erase(et);
			else
				if (et->type == REGION_EDGE_TILE_GRID)
					local_tile_edges.erase(et);
			delete et;
			et = e;
		} while (et != eh);
#endif
	} //end for each atom edge
/*
	for (auto & turn_edge : local_turn_edges) {
		RegionEdgeTurnPoint * ntp = turn_edge.second;
		RegionID region_id = turn_edge.first; //now ntp's region_id may not be original region_id, we need to use original region_id
		for (int di = 1; di > -2; di-=2) { //di means different erase direction
			Point p1, p2;
			vector<Point> left_pts;
			int si, i, j;
			if (ntp->next == ntp)
				if (di == 1) {
					ntp->pts.push_back(ntp->pts[0]);
					ntp->pts.push_back(ntp->pts[1]);
				}
				else {
					Point a2[2] = { ntp->pts[ntp->pts.size() - 2], ntp->pts.back() };
					ntp->pts.insert(ntp->pts.begin(), a2, a2 + 2);
				}
			if (di == 1) { //di==1 erase ntp->pts[j++]
				p2 = ntp->pts[0];
				si = 1;
			}
			else {
				p2 = ntp->pts.back();
				si = ntp->pts.size() - 2; //di==1 erase ntp->pts[j--]
			}
			if (ntp->next != ntp)
				left_pts.push_back(p2);
			while(1) {
				p1 = p2;
				CV_Assert(si >= 0 && si < (int)ntp->pts.size());
				p2 = ntp->pts[si];
				int dir = get_pts_dir(p1, p2);
				Point dv(dxy[dir][1], dxy[dir][0]); //dir vector
				CV_Assert(dir >= 0);
				si += di;
				if (si == ntp->pts.size() || si == -1) {
					left_pts.push_back(p2);
					break;					
				}
				
				Point p4 = ntp->pts[si];
				if (abs(pt2line_distance(p4, p2, dir)) > ERASE_EDGE_LEN) {
					left_pts.push_back(p2);
					continue;
				}
				j = si;
				int acc_area = 0;
				while (1) {
					Point p3 = p4;
					j += di;
					if (j == ntp->pts.size() || j == -1) {						
						left_pts.push_back(p2);
						break;						
					}
					p4 = ntp->pts[j];
					float distance = pt2line_distance(p4, p2, dir);
					if (abs(distance) > ERASE_EDGE_LEN) {
						left_pts.push_back(p2);
						break;
					}
					float p3distance = pt2line_distance(p3, p2, dir);
					float area_plus_neg = max(abs(p3.x - p4.x), abs(p3.y - p4.y)) * (p3distance + distance) / 2;
					int dir2 = get_pts_dir(p3, p4);
					CV_Assert(dir2 >= 0);

					if (dir > 3) {
						if (dir2 >= 3)
							area_plus_neg = area_plus_neg * 1.4;
						else
							area_plus_neg = area_plus_neg / 1.4;
					}
					acc_area += area_plus_neg;
					if (abs(area_plus_neg) >ERASE_EDGE_AREA || abs(acc_area)>ERASE_EDGE_AREA) {
						left_pts.push_back(p2);
						break;
					}
					Point pis;
					intersect_line(p2, dir, p4, dir_2[dir], pis);
					if (dv.dot(p2) < dv.dot(pis) && get_pts_dir(p2, pis) >= 0 && get_pts_dir(p4, pis) >= 0) {
						vector<Point> pts1, pts2;
						if (check_region(region_map, p2, pis, pts1, region_id) &&
							check_region(region_map, p4, pis, pts2, region_id)) {
							pts1.insert(pts1.end(), pts2.begin(), pts2.end());
							p2 = pis;
							si = j;
							for (auto & p : pts1)
								region_map.at<int>(p) = region_id; //mark region map with region_id
						}
					}
				}
			} //end for while
			for (int i = 1; i < (int)left_pts.size(); i++) {
				CV_Assert(get_pts_dir(left_pts[i - 1], left_pts[i]) >= 0 && left_pts[i - 1].inside(border_rect) && left_pts[i].inside(border_rect));
				if (left_pts[i] == left_pts[i - 1]) {
					left_pts.erase(left_pts.begin() + i);
					i--;
					continue;
				}
			}
			if (left_pts[0] == left_pts.back())
				left_pts.pop_back();
			//now left_pts is ready
			if (di == -1)
				reverse(left_pts.begin(), left_pts.end());
			ntp->pts.swap(left_pts);
		}
		for (auto & pt : ntp->pts)
			pt += org; //change to global
	}
*/
	while (!local_tile_edges.empty()) {
		RegionEdgeTile * e_tile = (RegionEdgeTile *)(*local_tile_edges.begin());
		if (e_tile->other) {
			CV_Assert(e_tile->next == e_tile->other && e_tile->prev == e_tile->other && e_tile->other->next == e_tile && e_tile->other->prev == e_tile);
			CV_Assert(e_tile->region == e_tile->other->region);
			e_tile->region->edges.erase(e_tile->other);
			e_tile->region->edges.erase(e_tile);
			if (!e_tile->region->has_tile()) {
				global_out->regions[e_tile->region->region_id] = e_tile->region;
				global_rs->regions.erase(e_tile->region->region_id);
			}
			local_tile_edges.erase(e_tile);
		}
	}
	qInfo("process_atom_edge left tile num=%d", local_tile_edges.size());
}
/*
inout c210, process c210
input via_cut_len
input wire_cut_len
input d, via diameter
1 do heng shu cut, e.g. 5 insu + 4 wire + 5 insu, then 4 wire is cut as insu, avoid wire wrong connect
*/
static void post_process(Mat & c210, int wire_cut_len)
{
	Mat m;
	blur(c210, m, Size(3, 3));
	CV_Assert(m.type() == CV_8UC1);
	vector<Range> l0(m.cols), l1(m.cols);

	for (auto & l : l0)
		l = Range(-1000, -1000);
	for (auto & l : l1)
		l = Range(-1000, -1000);

	//do heng shu cut, e.g. 5 insu + 4 wire + 5 insu, then 4 wire is cut as insu, avoid wire wrong connect
	for (int y = 0; y < m.rows; y++) {
		uchar * pm = m.ptr<uchar>(y);
		uchar * pc = c210.ptr<uchar>(y);
		Range h0(-1000,-1000), h1(-1000, -1000);
		for (int x = 0; x < m.cols; x++) {
			if (pc[x] == COLOR_VIA_INSU || pc[x] == COLOR_VIA_WIRE) { //not cut in via region
				h1.start = -1000;
				l1[x].start = -1000;
				continue;
			}
			if (pm[x] < COLOR_JUDGE) { //insu
				if (h1.start < 0)
					h1.start = x;
				else
					h1.end = x;
				if (l1[x].start < 0)
					l1[x].start = y;
				else
					l1[x].end = y;
			}
			else { //wire
				if (h1.start >= 0 && h1.end - h1.start >= wire_cut_len) { //valid insu len
					if (h1.start - h0.end <= wire_cut_len + 1) { //wire too short, cut
						for (int i = h0.end + 1; i < h1.start; i++) {
							pm[i] = COLOR_CUT_WIRE;
							pc[i] = COLOR_CUT_WIRE;
						}
					}
					h0 = h1;
				}
				h1.start = -1000;
				if (l1[x].start >= 0 && l1[x].end - l1[x].start >= wire_cut_len) { //valid insu len
					if (l1[x].start - l0[x].end <= wire_cut_len + 1) { //wire too short, cut
						for (int i = l0[x].end + 1; i < l1[x].start; i++) {
							m.at<uchar>(i, x) = COLOR_CUT_WIRE;
							c210.at<uchar>(i, x) = COLOR_CUT_WIRE;
						}
					}
					l0[x] = l1[x];
				}
				l1[x].start = -1000;
			}
		}
		if (h1.start >= 0 && h1.end - h1.start >= wire_cut_len) { //valid insu len
			if (h1.start - h0.end <= wire_cut_len + 1) { //wire too short, cut
				for (int i = h0.end + 1; i < h1.start; i++) {
					pm[i] = COLOR_CUT_WIRE;
					pc[i] = COLOR_CUT_WIRE;
				}
			}
			h0 = h1;
		}
	}

	for (int x = 0; x < m.cols; x++)
	if (l1[x].start >= 0 && l1[x].end - l1[x].start >= wire_cut_len) { //valid insu len
		if (l1[x].start - l0[x].end <= wire_cut_len + 1) { //wire too short, cut
			for (int i = l0[x].end + 1; i < l1[x].start; i++) {
				m.at<uchar>(i, x) = COLOR_CUT_WIRE;
				c210.at<uchar>(i, x) = COLOR_CUT_WIRE;
			}
		}
		l0[x] = l1[x];
	}
}

ProcessImageData process_img(const ProcessImageData & pi)
{
	qInfo("process_img (%d,%d,%d)", pi.cpd->x0, pi.cpd->y0, pi.cpd->layer);

	if (!pi.cpd->raw_img.empty()) {
		Mat via_mark, empty;
		Mat prob, prob1;
		pi.vwf->via_search(pi.cpd->raw_img, via_mark, (pi.cpd->via_mark_debug == NULL ? empty : *(pi.cpd->via_mark_debug)), pi.cpd->eo, pi.multi_thread);
		pi.vwf->edge_search(pi.cpd->raw_img, prob, via_mark, (pi.cpd->edge_mark_debug == NULL ? empty : pi.cpd->edge_mark_debug[0]), pi.multi_thread);
		if (!prob.empty()) {
			if (pi.cpd->edge_mark_debug)
				draw_prob(prob, pi.cpd->edge_mark_debug[0], 0.55);
			filter_prob(prob, prob1);
			//prob_color(prob1, (pi.cpd->edge_mark_debug1 == NULL ? empty : pi.cpd->edge_mark_debug1[0]));
			vector<EdgeLine> edges;
			Mat mark, mark2, c210, ccl;
			Range min_sweet_len = pi.wire_sweet_len_x.start < pi.wire_sweet_len_y.start ? pi.wire_sweet_len_x : pi.wire_sweet_len_y;
			mark_strong_edge(prob1, min_sweet_len, pi.insu_min, edges, mark, mark2);
			//prob_color(prob1, (pi.cpd->edge_mark_debug2 == NULL ? empty : pi.cpd->edge_mark_debug2[0]), 0.98);
			//self_enhance(prob1, prob, 3);	
			/*
			if (pi.cpd->edge_mark_debug1)
				draw_mark(mark, pi.cpd->edge_mark_debug1[0]);*/
			mark_wvi210(pi.cpd->raw_img, mark, c210, (pi.wire_sweet_len_x.start + pi.wire_sweet_len_y.start) / 2 - 1, pi);
			process_unsure(prob1, c210);
			if (pi.cpd->edge_mark_debug1)
				draw_210(c210, pi.cpd->edge_mark_debug1[0]);
			//post_process(c210, pi.wire_sweet_len.start);
			//mark_210(c210, (pi.cpd->edge_mark_debug3 == NULL ? empty : pi.cpd->edge_mark_debug3[0]));
			Mat cgrid, adjscore;
			compute_adj(c210, prob1, cgrid, adjscore, pi.wire_sweet_len_x.start, pi.wire_sweet_len_y.start);
			int B_2 = pi.border_size / CCL_GRID_SIZE;
			int copy_cols = cgrid.cols - B_2 * 2;
			int copy_rows = cgrid.rows - B_2 * 2;
			int cgrid_cols = pi.lpd ? cgrid.cols : copy_cols;
			int cgrid_rows = pi.upd ? cgrid.rows : copy_rows;
			int cgrid_x0 = pi.lpd ? B_2 * 2 : 0;
			int cgrid_y0 = pi.upd ? B_2 * 2 : 0;
			pi.cpd->cgrid.create(cgrid_rows, cgrid_cols, CV_8UC1);
			pi.cpd->adjscore.create(cgrid_rows, cgrid_cols, CV_8UC2);
			cgrid(Rect(B_2, B_2, copy_cols, copy_rows)).copyTo(pi.cpd->cgrid(Rect(cgrid_x0, cgrid_y0, copy_cols, copy_rows)));
			adjscore(Rect(B_2, B_2, copy_cols, copy_rows)).copyTo(pi.cpd->adjscore(Rect(cgrid_x0, cgrid_y0, copy_cols, copy_rows)));
			if (pi.lpd) {
				CV_Assert(pi.lpd->cgrid.rows == pi.cpd->cgrid.rows);
				pi.lpd->cgrid(Rect(pi.lpd->cgrid.cols - B_2 * 2, 0, B_2 * 2, pi.lpd->cgrid.rows)).copyTo(pi.cpd->cgrid(Rect(0, 0, B_2 * 2, pi.lpd->cgrid.rows)));
				pi.lpd->adjscore(Rect(pi.lpd->cgrid.cols - B_2 * 2, 0, B_2 * 2, pi.lpd->cgrid.rows)).copyTo(pi.cpd->adjscore(Rect(0, 0, B_2 * 2, pi.lpd->cgrid.rows)));
			}
			if (pi.upd) {
				CV_Assert(pi.upd->cgrid.cols == pi.cpd->cgrid.cols);
				pi.upd->cgrid(Rect(0, pi.upd->cgrid.rows - B_2 * 2, pi.upd->cgrid.cols, B_2 * 2)).copyTo(pi.cpd->cgrid(Rect(0, 0, pi.upd->cgrid.cols, B_2 * 2)));
				pi.upd->adjscore(Rect(0, pi.upd->cgrid.rows - B_2 * 2, pi.upd->cgrid.cols, B_2 * 2)).copyTo(pi.cpd->adjscore(Rect(0, 0, pi.upd->cgrid.cols, B_2 * 2)));
			}
			int via_area = pi.via_diameter * pi.via_diameter;
			do_ccl(pi.cpd->cgrid, pi.cpd->adjscore, ccl, max(CCL_MERGE_AREA, via_area), max(CCL_CUT_AREA, via_area));
			/*expand_ccl(ccl, c210, eccl);			
			if (pi.cpd->edge_mark_debug3)
				draw_ccl(eccl, pi.cpd->edge_mark_debug3[0]);*/
			RegionSet rs;
			mark_atom_edge(ccl, rs, B_2, max(CCL_CUT_LEN, pi.via_diameter * 6));
			if (pi.cpd->edge_mark_debug2)
				draw_region_edge(rs, pi.cpd->edge_mark_debug2[0], Point(pi.border_size, pi.border_size));
			process_atom_edge(pi.cpd->cgrid, rs, pi.global_rs, pi.global_out, Point(0, 0), 1, B_2, 0xff);
			if (pi.cpd->edge_mark_debug3)				
				draw_region_edge(*pi.global_out, pi.cpd->edge_mark_debug3[0], Point(pi.border_size, pi.border_size));
			rs.regions.clear();
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
	for (int i = 0; i < sizeof(insu_min) / sizeof(insu_min[0]); i++) {
		insu_min[i] = 3;
		wire_min_x[i] = 6;
		wire_min_y[i] = 6;
	}
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
		insu_min[layer_min] = d & 0xff;
		wire_min_x[layer_min] = d >> 8 & 0xff;
		wire_min_y[layer_min] = d >> 16 & 0xff;
	}
	return 0;
}

int VWExtractML::set_extract_param(int layer, int type, int d, int, int, int, int, int, int, float)
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
	if ((type & 0xff) == OBJ_POINT && ((type >> 8 & 0xff) == POINT_WIRE_INSU || (type >> 8 & 0xff) == POINT_WIRE
		|| (type >> 8 & 0xff) == POINT_INSU || (type >> 8 & 0xff) == POINT_WIRE_INSU_V)
		|| (type >> 8 & 0xff) == POINT_WIRE_V || (type >> 8 & 0xff) == POINT_INSU_V) {
		insu_min[layer_min] = d & 0xff;
		wire_min_x[layer_min] = d >> 8 & 0xff;
		wire_min_y[layer_min] = d >> 16 & 0xff;
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
		int loc = (int) img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer);
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
		vwf[current_layer].write_file(project_path, current_layer, insu_min[current_layer], min(wire_min_x[current_layer], wire_min_y[current_layer]));
	}
	return 0;
}

int VWExtractML::extract(string img_name, QRect rect, vector<MarkObj> & obj_sets)
{
	vector<ProcessImageData> pis;
	ProcessData ed[MAX_LAYER_NUM];
	RegionSet global_rs, global_out;
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
		wire_min_x[current_layer] = max(4, wire_min_x[current_layer]);
		wire_min_y[current_layer] = max(4, wire_min_y[current_layer]);
		insu_min[current_layer] = max(2, insu_min[current_layer]);
		string file_name(img_name);
		file_name[file_name.length() - 5] = current_layer + '0';
		Mat img = imread(file_name, 0);
		int img_cols = img.cols / CCL_GRID_SIZE  * CCL_GRID_SIZE;
		int img_rows = img.rows / CCL_GRID_SIZE  * CCL_GRID_SIZE;
		Mat raw_img = img(Rect(0, 0, img_cols, img_rows));
		int loc = (int) img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		if (!vwf[current_layer].read_file(project_path, current_layer))
			return -1;
		ProcessImageData pi;
		ed[current_layer].x0 = 0;
		ed[current_layer].y0 = 0;
		ed[current_layer].img_pixel_x0 = 0;
		ed[current_layer].img_pixel_y0 = 0;
		ed[current_layer].layer = current_layer;
		ed[current_layer].raw_img = raw_img;
		ed[current_layer].poly = NULL;
		via_mark[current_layer] = raw_img.clone();
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
		pi.global_rs = &global_rs;
		pi.global_out = &global_out;
		pi.multi_thread = false;
		pi.border_size = 12;
		pi.insu_min = insu_min[current_layer];
		pi.via_diameter = vwf[current_layer].get_max_d();
		pi.wire_sweet_len_x = Range(wire_min_x[current_layer], wire_min_x[current_layer] * 2 + insu_min[current_layer]);
		pi.wire_sweet_len_y = Range(wire_min_y[current_layer], wire_min_y[current_layer] * 2 + insu_min[current_layer]);
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
		int loc = (int) img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer);
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
		vwf[current_layer].write_file(project_path, current_layer, insu_min[current_layer], min(wire_min_x[current_layer], wire_min_y[current_layer]));
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
		int loc = (int) img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer);
		BORDER_SIZE = max(BORDER_SIZE, vwf[current_layer].get_max_d());
	}
	BORDER_SIZE += 8;
	BORDER_SIZE = (BORDER_SIZE + 2) / 4 * 4;

	vector<SearchAreaPoly> area_;
	QPolygon area_poly;
	RegionSet global_rs, global_out;
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
			ProcessData ed[MAX_LAYER_NUM];
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
				pi.global_rs = &global_rs;
				pi.global_out = &global_out;
				pi.border_size = BORDER_SIZE;
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
						for (int i = 0; i < 3; i++) {
							wide[i] = wide[i] / CCL_GRID_SIZE * CCL_GRID_SIZE;
							height[i] = height[i] / CCL_GRID_SIZE * CCL_GRID_SIZE;
						}
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
						pi.global_rs = &global_rs;
						pi.global_out = &global_out;
						pi.border_size = BORDER_SIZE;
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
