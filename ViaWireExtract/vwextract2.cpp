#include "vwextract2.h"
#include <list>
#include <algorithm>
#include <QDateTime>
#include <QScopedPointer>
#include <QDir>
#ifndef QT_DEBUG
#undef CV_Assert
#define CV_Assert(x) do {if (!(x)) {qFatal("Wrong at %s, %d", __FILE__, __LINE__);}} while(0)
#endif

#define BORDER_SIZE 64
#define OPT_DEBUG_EN 0x8000

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
#define BRICK_IN_USE		15
#define BRICK_ONE_POINT		64
#define BRICK_II_0			65
#define BRICK_II_90			66
#define BRICK_II_180		67
#define BRICK_II_270		68
#define BRICK_III			69
#define BRICK_HOLLOW		252
#define BRICK_FAKE_VIA		253
#define BRICK_VIA			254
#define BRICK_INVALID		255

#define DIR_UP				0
#define DIR_RIGHT			1
#define DIR_DOWN			2
#define DIR_LEFT			3

#define DIR_UP1_MASK		(1 << (DIR_UP * 2))
#define DIR_RIGHT1_MASK		(1 << (DIR_RIGHT * 2))
#define DIR_DOWN1_MASK		(1 << (DIR_DOWN * 2))
#define DIR_LEFT1_MASK		(1 << (DIR_LEFT * 2))

static const int dxy[4][2] = {
		{ -1, 0 }, //up
		{ 0, 1 }, //right
		{ 1, 0 }, //down
		{ 0, -1 } //left
};

static struct Brick {
	int a[3][3];
	int shape;
} bricks[] = {
		{		{
				{ 0, 0, 0 },
				{ 0, 0, 0 },
				{ 0, 0, 0 } },
				0
		},

		{		{
				{ 0, 1, 0 },
				{ 0, 1, 0 },
				{ 0, 0, 0 } },
				DIR_UP1_MASK
		},

		{		{
				{ 0, 0, 0 },
				{ 0, 1, 1 },
				{ 0, 0, 0 } },
				DIR_RIGHT1_MASK
		},

		{		{
				{ 0, 0, 0 },
				{ 0, 1, 0 },
				{ 0, 1, 0 } },
				DIR_DOWN1_MASK
		},

		{		{
				{ 0, 0, 0 },
				{ 1, 1, 0 },
				{ 0, 0, 0 } },
				DIR_LEFT1_MASK
		},
		{		{
				{ 0, 1, 0 },
				{ 0, 1, 0 },
				{ 0, 1, 0 } },
				DIR_UP1_MASK | DIR_DOWN1_MASK
		},

		{		{
				{ 0, 0, 0 },
				{ 1, 1, 1 },
				{ 0, 0, 0 } },
				DIR_LEFT1_MASK | DIR_RIGHT1_MASK
		},

		{		{
				{ 0, 1, 0 },
				{ 0, 1, 1 },
				{ 0, 0, 0 } },
				DIR_UP1_MASK | DIR_RIGHT1_MASK
		},

		{		{
				{ 0, 0, 0 },
				{ 0, 1, 1 },
				{ 0, 1, 0 } },
				DIR_RIGHT1_MASK | DIR_DOWN1_MASK
		},

		{		{
				{ 0, 0, 0 },
				{ 1, 1, 0 },
				{ 0, 1, 0 } },
				DIR_DOWN1_MASK | DIR_LEFT1_MASK
		},

		{		{
				{ 0, 1, 0 },
				{ 1, 1, 0 },
				{ 0, 0, 0 } },
				DIR_UP1_MASK | DIR_LEFT1_MASK
		},

		{		{
				{ 0, 1, 0 },
				{ 0, 1, 1 },
				{ 0, 1, 0 } },
				DIR_UP1_MASK | DIR_RIGHT1_MASK | DIR_DOWN1_MASK
		},

		{		{
				{ 0, 0, 0 },
				{ 1, 1, 1 },
				{ 0, 1, 0 } },
				DIR_RIGHT1_MASK | DIR_DOWN1_MASK | DIR_LEFT1_MASK
		},

		{		{
				{ 0, 1, 0 },
				{ 1, 1, 0 },
				{ 0, 1, 0 } },
				DIR_UP1_MASK | DIR_LEFT1_MASK | DIR_DOWN1_MASK
		},

		{		{
				{ 0, 1, 0 },
				{ 1, 1, 1 },
				{ 0, 0, 0 } },
				DIR_UP1_MASK | DIR_LEFT1_MASK | DIR_RIGHT1_MASK
		},

		{		{
				{ 0, 1, 0 },
				{ 1, 1, 1 },
				{ 0, 1, 0 } },
				DIR_UP1_MASK | DIR_DOWN1_MASK | DIR_LEFT1_MASK | DIR_RIGHT1_MASK
		}
};

#define START_BRICK		1
#define END_BRICK		2
#define SUSPECT_BRICK	4

static class BrickConnect {
protected:
	unsigned long long bfm[2][64];
	int action[2][16][16];
	int bf[2][16][16];

protected:
	int fix(int dir, int brick0, int brick1, int & brick_fix) {
		bool check_fit = fit(dir, brick0, brick1);
		if (dir == 0) {			
			switch (brick1) {
			case BRICK_I_0:
				brick_fix = BRICK_i_180;
				if (brick0 != BRICK_VIA)
					return check_fit ? CONTINUE_TRACE1 : FIX_TRACE;
				else
					return START_TRACE0;
					
			case BRICK_i_0:
				brick_fix = BRICK_i_180;
				if (brick0 != BRICK_VIA)
					return check_fit ? END_TRACE1 : FIX_TRACE;
				else
					return START_TRACE0_END_TRACE1;
				
			case BRICK_i_180:

			case BRICK_T_90:
			case BRICK_L_90:
			case BRICK_L_180:
				brick_fix = BRICK_i_0;
				if (brick0 != BRICK_VIA)
					return check_fit ? START_TRACE1 : FIX_TRACE;
				else
					return START_TRACE1;
				
			case BRICK_NO_WIRE:
			case BRICK_ONE_POINT:
				brick_fix = BRICK_i_0;
				if (brick0 != BRICK_VIA)
					return check_fit ? NO_NEED_TRACE : FIX_TRACE;
				else
					return NO_NEED_TRACE;			

			case BRICK_X_0:
			case BRICK_T_0:
			case BRICK_T_180:
				brick_fix = BRICK_i_180;
				if (brick0 != BRICK_VIA)
					return check_fit ? END_START_TRACE1 : FIX_TRACE;
				else
					return START_TRACE0_END_START_TRACE1;

			case BRICK_T_270:
			case BRICK_L_0:
			case BRICK_L_270:
				brick_fix = BRICK_i_180;
				if (brick0 != BRICK_VIA)
					return check_fit ? END_TRACE1 : FIX_TRACE;
				else
					return START_TRACE0_END_TRACE1;

			case BRICK_I_90:
			case BRICK_i_90:
			case BRICK_i_270:
				brick_fix = BRICK_i_0;
				if (brick0 != BRICK_VIA)
					return check_fit ? NO_NEED_TRACE : FIX_TRACE;
				else
					return NO_NEED_TRACE;

			case BRICK_VIA:
				check_fit = fit(dir, brick0, BRICK_i_0);
				if (brick0 != BRICK_VIA)
					return check_fit ? END_TRACE1 : NO_NEED_TRACE;
				else
					return NO_NEED_TRACE;

			default:
				qCritical("invalid brick %d", brick1);
				return NO_NEED_TRACE;
				break;
			}
		}
		else {
			switch (brick1) {
			case BRICK_I_90:
				brick_fix = BRICK_i_90;
				if (brick0 != BRICK_VIA)
					return check_fit ? CONTINUE_TRACE1 : FIX_TRACE;
				else
					return START_TRACE0;

			case BRICK_i_270:
				brick_fix = BRICK_i_90;
				if (brick0 != BRICK_VIA)
					return check_fit ? END_TRACE1 : FIX_TRACE;
				else
					return START_TRACE0_END_TRACE1;

			case BRICK_i_90:

			case BRICK_T_0:
			case BRICK_L_0:
			case BRICK_L_90:
				brick_fix = BRICK_i_270;
				if (brick0 != BRICK_VIA)
					return check_fit ? START_TRACE1 : FIX_TRACE;
				else
					return START_TRACE1;

			case BRICK_NO_WIRE:
			case BRICK_ONE_POINT:
				brick_fix = BRICK_i_270;
				if (brick0 != BRICK_VIA)
					return check_fit ? NO_NEED_TRACE : FIX_TRACE;
				else
					return NO_NEED_TRACE;

			case BRICK_X_0:
			case BRICK_T_270:
			case BRICK_T_90:
				brick_fix = BRICK_i_90;
				if (brick0 != BRICK_VIA)
					return check_fit ? END_START_TRACE1 : FIX_TRACE;
				else
					return START_TRACE0_END_START_TRACE1;

			case BRICK_T_180:
			case BRICK_L_180:
			case BRICK_L_270:
				brick_fix = BRICK_i_90;
				if (brick0 != BRICK_VIA)
					return check_fit ? END_TRACE1 : FIX_TRACE;
				else
					return START_TRACE0_END_TRACE1;

			case BRICK_I_0:
			case BRICK_i_0:
			case BRICK_i_180:
				brick_fix = BRICK_i_270;
				if (brick0 != BRICK_VIA)
					return check_fit ? NO_NEED_TRACE : FIX_TRACE;
				else
					return NO_NEED_TRACE;

			case BRICK_VIA:
				check_fit = fit(dir, brick0, BRICK_i_270);
				if (brick0 != BRICK_VIA)
					return check_fit ? END_TRACE1 : NO_NEED_TRACE;
				else
					return NO_NEED_TRACE;

			default:
				qCritical("invalid brick %d", brick1);
				return NO_NEED_TRACE;
				break;
			}
		}
	}

public:
	enum {
		START_TRACE0,
		START_TRACE1,
		CONTINUE_TRACE1,
		END_TRACE1,
		END_START_TRACE1,
		START_TRACE0_END_TRACE1,
		START_TRACE0_END_START_TRACE1,
		NO_NEED_TRACE,
		FIX_TRACE
	};

	bool fit(int dir, int brick0, int brick1) {
		if (brick0 == BRICK_ONE_POINT)
			brick0 = BRICK_NO_WIRE;
		if (brick1 == BRICK_ONE_POINT)
			brick1 = BRICK_NO_WIRE;
		if (brick0 < sizeof(bricks) / sizeof(bricks[0]) && brick1 < sizeof(bricks) / sizeof(bricks[0]))
			return (bfm[dir][brick0] & 1ULL << brick1) ? true : false;
		else
			return false;
	}

	BrickConnect() {
		memset(bfm, 0, sizeof(bfm));
		for (int i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++)
			for (int j = 0; j < sizeof(bricks) / sizeof(bricks[0]); j++) {
			if (bricks[i].a[2][1] == bricks[j].a[0][1])
				bfm[0][i] |= 1ULL << j;
			if (bricks[i].a[1][2] == bricks[j].a[1][0])
				bfm[1][i] |= 1ULL << j;
			}

		for (int dir = 0; dir < 2; dir++)
			for (int i = 0; i <= BRICK_IN_USE; i++)
				for (int j = 0; j <= BRICK_IN_USE; j++)
					action[dir][i][j] = fix(dir, i, j, bf[dir][i][j]);				
	}

	int quick_fix(int dir, int brick0, int brick1, int & brick_fix) {
		if (brick0 == BRICK_FAKE_VIA)
			brick0 = BRICK_VIA;
		if (brick1 == BRICK_FAKE_VIA)
			brick1 = BRICK_VIA;
		if (brick0 <= BRICK_IN_USE && brick1 <= BRICK_IN_USE) {
			brick_fix = bf[dir][brick0][brick1];
			return action[dir][brick0][brick1];
		}
		else
			return fix(dir, brick0, brick1, brick_fix);
	}

} brick_conn;

enum {
	TYPE_GRAY_LEVEL,
	TYPE_COARSE_WIRE,
	TYPE_VIA_MASK,
	TYPE_FINE_WIRE_MASK,
	TYPE_VIA_LOCATION,
	TYPE_SHADOW_PROB
};

enum {
	VIA_SUBTYPE_2CIRCLE=0,
	VIA_SUBTYPE_3CIRCLE,
	VIA_SUBTYPE_4CIRCLE
};

enum {
	WIRE_SUBTYPE_13RECT=0
};

/*     31..24  23..16   15..8   7..0
opt0:		  subtype   type    shape
opt1:				  remove_rd guard
opt2:    rd3    rd2      rd1     rd0
opt3:   gray3  gray2    gray1   gray0
opt4:
opt5:
opt6:
*/
struct ViaParameter {
	int shape;
	int type;
	int subtype;
	int guard; //guard radius
	int remove_rd; //remove radius	
	int rd0; //inter via radius
	int rd1; //middle via radius
	int rd2; //outter via radius
	int rd3; //outter via radius
	int gray0, gray1, gray2, gray3;	
	int opt0, opt1, opt2;
	float optf;
};

/*     31..24  23..16   15..8   7..0
opt0:		  subtype   type    shape
opt1:							guard
opt2:  w_wide  w_wide1  w_high  w_high1
opt3:					i_wide  i_high   
opt4:           
opt5:          
opt6:
*/
struct WireParameter {
	int shape;
	int type;
	int subtype;
	int guard; //guard radius for unique
	int w_wide, w_wide1, w_high, w_high1; //normally w_high1 <w_high & w_wide1 <w_wide, these is for wire
	int i_wide, i_high;	//this is for insu
	int gray_w, gray_i; //this is not set by ProcessParameter
};

union VWParameter {
	struct ViaParameter v;
	struct WireParameter w;
};

class VWSet {
	vector <VWParameter> vws;
public:
	VWParameter * get_vw_para(int shape, int type, int sub_type) {
		for (int i = 0; i < (int)vws.size(); i++)
			if (vws[i].v.type == type && vws[i].v.shape == shape && vws[i].v.subtype == sub_type)
				return &vws[i];
		return NULL;
	}
	void set_vw_para(ProcessParameter cpara) {
		int shape = cpara.opt0 & 0xff;
		int type = cpara.opt0 >> 8 & 0xff;
		int sub_type = cpara.opt0 >> 16 & 0xff;
		VWParameter * vw = get_vw_para(shape, type, sub_type);
		if (vw == NULL) {
			VWParameter v;
			v.v.type = type;
			v.v.shape = shape;
			v.v.subtype = sub_type;
			vws.push_back(v);
			vw = get_vw_para(shape, type, sub_type);
		}
		switch (shape) {
		case BRICK_I_0:
		case BRICK_I_90:	
			vw->w.guard = cpara.opt1 & 0xff;
			vw->w.w_wide = cpara.opt2 >> 24 & 0xff;
			vw->w.w_wide1 = cpara.opt2 >> 16 & 0xff;
			vw->w.w_high = cpara.opt2 >> 8 & 0xff;
			vw->w.w_high1 = cpara.opt2 & 0xff;
			vw->w.i_wide = cpara.opt3 >> 8 & 0xff;
			vw->w.i_high = cpara.opt3 & 0xff;
			qInfo("set_wire_para, guard=%d, w_wide=%d, w_wide1=%d, w_high=%d, w_high1=%d, i_wide=%d, i_high=%d",
				vw->w.guard, vw->w.w_wide, vw->w.w_wide1, vw->w.w_high, vw->w.w_high1, vw->w.i_wide, vw->w.i_high);
			break;

		case BRICK_VIA:			
			vw->v.guard = cpara.opt1 & 0xff;
			vw->v.remove_rd = cpara.opt1 >> 8 & 0xff;			
			vw->v.rd0 = cpara.opt2 & 0xff;
			vw->v.rd1 = cpara.opt2 >> 8 & 0xff;
			vw->v.rd2 = cpara.opt2 >> 16 & 0xff;
			vw->v.rd3 = cpara.opt2 >> 24 & 0xff;
			vw->v.gray0 = cpara.opt3 & 0xff;
			vw->v.gray1 = cpara.opt3 >> 8 & 0xff;
			vw->v.gray2 = cpara.opt3 >> 16 & 0xff;
			vw->v.gray3 = cpara.opt3 >> 24 & 0xff;
			vw->v.opt0 = cpara.opt4;
			vw->v.opt1 = cpara.opt5;
			vw->v.opt2 = cpara.opt6;
			vw->v.optf = cpara.opt_f0;
			qInfo("set_via_para, guard=%d, remove_rd=%d, rd0=%d, rd1=%d, rd2=%d, rd3=%d",
				vw->v.guard, vw->v.remove_rd, vw->v.rd0, vw->v.rd1, vw->v.rd2, vw->v.rd3);
			break;
		}
	}
};

#define MAKE_PROB(s, x, y) ((unsigned long long)(s) << 32 | (y) << 16 | (x))
#define PROB_X(p) ((int)((p) & 0xffff))
#define PROB_Y(p) ((int)((p) >> 16 & 0xffff))
#define PROB_S(p) ((unsigned)((p) >> 32 & 0xffffffff))
#define PROB_SHAPE(p) ((unsigned)((p) >> 32 & 0xff))
#define PROB_TYPE(p) ((unsigned)((p) >> 40 & 0xff))
#define PROB_SCORE(p) ((unsigned)((p) >> 48 & 0xffffffff))
#define MAKE_S(score, type, shape) ((unsigned)(score) << 16 | (unsigned)(type) << 8 | (shape))
#define PROB_TYPESHAPE(p) ((unsigned)((p) >> 32 & 0xffff))
#define SET_PROB_TYPE(p, t) (p = (p & 0xffff00ffffffffffULL) | (unsigned long long) (t) << 40)
#define SET_PROB_X(p, x) (p = (p & 0xffffffffffff0000ULL) | (unsigned long long) (x))
#define SET_PROB_Y(p, y) (p = (p & 0xffffffff0000ffffULL) | (unsigned long long) (y) << 16)
#define SET_PROB_SCORE(p, score) (p = (p & 0x0000ffffffffffffULL) | (unsigned long long) (score) << 48)
#define SET_PROB_SHAPE(p, shape) (p = (p & 0xffffff00ffffffffULL) | (unsigned long long) (shape) << 32)

//Input x, y, score, for new prob.
//Input gs is grid size, should be match with prob
//return true if (x,y) prob is changed
bool push_new_prob(Mat & prob, int x, int y, unsigned score, unsigned gs) {	
	bool ret = false;
	int y0 = y / gs, x0 = x / gs;
	unsigned long long * p_prob = prob.ptr<unsigned long long>(y0, x0);
	CV_Assert(prob.type() == CV_64FC2 && y >= 0 && x >= 0 && x0 < prob.cols && y0 < prob.rows);
	unsigned long long new_s = MAKE_PROB(score, x, y);
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

void integral_square(const Mat & img, Mat & ig, Mat & iig, Mat & lg, Mat & llg, bool compute_line_integral);


class PipeDataPerLayer {
public:
	Mat img;
	Mat ig, iig, lg, llg;
	Mat prob;
	int gs;
	struct {
		int type;
		Mat d;
	} v[16];
	VWSet vw;
	bool ig_valid;
	int compute_border;

	//_gs * 2 <= _compute_border
	void reinit(int _gs, int _compute_border) {
		gs = _gs;
		compute_border = _compute_border;

		prob.create((img.rows - 1) / gs + 1, (img.cols - 1) / gs + 1, CV_64FC2);
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols * 2; x++)
				p_prob[x] = 0xffffffffffffffffULL;
		}
	}
	void set_raw_img(Mat & _img) {
		img = _img;
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
					CV_Assert(shape <= BRICK_IN_USE || shape == BRICK_VIA || shape == BRICK_ONE_POINT || shape == BRICK_FAKE_VIA);
				}
					
		}
	}
};

struct PipeData {
	vector<PipeDataPerLayer> l;
	int gs;
	int compute_border;
};
enum {
	GRAY_ZERO=0,
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
		int dy2dx2 = 2 * dy -2 * dx;
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
/*
Compute integrate and line integral
in img
in compute_line
out ig, same as openCV integral
out iig, sum(sum(img(i,j)*img(i,j), j=0..x-1) i=0..y-1
out lg, sum(img(i,j), j=0..x-1)
out llg, sum(img(i,j)*img(i,j), j=0..x-1)
*/
static void integral_square(const Mat & img, Mat & ig, Mat & iig, Mat & lg, Mat & llg, bool compute_line_integral)
{	
	CV_Assert(img.type() == CV_8UC1);
	ig.create(img.rows + 1, img.cols + 1, CV_32SC1);
	iig.create(img.rows + 1, img.cols + 1, CV_32SC1);
	if (compute_line_integral) {
		lg.create(img.rows, img.cols + 1, CV_32SC1);
		llg.create(img.rows, img.cols + 1, CV_32SC1);
	}
	for (int y = 0; y < ig.rows; y++) {
		unsigned * p_iig = iig.ptr<unsigned>(y);
		unsigned * p_ig = ig.ptr<unsigned>(y);
		if (y == 0)
			for (int x = 0; x < ig.cols; x++) {
				p_ig[x] = 0;
				p_iig[x] = 0;
			}				
		else {
			unsigned * p_iig_1 = iig.ptr<unsigned>(y - 1);
			unsigned * p_ig_1 = ig.ptr<unsigned>(y - 1);
			const unsigned char * p_img = img.ptr<const unsigned char>(y - 1);
			unsigned * p_lg = compute_line_integral ? lg.ptr<unsigned>(y - 1) : NULL;
			unsigned * p_llg = compute_line_integral ? llg.ptr<unsigned>(y - 1) : NULL;
			p_ig[0] = 0;
			p_iig[0] = 0;
			if (compute_line_integral) {
				p_lg[0] = 0;
				p_llg[0] = 0;
			}
			unsigned lsum = 0, lsum2 = 0;
			for (int x = 0; x < img.cols; x++) {
				unsigned img2 = p_img[x];
				lsum += img2;
				lsum2 += img2 * img2;
				p_ig[x + 1] = p_ig_1[x + 1] + lsum;
				p_iig[x + 1] = p_iig_1[x + 1] + lsum2;
				if (compute_line_integral) {
					p_lg[x + 1] = lsum;
					p_llg[x + 1] = lsum2;
				}
			}
		}
	}
}

/*
input points
output lut, 256 * 1
spline hermite
*/
static void cvt_tbl_hermite(const Mat & points, Mat & lut, int debug_en=0)
{
	CV_Assert(points.type() == CV_32SC1 && points.cols == 4);
	float x0=0, y0=0, x1=0, y1;
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
		if (x0!=x1)
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
			uchar * px = & p_lut[i * 8];
			qDebug("%3d:%d,%d,%d,%d,%d,%d,%d,%d", i*8, px[0], px[1], px[2], px[3], px[4], px[5], px[6], px[7]);
		}
}

class ViaComputeScore {
public:
	static ViaComputeScore * create_via_compute_score(ViaParameter &vp, PipeData &);

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

	virtual void prepare(ViaParameter &vp, PipeData &) = 0;
	virtual unsigned compute(int x0, int y0, const Mat & img, const Mat & lg, const Mat & llg) = 0;
	virtual ~ViaComputeScore() {
		qDebug("ViaComputeScore freed");
	}
};

class TwoCircleCompute : public ViaComputeScore {
protected:
	int r, r1;
	int gray, gray1;
	vector<int> dx, dx1;
	float a, a1, b, b1;

public:
	void prepare(ViaParameter &vp, PipeData &) {
		r = vp.rd0;
		r1 = vp.rd1;
		gray = vp.gray0;
		gray1 = vp.gray1;
		qInfo("TwoCirclePrepare r0=%d,r1=%d,g0=%d,g1=%d", r, r1, gray, gray1);
		int n = compute_circle_dx(r, dx);		
		int n1 = compute_circle_dx(r1, dx1);
		CV_Assert(n < n1 && r < r1);
		a = 1.0 / n;
		a1 = 1.0 / (n1 - n);
		b = (float) n / n1;
		b1 = 1 - b;
	}

	unsigned compute(int x0, int y0, const Mat &, const Mat & lg, const Mat & llg) {		
		int s = 0, ss = 0, s1 = 0, ss1 = 0;
		for (int y = -r; y <= r; y++) {
			int x = dx[abs(y)];
			s += lg.at<unsigned>(y + y0, x + x0 + 1) - lg.at<unsigned>(y + y0, x0 - x);
			ss += llg.at<unsigned>(y + y0, x + x0 + 1) - llg.at<unsigned>(y + y0, x0 - x);
		}
		for (int y = -r1; y <= r1; y++) {
			int x = dx1[abs(y)];
			s1 += lg.at<unsigned>(y + y0, x + x0 + 1) - lg.at<unsigned>(y + y0, x0 - x);
			ss1 += llg.at<unsigned>(y + y0, x + x0 + 1) - llg.at<unsigned>(y + y0, x0 - x);
		}
		float f[4];
		f[0] = s;
		f[1] = ss;
		f[2] = s1 - s;
		f[3] = ss1 - ss;
		unsigned score =((f[1] - f[0] * 2 * gray) * a + gray*gray) * b + ((f[3] - f[2] * 2 * gray1) * a1 + gray1*gray1) * b1;

		CV_Assert(score < 65536 && f[1] >= f[0] * f[0] * a && f[2] >= 0 && f[3] >= f[2] * f[2] * a);
		return score;
	}
};

class ThreeCircleCompute : public ViaComputeScore {
protected:
	int r, r1, r2;
	int gray, gray1, gray2;
	vector<int> dx, dx1, dx2;
	float a, a1, a2, b, b1, b2;

public:
	void prepare(ViaParameter &vp, PipeData &) {
		r = vp.rd0;
		r1 = vp.rd1;
		r2 = vp.rd2;
		gray = vp.gray0;
		gray1 = vp.gray1;
		gray2 = vp.gray2;
		qInfo("ThreeCirclePrepare r0=%d,r1=%d,r2=%d,g0=%d,g1=%d,g2=%d", r, r1, r2, gray, gray1, gray2);
		int n = compute_circle_dx(r, dx);		
		int n1 = compute_circle_dx(r1, dx1);		
		int n2 = compute_circle_dx(r2, dx2);
		CV_Assert(r < r1 && r1 < r2 && n < n1 && n1 < n2);
		a = 1.0 / n;
		a1 = 1.0 / (n1 - n);
		a2 = 1.0 / (n2 - n1);
		b = (float)n / n2;
		b1 = (float)(n1 - n) / n2;
		b2 = (float)(n2 - n1) / n2;
	}

	unsigned compute(int x0, int y0, const Mat &, const Mat & lg, const Mat & llg) {
		int s = 0, ss = 0, s1 = 0, ss1 = 0, s2 = 0, ss2 = 0;
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
		float f[6];
		f[0] = s;
		f[1] = ss;
		f[2] = s1 - s;
		f[3] = ss1 - ss;
		f[4] = s2 - s1;
		f[5] = ss2 - ss1;
		CV_Assert(f[2] >= 0 && f[4] >= 0);
		CV_Assert(f[1] >= f[0] * f[0] * a && f[3] >= f[2] * f[2] * a1 && f[5] >= f[4] * f[4] * a2);
		unsigned score = ((f[1] - f[0] * 2 * gray) * a + gray*gray) * b + 
			((f[3] - f[2] * 2 * gray1) * a1 + gray1*gray1) * b1 +
			((f[5] - f[4] * 2 * gray2) * a2 + gray2*gray2) * b2;
		CV_Assert(score < 65536);
		return score;
	}
};

class FourCircleCompute : public  ViaComputeScore {
protected:
	int r, r1, r2, r3;
	int gray, gray1, gray2, gray3;
	vector<int> dx, dx1, dx2, dx3;
	float a, a1, a2, a3, b, b1, b2, b3;

public:
	void prepare(ViaParameter &vp, PipeData &) {
		r = vp.rd0;
		r1 = vp.rd1;
		r2 = vp.rd2;
		r3 = vp.rd3;
		gray = vp.gray0;
		gray1 = vp.gray1;
		gray2 = vp.gray2;
		gray3 = vp.gray3;
		qInfo("FourCirclePrepare r0=%d,r1=%d,r2=%d,r3=%d,g0=%d,g1=%d,g2=%d,g3=%d", r, r1, r2, r3, gray, gray1, gray2, gray3);
		int n = compute_circle_dx(r, dx);
		int n1 = compute_circle_dx(r1, dx1);
		int n2 = compute_circle_dx(r2, dx2);
		int n3 = compute_circle_dx(r3, dx3);
		CV_Assert(r < r1 && r1 < r2 && r2 < r3 && n < n1 && n1 < n2 && n2 < n3);
		a = 1.0 / n;		
		a1 = 1.0 / (n1 - n);		
		a2 = 1.0 / (n2 - n1);		
		a3 = 1.0 / (n3 - n2);
		b = (float)n / n3;
		b1 = (float)(n1 - n) / n3;
		b2 = (float)(n2 - n1) / n3;
		b3 = (float)(n3 - n2) / n3;
	}

	unsigned compute(int x0, int y0, const Mat &, const Mat & lg, const Mat & llg) {
		int s = 0, ss = 0, s1 = 0, ss1 = 0;
		int s2 = 0, ss2 = 0, s3 = 0, ss3 = 0;
		for (int y = -r; y <= r; y++) {
			int x = dx[abs(y)];
			s += lg.at<unsigned>(y + y0, x + x0 + 1) - lg.at<unsigned>(y + y0, x0 - x);
			ss += llg.at<unsigned>(y + y0, x + x0 + 1) - llg.at<unsigned>(y + y0, x0 - x);
		}
		for (int y = -r1; y <= r1; y++) {
			int x = dx1[abs(y)];
			s1 += lg.at<unsigned>(y + y0, x + x0 + 1) - lg.at<unsigned>(y + y0, x0 - x);
			ss1 += llg.at<unsigned>(y + y0, x + x0 + 1) - llg.at<unsigned>(y + y0, x0 - x);
		}
		for (int y = -r2; y <= r2; y++) {
			int x = dx2[abs(y)];
			s2 += lg.at<unsigned>(y + y0, x + x0 + 1) - lg.at<unsigned>(y + y0, x0 - x);
			ss2 += llg.at<unsigned>(y + y0, x + x0 + 1) - llg.at<unsigned>(y + y0, x0 - x);
		}
		for (int y = -r3; y <= r3; y++) {
			int x = dx3[abs(y)];
			s3 += lg.at<unsigned>(y + y0, x + x0 + 1) - lg.at<unsigned>(y + y0, x0 - x);
			ss3 += llg.at<unsigned>(y + y0, x + x0 + 1) - llg.at<unsigned>(y + y0, x0 - x);
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
		CV_Assert(f[1] >= f[0] * f[0] * a && f[3] >= f[2] * f[2] * a1 && f[5] >= f[4] * f[4] * a2 && f[7] >= f[6] * f[6] * a3);
		unsigned score = ((f[1] - f[0] * 2 * gray) * a + gray*gray) * b + 
			((f[3] - f[2] * 2 * gray1) * a1 + gray1*gray1) * b1 +
			((f[5] - f[4] * 2 * gray2) * a2 + gray2*gray2) * b2 + 
			((f[7] - f[6] * 2 * gray3) * a3 + gray3*gray3) * b3;
		CV_Assert(score < 65536);
		return score;
	}
};

ViaComputeScore * ViaComputeScore::create_via_compute_score(ViaParameter &vp, PipeData & d)
{
	ViaComputeScore * vc = NULL;
	switch (vp.subtype) {
	case VIA_SUBTYPE_2CIRCLE:
		vc = new TwoCircleCompute();
		vc->prepare(vp, d);
		break;
	case VIA_SUBTYPE_3CIRCLE:
		vc = new ThreeCircleCompute();
		vc->prepare(vp, d);
		break;
	case VIA_SUBTYPE_4CIRCLE:
		vc = new FourCircleCompute();
		vc->prepare(vp, d);
		break;
	default:
		qCritical("ViaComputeScore create failed, subtype=%d", vp.subtype);
		break;
	}
	return vc;
}

class ViaRemove {	
public:
	static ViaRemove * create_via_remove(ViaParameter &vp, PipeData & d);

	static void compute_circle_dd(int r, vector<int> & dd)
	{
		dd.resize(r + 1);		
		for (int i = 0; i <= r; i++) 
			dd[i] = sqrt(r * r - i * i);
	}

	virtual void prepare(ViaParameter &vp, PipeData & d) = 0;
	//dir==0 means remove up-down style, else remove left-right style
	virtual void remove(Mat & img, int x0, int y0, int dir) = 0;
	virtual void remove_mask(Mat & mask, int x0, int y0) = 0;
	virtual ~ViaRemove() {
		qDebug("ViaRemove freed");
	}	
};

class ViaCircleRemove : public ViaRemove {
protected:
	int r;
	int w;
	vector<int> d;
public:
	void prepare(ViaParameter &vp, PipeData &) {
		r = vp.remove_rd;
		w = 3;
		compute_circle_dd(r, d);
	}

	void remove(Mat & img, int x0, int y0, int dir) {
		CV_Assert(img.type() == CV_8UC1 && x0 >= r+w && y0 >= r+w && x0+r+w < img.cols && y0+r+w < img.rows);
		if (dir == 0) {
			for (int x = x0 - r; x <= x0 + r; x++) {
				int y1 = y0 - d[abs(x - x0)];
				int y2 = y0 + d[abs(x - x0)];
				int a = 0, b = 0;
				for (int i = 1; i <= w; i++) {
					a += img.at<unsigned char>(y1 - i, x);
					b += img.at<unsigned char>(y2 + i, x);
				}
				a = a / w;
				b = b / w;
				for (int y = y1; y < y0; y++)
					img.at<unsigned char>(y, x) = a;
				for (int y = y0 + 1; y <= y2; y++)
					img.at<unsigned char>(y, x) = b;
				img.at<unsigned char>(y0, x) = (a + b) / 2;
			}
		}
		else {
			for (int y = y0 - r; y <= y0 + r; y++) {
				unsigned char * p_img = img.ptr<unsigned char>(y);
				int x1 = x0 - d[abs(y - y0)];
				int x2 = x0 + d[abs(y - y0)];
				int a = 0, b = 0;				
				for (int i = 1; i <= w; i++) {
					a += p_img[x1 - i];
					b += p_img[x2 + i];
				}
				a = a / w;
				b = b / w;
				for (int x = x1; x < x0; x++)
					p_img[x] = a;
				for (int x = x0 + 1; x <= x2; x++)
					p_img[x] = b;
				p_img[x0] = (a + b) / 2;
			}
		}
	}

	void remove_mask(Mat & mask, int x0, int y0) {
		CV_Assert(mask.type() == CV_32SC1 && x0 >= r && y0 >= r && x0 + r < mask.cols && y0 + r < mask.rows);
		for (int y = y0 - r; y <= y0 + r; y++) {
			unsigned * p_mask = mask.ptr<unsigned>(y);
			int x1 = x0 - d[abs(y - y0)];
			int x2 = x0 + d[abs(y - y0)];
			for (int x = x1; x <= x2; x++)
				p_mask[x] = 0;
		}
	}
};

ViaRemove * ViaRemove::create_via_remove(ViaParameter &vp, PipeData & d)
{
	ViaRemove * vr = NULL;
	switch (vp.subtype) {
	case VIA_SUBTYPE_2CIRCLE:
	case VIA_SUBTYPE_3CIRCLE:
	case VIA_SUBTYPE_4CIRCLE:
		vr = new ViaCircleRemove();
		vr->prepare(vp, d);
		break;
	default:
		qCritical("ViaRemove create failed, subtype=%d", vp.subtype);
		break;
	}
	return vr;
}

typedef pair<unsigned long long, unsigned long long> PAIR_ULL;

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
	virtual void prepare(WireParameter & _wp, PipeData & d, int layer) = 0;
	virtual PAIR_ULL compute(int x0, int y0, const Mat & img, const Mat & ig, const Mat & iig) = 0;
	virtual int check_mark(int x0, int y0, const Mat & prob, Mat mask, int subtype) = 0;
};


struct ShapeConst {
	int use[13];
	int sel[13];
	int shape;
	float a[13]; //a is each rec_area / total_area
} shape[] = {
	{ { 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_ONE_POINT, { 0 } },
	{ { 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_i_0, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, { 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0 }, BRICK_i_90, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0 }, BRICK_i_180, { 0 } },
	{ { 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_i_270, { 0 } },
	{ { 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }, { 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0 }, BRICK_I_0, { 0 } },
	{ { 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, { 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 }, BRICK_I_90, { 0 } },
	{ { 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, { 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0 }, BRICK_L_0, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_L_90, { 0 } },
	{ { 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }, { 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0 }, BRICK_L_180, { 0 } },
	{ { 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_L_270, { 0 } },
	{ { 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_T_0, { 0 } },
	{ { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_T_90, { 0 } },
	{ { 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0 }, BRICK_T_180, { 0 } },
	{ { 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 }, BRICK_T_270, { 0 } },
	{ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_X_0, { 0 } },

	{ { 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, { 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }, { 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, { 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, { 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }, { 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 }, BRICK_HOLLOW, { 0 } },
	{ { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, BRICK_NO_WIRE, { 0 } },
	{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1 }, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1 }, BRICK_FAKE_VIA, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }, BRICK_FAKE_VIA, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1 }, BRICK_FAKE_VIA, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0 }, BRICK_FAKE_VIA, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1 }, BRICK_FAKE_VIA, { 0 } },
	{ { 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0 }, BRICK_FAKE_VIA, { 0 } },
};

class Wire_13RectCompute : public WireComputeScore {
protected:
	bool reset_offset;
	int offset[24], dx[24], dy[24]; //offset is compute based on dx and dy
	float area_1[13]; //area is 1 / rec_area
	int searchx_extend, searchy_extend;
	int gs, compute_border;
	WireParameter wp;

public:
	void prepare(WireParameter & _wp, PipeData & d, int layer) {
		reset_offset = true;
		wp = _wp;
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

		for (int i = 0; i < sizeof(shape) / sizeof(shape[0]); i++) {
			float area = 0;
			for (int j = 0; j < 13; j++)
				area += shape[i].use[j] * area_1[j];

			for (int j = 0; j < 13; j++)
				shape[i].a[j] = shape[i].use[j] * area_1[j] / area;
		}

		for (int i = 0; i < 13; i++)
			area_1[i] = 1 / area_1[i];

		gs = d.l[layer].gs;
		compute_border = d.l[layer].compute_border;
		searchx_extend = (wp.w_wide + wp.w_wide1 + wp.i_wide) / gs;
		searchy_extend = (wp.w_high + wp.w_high1 + wp.i_high) / gs;
	}

	//return 1st and 2nd likelihood brick for img, (x0,y0)
	PAIR_ULL compute(int x0, int y0, const Mat & , const Mat & ig, const Mat & iig) {
		PAIR_ULL ret = make_pair(0x100000000ULL, 0x100000000ULL);
		unsigned s[24], sq[24];
		int sum[13], ssum[13];
		float part[2][13];
		const unsigned * p_ig, *p_iig;
		if (reset_offset) {
			reset_offset = false;
			for (int i = 0; i < 24; i++)
				offset[i] = (dy[i] * ig.step.p[0] + dx[i] * ig.step.p[1]) / sizeof(unsigned);
		}
		p_ig = ig.ptr<unsigned>(y0, x0);
		p_iig = iig.ptr<unsigned>(y0, x0);
		for (int i = 0; i < 24; i++) {
			s[i] = p_ig[offset[i]];
			sq[i] = p_iig[offset[i]];
		}
		sum[0] = s[0] + s[4] - s[1] - s[3];
		ssum[0] = sq[0] + sq[4] - sq[1] - sq[3];
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
			part[0][i] = (ssum[i] - gi*sum[i] * 2) * area_1[i] + gii;
			part[1][i] = (ssum[i] - gm*sum[i] * 2) * area_1[i] + gmm;
		}
		
		for (int i = 0; i < sizeof(shape) / sizeof(shape[0]); i++) {
			unsigned score = 0;
			for (int j = 0; j < 13; j++)
				score += part[shape[i].sel[j]][j] * shape[i].a[j];
			score = MAKE_S(score, wp.type, shape[i].shape);
			push_min(ret, score);
		}
		ret.first = MAKE_PROB(ret.first, x0, y0);
		ret.second = MAKE_PROB(ret.second, x0, y0);
		return ret;
	}

	int check_mark(int x0, int y0, const Mat & prob, Mat mask, int subtype)
	{
		int ret = 0;
		CV_Assert(prob.rows * gs >= mask.rows && prob.cols * gs >= mask.cols && 
			mask.type() ==CV_32SC1 && prob.type() ==CV_64FC2);
		unsigned long long prob0 = prob.at<unsigned long long>(y0, 2 * x0);
		int b = PROB_SHAPE(prob0);
		if (b > BRICK_IN_USE)
			return ret;
		int x = PROB_X(prob0), y = PROB_Y(prob0);
		int xx[3], yy[3];
		for (int dir = 0; dir <= 3; dir++) {
			if (!bricks[b].a[dxy[dir][0] + 1][dxy[dir][1] + 1])//if need to check extend
				continue;
			int extend = (dir == DIR_UP || dir == DIR_DOWN) ? searchy_extend : searchx_extend;
			int xx0 = x0 + dxy[dir][1], yy0 = y0 + dxy[dir][0];
			//following check if extend is already computed
			bool check = false;
			for (int i = 0; i < extend; i++) {	
				if (yy0 <= compute_border / gs || yy0 >= (mask.rows - compute_border) / gs ||
					xx0 <= compute_border / gs || xx0 >= (mask.cols - compute_border) / gs) {
					check = true;
					break;
				}
					
				if (PROB_S(prob.at<unsigned long long>(yy0, xx0 * 2)) != 0xffffffff)
					check = true;
				xx0 += dxy[dir][1];
				yy0 += dxy[dir][0];
			}
			if (check) //if already compute, continue
				continue;
			switch (dir) {
			case DIR_UP:
				xx[0] = x, xx[1] = x, xx[2] = x;
				yy[0] = y - wp.w_high / 2 - wp.i_high - wp.w_high1;
				yy[1] = yy[0] - 1;
				yy[2] = yy[1] - 1;
				break;
			case DIR_RIGHT:
				yy[0] = y, yy[1] = y, yy[2] = y;
				xx[0] = x + wp.w_wide1 + wp.i_wide + wp.w_wide - wp.w_wide / 2;
				xx[1] = xx[0] + 1;
				xx[2] = xx[1] + 1;
				break;
			case DIR_DOWN:
				xx[0] = x, xx[1] = x, xx[2] = x;
				yy[0] = y + wp.w_high - wp.w_high / 2 + wp.i_high + wp.w_high1;
				yy[1] = yy[0] + 1;
				yy[2] = yy[1] + 1;
				break;
			case DIR_LEFT:
				yy[0] = y, yy[1] = y, yy[2] = y;
				xx[0] = x - wp.w_wide / 2 - wp.i_wide - wp.w_wide1;
				xx[1] = xx[0] - 1;
				xx[2] = xx[1] - 1;
				break;
			}
#if 1
			qInfo("new mark x=%d, y=%d", xx[0], yy[0]);
#endif
			for (int i = 0; i < 3; i++)
				mask.at<int>(yy[i], xx[i]) |= 1 << subtype;
			ret++;
		}
		return ret;
	}
};

WireComputeScore * WireComputeScore::create_wire_compute_score(WireParameter &wp, PipeData & d, int layer)
{
	WireComputeScore * wc = NULL;
	switch (wp.subtype) {
	case WIRE_SUBTYPE_13RECT:
		wc = new Wire_13RectCompute;
		wc->prepare(wp, d, layer);
		break;
	default:
		qCritical("WireComputeScore create failed, subtype=%d", wp.subtype);
		break;
	}
	return wc;
}

/*		31..24  23..16   15..8   7..0
opt0:				compute_border gs
*/

void set_pipeline_param(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	qInfo("set param for layer %d", layer);
	if (layer == -1) {
		d.gs = cpara.opt0 & 0xff;
		d.compute_border = cpara.opt0 >> 8 & 0xff;
		qInfo("set gs=%d, compute_border=%d", d.gs, d.compute_border);

		if (d.gs > 32 || d.gs<=3 || d.gs * 2 > d.compute_border) 
			qCritical("gs invalid");
		else {
			for (int i = 0; i < d.l.size(); i++)
				d.l[i].reinit(d.gs, d.compute_border);
		}
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
void imgpp_RGB2gray(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC3);
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
	if (cpara.method & OPT_DEBUG_EN) 
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_imgpp_RGB2gray.jpg", d.l[layer].img);
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
 d.img = d.img - filter_min 
*/
void imgpp_compute_min_stat(PipeData & d, ProcessParameter & cpara)
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
				x1 = (x1+1 >= filter_min.cols) ? filter_min.cols - 1 : x1 + 1;
			}
			p_img[x] = (p_img[x] > p_min[x1]) ? p_img[x] - p_min[x1] : 0;		
		}
	}
	d.l[layer].ig_valid = false;
	if (cpara.method & OPT_DEBUG_EN) {
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_imgpp_compute_min_stat_out.jpg", img);
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_imgpp_compute_min_stat_min.jpg", filter_min);
	}
}

/*     31..24  23..16   15..8   7..0
opt0: dis_min  wsize  sep_max  sep_min for most dark obj (may or may not be insu)
opt1: dis_min  wsize  sep_max  sep_min for middle bright obj (may or may not be wire)
opt2: dis_max  wsize  sep_max  sep_min for most bright obj (may or may not be via)
opt3:           k2       k1      k0      k0 is dark gray level compress rate, k1 & k2 is rate between most dark and middle bright
opt4:           k2       k1      k0      k0 is middle bright level compress rate, k1 & k2 is rate between middle bright and most bright
opt5:           k2       k1      k0      k0 is most bright compress rate, k1 & k2 is rate between most bright
sep_min & sep_max mins gray level difference
Choose gray level between [last gray level + sep_min, last gray level + sep_max]
normally choose k2 & k0 = 100, k1 < 100
method_opt
0: for gray level turn_points output
*/
void imgpp_adjust_gray_lvl(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);
	
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

	vector<int> stat(257, 0);
	for (int y = 0; y < img.rows; y += 2) {
		const unsigned char * p_img = img.ptr<unsigned char>(y);
		for (int x = 0; x < img.cols; x++) {
			stat[p_img[x]]++;
			stat[256]++;
		}
	}
	int idx0 = cpara.method_opt & 0xf;
	qInfo("imgpp_adjust_gray_lvl l=%d, tp_idx=%d", layer, idx0);
	if (cpara.method & OPT_DEBUG_EN) {
		qDebug("imgpp_adjust_gray_lvl gray distribution");
		for (int i = 0; i < 32; i++) {
			int * ps = &stat[i * 8];
			qDebug("%3d:%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f", i*8, 
				(float)ps[0] / stat[256], (float)ps[1] / stat[256], (float)ps[2] / stat[256], (float)ps[3] / stat[256],
				(float)ps[4] / stat[256], (float)ps[5] / stat[256], (float)ps[6] / stat[256], (float)ps[7] / stat[256]);
		}
	}

	//following is compute lmh[0..2].g0
	int base = 0;
	for (int i = 0; i < 3; i++) {
		int choose = base, max_sum = 0;
		qInfo("imgpp_adjust_gray_lvl %d:sep_min=%d, sep_max=%d,wsize=%d,dis_min=%f,dis_max=%f, k0=%f, k1=%f, k2=%f", i, 
			lmh[i].sep_min, lmh[i].sep_max, lmh[i].wsize, lmh[i].dis_min, lmh[i].dis_max, lmh[i].k0, lmh[i].k1, lmh[i].k2);
		if (lmh[i].wsize > 100 || lmh[i].dis_min >= 1 || lmh[i].dis_max > 1 || lmh[i].k0 > 1) {
			qCritical("imgpp_adjust_gray_lvl wrong para");
			return;
		}
		for (int j = base + lmh[i].sep_min; j <= base + lmh[i].sep_max; j++) {
			int kb = max(base + 1, j - lmh[i].wsize / 2), ke = min(j + (lmh[i].wsize - 1) / 2, 255);
			int sum = 0, th = lmh[i].dis_max * stat[256];
			for (int k = kb; k <= ke; k++)
				sum += stat[k];
			if (sum > max_sum && sum < th) {
				max_sum = sum;
				choose = j;
			}
		}
		if (max_sum < lmh[i].dis_min * stat[256])
			qCritical("Error choose=%d, max_sum(%d) < dis_min(%d)", choose, max_sum, (int) (lmh[i].dis_min * stat[256]));
		
		lmh[i].g0 = choose;
		base = choose + (lmh[i].wsize - 1) / 2;
	}
	qInfo("imgpp_adjust_gray_lvl, l0=%d, m0=%d, h0=%d", lmh[0].g0, lmh[1].g0, lmh[2].g0);

	//following compute Gray level table
	d.l[layer].v[idx0].type = TYPE_GRAY_LEVEL;
	Mat & m = d.l[layer].v[idx0].d;
	m.create(11, 5, CV_32SC1);
	m.at<int>(0, 0) = GRAY_ZERO;
	m.at<int>(0, 1) = 0;
	m.at<int>(0, 2) = 0;
	m.at<int>(1, 1) = 1;
	m.at<int>(1, 2) = 1;
	m.at<int>(1, 0) = GRAY_L1;
	m.at<int>(1, 1) = max(0, lmh[0].g0 - lmh[0].wsize / 2);
	m.at<int>(1, 2) = max(0, lmh[0].g0 - (int) (lmh[0].wsize * lmh[0].k0 / 2));
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
	m.at<int>(9, 2) = min(255, lmh[2].g0 + (int) ((lmh[2].wsize - 1) * lmh[2].k0 / 2));
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
	if (cpara.method & OPT_DEBUG_EN)
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_imgpp_adjust_gray_lvl.jpg", img);
}

/*     31..24 23..16   15..8   7..0
opt0:   inc1   inc0   w_long1 w_long0 
opt1:	up_prob	th1     th0	  w_num
opt2:                  w_dir  w_type    
opt3:                  w_dir  w_type    
opt4:                  w_dir  w_type    
opt5:                  w_dir  w_type    
dir=0 is shuxian, 1 is hengxian.
long0 & long1 decides detect rect, inc0 & & inc1 reduce compute time
th0 & th1 is maximum deviation
up_diff means update d.l[layer].prob
method_opt
0: for gray level turn_points  input
1: for prob output
It assume wire is lighter than insu, it will update d.prob
*/
void coarse_line_search(PipeData & d, ProcessParameter & cpara)
{	
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	int idx = cpara.method_opt & 0xf;
	int idx1 = cpara.method_opt >> 4 & 0xf;
	CV_Assert(img.type() == CV_8UC1);
	if (d.l[layer].v[idx].type != TYPE_GRAY_LEVEL) {
		qCritical("coarse_line_search gray level idx[%d]=%d, error", idx, d.l[layer].v[idx].type);
		return;
	}
	int w_long0 = cpara.opt0 & 0xff, w_long1 = cpara.opt0 >> 8 & 0xff;
	int w_inc0 = cpara.opt0 >> 16 & 0xff, w_inc1 = cpara.opt0 >> 24 & 0xff;
	int th0 = cpara.opt1 >> 8 & 0xff, th1 = cpara.opt1 >> 16 & 0xff;
	int w_num = cpara.opt1 & 0xff;
	int update_prob = cpara.opt1 >> 24 & 0xff;
	qInfo("coarse_line_search, l=%d, tp_idx=%d, prob_idx=%d, w_long0=%d,w_long1=%d, w_inc0=%d,w_inc1=%d, w_num=%d, th0=%d, th1=%d, up_prob=%d", 
		layer, idx, idx1, w_long0, w_long1, w_inc0, w_inc1, w_num, th0, th1, update_prob);

	struct WireDetectInfo {
		int w_dir;
		int w_type;
		int w_wide;
		int i_wide;
		int guard;
	} wpara[] = {
			{ cpara.opt2 >> 8 & 0xff, cpara.opt2 & 0xff , 0, 0, 0}, 
			{ cpara.opt3 >> 8 & 0xff, cpara.opt3 & 0xff , 0, 0, 0},
			{ cpara.opt4 >> 8 & 0xff, cpara.opt4 & 0xff , 0, 0, 0},
			{ cpara.opt5 >> 8 & 0xff, cpara.opt5 & 0xff , 0, 0, 0},
	};
	struct WireKeyPoint {
		Point offset[8];
		int * p_ig[8];
		int *p_iig[8];
		int type;
		int shape1, shape2, shape3;
		float a0, a1, a2;
	};
	vector<WireKeyPoint> wires[2];
	int w_check[2][256] = { 0 };
	int x0 = w_long1 / 2, y0 = w_long0 / 2;
	for (int i = 0; i < w_num; i++) {
		VWParameter * vw = d.l[layer].vw.get_vw_para((wpara[i].w_dir == 0) ? BRICK_I_0 : BRICK_I_90, wpara[i].w_type, WIRE_SUBTYPE_13RECT);
		if (vw == NULL) {
			qCritical("coarse_line_search invalid wire info %d, %d", wpara[i].w_dir, wpara[i].w_type);
			return;
		}
		wpara[i].w_wide = (wpara[i].w_dir == 0) ? vw->w.w_wide : vw->w.w_high;
		wpara[i].i_wide = (wpara[i].w_dir == 0) ? vw->w.i_wide : vw->w.i_high;
		wpara[i].guard = vw->w.guard;
		qInfo("%d:type=%d, dir=%d, w_wide=%d, i_wide=%d, guard=%d", i, wpara[i].w_type,
			wpara[i].w_dir, wpara[i].w_wide, wpara[i].i_wide, wpara[i].guard);
		if (wpara[i].w_dir > 3 || wpara[i].w_wide + wpara[i].i_wide >= d.l[layer].compute_border || 
			wpara[i].guard >= wpara[i].w_wide / 2 + wpara[i].i_wide - 1) {
			qCritical("coarse_line_search wrong para");
			return;
		}
		
		WireKeyPoint wire;		
		switch (wpara[i].w_dir) {
		case 0:
			wire.offset[0] = Point(-wpara[i].w_wide / 2 - wpara[i].i_wide, - w_long0 / 2);
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
			wire.shape1 = BRICK_I_0;
			wire.shape2 = BRICK_II_0;
			wire.shape3 = BRICK_II_180;
			wires[0].push_back(wire);
			x0 = max(x0, wpara[i].w_wide / 2 + wpara[i].i_wide + 1);
			w_check[0][wpara[i].w_type] = wpara[i].guard;
			break;
		case 1:
			wire.offset[0] = Point(-w_long1 / 2, - wpara[i].w_wide / 2 - wpara[i].i_wide);
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
			wire.shape1 = BRICK_I_90;
			wire.shape2 = BRICK_II_90;
			wire.shape3 = BRICK_II_270;
			wires[1].push_back(wire);
			y0 = max(y0, wpara[i].w_wide / 2 + wpara[i].i_wide + 1);
			w_check[1][wpara[i].w_type] = wpara[i].guard;
			break;
		default:
			qCritical("bad dir %d", wpara[i].w_dir);
			break;
		}

	}

	d.l[layer].validate_ig();

	int gl = find_index(d.l[layer].v[idx].d, (int) GRAY_L0);
	gl = d.l[layer].v[idx].d.at<int>(gl, 2);
	int gm = find_index(d.l[layer].v[idx].d, (int) GRAY_M0);
	gm = d.l[layer].v[idx].d.at<int>(gm, 2);
	qInfo("coarse_line_search, gl=%d, gm=%d", gl, gm);
	Mat & ig = d.l[layer].ig;
	Mat & iig = d.l[layer].iig;
	d.l[layer].v[idx1].type = TYPE_SHADOW_PROB;
	d.l[layer].v[idx1].d = d.l[layer].prob.clone();
	Mat & prob = d.l[layer].v[idx1].d;
	
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
				unsigned s0 = 0xffffffff;
				for (unsigned i = 0; i < w.size(); i++) {
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
					score0 = MAKE_S(score0, w[i].type, BRICK_NO_WIRE);
					score1 = MAKE_S(score1, w[i].type, w[i].shape1);
					score2 = MAKE_S(score2, w[i].type, w[i].shape2);
					score3 = MAKE_S(score3, w[i].type, w[i].shape3);
					score4 = MAKE_S(score4, w[i].type, BRICK_III);
					s0 = min(min(min(score1, score0), min(score2, score3)), min(score4, s0));
					for (int j = 0; j < 8; j++) {
						w[i].p_ig[j] += dx;
						w[i].p_iig[j] += dx;
					}
				}
				push_new_prob(prob, x, y, s0, d.l[layer].gs);
				if ((s0 & 0xff) == BRICK_NO_WIRE && update_prob)
					push_new_prob(d.l[layer].prob, x, y, s0, d.l[layer].gs);
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
			if (PROB_SHAPE(p_prob[2 * x]) == BRICK_I_0 && PROB_SCORE(p_prob[2 * x]) <= th0 * th0 ||
				PROB_SHAPE(p_prob[2 * x]) == BRICK_I_90 && PROB_SCORE(p_prob[2 * x]) <= th1 * th1) {
			bool pass = true;
			unsigned long long prob0 = p_prob[2 * x];
			int cr = w_check[0][PROB_TYPE(prob0)];
			int guard = (cr - 1) / d.l[layer].gs + 1;
			int y1 = max(0, y - guard), y2 = min(prob.rows - 1, y + guard);
			int x1 = max(0, x - guard), x2 = min(prob.cols - 1, x + guard);
			for (int yy = y1; yy <= y2; yy++)  {
				unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
				for (int xx = x1; xx <= x2; xx++) {
					if (p_prob2[2 * xx] < prob0 && PROB_SHAPE(p_prob2[2 * xx]) != PROB_SHAPE(prob0)) {
						if (abs(PROB_X(p_prob2[2 * xx]) - PROB_X(prob0)) <= cr &&
							abs(PROB_Y(p_prob2[2 * xx]) - PROB_Y(prob0)) <= cr) {
							pass = false;							
							yy = y2;
							break;
						}
					}
				}
			}
			if (PROB_SHAPE(p_prob[2 * x]) == BRICK_I_0) {
				for (int xx = x1; xx <= x2; xx++)
					if (p_prob[2 * xx] < prob0 && PROB_SHAPE(p_prob[2 * xx]) == BRICK_I_0) {
					if (abs(PROB_X(p_prob[2 * xx]) - PROB_X(prob0)) <= cr)
						pass = false;
					}
			}
			else {
				for (int yy = y1; yy <= y2; yy++) {
					unsigned long long prob1 = prob.at<unsigned long long>(y, 2 * x);
					if (prob1 < prob0 && PROB_SHAPE(prob1) == BRICK_I_90) {
						if (abs(PROB_Y(prob1) - PROB_Y(prob0)) <= cr)
							pass = false;
					}
				}
			}
			if (!pass)
				continue;
			if (update_prob)
				push_new_prob(d.l[layer].prob, p_prob[2 * x], d.l[layer].gs);
			if (cpara.method & OPT_DEBUG_EN)
				circle(debug_draw, Point(PROB_X(p_prob[2*x]), PROB_Y(p_prob[2*x])), 2, Scalar::all(255));
		}
	}	

	if (cpara.method & OPT_DEBUG_EN) {
		d.l[layer].check_prob();
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_coarse_line.jpg", debug_draw);
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
void coarse_via_search_mask(PipeData & d, ProcessParameter & cpara)
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
		if (v_wide[i] >= d.l[i].compute_border || v_percent[i] >= 50) {
			qCritical("coarse_via_search_mask wrong parameter");
			return;
		}
	}

	d.l[layer].v[idx].type = TYPE_VIA_MASK;
	Mat & mask = d.l[layer].v[idx].d;
	mask.create(img.rows, img.cols, CV_32S);
	mask = Scalar::all(0);
	d.l[layer].validate_ig();
	vector<unsigned> record[1025];
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

		total = total * v_percent[i] / 100;
		for (int j = 1023; j > 0; j--) {
			for (int k = 0; k < record[j].size(); k++) {
				int y = record[j][k] >> 16;
				int x = record[j][k] & 0xffff;
				mask.at<int>(y, x) |= 1 << v_subtype[i];
			}				
			total = total - (int) record[j].size();
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
	}
#undef MAX_VIA_NUM
}

/*
        31..24 23..16   15..8   7..0
opt0:							vnum
opt1:					subtype	type
opt2:					subtype	type
opt3:					subtype	type
opt4:					subtype	type
opt5:					subtype	type
method_opt
0: for gray level turn_points  input
1: for via search mask input
2: for via info output output
3: for shadow prob index input
*/
void fine_via_search(PipeData & d, ProcessParameter & cpara)
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
	int idx3 = cpara.method_opt >> 12 & 0xf;
	if (d.l[layer].v[idx3].type != TYPE_SHADOW_PROB) {
		qCritical("fine_via_search search mask idx3[%d], error", idx3, d.l[layer].v[idx3].type);
		return;
	}
	Mat & mask = d.l[layer].v[idx1].d;
	int vnum = cpara.opt0 & 0xff;
	qInfo("fine_via_search, l=%d, g_idx=%d, vmsk_idx=%d, prob_idx=%d, gl=%d, gm=%d, gh=%d, vnum=%d", 
		layer, idx, idx1, idx3, glv[0], glv[1], glv[2], vnum);
	
#define MAX_VIA_NUM 3	
	if (vnum > MAX_VIA_NUM || vnum==0) {
		qCritical("fine_via_search wrong parameter");
		return;
	}
	int v_type[MAX_VIA_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff };
	int v_subtype[MAX_VIA_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt3 >> 8 & 0xff };
	int v_guard[MAX_VIA_NUM];
	QScopedPointer<ViaComputeScore> vcs[MAX_VIA_NUM];

	for (int i = 0; i < vnum; i++) {
		ViaParameter via_para;
		via_para = d.l[layer].vw.get_vw_para(BRICK_VIA, v_type[i], v_subtype[i])->v;
		v_guard[i] = via_para.guard;
		if (via_para.gray0 < sizeof(glv) / sizeof(glv[0]))
			via_para.gray0 = glv[via_para.gray0];
		if (via_para.gray1 < sizeof(glv) / sizeof(glv[0]))
			via_para.gray1 = glv[via_para.gray1];
		if (via_para.gray2 < sizeof(glv) / sizeof(glv[0]))
			via_para.gray2 = glv[via_para.gray2];
		if (via_para.gray3 < sizeof(glv) / sizeof(glv[0]))
			via_para.gray3 = glv[via_para.gray3];
		qInfo("%d: fine_via_search v_type=%d, v_subtype=%d, g0=%d, g1=%d, g2=%d, g3=%d", i, v_type[i], 
			v_subtype[i], via_para.gray0, via_para.gray1, via_para.gray2, via_para.gray3);
		vcs[i].reset(ViaComputeScore::create_via_compute_score(via_para, d));
	}
	d.l[layer].validate_ig();
	Mat & prob = d.l[layer].v[idx3].d;
	for (int y = d.l[layer].compute_border; y < img.rows - d.l[layer].compute_border; y++) {
		unsigned * p_mask = mask.ptr<unsigned>(y);
		for (int x = d.l[layer].compute_border; x < img.cols - d.l[layer].compute_border; x++)
			if (p_mask[x]) {
				unsigned score = 0xffffffff;
				for (int i = 0; i < vnum; i++) 
				if (!vcs[i].isNull() && (p_mask[x] >> v_subtype[i] & 1)) {
					unsigned s = vcs[i]->compute(x, y, img, d.l[layer].lg, d.l[layer].llg);
					s = MAKE_S(s, i, BRICK_VIA);
					score = min(score, s);
				}
				push_new_prob(prob, x, y, score, d.l[layer].gs);
			}
	}

	struct ViaInfo {
		Point xy;
		int type, subtype;
		ViaInfo(Point _xy, int _type, int _subtype) {
			xy = _xy;
			type = _type;
			subtype = _subtype;
		}
	};
	//choose local unique via
	vector<ViaInfo> via_loc;
	for (int y = 0; y < prob.rows; y++) {
		unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
		for (int x = 0; x < prob.cols; x++)
			if (PROB_SHAPE(p_prob[2 * x]) == BRICK_VIA || PROB_SHAPE(p_prob[2 * x + 1]) == BRICK_VIA) {
				unsigned long long prob0 = (PROB_SHAPE(p_prob[2 * x]) == BRICK_VIA) ? p_prob[2 * x] : p_prob[2 * x + 1];
				bool pass = true; //pass means it is lowest prob among nearby guard probs
				bool pass2 = true; //pass2 means it is lowest prob among nearby guard via probs
				int gi = PROB_TYPE(prob0);
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
				if (pass2) {				
					unsigned long long score = prob0;
					SET_PROB_TYPE(score, v_type[gi]);
					push_new_prob(d.l[layer].prob, score, d.l[layer].gs);
				}
				if (pass)
					via_loc.push_back(ViaInfo(Point(PROB_X(prob0), PROB_Y(prob0)), v_type[gi], v_subtype[gi]));
			}
	}
	Mat debug_draw;
	if (cpara.method & OPT_DEBUG_EN)
		debug_draw = img.clone();
	
	int idx2 = cpara.method_opt >> 8 & 0xf;
	d.l[layer].v[idx2].type = TYPE_VIA_LOCATION;
	Mat & m = d.l[layer].v[idx2].d;
	m.create((int)via_loc.size(), 4, CV_32S);
	for (int i = 0; i < (int)via_loc.size(); i++) {
		m.at<int>(i, 0) = via_loc[i].type;
		m.at<int>(i, 1) = via_loc[i].subtype;
		m.at<int>(i, 2) = via_loc[i].xy.x;
		m.at<int>(i, 3) = via_loc[i].xy.y;
		if (cpara.method & OPT_DEBUG_EN)
			circle(debug_draw, via_loc[i].xy, 5, Scalar::all(0));
	}
	if (cpara.method & OPT_DEBUG_EN) {
		d.l[layer].check_prob();
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_via.jpg", debug_draw);
	}
#undef MAX_VIA_NUM
}


/*
        31..24 23..16   15..8   7..0
opt0:	cr_mask default_dir check_len vnum
opt1:					subtype type
opt2:					subtype	type
opt3:					subtype	type
opt4:					subtype	type
opt5:					subtype	type
vnum is via number
default_dir is default dir when computing dir not deternmined
check_len is used for computing dir range
cr_mask is clear via mask
method_opt
0: for via info input
1: for mask inout
*/
void remove_via(PipeData & d, ProcessParameter & cpara)
{
	int idx = cpara.method_opt & 0xf;
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);
	if (d.l[layer].v[idx].type != TYPE_VIA_LOCATION) {
		qCritical("remove_via via info error, idx[%d]=%d", idx, d.l[layer].v[idx].type);
		return;
	}
	int vnum = cpara.opt0 & 0xff;
	int check_len = cpara.opt0 >> 8 & 0xff;
	int default_dir = cpara.opt0 >> 16 & 0xff;
	int cr_mask = cpara.opt0 >> 24 & 0xff;
	qInfo("remove_via, l=%d check_len=%d, vnum=%d, default_dir=%d, cr_mask=%d", 
		layer, check_len, vnum, default_dir, cr_mask);

	Mat mask;
	if (cr_mask) {
		int idx1 = cpara.method_opt >> 4 & 0xf;
		mask = d.l[layer].v[idx1].d;
		if (d.l[layer].v[idx1].type != TYPE_FINE_WIRE_MASK) {
			qCritical("remove_via mask type %d error", d.l[layer].v[idx1].type);
			return;
		}
	}
	
#define MAX_VIA_NUM 3	
	if (vnum > MAX_VIA_NUM || vnum == 0) {
		qCritical("remove_via wrong vnum");
		return;
	}
	int v_type[MAX_VIA_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff };
	int v_subtype[MAX_VIA_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt3 >> 8 & 0xff };
	int v_rr[MAX_VIA_NUM];
	QScopedPointer<ViaRemove> vr[MAX_VIA_NUM];

	for (int i = 0; i < vnum; i++) {
		ViaParameter via_para;
		via_para = d.l[layer].vw.get_vw_para(BRICK_VIA, v_type[i], v_subtype[i])->v;
		v_rr[i] = via_para.remove_rd;
		vr[i].reset(ViaRemove::create_via_remove(via_para, d));
	}

	Mat & m = d.l[layer].v[idx].d;
	for (int i = 0; i < m.rows; i++) {
		int type = m.at<int>(i, 0);
		int subtype = m.at<int>(i, 1);
		int x0 = m.at<int>(i, 2);
		int y0 = m.at<int>(i, 3);
		int sel = -1;
		for (int j = 0; j < vnum; j++)
			if (v_type[j] == type && v_subtype[j] == subtype  && !vr[j].isNull()) {
				sel = j;
				break;
			}
		if (sel < 0)
			continue;
		//Following compute dir
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
		int dir = (i0 > i90) ? 0 : ((i90 == 0) ? default_dir : 1);
		vr[sel]->remove(img, x0, y0, dir);
		if (cr_mask)
			vr[sel]->remove_mask(mask, x0, y0);
	}
#undef MAX_VIA_NUM
	if (cpara.method & OPT_DEBUG_EN) {
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_delvia.jpg", img);
		if (cr_mask) {
			Mat debug_draw = img.clone();
			for (int y = 0; y < mask.rows; y++) {
				int * p_mask = mask.ptr<int>(y);
				unsigned char * p_img = debug_draw.ptr<unsigned char>(y);
				for (int x = 0; x < mask.cols; x++)
					if (p_mask[x])
						p_img[x] = 0xff;
			}
			imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_delvia_mask.jpg", debug_draw);
		}
	}
	d.l[layer].ig_valid = false;
}


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
For BRICK_I_0, cwide is small (1 or 2), clong need to be fit for coarse_line_search
method_opt
0: for search mask output
*/
void hotpoint2fine_search_stmask(PipeData & d, ProcessParameter & cpara)
{
	int idx = cpara.method_opt & 0xf;
	int wnum = cpara.opt0 & 0xff;
	int layer = cpara.layer;
	Mat & mask = d.l[layer].v[idx].d;
	Mat & img = d.l[layer].img;
	d.l[layer].v[idx].type = TYPE_FINE_WIRE_MASK;
	int clear_mask = cpara.opt0 >> 8 & 0xff;
	
	if (clear_mask) {
		mask.create(img.rows, img.cols, CV_32S);
		mask = Scalar::all(0);
	}
	if (mask.rows != img.rows || mask.cols != img.cols) {
		qCritical("hotpoint2fine_search_mask, mask.size(%d,%d)!=img.size(%d,%d)", mask.rows, mask.cols, img.rows, img.cols);
		return;
	}
	qInfo("hotpoint2fine_search_mask, l=%d, wnum=%d, clear_mask=%d", layer, wnum, clear_mask);
	if (wnum > 3) {
		qCritical("hotpoint2fine_search_mask wrong wnum");
		return;
	}
	struct WireMaskInfo {
		int type_shape;			
		int clong;
		int cwide;
		int subtype;
		int extend;
		int x1, y1, x2, y2, cx, cy;
		int mask;
	} wpara[] = {
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
			wpara[i].mask = 1 << wpara[i].subtype;
			w[0].push_back(wpara[i]);
			break;
		case BRICK_I_90:
			wpara[i].cx = wpara[i].clong;
			wpara[i].cy = wpara[i].cwide;
			wpara[i].x1 = 1;
			wpara[i].x2 = (wpara[i].clong - 1) / d.l[layer].gs + 1;
			wpara[i].y2 = (wpara[i].cwide - 1) / d.l[layer].gs + 1;
			wpara[i].y1 = -wpara[i].y2;
			wpara[i].mask = 1 << wpara[i].subtype;
			w[1].push_back(wpara[i]);
			break;
		default:
			qCritical("Error wrong shape");
			break;
		}		
	}
	
	Mat mark(d.l[layer].prob.rows, d.l[layer].prob.cols, CV_8UC1);
	mark = Scalar::all(0);
	for (int dir = 0; dir < 2; dir++)
	if (dir==0) {
		if (w[dir].empty())
			continue;
		for (int y = 0; y < d.l[layer].prob.rows; y++) {
			unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y);
			for (int x = 0; x < d.l[layer].prob.cols; x++) {
				int i;
				for (i = 0; i < w[dir].size(); i++)
					if (PROB_TYPESHAPE(p_prob[2 * x]) == w[dir][i].type_shape && (mark.at<unsigned char>(y,x) & 1) == 0)
						break;
				if (i == w[dir].size())
					continue;
				int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]);
				mark.at<unsigned char>(y, x) |= 1;
				mark_line(mask, Point(x0, y0), Point(x0, max(0, y0 - w[dir][i].extend)), w[dir][i].mask);
				bool check;
				do {
					int x1 = max(x0 / d.l[layer].gs + w[dir][i].x1, 0), x2 = min(x0 / d.l[layer].gs + w[dir][i].x2, d.l[layer].prob.cols - 1);
					int y1 = max(y0 / d.l[layer].gs + w[dir][i].y1, 0), y2 = min(y0 / d.l[layer].gs + w[dir][i].y2, d.l[layer].prob.rows - 1);
					check = false;
					for (int yy = y1; yy <= y2; yy++) {
						unsigned long long * p_prob2 = d.l[layer].prob.ptr<unsigned long long>(yy);
						for (int xx = x1; xx <= x2; xx++)
							if (PROB_TYPESHAPE(p_prob2[2 * xx]) == w[dir][i].type_shape) {
								if (abs(PROB_X(p_prob2[2 * xx]) - x0) <= w[dir][i].cx &&
									abs(PROB_Y(p_prob2[2 * xx]) - y0) <= w[dir][i].cy) {
									if (mark.at<unsigned char>(yy, xx) & 1) {
										qCritical("hotpoint2fine_search_mask mark line at intersect (%d,%d), maybe cwide=%d too big",
											x0, y0, w[dir][i].cx);
										return;
									}
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
		for (int x = 0; x < d.l[layer].prob.cols; x++) {
			for (int y = 0; y < d.l[layer].prob.rows; y++) {
				unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y);
				int i;
				for (i = 0; i < w[dir].size(); i++)
					if (PROB_TYPESHAPE(p_prob[2 * x]) == w[dir][i].type_shape && (mark.at<unsigned char>(y, x) & 2) == 0)
						break;
				if (i == w[dir].size())
					continue;
				int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]);
				mark.at<unsigned char>(y, x) |= 2;
				mark_line(mask, Point(x0, y0), Point(max(0, x0 - w[dir][i].extend), y0), w[dir][i].mask);
				bool check;
				do {
					int x1 = max(x0 / d.l[layer].gs + w[dir][i].x1, 0), x2 = min(x0 / d.l[layer].gs + w[dir][i].x2, d.l[layer].prob.cols - 1);
					int y1 = max(y0 / d.l[layer].gs + w[dir][i].y1, 0), y2 = min(y0 / d.l[layer].gs + w[dir][i].y2, d.l[layer].prob.rows - 1);
					check = false;
					for (int xx = x1; xx <= x2; xx++)
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = d.l[layer].prob.ptr<unsigned long long>(yy);
							if (PROB_TYPESHAPE(p_prob2[2 * xx]) == w[dir][i].type_shape) {
								if (abs(PROB_X(p_prob2[2 * xx]) - x0) <= w[dir][i].cx &&
									abs(PROB_Y(p_prob2[2 * xx]) - y0) <= w[dir][i].cy) {
									if (mark.at<unsigned char>(yy, xx) & 2) {
										qCritical("hotpoint2fine_search_mask mark line at intersect (%d,%d), maybe clong=%d too big",
											x0, y0, w[dir][i].cy);
										return;
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
	}
}

/*
		31..24  23..16   15..8  7..0
opt0:					 extend	wnum
opt1:			subtype  type  shape	
opt2:			subtype  type  shape
opt3:			subtype  type  shape
opt4:			subtype  type  shape
wnum is wire type num
extend is if it needs to search extend
method_opt
0: for gray level turn_points  input
1: for search mask input 
*/
void fine_line_search(PipeData & d, ProcessParameter & cpara)
{
	int idx = cpara.method_opt & 0xf;
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);
	if (d.l[layer].v[idx].type != TYPE_GRAY_LEVEL) {
		qCritical("fine_line_search gray level idx[%d]=%d, error", idx, d.l[layer].v[idx].type);
		return;
	}
	int idx1 = cpara.method_opt >> 4 & 0xf;
	Mat mask = d.l[layer].v[idx1].d.clone();
	if (d.l[layer].v[idx1].type != TYPE_FINE_WIRE_MASK) {
		qCritical("fine_line_search mask idx[%d]=%d, error", idx1, d.l[layer].v[idx1].type);
		return;
	}
	if (mask.rows != img.rows || mask.cols != img.cols) {
		qCritical("fine_line_search, mask.size(%d,%d)!=img.size(%d,%d)", mask.rows, mask.cols, img.rows, img.cols);
		return;
	}
	int gi, gm;
	gi = find_index(d.l[layer].v[idx].d, (int) GRAY_L0);
	gi = d.l[layer].v[idx].d.at<int>(gi, 2);
	gm = find_index(d.l[layer].v[idx].d, (int) GRAY_M0);
	gm = d.l[layer].v[idx].d.at<int>(gm, 2);
	int wnum = cpara.opt0 & 0xff;
	int extend = cpara.opt0 >> 8 & 0xff;
	qInfo("fine_line_search, l=%d, wnum=%d, gi=%d, gm=%d, extend=%d", layer, wnum, gi, gm, extend);

#define MAX_WIRE_NUM 4
	if (wnum > MAX_WIRE_NUM) {
		qCritical("fine_line_search, wnum too big");
		return;
	}
	int w_type[MAX_WIRE_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt3 >> 8 & 0xff, cpara.opt4 >> 8 & 0xff };
	int w_subtype[MAX_WIRE_NUM] = { cpara.opt1 >> 16 & 0xff, cpara.opt2 >> 16 & 0xff, cpara.opt3 >> 16 & 0xff, cpara.opt4 >> 16 & 0xff };
	int w_shape[MAX_WIRE_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff, cpara.opt4 & 0xff };
	int w_guard[MAX_WIRE_NUM];
	QScopedPointer<WireComputeScore> wcs[MAX_WIRE_NUM];

	for (int i = 0; i < wnum; i++) {
		WireParameter wire_para;
		wire_para = d.l[layer].vw.get_vw_para(w_shape[i], w_type[i], w_subtype[i])->w;
		wire_para.gray_i = gi;
		wire_para.gray_w = gm;
		wcs[i].reset(WireComputeScore::create_wire_compute_score(wire_para, d, layer));
		w_guard[i] = wire_para.guard;
	}
	d.l[layer].validate_ig();
	Mat new_shape(d.l[layer].prob.rows, d.l[layer].prob.cols, CV_8UC1);
	Mat new_prob(d.l[layer].prob.rows, d.l[layer].prob.cols, CV_8UC1);
	int mark_num = 100;

	while (mark_num != 0) {
		new_shape = Scalar::all(0);
		new_prob = Scalar::all(0);
		Mat prob = d.l[layer].prob.clone();
		//following compute new prob, new_prob means prob is updated with new value
		for (int y = d.l[layer].compute_border; y < img.rows - d.l[layer].compute_border; y++) {
			unsigned * p_mask = mask.ptr<unsigned>(y);
			for (int x = d.l[layer].compute_border; x < img.cols - d.l[layer].compute_border; x++)
				if (p_mask[x]) {
				for (int i = 0; i < wnum; i++)
					if (!wcs[i].isNull() && (p_mask[x] >> w_subtype[i] & 1)) {
					PAIR_ULL score = wcs[i]->compute(x, y, img, d.l[layer].ig, d.l[layer].iig);
					SET_PROB_TYPE(score.first, i);
					SET_PROB_TYPE(score.second, i);
					bool changed = false;
					if (PROB_SHAPE(score.first) != BRICK_HOLLOW)
						changed |= push_new_prob(prob, score.first, d.l[layer].gs);
					if (PROB_SHAPE(score.second) != BRICK_HOLLOW)
						changed |= push_new_prob(prob, score.second, d.l[layer].gs);
					new_prob.at<unsigned char>(y / d.l[layer].gs, x / d.l[layer].gs) = changed ?
						1 : new_prob.at<unsigned char>(y / d.l[layer].gs, x / d.l[layer].gs);
					}
				}
		}

		//write back to d.prob, new_shape means prob is updated with new prob
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			unsigned long long * p_org_prob = d.l[layer].prob.ptr<unsigned long long>(y);
			unsigned char * p_new_prob = new_prob.ptr<unsigned char>(y);
			for (int x = 0; x < prob.cols * 2; x++) {
				if (!p_new_prob[x / 2])
					continue;
				if (PROB_SHAPE(p_prob[x]) != BRICK_I_0 && PROB_SHAPE(p_prob[x]) != BRICK_I_90) { //following filter unique
					unsigned long long prob0 = p_prob[x];
					bool pass2 = true;
					//check unique
					int cr = w_guard[PROB_TYPE(prob0)];
					int guard = (cr - 1) / d.l[layer].gs + 1;
					int y1 = max(y - guard, 0), y2 = min(y + guard, prob.rows - 1);
					int x1 = max(x / 2 - guard, 0), x2 = min(x / 2 + guard, prob.cols - 1);
					for (int yy = y1; yy <= y2; yy++) {
						unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
						for (int xx = 2 * x1; xx < 2 * x2 + 2; xx++)
							if (p_prob2[xx] < prob0 && PROB_SHAPE(p_prob2[xx]) == PROB_SHAPE(prob0)) {
							if (abs(PROB_X(p_prob2[xx]) - PROB_X(prob0)) <= cr &&
								abs(PROB_Y(p_prob2[xx]) - PROB_Y(prob0)) <= cr) {
								pass2 = false;
								yy = y2;
								break;
							}
							}
					}
					if (pass2) { //if unique, push to d.prob
						unsigned long long score = prob0;
						SET_PROB_TYPE(score, w_type[PROB_TYPE(score)]);
						new_shape.at<unsigned char>(y, x / 2) = (PROB_SHAPE(score) != PROB_SHAPE(p_org_prob[x]) && x % 2 == 0);
						push_new_prob(d.l[layer].prob, score, d.l[layer].gs);
					}
				}
				else {
					unsigned long long score = p_prob[x];
					SET_PROB_TYPE(score, w_type[PROB_TYPE(score)]);
					new_shape.at<unsigned char>(y, x / 2) = (PROB_SHAPE(score) != PROB_SHAPE(p_org_prob[x]) && x % 2 == 0);
					push_new_prob(d.l[layer].prob, score, d.l[layer].gs);
				}
			}
		}

		mask = Scalar::all(0);
		mark_num = 0;
		for (int y = 0; y < prob.rows; y++) {
			unsigned char * p_new_shape = new_shape.ptr<unsigned char>(y);
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols; x++)
				if (p_new_shape[x]) {
				unsigned long long prob0 = p_prob[2 * x];
				mark_num += wcs[PROB_TYPE(prob0)]->check_mark(x, y, d.l[layer].prob, mask, w_subtype[PROB_TYPE(prob0)]);
				}
		}
		qDebug("new mark_num=%d", mark_num);
	}
#undef MAX_WIRE_NUM
	
	if (cpara.method & OPT_DEBUG_EN) {
		d.l[layer].check_prob();
		Mat debug_draw;
		debug_draw = img.clone();
		for (int y = 0; y < d.l[layer].prob.rows; y++) {
			unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y);
			for (int x = 0; x < d.l[layer].prob.cols; x++)
				if (PROB_SHAPE(p_prob[2 * x]) < sizeof(bricks)/sizeof(bricks[0])) {
					int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]);
					int shape = PROB_SHAPE(p_prob[2 * x]);
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							if (bricks[shape].a[i][j])
								debug_draw.at<unsigned char>(y0 + i, x0 + j) = 255;
				}
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_fine_line_search.jpg", debug_draw);
	}
}

/*
		31..24 23..16   15..8   7..0
opt0:							wnum
opt1:	clong_heng clong_shu cwide	type
opt2:	clong_heng clong_shu cwide	type		
opt3:	clong_heng clong_shu cwide	type		
opt4:			
opt5:			
opt6:			

method_opt
*/
void assemble_line(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & prob = d.l[layer].prob;	
	
	int wnum = cpara.opt0 & 0xff;
	qInfo("assemble_line l=%d, wnum=%d", layer, wnum);
	Mat mark(prob.rows, prob.cols, CV_8UC1);	

#define MAX_WIRE_NUM 6
	int wtype[MAX_WIRE_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff, cpara.opt4 & 0xff, cpara.opt5 & 0xff };
	int cwide[MAX_WIRE_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, 
		cpara.opt3 >> 8 & 0xff, cpara.opt4 >> 8 & 0xff, cpara.opt5 >> 8 & 0xff };
	int clong0[MAX_WIRE_NUM] = { cpara.opt1 >> 16 & 0xff, cpara.opt2 >> 16 & 0xff,
		cpara.opt3 >> 16 & 0xff, cpara.opt4 >> 16 & 0xff, cpara.opt5 >> 16 & 0xff };
	int clong1[MAX_WIRE_NUM] = { cpara.opt1 >> 24 & 0xff, cpara.opt2 >> 24 & 0xff,
		cpara.opt3 >> 24 & 0xff, cpara.opt4 >> 24 & 0xff, cpara.opt5 >> 24 & 0xff };

	struct WireLine {
		vector<unsigned long long> corner;
		vector<int> state;
		void push_brick(unsigned long long brick, int _state) {
			if (!state.empty()) {
				if (state.back() & START_BRICK)
					CV_Assert(_state & END_BRICK);
				if (state.back() & END_BRICK && !(state.back() & START_BRICK))
					CV_Assert(_state & START_BRICK);
			}
			corner.push_back(brick);
			state.push_back(_state);
		}
		void clear() {
			corner.clear();
			state.clear();
		}
	} cur_line;

	struct TraceState {
		unsigned long long prev_brick;
		int x, y;
		vector<unsigned long long> brick_order;
	} trace;
	vector<WireLine> lineset[2];

	for (int i = 0; i < wnum; i++) {
		qInfo("assemble_line, type=%d, cwide=%d, clong0=%d, clong1=%d", wtype[i], cwide[i], clong0[i], clong1[i]);
		if (cwide[i] > d.l[layer].gs) {
			qCritical("invalid cwide");
			return;
		}
	}

	int fix_count = 0, noend_count = 0;
	int x1 = BORDER_SIZE / d.l[layer].gs - 1;
	int x2 = prob.cols - BORDER_SIZE / d.l[layer].gs + 1;
	int y1 = BORDER_SIZE / d.l[layer].gs - 1;
	int y2 = prob.rows - BORDER_SIZE / d.l[layer].gs + 1;
	
	mark = Scalar::all(0);
	for (int y = y1; y < y2; y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
		for (int x = x1; x < x2; x++) 
			if (p_mark[x]==0 && p_prob[2*x] != 0xffffffffffffffffULL) {
				unsigned long long prob0 = p_prob[2 * x];
				int brick_fix;
				if (brick_conn.quick_fix(0, BRICK_NO_WIRE, PROB_SHAPE(prob0), brick_fix) != brick_conn.NO_NEED_TRACE) {
					trace.prev_brick = 0;
					SET_PROB_SHAPE(trace.prev_brick, BRICK_NO_WIRE);
					SET_PROB_TYPE(trace.prev_brick, PROB_TYPE(prob0));
					SET_PROB_X(trace.prev_brick, PROB_X(prob0));
					SET_PROB_Y(trace.prev_brick, PROB_Y(prob0) - d.l[layer].gs);
					
					cur_line.clear();

					int cw, cl;
					for (int i = 0; i < wnum; i++)
						if (wtype[i] == PROB_TYPE(prob0)) {
							cw = cwide[i];
							cl = clong0[i];
							break;
						}
					int cl1 = (cl - 1) / d.l[layer].gs + 1;
					bool in_trace = true;

					while (in_trace) {
						trace.brick_order.clear();
						trace.x = PROB_X(trace.prev_brick) / d.l[layer].gs;
						trace.y = PROB_Y(trace.prev_brick) / d.l[layer].gs;	
						if (trace.y >= y2) //TODO: need to do something?
							break;
						
						for (int yy = trace.y + 1; yy <= min(trace.y + cl1, mark.rows - 1); yy++)
							for (int xx = trace.x - 1; xx <= trace.x + 1; xx++) {
								mark.at<unsigned char>(yy, xx) = 1;
								prob0 = prob.at<unsigned long long>(yy, xx * 2);
								if (abs(PROB_X(trace.prev_brick) - PROB_X(prob0)) <= cw) {
									int i = (int)trace.brick_order.size() - 1;
									while (i >= 0 && trace.brick_order[i] > prob0) i--;
									trace.brick_order.insert(trace.brick_order.begin() + (i + 1), prob0);
								}
							}
						if (trace.brick_order.empty()) {
							noend_count++;
							SET_PROB_X(prob0, PROB_X(trace.prev_brick));
							SET_PROB_Y(prob0, PROB_Y(trace.prev_brick) + d.l[layer].gs);
							SET_PROB_TYPE(prob0, PROB_TYPE(trace.prev_brick));
							SET_PROB_SHAPE(prob0, BRICK_NO_WIRE);
						}
						else
							prob0 = trace.brick_order[0];
						if (PROB_TYPE(prob0) != PROB_TYPE(trace.prev_brick))
							qCritical("Fix me tracking line 0, x=%d, y=%d, prev_brick=%llx, prob0=%llx", 
							PROB_X(trace.prev_brick), PROB_Y(trace.prev_brick), trace.prev_brick, prob0);
						int fix_result = brick_conn.quick_fix(0, PROB_SHAPE(trace.prev_brick), PROB_SHAPE(prob0), brick_fix);
						int need_fix = 0;
						unsigned long long next_prob0;
						int suspect_brick = 0;
						int next_fix_result;
						if (fix_result == BrickConnect::FIX_TRACE) {
							int nouse_brick_fix;
							need_fix = 1;
							next_prob0 = prob0;
							fix_result = brick_conn.quick_fix(0, PROB_SHAPE(trace.prev_brick), brick_fix, nouse_brick_fix);
							next_fix_result = brick_conn.quick_fix(0, brick_fix, PROB_SHAPE(prob0), nouse_brick_fix);
							CV_Assert(fix_result != BrickConnect::FIX_TRACE && next_fix_result != BrickConnect::FIX_TRACE);
							
							if (trace.y > y1 && trace.brick_order.size() >= 2 && PROB_SHAPE(trace.brick_order[1]) != brick_fix) {
								fix_count++;
								suspect_brick = SUSPECT_BRICK;
							}
							if (trace.brick_order.size() < 2 || PROB_SHAPE(trace.brick_order[1]) != brick_fix) {
								SET_PROB_SHAPE(prob0, brick_fix);
								SET_PROB_X(prob0, (PROB_X(prob0) + PROB_X(trace.prev_brick)) / 2);
								SET_PROB_Y(prob0, (PROB_Y(prob0) + PROB_Y(trace.prev_brick)) / 2);
							}
							else
								prob0 = trace.brick_order[1];
						}
						
						for (int i = 0; i <= need_fix; i++) {
							CV_Assert(PROB_X(prob0) < d.l[layer].img.cols && PROB_Y(prob0) < d.l[layer].img.rows);
							switch (fix_result) {
							case BrickConnect::START_TRACE0:
								cur_line.push_brick(trace.prev_brick, START_BRICK | suspect_brick);
								break;

							case BrickConnect::START_TRACE1:
								cur_line.push_brick(prob0, START_BRICK | suspect_brick);
								break;

							case BrickConnect::CONTINUE_TRACE1:
								break;

							case BrickConnect::END_TRACE1:
								cur_line.push_brick(prob0, END_BRICK | suspect_brick);
								break;

							case BrickConnect::END_START_TRACE1:
								cur_line.push_brick(prob0, START_BRICK | END_BRICK | suspect_brick);
								break;

							case BrickConnect::START_TRACE0_END_TRACE1:
								cur_line.push_brick(trace.prev_brick, START_BRICK | suspect_brick);
								cur_line.push_brick(prob0, END_BRICK | suspect_brick);
								break;

							case BrickConnect::START_TRACE0_END_START_TRACE1:
								cur_line.push_brick(trace.prev_brick, START_BRICK | suspect_brick);
								cur_line.push_brick(prob0, START_BRICK | END_BRICK | suspect_brick);
								break;

							case BrickConnect::NO_NEED_TRACE:
								in_trace = false;
								break;

							default:
								qFatal("brick_conn fix have internal error");
							}

							trace.prev_brick = prob0;
							prob0 = next_prob0;
							fix_result = next_fix_result;
						}
					}
					lineset[0].push_back(cur_line);
				}
			}
	}

	mark = Scalar::all(0);
	for (int x = x1; x < x2; x++) {
		for (int y = y1; y < y2; y++)
			if (mark.at<unsigned char>(y, x) == 0 && prob.at<unsigned long long>(y, x * 2) != 0xffffffffffffffffULL) {
				unsigned long long prob0 = prob.at<unsigned long long>(y, x * 2);
				int brick_fix;
				if (brick_conn.quick_fix(1, BRICK_NO_WIRE, PROB_SHAPE(prob0), brick_fix) != brick_conn.NO_NEED_TRACE) {
					trace.prev_brick = 0;
					SET_PROB_SHAPE(trace.prev_brick, BRICK_NO_WIRE);
					SET_PROB_TYPE(trace.prev_brick, PROB_TYPE(prob0));
					SET_PROB_X(trace.prev_brick, PROB_X(prob0) - d.l[layer].gs);
					SET_PROB_Y(trace.prev_brick, PROB_Y(prob0));

					cur_line.clear();

					int cw, cl;
					for (int i = 0; i < wnum; i++)
						if (wtype[i] == PROB_TYPE(prob0)) {
							cw = cwide[i];
							cl = clong1[i];
							break;
						}
					int cl1 = (cl - 1) / d.l[layer].gs + 1;
					bool in_trace = true;

					while (in_trace) {
						trace.brick_order.clear();
						trace.x = PROB_X(trace.prev_brick) / d.l[layer].gs;
						trace.y = PROB_Y(trace.prev_brick) / d.l[layer].gs;
						if (trace.x >= x2) //TODO: need to do something?
							break;

						for (int yy = trace.y - 1; yy <= trace.y + 1; yy++)
							for (int xx = trace.x + 1; xx <= min(trace.x + cl1, mark.cols - 1); xx++) {
								mark.at<unsigned char>(yy, xx) = 1;
								prob0 = prob.at<unsigned long long>(yy, xx * 2);
								if (abs(PROB_Y(trace.prev_brick) - PROB_Y(prob0)) <= cw) {
									int i = (int)trace.brick_order.size() - 1;
									while (i >= 0 && trace.brick_order[i] > prob0) i--;
									trace.brick_order.insert(trace.brick_order.begin() + (i + 1), prob0);
								}
							}
						if (trace.brick_order.empty()) {
							noend_count++;
							SET_PROB_X(prob0, PROB_X(trace.prev_brick) + d.l[layer].gs);
							SET_PROB_Y(prob0, PROB_Y(trace.prev_brick));
							SET_PROB_TYPE(prob0, PROB_TYPE(trace.prev_brick));
							SET_PROB_SHAPE(prob0, BRICK_NO_WIRE);
						}
						else
							prob0 = trace.brick_order[0];
						if (PROB_TYPE(prob0) != PROB_TYPE(trace.prev_brick))
							qCritical("Fix me tracking line 1, x=%d, y=%d, prev_brick=%llx, prob0=%llx",
							PROB_X(trace.prev_brick), PROB_Y(trace.prev_brick), trace.prev_brick, prob0);
						int fix_result = brick_conn.quick_fix(1, PROB_SHAPE(trace.prev_brick), PROB_SHAPE(prob0), brick_fix);
						int need_fix = 0;
						unsigned long long next_prob0;
						int suspect_brick = 0;
						int next_fix_result;
						if (fix_result == BrickConnect::FIX_TRACE) {
							int nouse_brick_fix;
							need_fix = 1;
							next_prob0 = prob0;
							fix_result = brick_conn.quick_fix(1, PROB_SHAPE(trace.prev_brick), brick_fix, nouse_brick_fix);
							next_fix_result = brick_conn.quick_fix(1, brick_fix, PROB_SHAPE(prob0), nouse_brick_fix);
							CV_Assert(fix_result != BrickConnect::FIX_TRACE && next_fix_result != BrickConnect::FIX_TRACE);

							if (trace.y > y1 && trace.brick_order.size() >= 2 && PROB_SHAPE(trace.brick_order[1]) != brick_fix) {
								fix_count++;
								suspect_brick = SUSPECT_BRICK;
							}
							if (trace.brick_order.size() < 2 || PROB_SHAPE(trace.brick_order[1]) != brick_fix) {
								SET_PROB_SHAPE(prob0, brick_fix);
								SET_PROB_X(prob0, (PROB_X(prob0) + PROB_X(trace.prev_brick)) / 2);
								SET_PROB_Y(prob0, (PROB_Y(prob0) + PROB_Y(trace.prev_brick)) / 2);
							}
							else
								prob0 = trace.brick_order[1];
						}
						
						for (int i = 0; i <= need_fix; i++) {
							CV_Assert(PROB_X(prob0) < d.l[layer].img.cols && PROB_Y(prob0) < d.l[layer].img.rows);
							switch (fix_result) {
							case BrickConnect::START_TRACE0:
								cur_line.push_brick(trace.prev_brick, START_BRICK | suspect_brick);
								break;

							case BrickConnect::START_TRACE1:
								cur_line.push_brick(prob0, START_BRICK | suspect_brick);
								break;

							case BrickConnect::CONTINUE_TRACE1:
								break;

							case BrickConnect::END_TRACE1:
								cur_line.push_brick(prob0, END_BRICK | suspect_brick);
								break;

							case BrickConnect::END_START_TRACE1:
								cur_line.push_brick(prob0, START_BRICK | END_BRICK | suspect_brick);
								break;

							case BrickConnect::START_TRACE0_END_TRACE1:
								cur_line.push_brick(trace.prev_brick, START_BRICK | suspect_brick);
								cur_line.push_brick(prob0, END_BRICK | suspect_brick);
								break;

							case BrickConnect::START_TRACE0_END_START_TRACE1:
								cur_line.push_brick(trace.prev_brick, START_BRICK | suspect_brick);
								cur_line.push_brick(prob0, START_BRICK | END_BRICK | suspect_brick);
								break;

							case BrickConnect::NO_NEED_TRACE:
								in_trace = false;
								break;

							default:
								qFatal("brick_conn fix have internal error");
							}

							trace.prev_brick = prob0;
							prob0 = next_prob0;
							fix_result = next_fix_result;
						}
					}
					lineset[1].push_back(cur_line);
				}
			}
	}
#undef MAX_WIRE_NUM

	if (cpara.method & OPT_DEBUG_EN) {
		Mat debug_draw;		
		for (int dir = 0; dir < 2; dir++) {
			debug_draw = d.l[layer].img.clone();
			for (int i = 0; i < lineset[dir].size(); i++)
				for (int j = 0; j < lineset[dir][i].corner.size(); j++) {
					unsigned long long prob0 = lineset[dir][i].corner[j];
					int x0 = PROB_X(prob0), y0 = PROB_Y(prob0);
					int shape = PROB_SHAPE(prob0);
					int color = 0;
					switch (lineset[dir][i].state[j] & 3) {
					case START_BRICK:
						color = 180;
						break;
					case END_BRICK:
						color = 220;
						break;
					case START_BRICK | END_BRICK:
						color = 255;
						break;						
					}
					for (int y = 0; y < 3; y++)
						for (int x = 0; x < 3; x++)
							if (bricks[shape].a[y][x])
								debug_draw.at<unsigned char>(y0 + y, x0 + x) = color;
				}
			imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_assemble_line.jpg", debug_draw);
		}
	}
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
		{ PP_COARSE_VIA_MASK, coarse_via_search_mask },
		{ PP_FINE_VIA_SEARCH, fine_via_search },
		{ PP_REMOVE_VIA, remove_via },
		{ PP_FINE_SEARCH_MASK, hotpoint2fine_search_stmask },
		{ PP_FINE_LINE_SEARCH, fine_line_search },		
		{ PP_ASSEMBLE, assemble_line}		
};

VWExtractPipe::VWExtractPipe()
{
	private_data = NULL;
}

VWExtractPipe::~VWExtractPipe()
{
	if (private_data) {
		PipeData * d = (PipeData *) private_data;
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
	cpara.layer = layer;
	vwp.push_back(cpara);
	return 0;
}

int VWExtractPipe::set_extract_param(int layer, int type, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, float pf1)
{
	return set_train_param(layer, type, pi1, pi2, pi3, pi4, pi5, pi6, pi7, pf1);
}


int VWExtractPipe::extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets)
{
	QDir *qdir = new QDir;
	deldir("./DImg");
	bool exist = qdir->exists("./DImg");
	if (!exist) {
		bool ok = qdir->mkdir("./DImg");
		if (!ok)
			qCritical("mkdir failed");
	}
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
	unsigned char ll = file_name[file_name.size() - 5];
	for (int i = ll - '0'; i < layer_num; i++) {
		file_name[file_name.size() - 5] = i + '0';
		Mat img = imread(file_name, 0);
		if (!img.empty())
			d->l[i].set_raw_img(img);
		else {
			qCritical("extract read layer %d, image file error", i);
			return -1;
		}
	}

	for (int i = 0; i < vwp.size(); i++) {
		int method = vwp[i].method & 0xff;
		if (process_func[method] == NULL) {
			qCritical("process func method %d invalid", method);
			return -2;
		}
		process_func[method](*d, vwp[i]);
	}
	return 0;
}


int VWExtractPipe::extract(vector<ICLayerWr *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets)
{
	return 0;
}

VWExtract * VWExtract::create_extract(int)
{
	return new VWExtractPipe;
}