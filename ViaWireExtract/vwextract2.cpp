#include "vwextract2.h"
#include <list>
#include <algorithm>
#include <QDateTime>
#include <QScopedPointer>
#include <QDir>
#include <QtConcurrent>

#ifndef QT_DEBUG
#undef CV_Assert
#define CV_Assert(x) do {if (!(x)) {qFatal("Wrong at %s, %d", __FILE__, __LINE__);}} while(0)
#endif

#define PARALLEL 0
#define SAVE_RST_TO_FILE	1
#define DEBUG_LEFT_TOP_IMG  0

#define OPT_DEBUG_EN		0x8000
#define OPT_DEBUG_OUT_EN	0x4000
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

#define MIN_SCORE			10
#define DIR_UP				0
#define DIR_RIGHT			1
#define DIR_DOWN			2
#define DIR_LEFT			3

#define DIR_UP1_MASK		(1 << (DIR_UP * 2))
#define DIR_RIGHT1_MASK		(1 << (DIR_RIGHT * 2))
#define DIR_DOWN1_MASK		(1 << (DIR_DOWN * 2))
#define DIR_LEFT1_MASK		(1 << (DIR_LEFT * 2))

#define CLEAR_REMOVE_VIA_MASK					2
#define REMOVE_VIA_MASK							1

#define COARSE_LINE_CLEAR_PROB					2
#define COARSE_LINE_UPDATE_PROB					1
#define COARSE_LINE_UPDATE_WITH_MASK			4

#define HOTLINE_CLEAR_MASK						1

#define FINE_LINE_SEARCH_CLEAR_PROB				1
#define FINE_LINE_SEARCH_CLEAR_COLOR			2
#define FINE_LINE_SEARCH_NO_VIA					4
#define FILE_LINE_SEARCH_CHECK_VIA_WIRE_CONNECT	8

#define ASSEMBLE_LINE_NO_VIA					1

#define CHECK_VW_HOTLINE_CLEAR_MASK				1
#define CHECK_VW_SEARCH_CLEAR_COLOR				2
typedef pair<unsigned long long, unsigned long long> PAIR_ULL;

enum {
	TYPE_GRAY_LEVEL,
	TYPE_COARSE_WIRE,
	TYPE_VIA_MASK,
	TYPE_FINE_WIRE_MASK,
	TYPE_REMOVE_VIA_MASK,
	TYPE_SHADOW_PROB
};

enum {
	VIA_SUBTYPE_2CIRCLE = 0,
	VIA_SUBTYPE_3CIRCLE,
	VIA_SUBTYPE_4CIRCLE,
	VIA_SUBTYPE_2CIRCLEX,
	VIA_SUBTYPE_3CIRCLEX,
};

enum {
	WIRE_SUBTYPE_13RECT = 0
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
#define UP_BORDER		0x10
#define RIGHT_BORDER	0x20
#define DOWN_BORDER		0x40
#define LEFT_BORDER		0x80

static class BrickConnect {
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

protected:
	unsigned long long bfm[2][64];
	int action[2][BRICK_IN_USE + 1][BRICK_IN_USE + 1];
	int bf[2][BRICK_IN_USE + 1][BRICK_IN_USE + 1];
	int sa[BRICK_IN_USE + 1][BRICK_IN_USE + 1];

protected:
	/*
	Input, dir 0 is shu, 1 is heng
	Input, brick0, is previous brick
	Input, brick1, is current brick
	Output, brick_fix, if return is FIX_TRACE, brick_fix is suggested brick
	Return fix state
	*/
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
					return check_fit ? END_TRACE1 : CONTINUE_TRACE1;
				else
					return START_TRACE0_END_TRACE1;

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
					return check_fit ? END_TRACE1 : CONTINUE_TRACE1;
				else
					return START_TRACE0_END_TRACE1;

			default:
				qCritical("invalid brick %d", brick1);
				return NO_NEED_TRACE;
				break;
			}
		}
	}

	/*
	input: brick0, brick1
	output: brick0 + brick1
	*/
	int shape_add(int brick0, int brick1) {
		if (brick0 == BRICK_NO_WIRE || brick0 == BRICK_INVALID)
			return brick1;
		if (brick1 == BRICK_NO_WIRE || brick1 == BRICK_INVALID)
			return brick0;
		if (brick0 == BRICK_VIA || brick0 == BRICK_FAKE_VIA)
			return brick0;
		if (brick1 == BRICK_VIA || brick1 == BRICK_FAKE_VIA)
			return brick1;
		if (brick0 == BRICK_ONE_POINT && brick1 < BRICK_IN_USE)
			return brick1;
		if (brick1 == BRICK_ONE_POINT && brick0 < BRICK_IN_USE)
			return brick0;
		if (brick0 < BRICK_IN_USE && brick1 < BRICK_IN_USE) {
			int a[3][3];
			for (int y = 0; y < 3; y++)
			for (int x = 0; x < 3; x++)
				a[y][x] = bricks[brick0].a[y][x] | bricks[brick1].a[y][x];

			for (int i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++) {
				bool match = true;
				for (int y = 0; y < 3; y++)
				for (int x = 0; x < 3; x++)
					if (a[y][x] != bricks[i].a[y][x])
						match = false;
				if (match)
					return i;
			}
			CV_Assert(0);
		}
		return BRICK_FAKE_VIA;
	}
public:
	bool fit(int dir, int brick0, int brick1) {
		if (dir == DIR_DOWN || dir == DIR_LEFT) {
			swap(brick0, brick1);
			dir = dir - 2;
		}
		if (brick0 == BRICK_ONE_POINT)
			brick0 = BRICK_NO_WIRE;
		if (brick1 == BRICK_ONE_POINT)
			brick1 = BRICK_NO_WIRE;
		if (brick0 < sizeof(bricks) / sizeof(bricks[0]) && brick1 < sizeof(bricks) / sizeof(bricks[0]))
			return (bfm[dir][brick0] & 1ULL << brick1) ? true : false;
		else {
			if (brick0 == BRICK_FAKE_VIA || brick0 == BRICK_VIA || brick1 == BRICK_FAKE_VIA || brick1 == BRICK_VIA)
				return true;
			else
				return false;
		}			
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

		for (int i = 0; i <= BRICK_IN_USE; i++)
		for (int j = 0; j <= BRICK_IN_USE; j++)
			sa[i][j] = shape_add(i, j);
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

	int quick_shape_add(int brick0, int brick1) {
		if (brick0 <= BRICK_IN_USE && brick1 <= BRICK_IN_USE)
			return sa[brick0][brick1];
		else
			return shape_add(brick0, brick1);
	}
} brick_conn;

/*     31..24  23..16   15..8   7..0
opt0:		0  subtype   type  pattern
opt1:  pair_d  arfactor remove_rd guard
opt2:    rd3    rd2      rd1     rd0
opt3:   gray3  gray2    gray1   gray0
opt4:	connect_d cgray_d cgray_ratio connnect_rd
opt5:
opt6:
If use connect_rd, it should be bigger than guard,remove_rd,rd3,rd2,rd1,rd0
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
	int connect_d, cgray_d, cgray_ratio, connect_rd;
	int opt0, opt1;
	float optf;
};

/*     31..24  23..16   15..8   7..0
opt0:		0  subtype   type  pattern
opt1:				   arfactor guard
opt2:  w_wide  w_wide1  w_high  w_high1
opt3:	 dis_vw0  dis_vw1 i_wide  i_high   
opt4:           
opt5:          
opt6:
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
};

union VWParameter {
	struct ViaParameter v;
	struct WireParameter w;
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
		if (pattern < 192) {
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
			qInfo("set_wire_para, guard=%d, w_wide=%d, w_wide1=%d, w_high=%d, w_high1=%d, i_wide=%d, i_high=%d, dis_vw0=%d, dis_vw1=%d",
				vw->w.guard, vw->w.w_wide, vw->w.w_wide1, vw->w.w_high, vw->w.w_high1, 
				vw->w.i_wide, vw->w.i_high, vw->w.dis_vw0, vw->w.dis_vw1);
		} else {		
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
			vw->v.opt0 = cpara.opt5;
			vw->v.opt1 = cpara.opt6;
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
#define PROB_SCORE(p) ((int)((p) >> 48 & 0xffffffff))
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

class WireLine {
public:
	vector<unsigned long long> corner;
	vector<int> state;
	void push_brick(unsigned long long brick, int _state) {
		if (!state.empty()) {
			if (state.back() & START_BRICK)
				CV_Assert(_state & END_BRICK);
			if (state.back() & END_BRICK && !(state.back() & START_BRICK))
				CV_Assert(_state & START_BRICK);
		}
		if (!corner.empty() && corner.back() == brick)
			state[state.size() - 1] |= _state;
		else {
			corner.push_back(brick);
			state.push_back(_state);
		}
	}
	void clear() {
		corner.clear();
		state.clear();
	}
};

class CornerSets {
public:
	vector<QPoint> cs;
	int guard;

	CornerSets()
	{
		guard = 6;
	}

	void set_guard(int _g) {
		guard = _g;
	}

	bool merge(QPoint & co) {
		for (int i = 0; i < (int)cs.size(); i++) {
			if (abs(co.x() - cs[i].x()) < guard && abs(co.y() - cs[i].y()) < guard)
				return true;
		}
		cs.push_back(co);
		return false;
	}

	bool merge(CornerSets & fv) {
		bool ret = false;
		for (int i = 0; i < (int)fv.cs.size(); i++)
			ret |= merge(fv.cs[i]);
		return ret;
	}
	void clear() {
		cs.clear();
	}
};

bool comp_x(const QPoint &a, const QPoint &b)
{
	return a.x() < b.x();
}

bool comp_y(const QPoint &a, const QPoint &b)
{
	return a.y() < b.y();
}

class FinalWireLines {
public:
	vector<QRect> rects;
	vector<CornerSets> lines;
	int guard;

	FinalWireLines()
	{
		guard = 5;
	}
	/*
	inout: cl, it is input for new line and output merge line
	in: dir, 0 is for shu line, 1 is for heng line
	Return true if merge happen, else return false
	*/
	bool merge(CornerSets & cc, int dir, int depth = 0) {
		for (int i = 0; i < cc.cs.size(); i++)
		if (dir == 0)
			CV_Assert(cc.cs.back().y() >= cc.cs[i].y() && cc.cs[0].y() <= cc.cs[i].y());
		else
			CV_Assert(cc.cs.back().x() >= cc.cs[i].x() && cc.cs[0].x() <= cc.cs[i].x());

		bool ret = false;
		QRect cr(QPoint(min(cc.cs[0].x(), cc.cs.back().x()) - guard / 2, min(cc.cs[0].y(), cc.cs.back().y()) - guard / 2),
			QPoint(max(cc.cs[0].x(), cc.cs.back().x()) + guard / 2, max(cc.cs[0].y(), cc.cs.back().y()) + guard / 2));		

		for (int i = 0; i < (int) rects.size(); i++) {
			if (cr.left() > rects[i].right() || cr.right() < rects[i].left()) //quick check to save mips
				continue;
			if (cr.top() > rects[i].bottom() || cr.bottom() < rects[i].top()) //quick check to save mips
				continue;
			if (cr.intersects(rects[i])) {//if current line extend other line or overlap other line, merge other line to current line
				ret = true;
				cc.merge(lines[i]);
				if (dir == 0)
					sort(cc.cs.begin(), cc.cs.end(), comp_y);
				else
					sort(cc.cs.begin(), cc.cs.end(), comp_x);
				lines[i] = lines[lines.size() - 1];
				rects[i] = rects[rects.size() - 1];
				lines.pop_back();
				rects.pop_back();
				break;
			}
		}

		if (ret)
			merge(cc, dir, depth + 1);
		if (depth == 0) {			
			lines.push_back(cc);
			QRect cr(QPoint(min(cc.cs[0].x(), cc.cs.back().x()) - guard / 2, min(cc.cs[0].y(), cc.cs.back().y()) - guard / 2),
				QPoint(max(cc.cs[0].x(), cc.cs.back().x()) + guard / 2, max(cc.cs[0].y(), cc.cs.back().y()) + guard / 2));
			rects.push_back(cr);
		}
		CV_Assert(lines.size() == rects.size());
		return ret;
	}

	void recal_rect()
	{
		rects.clear();
		for (int i = 0; i < lines.size(); i++) {
			CornerSets & cc = lines[i];
			QRect cr(QPoint(min(cc.cs[0].x(), cc.cs.back().x()) - guard / 2, min(cc.cs[0].y(), cc.cs.back().y()) - guard / 2),
				QPoint(max(cc.cs[0].x(), cc.cs.back().x()) + guard / 2, max(cc.cs[0].y(), cc.cs.back().y()) + guard / 2));
			rects.push_back(cr);
		}
	}

	void set_guard(int _g) {
		guard = _g;
	}

	/*
	in: wl it is input line
	in: x0, y0 is image left-top zuobiao
	in: dir, 0 is for shu line, 1 is for heng line
	Return true if merge happen, else return false
	*/
	bool push(WireLine & wl, int x0, int y0, int dir) {
		bool ret = false;
		CV_Assert(wl.corner.size() == wl.state.size());
		CornerSets cc;
		for (int c = 0; c < wl.corner.size(); c++) {
			unsigned long long corner = wl.corner[c];			
			int state = wl.state[c];			
			cc.cs.push_back(QPoint(PROB_X(corner) + x0, PROB_Y(corner) + y0));
			
			if ((state & (START_BRICK | END_BRICK)) == END_BRICK) {
				ret |= merge(cc, dir);
				cc.clear();
			}
		}
		return ret;
	}
};

//obj_sets points unit is pixel
#define OBJ_WIRE			0
#define OBJ_VIA				1
typedef void(*ObjProcessFunc)(CornerSets & obj_sets, int layer, int obj_type, ProcessParameter & cpara);
struct ObjProcessHook {
	ObjProcessFunc func;
	ProcessParameter cpara;
};

struct ViaInfo {
	Point xy;
	int type, subtype;
	int via_adj_mask;
	int pair_d;
	int pattern;
	int v_pair[4];
	ViaInfo(Point _xy, int _type, int _subtype, int _pattern, int _pair_d) {
		xy = _xy;
		type = _type;
		subtype = _subtype;
		pattern = _pattern;
		pair_d = _pair_d;
		via_adj_mask = 0;
		for (int i = 0; i < 4; i++)
			v_pair[i] = -1;
	}
};

class PipeDataPerLayer {
public:
	Mat img, raw_img;
	Mat ig, iig, lg, llg;
	Mat prob;
	int gs;
	int border_size;
	int img_pixel_x0, img_pixel_y0; // it is load image pixel zuobiao
	struct {
		int type;
		Mat d;
	} v[16];
	VWSet vw;
	bool ig_valid;
	int compute_border;
	vector<WireLine> lineset[2]; //assemble_line output
	vector<QPoint> viaset; //fine_via_search output
	vector<ViaInfo> via_info;
	FinalWireLines fwl[2];
	CornerSets fv;
	//_gs * 2 <= _compute_border
	PipeDataPerLayer() {
		ig_valid = false;
		gs = 4;
		compute_border = 24;
		border_size = 48;
	}
	
	void reinit(int _gs, int _compute_border, int _border_size) {
		gs = _gs;
		compute_border = _compute_border;
		border_size = _border_size;

		prob.create((img.rows - 1) / gs + 1, (img.cols - 1) / gs + 1, CV_64FC2);
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols * 2; x++)
				p_prob[x] = 0xffffffffffffffffULL;
		}
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
					CV_Assert(shape <= BRICK_IN_USE || shape == BRICK_VIA  || shape == BRICK_FAKE_VIA || shape == BRICK_INVALID);
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
		viaset.clear();
		lineset[0].clear();
		lineset[1].clear();
		for (int i = 0; i < 16; i++)
			v[i].d.release();
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
		lg = & (d->l[layer].lg);
		llg = & (d->l[layer].llg);
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
		b = (float) n / n1;
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
		unsigned score =sqrt(((f[1] - f[0] * 2 * gray) * a + gray*gray) * b + ((f[3] - f[2] * 2 * gray1) * a1 + gray1*gray1) * b1);

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
		CV_Assert(f[1] + 1 >= f[0] * f[0] * a && f[3] + 1>= f[2] * f[2] * a1 && f[5] + 1 >= f[4] * f[4] * a2);
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
				p_img[x] = (p_img[x] < m0) ? m0 : ((p_img[x] < m1) ? m1 : ((p_img[x] > m2) ? m2 :p_img[x]));
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
		qInfo("ThreeCirclePrepare r0=%d,r1=%d,r2=%d,g0=%d,g1=%d,g2=%d, gamma=%f", tcc.r, tcc.r1, tcc.r2, vp.gray0, vp.gray1,vp.gray2, gamma);
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
		CV_Assert(f[1] + 1 >= f[0] * f[0] * a && f[3] + 1>= f[2] * f[2] * a1 && f[5] + 1 >= f[4] * f[4] * a2 && f[7] + 1>= f[6] * f[6] * a3);
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
	virtual void finish(Mat & img) = 0;
	virtual ~ViaRemove() {
		qDebug("ViaRemove freed");
	}	
};

class ViaCircleRemove : public ViaRemove {
protected:
	int r;
	int gd;
	int connect_rd;
	int w;
	vector<int> d;
	vector<int> d2;
	vector<int> d3;
	Mat img_buf;
public:
	void prepare(ViaParameter &vp, PipeData &) {
		r = vp.remove_rd;
		gd = vp.guard;
		connect_rd = vp.connect_rd;
		w = 3;
		compute_circle_dd(r, d);
		compute_circle_dd(gd, d2);
		compute_circle_dd(connect_rd, d3);
		img_buf.create(2 * r + 1, 2 * r + 1, CV_8UC1);
	}

	void remove(Mat & img, int x0, int y0, int dir) {
		CV_Assert(img.type() == CV_8UC1 && x0 >= r+w && y0 >= r+w && x0+r+w < img.cols && y0+r+w < img.rows);
		switch (dir) {
		case 0:
			//do up_down half-cover, which means overwrite up half circle and down half circle with new value
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
			break;
		case 1:
			//do left_right half-cover, which means overwrite left half circle and right half circle with new value
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
			break;
		default:
			//do up_down half-cover and left_right half-cover, and compute max overwrite value
			img_buf = Scalar::all(0);
			int lr_x = x0 - r, lr_y = y0 - r;
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
					img_buf.at<unsigned char>(y - lr_y, x - lr_x) = a;
				for (int y = y0 + 1; y <= y2; y++)
					img_buf.at<unsigned char>(y - lr_y, x - lr_x) = b;
				img_buf.at<unsigned char>(y0 - lr_y, x - lr_x) = (a + b) / 2;
			}
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
					p_img[x] = max(img_buf.at<unsigned char>(y - lr_y, x - lr_x), (unsigned char)a);
				for (int x = x0 + 1; x <= x2; x++)
					p_img[x] = max(img_buf.at<unsigned char>(y - lr_y, x - lr_x), (unsigned char)b);
				p_img[x0] = max(img_buf.at<unsigned char>(y - lr_y, x0 - lr_x), (unsigned char) ((a + b) / 2));
			}
			break;
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
		if (gd <= connect_rd) { //TODO remove
			for (int y = max(0, y0 - connect_rd); y <= min(mask.rows, y0 + connect_rd); y++) {
				unsigned char * p_mask = mask.ptr<unsigned char>(y);
				int x1 = max(0, x0 - d3[abs(y - y0)]);
				int x2 = min(mask.cols, x0 + d3[abs(y - y0)]);
				for (int x = x1; x <= x2; x++)
					p_mask[x] |= 2;
			}
		}
	}

	void finish(Mat &) {
	}
};

class ViaTwoCircleRemove : public ViaRemove {
protected:
	int r;
	int w;
	int pair_d;
	vector<int> dir;
	vector<Point> vias;
	ViaCircleRemove scr; //single circle remove

public:
	void prepare(ViaParameter &vp, PipeData & d) {
		r = vp.remove_rd;
		w = 3;	
		pair_d = vp.pair_distance;
		scr.prepare(vp, d);
	}

	void remove(Mat & , int x0, int y0, int _dir) {
		vias.push_back(Point(x0, y0));
		dir.push_back(_dir);
	}

	void remove_mask(Mat & mask, int x0, int y0) {
		scr.remove_mask(mask, x0, y0);
	}

	void finish(Mat & img) {
		vector<int> d1;
		Mat img_buf;
		compute_circle_dd(r, d1);
		img_buf.create(256, 256, CV_8UC1);
		int via_num = (int) vias.size();
		for (int i = 0; i < via_num; i++) {
			int pair = -1, pair_dir = 0;
			int x0, x1, y0, y1, distance = pair_d;
			//1 find pair via
			for (int j = i + 1; j < via_num; j++) {
				if (abs(vias[j].x - vias[i].x) <= distance && abs(vias[j].y - vias[i].y) <= 3) {
					if (vias[j].x > vias[i].x) {
						x0 = vias[i].x;	y0 = vias[i].y;	
						x1 = vias[j].x;	y1 = vias[j].y;
					}
					else {
						x0 = vias[j].x; y0 = vias[j].y;
						x1 = vias[i].x;	y1 = vias[i].y;
					}
					distance = abs(vias[j].x - vias[i].x);
					pair = j;
					pair_dir = 1;
					break;
				}
				if (abs(vias[j].x - vias[i].x) <= 3 && abs(vias[j].y - vias[i].y) <= distance) {
					if (vias[j].y > vias[i].y) {
						x0 = vias[i].x;	y0 = vias[i].y;
						x1 = vias[j].x;	y1 = vias[j].y;
					}
					else {
						x0 = vias[j].x; y0 = vias[j].y;
						x1 = vias[i].x;	y1 = vias[i].y;
					}
					distance = abs(vias[j].y - vias[i].y);
					pair = j;
					pair_dir = 0;
					break;
				}
			}
			if (pair >= 0) {				
				switch (dir[i]) {
				case 0:
					//do up-down 3cover, which means overwrite up-half circle and down-half circle and middle rect(if needed)
					for (int x = min(x0, x1) - r; x <= max(x0, x1) + r; x++) {
						int dy = d1[min(min(abs(x - x0), abs(x - x1)) , r)];
						int a = 0, b = 0;
						for (int i = 1; i <= w; i++) {
							a += img.at<unsigned char>(min(y0, y1) - dy - i, x);
							b += img.at<unsigned char>(max(y0, y1) + dy + i, x);
						}
						a = a / w;
						b = b / w;
						for (int y = min(y0, y1) - dy; y <= min(y0, y1); y++)
							img.at<unsigned char>(y, x) = a;
						for (int y = max(y0, y1); y <= max(y0, y1) + dy; y++)
							img.at<unsigned char>(y, x) = b;
						for (int y = min(y0, y1) + 1; y < max(y0, y1); y++)
							img.at<unsigned char>(y, x) = max(a, b);
					}
					break;
				case 1:
					//do left-right 3cover, which means overwrite left-half circle and right-half circle and middle rect(if needed)
					for (int y = min(y0, y1) - r; y <= max(y0, y1) + r; y++) {
						unsigned char * p_img = img.ptr<unsigned char>(y);
						int dx = d1[min(min(abs(y - y0), abs(y - y1)), r)];
						int a = 0, b = 0;
						for (int i = 1; i <= w; i++) {
							a += p_img[min(x0, x1) - dx - i];
							b += p_img[max(x0, x1) + dx + i];
						}
						a = a / w;
						b = b / w;
						for (int x = min(x0, x1) - dx; x <= min(x0, x1); x++)
							p_img[x] = a;
						for (int x = max(x0, x1); x <= max(x0, x1) + dx; x++)
							p_img[x] = b;
						for (int x = min(x0, x1) + 1; x < max(x0, x1); x++)
							p_img[x] = max(a, b);
					}					
					break;
				default:
					//do up-down 3cover and left-right 3cover and compute max overwrite value
					img_buf = Scalar::all(0);
					int lr_x = min(x0, x1) - r, lr_y = min(y0, y1) - r;
					for (int x = min(x0, x1) - r; x <= max(x0, x1) + r; x++) {
						int dy = d1[min(min(abs(x - x0), abs(x - x1)), r)];
						int a = 0, b = 0;
						for (int i = 1; i <= w; i++) {
							a += img.at<unsigned char>(min(y0, y1) - dy - i, x);
							b += img.at<unsigned char>(max(y0, y1) + dy + i, x);
						}
						a = a / w;
						b = b / w;
						for (int y = min(y0, y1) - dy; y <= min(y0, y1); y++)
							img_buf.at<unsigned char>(y - lr_y, x - lr_x) = a;
						for (int y = max(y0, y1); y <= max(y0, y1) + dy; y++)
							img_buf.at<unsigned char>(y - lr_y, x - lr_x) = b;
						for (int y = min(y0, y1) + 1; y < max(y0, y1); y++)
							img_buf.at<unsigned char>(y - lr_y, x - lr_x) = max(a, b);
					}
					for (int y = min(y0, y1) - r; y <= max(y0, y1) + r; y++) {
						unsigned char * p_img = img.ptr<unsigned char>(y);
						int dx = d1[min(min(abs(y - y0), abs(y - y1)), r)];
						int a = 0, b = 0;
						for (int i = 1; i <= w; i++) {
							a += p_img[min(x0, x1) - dx - i];
							b += p_img[max(x0, x1) + dx + i];
						}
						a = a / w;
						b = b / w;
						for (int x = min(x0, x1) - dx; x <= min(x0, x1); x++)
							p_img[x] = max(img_buf.at<unsigned char>(y - lr_y, x - lr_x), (unsigned char)a);
						for (int x = max(x0, x1); x <= max(x0, x1) + dx; x++)
							p_img[x] = max(img_buf.at<unsigned char>(y - lr_y, x - lr_x), (unsigned char)b);
						for (int x = min(x0, x1) + 1; x < max(x0, x1); x++)
							p_img[x] = max(img_buf.at<unsigned char>(y - lr_y, x - lr_x), (unsigned char)max(a, b));
					}
					break;
				}
				vias[pair] = vias[via_num - 1];
				dir[pair] = dir[--via_num];
			}
			else
				scr.remove(img, vias[i].x, vias[i].y, dir[i]);			
		}
		dir.clear();
		vias.clear();
	}
};

ViaRemove * ViaRemove::create_via_remove(ViaParameter &vp, PipeData & d)
{
	ViaRemove * vr = NULL;
	switch (vp.subtype) {
	case VIA_SUBTYPE_2CIRCLE:
	case VIA_SUBTYPE_3CIRCLE:
	case VIA_SUBTYPE_4CIRCLE:
	case VIA_SUBTYPE_2CIRCLEX:
	case VIA_SUBTYPE_3CIRCLEX:
		if (vp.pair_distance > 0)
			vr = new ViaTwoCircleRemove();
		else
			vr = new ViaCircleRemove();
		vr->prepare(vp, d);
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
	virtual int check_mark(int x0, int y0, const Mat & prob, Mat mask, int subtype, const Mat & already_mask) = 0;
	virtual void check_via_wire_connect(Mat & remove_via_mask, Mat & prob, Mat & img, Mat & ig) = 0;
};


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
	PAIR_ULL compute(int x0, int y0, const Mat & , const Mat & ig, const Mat & iig) {
		PAIR_ULL ret = make_pair(0xffffffffffffffffULL, 0xffffffffffffffffULL);
		unsigned s[24], sq[24];
		int sum[13], ssum[13];
		float part[3][13];
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
		score = sqrt((float) score);
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

	int check_mark(int x0, int y0, const Mat & prob, Mat mask, int type, const Mat & already_mask)
	{
		int ret = 0;
		CV_Assert(prob.rows * gs >= mask.rows && prob.cols * gs >= mask.cols && 
			mask.type() ==CV_32SC1 && prob.type() ==CV_64FC2);
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
			bool check = false;
#if 0			
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
#endif	
			if (check) //if already compute, continue
				continue;
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
		int gs = (int) ((float)img.rows / prob.rows + 0.5);
		int d = (wp.dis_vw1 - 1) / gs + 1;
		int fix_num=0;
		if (wp.dis_vw0==0 || wp.dis_vw1==0)
			return;
		if (wp.dis_vw0 > wp.dis_vw1) {
			qCritical("dis_vw0(%d) > dis_vw1(%d), error", wp.dis_vw0, wp.dis_vw1);
			return;
		}
		for (int y0 = 0; y0 < prob.rows; y0++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y0);
			for (int x0 = 0; x0 < prob.cols * 2; x0+=2) 
			if (PROB_SHAPE(p_prob[x0]) == BRICK_VIA) {
				int via_x0 = PROB_X(p_prob[x0]);
				int via_y0 = PROB_Y(p_prob[x0]);
				unsigned long long connect[4] = { 0, 0x7fff7fff, 0x7fff7fff, 0};
				for (int y = max(0, y0 - d); y <= min(prob.rows, y0 + d); y++) {
					unsigned long long * p_prob1 = prob.ptr<unsigned long long>(y);
					for (int x = max(0, x0 - d * 2); x <= min(prob.cols * 2 - 2, x0 + d * 2); x += 2)
					if (PROB_SHAPE(p_prob1[x]) <= BRICK_IN_USE && PROB_SHAPE(p_prob1[x]) != BRICK_NO_WIRE) {
						//1 check connect[0..3]
						switch (PROB_SHAPE(p_prob1[x])) {
						case BRICK_I_0:
						case BRICK_i_0:
						case BRICK_i_180:
							if (abs(PROB_X(p_prob1[x]) - via_x0) <= wp.dis_vw0 && abs(PROB_Y(p_prob1[x]) - via_y0) <= wp.dis_vw1) {
								if (y < y0 && PROB_Y(p_prob1[x]) > PROB_Y(connect[0])) //connect from up, choose nearest
									connect[0] = p_prob1[x];
								if (y > y0 && PROB_Y(p_prob1[x]) < PROB_Y(connect[2])) //connect from bottom, choose nearest
									connect[2] = p_prob1[x];
							}
							break;
						case BRICK_I_90:
						case BRICK_i_90:
						case BRICK_i_270:
							if (abs(PROB_X(p_prob1[x]) - via_x0) <= wp.dis_vw1 && abs(PROB_Y(p_prob1[x]) - via_y0) <= wp.dis_vw0) {
								if (x < x0 && PROB_X(p_prob1[x]) > PROB_X(connect[3])) //connect from left, choose nearest
									connect[3] = p_prob1[x];
								if (x > x0 && PROB_X(p_prob1[x]) < PROB_X(connect[1])) //connect from right, choose nearest
									connect[1] = p_prob1[x];
							}
							break;
						default:
							if (abs(PROB_X(p_prob1[x]) - via_x0) < abs(PROB_Y(p_prob1[x]) - via_y0)) {
								if (abs(PROB_X(p_prob1[x]) - via_x0) <= wp.dis_vw0 && abs(PROB_Y(p_prob1[x]) - via_y0) <= wp.dis_vw1) {
									if (y < y0 && PROB_Y(p_prob1[x]) > PROB_Y(connect[0])) //connect from up, choose nearest
										connect[0] = 0; //not handle other brick,
									if (y > y0 && PROB_Y(p_prob1[x]) < PROB_Y(connect[2])) //connect from bottom, choose nearest
										connect[2] = 0x7fff7fff; //not handle other brick
								}
							}
							else {
								if (abs(PROB_X(p_prob1[x]) - via_x0) <= wp.dis_vw1 && abs(PROB_Y(p_prob1[x]) - via_y0) <= wp.dis_vw0) {
									if (x < x0 && PROB_X(p_prob1[x]) > PROB_X(connect[3])) //connect from left, choose nearest
										connect[3] = 0; //not handle other brick
									if (x > x0 && PROB_X(p_prob1[x]) < PROB_X(connect[1])) //connect from right, choose nearest
										connect[1] = 0x7fff7fff; //not handle other brick
								}
							}
						}
					}
				}
				//connect[0..3] is ready now, it can only be BRICK_I or BRICK_i
				for (int i = 0; i <= 3; i++) 
				if (connect[i] > 0x7fffffff) {
					int l, r, t, b;
					switch (i) {
					case 0:
						l = PROB_X(connect[i]) + dx[14];
						r = PROB_X(connect[i]) + dx[15];
						t = PROB_Y(connect[i]) + dy[14];
						for (b = via_y0 - 1; b>=t; b--)
							if (!(via_mask.at<unsigned char>(b,l) & 1) && !(via_mask.at<unsigned char>(b,r) & 1))
								break;
						break;
					case 2:
						l = PROB_X(connect[i]) + dx[8];
						r = PROB_X(connect[i]) + dx[9];						
						b = PROB_Y(connect[i]) + dy[8];
						for (t = via_y0 + 1; b >= t; t++)
							if (!(via_mask.at<unsigned char>(t, l) & 1) && !(via_mask.at<unsigned char>(t, r) & 1))
								break;
						break;
					case 3:
						l = PROB_X(connect[i]) + dx[9];						
						t = PROB_Y(connect[i]) + dy[9];
						b = PROB_Y(connect[i]) + dy[15];
						for (r = via_x0 - 1; r >= l; r--)
							if (!(via_mask.at<unsigned char>(t, r) & 1) && !(via_mask.at<unsigned char>(b, r) & 1))
								break;
						break;
					case 1:						
						r = PROB_X(connect[i]) + dx[8];
						t = PROB_Y(connect[i]) + dy[8];
						b = PROB_Y(connect[i]) + dy[14];
						for (l = via_x0 + 1; r >= l; l++)
							if (!(via_mask.at<unsigned char>(t, l) & 1) && !(via_mask.at<unsigned char>(b, l) & 1))
								break;
						break;
					}
					CV_Assert(l>=0 && t>=0 && r<img.cols && b<img.rows);
					int th = (wp.gray_i + wp.gray_w) / 2;
					int sum = ig.at<int>(b + 1, r + 1) + ig.at<int>(t, l) - ig.at<int>(b + 1, l) - ig.at<int>(t, r + 1);
					if (b == t || r == l)
						continue;
					float avg = sum / ((b - t) * (r - l));
					bool is_connect;
					if (avg < th)
						is_connect = false;
					else
						is_connect = true;
					switch (i) {
					case 0:
						if (is_connect && PROB_SHAPE(connect[i])==BRICK_i_0) {
							SET_PROB_SHAPE(connect[i],BRICK_I_0);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;
						}
						if (!is_connect && PROB_SHAPE(connect[i]) == BRICK_I_0) {
							SET_PROB_SHAPE(connect[i],BRICK_i_0);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;
						}
						if (!is_connect && PROB_SHAPE(connect[i]) == BRICK_i_180) {
							SET_PROB_SHAPE(connect[i],BRICK_INVALID);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						break;	
					case 1:
						if (is_connect && PROB_SHAPE(connect[i]) == BRICK_i_90) {
							SET_PROB_SHAPE(connect[i],BRICK_I_90);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						if (!is_connect && PROB_SHAPE(connect[i]) == BRICK_I_90) {
							SET_PROB_SHAPE(connect[i],BRICK_i_90);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						if (!is_connect && PROB_SHAPE(connect[i]) == BRICK_i_270) {
							SET_PROB_SHAPE(connect[i],BRICK_INVALID);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						break;
					case 2:
						if (is_connect && PROB_SHAPE(connect[i]) == BRICK_i_180) {
							SET_PROB_SHAPE(connect[i],BRICK_I_0);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						if (!is_connect && PROB_SHAPE(connect[i]) == BRICK_I_0) {
							SET_PROB_SHAPE(connect[i],BRICK_i_180);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						if (!is_connect && PROB_SHAPE(connect[i]) == BRICK_i_0) {
							SET_PROB_SHAPE(connect[i],BRICK_INVALID);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						break;
					case 3:
						if (is_connect && PROB_SHAPE(connect[i]) == BRICK_i_270) {
							SET_PROB_SHAPE(connect[i],BRICK_I_90);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						if (!is_connect && PROB_SHAPE(connect[i]) == BRICK_I_90) {
							SET_PROB_SHAPE(connect[i],BRICK_i_270);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						if (!is_connect && PROB_SHAPE(connect[i]) == BRICK_i_90) {
							SET_PROB_SHAPE(connect[i],BRICK_INVALID);
							SET_PROB_SCORE(connect[i], PROB_SCORE(connect[i]) - 1);
							push_new_prob(prob, connect[i], gs);
							fix_num++;									
						}
						break;	
					}
				}
			}
		}
		if (fix_num!=0)
			qWarning("fix_num=%d", fix_num);
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
	default:
		qCritical("WireComputeScore create failed, subtype=%d", wp.subtype);
		break;
	}
	return wc;
}

/*		31..24  23..16		15..8		7..0
opt0:	  1  border_size compute_border gs
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

		if (gs > 32 || gs<=3 || gs * 2 > compute_border || compute_border >= border_size) 
			qCritical("gs invalid");
		else
			d.l[layer].reinit(gs, compute_border, border_size);
		
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
opt3:           k2       k1      k0      k0 is dark gray level compress rate, k1 & k2 is rate between most dark and middle bright
opt4:           k2       k1      k0      k0 is middle bright level compress rate, k1 & k2 is rate between middle bright and most bright
opt5:           k2       k1      k0      k0 is most bright compress rate, k1 & k2 is rate between most bright
sep_min & sep_max means gray level difference
sep_min better > wsize/2, e.g. M0.sep_min > M0.wsize/2
wsize means gray level window, e.g. [80,90] consider as GRAY_M0 level, wsize=10
Choose gray level between [last gray level+ last level wsize/2+ sep_min, last gray level+ last level wsize/2 +sep_max], 
e.g. GRAY_M0 is in [GRAY_L0+L0.wsize/2 + M0.sep_min, GRAY_L0+L0.wsize/2 +M0.sep_max]
normally choose k2 & k0 = 100, k1 < 100
method_opt
0: for gray level turn_points output
*/
static void imgpp_adjust_gray_lvl(PipeData & d, ProcessParameter & cpara)
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
		int choose = base + lmh[i].sep_min, max_sum = 0;
		qInfo("imgpp_adjust_gray_lvl %d:sep_min=%d, sep_max=%d,wsize=%d,dis_min=%f,dis_max=%f, k0=%f, k1=%f, k2=%f", i, 
			lmh[i].sep_min, lmh[i].sep_max, lmh[i].wsize, lmh[i].dis_min, lmh[i].dis_max, lmh[i].k0, lmh[i].k1, lmh[i].k2);
		if (lmh[i].wsize > 100) {
			qCritical("imgpp_adjust_gray_lvl wsize(%d) > 100", lmh[i].wsize);
			return;
		}
		if (lmh[i].dis_min >= 1 || lmh[i].dis_max > 1) {
			qCritical("imgpp_adjust_gray_lvl dis_min(%d), dis_max(%d) shoule < 1", lmh[i].dis_min, lmh[i].dis_max);
			return;
		}
		if (lmh[i].k0 > 1) {
			qCritical("imgpp_adjust_gray_lvl k0(%d) > 1", lmh[i].k0);
			return;
		}
		if (i > 0 && lmh[i].sep_min - lmh[i].wsize / 2 < 6)
			qWarning("imgpp_adjust_gray_lvl Warning sep_min(%d) - wize(%d)/2 < 6", lmh[i].sep_min, lmh[i].wsize);
		int th = lmh[i].dis_max * stat[256];
		for (int j = base + lmh[i].sep_min; j <= min(255, base + lmh[i].sep_max); j++) {
			int kb = max(base + 1, j - lmh[i].wsize / 2), ke = min(j + (lmh[i].wsize - 1) / 2, 255);
			int sum = 0;
			for (int k = kb; k <= ke; k++)
				sum += stat[k];
			if (sum > max_sum && sum < th) {
				max_sum = sum;
				choose = j;
			}
		}
		if (max_sum < lmh[i].dis_min * stat[256])
			qCritical("Error choose=%d, max_sum(%d) < dis_min(%d)", choose, max_sum, (int) (lmh[i].dis_min * stat[256]));
		if (max_sum > th)
			qCritical("Error choose=%d, max_sum(%d) > dis_max(%d)", choose, max_sum, th);
		if (choose == base + lmh[i].sep_min)
			qWarning("consider to make sep_min low, because choose%d = base(%d) + sep_min(%d)", choose, base, lmh[i].sep_min);
		if (choose == base + lmh[i].sep_max)
			qWarning("consider to make sep_max high, because choose%d = base(%d) + sep_max(%d)", choose, base, lmh[i].sep_max);
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
	int shape1, shape2, shape3;
	float a0, a1, a2;
	float arf;
};
/*     31..24 23..16   15..8   7..0
opt0:   inc1   inc0   w_long1 w_long0 
opt1:up_prob search_opt  th    w_num
opt2:         w_pattern w_dir  w_type    
opt3:         w_pattern w_dir  w_type    
opt4:         w_pattern w_dir  w_type 
opt5:         w_pattern w_dir  w_type    
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
	
	struct WireDetectInfo {
		int w_pattern;
		int w_dir;
		int w_type;
		int w_wide;
		int i_wide;
		float gamma;
	} wpara[] = {
			{ cpara.opt2 >> 16 & 0xff, cpara.opt2 >> 8 & 0xff, cpara.opt2 & 0xff, 0, 0, 0}, 
			{ cpara.opt3 >> 16 & 0xff, cpara.opt3 >> 8 & 0xff, cpara.opt3 & 0xff, 0, 0, 0 },
			{ cpara.opt4 >> 16 & 0xff, cpara.opt4 >> 8 & 0xff, cpara.opt4 & 0xff, 0, 0, 0 },
			{ cpara.opt5 >> 16 & 0xff, cpara.opt5 >> 8 & 0xff, cpara.opt5 & 0xff, 0, 0, 0 },
	};

	vector<WireKeyPoint> wires[2];
	int w_check[2][256] = { 0 };
	int x0 = w_long1 / 2, y0 = w_long0 / 2;
	for (int i = 0; i < w_num; i++) {
		VWParameter * vw = d.l[layer].vw.get_vw_para(wpara[i].w_pattern, wpara[i].w_type, WIRE_SUBTYPE_13RECT);
		if (vw == NULL) {
			qCritical("coarse_line_search invalid wire info dir=%d, type=%d, pattern=%d", wpara[i].w_dir, wpara[i].w_type, wpara[i].w_pattern);
			return;
		}
		wpara[i].w_wide = (wpara[i].w_dir == 0) ? vw->w.w_wide : vw->w.w_high;
		wpara[i].i_wide = (wpara[i].w_dir == 0) ? vw->w.i_wide : vw->w.i_high;
		wpara[i].gamma = vw->w.arfactor / 100.0;
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
			wire.arf = pow(((wpara[i].i_wide * 2 + wpara[i].w_wide) * w_long0), 2 * wpara[i].gamma);
			wire.shape1 = BRICK_I_0;
			wire.shape2 = BRICK_II_0;
			wire.shape3 = BRICK_II_180;
			wires[0].push_back(wire);
			x0 = max(x0, wpara[i].w_wide / 2 + wpara[i].i_wide + 1);
			w_check[0][wpara[i].w_type] = wpara[i].w_wide / 2 + wpara[i].i_wide;
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
			wire.arf = pow(((wpara[i].i_wide * 2 + wpara[i].w_wide) * w_long1), 2 * wpara[i].gamma);
			wire.shape1 = BRICK_I_90;
			wire.shape2 = BRICK_II_90;
			wire.shape3 = BRICK_II_270;
			wires[1].push_back(wire);
			y0 = max(y0, wpara[i].w_wide / 2 + wpara[i].i_wide + 1);
			w_check[1][wpara[i].w_type] = wpara[i].w_wide / 2 + wpara[i].i_wide;
			break;
		default:
			qCritical("bad dir %d", wpara[i].w_dir);
			break;
		}

	}

	int gl = find_index(d.l[layer].v[idx].d, (int) GRAY_L0);
	gl = d.l[layer].v[idx].d.at<int>(gl, 2);
	int gm = find_index(d.l[layer].v[idx].d, (int) GRAY_M0);
	gm = d.l[layer].v[idx].d.at<int>(gm, 2);
	qInfo("coarse_line_search, gl=%d, gm=%d", gl, gm);

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
				unsigned s0 = MAKE_S(0xffff, 0xff, BRICK_III);;
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
					score4 = MAKE_S(score4, i, BRICK_III);
					s1 = min(min(min(score1, score0), min(score2, score3)), score4);
					if (S_SHAPE(s0) == BRICK_I_0 || S_SHAPE(s0) == BRICK_I_90) {
						if (S_SHAPE(s1) == BRICK_I_0 || S_SHAPE(s1) == BRICK_I_90)
							s0 = min(s0, s1);
					}
					else {
						if (S_SHAPE(s1) == BRICK_I_0 || S_SHAPE(s1) == BRICK_I_90)
							s0 = s1;
						else
							s0 = min(s0, s1);
					}
					for (int j = 0; j < 8; j++) {
						w[i].p_ig[j] += dx;
						w[i].p_iig[j] += dx;
					}
				}
				unsigned score = S_SCORE(s0);
				unsigned type = S_TYPE(s0);
				score = sqrt(score * w[type].arf);
				CV_Assert(score < 65536);
				s0 = MAKE_S(score, w[type].type, S_SHAPE(s0));

				push_new_prob(prob, x, y, s0, d.l[layer].gs);
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
			if (PROB_SHAPE(p_prob[2 * x]) == BRICK_I_0 && PROB_SCORE(p_prob[2 * x]) <= (unsigned) (th * th) ||
				PROB_SHAPE(p_prob[2 * x]) == BRICK_I_90 && PROB_SCORE(p_prob[2 * x]) <= (unsigned) (th * th)) {
			bool pass = true; 
			/*pass = true means prob BRICK_I_0 < all row BRICK_I_0, && < all nearby guard BRICK_I_90
			                 or prob BRICK_I_90< all column BRICK_I_90, && < all nearby guard BRICK_I_0
			*/
			unsigned long long prob0 = p_prob[2 * x];
			int cr = (p_prob[2 * x] == BRICK_I_0) ? w_inc1 / 2 + 1 : w_inc0 / 2 + 1;
			int guard = (cr - 1) / d.l[layer].gs + 1;
			int y1 = max(0, y - guard), y2 = min(prob.rows - 1, y + guard);
			int x1 = max(0, x - guard), x2 = min(prob.cols - 1, x + guard);
			for (int yy = y1; yy <= y2; yy++)  {
				unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
				for (int xx = x1; xx <= x2; xx++) {
					if (p_prob2[2 * xx] < prob0 && PROB_SHAPE(p_prob2[2 * xx]) != PROB_SHAPE(prob0)
						&& PROB_SHAPE(p_prob2[2 * xx]) != BRICK_VIA) {
						if (abs(PROB_X(p_prob2[2 * xx]) - PROB_X(prob0)) <= cr &&
							abs(PROB_Y(p_prob2[2 * xx]) - PROB_Y(prob0)) <= cr) {
							pass = false;							
							yy = y2;
							break;
						}
					}
				}
			}
			cr = w_check[0][PROB_TYPE(prob0)];
			if (PROB_SHAPE(p_prob[2 * x]) == BRICK_I_0) {
				for (int xx = x1; xx <= x2; xx++)
					if (p_prob[2 * xx] < prob0) {
					if (abs(PROB_X(p_prob[2 * xx]) - PROB_X(prob0)) <= cr)
						pass = false;
					}
			}
			else {
				for (int yy = y1; yy <= y2; yy++) {
					unsigned long long prob1 = prob.at<unsigned long long>(yy, 2 * x);
					if (prob1 < prob0) {
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
				circle(debug_draw, Point(PROB_X(p_prob[2*x]), PROB_Y(p_prob[2*x])), 2, Scalar::all(255));
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
		if (v_wide[i] / 2 >= d.l[layer].compute_border || v_percent[i] >= 50) {
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
	int idx3 = cpara.method_opt >> 12 & 0xf;
	if (d.l[layer].v[idx3].type != TYPE_SHADOW_PROB) {
		qCritical("fine_via_search search mask idx3[%d], error", idx3, d.l[layer].v[idx3].type);
		return;
	}
	Mat & mask = d.l[layer].v[idx1].d;
	int vnum = cpara.opt0 & 0xff;
	int update_fv = cpara.opt0 >> 8 & 0xff;
	qInfo("fine_via_search, l=%d, g_idx=%d, vmsk_idx=%d, prob_idx=%d, gl=%d, gm=%d, gh=%d, vnum=%d, update_fv=%d", 
		layer, idx, idx1, idx3, glv[0], glv[1], glv[2], vnum, update_fv);
	
#define MAX_VIA_NUM 3	
	if (vnum > MAX_VIA_NUM || vnum==0) {
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
	Mat & prob = d.l[layer].v[idx3].d;
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
				bool pass = PROB_SCORE(prob0) < v_th[gi] * v_th[gi]; //pass means it is lowest prob among nearby guard probs,pass=true => pass2=true
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
				if (pass2) {				
					unsigned long long score = prob0;
					SET_PROB_TYPE(score, v_type[gi]);
					push_new_prob(d.l[layer].prob, score, d.l[layer].gs);
				}
				if (pass)
					via_loc.push_back(ViaInfo(Point(PROB_X(prob0), PROB_Y(prob0)), v_type[gi], v_subtype[gi], v_pattern[gi], v_pair_d[gi]));
			}
	}

	for (int i = 0; i < (int)via_loc.size(); i++) {
		for (int j = 0; j < i; j++) {
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
			if (via_loc[i].xy.y - via_loc[j].xy.y > via_loc[i].pair_d + d.l[layer].gs)
				break;
		}
		for (int j = i + 1; j < (int)via_loc.size(); j++) {
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
			if (via_loc[j].xy.y - via_loc[i].xy.y > via_loc[i].pair_d + d.l[layer].gs)
				break;
		}
	}
	Mat debug_draw;
	if (cpara.method & OPT_DEBUG_EN)
		debug_draw = img.clone();

	for (int i = 0; i < (int)via_loc.size(); i++) {
		if (update_fv)
			d.l[layer].viaset.push_back(QPoint(via_loc[i].xy.x, via_loc[i].xy.y));
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
remove_opt is Remove via option, REMOVE_VIA_MASK means mark mask
method_opt
0: N/A
1: for mask output
2: for gray level turn_points  input
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
		vr[i].reset(ViaRemove::create_via_remove(via_para, d));
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
		vr[sel]->remove(img, x0, y0, dir);
		if (remove_opt & REMOVE_VIA_MASK)
			vr[sel]->remove_mask(mask, x0, y0);
	}
	for (int i = 0; i < vnum; i++)
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
	if (dir==0) {
		if (w[dir].empty())
			continue;
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols; x++) {
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
					int x1 = max(x0 / d.l[layer].gs + w[dir][i].x1, 0), x2 = min(x0 / d.l[layer].gs + w[dir][i].x2, prob.cols - 1);
					int y1 = max(y0 / d.l[layer].gs + w[dir][i].y1, 0), y2 = min(y0 / d.l[layer].gs + w[dir][i].y2, prob.rows - 1);
					check = false;
					for (int yy = y1; yy <= y2; yy++) {
						unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
						for (int xx = x1; xx <= x2; xx++)
							if (PROB_TYPESHAPE(p_prob2[2 * xx]) == w[dir][i].type_shape) {
								if (abs(PROB_X(p_prob2[2 * xx]) - x0) <= w[dir][i].cx &&
									abs(PROB_Y(p_prob2[2 * xx]) - y0) <= w[dir][i].cy) {
									if (mark.at<unsigned char>(yy, xx) & 1) 
										qWarning("hotpoint2fine_search_mask intersect at (x=%d,y=%d) for (x0=%d,y0=%d), maybe cwide=%d too big",
											xx, yy, x0, y0, w[dir][i].cx);
									
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
					if (PROB_TYPESHAPE(p_prob[2 * x]) == w[dir][i].type_shape && (mark.at<unsigned char>(y, x) & 2) == 0)
						break;
				if (i == w[dir].size())
					continue;
				int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]);
				mark.at<unsigned char>(y, x) |= 2;
				mark_line(mask, Point(x0, y0), Point(max(0, x0 - w[dir][i].extend), y0), w[dir][i].mask);
				bool check;
				do {
					int x1 = max(x0 / d.l[layer].gs + w[dir][i].x1, 0), x2 = min(x0 / d.l[layer].gs + w[dir][i].x2, prob.cols - 1);
					int y1 = max(y0 / d.l[layer].gs + w[dir][i].y1, 0), y2 = min(y0 / d.l[layer].gs + w[dir][i].y2, prob.rows - 1);
					check = false;
					for (int xx = x1; xx <= x2; xx++)
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob.ptr<unsigned long long>(yy);
							if (PROB_TYPESHAPE(p_prob2[2 * xx]) == w[dir][i].type_shape) {
								if (abs(PROB_X(p_prob2[2 * xx]) - x0) <= w[dir][i].cx &&
									abs(PROB_Y(p_prob2[2 * xx]) - y0) <= w[dir][i].cy) {
									if (mark.at<unsigned char>(yy, xx) & 2) {
										qWarning("hotpoint2fine_search_mask intersect at (x=%d,y=%d) for (x0=%d,y0=%d), maybe cwide=%d too big",
											xx, yy, x0, y0, w[dir][i].cy);
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

struct WireExtendInfo {
	int w_type;
	int i_wide, w_wide, w_len_near, w_len_far;
	int extend;
	int gray_i, gray_w;
};

class ViaWireConnect {
public:
	virtual void prepare(ViaParameter & v, PipeDataPerLayer & d) = 0;
	virtual void add_wire(WireExtendInfo & w, PipeDataPerLayer & d) = 0;
	virtual void add_brick_hlmask(PipeDataPerLayer & d, Mat & hl_mask)	 = 0;
	static ViaWireConnect * create_via_wire_connect(ViaParameter & v, PipeDataPerLayer & d);
};

void add_new_shape(Mat & prob, int x, int y, unsigned s, unsigned gs) {
	int y0 = y / gs, x0 = x / gs;
	unsigned long long * p_prob = prob.ptr<unsigned long long>(y0, x0);
	CV_Assert(prob.type() == CV_64FC2 && y >= 0 && x >= 0 && x0 < prob.cols && y0 < prob.rows);
	unsigned score = min(PROB_SCORE(p_prob[0]), S_SCORE(s));
	unsigned shape = brick_conn.quick_shape_add(PROB_SHAPE(p_prob[0]), S_SHAPE(s));
	unsigned type = S_TYPE(s);
	s = MAKE_S(score, type, shape);
	p_prob[0] = MAKE_PROB(s, x, y);
}

struct Border {
	vector<Point> border_scan[4]; //border_scan is inited by subclass
};

struct WireExtendShapeConst {
	int sel[4];
	int has_wire;
} wire_extend_shape[] = {
	//  0  1  2  3
	{ { 0, 1, 0, 1 }, 1 },
	{ { 2, 2, 2, 0 }, 0 },
	{ { 2, 0, 2, 2 }, 0 },
};

class ExtendWireConnect {
protected:
	Mat ig;
	int offset[4][10];	
	float arf; //a is each rec_area / total_area
	int area[4]; //area is each rec_area;
	int gi, gm;

public:
	//it compute point offset for each area
	void prepare(ViaParameter & v, WireExtendInfo & w, PipeDataPerLayer & d) {
		if (d.border_size < w.w_len_far + w.w_len_near + v.connect_rd) {
			qCritical("border_size(%d) < w_len_far(%d) + w_len_near(%d) + connect_rd(%d)", d.border_size,
				w.w_len_far, w.w_len_near, v.connect_rd);
			return;
		}
		if (d.border_size < w.w_wide + w.i_wide + v.connect_rd) {
			qCritical("border_size(%d) < w_wide(%d) + i_wide(%d) + connect_rd(%d)", d.border_size,
				w.w_wide, w.i_wide, v.connect_rd);
			return;
		}
		d.validate_ig();
		ig = d.ig;
		gi = w.gray_i;
		gm = w.gray_w;
		for (int dir = 0; dir < 4; dir++) {
			int dx[10], dy[10];
			switch (dir) {
			case DIR_UP:
				dx[8] = -w.w_wide / 2, dy[8] = 0;
				dx[9] = w.w_wide - w.w_wide / 2, dy[9] = 0;
				dx[4] = dx[8] - w.i_wide, dy[4] = -w.w_len_near;
				dx[5] = dx[8], dy[5] = dy[4];
				dx[6] = dx[9], dy[6] = dy[4];
				dx[7] = dx[9] + w.i_wide, dy[7] = dy[4];
				dx[0] = dx[4], dy[0] = dy[4] - w.w_len_far;
				dx[1] = dx[8], dy[1] = dy[0];
				dx[2] = dx[9], dy[2] = dy[0];
				dx[3] = dx[7], dy[3] = dy[0];
				break;
			case DIR_RIGHT:
				dx[9] = 0, dy[9] = -w.w_wide / 2;
				dx[8] = 0, dy[8] = w.w_wide - w.w_wide / 2;
				dx[7] = w.w_len_near, dy[7] = dy[9] - w.i_wide;
				dx[6] = dx[7], dy[6] = dy[9];
				dx[5] = dx[7], dy[5] = dy[8];
				dx[4] = dx[7], dy[4] = dy[8] + w.i_wide;
				dx[3] = dx[7] + w.w_len_far, dy[3] = dy[7];
				dx[2] = dx[3], dy[2] = dy[9];
				dx[1] = dx[3], dy[1] = dy[8];
				dx[0] = dx[3], dy[0] = dy[4];
				break;
			case DIR_DOWN:
				dx[9] = -w.w_wide / 2, dy[9] = 0;
				dx[8] = w.w_wide - w.w_wide / 2, dy[8] = 0;
				dx[7] = dx[9] - w.i_wide, dy[7] = w.w_len_near;
				dx[6] = dx[9], dy[6] = dy[7];
				dx[5] = dx[8], dy[5] = dy[7];
				dx[4] = dx[8] + w.i_wide, dy[4] = dy[7];
				dx[3] = dx[7], dy[3] = dy[7] + w.w_len_far;
				dx[2] = dx[9], dy[2] = dy[3];
				dx[1] = dx[8], dy[1] = dy[3];
				dx[0] = dx[4], dy[0] = dy[3];
				break;
			case DIR_LEFT:
				dx[8] = 0, dy[8] = -w.w_wide / 2;
				dx[9] = 0, dy[9] = w.w_wide - w.w_wide / 2;
				dx[4] = -w.w_len_near, dy[4] = dy[8] - w.i_wide;
				dx[5] = dx[4], dy[5] = dy[8];
				dx[6] = dx[4], dy[6] = dy[9];
				dx[7] = dx[4], dy[7] = dy[9] + w.i_wide;
				dx[0] = dx[4] - w.w_len_far, dy[0] = dy[4];
				dx[1] = dx[0], dy[1] = dy[8];
				dx[2] = dx[0], dy[2] = dy[9];
				dx[3] = dx[0], dy[3] = dy[7];
				break;
			}
			for (int j = 0; j < 10; j++)
				offset[dir][j] = (dy[j] * ig.step.p[0] + dx[j] * ig.step.p[1]) / sizeof(unsigned);
		}
		
		area[0] = w.i_wide * w.w_len_far;
		area[1] = w.w_wide * w.w_len_far;
		area[2] = w.i_wide * w.w_len_far;
		area[3] = w.w_wide * w.w_len_near;
		float tot_area_1 = area[0] + area[1] + area[2] + area[3];
		arf = 1.0 / pow(tot_area_1, 0.5);	
	}

	/*
	Input via,
	Input dir
	Input bd, via + bd make up border line
	Return if no wire return 0xffffffffffffffff, else return wire's PROB
	*/
	unsigned long long compute_prob(const ViaInfo & via, int dir, Border & bd) {
		const unsigned * p_ig;
		int sum[4];
		unsigned s[10];
		int part[3][4];
		unsigned long long min_s = 0xffffffffffffffff;//it stores only wire score; if no wire, it maintain 0xffffffffffffffff
		
		for (int i = 0; i < (int)bd.border_scan[dir].size(); i++) { //scan border
			Point bp = via.xy + bd.border_scan[dir][i];
			p_ig = ig.ptr<unsigned>(bp.y, bp.x);
			for (int j = 0; j < 10; j++)
				s[j] = p_ig[offset[dir][j]];
			sum[0] = s[0] + s[5] - s[1] - s[4]; //area 0 gray sum
			sum[1] = s[1] + s[6] - s[2] - s[5]; //area 1 gray sum
			sum[2] = s[2] + s[7] - s[3] - s[6]; //area 2 gray sum
			sum[3] = s[5] + s[9] - s[6] - s[8]; //area 3 gray sum
			for (int j = 0; j < 4; j++) {
				part[0][j] = abs(sum[j] - gi * area[j]); //area i prob for insu
				part[1][j] = abs(sum[j] - gm * area[j]); //area i prob for wire
				part[2][j] = min(part[0][j], part[1][j]); //area i prob for not-care
			}
			
			int has_wire;
			unsigned min_score = 0xffffffff;
			//compute score for wire, nowire
			for (int j = 0; j < sizeof(wire_extend_shape) / sizeof(wire_extend_shape[0]); j++) { //scan for each shape
				unsigned score = 0;
				for (int k = 0; k < 4; k++)
					score += part[wire_extend_shape[j].sel[k]][k];
				if (score < min_score) {
					min_score = score;
					has_wire = wire_extend_shape[j].has_wire;
				}
			}
			if (has_wire) { //if it has wire, update best wire value
				min_score = min_score * arf * 8;
				CV_Assert(min_score < 65535);
				min_score = MAKE_S(min_score, 0, (dir & 1) ? BRICK_I_90 : BRICK_I_0);
				min_s = min(min_s, MAKE_PROB(min_score, bp.x + dxy[dir][1], bp.y + dxy[dir][0]));
			}
		}
		return min_s;
	}

	//Input: prob is destination brick, it can only be BRICK_I_0 or BRICK_I_90
	void add_path(PipeDataPerLayer & d, const ViaInfo & via, unsigned long long prob) {
		unsigned type = PROB_TYPE(prob);
		int dst_x = PROB_X(prob) / d.gs;
		int dst_y = PROB_Y(prob) / d.gs;
		int src_x = via.xy.x / d.gs;
		int src_y = via.xy.y / d.gs;
		int dx = (dst_x > src_x) ? 1 : ((dst_x == src_x) ? 0 : -1);
		int dy = (dst_y > src_y) ? 1 : ((dst_y == src_y) ? 0 : -1);
		if (dx == 0 && dy == 0) {
			qCritical("add path via_loc(x=%d,y=%d) in same grid as prob(x=%d,y=%d)", via.xy.x, via.xy.y, PROB_X(prob), PROB_Y(prob));
			return;
		}
		int x1 = via.xy.x, y1 = via.xy.y; //x1, y1 is middle station		
		if (dy != 0 && dx != 0) {
			unsigned long long b0 = d.prob.at<unsigned long long>(src_y + dy, src_x);
			unsigned long long b1 = d.prob.at<unsigned long long>(src_y, src_x + dx);
			if (PROB_SHAPE(b0) != BRICK_INVALID) //up or down already has path
				y1 = PROB_Y(prob); //go up or down first
			else
			if (PROB_SHAPE(b1) != BRICK_INVALID) //left or right already has path
				x1 = PROB_X(prob); //go left or right first
			else //No path exist
			if (PROB_SHAPE(prob) == BRICK_I_0) //for BRICK_I_0, go left or right first
				x1 = PROB_X(prob);
			else
			if (PROB_SHAPE(prob) == BRICK_I_90) //for BRICK_I_90, go up or down first
				y1 = PROB_Y(prob);
		} else
		if (dx == 0) //normally it is BRICK_I_0
			y1 = PROB_Y(prob);
		else
		if (dy == 0) //normally it is BRICK_I_90
			x1 = PROB_X(prob);
		int x0 = via.xy.x;
		int y0 = via.xy.y;		
		dx = (x0 > x1) ? -d.gs : ((x0 == x1) ? 0 : d.gs);
		dy = (y0 > y1) ? -d.gs : ((y0 == y1) ? 0 : d.gs);
		int mid_x = x1 / d.gs;
		int mid_y = y1 / d.gs;
		CV_Assert(dx == 0 && dy != 0 || dy == 0 && dx != 0);
		unsigned shape = (dx == 0) ? BRICK_I_0 : BRICK_I_90;
		unsigned score = 1; // (dx == 0) ? ((dy < 0) ? DIR_DOWN : DIR_UP) : ((dx < 0) ? DIR_RIGHT : DIR_LEFT);
		unsigned s = MAKE_S(score, type, shape);
		while (1) { //1 mark BRICK_I from via to mid
			x0 += dx;
			y0 += dy;
			if (x0 / d.gs == mid_x && y0 / d.gs == mid_y)
				break;
			add_new_shape(d.prob, x0, y0, s, d.gs);
		}
		shape = (dx == 0) ? ((dy < 0) ? BRICK_i_180 : BRICK_i_0) : ((dx < 0) ? BRICK_i_90 : BRICK_i_270);
		s = MAKE_S(score, type, shape);
		add_new_shape(d.prob, x1, y1, s, d.gs);
		x0 = x1;
		y0 = y1;
		if (mid_x == dst_x && mid_y == dst_y) {
			shape = (dx == 0) ? BRICK_I_0 : BRICK_I_90;
			dx = 0;
			dy = 0;
		}
		else {
			dx = (mid_x > dst_x) ? -d.gs : ((mid_x == dst_x) ? 0 : d.gs);
			dy = (mid_y > dst_y) ? -d.gs : ((mid_y == dst_y) ? 0 : d.gs);
			CV_Assert(dx == 0 && dy != 0 || dy == 0 && dx != 0);
			shape = (dx == 0) ? ((dy < 0) ? BRICK_i_0 : BRICK_i_180) : ((dx < 0) ? BRICK_i_270 : BRICK_i_90);
		}
		s = MAKE_S(score, type, shape);
		add_new_shape(d.prob, x0, y0, s, d.gs);
		shape = (dx == 0) ? BRICK_I_0 : BRICK_I_90;
		score = 5;
		s = MAKE_S(score, type, shape);
		while (1) { //3 mark BRICK_I from mid to dst
			x0 += dx;
			y0 += dy;
			if (x0 / d.gs == dst_x && y0 / d.gs == dst_y)
				break;
			add_new_shape(d.prob, x0, y0, s, d.gs);
		}
	}

	void mark_extend(Mat & m, const ViaInfo & via, const WireExtendInfo & wire, unsigned long long prob, int dir) {
		Point pt1, pt2;
		pt1.x = PROB_X(prob) + dxy[dir][1] * wire.extend;
		pt1.y = PROB_Y(prob) + dxy[dir][0] * wire.extend;
		pt2.x = PROB_X(prob) + dxy[dir][1] * (wire.extend + 3);
		pt2.y = PROB_Y(prob) + dxy[dir][0] * (wire.extend + 3);
		if (pt1.x >= 0 && pt1.y >= 0 && pt2.x >= 0 && pt2.y >= 0 &&
			pt1.x < m.cols && pt1.y < m.rows && pt2.x < m.cols && pt2.y < m.rows)
			mark_line(m, pt1, pt2, 1 << wire.w_type);
	}

	static float compute_connect_gray(const Mat & img, const vector<Point> & bd, int dir, int cgray_d) {
		int sum0 = 0, sum1 =0;
		int check_d = cgray_d + 1;
		vector<unsigned> cgray(check_d);

		CV_Assert(img.type() == CV_8UC1);
		for (int i = 0; i < (int)bd.size(); i++) {
			CV_Assert(bd[i].x + dxy[dir][0] * cgray_d >= 0 && bd[i].x + dxy[dir][0] * cgray_d < img.cols);
			CV_Assert(bd[i].y + dxy[dir][1] * cgray_d >= 0 && bd[i].y + dxy[dir][1] * cgray_d < img.rows);
			int t = 0;
			for (int j = 0, dx = 0, dy = 0; j < check_d; j++, dx += dxy[dir][0], dy += dxy[dir][1]) {
				cgray[j] = img.at<unsigned char>(bd[i].y + dy, bd[i].x + dx);
				t += cgray[j];
			}
			sum0 += t - cgray[0];
			sum1 += t - cgray[check_d - 1];
		}

		int sum = min(sum0, sum1);
		return (float)sum / (bd.size() * cgray_d);
	}
};

class SingleViaWireConnect : public ViaWireConnect {
protected:
	vector<ExtendWireConnect> ewc;
	vector<Border> bd;
	ViaParameter v;
	vector<WireExtendInfo> w;
	float gray_th;
	Mat img;
	Border circle;

public:
	void prepare(ViaParameter & _v, PipeDataPerLayer & d) {
		v = _v;
		img = d.img;
		vector<ViaInfo> & via_info = d.via_info;
		for (int i = 0; i < via_info.size(); i++) {
			//if it is within via remove range, mark it BRICK_INVALID
			for (int y = via_info[i].xy.y - v.connect_rd; y < via_info[i].xy.y + v.connect_rd + d.gs; y += d.gs)
			for (int x = via_info[i].xy.x - v.connect_rd; x < via_info[i].xy.x + v.connect_rd + d.gs; x += d.gs) {
				unsigned long long * p_prob = get_prob(d.prob, min(x, via_info[i].xy.x + v.connect_rd),
					min(y, via_info[i].xy.y + v.connect_rd), d.gs);
				if (PROB_SHAPE(p_prob[0]) != BRICK_VIA && PROB_SCORE(p_prob[0]) > 9)
					SET_PROB_S(p_prob[0], 0x0009ffff);
			}
		}
		for (int i = -v.connect_rd; i <= v.connect_rd; i++) {
			int d = sqrt(v.connect_rd * v.connect_rd - i * i);
			circle.border_scan[DIR_UP].push_back(Point(i, -d));
			circle.border_scan[DIR_DOWN].push_back(Point(i, d));
			circle.border_scan[DIR_LEFT].push_back(Point(-d, i));
			circle.border_scan[DIR_RIGHT].push_back(Point(d, i));
		}
	}

	void add_wire(WireExtendInfo & _w, PipeDataPerLayer & d)
	{	
		gray_th = (_w.gray_w * v.cgray_ratio + _w.gray_i * (100 - v.cgray_ratio)) / 100.0;
		w.push_back(_w);
		ewc.push_back(ExtendWireConnect());
		ewc.back().prepare(v, _w, d);
		bd.push_back(Border());
		for (int i = -v.connect_d - _w.w_wide / 2 - 2; i <= v.connect_d + _w.w_wide / 2 + 2; i++) {
			bd.back().border_scan[0].push_back(Point(i, -v.connect_rd));
			bd.back().border_scan[1].push_back(Point(v.connect_rd, i));
			bd.back().border_scan[2].push_back(Point(i, v.connect_rd));
			bd.back().border_scan[3].push_back(Point(-v.connect_rd, i));
		}
	}
	
	void add_brick_hlmask(PipeDataPerLayer & d, Mat & hl_mask)	{
		vector<ViaInfo> & via_info = d.via_info;
		for (int i = 0; i < via_info.size(); i++) {			
			for (int dir = 0; dir <= 3; dir++) {
				unsigned long long min_s = 0xffffffffffffffff;
				int sel = -1;
				for (int j = 0; j < ewc.size(); j++) { //loop all wire
					unsigned long long score = ewc[j].compute_prob(via_info[i], dir, bd[j]);
					if (score != 0xffffffffffffffff)
					switch (dir) { //check if distance < connect_d
					case DIR_UP:
					case DIR_DOWN:
						if (abs(PROB_X(score) - via_info[i].xy.x) > v.connect_d + w[j].w_wide / 2)
							score = 0xffffffffffffffff;
						break;
					case DIR_LEFT:
					case DIR_RIGHT:
						if (abs(PROB_Y(score) - via_info[i].xy.y) > v.connect_d + w[j].w_wide / 2)
							score = 0xffffffffffffffff;
						break;
					}
					if (min_s > score) {
						sel = j;
						SET_PROB_TYPE(score, w[j].w_type);
						min_s = score;
					}
				}
				if (sel >= 0 && v.cgray_d > 0) { //check arc connect
					vector <Point> circle_bd; //stort arc border
					switch (dir) { //compute arc border
						case DIR_UP:
						case DIR_DOWN:
						{
							int x0 = max(via_info[i].xy.x - v.connect_rd, PROB_X(min_s) - w[sel].w_wide / 2);
							int x1 = min(via_info[i].xy.x + v.connect_rd, PROB_X(min_s) + (w[sel].w_wide - 1) / 2);
							for (int k = 0; k < (int)circle.border_scan[dir].size(); k++) {
								Point bd = via_info[i].xy + circle.border_scan[dir][k];
								if (bd.x >= x0 && bd.x <= x1)
									circle_bd.push_back(bd);
							}
						}
						case DIR_LEFT:
						case DIR_RIGHT:
						{
							int y0 = max(via_info[i].xy.y - v.connect_rd, PROB_Y(min_s) - w[sel].w_wide / 2);
							int y1 = min(via_info[i].xy.y + v.connect_rd, PROB_Y(min_s) + (w[sel].w_wide - 1) / 2);
							for (int k = 0; k < (int)circle.border_scan[dir].size(); k++) {
								Point bd = via_info[i].xy + circle.border_scan[dir][k];
								if (bd.y >= y0 && bd.y <= y1)
									circle_bd.push_back(bd);
							}
						}
					}
					//following check arc connect
					if (ExtendWireConnect::compute_connect_gray(img, circle_bd, dir, v.cgray_d) < gray_th) 
						sel = -1;
				}
				if (sel >= 0) {
					ewc[sel].add_path(d, via_info[i], min_s);
					if (!hl_mask.empty())
						ewc[sel].mark_extend(hl_mask, via_info[i], w[sel], min_s, dir);
				}
			}			
		}		
	}
};

class PairViaWireConnect : public ViaWireConnect {
protected:
	vector<ExtendWireConnect> ewc;
	vector<Border> bd[5]; //bd0 & bd2 is for shu pair, bd1 & bd3 is for heng pair, bd4 is for single via
	ViaParameter v;
	vector<WireExtendInfo> w;
	float gray_th;
	Mat img;
	Border circle;

public:
	void prepare(ViaParameter & _v, PipeDataPerLayer & d) {
		v = _v;
		vector<ViaInfo> & via_info = d.via_info;
		img = d.img;
		for (int i = 0; i < via_info.size(); i++) {
			//if prob is within via remove range, mark it BRICK_INVALID
			for (int y = via_info[i].xy.y - v.connect_rd; y <= via_info[i].xy.y + v.connect_rd; y += 2)
			for (int x = via_info[i].xy.x - v.connect_rd; x <= via_info[i].xy.x + v.connect_rd; x += 2) {
				unsigned long long * p_prob = get_prob(d.prob, min(x, via_info[i].xy.x + v.connect_rd),
					min(y, via_info[i].xy.y + v.connect_rd), d.gs);
				if (PROB_SHAPE(p_prob[0]) != BRICK_VIA && PROB_SCORE(p_prob[0]) > 9)
					SET_PROB_S(p_prob[0], 0x0009ffff);
			}
			
			unsigned score = 1;
			unsigned type = v.type;			
			if (via_info[i].via_adj_mask & (1 << DIR_RIGHT)) { //put BRICK_I_90 to connect two via 
				unsigned shape = BRICK_I_90;
				unsigned s = MAKE_S(score, type, shape);
				push_new_prob(d.prob, via_info[i].xy.x + d.gs, via_info[i].xy.y, s, d.gs);
				int j = via_info[i].v_pair[DIR_RIGHT];
				CV_Assert(via_info[j].xy.x - via_info[i].xy.x >= d.gs * 2);
				push_new_prob(d.prob, via_info[j].xy.x - d.gs, via_info[j].xy.y, s, d.gs);
			}
			if (via_info[i].via_adj_mask & (1 <<DIR_DOWN)) { //put BRICK_I_90 to connect two via 
				unsigned shape = BRICK_I_0;
				unsigned s = MAKE_S(score, type, shape);
				push_new_prob(d.prob, via_info[i].xy.x, via_info[i].xy.y + d.gs, s, d.gs);
				int j = via_info[i].v_pair[DIR_DOWN];
				CV_Assert(via_info[j].xy.y - via_info[i].xy.y >= d.gs * 2);
				push_new_prob(d.prob, via_info[j].xy.x, via_info[j].xy.y - d.gs, s, d.gs);
			}
		}

		for (int i = -v.connect_rd; i <= v.connect_rd; i++) {
			int d = sqrt(v.connect_rd * v.connect_rd - i * i);
			circle.border_scan[DIR_UP].push_back(Point(i, -d));
			circle.border_scan[DIR_DOWN].push_back(Point(i, d));
			circle.border_scan[DIR_LEFT].push_back(Point(-d, i));
			circle.border_scan[DIR_RIGHT].push_back(Point(d, i));
		}
	}

	void add_wire(WireExtendInfo & _w, PipeDataPerLayer & d)
	{
		gray_th = (_w.gray_w * v.cgray_ratio + _w.gray_i * (100 - v.cgray_ratio)) / 100.0;
		w.push_back(_w);
		ewc.push_back(ExtendWireConnect());
		ewc.back().prepare(v, _w, d);

		bd[DIR_UP].push_back(Border()); //bd[DIR_UP] contain up, left, right border
		bd[DIR_DOWN].push_back(Border()); //bd[DIR_DOWN] only contain down border
		for (int i = -v.connect_d - _w.w_wide / 2 - 2; i <= v.connect_d + _w.w_wide / 2 + 2; i++) {
			bd[DIR_UP].back().border_scan[DIR_UP].push_back(Point(i, -v.connect_rd));
			bd[DIR_DOWN].back().border_scan[DIR_DOWN].push_back(Point(i, v.connect_rd));
		}
		for (int i = -v.connect_d - _w.w_wide / 2 - 2; i <= v.pair_distance + v.connect_d + _w.w_wide / 2 + 2; i++) {
			bd[DIR_UP].back().border_scan[DIR_RIGHT].push_back(Point(v.connect_rd, i));
			bd[DIR_UP].back().border_scan[DIR_LEFT].push_back(Point(-v.connect_rd, i));
		}
		
		bd[DIR_LEFT].push_back(Border()); //bd[DIR_LEFT] contain up, left, down border
		bd[DIR_RIGHT].push_back(Border()); //bd[DIR_RIGHT] only contain right border
		for (int i = -v.connect_d - _w.w_wide / 2 - 2; i <= v.connect_d + _w.w_wide / 2 + 2; i++) {
			bd[DIR_LEFT].back().border_scan[DIR_LEFT].push_back(Point(-v.connect_rd, i));
			bd[DIR_RIGHT].back().border_scan[DIR_RIGHT].push_back(Point(v.connect_rd, i));
		}
		for (int i = -v.connect_d - _w.w_wide / 2 - 2; i <= v.pair_distance + v.connect_d + _w.w_wide / 2 + 2; i++) {
			bd[DIR_LEFT].back().border_scan[DIR_UP].push_back(Point(i, -v.connect_rd));
			bd[DIR_LEFT].back().border_scan[DIR_DOWN].push_back(Point(i, v.connect_rd));
		}

		bd[4].push_back(Border());
		for (int i = -v.connect_d - _w.w_wide / 2 - 2; i <= v.connect_d + _w.w_wide / 2 + 2; i++) {
			bd[4].back().border_scan[DIR_UP].push_back(Point(i, -v.connect_rd));
			bd[4].back().border_scan[DIR_RIGHT].push_back(Point(v.connect_rd, i));
			bd[4].back().border_scan[DIR_DOWN].push_back(Point(i, v.connect_rd));
			bd[4].back().border_scan[DIR_LEFT].push_back(Point(-v.connect_rd, i));
		}
	}

	void add_brick_hlmask(PipeDataPerLayer & d, Mat & hl_mask)	{
		vector<ViaInfo> & via_info = d.via_info;
		for (int i = 0; i < via_info.size(); i++) {
			for (int dir = 0; dir <= 3; dir++) {
				unsigned long long min_s = 0xffffffffffffffff;
				int sel = -1;
				if ((via_info[i].via_adj_mask & (1 << DIR_RIGHT | 1 << DIR_DOWN)) == 0 && via_info[i].via_adj_mask != 0)
					continue; //neglect right and down pair_via, because it is handled by left and up pair_via
				for (int j = 0; j < ewc.size(); j++) {
					unsigned long long score;
					if (via_info[i].via_adj_mask == 0) { //single via
						score = ewc[j].compute_prob(via_info[i], dir, bd[4][j]);
						if (score != 0xffffffffffffffff)
						switch (dir) { //check if distance < connect_d
						case DIR_UP:
						case DIR_DOWN:
							if (abs(PROB_X(score) - via_info[i].xy.x) > v.connect_d + w[j].w_wide / 2)
								score = 0xffffffffffffffff;
							break;
						case DIR_LEFT:
						case DIR_RIGHT:
							if (abs(PROB_Y(score) - via_info[i].xy.y) > v.connect_d + w[j].w_wide / 2)
								score = 0xffffffffffffffff;
							break;
						}
					}
					if (via_info[i].via_adj_mask & 1 << DIR_DOWN) {						
						int k = via_info[i].v_pair[DIR_DOWN];
						CV_Assert(k >= 0);
						if (dir == DIR_DOWN)
							score = ewc[j].compute_prob(via_info[k], dir, bd[DIR_DOWN][j]);						
						else 
							score = ewc[j].compute_prob(via_info[i], dir, bd[DIR_UP][j]);	
						if (score != 0xffffffffffffffff)
						switch (dir) { //check if distance < connect_d
						case DIR_UP:
						case DIR_DOWN:
							if (abs(PROB_X(score) - via_info[i].xy.x) > v.connect_d + w[j].w_wide / 2)
								score = 0xffffffffffffffff;
							break;
						case DIR_LEFT:
						case DIR_RIGHT:
							if (via_info[i].xy.y > PROB_Y(score) + v.connect_d + w[j].w_wide / 2 || 
								PROB_Y(score)  > via_info[k].xy.y + v.connect_d + w[j].w_wide / 2)
								score = 0xffffffffffffffff;
							break;
						}
					}	
					if (min_s > score) {
						sel = j;
						SET_PROB_TYPE(score, w[j].w_type);
						min_s = score;
					}
					if (via_info[i].via_adj_mask & 1 << DIR_RIGHT) {
						int k = via_info[i].v_pair[DIR_RIGHT];
						CV_Assert(k >= 0);
						if (dir == DIR_RIGHT)
							score = ewc[j].compute_prob(via_info[k], dir, bd[DIR_RIGHT][j]);
						else
							score = ewc[j].compute_prob(via_info[i], dir, bd[DIR_LEFT][j]);
						if (score != 0xffffffffffffffff)
						switch (dir) { //check if distance < connect_d
						case DIR_UP:
						case DIR_DOWN:
							if (via_info[i].xy.x > PROB_X(score) + v.connect_d + w[j].w_wide / 2 ||
								PROB_X(score)  > via_info[k].xy.x + v.connect_d + w[j].w_wide / 2)
								score = 0xffffffffffffffff;
							break;
						case DIR_LEFT:
						case DIR_RIGHT:
							if (abs(PROB_Y(score) - via_info[i].xy.y) > v.connect_d + w[j].w_wide / 2)
								score = 0xffffffffffffffff;
							break;
						}
					}
					if (min_s > score) {
						sel = j;
						SET_PROB_TYPE(score, w[j].w_type);
						min_s = score;
					}
				}
				if (sel >= 0 && v.cgray_d > 0) { //check arc connect
					vector <Point> circle_bd; //stort arc border
					if (via_info[i].via_adj_mask == 0) { //compute single via arc, same as SingleViaWireConnect
						switch (dir) { //compute arc border
							case DIR_UP:
							case DIR_DOWN:
							{
								int x0 = max(via_info[i].xy.x - v.connect_rd, PROB_X(min_s) - w[sel].w_wide / 2);
								int x1 = min(via_info[i].xy.x + v.connect_rd, PROB_X(min_s) + (w[sel].w_wide - 1) / 2);
								for (int k = 0; k < (int)circle.border_scan[dir].size(); k++) {
									Point bd = via_info[i].xy + circle.border_scan[dir][k];
									if (bd.x >= x0 && bd.x <= x1)
										circle_bd.push_back(bd);
								}
							}
							case DIR_LEFT:
							case DIR_RIGHT:
							{
								int y0 = max(via_info[i].xy.y - v.connect_rd, PROB_Y(min_s) - w[sel].w_wide / 2);
								int y1 = min(via_info[i].xy.y + v.connect_rd, PROB_Y(min_s) + (w[sel].w_wide - 1) / 2);
								for (int k = 0; k < (int)circle.border_scan[dir].size(); k++) {
									Point bd = via_info[i].xy + circle.border_scan[dir][k];
									if (bd.y >= y0 && bd.y <= y1)
										circle_bd.push_back(bd);
								}
							}
							}
					}
					if (via_info[i].via_adj_mask & 1 << DIR_DOWN) { //compute up-down via arc
						int j = via_info[i].v_pair[DIR_DOWN];
						CV_Assert(via_info[j].xy.y > via_info[i].xy.y);
						switch (dir) {
						case DIR_UP:
						case DIR_DOWN:
							{
								int xx = (dir == DIR_UP) ? via_info[i].xy.x : via_info[j].xy.x;
								int x0 = max(xx - v.connect_rd, PROB_X(min_s) - w[sel].w_wide / 2);
								int x1 = min(xx + v.connect_rd, PROB_X(min_s) + (w[sel].w_wide - 1) / 2);
								for (int k = 0; k < (int)circle.border_scan[dir].size(); k++) {
									Point bd = (dir == DIR_UP) ? via_info[i].xy + circle.border_scan[DIR_UP][k] :
										via_info[j].xy + circle.border_scan[DIR_DOWN][k];
									if (bd.x >= x0 && bd.x <= x1)
										circle_bd.push_back(bd);
								}
							}
						case DIR_LEFT:
						case DIR_RIGHT:
							{
								for (int k = via_info[i].xy.y - v.connect_rd; k <= via_info[j].xy.y + v.connect_rd; k++) {
									int dy;
									Point bd;
									if (k <= (via_info[i].xy.y + via_info[j].xy.y) / 2) {
										dy = k + v.connect_rd - via_info[i].xy.y;
										bd = Point(via_info[i].xy.x, k);
									}
									else {
										dy = via_info[j].xy.y + v.connect_rd - k;
										bd = Point(via_info[j].xy.x, k);
									}
									bd.x += (dy >= circle.border_scan[dir].size()) ? 0 : circle.border_scan[dir][dy].x;
									circle_bd.push_back(bd);
								}
							}
						}
					}
					if (via_info[i].via_adj_mask & 1 << DIR_RIGHT) { //compute left-right via arc
						int j = via_info[i].v_pair[DIR_RIGHT];
						CV_Assert(via_info[j].xy.x > via_info[i].xy.x);
						switch (dir) {
						case DIR_UP:
						case DIR_DOWN:
							{
								for (int k = via_info[i].xy.x - v.connect_rd; k <= via_info[j].xy.x + v.connect_rd; k++) {
									int dx; 
									Point bd; 
									if (k <= (via_info[i].xy.x + via_info[j].xy.x) / 2) {
										dx = k + v.connect_rd - via_info[i].xy.x;
										bd = Point(k, via_info[i].xy.y);
									}
									else {
										dx = via_info[j].xy.x + v.connect_rd - k;
										bd = Point(k, via_info[j].xy.y);
									}
									bd.y += (dx >= circle.border_scan[dir].size()) ? 0 : circle.border_scan[dir][dx].y;
									circle_bd.push_back(bd);
								}
							}
						}
					}
					//following check arc connect
					if (ExtendWireConnect::compute_connect_gray(img, circle_bd, dir, v.cgray_d) < gray_th)
						sel = -1;
				}
				if (sel >= 0) {
					ewc[sel].add_path(d, via_info[i], min_s);
					if (!hl_mask.empty())
						ewc[sel].mark_extend(hl_mask, via_info[i], w[sel], min_s, dir);
				}
			}
		}
	}
};

ViaWireConnect* ViaWireConnect::create_via_wire_connect(ViaParameter & v, PipeDataPerLayer & d)
{
	ViaWireConnect * ret;
	if (v.pair_distance == 0) {
		ret = new SingleViaWireConnect();
		ret->prepare(v, d);
	}
	else {
		ret = new PairViaWireConnect();
		ret->prepare(v, d);
	}
	return ret;
}

/*
		31..24  23..16   15..8  7..0
opt0:	check_opt v_subtype v_type v_pattern
opt1:	w_type  i_wide  w_wide w_len_near	
opt2:					extend w_len_far
opt3:			
opt4:			
check_opt is
CHECK_VW_HOTLINE_CLEAR_MASK			clear hotline mask to 0
CHECK_VW_SEARCH_CLEAR_COLOR			clip color to [gi, gm]
0: for gray level turn_points input
1: for search mask inout
*/
static void check_via_wire_connect(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & img = d.l[layer].img;
	CV_Assert(img.type() == CV_8UC1);
	//get index for FINE_LINE_MASK output
	int idx = cpara.method_opt & 0xf;
	if (d.l[layer].v[idx].type != TYPE_GRAY_LEVEL) {
		qCritical("check_via_wire_connect gray level idx[%d]=%d, error", idx, d.l[layer].v[idx].type);
		return;
	}
	int idx1 = cpara.method_opt >> 4 & 0xf;
	d.l[layer].v[idx1].type = TYPE_FINE_WIRE_MASK;
	Mat & mask = d.l[layer].v[idx1].d;

	int check_opt = cpara.opt0 >> 24 & 0xff;
	if (check_opt & CHECK_VW_HOTLINE_CLEAR_MASK) {
		mask.create(img.rows, img.cols, CV_32S);
		mask = Scalar::all(0);
	}
	if (mask.rows != img.rows || mask.cols != img.cols) {
		qCritical("check_via_wire_connect, mask.size(%d,%d)!=img.size(%d,%d)", mask.rows, mask.cols, img.rows, img.cols);
		return;
	}
	//get color for wire and insu
	int gi, gm;
	gi = find_index(d.l[layer].v[idx].d, (int)GRAY_L0);
	gi = d.l[layer].v[idx].d.at<int>(gi, 2);
	gm = find_index(d.l[layer].v[idx].d, (int)GRAY_M0);
	gm = d.l[layer].v[idx].d.at<int>(gm, 2);

	if (check_opt & CHECK_VW_SEARCH_CLEAR_COLOR) {
		qInfo("check_via_wire_connect, clear color, gi=%d, gm=%d", gi, gm);
		for (int y = 0; y < img.rows; y++) {
			unsigned char * p_img = img.ptr<unsigned char>(y);
			for (int x = 0; x < img.cols; x++)
				p_img[x] = (p_img[x] > gm) ? gm : ((p_img[x] < gi) ? gi : p_img[x]);
		}
		d.l[layer].ig_valid = false;
	}

	int v_pattern = cpara.opt0 & 0xff;
	int v_type = cpara.opt0 >> 8 & 0xff;
	int v_subtype = cpara.opt0 >> 16 & 0xff;
	qInfo("check_via_wire_connect v_pattern=%d, v_type=%d, v_subtype=%d, gi=%d, gm=%d", v_pattern, v_type, v_subtype, gi, gm);	
	struct WireExtendInfo wei[] = {
		{ cpara.opt1 >> 24 & 0xff, cpara.opt1 >> 16 & 0xff, cpara.opt1 >> 8 & 0xff, cpara.opt1 & 0xff, cpara.opt2 & 0xff,
		cpara.opt2 >> 8 & 0xff, gi, gm },
		{ cpara.opt3 >> 24 & 0xff, cpara.opt3 >> 16 & 0xff, cpara.opt3 >> 8 & 0xff, cpara.opt3 & 0xff, cpara.opt4 & 0xff, 
		cpara.opt4 >> 8 & 0xff, gi, gm },
		{ cpara.opt5 >> 24 & 0xff, cpara.opt5 >> 16 & 0xff, cpara.opt5 >> 8 & 0xff, cpara.opt5 & 0xff, cpara.opt6 & 0xff,
		cpara.opt6 >> 8 & 0xff, gi, gm }
	};
	//get via info
	VWParameter * vwp = d.l[layer].vw.get_vw_para(v_pattern, v_type, v_subtype);
	if (vwp == NULL) {
		qCritical("check_via_wire_connect invalid v_pattern=%d, v_type=%d, v_subtype=%d", v_pattern, v_type, v_subtype);
		return;
	}
	ViaParameter vp = vwp->v;
#define MAX_WIRE_EXTEND_NUM 3	
	QScopedPointer<ViaWireConnect> vwc(ViaWireConnect::create_via_wire_connect(vp, d.l[layer]));
	for (int i = 0; i < MAX_WIRE_EXTEND_NUM; i++) {
		if (wei[i].w_wide == 0)
			continue;
		qInfo("check_via_wire_connect w%d, type=%d, i_wide=%d, w_wide=%d, w_len_n=%d, w_len_f=%d, extend=%d",
			i, wei[i].w_type, wei[i].i_wide, wei[i].w_wide, wei[i].w_len_near, wei[i].w_len_far, wei[i].extend);
		vwc->add_wire(wei[i], d.l[layer]);
	}

	vwc->add_brick_hlmask(d.l[layer], mask);	
	//debug draw
	if (cpara.method & OPT_DEBUG_EN) {
		d.l[layer].check_prob();
		Mat debug_draw;
		debug_draw = img.clone();
		Mat & prob = d.l[layer].prob;
		for (int y = 0; y < prob.rows; y++) {
			unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
			for (int x = 0; x < prob.cols; x++) {
				int x0 = PROB_X(p_prob[2 * x]), y0 = PROB_Y(p_prob[2 * x]);
				int shape = PROB_SHAPE(p_prob[2 * x]);
				if (shape < sizeof(bricks) / sizeof(bricks[0])) {
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
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "check_via_wire_connect.jpg", debug_draw);
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
	 FINE_LINE_SEARCH_CLEAR_COLOR   clip color to [gi, gm]
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
	Mat via_mask(mask.rows, mask.cols, CV_8UC1);

	int search_opt = cpara.opt0 >> 16 & 0xff;
	if ((search_opt & FINE_LINE_SEARCH_NO_VIA) == 0) {
		int idx2 = cpara.method_opt >> 8 & 0xf;
		if (d.l[layer].v[idx2].type != TYPE_REMOVE_VIA_MASK) {
			qCritical("fine_line_search mask idx2[%d]=%d, error", idx2, d.l[layer].v[idx2].type);
			return;
		}
		via_mask = d.l[layer].v[idx2].d;
	}
	else
		via_mask = Scalar::all(0);

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
	gi = find_index(d.l[layer].v[idx].d, (int) GRAY_L0);
	gi = d.l[layer].v[idx].d.at<int>(gi, 2);
	gm = find_index(d.l[layer].v[idx].d, (int) GRAY_M0);
	gm = d.l[layer].v[idx].d.at<int>(gm, 2);
	int wnum = cpara.opt0 & 0xff;
	
	qInfo("fine_line_search, l=%d, wnum=%d, gi=%d, gm=%d, cr_prob=%d", layer, wnum, gi, gm, search_opt);

	if (search_opt & FINE_LINE_SEARCH_CLEAR_PROB) { //clear prob if needed		
		if ((search_opt & FINE_LINE_SEARCH_NO_VIA) == 0) {
			vector<ViaInfo> & via_info = d.l[layer].via_info;
			for (int i = 0; i < (int) via_info.size(); i++) {
				int type = via_info[i].type;
				int x = via_info[i].xy.x;
				int y = via_info[i].xy.y;
				int y0 = y / d.l[layer].gs, x0 = x / d.l[layer].gs;
				unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y0, x0);
				if (PROB_SHAPE(p_prob[0]) != BRICK_VIA) { //mark BRICK_VIA with via info, via prob may be cleared by remove_via & coarse_line_search
					p_prob[1] = p_prob[0];
					p_prob[0] = MAKE_PROB(MAKE_S(1, type, BRICK_VIA), x, y);
				}
			}
		}
		for (int y = 0; y < d.l[layer].prob.rows; y++) {
			unsigned long long * p_prob = d.l[layer].prob.ptr<unsigned long long>(y);
			for (int x = 0; x < d.l[layer].prob.cols * 2; x+=2)
				if (PROB_SHAPE(p_prob[x]) != BRICK_VIA) { //if it is not VIA, Mark as NO_WIRE
					p_prob[x] = 0xffffff00ffffffffULL; 
					SET_PROB_X(p_prob[x], x / 2 * d.l[layer].gs + 1);
					SET_PROB_Y(p_prob[x], y*d.l[layer].gs + 1);
					p_prob[x + 1] = 0xffffff00ffffffffULL; 
					SET_PROB_X(p_prob[x + 1], x / 2 * d.l[layer].gs + 1);
					SET_PROB_Y(p_prob[x + 1], y*d.l[layer].gs + 1);
				}
				else {
					int y0 = PROB_Y(p_prob[x]);
					int x0 = PROB_X(p_prob[x]);
					if (!(via_mask.at<unsigned char>(y0, x0) & 1)) { //if it is not via, Mark as NO_WIRE
						p_prob[x] = 0xffffff00ffffffffULL;
						SET_PROB_X(p_prob[x], x / 2 * d.l[layer].gs + 1);
						SET_PROB_Y(p_prob[x], y*d.l[layer].gs + 1);
						p_prob[x + 1] = 0xffffff00ffffffffULL;
						SET_PROB_X(p_prob[x + 1], x / 2 * d.l[layer].gs + 1);
						SET_PROB_Y(p_prob[x + 1], y*d.l[layer].gs + 1);
					}
				}
		}
	}


	if (search_opt & FINE_LINE_SEARCH_CLEAR_COLOR) {
		qInfo("fine_line_search, clear color, gi=%d, gm=%d", gi, gm);
		for (int y = 0; y < img.rows; y++) {
			unsigned char * p_img = img.ptr<unsigned char>(y);
			for (int x = 0; x < img.cols; x++)
				p_img[x] = (p_img[x] > gm) ? gm : ((p_img[x] < gi) ? gi : p_img[x]);
		}
		d.l[layer].ig_valid = false;
	}
	d.l[layer].validate_ig();

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
		wire_para.gray_i = gi;
		wire_para.gray_w = gm;
		wcs[i].reset(WireComputeScore::create_wire_compute_score(wire_para, d, layer));
		w_guard[i] = wire_para.guard;
	}
	
	Mat new_prob0(d.l[layer].prob.rows, d.l[layer].prob.cols, CV_8UC1);
	int mark_num = 100;
	int loop_num = 0;
	Mat already_search(mask.rows, mask.cols, CV_32SC1);
	already_search = Scalar::all(0);
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
						&& PROB_TYPE(score_min[0]) == PROB_TYPE(score4fake_via)) {//BRICK_II can only take effect for same type
							push_new_prob(prob1, score4fake_via, d.l[layer].gs);
					}
					if (PROB_SHAPE(score_min[0]) == BRICK_FAKE_VIA && PROB_SHAPE(score_min[1]) != BRICK_FAKE_VIA
						&& PROB_TYPE(score_min[0]) != PROB_TYPE(score_min[1])) {
						swap(score_min[0], score_min[1]);
						SET_PROB_SCORE(score_min[1], PROB_SCORE(score_min[0]) + 1);
					}
				} else 
				if (p_via_mask[x] & 1) {
					int y0 = y / d.l[layer].gs, x0 = x / d.l[layer].gs;
					unsigned long long * p_prob = prob1.ptr<unsigned long long>(y0, x0);
					if (PROB_SHAPE(p_prob[0]) != BRICK_VIA) { //if it is within via mask range, mark it BRICK_INVALID
						p_prob[0] = MAKE_PROB(0x0009ffff, x, y);
						p_prob[1] = p_prob[0];
					}
				}
		}

		//2 update d.l.prob by filtering prob1
		for (int y = 0; y < prob1.rows; y++) {
			unsigned long long * p_prob1 = prob1.ptr<unsigned long long>(y);
			unsigned long long * p_prob0 = d.l[layer].prob.ptr<unsigned long long>(y);
			unsigned char * p_new_prob0 = new_prob0.ptr<unsigned char>(y);
			for (int x = 0; x < prob1.cols * 2; x+=2) {
				if (PROB_SHAPE(p_prob1[x]) != BRICK_VIA && PROB_S(p_prob1[x]) != 0xffffff00 &&
					PROB_SHAPE(p_prob1[x]) != BRICK_HOLLOW && PROB_SHAPE(p_prob1[x]) != BRICK_ONE_POINT &&
					PROB_SHAPE(p_prob1[x]) != BRICK_INVALID) { //following filter unique
					CV_Assert(PROB_SHAPE(p_prob1[x]) <= BRICK_IN_USE || PROB_SHAPE(p_prob1[x]) == BRICK_FAKE_VIA);
					bool pass = true;					
					int idx = PROB_TYPE(p_prob1[x]);
					int cr = w_guard[idx];
					int guard = (cr - 1) / d.l[layer].gs + 1;
					int y1 = max(y - guard, 0), y2 = min(y + guard, prob1.rows - 1);
					int x1 = max(x / 2 - guard, 0), x2 = min(x / 2 + guard, prob1.cols - 1);
					//2.1 check unique for brick i, T, L; NO_WIRE; 
					if (PROB_SHAPE(p_prob1[x]) != BRICK_I_0 && PROB_SHAPE(p_prob1[x]) != BRICK_I_90 && PROB_SHAPE(p_prob1[x]) != BRICK_FAKE_VIA) {
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW) { //Unique condition is it is minimal than nearyby brick
								if (PROB_SHAPE(p_prob2[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								if (abs(PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x])) <= cr &&
									abs(PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x])) <= cr) {
									pass = false;
									yy = y2;
									break;
								}
							}
						}
					}
					//2.2 check unique for BRICK_I_0
					if (PROB_SHAPE(p_prob1[x]) == BRICK_I_0) {
						for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
						if (p_prob1[xx] < p_prob1[x] && PROB_SHAPE(p_prob1[xx]) != BRICK_HOLLOW) { //Unique condition is it is minimal than row brick
							if (PROB_SHAPE(p_prob1[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob1[xx]) != PROB_TYPE(p_prob1[x]))
								continue; //BRICK_II can only take effect for same type
							if (abs(PROB_X(p_prob1[xx]) - PROB_X(p_prob1[x])) <= cr &&
								abs(PROB_Y(p_prob1[xx]) - PROB_Y(p_prob1[x])) <= cr) {
								pass = false;
								break;
							}
						}
						if (pass)
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							if (yy == y)
								continue;
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW 
								&& PROB_SHAPE(p_prob2[xx]) != BRICK_I_0) { //Unique condition is it is minimal than nearyby non BRICK_I_O brick
								if (PROB_SHAPE(p_prob2[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								if (abs(PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x])) <= cr &&
									abs(PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x])) <= cr) {
									pass = false;
									yy = y2;
									break;
								}
							}
						}
					}
					//2.3 check unique for BRICK_I_0
					if (PROB_SHAPE(p_prob1[x]) == BRICK_I_90) {
						for (int yy = y1; yy <= y2; yy++) { 
							unsigned long long prob2 = prob1.at<unsigned long long>(yy, x);
							if (prob2 < p_prob1[x] && PROB_SHAPE(prob2) != BRICK_HOLLOW) { //Unique condition is it is minimal than col brick
								if (PROB_SHAPE(prob2) == BRICK_FAKE_VIA && PROB_TYPE(prob2) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								if (abs(PROB_X(prob2) - PROB_X(p_prob1[x])) <= cr &&
									abs(PROB_Y(prob2) - PROB_Y(p_prob1[x])) <= cr) {
									pass = false;
									break;
								}
							}
						}
						if (pass)
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW
								&& PROB_SHAPE(p_prob2[xx]) != BRICK_I_90) { //Unique condition is it is minimal than nearyby non BRICK_I_9O brick
								if (PROB_SHAPE(p_prob2[xx]) == BRICK_FAKE_VIA && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x]))
									continue; //BRICK_II can only take effect for same type
								if (abs(PROB_X(p_prob2[xx]) - PROB_X(p_prob1[x])) <= cr &&
									abs(PROB_Y(p_prob2[xx]) - PROB_Y(p_prob1[x])) <= cr) {
									pass = false;
									yy = y2;
									break;
								}
							}
						}
					}
					//2.4 check unique for brick ii; FAKE_VIA;
					if (PROB_SHAPE(p_prob1[x]) == BRICK_FAKE_VIA) {
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = prob1.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2) {
								if (p_prob2[xx] < p_prob1[x] && PROB_SHAPE(p_prob2[xx]) != BRICK_HOLLOW ||
									PROB_SHAPE(p_prob2[xx]) <= BRICK_IN_USE && PROB_TYPE(p_prob2[xx]) != PROB_TYPE(p_prob1[x])) { //Unique condition is it is minimal than nearyby brick
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

					if (pass) {//if unique, copy to d.l[layer].prob,mark nearby NOWIRE brick BRICK_INVALID
						unsigned long long prob0 = p_prob1[x];
						p_new_prob0[x / 2] = (PROB_SHAPE(p_prob0[x]) != PROB_SHAPE(prob0));
						p_prob0[x] = prob0;	
						for (int yy = y1; yy <= y2; yy++) {
							unsigned long long * p_prob2 = d.l[layer].prob.ptr<unsigned long long>(yy);
							for (int xx = 2 * x1; xx <= 2 * x2; xx += 2)
							if (PROB_S(p_prob2[xx]) == 0xffffff00) 
								SET_PROB_SHAPE(p_prob2[xx], BRICK_INVALID);							
						}
					}
					else { //if it is not unique, mark self as BRICK_INVALID
						p_prob0[x] = p_prob1[x];
						SET_PROB_SHAPE(p_prob0[x], BRICK_INVALID);
					}
				}
				if (PROB_SHAPE(p_prob1[x]) == BRICK_HOLLOW || PROB_SHAPE(p_prob1[x]) == BRICK_ONE_POINT || 
					PROB_SHAPE(p_prob1[x]) == BRICK_INVALID) { //if it is Hollow, OnePoint, mark as Invalid
					p_prob0[x] = p_prob1[x];
					SET_PROB_SHAPE(p_prob0[x], BRICK_INVALID);
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
		for (int x = 0; x < d.l[layer].prob.cols * 2; x+=2)
		if (PROB_S(p_prob[x]) != 0xffffff00)
			if (PROB_SHAPE(p_prob[x]) <= BRICK_IN_USE || PROB_SHAPE(p_prob[x]) == BRICK_FAKE_VIA) {
				unsigned long long prob0 = p_prob[x];
				CV_Assert(PROB_TYPE(prob0) < MAX_WIRE_NUM);
				SET_PROB_TYPE(prob0, w_type[PROB_TYPE(prob0)]);
				p_prob[x] = prob0;
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
					if (shape < sizeof(bricks) / sizeof(bricks[0])) {
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

/*
		31..24 23..16   15..8   7..0
opt0:					assemble_opt wnum
opt1:	clong_heng clong_shu cwide	type
opt2:	clong_heng clong_shu cwide	type		
opt3:	clong_heng clong_shu cwide	type		
opt4:			
opt5:			
opt6:			
cwide, clong_heng clong_shu is for connectivity. Normally cwide <=3
assemble_opt
*/
static void assemble_line(PipeData & d, ProcessParameter & cpara)
{
	int layer = cpara.layer;
	Mat & prob = d.l[layer].prob;	
	
	int wnum = cpara.opt0 & 0xff;
	int assemble_opt = cpara.opt0 >> 8 & 0xff;
	qInfo("assemble_line l=%d, flag=%d, wnum=%d", layer, assemble_opt, wnum);
	Mat mark(prob.rows, prob.cols, CV_8UC1);	
	vector<ViaInfo> & via_info = d.l[layer].via_info;
	
#define MAX_WIRE_NUM 6
	int wtype[MAX_WIRE_NUM] = { cpara.opt1 & 0xff, cpara.opt2 & 0xff, cpara.opt3 & 0xff, cpara.opt4 & 0xff, cpara.opt5 & 0xff };
	int cwide[MAX_WIRE_NUM] = { cpara.opt1 >> 8 & 0xff, cpara.opt2 >> 8 & 0xff, 
		cpara.opt3 >> 8 & 0xff, cpara.opt4 >> 8 & 0xff, cpara.opt5 >> 8 & 0xff };
	int clong0[MAX_WIRE_NUM] = { cpara.opt1 >> 16 & 0xff, cpara.opt2 >> 16 & 0xff,
		cpara.opt3 >> 16 & 0xff, cpara.opt4 >> 16 & 0xff, cpara.opt5 >> 16 & 0xff };
	int clong1[MAX_WIRE_NUM] = { cpara.opt1 >> 24 & 0xff, cpara.opt2 >> 24 & 0xff,
		cpara.opt3 >> 24 & 0xff, cpara.opt4 >> 24 & 0xff, cpara.opt5 >> 24 & 0xff };

	struct TraceState {
		unsigned long long prev_brick;
		int x, y;
		bool found;
	} trace;
	WireLine cur_line;

	for (int i = 0; i < wnum; i++) {
		qInfo("assemble_line, type=%d, cwide=%d, clong0=%d, clong1=%d", wtype[i], cwide[i], clong0[i], clong1[i]);
		if (cwide[i] > d.l[layer].gs) {
			qCritical("invalid cwide");
			return;
		}
	}

	int fix_count = 0, noend_count = 0;
	int x1 = d.l[layer].border_size / d.l[layer].gs - 1;
	int x2 = prob.cols - d.l[layer].border_size / d.l[layer].gs;
	int y1 = d.l[layer].border_size / d.l[layer].gs - 1;
	int y2 = prob.rows - d.l[layer].border_size / d.l[layer].gs;
	
	mark = Scalar::all(0);
	for (int y = y1; y <= y2; y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		unsigned long long * p_prob = prob.ptr<unsigned long long>(y);
		for (int x = x1; x <= x2; x++) 
			if (p_mark[x]==0) { 
				unsigned long long prob0 = p_prob[2 * x];
				int brick_fix;
				if (PROB_SHAPE(prob0) != BRICK_NO_WIRE && PROB_SHAPE(prob0)!=BRICK_INVALID) { //scan to find one begining brick
					CV_Assert(PROB_SHAPE(prob0) <= BRICK_IN_USE || PROB_SHAPE(prob0) == BRICK_FAKE_VIA || PROB_SHAPE(prob0) == BRICK_VIA);
					if (brick_conn.quick_fix(0, BRICK_NO_WIRE, PROB_SHAPE(prob0), brick_fix) == brick_conn.NO_NEED_TRACE)
						continue;
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
						trace.found = false;
						trace.x = PROB_X(trace.prev_brick) / d.l[layer].gs;
						trace.y = PROB_Y(trace.prev_brick) / d.l[layer].gs;	
						if (trace.y >= y2) 
							in_trace = false;
						
						for (int yy = trace.y + 1; yy <= min(trace.y + cl1, y2); yy++)
							for (int xx = max(x1, trace.x - 1); xx <= min(trace.x + 1, x2); xx++) { //search first non invalid BRICK 
								prob0 = prob.at<unsigned long long>(yy, xx * 2);
								if (abs(PROB_X(trace.prev_brick) - PROB_X(prob0)) <= cw && PROB_SHAPE(prob0)!= BRICK_INVALID) {
									mark.at<unsigned char>(yy, xx) = 1;
									trace.found = true;
									yy = y2;
									break;									
								}
							}
						if (!trace.found) { //it also add a BRICK_NO_WIRE ending for DOWN_BORDER line
							noend_count++;
							SET_PROB_X(prob0, PROB_X(trace.prev_brick));
							SET_PROB_Y(prob0, PROB_Y(trace.prev_brick) + d.l[layer].gs);
							SET_PROB_TYPE(prob0, PROB_TYPE(trace.prev_brick));
							SET_PROB_SHAPE(prob0, BRICK_NO_WIRE);
						}
						
						//now prob0 is most possible brick
						if (PROB_TYPE(prob0) != PROB_TYPE(trace.prev_brick) && 
							PROB_SHAPE(prob0) != BRICK_NO_WIRE && PROB_SHAPE(trace.prev_brick)!=BRICK_NO_WIRE)
							qWarning("Tracking line 0 different type, x=%d, y=%d, prev_brick=%llx, prob0=%llx", 
							PROB_X(trace.prev_brick), PROB_Y(trace.prev_brick), trace.prev_brick, prob0);
						int fix_result = brick_conn.quick_fix(0, PROB_SHAPE(trace.prev_brick), PROB_SHAPE(prob0), brick_fix);
						int need_fix = 0;
						unsigned long long next_prob0;
						int suspect_brick = 0;
						int next_fix_result;
						if (PROB_SHAPE(trace.prev_brick) == BRICK_VIA && PROB_SHAPE(prob0) == BRICK_VIA) {
							int via_mask = 0;
							if (!(assemble_opt & ASSEMBLE_LINE_NO_VIA))
							for (int i = 0; i < via_info.size(); i++) {
								if (PROB_X(trace.prev_brick) == via_info[i].xy.x &&
									PROB_Y(trace.prev_brick) == via_info[i].xy.y &&
									PROB_TYPE(trace.prev_brick) == via_info[i].type) {
									via_mask = via_info[i].via_adj_mask;
									break;
								}
							}
							if ((via_mask & 1 << DIR_DOWN) == 0) {
								fix_result = BrickConnect::FIX_TRACE;
								brick_fix = BRICK_NO_WIRE;
							}
						}
						if (fix_result == BrickConnect::FIX_TRACE) {
							int nouse_brick_fix;
							need_fix = 1;
							next_prob0 = prob0;
							fix_result = brick_conn.quick_fix(0, PROB_SHAPE(trace.prev_brick), brick_fix, nouse_brick_fix);
							next_fix_result = brick_conn.quick_fix(0, brick_fix, PROB_SHAPE(prob0), nouse_brick_fix);
							CV_Assert(fix_result != BrickConnect::FIX_TRACE && next_fix_result != BrickConnect::FIX_TRACE);
							
							fix_count++;
							suspect_brick = SUSPECT_BRICK;

							SET_PROB_SHAPE(prob0, brick_fix);
							SET_PROB_X(prob0, (PROB_X(prob0) + PROB_X(trace.prev_brick)) / 2);
							SET_PROB_Y(prob0, (PROB_Y(prob0) + PROB_Y(trace.prev_brick)) / 2);							
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
								if (i == need_fix)
									in_trace = false;
								break;

							default:
								qFatal("brick_conn fix have internal error");
							}

							trace.prev_brick = prob0;
							prob0 = next_prob0;
							fix_result = next_fix_result;
							suspect_brick = 0;
						}
					}
					d.l[layer].lineset[0].push_back(cur_line);
				}
			}
	}

	mark = Scalar::all(0);
	for (int x = x1; x <= x2; x++) {
		for (int y = y1; y <= y2; y++)
			if (mark.at<unsigned char>(y, x) == 0) {
				unsigned long long prob0 = prob.at<unsigned long long>(y, x * 2);
				int brick_fix;
				if (PROB_SHAPE(prob0) != BRICK_NO_WIRE && PROB_SHAPE(prob0) != BRICK_INVALID) {//scan to find one begining brick
					CV_Assert(PROB_SHAPE(prob0) <= BRICK_IN_USE || PROB_SHAPE(prob0) == BRICK_FAKE_VIA || PROB_SHAPE(prob0) == BRICK_VIA);
					if (brick_conn.quick_fix(1, BRICK_NO_WIRE, PROB_SHAPE(prob0), brick_fix) == brick_conn.NO_NEED_TRACE)
						continue;
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
						trace.found = false;
						trace.x = PROB_X(trace.prev_brick) / d.l[layer].gs;
						trace.y = PROB_Y(trace.prev_brick) / d.l[layer].gs;
						if (trace.x >= x2)
							in_trace = false;

						for (int xx = trace.x + 1; xx <= min(trace.x + cl1, x2); xx++)
							for (int yy = max(y1, trace.y - 1); yy <= min(y2, trace.y + 1); yy++) { //search first non invalid BRICK								
								prob0 = prob.at<unsigned long long>(yy, xx * 2);
								if (abs(PROB_Y(trace.prev_brick) - PROB_Y(prob0)) <= cw && PROB_SHAPE(prob0) != BRICK_INVALID) {
									mark.at<unsigned char>(yy, xx) = 1;
									trace.found = true;
									xx = x2;
									break;
								}
							}
						if (!trace.found) { //it also add a BRICK_NO_WIRE ending for DOWN_BORDER line
							noend_count++;
							SET_PROB_X(prob0, PROB_X(trace.prev_brick) + d.l[layer].gs);
							SET_PROB_Y(prob0, PROB_Y(trace.prev_brick));
							SET_PROB_TYPE(prob0, PROB_TYPE(trace.prev_brick));
							SET_PROB_SHAPE(prob0, BRICK_NO_WIRE);
						}
						
						//now prob0 is most possible brick
						if (PROB_TYPE(prob0) != PROB_TYPE(trace.prev_brick) &&
							PROB_SHAPE(prob0) != BRICK_NO_WIRE && PROB_SHAPE(trace.prev_brick) != BRICK_NO_WIRE)
							qWarning("Tracking line 1, x=%d, y=%d, prev_brick=%llx, prob0=%llx",
							PROB_X(trace.prev_brick), PROB_Y(trace.prev_brick), trace.prev_brick, prob0);
						int fix_result = brick_conn.quick_fix(1, PROB_SHAPE(trace.prev_brick), PROB_SHAPE(prob0), brick_fix);
						int need_fix = 0;
						unsigned long long next_prob0;
						int suspect_brick = 0;
						int next_fix_result;
						if (PROB_SHAPE(trace.prev_brick) == BRICK_VIA && PROB_SHAPE(prob0) == BRICK_VIA) {
							int via_mask = 0;
							if (!(assemble_opt & ASSEMBLE_LINE_NO_VIA))
							for (int i = 0; i < via_info.size(); i++) {
								if (PROB_X(trace.prev_brick) == via_info[i].xy.x &&
									PROB_Y(trace.prev_brick) == via_info[i].xy.y &&
									PROB_TYPE(trace.prev_brick) == via_info[i].type) {
									via_mask = via_info[i].via_adj_mask;
									break;
								}
							}
							if ((via_mask & 1 << DIR_RIGHT) == 0) {
								fix_result = BrickConnect::FIX_TRACE;
								brick_fix = BRICK_NO_WIRE;
							}
						}
						if (fix_result == BrickConnect::FIX_TRACE) {
							int nouse_brick_fix;
							need_fix = 1;
							next_prob0 = prob0;
							fix_result = brick_conn.quick_fix(1, PROB_SHAPE(trace.prev_brick), brick_fix, nouse_brick_fix);
							next_fix_result = brick_conn.quick_fix(1, brick_fix, PROB_SHAPE(prob0), nouse_brick_fix);
							CV_Assert(fix_result != BrickConnect::FIX_TRACE && next_fix_result != BrickConnect::FIX_TRACE);

							fix_count++;
							suspect_brick = SUSPECT_BRICK;
							
							SET_PROB_SHAPE(prob0, brick_fix);
							SET_PROB_X(prob0, (PROB_X(prob0) + PROB_X(trace.prev_brick)) / 2);
							SET_PROB_Y(prob0, (PROB_Y(prob0) + PROB_Y(trace.prev_brick)) / 2);							
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
								if (i==need_fix)
									in_trace = false;
								break;

							default:
								qFatal("brick_conn fix have internal error");
							}

							trace.prev_brick = prob0;
							prob0 = next_prob0;
							fix_result = next_fix_result;
							suspect_brick = 0;
						}
					}
					d.l[layer].lineset[1].push_back(cur_line);
				}
			}
	}
#undef MAX_WIRE_NUM
	qDebug("Assemble noend_count =%d, fix_count=%d", noend_count, fix_count);
	if (cpara.method & OPT_DEBUG_EN) {
		Mat debug_draw = d.l[layer].img.clone();
		for (int dir = 0; dir < 2; dir++) {
			for (int i = 0; i < d.l[layer].lineset[dir].size(); i++)
				for (int j = 0; j < d.l[layer].lineset[dir][i].corner.size(); j++) {
					unsigned long long prob0 = d.l[layer].lineset[dir][i].corner[j];
					int x0 = PROB_X(prob0), y0 = PROB_Y(prob0);
					int shape = PROB_SHAPE(prob0);
					int color = 0;
					switch (d.l[layer].lineset[dir][i].state[j] & (START_BRICK | END_BRICK)) {
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
					if (shape <= BRICK_IN_USE) {
						for (int y = 0; y < 3; y++)
							for (int x = 0; x < 3; x++)
								if (bricks[shape].a[y][x])
									debug_draw.at<unsigned char>(y0 + y - 1, x0 + x - 1) = color;
					}
					else
						if (shape == BRICK_VIA) {
							for (int y = 0; y < 3; y++)
								for (int x = 0; x < 3; x++)
									debug_draw.at<unsigned char>(y0 + y - 1, x0 + x - 1) = color;
						}
				}			
		}
		imwrite(get_time_str() + "_l" + (char)('0' + layer) + "_assemble_line.jpg", debug_draw);
		if (cpara.method & OPT_DEBUG_OUT_EN) {
			int debug_idx = cpara.method >> 12 & 3;
			d.l[layer].v[debug_idx + 12].d = debug_draw.clone();
		}
	}
}

/*
		31..24 23..16   15..8   7..0
opt0:					filt_method process_method
opt1:							len	
opt2:							dir
method_opt
idx: via info
*/
static void filter_obj(CornerSets & obj_sets, int layer, int obj_type, ProcessParameter & cpara)
{
	int filt_method = cpara.opt0 >> 8 & 0xff;
	
	if (obj_type == OBJ_WIRE && layer==cpara.layer) {		
		switch (filt_method) {
		case 0: //filter wire by length
			{
				int len = cpara.opt1, dir;
				if (abs(obj_sets.cs.back().x() - obj_sets.cs[0].x()) < abs(obj_sets.cs.back().y() - obj_sets.cs[0].y()))
					dir = 0;
				else
					dir = 1;
				if (dir == cpara.opt2 && abs(obj_sets.cs.back().x() - obj_sets.cs[0].x()) < len 
					&& abs(obj_sets.cs.back().y() - obj_sets.cs[0].y()) < len)
					obj_sets.cs.clear();
			}
			break;
		default:
			qCritical("filter_obj wrong filt_method(%d)", filt_method);
		}		
	}
}

#define OP_FILTER 0
struct ObjProcess {
	int method;
	ObjProcessFunc process_func;
} obj_process_array[] = {
	OP_FILTER, filter_obj
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
#define PP_CHECK_VIA_WIRE_CONNECT	11
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
		{ PP_COARSE_VIA_MASK, coarse_via_search_mask },
		{ PP_FINE_VIA_SEARCH, fine_via_search },
		{ PP_REMOVE_VIA, remove_via },
		{ PP_FINE_SEARCH_MASK, hotpoint2fine_search_stmask },
		{ PP_FINE_LINE_SEARCH, fine_line_search },		
		{ PP_ASSEMBLE, assemble_line },
		{ PP_CHECK_VIA_WIRE_CONNECT, check_via_wire_connect }
};

class ProcessTileData {
public:
	vector<ProcessFunc> * process_func;
	vector<ProcessParameter> * vwp;
	PipeData * d;
	ProcessTileData() {
		process_func = NULL;
		vwp = NULL;
		d = NULL;
	}
};

//Process tile for all layer
static void process_tile(ProcessTileData & t)
{
	vector<ProcessParameter> & vwp = *(t.vwp);
	vector<ProcessFunc> & process_func = *(t.process_func);
	PipeData & d = *(t.d);
	for (int i = 0; i < vwp.size(); i++) {
		int method = vwp[i].method & 0xff;
		if (process_func[method] == NULL && method != PP_OBJ_PROCESS) {
			qCritical("process func method %d invalid", method);
			return;
		}
		if (process_func[method] != NULL)
			process_func[method](d, vwp[i]);
	}
}

VWExtractPipe::VWExtractPipe()
{
	private_data = NULL;
	prev_layer = -1;
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

	for (int l = 0; l < layer_num; l++) {
		for (int dir = 0; dir < 2; dir++) {
			for (int j = 0; j < d->l[l].lineset[dir].size(); j++) {
				WireLine & wl = d->l[l].lineset[dir][j];
				CV_Assert(wl.corner.size() == wl.state.size());
				d->l[l].fwl[dir].push(wl, 0, 0, dir);
			}

			for (int j = 0; j < d->l[l].fwl[dir].lines.size(); j++) {
				CornerSets & wl = d->l[l].fwl[dir].lines[j];
				for (int i = 0; i < (int)obj_process.size(); i++)
					obj_process[i].func(wl, l, OBJ_WIRE, obj_process[i].cpara);
				for (int i = 0; i < (int) wl.cs.size() - 1; i++) {
					MarkObj wire;
					wire.type = OBJ_LINE;
					wire.type2 = LINE_WIRE_AUTO_EXTRACT;
					wire.type3 = l;
					wire.state = 0;
					wire.prob = 1;
					wire.p0 = wl.cs[i];
					wire.p1 = wl.cs[i+1];
					obj_sets.push_back(wire);
				}
			}
		}

		for (int j = 0; j < d->l[l].viaset.size(); j++) {
			QPoint & point = d->l[l].viaset[j];
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
	return 0;
}


int VWExtractPipe::extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets)
{
	int BORDER_SIZE = 16;
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
	for (int i = 0; i < vwp.size(); i++) {
#if !DEBUG_LEFT_TOP_IMG
		vwp[i].method &= ~OPT_DEBUG_EN;
#endif
		qInfo("extract vw%d:l=0x%x,m=0x%x,mo=0x%x,o0=0x%x,o1=0x%x,o2=0x%x,o3=0x%x,o4=0x%x,o5=0x%x,o6=0x%x,i8=0x%x,f0=%f",
			i, vwp[i].layer, vwp[i].method, vwp[i].method_opt, vwp[i].opt0, vwp[i].opt1, vwp[i].opt2,
			vwp[i].opt3, vwp[i].opt4, vwp[i].opt5, vwp[i].opt6, vwp[i].opt_f0);	
		if ((vwp[i].opt0 >> 24) == 1 && (vwp[i].method & 0xff) == PP_SET_PARAM)
			BORDER_SIZE = vwp[i].opt0 >> 16 & 0xff;
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
			}
#if PARALLEL 
			//each thread process all layer on same tile
			QtConcurrent::blockingMap<vector <ProcessTileData> >(ptd_sets, process_tile);
#else
			for (int i = 0; i < ptd_sets.size(); i++)
				process_tile(ptd_sets[i]);

#endif
#if DEBUG_LEFT_TOP_IMG
			for (int i = 0; i < vwp.size(); i++)
				vwp[i].method &= ~OPT_DEBUG_EN;
#endif
			//3 output result to obj_sets
			for (int i = 0; i < cl_num; i++) {
				PipeData * lpd = &diag_line[1 - cl][i];
				PipeData * upd = &diag_line[1 - cl][i];
				PipeData & cpd = diag_line[cl][i];
				if (cpd.y0 > sb.top() && cpd.x0 != upd->x0)
					upd = &diag_line[1 - cl][i + 1];
				if (cpd.x0 > sb.left() && cpd.y0 != lpd->y0)
					lpd = &diag_line[1 - cl][i - 1];
				if (cpd.x0 > sb.left())
					CV_Assert(cpd.y0 == lpd->y0);
				if (cpd.y0 > sb.top())
					CV_Assert(cpd.x0 == upd->x0);
				
				for (int l = 0; l < layer_num; l++) {
					for (int dir = 0; dir < 2; dir++) {
						//3.1 copy left RIGHT_BORDER line to FinalWireLine
						if (cpd.x0 > sb.left())
						for (int j = 0; j < lpd->l[l].fwl[dir].lines.size(); j++) {
							CornerSets & line = lpd->l[l].fwl[dir].lines[j];
							if (line.cs[0].x() >= cpd.l[l].img_pixel_x0 ||
								line.cs.back().x() >= cpd.l[l].img_pixel_x0 )
								cpd.l[l].fwl[dir].merge(line, dir);
						}
						//3.2 copy up DOWN_BORDER line to FinalWireLine
						if (cpd.y0 > sb.top())
						for (int j = 0; j < upd->l[l].fwl[dir].lines.size(); j++) {
							CornerSets & line = upd->l[l].fwl[dir].lines[j];
							if (line.cs[0].y() >= cpd.l[l].img_pixel_y0 ||
								line.cs.back().y() >= cpd.l[l].img_pixel_y0)
								cpd.l[l].fwl[dir].merge(line, dir);
						}
						//3.3 push current line to FinalWireLine
						for (int j = 0; j < cpd.l[l].lineset[dir].size(); j++) {
							WireLine & wl = cpd.l[l].lineset[dir][j];
							CV_Assert(wl.corner.size() == wl.state.size());
							cpd.l[l].fwl[dir].push(wl, cpd.l[l].img_pixel_x0, cpd.l[l].img_pixel_y0, dir);
						}

						vector<CornerSets> border_lines;
						for (int j = 0; j < cpd.l[l].fwl[dir].lines.size(); j++) {
							CornerSets & wl = cpd.l[l].fwl[dir].lines[j];
							if ((wl.cs[0].x() < cpd.l[l].img_pixel_x0 + cpd.l[l].raw_img.cols - BORDER_SIZE * 2 &&
								wl.cs.back().x() < cpd.l[l].img_pixel_x0 + cpd.l[l].raw_img.cols - BORDER_SIZE * 2 &&
								wl.cs[0].y() < cpd.l[l].img_pixel_y0 + cpd.l[l].raw_img.rows - BORDER_SIZE * 2 &&
								wl.cs.back().y() < cpd.l[l].img_pixel_y0 + cpd.l[l].raw_img.rows - BORDER_SIZE * 2) ||
								(wl.cs[0].x() < cpd.l[l].img_pixel_x0 + cpd.l[l].raw_img.cols - BORDER_SIZE * 2 &&
								wl.cs.back().x() < cpd.l[l].img_pixel_x0 + cpd.l[l].raw_img.cols - BORDER_SIZE * 2 &&
								cpd.y0 == sb.bottom()) ||
								(wl.cs[0].y() < cpd.l[l].img_pixel_y0 + cpd.l[l].raw_img.rows - BORDER_SIZE * 2 &&
								wl.cs.back().y() < cpd.l[l].img_pixel_y0 + cpd.l[l].raw_img.rows - BORDER_SIZE * 2 &&
								cpd.x0 == sb.right()) ||
								(cpd.x0 == sb.right() && cpd.y0 == sb.bottom())) {
								//call hook to do obj process
								for (int i = 0; i < (int)obj_process.size(); i++)
									obj_process[i].func(wl, l, OBJ_WIRE, obj_process[i].cpara);
								//3.4 for internal points, push to output
								for (int i = 0; i < (int) wl.cs.size() - 1; i++) {
									MarkObj wire;
									wire.type = OBJ_LINE;
									wire.type2 = LINE_WIRE_AUTO_EXTRACT;
									wire.type3 = l;
									wire.state = 0;
									wire.prob = 1;
									wire.p0 = wl.cs[i] + sr_tl_pixel;
									wire.p1 = wl.cs[i+1] + sr_tl_pixel;
									obj_sets.push_back(wire);
								}
							}
							else //for border ppints, leave it for future merge
								border_lines.push_back(wl);
						}
						cpd.l[l].fwl[dir].lines.swap(border_lines);
						cpd.l[l].fwl[dir].recal_rect();
					}
					//3.5 copy left RIGHT_BORDER via to FinalVia
					if (cpd.x0 > sb.left())
						cpd.l[l].fv.merge(lpd->l[l].fv);
					
					//3.6 copy up DOWN_BORDER via to FinalVia
					if (cpd.y0 > sb.top())
						cpd.l[l].fv.merge(upd->l[l].fv);
					
					//3.7 push current via to FinalVia
					for (int j = 0; j < cpd.l[l].viaset.size(); j++) {
						QPoint & point = cpd.l[l].viaset[j];
						point = point + QPoint(cpd.l[l].img_pixel_x0, cpd.l[l].img_pixel_y0);
						cpd.l[l].fv.merge(point);
					}
					vector<QPoint> border_vias;
					for (int j = 0; j < cpd.l[l].fv.cs.size(); j++) {
						QPoint & point = cpd.l[l].fv.cs[j];
						if ((point.x() < cpd.l[l].img_pixel_x0 + cpd.l[l].raw_img.cols - BORDER_SIZE * 2 &&
							point.y() < cpd.l[l].img_pixel_y0 + cpd.l[l].raw_img.rows - BORDER_SIZE * 2) ||
							(point.x() < cpd.l[l].img_pixel_x0 + cpd.l[l].raw_img.cols - BORDER_SIZE * 2 && cpd.y0 == sb.bottom()) ||
							(point.y() < cpd.l[l].img_pixel_y0 + cpd.l[l].raw_img.rows - BORDER_SIZE * 2 && cpd.x0 == sb.right()) ||
							(cpd.x0 == sb.right() && cpd.y0 == sb.bottom())) {
							//3.4 for internal points, push to output
							MarkObj via;
							via.type = OBJ_POINT;
							via.type2 = POINT_VIA_AUTO_EXTRACT;
							via.type3 = l;
							via.state = 0;
							via.prob = 1;
							via.p0 = point + sr_tl_pixel;
							via.p1 = point + sr_tl_pixel;
							obj_sets.push_back(via);
						}
						else
							border_vias.push_back(point);
					}
					cpd.l[l].fv.cs.swap(border_vias);
				}
			}
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
		t = (t << 8) | obj_sets[i].type2;
		t = (t << 8) | obj_sets[i].type3;
		fprintf(fp, "t=%d, (%d,%d)->(%d,%d)\n", t, obj_sets[i].p0.x(), obj_sets[i].p0.y(),
			obj_sets[i].p1.x(), obj_sets[i].p1.y());
	}
	fclose(fp);
#endif	

	for (int i = 0; i < obj_sets.size(); i++) {
		obj_sets[i].p0 = obj_sets[i].p0 * scale;
		obj_sets[i].p1 = obj_sets[i].p1 * scale;
	}
	
	return 0;
}

VWExtract * VWExtract::create_extract(int)
{
	return new VWExtractPipe;
}
