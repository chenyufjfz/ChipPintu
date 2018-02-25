#ifndef VWEXTRACT_PUBLIC_H
#define VWEXTRACT_PUBLIC_H
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;

#ifndef QT_DEBUG
void print_stack(void);
#undef CV_Assert
#define CV_Assert(x) do {if (!(x)) {print_stack(); qFatal("Wrong at %s, %d", __FILE__, __LINE__);}} while(0)
#endif

#ifdef QT_DEBUG
#ifdef Q_OS_WIN
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif

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
#define BRICK_J_0			16
#define BRICK_J_90			17
#define BRICK_J_180			18
#define BRICK_J_270			19
#define BRICK_l_0			20
#define BRICK_l_90			21
#define BRICK_l_180			22
#define BRICK_l_270			23
#define BRICK_Z_0			24
#define BRICK_Z_90			25
#define BRICK_P_0			26
#define BRICK_P_90			27
#define BRICK_P_180			28
#define BRICK_P_270			29
#define BRICK_Y_45			30
#define BRICK_Y_135			31
#define BRICK_Y_225			32
#define BRICK_Y_315			33
#define BRICK_IN_USE		33
#define BRICK_ONE_POINT		64
#define BRICK_II_0			65
#define BRICK_II_90			66
#define BRICK_II_180		67
#define BRICK_II_270		68
#define BRICK_III_0			69
#define BRICK_III_90		70
#define BRICK_HOLLOW		252
#define BRICK_FAKE_VIA		253
#define BRICK_VIA			254
#define BRICK_INVALID		255

#define MIN_SCORE			10

#define DIR_UP				0
#define DIR_RIGHT			1
#define DIR_DOWN			2
#define DIR_LEFT			3
#define DIR_UPRIGHT			4
#define DIR_DOWNRIGHT		5
#define DIR_DOWNLEFT		6
#define DIR_UPLEFT			7

#define DIR_UP1_MASK		(1 << DIR_UP)
#define DIR_RIGHT1_MASK		(1 << DIR_RIGHT)
#define DIR_DOWN1_MASK		(1 << DIR_DOWN)
#define DIR_LEFT1_MASK		(1 << DIR_LEFT)
#define DIR_UPLEFT_MASK		(1 << DIR_UPLEFT)
#define DIR_UPRIGHT_MASK	(1 << DIR_UPRIGHT)
#define DIR_DOWNLEFT_MASK	(1 << DIR_DOWNLEFT)
#define DIR_DOWNRIGHT_MASK	(1 << DIR_DOWNRIGHT)

struct Brick {
	int a[3][3];
	int shape;
};

class BrickConnect {
protected:
	unsigned long long bfm[8][64];
	int sa[BRICK_IN_USE + 1][BRICK_IN_USE + 1];
protected:

	/*
	input: brick0, brick1
	output: brick0 + brick1
	*/
	int shape_add(int brick0, int brick1);

public:
	//fit check if brick0's dir can be brick1
	bool fit(int dir, int brick0, int brick1);
	BrickConnect();
	int quick_shape_add(int brick0, int brick1);
};

#ifndef VWEXTRACT_PUBLIC_C
extern struct Brick bricks[];
extern int dxy[8][2];
extern int dir_1[8];
extern int dir_2[8];
extern BrickConnect brick_conn;
#endif

/*
Compute integrate and line integral
in img
in compute_line
out ig, same as openCV integral
out iig, sum(sum(img(i,j)*img(i,j), j=0..x-1) i=0..y-1
out lg, sum(img(i,j), j=0..x-1)
out llg, sum(img(i,j)*img(i,j), j=0..x-1)
*/
void integral_square(const Mat & img, Mat & ig, Mat & iig, Mat & lg, Mat & llg, bool compute_line_integral);
void clip_img(Mat & img, int gray_low, int gray_high, Mat & new_img);
#endif // VWEXTRACT_PUBLIC_H

