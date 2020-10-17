#include <string.h>
#define VWEXTRACT_PUBLIC_C
#include "vwextract_public.h"
#include <qmath.h>
#include <QRect>
#include <algorithm>
#include <qdir.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include<functional>
#include <QMutexLocker>
using namespace cv;

#ifdef Q_OS_WIN
#ifdef QT_DEBUG
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif

#define VIA_MINIMUM_GRAD			-1000
#define NOVIA_MINIMUM_GRAD			-1500
#define CIRCLE_EDGE_AVG_LEN			6
#define TRAIN_NUM_TH				5
#define MIN_DARK_GAP				5
#define EDGE_GRAD_GAP				4
#define VIA_REGION_RADIUS			4
#define VIA_MARK_VIA_CENTER			8
#define VIA_MARK_VIA_REGION			4
#define VIA_MARK_NEAR_VIA			2
#define VIA_MARK_NO_VIA				1

int dxy[8][2] = {
	//y , x
	{ -1, 0 }, //up
	{ 0, 1 }, //right
	{ 1, 0 }, //down
	{ 0, -1 }, //left
	{ -1, 1 }, //upright
	{ 1, 1 }, //downright
	{ 1, -1 }, //downleft
	{ -1, -1 } //upleft
};

int dir_1[8] = {
	DIR_DOWN,
	DIR_LEFT,
	DIR_UP,
	DIR_RIGHT,
	DIR_DOWNLEFT,
	DIR_UPLEFT,
	DIR_UPRIGHT,
	DIR_DOWNRIGHT
};

int dir_2[8] = {
	DIR_RIGHT,
	DIR_DOWN,
	DIR_LEFT,
	DIR_UP,
	DIR_DOWNRIGHT,
	DIR_DOWNLEFT,
	DIR_UPLEFT,
	DIR_UPRIGHT
};

int dir_3[8] = {
	DIR_UPRIGHT,
	DIR_DOWNRIGHT,
	DIR_DOWNLEFT,
	DIR_UPLEFT,
	DIR_RIGHT,
	DIR_DOWN,
	DIR_LEFT,
	DIR_UP
};

bool lessPoint2(const Point & a, const Point & b) { return a.x < b.x; }

bool contain_dir(int d1, int d2)
{
	if (d1 == d2)
		return true;
	if (d1 > 3) {
		switch (d1) {
		case DIR_UPRIGHT:
			return (d2 == DIR_UP || d2 == DIR_RIGHT);
		case DIR_UPLEFT:
			return (d2 == DIR_UP || d2 == DIR_LEFT);
		case DIR_DOWNLEFT:
			return (d2 == DIR_DOWN || d2 == DIR_LEFT);
		case DIR_DOWNRIGHT:
			return (d2 == DIR_DOWN || d2 == DIR_RIGHT);
		}
	}
	return false;
}

BrickConnect brick_conn;

struct Brick bricks[] = {
	{ {
		{ 0, 0, 0 },
		{ 0, 0, 0 },
		{ 0, 0, 0 } },
		0
	},

	{ {
		{ 0, 1, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 0 } },
		DIR_UP1_MASK
	},

	{ {
		{ 0, 0, 0 },
		{ 0, 1, 1 },
		{ 0, 0, 0 } },
		DIR_RIGHT1_MASK
	},

	{ {
		{ 0, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 1, 0 } },
		DIR_DOWN1_MASK
	},

	{ {
		{ 0, 0, 0 },
		{ 1, 1, 0 },
		{ 0, 0, 0 } },
		DIR_LEFT1_MASK
	},
	{ {
		{ 0, 1, 0 },
		{ 0, 1, 0 },
		{ 0, 1, 0 } },
		DIR_UP1_MASK | DIR_DOWN1_MASK
	},

	{ {
		{ 0, 0, 0 },
		{ 1, 1, 1 },
		{ 0, 0, 0 } },
		DIR_LEFT1_MASK | DIR_RIGHT1_MASK
	},

	{ {
		{ 0, 1, 0 },
		{ 0, 1, 1 },
		{ 0, 0, 0 } },
		DIR_UP1_MASK | DIR_RIGHT1_MASK
	},

	{ {
		{ 0, 0, 0 },
		{ 0, 1, 1 },
		{ 0, 1, 0 } },
		DIR_RIGHT1_MASK | DIR_DOWN1_MASK
	},

	{ {
		{ 0, 0, 0 },
		{ 1, 1, 0 },
		{ 0, 1, 0 } },
		DIR_DOWN1_MASK | DIR_LEFT1_MASK
	},

	{ {
		{ 0, 1, 0 },
		{ 1, 1, 0 },
		{ 0, 0, 0 } },
		DIR_UP1_MASK | DIR_LEFT1_MASK
	},

	{ {
		{ 0, 1, 0 },
		{ 0, 1, 1 },
		{ 0, 1, 0 } },
		DIR_UP1_MASK | DIR_RIGHT1_MASK | DIR_DOWN1_MASK
	},

	{ {
		{ 0, 0, 0 },
		{ 1, 1, 1 },
		{ 0, 1, 0 } },
		DIR_RIGHT1_MASK | DIR_DOWN1_MASK | DIR_LEFT1_MASK
	},

	{ {
		{ 0, 1, 0 },
		{ 1, 1, 0 },
		{ 0, 1, 0 } },
		DIR_UP1_MASK | DIR_LEFT1_MASK | DIR_DOWN1_MASK
	},

	{ {
		{ 0, 1, 0 },
		{ 1, 1, 1 },
		{ 0, 0, 0 } },
		DIR_UP1_MASK | DIR_LEFT1_MASK | DIR_RIGHT1_MASK
	},

	{ {
		{ 0, 1, 0 },
		{ 1, 1, 1 },
		{ 0, 1, 0 } },
		DIR_UP1_MASK | DIR_DOWN1_MASK | DIR_LEFT1_MASK | DIR_RIGHT1_MASK
	},
	{ {
		{ 0, 1, 0 },
		{ 0, 1, 0 },
		{ 1, 0, 0 } },
		DIR_UP1_MASK | DIR_DOWNLEFT_MASK
	},
	{ {
		{ 1, 0, 0 },
		{ 0, 1, 1 },
		{ 0, 0, 0 } },
		DIR_RIGHT1_MASK | DIR_UPLEFT_MASK
	},
	{ {
		{ 0, 0, 1 },
		{ 0, 1, 0 },
		{ 0, 1, 0 } },
		DIR_UPRIGHT_MASK | DIR_DOWN1_MASK
	},
	{ {
		{ 0, 0, 0 },
		{ 1, 1, 0 },
		{ 0, 0, 1 } },
		DIR_DOWNRIGHT_MASK | DIR_LEFT1_MASK
	},
	{ {
		{ 0, 1, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 } },
		DIR_DOWNRIGHT_MASK | DIR_UP1_MASK
	},
	{ {
		{ 0, 0, 0 },
		{ 0, 1, 1 },
		{ 1, 0, 0 } },
		DIR_DOWNLEFT_MASK | DIR_RIGHT1_MASK
	},
	{ {
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 1, 0 } },
		DIR_UPLEFT_MASK | DIR_DOWN1_MASK
	},
	{ {
		{ 0, 0, 1 },
		{ 1, 1, 0 },
		{ 0, 0, 0 } },
		DIR_UPRIGHT_MASK | DIR_LEFT1_MASK
	},
	{ {
		{ 0, 0, 1 },
		{ 0, 1, 0 },
		{ 1, 0, 0 } },
		DIR_UPRIGHT_MASK | DIR_DOWNLEFT_MASK
	},
	{ {
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 } },
		DIR_UPLEFT_MASK | DIR_DOWNRIGHT_MASK
	},
	{ {
		{ 0, 0, 1 },
		{ 0, 1, 0 },
		{ 0, 0, 0 } },
		DIR_UPRIGHT_MASK
	},
	{ {
		{ 0, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 } },
		DIR_DOWNRIGHT_MASK
	},
	{ {
		{ 0, 0, 0 },
		{ 0, 1, 0 },
		{ 1, 0, 0 } },
		DIR_DOWNLEFT_MASK
	},
	{ {
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 0 } },
		DIR_UPLEFT_MASK
	},
	{ {
		{ 0, 1, 0 },
		{ 0, 1, 1 },
		{ 1, 0, 0 } },
		DIR_UP1_MASK | DIR_RIGHT1_MASK | DIR_DOWNLEFT_MASK,
	},
	{ {
		{ 1, 0, 0 },
		{ 0, 1, 1 },
		{ 0, 1, 0 } },
		DIR_UPLEFT_MASK | DIR_RIGHT1_MASK | DIR_DOWN1_MASK,
	},
	{ {
		{ 0, 0, 1 },
		{ 1, 1, 0 },
		{ 0, 1, 0 } },
		DIR_LEFT1_MASK | DIR_UPRIGHT_MASK | DIR_DOWN1_MASK,
	},
	{ {
		{ 0, 1, 0 },
		{ 1, 1, 0 },
		{ 0, 0, 1 } },
		DIR_LEFT1_MASK | DIR_UP1_MASK | DIR_DOWNRIGHT_MASK,
	}
};

/*
input: brick0, brick1
output: brick0 + brick1
*/
int BrickConnect::shape_add(int brick0, int brick1)
{
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
	}
	return BRICK_FAKE_VIA;
}

//fit check if brick0's dir can be brick1
bool BrickConnect::fit(int dir, int brick0, int brick1)
{
	if (brick0 == BRICK_ONE_POINT || brick0 == BRICK_INVALID)
		brick0 = BRICK_NO_WIRE;
	if (brick1 == BRICK_ONE_POINT || brick1 == BRICK_INVALID)
		brick1 = BRICK_NO_WIRE;
	if (brick0 == BRICK_II_0 || brick0 == BRICK_II_180 || brick0 == BRICK_III_0)
		brick0 = BRICK_I_0;
	if (brick0 == BRICK_II_90 || brick0 == BRICK_II_270 || brick0 == BRICK_III_90)
		brick0 = BRICK_I_90;
	if (brick0 < sizeof(bricks) / sizeof(bricks[0]) && brick1 < sizeof(bricks) / sizeof(bricks[0]))
		return (bfm[dir][brick0] & 1ULL << brick1) ? true : false;
	else {
		if ((brick0 == BRICK_FAKE_VIA || brick0 == BRICK_VIA) && brick1 < sizeof(bricks) / sizeof(bricks[0])) {
			int adir = dir_1[dir];
			return bricks[brick1].a[1 + dxy[adir][0]][1 + dxy[adir][1]] ? true : false;
		}
		if ((brick1 == BRICK_FAKE_VIA || brick1 == BRICK_VIA) && brick0 < sizeof(bricks) / sizeof(bricks[0]))
			return bricks[brick0].a[1 + dxy[dir][0]][1 + dxy[dir][1]] ? true : false;
		if (brick0 == BRICK_FAKE_VIA || brick0 == BRICK_VIA || brick1 == BRICK_FAKE_VIA || brick1 == BRICK_VIA)
			return true;
		return false;
	}
}

BrickConnect::BrickConnect() {
	memset(bfm, 0, sizeof(bfm));
	CV_Assert(sizeof(bricks) / sizeof(bricks[0]) == BRICK_IN_USE + 1 && sizeof(bricks) / sizeof(bricks[0]) < 64);
	for (int i = 0; i < sizeof(bricks) / sizeof(bricks[0]); i++)
	for (int j = 0; j < sizeof(bricks) / sizeof(bricks[0]); j++) {
		if (bricks[i].a[0][0] == bricks[j].a[2][0] && bricks[i].a[0][1] == bricks[j].a[2][1] &&
			bricks[i].a[0][2] == bricks[j].a[2][2]) {
			bfm[DIR_UP][i] |= 1ULL << j;
			bfm[DIR_DOWN][j] |= 1ULL << i;
		}
		if (bricks[i].a[0][2] == bricks[j].a[0][0] && bricks[i].a[1][2] == bricks[j].a[1][0] &&
			bricks[i].a[2][2] == bricks[j].a[2][0]) {
			bfm[DIR_RIGHT][i] |= 1ULL << j;
			bfm[DIR_LEFT][j] |= 1ULL << i;
		}
		if (bricks[i].a[0][2] == bricks[j].a[2][0]) {
			bfm[DIR_UPRIGHT][i] |= 1ULL << j;
			bfm[DIR_DOWNLEFT][j] |= 1ULL << i;
		}
		if (bricks[i].a[2][2] == bricks[j].a[0][0]) {
			bfm[DIR_DOWNRIGHT][i] |= 1ULL << j;
			bfm[DIR_UPLEFT][j] |= 1ULL << i;
		}
	}

	for (int i = 0; i <= BRICK_IN_USE; i++)
	for (int j = 0; j <= BRICK_IN_USE; j++)
		sa[i][j] = shape_add(i, j);
}

int BrickConnect::quick_shape_add(int brick0, int brick1) 
{
	if (brick0 <= BRICK_IN_USE && brick1 <= BRICK_IN_USE)
		return sa[brick0][brick1];
	else
		return shape_add(brick0, brick1);
}

void deldir(const string &path)
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

/*
input pt1, pt2
output pts, line points include [pt1,pt2)
go with - / or | /
*/
void get_line_pts(Point pt1, Point pt2, vector <Point> & pts)
{
	int dx = abs(pt2.x - pt1.x);
	int dy = abs(pt2.y - pt1.y);
	bool dir = dx > dy;
	int ix = (pt2.x - pt1.x > 0) ? 1 : -1;
	int iy = (pt2.y - pt1.y > 0) ? 1 : -1;
	pts.clear();
	if (dir) {
		int dy2 = 2 * dy;
		int dy2dx2 = 2 * dy - 2 * dx;
		int d = 2 * dy - dx;
		int x = pt1.x, y = pt1.y;
		for (; x != pt2.x; x += ix) {
			pts.push_back(Point(x, y));
			if (d < 0)
				d += dy2;
			else {
				d += dy2dx2;
				y += iy;
			}
		}
		CV_Assert(y == pt2.y);
	}
	else {
		int dx2 = 2 * dx;
		int dx2dy2 = 2 * dx - 2 * dy;
		int d = 2 * dx - dy;
		int x = pt1.x, y = pt1.y;
		for (; y != pt2.y; y += iy) {
			pts.push_back(Point(x, y));
			if (d < 0)
				d += dx2;
			else {
				d += dx2dy2;
				x += ix;
			}
		}
		CV_Assert(x == pt2.x);
	}
}

/*
input pt1, pt2
output pts, line points include [pt1,pt2]
go with - |
*/
void get_line_pts2(Point pt1, Point pt2, vector <Point> & pts)
{
	int dx = abs(pt2.x - pt1.x);
	int dy = abs(pt2.y - pt1.y);
	bool dir = dx > dy;
	int ix = (pt2.x - pt1.x > 0) ? 1 : -1;
	int iy = (pt2.y - pt1.y > 0) ? 1 : -1;
	pts.clear();
	if (dir) {
		int dy2 = 2 * dy;
		int dy2dx2 = 2 * dy - 2 * dx;
		int d = 2 * dy - dx;
		int x = pt1.x, y = pt1.y;
		for (; x != pt2.x; x += ix) {
			pts.push_back(Point(x, y));
			if (d < 0)
				d += dy2;
			else {
				d += dy2dx2;
				pts.push_back(Point(x + ix, y));
				y += iy;
			}
		}
		pts.push_back(Point(x, y));
		CV_Assert(y == pt2.y);
	}
	else {
		int dx2 = 2 * dx;
		int dx2dy2 = 2 * dx - 2 * dy;
		int d = 2 * dx - dy;
		int x = pt1.x, y = pt1.y;
		for (; y != pt2.y; y += iy) {
			pts.push_back(Point(x, y));
			if (d < 0)
				d += dx2;
			else {
				d += dx2dy2;
				pts.push_back(Point(x, y + iy));
				x += ix;
			}
		}
		pts.push_back(Point(x, y));
		CV_Assert(x == pt2.x);
	}
}

void save_rst_to_file(const vector<MarkObj> & obj_sets, int scale)
{
	FILE * fp;
	fp = fopen("result.txt", "w");
	if (fp) {
		for (int i = 0; i < obj_sets.size(); i++) {
			unsigned t = obj_sets[i].type;
			if (t == OBJ_POINT) {
				if (obj_sets[i].type2 == POINT_NO_VIA)
					fprintf(fp, "novia, l=%d, x=%d, y=%d, prob=%f\n", obj_sets[i].type3, obj_sets[i].p0.x() / scale, obj_sets[i].p0.y() / scale, obj_sets[i].prob);
				else
					fprintf(fp, "via, l=%d, x=%d, y=%d, prob=%f\n", obj_sets[i].type3, obj_sets[i].p0.x() / scale, obj_sets[i].p0.y() / scale, obj_sets[i].prob);
			}
			else {
				fprintf(fp, "wire, l=%d, (x=%d,y=%d)->(x=%d,y=%d), prob=%f\n", obj_sets[i].type3, obj_sets[i].p0.x() /scale, 
					obj_sets[i].p0.y() / scale, obj_sets[i].p1.x() / scale, obj_sets[i].p1.y() / scale, obj_sets[i].prob);
			}
			continue;
		}
		fclose(fp);
	}
}
/*
Compute integrate and line integral
in img
in compute_line_integral, compute lg, llg
out ig, same as openCV integral
out iig, sum(sum(img(i,j)*img(i,j), j=0..x-1) i=0..y-1
out lg, sum(img(i,j), j=0..x-1)
out llg, sum(img(i,j)*img(i,j), j=0..x-1)
*/
void integral_square(const Mat & img, Mat & ig, Mat & iig, Mat & lg, Mat & llg, bool compute_line_integral)
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
input img
input gray_low
input gray_high
output new_img
clip img gray to [0, gray_high - gray_low] and output to new_img
*/
void clip_img(Mat & img, int gray_low, int gray_high, Mat & new_img)
{
	CV_Assert(img.type() == CV_8UC1);
	new_img.create(img.size(), img.type());
	for (int y = 0; y < img.rows; y++) {
		unsigned char * p_img = img.ptr<unsigned char>(y);
		unsigned char * p_newimg = new_img.ptr<unsigned char>(y);
		for (int x = 0; x < img.cols; x++)
			p_newimg[x] = (p_img[x] > gray_high) ? gray_high - gray_low :
			((p_img[x] < gray_low) ? 0 : p_img[x] - gray_low);
	}
}

static const Mat Horizon = (Mat_<char>(5, 5) << -1, -1, 0, 1, 1, -1, -2, 0, 2, 1, -2, -4, 0, 4, 2, -1, -2, 0, 2, 1, -1, -1, 0, 1, 1);
static const Mat Vert = (Mat_<char>(5, 5) << -1, -1, -2, -1, -1, -1, -2, -4, -2, -1, 0, 0, 0, 0, 0, 1, 2, 4, 2, 1, 1, 1, 2, 1, 1);
static const Mat Deg45 = (Mat_<char>(5, 5) << 0, -1, -2, -1, 0, -1, -2, -3, 0, 1, -2, -3, 0, 3, 2, -1, 0, 3, 2, 1, 0, 1, 2, 1, 0);
static const Mat Deg135 = (Mat_<char>(5, 5) << 0, 1, 2, 1, 0, -1, 0, 3, 2, 1, -2, -3, 0, 3, 2, -1, -2, -3, 0, 1, 0, -1, -2, -1, 0);

void CircleCheck::init(int _d0, int _th, const Mat * d, const Mat * s)
{
	if (d0 != _d0) {
		for (int i = 0; i < 8; i++) 
			dir_shift[i] = (dxy[i][0] * (int)s->step.p[0] + dxy[i][1] * (int)s->step.p[1]) / sizeof(Vec2f);
		
		d0 = _d0;
		margin = d0 + 1;
		eo.clear();
		ei.clear();
		eshift.clear();
		for (int d1 = d0 - 1; d1 <= d0 + 1; d1++)
		for (int d2 = d0 - 1; d2 <= d0 + 1; d2++) {
			if (abs(d1 - d2) > 1)
				continue;
			float r1 = d1 * 0.5f;
			float r2 = d2 * 0.5f;
			vector<Point2f> e[8];
			float minx = 10000, miny = 10000;
			for (int i = 0; i < 8; i++) {
				for (float beta = M_PI_4 * (i - 0.51); beta <= M_PI_4 * (i + 0.52); beta += 0.06) {
					e[i].push_back(Point2f(r1 * cosf(beta), -r2 * sinf(beta)));
					minx = min(minx, e[i].back().x);
					miny = min(miny, e[i].back().y);
				}
			}
			Point2f tl(minx, miny);
			Vec8i eo_idx;
			for (int i = 0; i < 8; i++) {
				vector<Point> new_eo;
				vector<int> new_eshift;
				switch (i) {
				case 0:
					new_eo.push_back(Point(DIR_RIGHT, -9));
					break;
				case 1:
					new_eo.push_back(Point(DIR_UPRIGHT, -9));
					break;
				case 2:
					new_eo.push_back(Point(DIR_UP, -9));
					break;
				case 3:
					new_eo.push_back(Point(DIR_UPLEFT, -9));
					break;
				case 4:
					new_eo.push_back(Point(DIR_LEFT, -9));
					break;
				case 5:
					new_eo.push_back(Point(DIR_DOWNLEFT, -9));
					break;
				case 6:
					new_eo.push_back(Point(DIR_DOWN, -9));
					break;
				case 7:
					new_eo.push_back(Point(DIR_DOWNRIGHT, -9));
					break;
				}
				new_eshift.push_back(new_eo[0].x);
				for (int j = 0; j < (int)e[i].size(); j++) {
					e[i][j] -= tl;
					Point p(e[i][j].x + 0.5f, e[i][j].y + 0.5f);
					CV_Assert(p.x <= margin && p.y <= margin);
					if (new_eo.back() != p) { //new edge point
						new_eo.push_back(p);
						CV_Assert((p.y * (int)d->step.p[0] + p.x * (int)d->step.p[1]) / sizeof(Vec2b) ==
							(p.y * (int)s->step.p[0] + p.x * (int)s->step.p[1]) / sizeof(Vec2f));
						new_eshift.push_back((p.y * (int)s->step.p[0] + p.x * (int)s->step.p[1]) / sizeof(Vec2f));
					}
				}
				bool found_same = false;
				for (int j = 0; j < (int)eo.size(); j++)
				if (new_eo == eo[j]) {
					found_same = true;
					eo_idx[i] = j;
					break;
				}
				if (!found_same) {
					eo_idx[i] = (int)eo.size();
					eo.push_back(new_eo);
					eshift.push_back(new_eshift);
				}
			}
			ei.push_back(eo_idx);
		}
	}
	if (th != _th) {
		th = _th;
		for (int i = 0; i < 8; i++)
		for (int j = 0; j < 8; j++) {
			if (i == j)
				coef[i][j] = 1; //same dir
			else {
				int a = dxy[i][0] * dxy[j][0] + dxy[i][1] * dxy[j][1];
				coef[i][j] = (a > 0) ? 0 : ((a == 0) ? -0.35 : -1); //a=0, means orthogonality, a<0 means opposite
				coef[i][j] = coef[i][j] * th / 20; //add more punish when th go higher
			}
		}
	}
}

Point CircleCheck::check(const Mat * d, const Mat * s, const Mat * img, int _d0, int _th, int _thr, vector<Point> * vs)
{
	if (d0 != _d0 || th != _th)
		init(_d0, _th, d, s);
	if (vs != NULL)
		vs->clear();
	vector<float> es(eo.size()), es2(eo.size()); //edge score, es is grad score, es2 is ratio score
	float max_score = 0; //best score
	int max_ei = -1; //best octagon shape
	Point max_o; //best location
	bool pass = false; //pass=true means at least one octagon satisfy
	float thf = th / 100.0, thrf = 1 + _thr / 100.0, thrf2 = 1 + _thr / 200.0;
	for (int y = 1; y < d->rows - margin - 1; y++)
	for (int x = 1; x < d->cols - margin - 1; x++) {
		Point o(x, y);
		const Vec2b * pd = d->ptr<Vec2b>(o.y, o.x);
		const Vec2f * ps = s->ptr<Vec2f>(o.y, o.x);
		const uchar * pimg = img->ptr<uchar>(o.y, o.x);
		for (int i = 0; i < (int)eo.size(); i++) { //compute every edge
			int exp_dir = eo[i][0].x;
			int g0 = 0, g1 = 0;
			es[i] = 0;
			for (int j = 1; j < (int)eo[i].size(); j++) {
				Vec2b dir = pd[eshift[i][j]];
				Vec2f score = ps[eshift[i][j]];
				g0 += pimg[eshift[i][j] + dir_shift[exp_dir]];
				g1 += pimg[eshift[i][j] - dir_shift[exp_dir]];
				CV_Assert(exp_dir < 8 && dir[0] < 8 && dir[1] < 8);
				if (coef[exp_dir][dir[0]] != 0)
					es[i] += coef[exp_dir][dir[0]] * score[0];
				else { //main dir is compatible with exp_dir, check 2nd dir
					if (coef[exp_dir][dir[1]] < 0)
						es[i] += coef[exp_dir][dir[1]] * score[1];
					else
						es[i] += coef[exp_dir][dir[1]] * score[1] * 0.6;
				}

			}
			es[i] = es[i] / (eo[i].size() - 1); //-1 because eo[i][0] is for dir
			es2[i] = (g0 == 0) ? ((g1 <= 10) ? 0 : 10) : (float) g1 / g0;
			if (exp_dir > 3)
				es[i] = es[i] * 1.1; //increase 45, 135 degree
		}

		for (int i = 0; i < (int)ei.size(); i++) {//check every octagon
			int pass1 = 0, pass0 = 0; //pass1 means thf pass number, pass0 means half thf pass number
			float score = 0;
			for (int j = 0; j < 8; j++) {
				if (es[ei[i][j]] < 0 || es2[ei[i][j]] <= 1) {
					pass0 = 0;
					break;
				}
				if (es[ei[i][j]] > thf && es2[ei[i][j]] > thrf)
					pass1++;
				if (es[ei[i][j]] > thf / 2 && es2[ei[i][j]] > thrf2)
					pass0++;
				score += es[ei[i][j]];
			}

			if (pass) {
				if (pass1 >= 7 && pass0 == 8 && score > max_score) {
					max_score = score;
					max_ei = i;
					max_o = o;
				}
			}
			else {
				if (score > max_score) {
					max_score = score;
					max_ei = i;
					max_o = o;
				}
				if (pass1 >= 7 && pass0 == 8) {
					pass = true;
					max_score = score;
					max_ei = i;
					max_o = o;
				}
			}
		}

		if (pass && abs(max_o.x + d0 / 2 - d->cols / 2) <= 1 && abs(max_o.y + d0 / 2 - d->rows / 2) <= 1 &&
			y + d0 / 2 > d->rows / 2 + 1) { //Already pass, quit quickly to optimize speed
			y = d->rows;
			break;
		}

	}
	if (vs != NULL && max_ei >= 0) {
		for (int j = 0; j < 8; j++) {
			int idx = ei[max_ei][j];
			int exp_dir = eo[idx][0].x;
			for (int k = 1; k < (int)eo[idx].size(); k++) {
				Vec2b dir = d->at<Vec2b>(max_o + eo[idx][k]);
				if (exp_dir == dir[0] || coef[exp_dir][dir[0]] == 0 && exp_dir == dir[1])
					vs->push_back(max_o + eo[idx][k]); //push border point
			}
		}
	}
	if (pass)
		return max_o + Point(d0 / 2 + 1, d0 / 2 + 1);
	else
		return Point(-1, -1);
}

Point CircleCheck::via_double_check(const Mat * img, int xo, int yo, int d0, int r1, int th, int th2, int thr, vector<Point> * vs)
{
	Mat_<Vec2b> d(r1 * 2 + 1, r1 * 2 + 1); //vec0 means main dir, vec1 means 2nd dir
	Mat_<Vec2f> s(r1 * 2 + 1, r1 * 2 + 1); //vec0 means main dir score, vec1 means 2nd dir score
	th = th * 16;

	//following compute grad
	for (int y = yo - r1; y <= yo + r1; y++)
	for (int x = xo - r1; x <= xo + r1; x++) {
		int sv = 0, sh = 0, s45 = 0, s135 = 0;
		for (int dy = -2; dy <= 2; dy++) {
			const uchar * p_img = img->ptr<uchar>(y + dy, x);
			const char * p_vert = Vert.ptr<char>(dy + 2, 2);
			const char * p_hori = Horizon.ptr<char>(dy + 2, 2);
			const char * p_deg45 = Deg45.ptr<char>(dy + 2, 2);
			const char * p_deg135 = Deg135.ptr<char>(dy + 2, 2);
			for (int dx = -2; dx <= 2; dx++) {
				int t = p_img[dx];
				sv += t * p_vert[dx];
				sh += t * p_hori[dx];
				s45 += t * p_deg45[dx];
				s135 += t * p_deg135[dx];
			}
		}
		int absv = abs(sv), absh = abs(sh);
		int abs45 = abs(s45), abs135 = abs(s135);
		float score0, score1; //score0 is bigest, score1 is 2nd big
		int dir0, dir1; //dir0 is best dir, dir1 is 2nd
		int dir = (sv > 0) ? DIR_UP : DIR_DOWN;
		score0 = absv;
		dir0 = dir;

		dir = (sh > 0) ? DIR_LEFT : DIR_RIGHT;
		if (absh > score0) {
			score1 = score0;
			score0 = absh;
			dir1 = dir0;
			dir0 = dir;
		}
		else {
			score1 = absh;
			dir1 = dir;
		}

		dir = (s45 > 0) ? DIR_UPLEFT : DIR_DOWNRIGHT;
		if (abs45 > score0) {
			score1 = score0;
			score0 = abs45;
			dir1 = dir0;
			dir0 = dir;
		}
		else
		if (abs45 > score1) {
			score1 = abs45;
			dir1 = dir;
		}

		dir = (s135 > 0) ? DIR_DOWNLEFT : DIR_UPRIGHT;
		if (abs135 > score0) {
			score1 = score0;
			score0 = abs135;
			dir1 = dir0;
			dir0 = dir;
		}
		else
		if (abs135 > score1) {
			score1 = abs135;
			dir1 = dir;
		}
		if (score0 > 1.5 * score1) { //main grad is very biger than 2nd grad, then no 2nd grad
			dir1 = dir0;
			score1 = score0;
		}
		score0 = min(score0 / th, 1.0f);
		score1 = min(score1 / th, 1.0f);
		s(y - yo + r1, x - xo + r1) = Vec2f(score0, score1);
		d(y - yo + r1, x - xo + r1) = Vec2b(dir0, dir1);
	}

	Mat img_in = (*img)(Rect(xo - r1, yo - r1, r1 * 2 + 1, r1 * 2 + 1)).clone();
	Point ret = check(&d, &s, &img_in, d0, th2, thr, vs);
	if (vs != NULL) {
		for (int i = 0; i < (int)vs->size(); i++)
			vs[0][i] += Point(xo - r1, yo - r1);
	}
	if (ret == Point(-1, -1))
		return ret;
	else
		return ret + Point(xo - r1, yo - r1);
}

/*
input d:
output d0: d0[0][0] is in [-d/2..d/2], d0[0][1] is in [0..d/2], d0[0][2] is in [-d/2..0]
*/
int compute_circle_dx(int d, vector<Vec3i> & d0)
{
	float r0 = d * 0.5;
	int r = d / 2;
	d0.clear();
	int n = 0;
	for (int y = -r; y <= r; y++) {
		float x1 = sqrt(r0 * r0 - y * y);
		float x2 = -x1;
		d0.push_back(Vec3i(y, x1 + 0.501, x2 - 0.501));
		n += d0.back()[1] - d0.back()[2] + 1;
	}
	return n;
}

/*
input d:
output d0: d0[0][0] is in [0..d], d0[0][1] is in [d/2..d], d0[0][2] is in [0..d/2]
*/
int compute_circle_dx1(int d, vector<Vec3i> & d0)
{
	float r0 = d * 0.5;
	int r = d / 2;
	d0.clear();
	int n = 0;
	for (int y = -r; y <= r; y++) {
		float x1 = sqrt(r0 * r0 - y * y) + r0;
		float x2 = r0 - sqrt(r0 * r0 - y * y);
		d0.push_back(Vec3i(y + r, x1 + 0.5, x2 + 0.5));
		n += d0.back()[1] - d0.back()[2];
	}
	return n;
}

void compute_grad(const Mat & img, Mat & grad)
{
	grad.create(img.rows, img.cols, CV_32SC2);

	for (int y = 0; y < img.rows; y++) {
		Vec2i * p_grad = grad.ptr<Vec2i>(y);
		for (int x = 0; x < img.cols; x++) {
			int sv = 0, sh = 0;
			for (int dy = -2; dy <= 2; dy++) {
				const uchar * p_img = (y + dy < 0) ? img.ptr<uchar>(0, x) : 
					(y + dy >= img.rows) ? img.ptr<uchar>(img.rows - 1, x) : img.ptr<uchar>(y + dy, x);
				const char * p_vert = Vert.ptr<char>(dy + 2, 2);
				const char * p_hori = Horizon.ptr<char>(dy + 2, 2);
				if (x - 2 < 0) {
					for (int dx = -2; dx <= 2; dx++) {
						int t = p_img[dx < 0 ? 0 : dx];
						sv += t * p_vert[dx];
						sh += t * p_hori[dx];
					}
				}
				else if (x + 2 >= img.cols) {
					for (int dx = -2; dx <= 2; dx++) {
						int t = p_img[dx > 0 ? img.cols - 1 : dx];
						sv += t * p_vert[dx];
						sh += t * p_hori[dx];
					}
				}
				else
				for (int dx = -2; dx <= 2; dx++) {
					int t = p_img[dx];
					sv += t * p_vert[dx];
					sh += t * p_hori[dx];
				}
			}
			p_grad[x] = Vec2i(sv, sh);
		}
	}
}

/*
Input img
Output grad, v[0] is vertical,
faster than compute_grad*/
void compute_grad_fast(const Mat & img, Mat & grad)
{
	Mat grad0(img.rows, img.cols, CV_32SC2);
	grad.create(img.rows, img.cols, CV_32SC2);
	for (int y = 0; y < img.rows; y++) {
		Vec2i * p_grad0 = grad0.ptr<Vec2i>(y);
		const uchar * p_img = img.ptr<uchar>(y);
		for (int x = 2; x < img.cols - 2; x++) {
			int a1 = 2 * p_img[x + 1] + p_img[x + 2];
			int a2 = 2 * p_img[x - 1] + p_img[x - 2];
			p_grad0[x] = Vec2i(a1 + a2 + 3 * p_img[x], a1 - a2);
		}
	}
	for (int y = 2; y < img.rows - 2; y++) {
		Vec2i * p_grad0_2 = grad0.ptr<Vec2i>(y - 2);
		Vec2i * p_grad0_1 = grad0.ptr<Vec2i>(y - 1);
		Vec2i * p_grad0 = grad0.ptr<Vec2i>(y);
		Vec2i * p_grad01 = grad0.ptr<Vec2i>(y + 1);
		Vec2i * p_grad02 = grad0.ptr<Vec2i>(y + 2);
		Vec2i * p_grad = grad.ptr<Vec2i>(y);
		for (int x = 2; x < img.cols - 2; x++) {
			int a1 = p_grad0[x][1] * 3 + (p_grad01[x][1] + p_grad0_1[x][1]) * 2 + p_grad02[x][1] + p_grad0_2[x][1];
			int a0 = (p_grad01[x][0] - p_grad0_1[x][0]) * 2 + p_grad02[x][0] - p_grad0_2[x][0];
			p_grad[x] = Vec2i(a0 / 27, a1 / 27);
		}
	}
	for (int y = 0; y < 2; y++) {
		Vec2i * p_grad = grad.ptr<Vec2i>(y);
		Vec2i * p_grad2 = grad.ptr<Vec2i>(2);
		for (int x = 2; x < img.cols - 2; x++)
			p_grad[x] = p_grad2[x];
	}
	for (int y = img.rows - 2; y < img.rows; y++) {
		Vec2i * p_grad = grad.ptr<Vec2i>(y);
		Vec2i * p_grad2 = grad.ptr<Vec2i>(img.rows - 3);
		for (int x = 2; x < img.cols - 2; x++)
			p_grad[x] = p_grad2[x];
	}
	for (int y = 0; y < img.rows; y++) {
		Vec2i * p_grad = grad.ptr<Vec2i>(y);
		p_grad[0] = p_grad[2];
		p_grad[1] = p_grad[2];
		p_grad[img.cols - 2] = p_grad[img.cols - 3];
		p_grad[img.cols - 1] = p_grad[img.cols - 3];
	}
}

void convert_element_obj(const vector<ElementObj *> & es, vector<MarkObj> & ms, int scale)
{
	ms.reserve(ms.size() + es.size());
	for (auto & eo : es) {
		MarkObj mo;
		mo.p0 = eo->p0 * scale;
		if (eo->type == OBJ_POINT)
			mo.p1 = eo->p1;
		else
			mo.p1 = eo->p1 * scale;
		mo.prob = eo->prob;
		mo.state = eo->state;
		mo.type = eo->type;
		mo.type2 = eo->type2;
		mo.type3 = eo->type3;
		ms.push_back(mo);
		delete eo;
	}
}

CvSVM * train_svm(Mat & train_data_svm, Mat & label, float weight, double cost, bool force_poly=false)
{
	CvSVM * ret_svm = new CvSVM;
	// Set up SVM's parameters
	CvSVMParams params;
	Mat class_wts(2, 1, CV_32FC1);
	CvMat class_wts_cv;
	class_wts.at<float>(0) = weight;
	class_wts.at<float>(1) = 1 - weight;
	class_wts_cv = class_wts;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::LINEAR;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	params.C = cost;
	params.class_weights = &class_wts_cv;
	int mistake1 = 0;
	if (!force_poly) {
		ret_svm->train(train_data_svm, label, Mat(), Mat(), params);
		qInfo("SVM:train linear, var_cnt=%d, sup_vec_cnt=%d", ret_svm->get_var_count(), ret_svm->get_support_vector_count());
		char s[600];
		for (int i = 0; i < ret_svm->get_support_vector_count(); i++) {
			const float * sv = ret_svm->get_support_vector(i);
			int k = 0;
			for (int j = 0; j < ret_svm->get_var_count(); j++)
				k += sprintf(&s[k], "%f,", sv[j]);
			qInfo(s);
		}

		for (int i = 0; i < train_data_svm.rows; i++) {
			float response = -ret_svm->predict(train_data_svm.row(i), true);
			if (response * label.at<float>(i) < 0)
				mistake1++;
			if (train_data_svm.cols == 5)
				qInfo("ViaML:train linear, is_via=%d, predict=%3f,d=%4f,s=%4f,g=%5.1f,%5.1f,%5.1f", (int)label.at<float>(i),
				response, train_data_svm.at<float>(i, 0), train_data_svm.at<float>(i, 1),
				train_data_svm.at<float>(i, 2), train_data_svm.at<float>(i, 3), train_data_svm.at<float>(i, 4));
			else
				qInfo("Edge:train linear, is_vwi=%d, predict=%3f,d=%5f,g=%5f", (int)label.at<float>(i),
				response, train_data_svm.at<float>(i, 0), train_data_svm.at<float>(i, 1));
		}
	}
	else
		mistake1 = 1000000;
	if (mistake1 > 0) {
		params.kernel_type = CvSVM::POLY;
		params.degree = 2;
		params.gamma = 1;
		params.coef0 = 1;
		CvSVM * ret_svm1 = new CvSVM;
		ret_svm1->train_auto(train_data_svm, label, Mat(), Mat(), params);
		qInfo("ViaML:train poly, var_cnt=%d, sup_vec_cnt=%d", ret_svm1->get_var_count(), ret_svm1->get_support_vector_count());
		char s[600];
		for (int i = 0; i < ret_svm1->get_support_vector_count(); i++) {
			const float * sv = ret_svm1->get_support_vector(i);
			int k = 0;
			for (int j = 0; j < ret_svm1->get_var_count(); j++)
				k += sprintf(&s[k], "%f,", sv[j]);
			qInfo(s);
		}
		int mistake2 = 0;
		for (int i = 0; i < train_data_svm.rows; i++) {
			float response = -ret_svm1->predict(train_data_svm.row(i), true);
			if (response * label.at<float>(i) < 0)
				mistake2++;
			if (train_data_svm.cols == 5)
				qInfo("ViaML:train poly, is_via=%d, predict=%f,d=%4f,s=%4f,g=%5.1f,%5.1f,%5.1f", (int)label.at<float>(i), 
				response, train_data_svm.at<float>(i, 0), train_data_svm.at<float>(i, 1), 
				train_data_svm.at<float>(i, 2),	train_data_svm.at<float>(i, 3), train_data_svm.at<float>(i, 4));
			else
				qInfo("Edge:train poly, is_vwi=%d, predict=%3f,d=%5f,g=%5f", (int)label.at<float>(i),
				response, train_data_svm.at<float>(i, 0), train_data_svm.at<float>(i, 1));
		}		
		if (mistake2 < mistake1) {
			mistake1 = mistake2;
			delete ret_svm;
			ret_svm = ret_svm1;
		}
		else
			delete ret_svm1;
	}
	qInfo("mistake = %d", mistake1);
	return ret_svm;
}

CvSVM * train_svm(vector<vector<int> > train_data_svm, vector<int> label, float weight, double cost, bool force_poly = false)
{
	CV_Assert(train_data_svm.size() == label.size());
	Mat train_m((int)label.size(), (int) train_data_svm[0].size(), CV_32FC1);
	Mat label_m((int)label.size(), 1, CV_32FC1);
	for (int i = 0; i < (int)label.size(); i++) {
		for (int j = 0; j < train_m.cols; j++)
			train_m.at<float>(i, j) = train_data_svm[i][j];
		label_m.at<float>(i) = label[i];
	}

	return train_svm(train_m, label_m, weight, cost, force_poly);
}

#define DARK_NUM		6
static const int dark_pick[8][DARK_NUM][2] = {
	//y , x
	{ { -1, 0 }, { -2, -1 }, { -2, 0 }, { -2, 1 }, { -2, -2 }, { -2, 2 } },
	{ { 0, 1 }, { 1, 2 }, { 0, 2 }, { -1, 2 }, { -2, 2 }, { 2, 2 } },
	{ { 1, 0 }, { 2, -1 }, { 2, 0 }, { 2, 1 }, { 2, -2 }, { 2, 2 } },
	{ { 0, -1 }, { 1, -2 }, { 0, -2 }, { -1, -2 }, { -2, -2 }, { 2, -2 } },
	{ { -1, 1 }, { -1, 2 }, { -2, 1 }, { -2, 2 }, { -2, 0 }, { 0, 2 } },
	{ { 1, 1 }, { 1, 2 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 2, 0 } },
	{ { 1, -1 }, { 1, -2 }, { 2, -1 }, { 2, -2 }, { 2, 0 }, { 0, -2 } },
	{ { -1, -1 }, { -1, -2 }, { -2, -1 }, { -2, -2 }, { 0, -2 }, { -2, 0 } }
};

EdgeFeatureML::EdgeFeatureML()
{
	is_valid = false;
}

/*
Input image raw image
Input grad, raw image grad
Input o, edge location
output e: e[0] is gray level, e[1] is grad in dir, e[2] is dir
inout: dir, if dir<0, find max grad dir, else  search specified dir.
*/
void EdgeFeatureML::get_feature_vector(const Mat & img, const Mat &grad, Point o, int e[], int & dir)
{
	CV_Assert(o.x >= 4 && o.y >= 4 && o.x < img.cols - 4 && o.y < img.rows - 4);
	const Vec2i * p_grad = grad.ptr<Vec2i>(o.y, o.x);
	int gv = p_grad[0][0];
	int gh = p_grad[0][1];
	int g1 = gv * 0.717 + gh * 0.717;
	int g2 = gv * 0.717 - gh * 0.717;
	int max_g = max(max(abs(gv), abs(gh)), max(abs(g1), abs(g2)));

	if (dir < 0) { //if dir is not specified, find max grad as dir
		if (max_g == abs(gv))
			dir = (gv > 0) ? DIR_UP : DIR_DOWN; //DIR_UP means bottom is wire
		else
		if (max_g == abs(gh))
			dir = (gh > 0) ? DIR_LEFT : DIR_RIGHT; //DIR_LEFT means right is wire
		else
		if (max_g == abs(g1))
			dir = (g1 > 0) ? DIR_UPLEFT : DIR_DOWNRIGHT;
		else
			dir = (g2 > 0) ? DIR_UPRIGHT : DIR_DOWNLEFT;
	}
	//compute insu gray
	uchar dark[4]; //all 4 is insu, 
	e[0] = 0; //e[0] is sum of 3 low dark
	uchar max_dark = 0; //most bright insu,
	uchar min_dark = 255; //most dark insu
	for (int i = 0; i < DARK_NUM; i++) {
		dark[i] = img.at<uchar>(o.y + dark_pick[dir][i][0], o.x + dark_pick[dir][i][1]);
		e[0] += dark[i];
		max_dark = max(max_dark, dark[i]);
		min_dark = min(min_dark, dark[i]);
	}
	e[0] = (e[0] - max_dark - min_dark) / (DARK_NUM - 2); // /3 range is [0.255}, /4 is compress range to [0..191]
	const Vec2i * p_grad1 = grad.ptr<Vec2i>(o.y + dxy[dir_2[dir]][0], o.x + dxy[dir_2[dir]][1]); //acc in edge direction
	const Vec2i * p_grad2 = grad.ptr<Vec2i>(o.y + dxy[dir_1[dir_2[dir]]][0], o.x + dxy[dir_1[dir_2[dir]]][1]); //acc in edge direction
	o.y += dxy[dir][0];
	o.x += dxy[dir][1];
	const Vec2i * p_grad3 = grad.ptr<Vec2i>(o.y, o.x);
	const Vec2i * p_grad4 = grad.ptr<Vec2i>(o.y + dxy[dir_2[dir]][0], o.x + dxy[dir_2[dir]][1]); //acc in edge direction
	const Vec2i * p_grad5 = grad.ptr<Vec2i>(o.y + dxy[dir_1[dir_2[dir]]][0], o.x + dxy[dir_1[dir_2[dir]]][1]); //acc in edge direction
	switch (dir) {
	case DIR_UP:
	case DIR_DOWN:
		e[1] = (p_grad[0][0] + p_grad1[0][0] + p_grad2[0][0] + p_grad3[0][0] + p_grad4[0][0] + p_grad5[0][0]) / 6;
		break;
	case DIR_LEFT:
	case DIR_RIGHT:
		e[1] = (p_grad[0][1] + p_grad1[0][1] + p_grad2[0][1] + p_grad3[0][1] + p_grad4[0][1] + p_grad5[0][1]) / 6;
		break;
	case DIR_UPLEFT:
	case DIR_DOWNRIGHT:
		e[1] = (p_grad[0][0] + p_grad1[0][0] + p_grad2[0][0] + p_grad3[0][0] + p_grad4[0][0] + p_grad5[0][0]
			+ p_grad[0][1] + p_grad1[0][1] + p_grad2[0][1] + p_grad3[0][1] + p_grad4[0][1] + p_grad5[0][1]) * 0.11785;
		break;
	default:
		e[1] = (p_grad[0][0] + p_grad1[0][0] + p_grad2[0][0] + p_grad3[0][0] + p_grad4[0][0] + p_grad5[0][0]
			- p_grad[0][1] - p_grad1[0][1] - p_grad2[0][1] - p_grad3[0][1] - p_grad4[0][1] - p_grad5[0][1]) * 0.11785;
		break;
	}
	e[1] = abs(e[1]);
	CV_Assert(e[0] < 256 && e[1] < 256);
	e[2] = dir;
}

bool EdgeFeatureML::feature_extract(const Mat & img, Point & org, Point & range, int label, vector<vector<int> > & features)
{
	Mat grad;
	compute_grad_fast(img, grad);
	features.clear();
	int x0 = max(4, org.x - range.x);
	int x1 = min(img.cols - 5, org.x + range.x);
	int y0 = max(4, org.y - range.y);
	int y1 = min(img.rows - 5, org.y + range.y);
	int max_grad = 0;
	//now find point with max grad
	int e[EdgeFeatureVecLen + 1];
	for (int y = y0; y <= y1; y++)
	for (int x = x0; x <= x1; x++) {
		int dir = -1;
		get_feature_vector(img, grad, Point(x, y), e, dir);
		if (e[1] > max_grad) {
			max_grad = e[1];
			org = Point(x, y);
		}
	}
	Point org1 = org;
	if (label & EDGE_IS_WIRE_INSU) {
		if (max_grad < MINIMUM_EDGE_GRAD)
			return false;
		int dir = -1;
		get_feature_vector(img, grad, org1, e, dir);
		e[EdgeFeatureVecLen] = label;
		features.push_back(vector<int>(e, e + EdgeFeatureVecLen + 1)); //push edge wire-insu
		org1.y += dxy[dir][0] * 2;
		org1.x += dxy[dir][1] * 2;
		int dir1 = dir_2[dir]; //use edge dir instead of grad dir
		get_feature_vector(img, grad, org1, e, dir1);
		if (e[0] + e[1] + MINIMUM_EDGE_GRAD < features.back()[0] + features.back()[1]) {
			e[EdgeFeatureVecLen] = EDGE_FEATURE | EDGE_IS_INSU | (label & EDGE_NEAR_VIA);
			features.push_back(vector<int>(e, e + EdgeFeatureVecLen + 1)); //push edge insu
		}
		org1.y -= dxy[dir][0] * 5;
		org1.x -= dxy[dir][1] * 5;
		dir1 = dir_2[dir]; //use edge dir instead of grad dir
		get_feature_vector(img, grad, org1, e, dir1);
		range = org1;
		e[EdgeFeatureVecLen] = EDGE_FEATURE | EDGE_IS_WIRE | (label & EDGE_NEAR_VIA); //push wire
		features.push_back(vector<int>(e, e + EdgeFeatureVecLen + 1)); //push edge insu
	}
	else
	if (label & EDGE_IS_INSU) {
		int dir = -1;
		get_feature_vector(img, grad, org1, e, dir);
		e[EdgeFeatureVecLen] = label;
		features.push_back(vector<int>(e, e + EdgeFeatureVecLen + 1)); //push edge insu
	}
	else
	if (label & EDGE_IS_WIRE) {
		int dir = -1;
		get_feature_vector(img, grad, org1, e, dir);
		e[EdgeFeatureVecLen] = label;
		features.push_back(vector<int>(e, e + EdgeFeatureVecLen + 1)); //push edge wire
	}
	qInfo("Edge feature_extract %d", label & 7);
	return true;
}

/*
Input low: sorted from big to small
Input high: sorted from small to big
Output: pdf, probability distributed function
Inout: tran_zone, tran_zone.start and .end is for transition zone
Return best seperate point
*/
int compute_pdf(const vector<int> & low0, const vector<int> & high0, vector<int> & pdf, Range &tran_zone)
{
	vector<int> low(low0), high(high0);
	bool change = true;
	while (change && !high.empty() && !low.empty()) {
		float avg = 0;
		change = false;
		for (auto a : high)
			avg += a;
		avg /= high.size();
		if (low[0] > avg) { //low[0] is too big, drop
			low.erase(low.begin());
			change = true;
		}
		avg = 0;
		for (auto a : low)
			avg += a;
		avg /= low.size();
		if (high[0] < avg) { //high[0] is too small, drop
			high.erase(high.begin());
			change = true;
		}
	}
	if (high.size() < 2 || low.size() < 2)
		return -1;
	
	int min_s = 100000, best_start, best_end;
	int margin = tran_zone.start;
	tran_zone = Range(0, pdf.size() - 1);
	for (int i = 0; i < (int)pdf.size(); i++) {
		int s = 0; //s is wrong number
		for (int j = 0; j < (int)low.size(); j++)
		if (low[j] >= i) //acc low number >= i
			s++;
		else
			break;
		for (int j = 0; j < (int)high.size(); j++)
		if (high[j] <= i) //acc low number <= i
			s++;
		else
			break;
		if (min_s > s) { //best_start and best_end is location with min wrong number={(low number > location) + (high number < location)}
			min_s = s;
			best_start = i;
			best_end = i;
		} 
		else
		if (min_s == s)
			best_end = i;
	}
	int best_sep = (best_start + best_end) / 2; //best_start and best_end has same wrong number
	margin = min(margin, best_sep / 2);
	for (int i = 0; i < pdf.size(); i++)
		pdf[i] = (i < best_sep) ? 0 : (i == best_sep) ? POINT_TOT_PROB / 2 : POINT_TOT_PROB;
	for (int i = best_sep; i < (int)pdf.size(); i++) {
		if (low[0] >= i || i==best_sep) {
			pdf[i] = POINT_TOT_PROB / 2;
			for (int j = i + 1; j < min(i + margin, (int)pdf.size()); j++) //extend to margin
				pdf[j] = POINT_TOT_PROB / 2 + POINT_TOT_PROB * (j - i) / (2 * margin);
			continue;
		}
		int s = 0, s1 = 0; //s is low number which is higher than i - margin
		for (int j = 0; j < (int)low.size(); j++)
		if (low[j] + margin >= i)
			s++;
		else
			break;
		if (s == 0) {
			tran_zone.end = i;
			break; //above already extend margin, quit directly
		}
		for (int j = 0; j < (int)high.size(); j++)
		if (high[j] < i)
			s1++;
		else
			break;
		s1 = high.size() - s1; //s1 is high number which is higher than i
		if (s1 == 0) {
			tran_zone.end = i;
			break; //above already extend margin, quit directly
		}
		int v = POINT_TOT_PROB - POINT_TOT_PROB * s * high.size() / (s1 * low.size()); //v is (low number > location) / (high number > location)
		if (v < pdf[i - 1])
			v = pdf[i - 1];
		pdf[i] = min(v, pdf[i]);
	}
	for (int i = best_sep; i >= 0; i--) {
		if (high[0] <= i || i == best_sep) {
			pdf[i] = POINT_TOT_PROB / 2;
			for (int j = i - 1; j > max(0, i - margin); j--) //extend to margin
				pdf[j] = POINT_TOT_PROB / 2 - POINT_TOT_PROB * (i - j) / (2 * margin);
			continue;
		}
		int s = 0, s1 = 0; //s is hign number which is lower than i + margin
		for (int j = 0; j < (int)high.size(); j++)
		if (high[j] <= i + margin)
			s++;
		else
			break;
		if (s == 0) {
			tran_zone.start = i;
			break;  //above already extend margin, quit directly
		}
		for (int j = 0; j < (int)low.size(); j++)
		if (low[j] > i)
			s1++;
		else
			break; 
		s1 = low.size() - s1; //s1 is low number which is lower than i
		if (s1 == 0) {
			tran_zone.start = i;
			break;  //above already extend margin, quit directly
		}
		int v = POINT_TOT_PROB * s * low.size() / (s1 * high.size());
		if (v > pdf[i + 1])
			v = pdf[i + 1];
		pdf[i] = max(v, pdf[i]);
	}
	for (int i = 0; i < (int)pdf.size() - 1; i++)
		CV_Assert(pdf[i] <= pdf[i + 1]);
	return best_sep;
}

bool EdgeFeatureML::train(const vector<vector<int> > & features) 
{
	int wire_num = 0;
	int wire_bright_max = 0;

	//1 prepare insu vector and train insu svm
	vector<int> insu_dark;
	vector<int> wire_dark;
	vector<Point> edge_point;
	vector<Point> wi_point;
	for (auto & v : features) {
		if (v[EdgeFeatureVecLen] & EDGE_IS_WIRE) {
			wire_dark.push_back(v[0]);
			wi_point.push_back(Point(v[0], v[1]));
		}
		else
		if (v[EdgeFeatureVecLen] & EDGE_IS_INSU) {
			insu_dark.push_back(v[0]);
			wi_point.push_back(Point(v[0], v[1]));
		}
		else
		if (v[EdgeFeatureVecLen] & EDGE_IS_WIRE_INSU)
			edge_point.push_back(Point(v[0], v[1])); //v[0] is dark, v[1] is grad
	}
	if (wire_dark.empty() || insu_dark.empty() || edge_point.empty()) {
		is_valid = false;
		return false;
	}
	//2 compute insu and wire pdf
	sort(wire_dark.begin(), wire_dark.end()); //dark for wire
	sort(insu_dark.begin(), insu_dark.end(), greater<int>());

	vector<int> wire_pdf(MAX_DARK_LEVEL);
	Range wi_range(MIN_DARK_GAP, MIN_DARK_GAP);
	int dark_sep = compute_pdf(insu_dark, wire_dark, wire_pdf, wi_range);
	qInfo("train, dark_sep=%d, insu_high=%d, wire_low=%d", dark_sep, wi_range.start, wi_range.end);
	if (dark_sep < 0){
		is_valid = false;
		return false;
	}

	//3 compute edge pdf for each dark level
	vector<vector<int> > edge_pdf; //edge_pdf is pdf for different tp, edge_pdf[0] is for tp[0]
	vector<int> tp; //tp is dark level
	if (edge_point.size() <= TRAIN_NUM_TH) {		
		vector<int> wi_grad;
		vector<int> edge_grad;
		for (auto & v : features) {
			if (v[EdgeFeatureVecLen] & EDGE_IS_WIRE)
				wi_grad.push_back(v[1]);
			else
			if (v[EdgeFeatureVecLen] & EDGE_IS_INSU)
				wi_grad.push_back(v[1]);
			else
			if (v[EdgeFeatureVecLen] & EDGE_IS_WIRE_INSU)
				edge_grad.push_back(v[1]);
		}
		int x0 = 0;
		for (auto & e : edge_point)
			x0 += e.x;
		x0 /= edge_point.size();
		tp.push_back(x0);
		sort(edge_grad.begin(), edge_grad.end());
		sort(wi_grad.begin(), wi_grad.end(), greater<int>());
		vector<int> temp_pdf(MAX_GRAD_LEVEL);
		Range temp(EDGE_GRAD_GAP, EDGE_GRAD_GAP);
		int edge_sep = compute_pdf(wi_grad, edge_grad, temp_pdf, temp);
		if (edge_sep < 0){
			is_valid = false;
			return false;
		}
		edge_pdf.push_back(temp_pdf);
		qInfo("train, edge_sep=%d, edge_high=%d, edge_low=%d", edge_sep, temp.start, temp.end);
	}
	else {
		sort(wi_point.begin(), wi_point.end(), lessPoint2);
		sort(edge_point.begin(), edge_point.end(), lessPoint2);
		vector<int> wi_point_acc_x; //wi_point_acc_x is accumulate of wi_point x
		int acc_temp = 0;
		for (int i = 0; i < (int)wi_point.size(); i++) {
			wi_point_acc_x.push_back(acc_temp);
			acc_temp += wi_point[i].x;
		}
		for (int i = 0; i < (int) edge_point.size() - TRAIN_NUM_TH + 1; i++) {
			vector<int> wi_grad;
			vector<int> edge_grad;
			int x0 = 0;
			for (int j = i; j < i + TRAIN_NUM_TH; j++) {
				edge_grad.push_back(edge_point[j].y);
				x0 += edge_point[j].x;
			}
			int best_diff = 100000, best_idx; //best_idx is best location for wi_point
			for (int j = 0; j < wi_point_acc_x.size() - TRAIN_NUM_TH; j++) {
				int x = wi_point_acc_x[j + TRAIN_NUM_TH] - wi_point_acc_x[j];
				if (abs(x - x0) < best_diff) {
					best_diff = abs(x - x0);
					best_idx = j;
				}
			}
			for (int j = best_idx; j < best_idx + TRAIN_NUM_TH; j++)
				wi_grad.push_back(wi_point[j].y);
			sort(edge_grad.begin(), edge_grad.end()); 
			sort(wi_grad.begin(), wi_grad.end(), greater<int>());
			vector<int> temp_pdf(MAX_GRAD_LEVEL);
			Range temp(EDGE_GRAD_GAP, EDGE_GRAD_GAP);
			int edge_sep = compute_pdf(wi_grad, edge_grad, temp_pdf, temp);
			if (edge_sep < 0){
				is_valid = false;
				return false;
			}
			edge_pdf.push_back(temp_pdf);
			tp.push_back(x0 / TRAIN_NUM_TH);
			qInfo("train, tp=%d,edge_sep=%d, edge_high=%d, edge_low=%d", tp.back(), edge_sep, temp.start, temp.end);
		}
	}

	//4 combine wire_pdf and edge_pdf with linear interpolate
	memset(wi_prob, 0, sizeof(wi_prob)); 
	for (int g = 0; g < MAX_GRAD_LEVEL; g++) {
		for (int dark = 0, j = 0; dark < MAX_DARK_LEVEL; dark++) {
			int wi_tot;
			if (dark >= wi_range.end + MIN_DARK_GAP - 1)
				wi_tot = 0;
			else
			if (j >= tp.size()) {
				int x1 = wi_range.end + MIN_DARK_GAP - 1 - dark;
				wi_tot = edge_pdf.back()[g] * x1 / (wi_range.end + MIN_DARK_GAP - 1 - tp.back());
			}
			else
			if (dark < tp[0])
				wi_tot = edge_pdf[0][g];
			else
			if (dark == tp[j]) {
				wi_tot = edge_pdf[j][g];
				j++;				
			}
			else 
				wi_tot = (edge_pdf[j][g] * (dark - tp[j - 1]) + edge_pdf[j - 1][g] * (tp[j] - dark)) / (tp[j] - tp[j - 1]);
			
			wi_tot = POINT_TOT_PROB - wi_tot;			
			wi_prob[dark][g][POINT_IS_WIRE] = wi_tot * wire_pdf[dark] / POINT_TOT_PROB;
			wi_prob[dark][g][POINT_IS_INSU] = wi_tot - wi_prob[dark][g][POINT_IS_WIRE];
		}
	}
	is_valid = true;
	return true;
}

void EdgeFeatureML::judge(const Mat & img, Mat & grad, Mat & prob, const Mat & via_mark, Mat & debug_mark)
{
	CV_Assert(img.type() == CV_8UC1);
	if (grad.empty())
		compute_grad_fast(img, grad);
	prob.create(img.rows, img.cols, CV_8UC4);
	int shift[8][15];
	for (int dir = 0; dir < 8; dir++) {
		for (int i = 0; i < DARK_NUM; i++)
			shift[dir][i] = (dark_pick[dir][i][0] * (int)img.step.p[0] + dark_pick[dir][i][1] * (int)img.step.p[1]) / sizeof(uchar);
		int dir2 = dir_2[dir];
		int dir3 = dir_1[dir_2[dir]];
		shift[dir][10] = (dxy[dir2][0] * (int)grad.step.p[0] + dxy[dir2][1] * (int)grad.step.p[1]) / sizeof(Vec2i);
		shift[dir][11] = (dxy[dir3][0] * (int)grad.step.p[0] + dxy[dir3][1] * (int)grad.step.p[1]) / sizeof(Vec2i);
		shift[dir][12] = (dxy[dir][0] * (int)grad.step.p[0] + dxy[dir][1] * (int)grad.step.p[1]) / sizeof(Vec2i);
		shift[dir][13] = shift[dir][10] + shift[dir][12];
		shift[dir][14] = shift[dir][11] + shift[dir][12];
	}
	for (int y = EDGE_JUDGE_BORDER; y < img.rows - EDGE_JUDGE_BORDER; y++) {
		const Vec2i * p_grad = grad.ptr<Vec2i>(y);
		const uchar * p_img = img.ptr<uchar>(y);
		Vec4b * p_prob = prob.ptr<Vec4b>(y);
		const uchar * p_mask = via_mark.empty() ? NULL : via_mark.ptr<uchar>(y);
		for (int x = EDGE_JUDGE_BORDER; x < img.cols - EDGE_JUDGE_BORDER; x++)
		if (!p_mask || p_mask[x] == VIA_MARK_NO_VIA) {
			//following is same as get_feature_vector
			int dir;
			const Vec2i * p_gradx = p_grad + x;
			int gv = p_gradx[0][0];
			int gh = p_gradx[0][1];
			int g1 = gv * 0.717 + gh * 0.717;
			int g2 = gv * 0.717 - gh * 0.717;
			int max_g = max(max(abs(gv), abs(gh)), max(abs(g1), abs(g2)));
			if (max_g == abs(gv))
				dir = (gv > 0) ? DIR_UP : DIR_DOWN; //DIR_UP means bottom is wire
			else
			if (max_g == abs(gh))
				dir = (gh > 0) ? DIR_LEFT : DIR_RIGHT;
			else
			if (max_g == abs(g1))
				dir = (g1 > 0) ? DIR_UPLEFT : DIR_DOWNRIGHT;
			else
				dir = (g2 > 0) ? DIR_UPRIGHT : DIR_DOWNLEFT;

			int e[EdgeFeatureVecLen] = { 0, 0, dir };
			uchar max_dark = 0; //most bright insu,
			uchar min_dark = 255; //most dark insu
			for (int i = 0; i < DARK_NUM; i++) {
				uchar d = p_img[x + shift[dir][i]];
				e[0] += d;
				max_dark = max(max_dark, d);
				min_dark = min(min_dark, d);
			}
			e[0] = (e[0] - max_dark - min_dark) / (DARK_NUM - 2);
			switch (dir) {
			case DIR_UP:
			case DIR_DOWN:
				e[1] = (p_gradx[0][0] + p_gradx[shift[dir][10]][0] + p_gradx[shift[dir][11]][0] +
					p_gradx[shift[dir][12]][0] + p_gradx[shift[dir][13]][0] + p_gradx[shift[dir][14]][0]) / 6;
				break;
			case DIR_LEFT:
			case DIR_RIGHT:
				e[1] = (p_gradx[0][1] + p_gradx[shift[dir][10]][1] + p_gradx[shift[dir][11]][1] + 
					p_gradx[shift[dir][12]][1] + p_gradx[shift[dir][13]][1] + p_gradx[shift[dir][14]][1]) / 6;
				break;
			case DIR_UPLEFT:
			case DIR_DOWNRIGHT:
				e[1] = (p_gradx[0][0] + p_gradx[shift[dir][10]][0] + p_gradx[shift[dir][11]][0] +
					p_gradx[shift[dir][12]][0] + p_gradx[shift[dir][13]][0] + p_gradx[shift[dir][14]][0] +
					p_gradx[0][1] + p_gradx[shift[dir][10]][1] + p_gradx[shift[dir][11]][1] +
					p_gradx[shift[dir][12]][1] + p_gradx[shift[dir][13]][1] + p_gradx[shift[dir][14]][1]) * 0.11785;
				break;
			default:
				e[1] = (p_gradx[0][0] + p_gradx[shift[dir][10]][0] + p_gradx[shift[dir][11]][0] +
					p_gradx[shift[dir][12]][0] + p_gradx[shift[dir][13]][0] + p_gradx[shift[dir][14]][0] -
					p_gradx[0][1] - p_gradx[shift[dir][10]][1] - p_gradx[shift[dir][11]][1] -
					p_gradx[shift[dir][12]][1] - p_gradx[shift[dir][13]][1] - p_gradx[shift[dir][14]][1]) * 0.11785;
				break;
			}
			e[1] = abs(e[1]);
			if (e[0] >= MAX_DARK_LEVEL) {
				p_prob[x][POINT_IS_INSU] = 0;
				p_prob[x][POINT_IS_WIRE] = POINT_TOT_PROB;
			} else
			if (e[1] >= MAX_GRAD_LEVEL) {
				p_prob[x][POINT_IS_INSU] = 0;
				p_prob[x][POINT_IS_WIRE] = 0;
			}
			else {
				p_prob[x][POINT_IS_INSU] = wi_prob[e[0]][e[1]][POINT_IS_INSU]; //compute prob based on wi_prob table
				p_prob[x][POINT_IS_WIRE] = wi_prob[e[0]][e[1]][POINT_IS_WIRE];
			}
			p_prob[x][POINT_IS_EDGE_WIRE_INSU] = POINT_TOT_PROB - p_prob[x][POINT_IS_INSU] - p_prob[x][POINT_IS_WIRE];
			p_prob[x][POINT_DIR] = dir;
		}
	}
	for (int y = 0; y < EDGE_JUDGE_BORDER; y++) {
		Vec4b * p_prob = prob.ptr<Vec4b>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			p_prob[x][POINT_IS_INSU] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_WIRE] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_EDGE_WIRE_INSU] = 0;
			p_prob[x][POINT_DIR] = 64;
		}
	}
	for (int y = img.rows - EDGE_JUDGE_BORDER; y < img.rows; y++) {
		Vec4b * p_prob = prob.ptr<Vec4b>(y);
		for (int x = EDGE_JUDGE_BORDER; x < prob.cols - EDGE_JUDGE_BORDER; x++) {
			p_prob[x][POINT_IS_INSU] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_WIRE] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_EDGE_WIRE_INSU] = 0;
			p_prob[x][POINT_DIR] = 64;
		}
	}
	for (int y = 0; y < img.rows; y++) {
		Vec4b * p_prob = prob.ptr<Vec4b>(y);
		for (int x = 0; x < EDGE_JUDGE_BORDER; x++) {
			p_prob[x][POINT_IS_INSU] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_WIRE] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_EDGE_WIRE_INSU] = 0;
			p_prob[x][POINT_DIR] = 64;
		}
		for (int x = img.cols - EDGE_JUDGE_BORDER; x < img.cols; x++) {
			p_prob[x][POINT_IS_INSU] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_WIRE] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_EDGE_WIRE_INSU] = 0;
			p_prob[x][POINT_DIR] = 64;
		}
	}
}


class ViaFeature2Vector {
public:
	Mat train_vec;
	vector<int> train_label;
	bool has_via[2];
public:
	ViaFeature2Vector() {
		has_via[0] = has_via[1] = false;
	}
	Mat compute_via_vec(const Mat & feature, bool training = false) {
		CV_Assert(feature.cols == 8 && feature.rows == FEATURE_ROW);
		vector<int> g;
		for (int j = 0; j < 8; j++)
			g.push_back(feature.at<int>(1, j));
		sort(g.begin(), g.end());
		int s = feature.at<int>(0, 0) & VIA_IS_VIA;
		int d1 = feature.at<int>(0, 2);
		int d2 = feature.at<int>(0, 3);
		float diameter = (d1 + d2) * 5;
		float sum = feature.at<int>(0, 4);
		has_via[s] = true;
		Mat sample = (Mat_<float>(1, 5) << diameter, sum * 4, g[0] * 0.1, (g[1] - g[0]) * 0.1, (g[2] - g[1]) * 0.1);
		if (training) {
			train_vec.push_back(sample);
			train_label.push_back(s);
		}
		return sample;
	}

	void get_addition_vec(Mat & train_data_svm, Mat & label) {
		if (!has_via[1])
			return;
		CV_Assert(train_vec.rows ==(int) train_label.size());
		if (!has_via[0]) {
			Mat t = train_vec.clone();
			Mat l(train_vec.rows, 1, CV_32FC1);
			for (int i = 0; i < t.rows; i++) { //make sum and grad reduce for non-via training
				t.at<float>(i, 1) = t.at<float>(i, 1) / 2;
				t.at<float>(i, 2) = t.at<float>(i, 2) / 4;
				t.at<float>(i, 3) = t.at<float>(i, 3) / 4;
				t.at<float>(i, 4) = t.at<float>(i, 4) / 4;
				l.at<float>(i, 0) = -1;
			}
			train_data_svm.push_back(t);
			label.push_back(l);
		}
		else {
			bool need_adjust = true;
			for (int i = 0; i < train_vec.rows; i++) 
			if (train_label[i] && train_vec.at<float>(i, 2) < 100)
				need_adjust = false;
			if (need_adjust) {
				Mat sample(1, 5, CV_32F);
				for (int i = 0; i < train_vec.rows; i++)
				if (!train_label[i] && train_vec.at<float>(i, 2) < 0) { //if g[0]<0,make g[0] as 0
					train_vec.row(i).copyTo(sample);
					float temp = sample.at<float>(0, 3) + sample.at<float>(0, 2);
					sample.at<float>(0, 3) = max(temp, 0.0f);
					sample.at<float>(0, 2) = 0;
					train_data_svm.push_back(sample);
					Mat l(1, 1, CV_32F);
					l.at<float>(0, 0) = -1;
					label.push_back(l);
				}
			}
		}
	}
};
static QMutex circle_mutex;

ViaML::ViaML()
{
	dir_remap[0] = DIR_RIGHT;
	dir_remap[1] = DIR_UPRIGHT;
	dir_remap[2] = DIR_UP;
	dir_remap[3] = DIR_UPLEFT;
	dir_remap[4] = DIR_LEFT;
	dir_remap[5] = DIR_DOWNLEFT;
	dir_remap[6] = DIR_DOWN;
	dir_remap[7] = DIR_DOWNRIGHT;
	is_valid = false;
}

ViaML::~ViaML()
{
}

void ViaML::find_best_bright(const Mat & lg, int d, int sep0, vector<Point> & loc, bool multi_thread)
{
	auto d0it = d0s.find(d);
	if (d0it == d0s.end()) {
		QMutexLocker locker(&circle_mutex);
		vector<Vec3i> dd;
		compute_circle_dx1(d, dd);
		d0it = d0s.find(d);
		if (d0it == d0s.end()) {
			d0s[d] = dd;
			d0it = d0s.find(d);
		}
	}
	vector<Vec3i> & dd = d0it->second;
	Mat sum(lg.rows, lg.cols, CV_32SC1, Scalar::all(0x20000000)); //circle gray sum
	//compute sum
	for (int y0 = sep0; y0 < lg.rows - d - sep0; y0++)
	for (int x0 = sep0; x0 < lg.cols - d - sep0 - 1; x0++) {
		int s = 0;
		for (int i = 0; i < (int)dd.size(); i++) {
			int y = dd[i][0];
			int x1 = dd[i][1];
			int x2 = dd[i][2];
			s += lg.at<int>(y + y0, x0 + x1 + 1) - lg.at<int>(y + y0, x0 + x2); //sum from x0-x to x0+x
		}
		sum.at<int>(y0, x0) = s;
	}
	loc.clear();
	//choose local maximum sum
	for (int y0 = 1; y0 < lg.rows - d - 1; y0++) {
		int * p_sum = sum.ptr<int>(y0);
		int * p_sum1 = sum.ptr<int>(y0 + 1);
		int * p_sum_1 = sum.ptr<int>(y0 - 1);
		for (int x0 = 1; x0 < lg.cols - d - 2; x0++) {
			if (p_sum[x0] > p_sum[x0 + 1] && p_sum[x0] >= p_sum[x0 - 1] &&
				p_sum[x0] > p_sum1[x0 + 1] && p_sum[x0] >= p_sum1[x0 - 1] && p_sum[x0] >= p_sum1[x0] &&
				p_sum[x0] > p_sum_1[x0 + 1] && p_sum[x0] >= p_sum_1[x0 - 1] && p_sum[x0] > p_sum_1[x0])
				loc.push_back(Point(x0, y0));
		}
	}
}

bool greaterPoint3i(const Point3i & a, const Point3i & b) { return a.z > b.z; }

int ViaML::compute_feature(const Mat & img, const Mat & lg, const Mat & grad, int d, const vector<Point> & loc, Mat features[], bool multi_thread, int score_min)
{
	CV_Assert(grad.type() == CV_32SC2 && img.type() == CV_8UC1 && d > 5);
	int score0 = score_min, score1 = score_min;
	features[0].release();
	features[1].release();
	for (auto o : loc) {
		int best_score = score_min;
		Mat feature(FEATURE_ROW, 8, CV_32S);
		for (int dir = 0; dir < 5; dir++) {
			Point org = o;
			if (dir < 4) {
				org.x += dxy[dir][1];
				org.y += dxy[dir][0];
			}
			const Vec2i *p_grad = grad.ptr<Vec2i>(org.y, org.x);
			for (int d1 = d - 1; d1 <= d + 1; d1++)
			for (int d2 = d - 1; d2 <= d + 1; d2++) { //search all shape, d1 for x, d2 for y
				float r1 = d1 * 0.5f;
				float r2 = d2 * 0.5f;
				auto circle_it = circles.find(grad.step.p[0] * 10000 + d1 * 100 + d2);
				Point2f tl;
				if (circle_it == circles.end()) {
					QMutexLocker locker(&circle_mutex);
					//following compute circle(d1, d2) and d0s(d1, d2)
					vector<vector<Vec3i> > cir;
					vector<Point2f> ef[8];
					float minx = 10000, miny = 10000;
					for (int i = 0; i < 8; i++)
					for (float beta = M_PI_4 * (i - 0.6); beta <= M_PI_4 * (i + 0.6); beta += 0.05) { //compute octagon's edge point
						ef[i].push_back(Point2f(r1 * cosf(beta), -r2 * sinf(beta)));
						minx = min(minx, ef[i].back().x);
						miny = min(miny, ef[i].back().y);
					}
					tl = Point2f(minx - 0.5, miny - 0.5);
					vector<Vec3i> xy(d2 + 1);
					for (int i = 0; i <= d2; i++) {
						xy[i][0] = i;
						xy[i][1] = 0;
						xy[i][2] = 1000;
					}
					for (int i = 0; i < 8; i++) {
						vector<Vec3i> edge;
						Point prev_edge_point(0, 0);
						for (auto edge_point : ef[i]) { //shift one edge
                            Point2f epf(edge_point - tl);
                            Point ep((int) epf.x, (int)epf.y);
							if (prev_edge_point != ep) {
								Vec3i v3; //v3[0] is grad y, v3[1] is grad x, v3[2] is shift related to top left
								v3[0] = -100 * edge_point.y / r1;
								v3[1] = -100 * edge_point.x / r2;
								prev_edge_point = ep;
								v3[2] = ((prev_edge_point.y * (int)grad.step.p[0] + prev_edge_point.x * (int)grad.step.p[1]) / sizeof(Vec2i));
								edge.push_back(v3);
								CV_Assert(ep.y < xy.size());
								xy[ep.y][1] = max(xy[ep.y][1], ep.x);
								xy[ep.y][2] = min(xy[ep.y][2], ep.x);
							}
						}
						cir.push_back(edge);
					}
					circle_it = circles.find(grad.step.p[0] * 10000 + d1 * 100 + d2);
					if (circle_it == circles.end()) {
						circles[grad.step.p[0] * 10000 + d1 * 100 + d2] = cir;
						d0s[d1 * 100 + d2] = xy;
						circle_it = circles.find(grad.step.p[0] * 10000 + d1 * 100 + d2);
					}
				}
				vector<vector<Vec3i> > & cir = circle_it->second; //cir[i] is one edge, cir[i][j] is one point
				int dd = max(d1, d2);
				float rr = dd * 0.5 + 1;
				auto circle_edges_it = circle_edges.find(dd);

				if (circle_edges_it == circle_edges.end()) {
					//following compute circle_edges outside
					CV_Assert(!multi_thread);
					vector<vector<Point3i> > vec(FEATURE_ROW - 2);
					for (int y = -FEATURE_ROW - dd / 2 - 2; y <= FEATURE_ROW + dd / 2 + 2; y++) 
					for (int x = -FEATURE_ROW - dd / 2 - 2; x <= FEATURE_ROW + dd / 2 + 2; x++) {
						float r = sqrt(y * y + x * x);
						if (r < rr + 1 || r >= rr + FEATURE_ROW)
							continue;
						int sita = atan2(-y, x) * 180 / M_PI;
						if (sita < 0)
							sita += 360;
						r -= rr + 1;
						int ir = (int)r;
						CV_Assert(ir >= 0 && ir <= FEATURE_ROW - 2);
						if (ir < FEATURE_ROW - 2)
							vec[ir].push_back(Point3i(x, y, sita));
						if (ir >= 1)
							vec[ir - 1].push_back(Point3i(x, y, sita));
					}
					for (int i = 0; i < (int)vec.size(); i++) {
						sort(vec[i].begin(), vec[i].end(), greaterPoint3i); //now vec is in clockwise
						for (int j = 0; j < (int)vec[i].size(); j++) {
							vec[i][j].x = vec[i][j].x - tl.x;
							vec[i][j].y = vec[i][j].y - tl.y;
						}
					}
					circle_edges[dd] = vec;
					circle_edges_it = circle_edges.find(dd);
				}
				
				int min_es = 0x20000000, submin_es = 0x20000000, total_es = 0;
				Vec8i e8;
				for (int i = 0; i < 8; i++) {
					int es = 0;
					for (auto v3 : cir[i]) {
						int cross = p_grad[v3[2]][0] * v3[0] + p_grad[v3[2]][1] * v3[1]; //grad and dir-vector inner product
						es += (cross >= 0) ? cross : cross * 4; //cross <0 ,punish
					}
					es = es / (int) cir[i].size();
					es = es / 16; //grad sum is 16
					if (es < min_es) {
						submin_es = min_es;
						min_es = es;
					}
					else 
					if (es < submin_es)
						submin_es = es;
					if (es < score_min)
						break;
					e8[i] = es; //output feature is grad * 100
					total_es += es;
				}
				if (min_es < score_min)
					continue;
				int score = min_es + submin_es / 2 + total_es / 32;
				if (score > best_score) {
					best_score = score;
					feature.at<int>(0, 0) = org.x; 
					feature.at<int>(0, 1) = org.y;  
					feature.at<int>(0, 2) = d1; 
					feature.at<int>(0, 3) = d2; 
					for (int i = 0; i < 8; i++)
						feature.at<int>(1, dir_remap[i]) = e8[i];
				}
			}
		}
		if (best_score > score0) {
			score1 = score0;
			score0 = best_score;
			features[1] = features[0];
			features[0] = feature;
		}
		else 
		if (best_score > score1) {
			score1 = best_score;
			features[1] = feature;
		}
			
	}
	if (score0 == score_min)
		return score_min;
	for (int k = 0; k < 2; k++) 
	if (!features[k].empty()) {
		Point org(features[k].at<int>(0, 0), features[k].at<int>(0, 1));
		int d1 = features[k].at<int>(0, 2);
		int d2 = features[k].at<int>(0, 3);
		vector<vector<Point3i> > & cir_edge = circle_edges.find(max(d1, d2))->second;
		CV_Assert(cir_edge.size() == FEATURE_ROW - 2);
#if 0
		vector<uchar> flt_out;
		for (int i = 0; i < (int)cir_edge.size(); i++) {
			vector<uchar> img_gray;
			int circle_edge_avg_len = max(CIRCLE_EDGE_AVG_LEN, (int) cir_edge[i].size() / 8);
			img_gray.reserve(cir_edge[i].size() + circle_edge_avg_len);
			for (int j = 0; j < (int)cir_edge[i].size(); j++) {
				Point pt(org.x + cir_edge[i][j].x, org.y + cir_edge[i][j].y);
				img_gray.push_back(img.at<uchar>(pt));
			}
			vector<int> img_sum(cir_edge[i].size()), img_dif(cir_edge[i].size());
			img_sum[0] = 0;
			for (int j = 0; j < circle_edge_avg_len; j++) {
				img_sum[0] += img_gray[j + 1];
				img_gray.push_back(img_gray[j]); //fill img_gray to easy handle as a circle
			}
			for (int j = 1; j < (int)img_sum.size(); j++)
				img_sum[j] = img_sum[j - 1] - img_gray[j] + img_gray[j + circle_edge_avg_len];
			for (int j = 0; j < (int)img_dif.size(); j++) {
				int kk = j - circle_edge_avg_len - 1;
				img_dif[j] = img_sum[j] - img_sum[kk < 0 ? kk + (int)cir_edge[i].size() : kk]; //img_dif is in clockwise, 6:00 - 5:00
				img_dif[j] = (abs(img_dif[j]) << 10) + j;
			}
			sort(img_dif.begin(), img_dif.end(), greater<int>());
			for (int j = 0; j < (int)img_sum.size(); j++)
				img_sum[j] /= circle_edge_avg_len;
			vector<int> choose;

			for (auto d : img_dif) {
				bool pass = true;
				int l = cir_edge[i][d & 0x3ff].z;
				int ll = (l > 270) ? l - 360 : l;
				for (auto c : choose)
				if (abs(l - cir_edge[i][c].z) < 45 || abs(ll - cir_edge[i][c].z) < 45) {
					pass = false;
					break;
				}
				if (pass)
					choose.push_back(d & 0x3ff);
			}
						
			for (int j = 0; j < 8; j++) {
				if (j >= choose.size()) {
					features[k].at<int>(i + 2, j) = 0;
					continue;
				}
				int idx = choose[j];
				int idx1 = idx - circle_edge_avg_len - 1;
				if (idx1 < 0)
					idx1 += (int)cir_edge[i].size();
				features[k].at<int>(i + 2, j) = cir_edge[i][idx].z * 1000000 + img_sum[idx] * 1000 + img_sum[idx1];
			}
		}
#endif

		vector<Vec3i> & dd = d0s.find(100 * d1 + d2)->second;
		int s = 0, n = 0;
		for (int i = 0; i < (int)dd.size(); i++) {
			int y = dd[i][0];
			int x1 = dd[i][1];
			int x2 = dd[i][2];
			if (x1 > x2 + 1) {
				s += lg.at<int>(y + org.y, org.x + x1) - lg.at<int>(y + org.y, org.x + x2 + 1); //sum from x0-x to x0+x
				n += x1 - x2 - 1;
			}
		}
		features[k].at<int>(0, 4) = s / n;
		s = 0, n = 0;
		int y = d2 *0.4;
		int x = d1 *0.4;
		for (int i = -y; i <= y; i++) {
			s += lg.at<int>(org.y + d2/2 + i, org.x + d1 / 2 + x + 1) - lg.at<int>(org.y + d2/2 + i, org.x + d1 / 2- x);
			n += 2 * x + 1;
		}
		features[k].at<int>(0, 5) = s / n;
		features[k].at<int>(0, 6) = 0;
		features[k].at<int>(0, 7) = 0;
	}
	return score0;
}

int ViaML::feature_extract(const Mat & img, Point & org, Point & range, const vector<int> & d0, Mat & feature, vector<Point> * vs, bool multi_thread, int score_min)
{
	if (d0.empty())
		return 0;
	int max_d = *max_element(d0.begin(), d0.end());
	int x0 = max(0, org.x - range.x - max_d);
	int x1 = min(img.cols - 1, org.x + range.x + max_d);
	int y0 = max(0, org.y - range.y - max_d);
	int y1 = min(img.rows - 1, org.y + range.y + max_d);
	Rect via_rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
	
	Mat ig, iig, lg, llg, grad;
	Mat img_via_rect = img(via_rect);
	integral_square(img_via_rect, ig, iig, lg, llg, true);
	compute_grad(img_via_rect, grad);
	Mat feature2[2];
	int best_score = score_min;
	for (auto d : d0) { //find biggest score in d
		vector<Point> loc;
		find_best_bright(lg, d, 2, loc, multi_thread);
		int score = compute_feature(img_via_rect, lg, grad, d, loc, feature2, multi_thread, score_min);
		if (score > best_score) {
			best_score = score;
			feature = feature2[0].clone();
		}
	}
	if (best_score == score_min)
		return score_min;
	range.x = feature.at<int>(0, 2);
	range.y = feature.at<int>(0, 3);
	org.x = feature.at<int>(0, 0) + x0 + range.x / 2;
	org.y = feature.at<int>(0, 1) + y0 + range.y / 2;
	//compute vs for via shape
	if (vs) {
		vs->clear();
		int xx0 = feature.at<int>(0, 0);
		int yy0 = feature.at<int>(0, 1);
		int d1 = feature.at<int>(0, 2);
		int d2 = feature.at<int>(0, 3);
		vector<Vec3i> & dd = d0s.find(100 * d1 + d2)->second;

		for (int i = 0; i < (int)dd.size(); i++) {
			int y = dd[i][0];
			int x1 = dd[i][1];
			int x2 = dd[i][2];

			vs->push_back(Point(x0 + xx0 + x1, y0 + yy0 + y));
			vs->push_back(Point(x0 + xx0 + x2, y0 + yy0 + y));
		}
	}
	
	return best_score;
}

bool ViaML::feature_extract(const Mat & img, Point & org, Point & range, int min_d0, int max_d0, int label, vector<Mat> & features, vector<Point> * vs, bool multi_thread)
{
	features.clear();
	if (label & VIA_IS_VIA) { //it is via
		vector<int> d;
		Mat feature;
		for (int i = min_d0; i <= max_d0; i++)
		if (i > 5)
			d.push_back(i);
		int score = feature_extract(img, org, range, d, feature, vs, multi_thread, VIA_MINIMUM_GRAD);
		if (score > VIA_MINIMUM_GRAD) {
			feature.at<int>(0, 0) = label;
			features.push_back(feature);
			return true;
		}
		else
			return false;
	}
	else { //it is not via		
		int best_score = NOVIA_MINIMUM_GRAD;
		for (int i = min_d0; i <= max_d0; i++) {
			vector<Point> vs0;
			vector<int> d;
			Mat feature;
			if (i > 5)
				d.push_back(i);
			else
				continue;
			int score = feature_extract(img, org, range, d, feature, vs ? &vs0 : NULL, multi_thread, NOVIA_MINIMUM_GRAD);
			if (best_score < score) {
				best_score = score;
				if (vs)
					vs->swap(vs0);
			}
			if (score > NOVIA_MINIMUM_GRAD) {
				feature.at<int>(0, 0) = label;
				bool same = false;
				for (auto & f : features)
				if (f.at<int>(0, 2) == feature.at<int>(0, 2) && f.at<int>(0, 3) == feature.at<int>(0, 3))
					same = true;
				if (!same)
					features.push_back(feature);
			}
		}
		if (best_score > NOVIA_MINIMUM_GRAD)
			return true;
		else
			return false;
	}
}

/*
Input features
Input weight
Return true if success
*/
bool ViaML::train(const vector<Mat> & features, float weight)
{
	Mat train_data_svm;
	Mat label((int)features.size(), 1, CV_32FC1);
	//prepare train via svm
	bool has_via[2] = { false, false };
	via_dia.clear();
	ViaFeature2Vector f2v;
	for (int i = 0; i < (int) features.size(); i++) {
		const Mat &feature = features[i];
		int s = feature.at<int>(0, 0) & 1;
		int d1 = feature.at<int>(0, 2);
		int d2 = feature.at<int>(0, 3);
		label.at<float>(i) = s * 2 - 1;
		Mat sample = f2v.compute_via_vec(feature, true);
		train_data_svm.push_back(sample);
		qInfo("ViaML:train, is_via=%d, d=%4f, s=%5f,g=%5.1f,%5.1f,%5.1f", (int)label.at<float>(i),
			train_data_svm.at<float>(i, 0), train_data_svm.at<float>(i, 1), train_data_svm.at<float>(i, 2),
			train_data_svm.at<float>(i, 3), train_data_svm.at<float>(i, 4));		
		has_via[s] = true;
		int diameter = (d1 + d2) / 2;
		//fill via diameter set
		if (abs(d1 - d2) == 1) {
			if (via_dia.find(d1) == via_dia.end() && via_dia.find(d2) == via_dia.end())
				via_dia.insert(diameter);
		}
		else
			via_dia.insert(diameter);
	}
	via_svm.reset();
	if (!has_via[1]) {
		is_valid = false;
		return false;
	}
	f2v.get_addition_vec(train_data_svm, label);
	//train via svm
	via_svm.reset(train_svm(train_data_svm, label, weight, 5));
	is_valid = true;
	return true;
}

float ViaML::judge(const Mat & img, Mat & via_mark, Point & org, Point & range, int &label, bool multi_thread)
{
	label = 0;
	
	int max_d = *max_element(via_dia.begin(), via_dia.end());
	int x0 = max(0, org.x - range.x - max_d);
	int x1 = min(img.cols - 1, org.x + range.x + max_d);
	int y0 = max(0, org.y - range.y - max_d);
	int y1 = min(img.rows - 1, org.y + range.y + max_d);
	Rect via_rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

	Mat ig, iig, lg, llg, grad;
	Mat img_via_rect = img(via_rect);
	integral_square(img_via_rect, ig, iig, lg, llg, true);
	compute_grad(img_via_rect, grad);
	Mat feature2[2];

	float max_response = -1;
	ViaFeature2Vector f2v;
	Mat best_feature;
	for (auto d : via_dia) {
		vector<Point> loc;
		find_best_bright(lg, d, 2, loc, multi_thread);
		compute_feature(img_via_rect, lg, grad, d, loc, feature2, multi_thread, VIA_MINIMUM_GRAD);
		for (int i = 0; i < 2; i++) 
		if (!feature2[i].empty()) {
			const Mat &feature = feature2[i];
			Mat sample = f2v.compute_via_vec(feature, false);
			float response = -via_svm->predict(sample, true);
			if (response > max_response) {
				max_response = response;
				range.x = feature.at<int>(0, 2);
				range.y = feature.at<int>(0, 3);
				org.x = feature.at<int>(0, 0) + x0 + (range.x + 1) / 2;
				org.y = feature.at<int>(0, 1) + y0 + (range.y + 1) / 2;
				if (response > 0)
					best_feature = feature.clone();
			}
		}
	}
	if (max_response > 0) {
		label |= VIA_IS_VIA;
		int d = max(range.x, range.y);
		Mat via_mark_rect = via_mark(via_rect);
		Point o = org - Point(x0, y0);
		circle(via_mark_rect, o, d / 2, Scalar::all(VIA_MARK_VIA_REGION), CV_FILLED);
		circle(via_mark_rect, o, 2, Scalar::all(VIA_MARK_VIA_CENTER), CV_FILLED);
		int r2 = (d / 2 + VIA_REGION_RADIUS) * (d / 2 + VIA_REGION_RADIUS);
		for (int y = 0; y < via_mark_rect.rows; y++) {
			uchar * p_mark = via_mark_rect.ptr<uchar>(y);
			Vec2i * p_grad = grad.ptr<Vec2i>(y);
			for (int x = 0; x < via_mark_rect.cols; x++) 
			if (p_mark[x] != VIA_MARK_VIA_REGION && p_mark[x] != VIA_MARK_VIA_CENTER) {
				int r = (y - o.y) * (y - o.y) + (x - o.x) * (x - o.x);
				if (r > r2)
					continue;
				float deg = (y - o.y) * p_grad[x][0] + (x - o.x) * p_grad[x][1];
				if (deg < 0) {
					deg = -deg / sqrt(r * (p_grad[x][0] * p_grad[x][0] + p_grad[x][1] * p_grad[x][1]) + 0.01);
					if (deg > 0.7)
						p_mark[x] = VIA_MARK_VIA_REGION;
				}
			}
		}
		for (int y = max(org.y - 2 * d, 0); y <= min(org.y + 2 * d, via_mark.rows - 1); y++) {
			uchar * p_mark = via_mark.ptr<uchar>(y);
			for (int x = max(org.x - 2 * d, 0); x <= min(org.x + 2 * d, via_mark.cols - 1); x++)
			if (p_mark[x] != VIA_MARK_VIA_REGION &&  p_mark[x] != VIA_MARK_VIA_CENTER)
				p_mark[x] = VIA_MARK_NEAR_VIA;
		}
	}
	return max_response;
}

VWfeature::VWfeature() 
{
	retrain_via = false;
	retrain_edge = false;
	via_weight = 0.5;
}

void VWfeature::set_via_para(float via_weight_)
{
	if (via_weight != via_weight_)
		retrain_via = true;
	via_weight = via_weight_;
}

bool VWfeature::add_feature(const Mat & img, Point local, Point & global, Point & range, int min_d0, int max_d0, int label, vector<Point> * vs)
{
	if (max_d0 <= 5) {
		qCritical("add_feature d0 too low");
		return false;
	}
	if (local.x <= max_d0 || local.y <= max_d0 || local.x >= img.cols - max_d0 || local.y >= img.rows - max_d0) {
		qCritical("add_feature (%d,%d) out of range", local.x, local.y);
		return false;
	}
	Point local1 = local, global1;
	if (label & VIA_FEATURE) {
		range = Point(0, 0);
		vector<Mat> features;
		bool ret = vml.feature_extract(img, local1, range, min_d0, max_d0, label, features, vs, false);
		if (!ret)
			return false;
		global1 = global + local1 - local;
		del_feature(global1, max_d0);
		for (int i = 0; i < features.size(); i++) {
			via_locs.push_back(global1);
			via_features.push_back(features[i]);
		}
		if (vs) {
			for (int i = 0; i < (int)vs->size(); i++)
				(*vs)[i] += global - local;
		}
		global = global1;
		retrain_via = true;
		retrain_edge = true;
	}
	if (label & EDGE_FEATURE) {
		vector<vector<int> > features;
		bool ret = eml.feature_extract(img, local1, range, label, features);
		if (!ret)
			return false;
		global1 = global + local1 - local;
		del_feature(global1, 2);
		for (int i = 0; i < features.size(); i++) {
			edge_locs.push_back(global1);
			edge_features.push_back(features[i]);
		}
		global = global1;
		retrain_edge = true;
	}
	return true;
}

bool VWfeature::del_feature(Point global, int d0)
{
	for (int i = 0; i < via_locs.size(); i++) {
		Point shift = global - via_locs[i];
		if (abs(shift.x) <= d0 && abs(shift.y) <= d0) {
			via_locs.erase(via_locs.begin() + i);
			via_features.erase(via_features.begin() + i);
			i--;
			retrain_via = true;
		}
	}
	for (int i = 0; i < edge_locs.size(); i++) {
		Point shift = global - edge_locs[i];
		if (abs(shift.x) <= d0 && abs(shift.y) <= d0) {
			edge_locs.erase(edge_locs.begin() + i);
			edge_features.erase(edge_features.begin() + i);
			i--;
			retrain_edge = true;
		}
	}
	return true;
}

int VWfeature::get_via_label(Point & global, int d0)
{
	int min_d = 10000;
	int choose = -1;
	for (int i = 0; i < via_locs.size(); i++) {
		Point shift = global - via_locs[i];
		if (abs(shift.x) <= d0 && abs(shift.y) <= d0) 
			if (min_d > abs(shift.x) + abs(shift.y)) {
				min_d = abs(shift.x) + abs(shift.y);
				choose = i;
			}
	}

	if (choose >= 0) {
		global = via_locs[choose];
		return via_features[choose].at<int>(0, 0);
	}
	return 0;
}

void VWfeature::clear_feature()
{
	via_locs.clear();
	via_features.clear();
	retrain_via = true;
	edge_locs.clear();
	edge_features.clear();
	retrain_edge = true;
}

void VWfeature::get_features(vector<MarkObj> & obj_sets)
{
	CV_Assert(via_locs.size() == via_features.size() && edge_locs.size() == edge_features.size());
	for (int i = 0; i < (int)via_locs.size(); i++) {
		MarkObj o;
		int label = via_features[i].at<int>(0, 0);
		o.type = OBJ_POINT;
		o.type2 = (label & VIA_IS_VIA) ? POINT_NORMAL_VIA0 : POINT_NO_VIA;
		o.prob = 1;
		o.state = 0;
		o.p0 = QPoint(via_locs[i].x, via_locs[i].y);
		o.p1 = QPoint(via_features[i].at<int>(0, 2), via_features[i].at<int>(0, 3));
		obj_sets.push_back(o);
	}		
	
	for (int i = 0; i < (int)edge_locs.size(); i++) {
		MarkObj o;
		bool same = false;
		for (int j = 0; j < i; j++)
		if (edge_locs[j] == edge_locs[i]) {
			same = true;
			break;
		}
		if (same)
			continue;
		int label = edge_features[i][EdgeFeatureVecLen];
		o.type = OBJ_POINT;
		o.p0 = QPoint(edge_locs[i].x, edge_locs[i].y);
		o.p1 = o.p0;
		if (label & EDGE_IS_WIRE_INSU) {
			o.type2 = (label & EDGE_NEAR_VIA) ? POINT_WIRE_INSU_V : POINT_WIRE_INSU;
			int dir = edge_features[i][2];
			o.p1 += QPoint(dxy[dir][1] * -3, dxy[dir][0] * -3);
		}
		if (label & EDGE_IS_WIRE)
			o.type2 = (label & EDGE_NEAR_VIA) ? POINT_WIRE_V : POINT_WIRE;
		if (label & EDGE_IS_INSU)
			o.type2 = (label & EDGE_NEAR_VIA) ? POINT_INSU_V : POINT_INSU;		
		o.prob = 1;
		o.state = 0;
		obj_sets.push_back(o);
	}
}

int VWfeature::get_max_d()
{
	int max_d0 = 0;

	for (auto & feature : via_features)
	if (feature.at<int>(0, 0) & 1) {
		max_d0 = max(feature.at<int>(0, 2), max_d0);
		max_d0 = max(feature.at<int>(0, 3), max_d0);
	}		
	return max_d0;
}

void VWfeature::write_file(string project_path, int layer, int insu_min, int wire_min)
{
	string filename = project_path + "/WorkData/";
	QDir work_dir(QString::fromStdString(filename));
	if (!work_dir.exists()) {
		bool ret = work_dir.mkdir(QString::fromStdString(filename));
		if (!ret) {
			qFatal("Unable to create work dir %s", work_dir.absolutePath().toStdString().c_str());
			return;
		}
	}
	char sh[100];
	sprintf(sh, "vwfeature%d.txt", layer);
	filename = filename + sh;
	FILE * fp = fopen(filename.c_str(), "wt");
	fprintf(fp, "insu_min=%d, wire_min=%d\n", insu_min, wire_min);
	fprintf(fp, "vias=%d\n", (int) via_features.size());
	for (int i = 0; i < (int)via_features.size(); i++) {
		fprintf(fp, "%10d%10d\n", via_locs[i].x, via_locs[i].y);
		for (int y = 0; y < FEATURE_ROW; y++) {
			for (int x = 0; x < 8; x++)
				fprintf(fp, "%10d", via_features[i].at<int>(y, x));
			fprintf(fp, "\n");
		}
	}
	fprintf(fp, "edges=%d\n", (int)edge_features.size());
	for (int i = 0; i < (int)edge_features.size(); i++) {
		fprintf(fp, "%10d%10d\n", edge_locs[i].x, edge_locs[i].y);
		for (int j = 0; j < (int)edge_features[i].size(); j++)
			fprintf(fp, "%10d", edge_features[i][j]);
		fprintf(fp, "\n");
	}
	fclose(fp);
}

bool VWfeature::read_file(string project_path, int layer)
{
	string filename = project_path + "/WorkData/";
	char sh[100];
	sprintf(sh, "vwfeature%d.txt", layer);
	filename = filename + sh;
	retrain_via = true;
	via_features.clear();
	via_locs.clear();
	FILE * fp = fopen(filename.c_str(), "rt");
	if (fp == NULL)
		return false;
	int s, t;
	int insu_min, wire_min;
	if (fscanf(fp, "insu_min=%d, wire_min=%d\n", &insu_min, &wire_min) != 2) {
		qCritical("%s corrupted, no insu or wire min", filename.c_str());
		insu_min = 2;
		wire_min = 5;
	}
	if (fscanf(fp, "vias=%d", &s) != 1) {
		qCritical("%s corrupted, no vias", filename.c_str());
		return false;
	}
	via_features.resize(s);
	via_locs.resize(s);
	for (int i = 0; i < (int)via_features.size(); i++) {
		via_features[i].create(FEATURE_ROW, 8, CV_32S);
		if (fscanf(fp, "%d%d", &s, &t) != 2) {
			qCritical("%s corrupted, no via loc", filename.c_str());
			return false;
		}
		via_locs[i] = Point(s, t);
		for (int y = 0; y < FEATURE_ROW; y++) {
			for (int x = 0; x < 8; x++) {
				if (fscanf(fp, "%d", &s) != 1) {
					qCritical("%s corrupted, no via feature", filename.c_str());
					return false;
				}
				via_features[i].at<int>(y, x) = s;
			}
		}
	}
	if (fscanf(fp, "\nedges=%d", &s) != 1) {
		qCritical("%s corrupted, no edges", filename.c_str());
		return false;
	}
	retrain_edge = true;
	edge_features.resize(s);
	edge_locs.resize(s);
	for (int i = 0; i < (int)edge_features.size(); i++) {
		if (fscanf(fp, "%d%d", &s, &t) != 2) {
			qCritical("%s corrupted, no edge loc", filename.c_str());
			return false;
		}
		edge_locs[i] = Point(s, t);
		edge_features[i].resize(EdgeFeatureVecLen + 1);
		for (int j = 0; j <= EdgeFeatureVecLen; j++)
		if (fscanf(fp, "%d", &edge_features[i][j]) != 1) {
			qCritical("%s corrupted, no edge feature", filename.c_str());
			return false;
		}
	}
	fclose(fp);
	return true;
}

bool VWfeature::via_valid(bool multi_thread)
{
	if (retrain_via) {
		CV_Assert(!multi_thread);
		vml.train(via_features, via_weight);
		retrain_via = false;
	}
	return vml.is_valid;
}

void VWfeature::split_via_edges(vector<vector<int> > & edge_via, vector<vector<int> > & edge_no_via)
{
	int d0 = get_max_d();
	edge_via.clear();
	edge_no_via.clear();
	CV_Assert(edge_locs.size() == edge_features.size());
	for (int i = 0; i < (int)edge_locs.size(); i++) {
		int near_via = edge_features[i][EdgeFeatureVecLen] & EDGE_NEAR_VIA;
		if (!near_via)
		for (auto v : via_locs) 
		if (abs(edge_locs[i].x - v.x) < d0 && abs(edge_locs[i].y - v.y) < d0 * 2) {
			near_via = 1;
			break;
		}
		if (near_via)
			edge_via.push_back(edge_features[i]);
		else
			edge_no_via.push_back(edge_features[i]);
	}
}

float VWfeature::via_judge(const Mat & img, Mat & via_mark, Point & org, Point & range, int & label, bool multi_thread)
{
	if (via_valid(multi_thread))
		return vml.judge(img, via_mark, org, range, label, multi_thread);
	else
		return -1;
}

#define MAKE_PROB(s, x, y) ((unsigned long long)(s) << 32 | (y) << 16 | (x))
#define PROB_X(p) ((int)((p) & 0xffff))
#define PROB_Y(p) ((int)((p) >> 16 & 0xffff))
#define PROB_S(p) ((unsigned)((p) >> 32 & 0xffffffff))

void VWfeature::via_search(const Mat & img, Mat & via_mark, Mat & mark_dbg, vector<ElementObj *> & vs, bool multi_thread)
{
	vs.clear();
	if (!via_valid(multi_thread))
		return;
	
	via_mark.create(img.rows, img.cols, CV_8U);
	via_mark = VIA_MARK_NO_VIA;
	//1 find min diameter and min via gray
	int min_d0 = 1000, max_d0 = 0;
	int min_a1 = 1000, max_a0 = 0;
	for (auto & feature : via_features) {
		if (feature.at<int>(0, 0) & 1) {
			min_d0 = min(feature.at<int>(0, 2), min_d0);
			min_d0 = min(feature.at<int>(0, 3), min_d0);
			max_d0 = max(feature.at<int>(0, 2), max_d0);
			max_d0 = max(feature.at<int>(0, 3), max_d0);
			min_a1 = min(feature.at<int>(0, 5), min_a1);
		}
		else 
			max_a0 = max(feature.at<int>(0, 5), max_a0);
	}
	int grid = min_d0 - 2;
	CV_Assert(grid >= 2);
	Mat prob(img.rows / grid + 1, img.cols / grid + 1, CV_64FC1);
	for (int y = 0; y < prob.rows; y++) {
		uint64 *p_prob = prob.ptr<uint64>(y);
		for (int x = 0; x < prob.cols; x++)
			p_prob[x] = 0ULL;
	}
	//2 build grid
    Mat ig, iig, lg, llg;
    integral_square(img, ig, iig, lg, llg, false);
	int r0 = min_d0 * 0.4;
	int th = min((min_a1 + max_a0) / 2, min_a1);
	th = th * (2 * r0 + 1) *(2 * r0 + 1);
	for (int y = max_d0 + FEATURE_ROW; y < img.rows - max_d0 - FEATURE_ROW; y++) {
		unsigned *p_ig = ig.ptr<unsigned>(y - r0);
		unsigned *p_ig1 = ig.ptr<unsigned>(y + r0 + 1);
		uint64 *p_prob = prob.ptr<uint64>(y / grid);
		for (int x = max_d0 + FEATURE_ROW; x < img.cols - max_d0 - FEATURE_ROW; x++) {
			int s = p_ig1[x + r0 + 1] + p_ig[x - r0] - p_ig1[x - r0] - p_ig[x + r0 + 1];
			if (s < th) //filter gray < th
				continue;
			int xm = x / grid;
            p_prob[xm] = max(p_prob[xm], (uint64)(MAKE_PROB(s, x, y)));
		}
	}
	//3 find local maximum gray and judge
	for (int y = 0; y < prob.rows; y++) {
		uint64 *p_prob = prob.ptr<uint64>(y);
		for (int x = 0; x < prob.cols; x++) 
		if (p_prob[x]) {
			unsigned s0 = PROB_S(p_prob[x]);
			int x0 = PROB_X(p_prob[x]);
			int y0 = PROB_Y(p_prob[x]);
			bool pass = true; //pass means local maximum
			for (int dir = 0; dir < 8; dir++) {
				int xx = x + dxy[dir][1];
				int yy = y + dxy[dir][0];
				if (xx < 0 || yy < 0 || xx >= prob.cols || yy >= prob.rows) {
					pass = false;
					break;
				}
				uint64 *p_prob1 = prob.ptr<uint64>(yy, xx);
				if (PROB_S(p_prob1[0]) > s0 && abs(PROB_X(p_prob1[0]) - x0) < min_d0 && abs(PROB_Y(p_prob1[0]) - y0) < min_d0)
					pass = false;
			}
			if (pass) { 
				Point org(x0, y0), range(0, 0);
				if (!mark_dbg.empty())
					rectangle(mark_dbg, Point(org.x - 1, org.y - 1), Point(org.x + 1, org.y + 1), 0);
				int label;
				float ret = via_judge(img, via_mark, org, range, label, multi_thread);
				if (label & VIA_IS_VIA) {
					ElementObj * m = new ElementObj;
					m->type = OBJ_POINT;
					m->type2 = POINT_VIA_AUTO_EXTRACT;
					m->prob = min(ret, 1.0f);
					m->p0 = QPoint(org.x, org.y);
					m->p1 = QPoint(range.x, range.y);
					m->un.attach = 1;
					vs.push_back(m);
					if (!mark_dbg.empty()) {
						rectangle(mark_dbg, Point(org.x - range.x / 2, org.y - range.y / 2),
							Point(org.x - range.x / 2 + range.x, org.y - range.y / 2 + range.y), 255);
					}
				}
				else 
				if (ret > -0.3) {
					ElementObj * m = new ElementObj;
					m->type = OBJ_POINT;
					m->type2 = POINT_NO_VIA;
					m->prob = -ret * 2;
					m->p0 = QPoint(org.x, org.y);
					m->p1 = QPoint(range.x, range.y);
					m->un.attach = 1; //init as reference by current tile
					vs.push_back(m);
					if (!mark_dbg.empty())
						rectangle(mark_dbg, Point(org.x - range.x / 2, org.y - range.y / 2),
						Point(org.x - range.x / 2 + range.x, org.y - range.y / 2 + range.y), 100);
				}
			}
		}
	}
}

void VWfeature::edge_search(const Mat & img, Mat & prob, const Mat & via_mark, Mat & debug_mark, bool multi_thread)
{
	if (retrain_edge) {
		CV_Assert(!multi_thread);
		vector<vector<int> > ev, env;
		split_via_edges(ev, env);
		eml.train(env);
		veml.train(ev);
		retrain_edge = false;
	}
	if (!eml.is_valid)
		return;
	Mat grad;
	if (veml.is_valid) {
		eml.judge(img, grad, prob, via_mark, debug_mark);
		veml.judge(img, grad, prob, via_mark / 2, debug_mark);
	}
	else {
		Mat empty;
		eml.judge(img, grad, prob, empty, debug_mark);
	}
	for (int y = 0; y < via_mark.rows; y++) {
		Vec4b * p_prob = prob.ptr<Vec4b>(y);
		const uchar * p_vm = via_mark.ptr<uchar>(y);
		Vec3b * p_debugm = debug_mark.empty() ? NULL : debug_mark.ptr<Vec3b>(y);
		for (int x = 0; x < via_mark.cols; x++)
		if (p_vm[x] == VIA_MARK_VIA_REGION) {
			p_prob[x][POINT_DIR] |= POINT_VIA_REGION;
			p_prob[x][POINT_IS_INSU] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_WIRE] = POINT_TOT_PROB / 2;
			p_prob[x][POINT_IS_EDGE_WIRE_INSU] = 0;
			if (p_debugm) {
				p_debugm[x][0] = 255;
				p_debugm[x][1] = 255;
				p_debugm[x][2] = 255;
			}
		}
		else
			if (p_vm[x] == VIA_MARK_VIA_CENTER) {
				p_prob[x][POINT_DIR] |= POINT_VIA_REGION | POINT_VIA_CENTER;
				p_prob[x][POINT_IS_INSU] = POINT_TOT_PROB / 2;
				p_prob[x][POINT_IS_WIRE] = POINT_TOT_PROB / 2;
				p_prob[x][POINT_IS_EDGE_WIRE_INSU] = 0;
				if (p_debugm) {
					p_debugm[x][0] = 255;
					p_debugm[x][1] = 255;
					p_debugm[x][2] = 255;
				}
			}

	}
}