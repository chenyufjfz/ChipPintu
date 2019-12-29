#include <string.h>
#define VWEXTRACT_PUBLIC_C
#include "vwextract_public.h"
#include <qmath.h>
#include <QRect>
#include <algorithm>
#include <qdir.h>
using namespace cv;
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

static Mat Horizon = (Mat_<char>(5, 5) << -1, -1, 0, 1, 1, -1, -2, 0, 2, 1, -2, -4, 0, 4, 2, -1, -2, 0, 2, 1, -1, -1, 0, 1, 1);
static Mat Vert = (Mat_<char>(5, 5) << -1, -1, -2, -1, -1, -1, -2, -4, -2, -1, 0, 0, 0, 0, 0, 1, 2, 4, 2, 1, 1, 1, 2, 1, 1);
static Mat Deg45 = (Mat_<char>(5, 5) << 0, -1, -2, -1, 0, -1, -2, -3, 0, 1, -2, -3, 0, 3, 2, -1, 0, 3, 2, 1, 0, 1, 2, 1, 0);
static Mat Deg135 = (Mat_<char>(5, 5) << 0, 1, 2, 1, 0, -1, 0, 3, 2, 1, -2, -3, 0, 3, 2, -1, -2, -3, 0, 1, 0, -1, -2, -1, 0);

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

uchar _find_middle(uchar * s16, uchar * s128, int midnum, int odd)
{
	int n = 0;
	for (int i = 0; i < 16; i++) {
		if (n + s16[i] >= midnum) {
			int j = 8 * i - 1;
			do {
				j++;
				n += s128[j];
			} while (n < midnum);
			if (n == midnum) {
				if (odd)
					return 2 * j + 1;
				int k = j;
				do {
					k++;
					n += s128[k];
				} while (n == midnum);
				return j + k; // 2 * j + k - j
			}
			else //n > midnum
				return 2 * j;
		}
		n += s16[i];
	}
	return 128;
}
uchar middle_sort(const vector<uchar> & data)
{
	uchar s16[16];
	uchar s128[128];
	memset(s16, 0, sizeof(s16));
	memset(s128, 0, sizeof(s128));
	for (auto d : data) {
		s16[d >> 4]++;
		s128[d >> 1]++;
	}
	return _find_middle(s16, s128, (int) data.size() / 2, (int) data.size() % 2);
}

/*
Input img, image
Input start_pt, start point
Input dir0, scan direction
Input dir1, middle sort direction, vertical to scan direction
Input num0, scan number
Input num1, middle sort number = num1 *2
Output flt_out, its size is num0
middle_sort_line move a rectangle [2, num1] in dir0 direction.
*/
void middle_sort_line(const Mat & img, Point start_pt, int dir0, int dir1, int num0, int num1, vector<uchar> & flt_out)
{
	int d0_shift = ((dxy[dir0][0] * (int)img.step.p[0] + dxy[dir0][1] * (int)img.step.p[1]) / sizeof(uchar));
	int d1_shift = ((dxy[dir1][0] * (int)img.step.p[0] + dxy[dir1][1] * (int)img.step.p[1]) / sizeof(uchar));
	uchar s16[16];
	uchar s128[128];
	memset(s16, 0, sizeof(s16));
	memset(s128, 0, sizeof(s128));
	flt_out.resize(num0);
	const uchar * p_img = img.ptr<uchar>(start_pt.y, start_pt.x);
	const uchar * p_img_1 = NULL;
	const uchar * p_img_2 = NULL;
	for (int i = 0; i < num0 + 1; i++) {
		for (int j = 0, s = 0; j < num1; j++, s+=d1_shift) {
			uchar data = p_img[s];
			s16[data >> 4]++;
			s128[data >> 1]++;
		}
		if (p_img_2) {
			for (int j = 0, s = 0; j < num1; j++, s+=d1_shift) {
				uchar data = p_img_2[s];
				s16[data >> 4]--;
				s128[data >> 1]--;
			}
		}
		if (p_img_1)
			flt_out[i - 1] = _find_middle(s16, s128, num1, 0);
		p_img_2 = p_img_1;
		p_img_1 = p_img;
		p_img += d0_shift;
	}
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

ViaML::ViaML()
{
	via_svm = NULL;
}

void ViaML::find_best_bright(const Mat & lg, int d, vector<Point> & loc)
{
	auto d0it = d0s.find(d);
	if (d0it == d0s.end()) {
		vector<Vec3i> dd;
		compute_circle_dx1(d, dd);
		d0s[d] = dd;
		d0it = d0s.find(d);
	}
	vector<Vec3i> & dd = d0it->second;
	Mat sum(lg.rows, lg.cols, CV_32SC1, Scalar::all(0x20000000)); //circle gray sum
	//compute sum
	for (int y0 = 0; y0 < lg.rows - d; y0++)
	for (int x0 = 0; x0 < lg.cols - d - 1; x0++) {
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

#define VIA_MINIMUM_GRAD 1000

int ViaML::compute_feature(const Mat & img, const Mat & lg, const Mat & grad, int d, const vector<Point> & loc, Mat features[])
{
	CV_Assert(grad.type() == CV_32SC2 && img.type() == CV_8UC1);
	int score0 = 0, score1 = 0;
	features[0].release();
	features[1].release();
	for (auto o : loc) {
		int best_score = 0;
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
							Point ep(edge_point - tl);
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
					circles[grad.step.p[0] * 10000 + d1 * 100 + d2] = cir;
					d0s[d1 * 100 + d2] = xy;
					circle_it = circles.find(grad.step.p[0] * 10000 + d1 * 100 + d2);
				}
				vector<vector<Vec3i> > & cir = circle_it->second; //cir[i] is one edge, cir[i][j] is one point
				auto circle_edges_it = circle_edges.find(d1 * 100 + d2);

				if (circle_edges_it == circle_edges.end()) {
					//following compute circle_edges outside
					vector<Vec6i> v6(8);					
					for (int i = 0; i < 8; i++) {
						Point2f p1;
						switch (i) {
						case 0:
							v6[i][3] = DIR_RIGHT;
							v6[i][4] = 2 * r2 * sinf(M_PI_4 / 2);
							v6[i][5] = DIR_DOWN;
							p1 = Point2f(r1 + 1, -r2 * sinf(M_PI_4 / 2));
							break;
						case 1:
							v6[i][3] = DIR_UPRIGHT;
							v6[i][4] = (r1 + r2) * sinf(M_PI_4 / 2);
							v6[i][5] = DIR_UPLEFT;
							p1 = Point2f((r1 + 1) * cosf(M_PI_4 / 2), -(r2 + 1) * sinf(M_PI_4 / 2));
							break;
						case 2:
							v6[i][3] = DIR_UP;
							v6[i][4] = 2 * r1 * sinf(M_PI_4 / 2);
							v6[i][5] = DIR_RIGHT;
							p1 = Point2f(-r1 * sinf(M_PI_4 / 2), -r2 - 1);
							break;
						case 3:
							v6[i][3] = DIR_UPLEFT;
							v6[i][4] = (r1 + r2) * sinf(M_PI_4 / 2);
							v6[i][5] = DIR_DOWNLEFT;
							p1 = Point2f(-(r1 + 1) * sinf(M_PI_4 / 2), -(r2 + 1) * cosf(M_PI_4 / 2));
							break;
						case 4:
							v6[i][3] = DIR_LEFT;
							v6[i][4] = 2 * r2 * sinf(M_PI_4 / 2);
							v6[i][5] = DIR_DOWN;
							p1 = Point2f(-r1 - 1, -r2 * sinf(M_PI_4 / 2));
							break;
						case 5:
							v6[i][3] = DIR_DOWNLEFT;
							v6[i][4] = (r1 + r2) * sinf(M_PI_4 / 2);
							v6[i][5] = DIR_DOWNRIGHT;
							p1 = Point2f(-(r1 + 1) * cosf(M_PI_4 / 2), (r2 + 1) * sinf(M_PI_4 / 2));
							break;
						case 6:
							v6[i][3] = DIR_DOWN;
							v6[i][4] = 2 * r1 * sinf(M_PI_4 / 2);
							v6[i][5] = DIR_RIGHT;
							p1 = Point2f(-r1 * sinf(M_PI_4 / 2), r2 + 1);
							break;
						case 7:
							v6[i][3] = DIR_DOWNRIGHT;
							v6[i][4] = (r1 + r2) * sinf(M_PI_4 / 2);
							v6[i][5] = DIR_UPRIGHT;
							p1 = Point2f((r1 + 1) * sinf(M_PI_4 / 2), (r2 + 1) * cosf(M_PI_4 / 2));
							break;
						}
						v6[i][0] = p1.y - tl.y;
						v6[i][1] = p1.x - tl.x;
					}
					circle_edges[d1 * 100 + d2] = v6;
					circle_edges_it = circle_edges.find(d1 * 100 + d2);
				}
				
				vector<Vec6i> & cir_edge = circle_edges_it->second;
				CV_Assert(cir.size() == 8);
				int min_es = 0x20000000, submin_es = 0x20000000, total_es = 0;
				Vec8i e8;
				for (int i = 0; i < 8; i++) {
					int es = 0;
					for (auto v3 : cir[i]) {
						int cross = p_grad[v3[2]][0] * v3[0] + p_grad[v3[2]][1] * v3[1];
						es += (cross >= 0) ? cross : cross * 4;
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
					if (es < VIA_MINIMUM_GRAD)
						break;
					e8[i] = es;
					total_es += es;
				}
				if (min_es < VIA_MINIMUM_GRAD)
					continue;
				int score = min_es + submin_es / 2 + total_es / 64;
				if (score > best_score) {
					best_score = score;
					feature.at<int>(0, 0) = org.x; 
					feature.at<int>(0, 1) = org.y;  
					feature.at<int>(0, 2) = d1; 
					feature.at<int>(0, 3) = d2; 
					for (int i = 0; i < 8; i++)
						feature.at<int>(1, cir_edge[i][3]) = e8[i];
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
	if (score0 == 0)
		return 0;
	for (int k = 0; k < 2; k++) 
	if (!features[k].empty()) {
		Point org(features[k].at<int>(0, 0), features[k].at<int>(0, 1));
		int d1 = features[k].at<int>(0, 2);
		int d2 = features[k].at<int>(0, 3);
		vector<Vec6i> & cir_edge = circle_edges.find(d1 * 100 + d2)->second;
		CV_Assert(cir_edge.size() == 8);

		vector<uchar> flt_out;
		for (int i = 0; i < 8; i++) {
			Point start_pt(org.x + cir_edge[i][1], org.y + cir_edge[i][0]);
			middle_sort_line(img, start_pt, cir_edge[i][3], cir_edge[i][5], FEATURE_ROW - 2, cir_edge[i][4], flt_out);
			for (int j = 0; j < FEATURE_ROW - 2; j++)
				features[k].at<int>(j + 2, cir_edge[i][3]) = flt_out[j];
		}

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

int ViaML::feature_extract(const Mat & img, Point & org, Point & range, const vector<int> & d0, Mat & feature, vector<Point> * vs)
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
	int best_score = 0;
	for (auto d : d0) { //find biggest score in d
		vector<Point> loc;
		find_best_bright(lg, d, loc);
		int score = compute_feature(img_via_rect, lg, grad, d, loc, feature2);
		if (score > best_score) {
			best_score = score;
			feature = feature2[0].clone();
		}
	}
	if (best_score == 0)
		return 0;
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

bool ViaML::feature_extract(const Mat & img, Point & org, Point & range, int min_d0, int max_d0, int label, vector<Mat> & features, vector<Point> * vs)
{
	features.clear();
	if (label & 1) { //it is via
		vector<int> d;
		Mat feature;
		for (int i = min_d0; i <= max_d0; i++)
		if (i > 5)
			d.push_back(i);
		int score = feature_extract(img, org, range, d, feature, vs);
		if (score > 0) {
			feature.at<int>(0, 0) = label;
			features.push_back(feature);
			return true;
		}
		else
			return false;
	}
	else { //it is not via		
		int best_score = 0;
		for (int i = min_d0; i <= max_d0; i++) {
			vector<Point> vs0;
			vector<int> d;
			Mat feature;
			if (i > 5)
				d.push_back(i);
			else
				continue;
			int score = feature_extract(img, org, range, d, feature, &vs0);
			if (best_score < score) {
				best_score = score;
				vs->swap(vs0);
			}
			if (score > 0) {
				feature.at<int>(0, 0) = label;
				bool same = false;
				for (auto & f : features)
				if (f.at<int>(0, 2) == feature.at<int>(0, 2) && f.at<int>(0, 3) == feature.at<int>(0, 3))
					same = true;
				if (!same)
					features.push_back(feature);
			}
		}
		if (best_score > 0)
			return true;
		else
			return false;
	}
}

bool ViaML::train(const vector<Mat> & features)
{
	Mat train_data_svm((int)features.size(), 5, CV_32FC1);
	Mat label((int)features.size(), 1, CV_32FC1);
	//prepare train data svm
	bool has_via[2] = { false, false };
	via_dia.clear();
	for (int i = 0; i < (int) features.size(); i++) {
		const Mat &feature = features[i];
		CV_Assert(feature.cols == 8 && feature.rows == FEATURE_ROW);
		vector<int> g;
		for (int j = 0; j < 8; j++)
			g.push_back(feature.at<int>(1, j));
		sort(g.begin(), g.end());		
		int s = feature.at<int>(0, 0) & 1;
		int d1 = feature.at<int>(0, 2);
		int d2 = feature.at<int>(0, 3);
		int sum = feature.at<int>(0, 4);
		train_data_svm.at<float>(i, 0) = (d1 + d2) * 5;
		train_data_svm.at<float>(i, 1) = sum;
		train_data_svm.at<float>(i, 2) = g[0] * 0.1;
		train_data_svm.at<float>(i, 3) = g[1] * 0.1;
		train_data_svm.at<float>(i, 4) = g[2] * 0.1;
		label.at<float>(i) = s * 2 - 1;
		qInfo("ViaML:train, is_via=%d, d=%f, s=%d,g=%f,%f,%f", (int)label.at<float>(i),
			train_data_svm.at<float>(i, 0), sum, train_data_svm.at<float>(i, 2),
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
	if (!has_via[0] || !has_via[1])
		return false;
	
	// Set up SVM's parameters
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::POLY;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	params.C = 2;
	params.degree = 2;
	params.gamma = 1;
	params.coef0 = 0;
	if (via_svm)
		delete via_svm;
	via_svm = new CvSVM;
	via_svm->train(train_data_svm, label, Mat(), Mat(), params);
	qInfo("ViaML:train, var_cnt=%d, sup_vec_cnt=%d", via_svm->get_var_count(), via_svm->get_support_vector_count());
	char s[600];
	for (int i = 0; i < via_svm->get_support_vector_count(); i++) {
		const float * sv = via_svm->get_support_vector(i);
		int k = 0;
		for (int j = 0; j < via_svm->get_var_count(); j++)
			k += sprintf(&s[k], "%f,", sv[j]);
		qInfo(s);
	}
	for (int i = 0; i < (int)features.size(); i++) {
		float response = via_svm->predict(train_data_svm.row(i), true);
		qInfo("ViaML:train, is_via=%d, predict=%f", (int)label.at<float>(i), response);
	}
	return true;
}

float ViaML::judge(const Mat & img, Point & org, Point & range, int &label)
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

	int choose = 0;
	float max_response = -1;
	for (auto d : via_dia) {
		vector<Point> loc;
		find_best_bright(lg, d, loc);
		compute_feature(img_via_rect, lg, grad, d, loc, feature2);
		for (int i = 0; i < 2; i++) 
		if (!feature2[i].empty()) {
			const Mat &feature = feature2[i];
			vector<int> g;
			for (int j = 0; j < 8; j++)
				g.push_back(feature.at<int>(1, j));
			sort(g.begin(), g.end());
			Mat gm = (Mat_<float>(1, 3) << g[0], g[1], g[2]);
			float diameter = (feature.at<int>(0, 2) + feature.at<int>(0, 3)) * 5;
			float sum = feature.at<int>(0, 4);
			Mat sample = (Mat_<float>(1, 5) << diameter, sum, g[0] * 0.1, g[1] * 0.1, g[2] * 0.1);
			float response = -via_svm->predict(sample, true);
			if (response > max_response) {
				max_response = response;
				range.x = feature.at<int>(0, 2);
				range.y = feature.at<int>(0, 3);
				org.x = feature.at<int>(0, 0) + x0 + range.x / 2;
				org.y = feature.at<int>(0, 1) + y0 + range.y / 2;
				choose = i;
			}
			if (max_response > 0.999)
				break;
		}
		if (max_response > 0.999)
			break;
	}
	if (max_response > 0)
		label |= 1;
	return max_response;
}

VWfeature::VWfeature() 
{
	retrain_via = false;
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
	range = Point(0, 0);
	vector<Mat> features;
	Point local1 = local, global1;
	bool ret = vml.feature_extract(img, local1, range, min_d0, max_d0, label, features, vs);
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
		}
	}
	retrain_via = true;
	return true;
}

void VWfeature::clear_feature()
{
	via_locs.clear();
	via_features.clear();
	retrain_via = true;
}

void VWfeature::write_file(string project_path, int layer)
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
	fprintf(fp, "%d\n", (int) via_features.size());
	for (int i = 0; i < (int)via_features.size(); i++) {
		fprintf(fp, "%10d%10d\n", via_locs[i].x, via_locs[i].y);
		for (int y = 0; y < FEATURE_ROW; y++) {
			for (int x = 0; x < 8; x++)
				fprintf(fp, "%10d", via_features[i].at<int>(y, x));
			fprintf(fp, "\n");
		}
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
	FILE * fp = fopen(filename.c_str(), "rt");
	if (fp == NULL)
		return false;
	int s, t;
	fscanf(fp, "%d", &s);
	via_features.resize(s);
	via_locs.resize(s);
	for (int i = 0; i < (int)via_features.size(); i++) {
		via_features[i].create(FEATURE_ROW, 8, CV_32S);
		if (fscanf(fp, "%d%d", &s, &t) != 2)
			return false;
		via_locs[i] = Point(s, t);
		for (int y = 0; y < FEATURE_ROW; y++) {
			for (int x = 0; x < 8; x++) {
				if (fscanf(fp, "%d", &s) != 1)
					return false;
				via_features[i].at<int>(y, x) = s;
			}
		}
	}
	return true;
}

float VWfeature::via_judge(const Mat & img, Point & org, Point & range, int & label)
{
	if (retrain_via) {
		vml.train(via_features);
		retrain_via = false;
	}
	return vml.judge(img, org, range, label);
}

#define MAKE_PROB(s, x, y) ((unsigned long long)(s) << 32 | (y) << 16 | (x))
#define PROB_X(p) ((int)((p) & 0xffff))
#define PROB_Y(p) ((int)((p) >> 16 & 0xffff))
#define PROB_S(p) ((unsigned)((p) >> 32 & 0xffffffff))

void VWfeature::via_search(const Mat & img, Mat & mark_dbg, vector<MarkObj > & vs)
{
	vs.clear();
	if (!mark_dbg.empty())
		mark_dbg = 0;
	
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
	Mat ig, iig;
	integral_square(img, ig, iig, Mat(), Mat(), false);
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
			p_prob[xm] = max(p_prob[xm], MAKE_PROB(s, x, y));
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
					mark_dbg.at<uchar>(org) = 100;
				int label;
				float ret = via_judge(img, org, range, label);
				if (label & 1) {
					MarkObj m;
					m.type = OBJ_POINT;
					m.type2 = POINT_VIA_AUTO_EXTRACT;
					m.prob = ret;
					m.p0 = QPoint(org.x, org.y);
					m.p1 = QPoint(range.x, range.y);
					vs.push_back(m);
					if (!mark_dbg.empty())
						rectangle(mark_dbg, Point(org.x - range.x / 2, org.y - range.y / 2), 
						Point(org.x - range.x / 2 + range.x, org.y - range.y / 2 + range.y), 255);
				}
			}
		}
	}
}