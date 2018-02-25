#include <string.h>
#define VWEXTRACT_PUBLIC_C
#include "vwextract_public.h"
#include <QRect>

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
