#define FEATEXT_C
#include "featext.h"
#include <iostream>
#include <vector>
#include <string>
#include <QtConcurrent>
#include <algorithm>
#include "opencv2/highgui/highgui.hpp"

#ifdef QT_DEBUG
#ifdef Q_OS_WIN
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif

#define FEAT_EXT_VERSION 20

#define PARALLEL 0
#define BLOCK4_SIZE 5
#define BLOCK2_SIZE 7
#define K4_SIZE 1
#define K2_SIZE 3
#define MIN_OVERLAP 12
#define CORNER_SUBMIN_SCORE			3
#define CORNER_TH_SCORE				9
#define NONE_CORNER_SUBMIN_SCORE	7
#define NONE_CORNER_TH_SCORE		9

#define HIT_LEVEL 8

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

struct ImageData {
	int x, y;
	string filename;
	bool is_black_img;
	struct {
		int type;
		Mat d;
	} v[16];
	const TuningPara * tvar;
	const ConfigPara * cvar;
	ImageData() {
		is_black_img = false;
	}
};

struct ImageDiff {
	ImageData *img_d0;
	ImageData *img_d1;
	int dir; //0 is up-down, 1 is left-right
	int overlap; //for left_right compare, image_width + n.x- n+1.x
	int shift; //for left_right compare, n+1.y - n.y
	EdgeDiff * e;
	TuningPara * tvar;
	ConfigPara * cvar;
};


bool greaterPoint3i(const Point3i & a, const Point3i & b) { return a.z > b.z; }

#define MORE_EIG_NUMBER  10

/*
inout vs
Input v
input mindistance
if v is at least mindistance away from all points in vs, push v to vs
Return 0 if fail to push
*/
static int push_point(vector<Point3i> & vs, Point3i v, double mindistance, bool checkonly = false)
{
	for (auto & o : vs) {
		int dx = v.x - o.x;
		int dy = v.y - o.y;
		if (dx*dx + dy*dy <= mindistance)
			return 0;
	}
	if (checkonly)
		return 1;
	vs.push_back(v);
	return 1;
}


/*
inout vs
input remain_number
input d1
input d2
reduce vs.z if vs.x and vs.y is within range d1, d2. then resort, this is to prevent
same line occupy too much weight.
*/
static void resort_points(vector<Point3i> & vs, int remain_number, float d0, float d1, float d2, int min_distance)
{
	if (vs.empty())
		return;
	sort(vs.begin(), vs.end(), greaterPoint3i);
	for (int i = 1; i<(int)vs.size(); i++) {
		double reduce = 1;
		for (int j = 0; j < i; j++) {
			int dx = abs(vs[i].x - vs[j].x);
			int dy = abs(vs[i].y - vs[j].y);
			int d = dx * dx + dy * dy;

			if (d <= min_distance) { //erase near corner
				reduce = -1;
				break;
			}
			if (dx <= d1) { //reduce same line
				double w = (dx == d1) ? 0.95 : 0.9;
				float r = dy / d2;
				r = (r > 1) ? 1 : (1 - w) * r + w;
				reduce = reduce * r;
			}
			if (dy <= d1) { //reduce same line
				double w = (dy == d1) ? 0.95 : 0.9;
				float r = dx / d2;
				r = (r > 1) ? 1 : (1 - w) * r + w;
				reduce = reduce * r;
			}
			
			float r = d / (d0 * d0);
			if (r < 1) //reduce circle
				reduce = reduce * (1 + r) / 2;
		}
		vs[i].z *= reduce;
	}
	sort(vs.begin(), vs.end(), greaterPoint3i);
	if (vs.size() > remain_number) //keep only remain_number
		vs.erase(vs.begin() + remain_number, vs.end());
	while (vs.back().z < 0) //erase near corner
		vs.pop_back();
}

template<typename T> struct greaterThanPtr
{
	bool operator()(const T* a, const T* b) const { return *a > *b; }
};


/*
Compute integrate and line integral
in img
in compute_line_integral, compute lg, llg
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

static int shape_dir[] = { //0 is updown, 1 is leftright, 2 is /, 3 is \.
	0x10, 0x23, 0x10, 0x23, 0x10, 0x23, 0x10, 0x23, //0..7 for 1/4 corner
	0x30, 0x20, 0x21, 0x31, 0x30, 0x20, 0x21, 0x31, //8..15 for 3/8 corner
	0x40, 0x42, 0x41, 0x43, 0x40, 0x42, 0x41, 0x43, //16..23 for 1/2 corner
	0x20, 0x21, 0x31, 0x30, 0x20, 0x21, 0x31, 0x30, //24..31 for 5/8 corner
	0x10, 0x23, 0x10, 0x23, 0x10, 0x23, 0x10, 0x23 //32..39 for 3/4 corner
	//40..47 for 1/8 corner
	//48..55 for 7/8 corner
};
#define IS_CORNER(x) ( x < 16 || x>=24 && x < 40)

class ShapeDis {
protected:
	float cos_shape[56][56];
	void shift_vec(vector<float> & s) const {
		float a = s.back();
		s.pop_back();
		s.insert(s.begin(), a);
	}
	void normalize(vector<float> & s) const {
		float avg = 0;
		for (auto v : s)
			avg += v;
		avg /= s.size();
		float sum = 0;
		for (auto & v : s) {
			v = v - avg;
			sum += v*v;
		}
		sum = sqrt(sum);
		for (auto & v : s)
			v /= sum;
	}

	float inner_product(vector<float> & s1, vector<float> & s2) const {
		float ret = 0;
		for (int i = 0; i < (int)s1.size(); i++)
			ret += s1[i] * s2[i];
		return ret;
	}
	
public:	
	ShapeDis() {
		float s[][8] = {
			{ 1, 1, 0, 0, 0, 0, 0, 0 }, //1/4 corner
			{ 1, 1, 1, 0, 0, 0, 0, 0 }, //3/8 corner
			{ 1, 1, 1, 1, 0, 0, 0, 0 }, //1/2 corner
			{ 1, 1, 1, 1, 1, 0, 0, 0 }, //5/8 corner
			{ 1, 1, 1, 1, 1, 1, 0, 0 }, //3/4 corner
			{ 1, 0, 0, 0, 0, 0, 0, 0 }, //1/8 corner
			{ 1, 1, 1, 1, 1, 1, 1, 0 }  //7/8 corner
		};

		for (int i = 0; i < 7; i++) {
			vector<float> s1;
			s1.assign(&s[i][0], &s[i][8]);
			normalize(s1);
			for (int ri = 0; ri < 8; ri++) {
				int s1_idx = i * 8 + ri;
				for (int j = 0; j < 7; j++) {
					vector<float> s2;
					s2.assign(&s[j][0], &s[j][8]);
					normalize(s2);
					for (int rj = 0; rj < 8; rj++) {
						int s2_idx = j * 8 + rj;
						if (s2_idx > s1_idx)
							break;						
						float c = inner_product(s1, s2);
						cos_shape[s1_idx][s2_idx] = c;
						cos_shape[s2_idx][s1_idx] = cos_shape[s1_idx][s2_idx];
						float d = 0;
						for (int k = 0; k < (int)s1.size(); k++)
							d += (s1[k] - s2[k]) * (s1[k] - s2[k]);
						float e = d - (2 - 2 * c);
						CV_Assert(fabs(e) < 0.001);
						shift_vec(s2);
					}
				}
				shift_vec(s1);
			}
		}
	}

	float distance(float var_a, uchar shape_a, float var_b, uchar shape_b) const {
		if (shape_a >= 56)
			return var_b;
		if (shape_b >= 56)
			return var_a;
		return var_a + var_b - 2 * sqrt(var_a * var_b) * cos_shape[shape_a][shape_b];
	}

	float similar_num(Point c, Size sz, Mat & var, Mat & shape) const {
		float v0 = var.at<float>(c.y, c.x);
		uchar s0 = shape.at<uchar>(c.y, c.x);
		float cnum = 1; //similar corner number
		bool is_corner = IS_CORNER(s0);
		int ex = sz.width, ey = sz.height;
		for (int y = max(0, c.y - ey); y <= min(var.rows - 1, c.y + ey); y++) {
			float * pvar = var.ptr<float>(y);
			uchar * pshape = shape.ptr<uchar>(y);
			int endx = min(var.cols - 1, c.x + ex);
			for (int x = max(0, c.x - ex); x <= endx; x++) {
				if (is_corner && !IS_CORNER(pshape[x]) || pshape[x] == 100)
					continue;
				float d = distance(v0, s0, pvar[x], pshape[x]); //difference distance
				CV_Assert(d > -0.1);
				if (d > 0.7 * v0)
					continue;
				cnum += 1 - d / v0; //new similare corner
			}
		}
		return cnum;
	}
} shape_dis;
/*
Input image
Ouput eig1, var
Ouput eig2, corner similar
Ouput shape, var's shape
input blockSize, size for corner & var
input similar_th, it is similar threshold for corner
input var_th, it is var ratio threshold to compute similar, save time.
input ox, oy for corner search region
input ex, ey for corner move
output cpts, cpts[0] for corner, cpts[1] for edge
*/
static void my_corner_eigenval(const Mat & image, Mat & eig1, Mat & eig2, Mat & shape, 
	int blockSize, double similar_th, double var_th, int ox, int oy, vector<Point3i> cpts[])
{
	CV_Assert(image.type() == CV_8UC1 && blockSize % 2 == 1);
	if (ox < 0)
		ox = image.cols / 2;
	if (oy < 0)
		oy = image.rows / 2;
	
	eig1.create(image.size(), CV_32FC1);
	eig2.create(image.size(), CV_32FC1);
	shape.create(image.size(), CV_8UC1);
	eig1 = Scalar::all(0);
	eig2 = Scalar::all(0);
	shape = Scalar::all(100);

	Mat ig, iig;
	integral_square(image, ig, iig, Mat(), Mat(), false);

	vector<int> offset; //offset
	vector<int> b; //eighth belong
	int bs2 = blockSize / 2;
	double area = 1.0 / (blockSize * blockSize);
	float a0 = 0, a1 = 0, a2 = 0, a3 = 0;

	//prepare offset and eighth belong
	for (int y = -bs2; y <= bs2; y++)
	for (int x = -bs2; x <= bs2; x++) {
		offset.push_back((y * (int)image.step.p[0] + x * (int)image.step.p[1]) / sizeof(uchar));
		int _b = 0;
		if (y < x && y < -x)
			a1 += 1;
		else
		if (y <= x && y == -x)
			a1 += 0.25;
		else
		if (y == x && y < -x)
			a1 += 0.25;

		if (y < 0 && y < -x)
			a2 += 1;
		else
		if (y == 0 && x <= 0)
			a2 += 0.25;
		else
		if (y < 0 && y == -x)
			a2 += 0.25;

		if (x > 0)
			a3 += 1;
		else
		if (x == 0)
			a3 += 0.25;

		if (x == 0) {
			if (y < 0)
				_b = 1 << 5 | 8; //up y axis
			else
			if (y > 0)
				_b = 4 << 5 | 5; //down y axis
			else
				_b = 0;
		}
		else
		if (y == 0)
			_b = (x < 0) ? 6 << 5 | 7 : 2 << 5 | 3; //x axis
		else
		if (y == x)
			_b = (x < 0) ? 7 << 5 | 8 : 3 << 5 | 4; //y=x axis
		else
		if (y == -x)
			_b = (x < 0) ? 5 << 5 | 6 : 1 << 5 | 2; //y+x axis
		else
		if (x > 0) {
			if (y < 0)
				_b = (y < -x) ? 1 : 2;
			else
				_b = (y < x) ? 3 : 4;
		}
		else {
			if (y < 0)
				_b = (y < x) ? 8 : 7;
			else
				_b = (y < -x) ? 6 : 5;
		}
		if (_b == 1)
			a0 += 1;
		if ((_b >> 5) == 1)
			a0 += 0.25;
		b.push_back(_b);
	}

	CV_Assert(offset.size() == blockSize * blockSize);
	int offset_size = (int)offset.size();
	vector<int> stat[4];
	int total[4] = { 0 };
	for (int i = 0; i < 4; i++)
		stat[i].resize(1024, 0);
	int border = bs2 + 1;
	//compute var stat
	for (int y = border; y < image.rows - border; y++) {
		int * pig = ig.ptr<int>(y - bs2);
		int * pig1 = ig.ptr<int>(y - bs2 + blockSize);
		int * piig = iig.ptr<int>(y - bs2);
		int * piig1 = iig.ptr<int>(y - bs2 + blockSize);
		float * peig1 = eig1.ptr<float>(y);
		float * peig2 = eig2.ptr<float>(y);
		for (int x = border; x < image.cols - border; x++) {
			if (y >= oy && y < image.rows - oy && x >= ox && x < image.cols - ox)
				x = image.cols - ox;
			int s = pig[x - bs2] + pig1[x - bs2 + blockSize] - pig1[x - bs2] - pig[x - bs2 + blockSize];
			int ss = piig[x - bs2] + piig1[x - bs2 + blockSize] - piig1[x - bs2] - piig[x - bs2 + blockSize];
			float var = ss * area - s * s * area * area; //compute variance
			peig2[x] = s;
			peig1[x] = var;
			CV_Assert(var < 16384);
			int idx = (int)var / 16;
			if (y < oy) {
				stat[DIR_UP][idx]++;
				total[DIR_UP]++;
			}
			if (y >= image.rows - oy) {
				stat[DIR_DOWN][idx]++;
				total[DIR_DOWN]++;
			}
			if (x < ox) {
				stat[DIR_LEFT][idx]++;
				total[DIR_LEFT]++;
			}
			if (x >= image.cols - ox) {
				stat[DIR_RIGHT][idx]++;
				total[DIR_RIGHT]++;
			}
		}
	}

	float th[4] = { 9, 9, 9, 9 }; //var threshold for corner UP, DOWN, LEFT RIGHT
	for (int j = 0; j < 4; j++) {
		int agg = 0;
		for (int i = 1023; i > 1; i--) {
			agg += stat[j][i];
			if (agg >= total[j] * var_th) {
				th[j] = i * 8;
				break;
			}
		}
	}
	qInfo("th0=%4f, th1=%4f, th2=%4f, th3=%4f", th[0], th[1], th[2], th[3]);
	vector<Point> corner_pts;//corner_pts contain 1/4 and 3/8 corner
	vector<Point3i> edge_pts; // edge_pts contain 1/2 corner
	for (int y = border; y < image.rows - border; y++) {
		float * peig1 = eig1.ptr<float>(y);
		float * peig2 = eig2.ptr<float>(y);
		const uchar * pi = image.ptr<uchar>(y);
		uchar * pshape = shape.ptr<uchar>(y);
		int cor[31];
		for (int x = border; x < image.cols - border; x++) {
			if (y >= oy && y < image.rows - oy && x >= ox && x < image.cols - ox)
				x = image.cols - ox;
			bool pass = (y < oy && peig1[x] > th[DIR_UP] || y >= image.rows - oy && peig1[x] > th[DIR_DOWN]
				|| x < ox && peig1[x] > th[DIR_LEFT] || x >= image.cols - ox && peig1[x] > th[DIR_RIGHT]);
			if (!pass) { //< th, corner pass
				peig2[x] = 0;
				continue;
			}

			int s = peig2[x];
			float avg = (float) s / offset_size;
			const uchar * p0 = pi + x;
			int low_avg = 0, high_avg = 0, low_num = 0, high_num = 0;
			for (int i = 0; i < offset_size; i++) { //compute min, max avg
				uchar a = p0[offset[i]];
				if (a >= avg) {
					high_num++;
					high_avg += a;
				}
				else {
					low_num++;
					low_avg += a;
				}
			}
			bool choose_min = (low_num > high_num); //choose min-adjust or max-adjust	
			low_avg = low_avg / low_num;
			high_avg = high_avg / high_num;
			int bs[9] = { 0 }; //store eight sum
			int ss = 0;
			//following compute xiang guan
			for (int i = 0; i < offset_size; i++) { 
				int a = p0[offset[i]];
				a = choose_min ? a - low_avg : high_avg - a;
				if (b[i] >= 32) {
					bs[b[i] >> 5] += a / 2;
					bs[b[i] & 31] += a / 2;
				}
				else
					bs[b[i]] += a;
				ss += a * a;
			}
			bs[0] = bs[0] / 2;
			for (int i = 0; i < 7; i++)
				cor[i] = bs[0] + bs[i + 1] + bs[i + 2]; //cor[0..7] for 1/4 corner
			cor[7] = bs[0] + bs[8] + bs[1];
			for (int i = 8; i < 14; i++)
				cor[i] = bs[0] + bs[i - 7] + bs[i - 6] + bs[i - 5]; //cor[8..15] for 3/8 corner
			cor[14] = bs[0] + bs[7] + bs[8] + bs[1];
			cor[15] = bs[0] + bs[8] + bs[1] + bs[2];
			for (int i = 16; i < 21; i++) // cor[16..23] for 1/2 corner
				cor[i] = bs[0] + bs[i - 15] + bs[i - 14] + bs[i - 13] + bs[i - 12];
			cor[21] = bs[0] + bs[6] + bs[7] + bs[8] + bs[1];
			cor[22] = bs[0] + bs[7] + bs[8] + bs[1] + bs[2];
			cor[23] = bs[0] + bs[8] + bs[1] + bs[2] + bs[3];
			float mc0 = -1, mc1 = -1, mc2 = -1, mc3 = -1, max_cor, max_cor0;
			int shape0, shape1, shape2, shape3;
			for (int i = 0; i < 8; i++) 
			if (mc0 < bs[i]) {
				mc0 = bs[i];
				shape0 = 40 + i;
			}
			mc0 = mc0 * mc0 / (a0 * ss);
			for (int i = 0; i < 8; i++)
			if (mc1 < cor[i]) {
				mc1 = cor[i];
				shape1 = i; //for 1/4 corner
			}
			mc1 = mc1 * mc1 / (a1 * ss);
			for (int i = 8; i < 16; i++)
			if (mc2 < cor[i]) {
				mc2 = cor[i];
				shape2 = i; //for 3/8 corner
			}
			mc2 = mc2 * mc2 / (a2 * ss);
			for (int i = 16; i < 24; i++)
			if (mc3 < cor[i]) {
				mc3 = cor[i];
				shape3 = i; //for 1/2 corner
			}
			mc3 = mc3 * mc3 / (a3 * ss);
			max_cor = max(mc1, mc2);
			max_cor0 = max(mc3, mc0);
			CV_Assert(max_cor <= 1.001 && max_cor0 <= 1.001); //between 0 to 1			
			if (max_cor > max_cor0) {
				peig2[x] = max_cor;
				if (max_cor > similar_th)
					corner_pts.push_back(Point(x, y));
				shape1 = choose_min ? shape1 : (shape1 >= 6 ? 26 + shape1: 34 + shape1); //1/4 corner change to 3/4 corner if !choose_min
				shape2 = choose_min ? shape2 : (shape2 >= 13 ? 11 + shape2 : 19 + shape2); //3/8 corner change to 5/8 corner if !choose_min
				pshape[x] = mc1 > mc2 ? shape1 : shape2; //1/4 or 3/8 corner
			}
			else {
				peig2[x] = max_cor0;
				if (!choose_min)
					shape3 = (shape3 < 20) ? shape3 + 4 : shape3 - 4; //1/2 corner change to 1/2 corner if !choose_min
				shape0 = choose_min ? shape0 : (shape0 == 47 ? shape0 + 1 : shape0 + 9); //1/8 corner change to 7/8 corner if !choose_min
				pshape[x] = mc3 > mc0 ? shape3 : shape0; //1/8 or 1/2 corner
				if (mc3 > mc0 && max_cor0 > similar_th)
					edge_pts.push_back(Point3i(x, y, peig1[x]));
			}
		}
	}
	qDebug("Found corner=%d, edge=%d", (int)corner_pts.size(), (int)edge_pts.size());

#define T(x) ((x >= 20) ? x-4:x)
	for (auto & c : corner_pts) { //choose cpts[0] from corner_pts, reduce corner number to save MIPs later
		float * peig2 = eig2.ptr<float>(c.y, c.x);
		uchar * pshape = shape.ptr<uchar>(c.y, c.x);
		uchar * pshape1 = shape.ptr<uchar>(c.y + 1, c.x);
		uchar * pshape_1 = shape.ptr<uchar>(c.y - 1, c.x);

		bool in_void = (pshape[1] == 100 && pshape[-1] == 100) || (pshape1[0] == 100 && pshape_1[0] == 100)
				|| (pshape1[1] == 100 && pshape_1[-1] == 100) || (pshape1[-1] == 100 && pshape_1[1] == 100);
		if (in_void)
			continue;

		bool check_fake = (T(pshape_1[0]) == 16 && T(pshape1[0]) == 16 ||
			T(pshape_1[-1]) == 19 && T(pshape1[1]) == 19 ||
			T(pshape_1[1]) == 17 && T(pshape1[-1]) == 17 ||
			T(pshape[1]) == 18 && T(pshape[-1]) == 18); //in line

		if (check_fake)
			continue;
		
		bool pass = false;
		if (peig2[0] >= 8 && peig2[0] < 32) {// 3/8 or 5/8 corner, choose local best point
			if ((peig2[0] > peig2[1] || !IS_CORNER(pshape[1])) && (peig2[0] > peig2[-1] || !IS_CORNER(pshape[-1]))
				&& (peig2[0] > eig2.at<float>(c.y - 1, c.x) || !IS_CORNER(pshape_1[0]))
				&& (peig2[0] > eig2.at<float>(c.y + 1, c.x) || !IS_CORNER(pshape1[0])))
				pass = true;
		}
		else {// 1/4 or 3/4 corner, choose local best point
			CV_Assert(peig2[0] < 40);
			if (peig2[0] > peig2[1] && peig2[0] > peig2[-1]	&& peig2[0] > eig2.at<float>(c.y - 1, c.x) 
				&& peig2[0] > eig2.at<float>(c.y + 1, c.x))
				pass = true;
		}
		if (!pass)
			continue;
		cpts[0].push_back(Point3i(c.x, c.y, 2 * (peig2[0] - 0.5) * sqrt(eig1.at<float>(c.y, c.x))));
	}
	sort(cpts[0].begin(), cpts[0].end(), greaterPoint3i);
	for (auto & c : edge_pts) { //choose cpts[1] from edge_pts, reduce edge number to save MIPs later
		float * peig1 = eig2.ptr<float>(c.y, c.x);
		float * peig1_1 = eig2.ptr<float>(c.y - 1, c.x);
		float * peig11 = eig2.ptr<float>(c.y + 1, c.x);
		uchar * pshape = shape.ptr<uchar>(c.y, c.x);
		uchar * pshape1 = shape.ptr<uchar>(c.y + 1, c.x);
		uchar * pshape_1 = shape.ptr<uchar>(c.y - 1, c.x);

		bool in_void = (pshape[1] == 100 && pshape[-1] == 100) || (pshape1[0] == 100 && pshape_1[0] == 100)
			|| (pshape1[1] == 100 && pshape_1[-1] == 100) || (pshape1[-1] == 100 && pshape_1[1] == 100);
		if (in_void)
			continue;
		int check = peig1[0] > peig1[1] ? 1 : 0;
		check += peig1[0] > peig1[-1] ? 1 : 0;
		check += peig1[0] > peig11[0] ? 1 : 0;
		check += peig1[0] > peig11[-1] ? 1 : 0;
		check += peig1[0] > peig11[1] ? 1 : 0;
		check += peig1[0] > peig1_1[0] ? 1 : 0;
		check += peig1[0] > peig1_1[-1] ? 1 : 0;
		check += peig1[0] > peig1_1[1] ? 1 : 0;
		if (check >=7) //1/2 corner local best point
			cpts[1].push_back(c);
	}
#undef T(x)
	sort(cpts[1].begin(), cpts[1].end(), greaterPoint3i);
	qDebug("Filter corner=%d, edge=%d", (int)cpts[0].size(), (int)cpts[1].size());
}

struct CornerInfo {
	vector<Point3i> corners[4];  //used as good_features_to_track output para
	int max_number; //used as good_features_to_track input para
	double th;  //used as good_features_to_track input para
	double minDistance;  //used as good_features_to_track input para
};

/*
Inout img, output img's medianBlur
inout cinfo
Input scale
Input ox, oy, define corner search region, up is [0,oy), left is [0,ox), down is [h-oy,h), right is [w-ox,w)
Input sz, shift range
Output dir_stat
*/
static void good_features_to_track(Mat & img, CornerInfo * cinfo, int scale, int ox, int oy, Size sz[], bool debug_en)
{
	CV_Assert(cinfo[1].minDistance >= cinfo[0].minDistance);
	int blockSize, ksize;
	Mat eig[2], cov, shape; //eig[1] is var, eig[0] is similar

	if (ox < 0)
		ox = img.cols / 4;
	if (oy < 0)
		oy = img.rows / 4;
	qInfo("good feature to track");
	ksize = (scale == 4) ? K4_SIZE : K2_SIZE;
	medianBlur(img, img, ksize);
	Mat image;

	blockSize = (scale == 4) ? BLOCK4_SIZE : BLOCK2_SIZE;
	cinfo[0].th /= 100;
	cinfo[1].th /= 100;
	if (scale == 1) {
		resize(img, image, Size(img.cols / 2, img.rows / 2)); //save compute time
		cinfo[0].minDistance /= 2;
		cinfo[1].minDistance /= 2;
		ox = ox / 2;
		oy = oy / 2;
		for (int i = 0; i < 2; i++) {
			sz[i].width = sz[i].width / 2;
			sz[i].height = sz[i].height / 2;
		}
	}
	else {
		image = img;
		for (int i = 0; i < 2; i++) {
			sz[i].width = sz[i].width / scale;
			sz[i].height = sz[i].height / scale;
		}
	}
	vector<Point3i> cpts[2];
	my_corner_eigenval(image, eig[1], eig[0], shape, blockSize, cinfo[0].th, cinfo[1].th, ox, oy, cpts);

	int total_num = cinfo[0].max_number + cinfo[1].max_number;
	for (int eig_idx = 0; eig_idx < 2; eig_idx++) {
		CornerInfo & c = cinfo[eig_idx];
		c.minDistance *= c.minDistance;
		for (auto & v : cpts[eig_idx])
		{
			//push to different corner set
			Point3i v1 = v;
			float s0 = eig[0].at<float>(v.y, v.x);
			float v0 = eig[1].at<float>(v.y, v.x);
			/* v0 = min(v0, 500.0f); //Need at it?*/
			if (v.y < oy && c.corners[DIR_UP].size() < c.max_number * MORE_EIG_NUMBER) {//push more than needed for resort
				bool check = true;
				if (eig_idx == 1) {
					if (!push_point(cinfo[0].corners[DIR_UP], v1, c.minDistance, true) ||
						!push_point(cinfo[1].corners[DIR_UP], v1, c.minDistance, true))
						check = false;
				}
				if (check) {
					float cnum = shape_dis.similar_num(Point(v.x, v.y), sz[0], eig[1], shape);
					int ex = sz[0].width, ey = sz[0].height;
					v1.z = (s0 - 0.5) * 2 * sqrt(v0) / cnum * 2 * ex * min(oy - v.y, ey);
					c.corners[DIR_UP].push_back(v1);
				}
			}
			if (v.y > image.rows - oy && c.corners[DIR_DOWN].size() < c.max_number * MORE_EIG_NUMBER) {
				bool check = true;
				if (eig_idx == 1) {
					if (!push_point(cinfo[0].corners[DIR_DOWN], v1, c.minDistance, true) ||
						!push_point(cinfo[1].corners[DIR_DOWN], v1, c.minDistance, true))
						check = false;
				}
				if (check) {
					float cnum = shape_dis.similar_num(Point(v.x, v.y), sz[0], eig[1], shape);
					int ex = sz[0].width, ey = sz[0].height;
					v1.z = (s0 - 0.5) * 2 * sqrt(v0) / cnum * 2 * ex * min(v.y + oy - image.rows, ey);
					c.corners[DIR_DOWN].push_back(v1);
				}
			}
			if (v.x < ox && c.corners[DIR_LEFT].size() < c.max_number * MORE_EIG_NUMBER) {
				bool check = true;
				if (eig_idx == 1) {
					if (!push_point(cinfo[0].corners[DIR_LEFT], v1, c.minDistance, true) ||
						!push_point(cinfo[1].corners[DIR_LEFT], v1, c.minDistance, true))
						check = false;
				}
				if (check) {
					float cnum = shape_dis.similar_num(Point(v.x, v.y), sz[1], eig[1], shape);
					int ex = sz[1].width, ey = sz[1].height;
					v1.z = (s0 - 0.5) * 2 * sqrt(v0) / cnum * 2 * ey * min(ox - v.x, ex);
					c.corners[DIR_LEFT].push_back(v1);
				}
			}
			if (v.x > image.cols - ox && c.corners[DIR_RIGHT].size() < c.max_number * MORE_EIG_NUMBER) {
				bool check = true;
				if (eig_idx == 1) {
					if (!push_point(cinfo[0].corners[DIR_RIGHT], v1, c.minDistance, true) ||
						!push_point(cinfo[1].corners[DIR_RIGHT], v1, c.minDistance, true))
						check = false;
				}
				if (check) {
					float cnum = shape_dis.similar_num(Point(v.x, v.y), sz[1], eig[1], shape);
					int ex = sz[1].width, ey = sz[1].height;
					v1.z = (s0 - 0.5) * 2 * sqrt(v0) / cnum * 2 * ey * min(v.x + ox - image.cols, ex);
					c.corners[DIR_RIGHT].push_back(v1);
				}
			}
		}
		qDebug("Before resort eig%d, up=%d, down=%d, left=%d, right=%d", eig_idx, c.corners[DIR_UP].size(), c.corners[DIR_DOWN].size(),
			c.corners[DIR_LEFT].size(), c.corners[DIR_RIGHT].size());
		int num = (eig_idx == 0) ? c.max_number : total_num - cinfo[0].corners[DIR_UP].size();
		resort_points(c.corners[DIR_UP], num, 48 / scale, 4 / scale, image.cols / 2, c.minDistance); //only pick max_number
		num = (eig_idx == 0) ? c.max_number : total_num - cinfo[0].corners[DIR_DOWN].size();
		resort_points(c.corners[DIR_DOWN], num, 48 / scale, 4 / scale, image.cols / 2, c.minDistance);
		num = (eig_idx == 0) ? c.max_number : total_num - cinfo[0].corners[DIR_LEFT].size();
		resort_points(c.corners[DIR_LEFT], num, 48 / scale, 4 / scale, image.rows / 2, c.minDistance);
		num = (eig_idx == 0) ? c.max_number : total_num - cinfo[0].corners[DIR_RIGHT].size();
		resort_points(c.corners[DIR_RIGHT], num, 48 / scale, 4 / scale, image.rows / 2, c.minDistance);
		qDebug("After resort eig%d, up=%d, down=%d, left=%d, right=%d", eig_idx, c.corners[DIR_UP].size(), c.corners[DIR_DOWN].size(),
			c.corners[DIR_LEFT].size(), c.corners[DIR_RIGHT].size());
	}

	if (debug_en)
	for (int eig_idx = 0; eig_idx < 2; eig_idx++)
	for (int i = 0; i < 4; i++) {
		char line[1000] = { 0 };
		int idx = 0;
		for (int j = 0; j < min((int)cinfo[eig_idx].corners[i].size(), 5); j++) {
			int x = cinfo[eig_idx].corners[i][j].x;
			int y = cinfo[eig_idx].corners[i][j].y;
			idx += sprintf(line + idx, "%4d,%4d,%2d,%4.2f,%5.0f  ", y, x, (int) shape.at<uchar>(y,x),
				eig[0].at<float>(y, x), eig[1].at<float>(y, x));
		}
		qInfo(line);
	}

	for (int eig_idx = 0; eig_idx < 2; eig_idx++)
	for (int i = 0; i < 4; i++) 
	for (auto & c : cinfo[eig_idx].corners[i])
		c.z = shape.at<uchar>(c.y, c.x);
	
	if (scale == 1)
	for (int eig_idx = 0; eig_idx < 2; eig_idx++) {
		for (int i = 0; i < 4; i++)
		for (auto & v : cinfo[eig_idx].corners[i]) {
			v.x = v.x * 2;
			v.y = v.y * 2;
		}
	}
}

/*
Input e, edge.dif is mark
Input dif, dif is raw score
Input th2, minimum threshold
Output minloc, local min location
Return min value
*/
int find_local_minimum(const EdgeDiff & e, const Mat & dif, int th, Point & minloc, bool check_local_min)
{
	int mind = DIFF_AVG_VALUE;
	for (int y = 0; y < dif.rows; y++) {
		const int * pd = dif.ptr<int>(y);
		const int * pd1 = (y + 1 == dif.rows) ? pd : dif.ptr<int>(y + 1);
		const int * pd_1 = (y == 0) ? pd : dif.ptr<int>(y - 1);
		const int * pe = e.dif.ptr<int>(y);
		for (int x = 0; x < dif.cols; x++)
		if (pe[x] < 0 && pd[x] < th && pd[x] < mind) {
			if (check_local_min) {
				if (pd[x] > pd1[x] || pd[x] > pd_1[x])
					continue;
				if (x > 0 && (pd[x] > pd1[x - 1] || pd[x] > pd_1[x - 1] || pd[x] > pd[x - 1]))
					continue;
				if (x + 1 < dif.cols && (pd[x] > pd1[x + 1] || pd[x] > pd_1[x + 1] || pd[x] > pd[x + 1]))
					continue;
			}
			mind = pd[x];
			minloc = Point(x, y);
		}
	}
	return mind;
}

/*
Output e
Input score
Input filter_radius, radius aound minium point
Input bus_len, use for judge bus
Input check_bus, use for judge bus
*/
void compute_edge_type2(EdgeDiff & e, const Mat & score, int filter_radius, int bus_len, int check_bus, int max_level, bool debug_en)
{
	e.dif.create(score.size());
	e.dif = -1; //at begining baned all location
	e.avg = DIFF_AVG_VALUE;
	//find min point	
	Point minloc;
	int mind = find_local_minimum(e, score, DIFF_AVG_VALUE - 1, minloc, false);

	vector<int> mins;
	int min_th = DIFF_AVG_VALUE;
	if (mind == DIFF_AVG_VALUE || check_bus == EDGE_NEARLY_BLACK) {
		e.mind = DIFF_AVG_VALUE;
		e.minloc = Point(score.cols / 2, score.rows / 2);
		e.submind = DIFF_AVG_VALUE;
		e.edge_type = EDGE_NEARLY_BLACK;	
	}
	else {
		e.mind = mind;
		e.minloc = minloc;
		e.edge_type = EDGE_HAS_CORNER;
		if (check_bus >= EDGE_BUS_SHU && check_bus <= EDGE_BUS_XIE2) { //double check if it is bus
			min_th = min(mind + max_level * NONE_CORNER_TH_SCORE, DIFF_AVG_VALUE);
			//compute bus type
			int count[8] = { 0 }; //count for each direction
			for (int i = 0; i < 8; i++) {
				for (int j = 1; 1; j++) {
					Point loc = minloc + Point(j * dxy[i][1], j * dxy[i][0]);
					if (loc.x >= 0 && loc.x < score.cols && loc.y >= 0 && loc.y < score.rows) { //check bus len
						if (score.at<int>(loc) < min_th)
							count[i]++;
					}
					else
						break;
				}
			}
			if (check_bus == EDGE_BUS_SHU && count[0] + count[2] >= bus_len)
				e.edge_type = EDGE_BUS_SHU;
			
			if (check_bus == EDGE_BUS_HENG && count[1] + count[3] >= bus_len)
				e.edge_type = EDGE_BUS_HENG;
				
			if (check_bus == EDGE_BUS_XIE2 && count[4] + count[6] >= bus_len)
				e.edge_type = EDGE_BUS_XIE2;
				
			if (check_bus == EDGE_BUS_XIE && count[5] + count[7] >= bus_len)
				e.edge_type = EDGE_BUS_XIE;
		}
		int th1, th2;
		if (e.edge_type != EDGE_HAS_CORNER) {
			th1 = max_level * NONE_CORNER_SUBMIN_SCORE;
			th2 = max_level * NONE_CORNER_TH_SCORE;
			min_th = DIFF_AVG_VALUE;
		}
		else {
			th1 = max_level * CORNER_SUBMIN_SCORE;
			th2 = max_level * CORNER_TH_SCORE;
			min_th = DIFF_AVG_VALUE - 1;
		}
		e.submind = min(mind + th1, min_th);
		min_th = min(mind + th2, min_th);
	}
	if (e.edge_type != EDGE_NEARLY_BLACK) {
		//now compute subminloc
		vector<int> sy(score.rows, 0);
		vector<int> sx(score.cols, 0);
		for (int y = 0; y < score.rows; y++) {
			const int * ps = score.ptr<int>(y);
			for (int x = 0; x < score.cols; x++)
			if (ps[x] < e.submind) {
				sy[y] |= 2; //mark y row exist submin
				sx[x] |= 2; //mark x col exist submin
			}
			else
			if (ps[x] < min_th) {
				sy[y] |= 1;
				sx[x] |= 1;
			}
		}
		e.subminloc = Point(0, 0);
		for (int y = 0; y < score.rows; y++)
		if (sy[y] & 2)
			e.subminloc.y += 3;
		else
		if (sy[y] & 1)
			e.subminloc.y++;
		for (int x = 0; x < score.cols; x++)
		if (sx[x] & 2)
			e.subminloc.x += 3;
		else
		if (sx[x] & 1)
			e.subminloc.x++;

		while (mind < DIFF_AVG_VALUE) {
			if (debug_en)
				qDebug("Type=%d, New min=%d, minloc=(x=%d,y=%d)", e.edge_type, mind, minloc.x, minloc.y);
			mins.push_back(mind);
			//fill minloc near domain
			for (int y = max(0, minloc.y - filter_radius); y <= min(score.rows - 1, minloc.y + filter_radius); y++) {
				const int * pd = score.ptr<int>(y);
				int * pe = e.dif.ptr<int>(y);
				for (int x = max(0, minloc.x - filter_radius); x <= min(score.cols - 1, minloc.x + filter_radius); x++)
				if (pe[x] < 0) {
					pe[x] = 10000 - 10000 * (DIFF_AVG_VALUE - pd[x]) / (DIFF_AVG_VALUE - mind);
					if (pe[x] < 0) 
						pe[x] = 0;
					if (pe[x] > 10000)
						pe[x] = 10000;
					pe[x] |= (mins.size() - 1) << 16;
				}
			}
			mind = find_local_minimum(e, score, min_th, minloc, e.edge_type == EDGE_HAS_CORNER);
		}
	}
	else
		e.subminloc = Point(1000, 1000);
	
	e.min_num = (e.edge_type == EDGE_NEARLY_BLACK) ? score.cols * score.rows : mins.size();
	
	for (int y = 0; y < score.rows; y++) {
		const int * pd = score.ptr<int>(y);
		int * pe = e.dif.ptr<int>(y);
		for (int x = 0; x < score.cols; x++) {
			if (pd[x] >= DIFF_NOT_CONTACT) {
				pe[x] = DIFF_NOT_CONTACT;
				continue;
			}
			if (e.edge_type == EDGE_NEARLY_BLACK) {
				pe[x] = pd[x];
				continue;
			}
			if (pe[x] < 0)
				pe[x] = DIFF_AVG_VALUE;
			else {
				int min_idx = pe[x] >> 16;
				float coef = (pe[x] & 0xffff) / 10000.0;
				CV_Assert(min_idx < (int)mins.size() && coef < 1.001);
				float k = (min_idx + 1 == (int)mins.size()) ? (mins.back() - DIFF_AVG_VALUE) : mins[min_idx] - mins[min_idx + 1];
				pe[x] = min(pd[x], (int)(mins[min_idx] - k * coef));
			}
		}
	}
}

/*     31..24  23..16   15..8   7..0
opt0:					method layer
opt1: debug_opt method corner_num edge_num
opt2: corner_distance edge_distance corner_th edge_th
Prepare Corner for compute_weight_diff
*/
static void prepare_corner(ImageData & img_d, const ParamItem & param)
{
	int method = param.pi[0] >> 8 & 0xff;
	int debug_opt = param.pi[1] >> 24 & 0xff;
	CornerInfo cinfo[2];
	int scale = img_d.cvar->rescale;

	cinfo[0].max_number = param.pi[1] >> 8 & 0xff;
	cinfo[1].max_number = param.pi[1] & 0xff;
	cinfo[0].max_number -= cinfo[1].max_number;
	cinfo[0].minDistance = param.pi[2] >> 24 & 0xff;
	cinfo[1].minDistance = param.pi[2] >> 16 & 0xff;
	cinfo[0].minDistance /= scale;
	cinfo[1].minDistance /= scale;

	cinfo[0].th = param.pi[2] >> 8 & 0xff;
	cinfo[1].th = param.pi[2] & 0xff;

	int overlap_x = (img_d.x > 0) ? img_d.cvar->offset(img_d.y, img_d.x)[1] - img_d.cvar->offset(img_d.y, img_d.x - 1)[1]
		: img_d.cvar->offset(img_d.y, img_d.x + 1)[1] - img_d.cvar->offset(img_d.y, img_d.x)[1];
	overlap_x = img_d.v[0].d.cols - (overlap_x - img_d.cvar->max_lr_xshift) / scale;
	int overlap_y = (img_d.y > 0) ? img_d.cvar->offset(img_d.y, img_d.x)[0] - img_d.cvar->offset(img_d.y - 1, img_d.x)[0]
		: img_d.cvar->offset(img_d.y + 1, img_d.x)[0] - img_d.cvar->offset(img_d.y, img_d.x)[0];
	overlap_y = img_d.v[0].d.rows - (overlap_y - img_d.cvar->max_ud_yshift) / scale;
	Size sz[2];
	sz[0] = Size(img_d.cvar->max_ud_xshift, img_d.cvar->max_ud_yshift);
	sz[1] = Size(img_d.cvar->max_lr_xshift, img_d.cvar->max_lr_yshift);
	good_features_to_track(img_d.v[0].d, cinfo, scale, overlap_x, overlap_y, sz, debug_opt & 2);

	if (debug_opt & 1) {
		Mat cf = img_d.v[0].d.clone();
		for (int i = 0; i < 4; i++) {
			for (auto & v : cinfo[0].corners[i])
				circle(cf, Point(v.x, v.y), 1, Scalar::all(255));
			for (auto & v : cinfo[1].corners[i]) {
				circle(cf, Point(v.x, v.y), 1, Scalar::all(255));
				cf.at<uchar>(Point(v.x, v.y)) = 255;
			}
		}
		string filename = img_d.filename.substr(0, img_d.filename.find_last_of("."));
		filename += "_c.jpg";
		imwrite(filename, cf);
	}

	cinfo[0].max_number += cinfo[1].max_number;
	
	for (int i = 0; i < 4; i++) {
		for (auto v : cinfo[1].corners[i])
		if (cinfo[0].corners[i].size() < cinfo[0].max_number) 			
			cinfo[0].corners[i].push_back(v); //append cinfo[1] to cinfo[0]			
		img_d.v[i + 1].d = Mat(cinfo[0].corners[i], true); //change vector to mat
	}
}

#define SCALE_COMPARE_RANGE(scale) ((scale == 4) ? BLOCK4_SIZE / 2 + 1 : (scale == 2) ? BLOCK2_SIZE / 2 + 1 : BLOCK2_SIZE / 2 + 3)

static void prepare_gray_vec(Mat & m, vector<int> & offset, int scale)
{
	int d = SCALE_COMPARE_RANGE(scale); //should be same as get_gray_vec
	CV_Assert(m.type() == CV_8UC1);
	offset.clear();
	for (int y = -d; y <= d; y++)
	for (int x = -d; x <= d; x++)
		offset.push_back((y * (int)m.step.p[0] + x * (int)m.step.p[1]) / sizeof(uchar));

}
//return average gray
static bool get_gray_vec(const Mat & m, Point c, vector<float> & g, const vector<int> & offset, int scale)
{
	int d = SCALE_COMPARE_RANGE(scale); //should be same as prepare_gray_vec and abs_diff_gray
	if (c.y < d || c.x < d || c.x + d >= m.cols || c.y + d >= m.rows)
		return false;
	g.resize(offset.size());
	float sum = 0;
	const uchar * pm = m.ptr<uchar>(c.y, c.x);
	for (int i = 0; i < (int)g.size(); i++) {
		g[i] = pm[offset[i]];
		sum += g[i];
	}
	float avg = sum / ((2 * d + 1) * (2 * d + 1));
	for (auto & dg : g)
		dg -= avg;
	return true;
}

static float abs_diff(const vector<float> & g1, const vector<float> &g2)
{
	float sum = 0;
	for (int i = 0; i < g1.size(); i++) {
		float dg = g1[i] - g2[i];
		sum += dg * dg;
	}
	return sum;
}
/*
abs_diff_gray means get_gray_vec and abs_diff, merge in one lopp to save time
*/
static float abs_diff_gray(const Mat & m, Point c, const vector<float> & g, const vector<int> & offset, int scale)
{
	int d = SCALE_COMPARE_RANGE(scale); //should be same as prepare_gray_vec
	if (c.y < d || c.x < d || c.x + d >= m.cols || c.y + d >= m.rows)
		return -1;
	float sum = 0, ss = 0;
	const uchar * pm = m.ptr<uchar>(c.y, c.x);
	for (int i = 0; i < (int)g.size(); i++) {
		float dg = g[i] - pm[offset[i]];
		ss += dg * dg;
		sum += pm[offset[i]];
	}
	return ss - sum * sum / g.size();
}

static float abs(vector<float> & g1)
{
	float sum = 0;
	for (int i = 0; i < g1.size(); i++) {
		float dg = g1[i];
		sum += dg * dg;
	}
	return sum;
}

#define FILTER_R(s) ((s == 4) ? 2 : (s == 2) ? 2 : 3)
#define BUS_LEN(s) ((s == 4) ? 13 : (s == 2) ? 15 : 22)

/*
Input snr
Input n1, 5 score stage
Input n2, 0 score stage
Input th, 
Output score
Return 1 if min_snr < th, give bonus; else return 0, don't give bonus
*/
static int bonus(const Mat_<Vec2f> & snr, float n1, float n2, float th, float top_bonus, Mat & score, int debug_en)
{
	CV_Assert(snr.size() == score.size() && n1 > 0 && n2 > 0);
	float min0 = 10000, min1 = 10000, min2 = 10000;
	for (int y = 0; y < snr.rows; y++) {
		const Vec2f *pv = snr.ptr<Vec2f>(y);
		for (int x = 0; x < snr.cols; x++) {
			float n = pv[x][0] / pv[x][1];
			min0 = min(min0, n);
			n = (pv[x][0] + n1) / pv[x][1];
			min1 = min(min1, n);
			n = (pv[x][0] + n2) / pv[x][1];
			min2 = min(min2, n);
		}
	}
	if (min0 > th || min2 > 0.9)
		return 0;
	if (debug_en)
		qDebug("min0=%f, min1=%f, min2=%f", min0, min1, min2);
	float half_bonus = top_bonus / 2;
	for (int y = 0; y < snr.rows; y++) {
		const Vec2f *pv = snr.ptr<Vec2f>(y);
		int * pscore = score.ptr<int>(y);
		for (int x = 0; x < snr.cols; x++) {
			float n = pv[x][0] / pv[x][1];
			if (n > min2)
				continue;
			if (n > min1) {
				n = (n - min1) * half_bonus / (min2 - min1);
				n += half_bonus;
			}
			else
				n = (n - min0) * half_bonus / (min1 - min0);
			int s = top_bonus + 0.49 - n;
			CV_Assert(s >= 0 && s <= top_bonus);
			pscore[x] += s;
		}
	}
	return 1;
}

/*
      31..24  23..16   15..8   7..0
opt0:				   method layer
opt1: debug_opt method(8) var_th(16)
opt2: lvl_th(8) hit_th(8) level5(8) level10(8)
opt3:                 var_th2(8) edge_reduce(8)
*/
static void compute_weight_diff(const ImageDiff & gd, const ParamItem & param)
{
	int debug_opt = param.pi[1] >> 24 & 0xff;
	int method = param.pi[0] >> 8 & 0xff;
	float var_th = param.pi[1] & 0xffff;
	int lvl_th = param.pi[2] >> 24 & 0xff;
	int hit_th = param.pi[2] >> 16 & 0xff;
	float level5 = param.pi[2] >> 8 & 0xff;
	level5 /= 10;
	float level10 = param.pi[2] & 0xff;
	level10 /= 10;
	float edge_reduce = param.pi[3] & 0xff;
	int var_th2 = param.pi[3] >> 8 & 0xff;
	edge_reduce /= 100;
	int scale = gd.cvar->rescale;
	int x_shift = (gd.dir) ? gd.cvar->max_lr_xshift / scale : gd.cvar->max_ud_xshift / scale; //dif size x
	int y_shift = (gd.dir) ? gd.cvar->max_lr_yshift / scale : gd.cvar->max_ud_yshift / scale; //dif size y
	int x_org0 = (gd.dir) ? gd.img_d0->v[0].d.cols - gd.overlap / scale : gd.shift / scale; //image initial bias
	int y_org0 = (gd.dir) ? gd.shift / scale : gd.img_d0->v[0].d.rows - gd.overlap / scale;
	gd.e->offset.x = (x_org0 - x_shift) * scale;
	gd.e->offset.y = (y_org0 - y_shift) * scale;

	qDebug("compute_weight_diff for img1=%s, img2=%s", gd.img_d0->filename.c_str(), gd.img_d1->filename.c_str());
	if (gd.img_d0->is_black_img || gd.img_d1->is_black_img || gd.overlap <= 0) {
		gd.e->dif.create(2 * y_shift + 1, 2 * x_shift + 1); //start compute dif
		gd.e->img_num = 0;
		gd.e->edge_type = EDGE_BLACK;
		gd.e->mind = gd.e->submind = DIFF_AVG_VALUE;
		gd.e->minloc = Point(x_shift, y_shift);
		gd.e->subminloc = Point(0, 0);
		gd.e->dif = DIFF_AVG_VALUE;
		gd.e->min_num = (2 * x_shift + 1) * (2 * y_shift + 1);
		return;
	}
	Mat_<Vec2f> snr1(2 * y_shift + 1, 2 * x_shift + 1); 
	Mat_<Vec2f> snr2(2 * y_shift + 1, 2 * x_shift + 1);
	Mat_<Vec2f> hit_rate[HIT_LEVEL];
	Mat score(2 * y_shift + 1, 2 * x_shift + 1, CV_32SC1);
	score = Scalar::all(0);
	for (int i = 0; i < HIT_LEVEL; i++)
		hit_rate[i].create(2 * y_shift + 1, 2 * x_shift + 1);

	Mat img[2];
	img[0] = gd.img_d0->v[0].d, img[1] = gd.img_d1->v[0].d; //load image
	CV_Assert(img[0].type() == img[1].type() && img[0].size() == img[1].size() && img[0].type() == CV_8UC1);
	vector<Point3i> corners[2]; //load corner
	int d0_idx, d1_idx;
	if (gd.dir) {
		d0_idx = DIR_RIGHT;
		d1_idx = DIR_LEFT;
	}
	else {
		d0_idx = DIR_DOWN;
		d1_idx = DIR_UP;
	}	
	corners[0] = Mat_<Point3i>(gd.img_d0->v[1 + d0_idx].d); //corners[0] store img[0] corner
	corners[1] = Mat_<Point3i>(gd.img_d1->v[1 + d1_idx].d); //corners[1] store img[1] corner

	vector<int> offset;
	prepare_gray_vec(img[0], offset, scale);
	vector<vector<float> > corner_gray[2]; //corner_gray is corner nearby gray "feature vector"
	vector<float> corner_gray_abs[2]; //corner_gray_abs = sum{corner_gray^2}
	vector<float> corner_var_th[2]; //corner weight
	vector<float> arrow[2]; //if arrow's var big enough, it is 1.0 else it < 1.0
	vector<int> is_edge[2];

	int dir_eng[4] = { 0 }; //dir_eng[UP] store img0 + img1 UP energe
	int tot_eng = 0;
	float tot_arrow = 0;
	int tot_edge = 0;
	for (int i = 0; i < 2; i++) { //prepare corner_gray and corner_gray_abs
		for (const auto & c : corners[i]) {
			vector<float> g;
			if (!get_gray_vec(img[i], Point(c.x, c.y), g, offset, scale)) //get corner nearby feature
				CV_Assert(0);
			corner_gray[i].push_back(g);
			float var = abs(g);
			corner_gray_abs[i].push_back(var); //get absolute var
			var = var / g.size();			
			if (IS_CORNER(c.z))
				is_edge[i].push_back(0);
			else {
				is_edge[i].push_back(1);
				var *= edge_reduce; //reduce edge var
			}
			if (var > var_th)
				var = pow(var * var_th * var_th * var_th, 0.25); //limite corner and edge var
			corner_var_th[i].push_back(var);
			tot_eng += var;
			if (!IS_CORNER(c.z)) {
				dir_eng[shape_dir[c.z] & 0xf] += var;
				tot_edge++;
			}
		}
	}
	for (int i = 0; i < 2; i++) 
	if (!corner_var_th[i].empty()) {
		int stat_var[200] = { 0 };
		for (auto v : corner_var_th[i]) {
			int idx = v / 4;
			idx = min(idx, 199);
			stat_var[idx]++;
		}
		int temp_sum = 0, new_var_th2;
		for (int j = 0; j < 200; j++) {
			 temp_sum += stat_var[j];
			 if (temp_sum > corner_var_th[i].size() / 2) {
				 new_var_th2 = min(j * 4, var_th2);
				 break;
			 }
		}

		for (int j = 0; j < (int)corner_var_th[i].size(); j++) {
			arrow[i].push_back((corner_var_th[i][j] > new_var_th2) ? 1 : corner_var_th[i][j] / new_var_th2);
			tot_arrow += arrow[i].back();
		}
	}
	int total_dir_eng = dir_eng[0] + dir_eng[1] + dir_eng[2] + dir_eng[3];
	int max_dir, max_dir_eng, other_dir_eng;
	int avg_eng = (tot_edge == 0) ? 0 : total_dir_eng / tot_edge;
	max_dir = max_element(dir_eng, dir_eng + 4) - dir_eng;
	max_dir_eng = dir_eng[max_dir];
	other_dir_eng = total_dir_eng - max_dir_eng;
	qDebug("Total arrow %d, tot_eng=%d, max_d_eng=%d, oth_d_eng=%d, valid_arrow=%f, avg_eng=%d", corners[0].size() + corners[1].size(),
		tot_eng, max_dir_eng, other_dir_eng, tot_arrow, avg_eng);
	int check_bus = EDGE_HAS_CORNER;
	if (max_dir_eng > other_dir_eng * 6) {
		check_bus = (max_dir == 0) ? EDGE_BUS_SHU :
			(max_dir == 1) ? EDGE_BUS_HENG :
			(max_dir == 2) ? EDGE_BUS_XIE2 : EDGE_BUS_XIE;
		level5 += 0.5;
		level10 += 1.0;
	}

	for (int yy = -y_shift; yy <= y_shift; yy++)
	for (int xx = -x_shift; xx <= x_shift; xx++) {
		int y_org = y_org0 + yy; //(x_org, y_org) is shift between img[0] and img[1]
		int x_org = x_org0 + xx;
		int y_overlap = img[0].rows - abs(y_org);
		int x_overlap = img[0].cols - abs(x_org);
		bool out_border = false;
		if (x_overlap >= MIN_OVERLAP && y_overlap >= MIN_OVERLAP) {	
			float fenzi = 0, fenmu = 0, fenzi2 = 0, fenmu2 = 0; //compute snr, fenzi is noise, fenmu is signal
			int hit[HIT_LEVEL] = { 0 }; //arrow hit for snr = 50%, 40%, 30%, 20%, 10%
			int tot_hit = 0; //total hit arrow number
			for (int i = 0; i < 2; i++) {
				int xs = (i == 0) ? x_org : -x_org;
				int ys = (i == 0) ? y_org : -y_org;
				for (int j = 0; j < (int) corners[1 - i].size(); j++) {
					Point3i c = corners[1 - i][j];
					float r = abs_diff_gray(img[i], Point(c.x + xs, c.y + ys), corner_gray[1 - i][j], offset, scale);
					if (r < 0)  //corner out of range						
						continue;
					
					r = r / corner_gray_abs[1 - i][j]; //compute SNR
					r = min(r, 3.0f);
					int h_idx = r / 0.1f;
					for (int k = 0; k < HIT_LEVEL; k++)
					if (k >= h_idx)
						hit[k] += arrow[1 - i][j];
					tot_hit += arrow[1 - i][j];
					fenzi += r * corner_var_th[1 - i][j];
					fenmu += corner_var_th[1 - i][j];
					if (is_edge[1 - i][j]) {
						fenzi2 += r * corner_var_th[1 - i][j];
						fenmu2 += corner_var_th[1 - i][j];
					}
				}				
			}
			if (tot_hit < tot_arrow / 5 || tot_hit < hit_th)
				out_border = true;
			else {
				CV_Assert(fenmu > 0);
				for (int i = 0; i < HIT_LEVEL; i++)
					hit_rate[i](yy + y_shift, xx + x_shift) = Vec2f(tot_hit - hit[i], tot_hit);
				snr1(yy + y_shift, xx + x_shift) = Vec2f(fenzi, fenmu);
				snr2(yy + y_shift, xx + x_shift) = Vec2f(fenzi2, fenmu2);
			}
		}
		else {
			out_border = true;
			score.at<int>(yy + y_shift, xx + x_shift) = -DIFF_NOT_CONTACT;
		}
		if (out_border) {
			for (int i = 0; i < HIT_LEVEL; i++)
				hit_rate[i](yy + y_shift, xx + x_shift) = Vec2f(100, 100);
			snr1(yy + y_shift, xx + x_shift) = Vec2f(10000, 10000);
			snr2(yy + y_shift, xx + x_shift) = Vec2f(10000, 10000);
		}
	}
	
	int max_level = 0;

	for (int i = 0; i < HIT_LEVEL; i++) {
		max_level += bonus(hit_rate[i], level5, level10, 0.2, 10, score, debug_opt & 1);
		if (max_level >= 5)
			break;
	}
	if (max_level <= 1)
		check_bus = EDGE_NEARLY_BLACK;
	if (avg_eng > 0) {
		max_level += bonus(snr1, level5 * avg_eng, level10 * avg_eng, 0.6, 20, score, debug_opt & 1) * 2;
		//max_level += bonus(snr2, level5 * avg_eng, level10 * avg_eng, 0.6, 10, score, debug_opt & 1);
	}
	gd.e->dif.create(2 * y_shift + 1, 2 * x_shift + 1);
	score = DIFF_AVG_VALUE - score;
	max_level = max(lvl_th, max_level);
	//gd.e->dif = score;
	//gd.e->compute_score();
	compute_edge_type2(*(gd.e), score, FILTER_R(scale), BUS_LEN(scale), check_bus, max_level, debug_opt & 1);
}

static void edge_mixer(const Mat & img_in, const Mat & mask, Mat & img_out, char lut[], int lut_size)
{
    CV_Assert(img_in.type() == CV_16SC1 && mask.type() == CV_8UC1);
    Mat out(img_in.rows, img_in.cols, CV_8SC1);
    for (int y = 0; y < img_in.rows; y++) {
        const short * p_img_in = img_in.ptr<short>(y);
        const unsigned char * p_mask = mask.ptr<unsigned char>(y);
        char * p_img_out = out.ptr<char>(y);
        for (int x = 0; x < img_in.cols; x++)
		if (p_mask[x]) {
			p_img_out[x] = (abs(p_img_in[x]) >= lut_size) ? lut[lut_size - 1] : lut[abs((int)p_img_in[x])];
			p_img_out[x] = p_img_in[x] > 0 ? p_img_out[x] : -p_img_out[x];
		}
        else
            p_img_out[x] = 0;

    }
    img_out = out;
}


/*     31..24  23..16   15..8   7..0
opt0:							layer
opt1: debug_opt method  opidx_src opidx_gy opidx_gx
opt2: canny_high_th canny_low_th bfilt_csigma bfilt_w
opt3: alpha edge_dialte sobel_th sobel_w
*/
static void prepare_grad(ImageData & img_d, const ParamItem & param)
{
	int debug_opt = param.pi[1] >> 24 & 0xff;
	int opidx_src = param.pi[1] >> 8 & 0xf;
	int opidx_gy = param.pi[1] >> 4 & 0xf;
	int opidx_gx = param.pi[1] & 0xf;
	int bfilt_w = param.pi[2] & 0xff;
	int bfilt_csigma = param.pi[2] >> 8 & 0xff;
	int canny_low_th = param.pi[2] >> 16 & 0xff;
	int canny_high_th = param.pi[2] >> 24 & 0xff;
	int sobel_w = param.pi[3] & 0xff;
	int sobel_th = param.pi[3] >> 8 & 0xff;
	int edge_dialte = param.pi[3] >> 16 & 0xff;
	float alpha = (param.pi[3] >> 24 & 0xff) / 100.0;
	const ConfigPara * cvar = img_d.cvar;

	qDebug("prepare_grad, opidx_src=%d, opidx_gy=%d, opidx_gx=%d, bfilt_w=%d, bfilt_csigma=%d", 
		opidx_src, opidx_gy, opidx_gx, bfilt_w, bfilt_csigma);
	qDebug("canny_low_th=%d, canny_high_th=%d, sobel_w=%d, sobel_th=%d, edge_dialte=%d, alpha=%f",
		canny_low_th, canny_high_th, sobel_w, sobel_th, edge_dialte, alpha);

	if (opidx_gy == opidx_gx) {
		qCritical("wrong opidx_gx == opidx_gy");
		return;
	}
	Mat & img_in = img_d.v[opidx_src].d;
	Mat & grad_x = img_d.v[opidx_gx].d;
	Mat & grad_y = img_d.v[opidx_gy].d;

	char lut[256];
	for (int i = 0; i < sizeof(lut) / sizeof(lut[0]); i++) {
		if (i <= sobel_th)
			lut[i] = 0;
		else
			lut[i] = pow(i - sobel_th, alpha) + 0.5;
	}

    Mat filt_mat, edge_mask;

	if (bfilt_w > cvar->rescale)
		bilateralFilter(img_in, filt_mat, bfilt_w / cvar->rescale, bfilt_csigma, 0);
	else
		filt_mat = img_in;

	if (canny_low_th > 0) {
		Canny(filt_mat, edge_mask, canny_low_th, canny_high_th, 3);
		if (edge_dialte)
			dilate(edge_mask, edge_mask, Mat());
	}
	else {
		edge_mask.create(filt_mat.rows, filt_mat.cols, CV_8UC1);
		edge_mask = Scalar::all(1);
	}

    Sobel(filt_mat, grad_x, CV_16S, 1, 0, sobel_w);
    Sobel(filt_mat, grad_y, CV_16S, 0, 1, sobel_w);

	edge_mixer(grad_x, edge_mask, grad_x, lut, sizeof(lut) / sizeof(lut[0]));
	edge_mixer(grad_y, edge_mask, grad_y, lut, sizeof(lut) / sizeof(lut[0]));
	CV_Assert(grad_x.rows == img_in.rows && grad_x.cols == img_in.cols);
}

/*     31..24  23..16   15..8   7..0
opt0:							layer
opt1: debug_opt  method   th     ratio
*/
void detect_black_img(ImageData & img_d, const ParamItem & param)
{
	int th = param.pi[1] >> 8 & 0xff;
	unsigned ratio = param.pi[1] & 0xff;
	Mat & m0 = img_d.v[0].d;
	CV_Assert(m0.type() == CV_8UC1);
	unsigned long long sum2 = 0;
	for (int y = 0; y < m0.rows; y++) { //y,x is grad_x0, grad_y0
		const unsigned char * pm0 = m0.ptr<unsigned char>(y);
		for (int x = 0; x < m0.cols; x++) {
			if (pm0[x] > th)
				sum2++;
		}
	}
	unsigned long long sum1 = m0.rows * m0.cols;

	if (sum2 * 1000 < sum1 * ratio) {
		img_d.is_black_img = true;
		qInfo("detect_black_img %s", img_d.filename.c_str());
	}
	else
		img_d.is_black_img = false;
}
/*     31..24  23..16   15..8   7..0
opt0:							layer
opt1: debug_opt method  opidx_diff3 opidx_diff2 opidx_diff1 opidx_diff0
opt2: mix_diff2 mix_diff2 mix_diff1 mix_diff0
*/
void compute_diff(const ImageDiff & img_diff, const ParamItem & param, const Rect & r0, const Rect &r1, int xshift, int yshift, EdgeDiff * e, int step = 1)
{
	int opidx[4] = { param.pi[1] >> 12 & 0xf, param.pi[1] >> 8 & 0xf, param.pi[1] >> 4 & 0xf, param.pi[1] & 0xf };
	int mix[4] = { param.pi[2] >> 24 & 0xff, param.pi[2] >> 16 & 0xff, param.pi[2] >> 8 & 0xff, param.pi[2] & 0xff };
	qDebug("compute_diff for img1=%s, img2=%s", img_diff.img_d0->filename.c_str(), img_diff.img_d1->filename.c_str());
	qDebug("compute_diff, op0=%d, op1=%d, op2=%d, op3=%d, mix0=%d, mix1=%d, mix2=%d, mix3=%d",		
		opidx[0], opidx[1], opidx[2], opidx[3], mix[0], mix[1], mix[2], mix[3]);
    vector<int> out((2 * yshift + 1)*(2 * xshift + 1), 0);
    vector<int> num((2 * yshift + 1)*(2 * xshift + 1), 0);
    vector<double> normal_out((2 * yshift + 1)*(2 * xshift + 1));

	if (img_diff.img_d0->is_black_img || img_diff.img_d1->is_black_img || r0.height <= 0 || r0.width <= 0 || img_diff.overlap <=0) {
		e->img_num = 0;
		e->dif.create(2 * yshift + 1, 2 * xshift + 1);
		e->dif = 0;
		e->compute_score();
		return;
	}
	for (int i = 0; i < 4; i++)
	if (mix[i] > 0) {
		Mat m0 = img_diff.img_d0->v[opidx[i]].d(r0);
		Mat m1 = img_diff.img_d1->v[opidx[i]].d(r1);		
		CV_Assert(m0.type() == m1.type());
		if (m0.type() == CV_8SC1)
		for (int y = 0; y < m0.rows; y += step) { //y,x is grad_x0, grad_y0
			const char * pm0 = m0.ptr<char>(y);
			for (int x = 0; x < m0.cols; x += step) {
				int y_org = y - yshift;
				int x_org = x - xshift;
				//Following scan [-yshift, yshift] * [-xshift, xshift], it is for good cache hit
				for (int yy = max(0, y_org); yy < min(m1.rows, y + yshift + 1); yy++) { 
					int * pout = &out[(yy - y_org) * (2 * xshift + 1)];
					int * pnum = &num[(yy - y_org) * (2 * xshift + 1)];
					const char * pm1 = m1.ptr<char>(yy);
					for (int xx = max(0, x_org); xx < min(m1.cols, x + xshift + 1); xx++) {
						int a0 = pm0[x], a1 = pm1[xx];
						pout[xx - x_org] += abs(a0 - a1) * mix[i];
						pnum[xx - x_org]++;
					}
				}
			}
		}
		if (m0.type() == CV_8UC1)
		for (int y = 0; y < m0.rows; y += step) { //y,x is grad_x0, grad_y0
			const unsigned char * pm0 = m0.ptr<unsigned char>(y);
			for (int x = 0; x < m0.cols; x += step) {
				int y_org = y - yshift;
				int x_org = x - xshift;
				//Following scan [-yshift, yshift] * [-xshift, xshift], it is good for cache hit
				for (int yy = max(0, y_org); yy < min(m1.rows, y + yshift + 1); yy++) {
					int * pout = &out[(yy - y_org) * (2 * xshift + 1)];
					int * pnum = &num[(yy - y_org) * (2 * xshift + 1)];
					const unsigned char * pm1 = m1.ptr<unsigned char>(yy);
					for (int xx = max(0, x_org); xx < min(m1.cols, x + xshift + 1); xx++) {
						int a0 = pm0[x], a1 = pm1[xx];
						pout[xx - x_org] += abs(a0 - a1) * mix[i];
						pnum[xx - x_org]++;
					}
				}
			}
		}

		if (m0.type() == CV_8UC3)
		for (int y = 0; y < m0.rows; y += step) { //y,x is grad_x0, grad_y0
			const unsigned char * pm0 = m0.ptr<unsigned char>(y);
			for (int x = 0; x < m0.cols; x += step) {
				int y_org = y - yshift;
				int x_org = x - xshift;
				//Following scan [-yshift, yshift] * [-xshift, xshift], it is good for cache hit
				for (int yy = max(0, y_org); yy < min(m1.rows, y + yshift + 1); yy++) {
					int * pout = &out[(yy - y_org) * (2 * xshift + 1)];
					int * pnum = &num[(yy - y_org) * (2 * xshift + 1)];
					const unsigned char * pm1 = m1.ptr<unsigned char>(yy);
					for (int xx = max(0, x_org); xx < min(m1.cols, x + xshift + 1); xx++) {
						const unsigned char * ppm0 = &pm0[3 * x];
						const unsigned char * ppm1 = &pm1[3 * xx];
						int a0 = ppm0[0], b0 = ppm1[0];
						int a1 = ppm0[1], b1 = ppm1[1];
						int a2 = ppm0[2], b2 = ppm1[2];
						pout[xx - x_org] += (abs(a0 - b0) + abs(a1 - b1) + abs(a2 - b2)) * mix[i];
						pnum[xx - x_org]++;
					}
				}
			}
		}
	}

	for (int i = 0; i < out.size(); i++) 
	if (num[i]==0)
		normal_out[i] = DIFF_NOT_CONTACT / 100;
	else {
		CV_Assert(out[i] >= 0);
		normal_out[i] = (double)out[i] / num[i];
	}
    e->dif.create(2 * yshift + 1, 2 * xshift + 1);
    for (int y = 0; y < 2 * yshift + 1; y++) {
		int * pdiff = e->dif.ptr<int>(2 * yshift - y);
        double * pout = &normal_out[y*(2 * xshift + 1)];
        for (int x = 0; x < 2 * xshift + 1; x++)
			pdiff[2 * xshift - x] = pout[x] * 100;
    }
	e->compute_score();
	if (e->avg == 0) {
		qDebug("compute_diff found black img1=%s, img2=%s", img_diff.img_d0->filename.c_str(), img_diff.img_d1->filename.c_str());
		e->img_num = 0;	
	}		
}

void write_mat_binary(FILE * fp, unsigned short para0, unsigned short para1, const Mat &m)
{
    CV_Assert(m.type() == CV_8UC1 && m.isContinuous());
    unsigned short para[4];
    para[0] = para0;
    para[1] = para1;
    para[2] = m.rows;
    para[3] = m.cols;
    fwrite(para, sizeof(para), 1, fp);
    fwrite(m.data, m.rows*m.cols, 1, fp);
}


int read_mat_binary(FILE * fp, unsigned short &para0, unsigned short &para1, Mat &m)
{
	unsigned short para[4];
	int row, col;

	if (feof(fp))
		return -1;
	if (fread(para, sizeof(para), 1, fp) != 1)
		return -1;
	para0 = para[0];
	para1 = para[1];
	row = para[2];
	col = para[3];
	m.create(row, col, CV_8UC1);
	fread(m.data, m.rows*m.cols, 1, fp);
	return 0;
}

void prepare_extract(ImageData & img_dat)
{
	try {
		Mat img_in = imread(img_dat.filename, 0);
		if (img_in.empty())
			return;
		qInfo("prepare img %s", img_dat.filename.c_str());
		const ConfigPara * cvar = img_dat.cvar;
		Mat tailor_mat = img_in(Rect(cvar->clip_l, cvar->clip_u, img_in.cols - cvar->clip_l - cvar->clip_r, img_in.rows - cvar->clip_u - cvar->clip_d));
		if (cvar->rescale != 1)
			resize(tailor_mat, img_dat.v[0].d, Size(tailor_mat.cols / cvar->rescale, tailor_mat.rows / cvar->rescale));
		else
			img_dat.v[0].d = tailor_mat;

		for (int i = 0; i < img_dat.tvar->params.size(); i++) {
			int method = img_dat.tvar->params[i].pi[1] >> 16 & 0xff;
			if (method >= PP_END)
				break;
			switch (method) {
			case PP_COMPUTE_GRAD:
				prepare_grad(img_dat, img_dat.tvar->params[i]);
				break;
			case PP_DETECT_BLACK_IMG:
				detect_black_img(img_dat, img_dat.tvar->params[i]);
				break;
			case PP_COMPUTE_CORNER:
				prepare_corner(img_dat, img_dat.tvar->params[i]);
				break;
			}
		}
	}
	catch (std::exception & e) {
		qFatal("Error in prepare img %s, Exception %s.", img_dat.filename.c_str(), e.what());
	}
}

//diff and erode
void compute_diff1(const ImageDiff & gd, const ParamItem & param)
{
	int x_shift = (gd.dir) ? gd.cvar->max_lr_xshift / gd.cvar->rescale : gd.cvar->max_ud_xshift / gd.cvar->rescale;
	int y_shift = (gd.dir) ? gd.cvar->max_lr_yshift / gd.cvar->rescale : gd.cvar->max_ud_yshift / gd.cvar->rescale;
	int x_org0 = (gd.dir) ? gd.img_d0->v[0].d.cols - gd.overlap / gd.cvar->rescale : gd.shift / gd.cvar->rescale;
	int y_org0 = (gd.dir) ? gd.shift / gd.cvar->rescale : gd.img_d0->v[0].d.rows - gd.overlap / gd.cvar->rescale;
	int opidx[4] = { param.pi[1] >> 12 & 0xf, param.pi[1] >> 8 & 0xf, param.pi[1] >> 4 & 0xf, param.pi[1] & 0xf };
	int mix[4] = { param.pi[2] >> 24 & 0xff, param.pi[2] >> 16 & 0xff, param.pi[2] >> 8 & 0xff, param.pi[2] & 0xff };
	gd.e->offset.x = (x_org0 - x_shift) * gd.cvar->rescale;
	gd.e->offset.y = (y_org0 - y_shift) * gd.cvar->rescale;
	qDebug("compute_diff for img1=%s, img2=%s", gd.img_d0->filename.c_str(), gd.img_d1->filename.c_str());
	qDebug("compute_diff, op0=%d, op1=%d, op2=%d, op3=%d, mix0=%d, mix1=%d, mix2=%d, mix3=%d",
		opidx[0], opidx[1], opidx[2], opidx[3], mix[0], mix[1], mix[2], mix[3]);
	gd.e->dif.create(2 * y_shift + 1, 2 * x_shift + 1);
	if (gd.img_d0->is_black_img || gd.img_d1->is_black_img || gd.overlap <= 0) {
		gd.e->img_num = 0;
		gd.e->dif = 0;
		gd.e->compute_score();
		return;
	}
	for (int i = 0; i < 4; i++)
	if (mix[i] > 0) {
		Mat m0 = gd.img_d0->v[opidx[i]].d;
		Mat m1 = gd.img_d1->v[opidx[i]].d;
		CV_Assert(m0.type() == m1.type() && m0.size() == m1.size());
		if (m0.type() == CV_8SC1)
		for (int yy = -y_shift; yy <= y_shift; yy++)
		for (int xx = -x_shift; xx <= x_shift; xx++) {
			int y_org = y_org0 + yy;
			int x_org = x_org0 + xx;
			int y_overlap = m0.rows - abs(y_org);
			int x_overlap = m0.cols - abs(x_org);
			unsigned sum;
            if (x_overlap >= MIN_OVERLAP && y_overlap >= MIN_OVERLAP) {
				vector<int> last_row(x_overlap + 2, 0x10000000), this_row(x_overlap + 2), last_row_m(x_overlap + 2, 0x10000000);
				this_row[0] = 0x10000000, this_row[x_overlap + 1] = 0x10000000;
				for (int y = 0; y < y_overlap; y++) {
					const char * pm0 = m0.ptr<char>(y + max(0, y_org), max(0, x_org));
					const char * pm1 = m1.ptr<char>(y + max(0, -y_org), max(0, -x_org));
					for (int x = 0; x < x_overlap; x++) {
						int a0 = pm0[x], a1 = pm1[x];
						this_row[x + 1] = abs(a0 - a1);
					}
					for (int x = 0; x < x_overlap; x++) {
						sum += min(last_row_m[x + 1], this_row[x + 1]) * mix[i];
						last_row_m[x + 1] = min(min(last_row[x + 1], this_row[x + 1]), min(this_row[x], this_row[x + 2]));
						last_row[x + 1] = this_row[x + 1];
					}
					if (y == 0)
						sum = 0;
				}
				for (int x = 0; x < x_overlap; x++)
					sum += last_row_m[x + 1] * mix[i];
			}
            if (y_overlap >= MIN_OVERLAP && x_overlap >= MIN_OVERLAP)
				gd.e->dif(yy + y_shift, xx + x_shift) = sum * 100.0 / (x_overlap * y_overlap);
			else
				gd.e->dif(yy + y_shift, xx + x_shift) = DIFF_NOT_CONTACT;
		}
		if (m0.type() == CV_8UC1)
		for (int yy = -y_shift; yy <= y_shift; yy++) 
		for (int xx = -x_shift; xx <= x_shift; xx++) {
			int y_org = y_org0 + yy;
			int x_org = x_org0 + xx;
			int y_overlap = m0.rows - abs(y_org);
			int x_overlap = m0.cols - abs(x_org);
			unsigned sum = 0;
            if (x_overlap >= MIN_OVERLAP && y_overlap >= MIN_OVERLAP) {
				vector<int> last_row(x_overlap + 2, 0x10000000), this_row(x_overlap + 2), last_row_m(x_overlap + 2, 0x10000000);
				this_row[0] = 0x10000000, this_row[x_overlap + 1] = 0x10000000;
				for (int y = 0; y < y_overlap; y++) {
					const unsigned char * pm0 = m0.ptr<unsigned char>(y + max(0, y_org), max(0, x_org));
					const unsigned char * pm1 = m1.ptr<unsigned char>(y + max(0, -y_org), max(0, -x_org));
					for (int x = 0; x < x_overlap; x++) {
						int a0 = pm0[x], a1 = pm1[x];
						this_row[x + 1] = abs(a0 - a1);
					}
					for (int x = 0; x < x_overlap; x++) {
						sum += min(last_row_m[x + 1], this_row[x + 1]) * mix[i];
						last_row_m[x + 1] = min(min(last_row[x + 1], this_row[x + 1]), min(this_row[x], this_row[x + 2]));
						last_row[x + 1] = this_row[x + 1];
					}
					if (y == 0)
						sum = 0;
				}
				for (int x = 0; x < x_overlap; x++)
					sum += last_row_m[x + 1] * mix[i];
			}
            if (y_overlap >= MIN_OVERLAP && x_overlap >= MIN_OVERLAP)
				gd.e->dif(yy + y_shift, xx + x_shift) = sum * 100.0 / (x_overlap * y_overlap);
			else
				gd.e->dif(yy + y_shift, xx + x_shift) = DIFF_NOT_CONTACT;
		}
		if (m0.type() == CV_8UC3)
		for (int yy = -y_shift; yy <= y_shift; yy++)
		for (int xx = -x_shift; xx <= x_shift; xx++) {
			int y_org = y_org0 + yy;
			int x_org = x_org0 + xx;
			int y_overlap = m0.rows - abs(y_org);
			int x_overlap = m0.cols - abs(x_org);
			unsigned sum = 0;
            if (y_overlap > 2 && x_overlap > 2) {
				vector<int> last_row(x_overlap + 2, 0x10000000), this_row(x_overlap + 2), last_row_m(x_overlap + 2, 0x10000000);
				this_row[0] = 0x10000000, this_row[x_overlap + 1] = 0x10000000;
				for (int y = 0; y < y_overlap; y++) {
					const unsigned char * pm0 = m0.ptr<unsigned char>(y + max(0, y_org), max(0, x_org));
					const unsigned char * pm1 = m1.ptr<unsigned char>(y + max(0, -y_org), max(0, -x_org));
					for (int x = 0; x < x_overlap; x++) {
						const unsigned char * ppm0 = &pm0[3 * x];
						const unsigned char * ppm1 = &pm1[3 * x];
						int a0 = ppm0[0], b0 = ppm1[0];
						int a1 = ppm0[1], b1 = ppm1[1];
						int a2 = ppm0[2], b2 = ppm1[2];
						this_row[x + 1] = abs(a0 - b0) + abs(a1 - b1) + abs(a2 - b2);
					}
					for (int x = 0; x < x_overlap; x++) {
						sum += min(last_row_m[x + 1], this_row[x + 1]) * mix[i];
						last_row_m[x + 1] = min(min(last_row[x + 1], this_row[x + 1]), min(this_row[x], this_row[x + 2]));
						last_row[x + 1] = this_row[x + 1];
					}
					if (y == 0)
						sum = 0;
				}
				for (int x = 0; x < x_overlap; x++)
					sum += last_row_m[x + 1] * mix[i];
			}
            if (y_overlap >= MIN_OVERLAP && x_overlap >= MIN_OVERLAP)
				gd.e->dif(yy + y_shift, xx + x_shift) = sum * 100.0 / (x_overlap * y_overlap);
			else
				gd.e->dif(yy + y_shift, xx + x_shift) = DIFF_NOT_CONTACT;
		}
	}
	gd.e->compute_score();
	if (gd.e->avg == 0) {
		qDebug("compute_diff found black img1=%s, img2=%s", gd.img_d0->filename.c_str(), gd.img_d1->filename.c_str());
		gd.e->img_num = 0;
	}
}

void extract_diff1(ImageDiff & gd)
{
	try {
		if (gd.img_d0->v[0].d.empty() || gd.img_d1->v[0].d.empty()) {
			gd.e->img_num = 0;
			return;
		}
		for (int i = 0; i < gd.tvar->params.size(); i++) {
			ParamItem param = gd.tvar->params[i];
			int method = param.pi[1] >> 16 & 0xff;
			if (method == DIFF_NORMAL)
				compute_diff1(gd, param);
			if (method == DIFF_WEIGHT)
				compute_weight_diff(gd, param);
		}
	}
	catch (std::exception & e) {
		qFatal("Error in extract_diff1 img1=%s img2=%s, Exception %s.",
			gd.img_d0->filename.c_str(), gd.img_d1->filename.c_str(), e.what());
	}
}

void extract_diff(ImageDiff & gd)
{
	try {
		int cols = gd.img_d0->v[0].d.cols;
		int rows = gd.img_d0->v[0].d.rows;

		if (gd.img_d0->v[0].d.empty() || gd.img_d1->v[0].d.empty()) {
			gd.e->img_num = 0;
			return;
		}
		for (int i = 0; i < gd.tvar->params.size(); i++) {
			ParamItem param = gd.tvar->params[i];
			int method = param.pi[1] >> 16 & 0xff;
			if (method == DIFF_NORMAL) {
				if (gd.dir) { //do left-right compare
					int start_x = cols - gd.overlap / gd.cvar->rescale;
					int shift = gd.shift / gd.cvar->rescale;
					compute_diff(
						gd, param, Rect(start_x, max(shift, 0), gd.overlap / gd.cvar->rescale, rows - abs(shift)),
						Rect(0, max(-shift, 0), gd.overlap / gd.cvar->rescale, rows - abs(shift)),
						gd.cvar->max_lr_xshift / gd.cvar->rescale, gd.cvar->max_lr_yshift / gd.cvar->rescale, gd.e);
					gd.e->offset.x = (start_x - gd.cvar->max_lr_xshift / gd.cvar->rescale) * gd.cvar->rescale;
					gd.e->offset.y = (shift - gd.cvar->max_lr_yshift / gd.cvar->rescale) * gd.cvar->rescale;
				}
				else { //do up-down compare
					int start_y = rows - gd.overlap / gd.cvar->rescale;
					int shift = gd.shift / gd.cvar->rescale;
					compute_diff(
						gd, param, Rect(max(shift, 0), start_y, cols - abs(shift), gd.overlap / gd.cvar->rescale),
						Rect(max(-shift, 0), 0, cols - abs(shift), gd.overlap / gd.cvar->rescale),
						gd.cvar->max_ud_xshift / gd.cvar->rescale, gd.cvar->max_ud_yshift / gd.cvar->rescale, gd.e);
					gd.e->offset.x = (shift - gd.cvar->max_ud_xshift / gd.cvar->rescale) * gd.cvar->rescale;
					gd.e->offset.y = (start_y - gd.cvar->max_ud_yshift / gd.cvar->rescale) * gd.cvar->rescale;
				}
			}
			if (method == DIFF_WEIGHT)
				compute_weight_diff(gd, param);
		}
	}
	catch (std::exception & e) {
		qFatal("Error in extract_diff img1=%s img2=%s, Exception %s.",
			gd.img_d0->filename.c_str(), gd.img_d1->filename.c_str(), e.what());
	}
}


FeatExt::FeatExt()
{
    progress = 0;
}

void FeatExt::set_tune_para(const TuningPara & _tpara)
{
	tpara = _tpara;
}

void FeatExt::set_cfg_para(const ConfigPara & _cpara)
{
	CV_Assert(_cpara.img_num_h == _cpara.offset.rows && _cpara.img_num_w == _cpara.offset.cols);
	cpara = _cpara;
	cpara.offset = cpara.offset.clone();
}

int FeatExt::filter_edge_diff(const FeatExt & fe1, int w0, int w1)
{
	if (fe1.edge[0][0].edge_type != EDGE_UNKNOW)
		return 0;

	*this = fe1;
	int row = edge[0][0].dif.rows;
	int col = edge[0][0].dif.cols;
#define PICK(x1, x, xmin, xmax) (((x1) >= (xmin) && (x1) <=(xmax)) ? (x1) : (x))
	float a0, a1, a2, a3, a4;
	const int * p0, *p1, *p2, *p3, *p4;
	int *p;
	switch (w0) {
	case 0:
		a0 = a4 = a1 = a3 = 0;
		a2 = 1;
		break;
	case 1:
		a0 = a4 = 0.07;
		a1 = a3 = 0.18;
		a2 = 0.5;
		break;
	case 2:
		a0 = a4 = 0.1;
		a1 = a3 = 0.2;
		a2 = 0.4;
		break;
	default:
		a0 = a4 = 0.15;
		a1 = a3 = 0.2;
		a2 = 0.3;
		break;
	}
	for (int y = 0; y < cpara.img_num_h - 1; y++) //up down edge
	for (int x = 0; x < cpara.img_num_w; x++) {
		edge[0][y*cpara.img_num_w + x].dif = fe1.edge[0][y*cpara.img_num_w + x].dif.clone();
		Point offset = edge[0][y*cpara.img_num_w + x].offset - edge[0][y*cpara.img_num_w].offset;
		if (abs(offset.x) > 4 || abs(offset.y) > 4)
			return -1;
		for (int yy = 0; yy < row; yy++) {
			p0 = fe1.edge[0][y * cpara.img_num_w + PICK(x - 2, x, 1, cpara.img_num_w - 2)].dif.ptr<int>(yy);
			p1 = fe1.edge[0][y * cpara.img_num_w + PICK(x - 1, x, 1, cpara.img_num_w - 2)].dif.ptr<int>(yy);
			p2 = fe1.edge[0][y * cpara.img_num_w + x].dif.ptr<int>(yy);
			p3 = fe1.edge[0][y * cpara.img_num_w + PICK(x + 1, x, 1, cpara.img_num_w - 2)].dif.ptr<int>(yy);
			p4 = fe1.edge[0][y * cpara.img_num_w + PICK(x + 2, x, 1, cpara.img_num_w - 2)].dif.ptr<int>(yy);
			p = edge[0][y * cpara.img_num_w + x].dif.ptr<int>(yy);
			for (int xx = 0; xx < col; xx++)
				p[xx] = a0 * p0[xx] + a1 * p1[xx] + a2 * p2[xx] + a3 * p3[xx] + a4 * p4[xx];
		}
		edge[0][y * cpara.img_num_w + x].compute_score();
		if (edge[0][y * cpara.img_num_w + x].avg <= 1)
			edge[0][y * cpara.img_num_w + x].img_num = 0;
	}

	row = edge[1][0].dif.rows;
	col = edge[1][0].dif.cols;
	switch (w1) {
	case 0:
		a0 = a4 = a1 = a3 = 0;
		a2 = 1;
		break;
	case 1:
		a0 = a4 = 0.05;
		a1 = a3 = 0.2;
		a2 = 0.5;
		break;
	case 2:
		a0 = a4 = 0.1;
		a1 = a3 = 0.2;
		a2 = 0.4;
		break;
	default:
		a0 = a4 = 0.12;
		a1 = a3 = 0.2;
		a2 = 0.36;
		break;
	}
	for (int x = 0; x < cpara.img_num_w - 1; x++) //left right edge
	for (int y = 0; y < cpara.img_num_h; y++) {
		edge[1][y*(cpara.img_num_w - 1) + x].dif = fe1.edge[1][y*(cpara.img_num_w - 1) + x].dif.clone();
		Point offset = edge[1][y*(cpara.img_num_w - 1) + x].offset - edge[1][x].offset;
		if (abs(offset.x) > 4 || abs(offset.y) > 4)
			return -1;
		for (int yy = 0; yy < row; yy++) {
			p0 = fe1.edge[1][PICK(y - 2, y, 1, cpara.img_num_h - 2) * (cpara.img_num_w - 1) + x].dif.ptr<int>(yy);
			p1 = fe1.edge[1][PICK(y - 1, y, 1, cpara.img_num_h - 2) * (cpara.img_num_w - 1) + x].dif.ptr<int>(yy);
			p2 = fe1.edge[1][y * (cpara.img_num_w - 1) + x].dif.ptr<int>(yy);
			p3 = fe1.edge[1][PICK(y + 1, y, 1, cpara.img_num_h - 2) * (cpara.img_num_w - 1) + x].dif.ptr<int>(yy);
			p4 = fe1.edge[1][PICK(y + 2, y, 1, cpara.img_num_h - 2) * (cpara.img_num_w - 1) + x].dif.ptr<int>(yy);
			p = edge[1][y * (cpara.img_num_w - 1) + x].dif.ptr<int>(yy);
			for (int xx = 0; xx < col; xx++)
				p[xx] = a0 * p0[xx] + a1 * p1[xx] + a2 * p2[xx] + a3 * p3[xx] + a4 * p4[xx];
		}
		edge[1][y * (cpara.img_num_w - 1) + x].compute_score();
		if (edge[1][y * (cpara.img_num_w - 1) + x].avg <= 1)
			edge[1][y * (cpara.img_num_w - 1) + x].img_num = 0;
	}
	return 0;
}

int FeatExt::filter_edge_diff1(const FeatExt & fe1, int w0, int w1)
{
	*this = fe1;
	int scale = fe1.cpara.rescale;
	int filter_r = FILTER_R(scale);
	int reduce_min_num = 0;
	int upgrade_bus = 0;
	int upgrade_solo = 0;
	for (int e = 0; e <= 1; e++) {//e=0 is up down edge
		int a0, a1; //a0 for shoot corner, a1 for shoot bus
		int strength = (e == 0) ? w0 : w1;
		switch (strength) {
		case 0:
			a0 = 1;
			a1 = 1;
			break;
		case 1:
			a0 = 2;
			a1 = 2;
			break;
		case 2:
			a0 = 3;
			a1 = 3;
			break;
		default:
			a0 = 4;
			a1 = 4;
			break;
		}
		int yend = (e == 0) ? cpara.img_num_h - 1 : cpara.img_num_h;
		int xend = (e == 0) ? cpara.img_num_w : cpara.img_num_w - 1;
		for (int y = 0; y < yend; y++) 
		for (int x = 0; x < xend; x++) {
			EdgeDiff * ptarget = &edge[e][y*xend + x];			
			if (ptarget->edge_type == EDGE_HAS_CORNER && ptarget->min_num == 1) {
				ptarget->dif = fe1.edge[e][y*xend + x].dif.clone();
				continue;
			}
			
			Mat_<int> dif = fe1.edge[e][y*xend + x].dif.clone();
			int len = 2; //for bus and black, len = 2 means 8 bullets
			int a = a1; //for bus, a = a1
			int has_corner = 0; //for bus
			CV_Assert(ptarget->edge_type != EDGE_UNKNOW);
			if (ptarget->edge_type == EDGE_HAS_CORNER) { //len=1 means 4 bullets
				a = a0;
				has_corner = 1;
			}
			int hit = 0;
			Point avg_loc(0, 0);
			for (int dir = 1 - e; dir < 4; dir+=2) 
			for (int l = 1; l <= len; l++) {
				int y0 = y + dxy[dir][0] * l;
				int x0 = x + dxy[dir][0] * l;
				if (y0 < 0 || x0 < 0 || y0 >= yend || x0 >= xend)
					continue;
				const EdgeDiff *pshoot = &fe1.edge[e][y0*xend + x0]; //got bullet
				if (!(pshoot->edge_type == EDGE_HAS_CORNER && pshoot->min_num == 1))
					continue;
				Point shoot_min_pos = pshoot->minloc * scale + pshoot->offset;
				Point shoot_loc = shoot_min_pos - ptarget->offset;
				shoot_loc.x /= scale;
				shoot_loc.y /= scale;
				if (shoot_loc.x < 0 || shoot_loc.x >= dif.cols ||
					shoot_loc.y < 0 || shoot_loc.y >= dif.rows) //shoot off
					continue;
				//now bullet is ready
				hit++;
				avg_loc += shoot_loc;
				if (ptarget->edge_type != EDGE_BLACK)
				for (int dy = -filter_r; dy <= filter_r; dy++)
				for (int dx = -filter_r; dx <= filter_r; dx++)
				if (shoot_loc.y + dy >= 0 && shoot_loc.y + dy < dif.rows &&
					shoot_loc.x + dx >= 0 && shoot_loc.x + dx < dif.cols) { //bullet range
					Point loc = shoot_loc + Point(dx, dy);
					int v = dif(loc);
					if (v >= DIFF_AVG_VALUE)
						continue;
					dif(loc) = max(0, v - a);
				}				
			}
			
			int prev_min_num = ptarget->min_num;
			//compute_edge_type2(*ptarget, dif, filter_r, BUS_LEN(scale), ptarget->edge_type, false);
			if (ptarget->edge_type == EDGE_BLACK && hit > 1) {
				ptarget->minloc.x = avg_loc.x / hit;
				ptarget->minloc.y = avg_loc.y / hit;
			}
			if (ptarget->edge_type == EDGE_HAS_CORNER && !has_corner) //for bus upgrade to corner
				upgrade_bus++;
			
			if (has_corner) {
				if (prev_min_num > 1 && ptarget->min_num == 1)
					upgrade_solo++;
				if (prev_min_num > ptarget->min_num)
					reduce_min_num += prev_min_num - ptarget->min_num;
			}
		}
	}
	qInfo("filter_edge, upgrade_bus=%d,upgrade_solo=%d,reduce_min=%d", upgrade_bus, upgrade_solo, reduce_min_num);
	return 0;
}

void FeatExt::generate_feature_diff(int start_x, int start_y, int debug_en)
{
	progress = 0;
    int img_load_num = 15;
	int total_load = 0;
	vector<ImageData> image[2];
	vector<ImageDiff> gdiff;

	for (int i = 0; i < 2; i++) {
		image[i].resize(img_load_num + 1);
		for (int j = 0; j < image[i].size(); j++) {
			image[i][j].cvar = &cpara;
			image[i][j].tvar = &tpara;
		}		
	}
	edge[0].clear();
	edge[1].clear();
	edge[0].resize((cpara.img_num_h - 1) * cpara.img_num_w);
	for (int y = 0; y < cpara.img_num_h - 1; y++)
		for (int x = 0; x < cpara.img_num_w; x++)
			edge[0][y*cpara.img_num_w + x].edge_idx = MAKE_EDGE_IDX(x, y, 0);
	edge[1].resize(cpara.img_num_h * (cpara.img_num_w - 1));
	for (int y = 0; y < cpara.img_num_h; y++)
		for (int x = 0; x < cpara.img_num_w - 1; x++)
			edge[1][y*(cpara.img_num_w - 1) + x].edge_idx = MAKE_EDGE_IDX(x, y, 1);
	
	for (int row = start_y; row < start_y + cpara.img_num_h - 1; row += img_load_num) { //once load img_load_num
		for (int x = start_x; x < start_x + cpara.img_num_w; x++) {
			image[x & 1].resize(min(img_load_num + 1, start_y + cpara.img_num_h - row));
			for (int y = row; y < min(row + img_load_num + 1, start_y + cpara.img_num_h); y++) {
				image[x & 1][y - row].filename = cpara.get_img_name(x, y);
				image[x & 1][y - row].x = x - start_x;
				image[x & 1][y - row].y = y - start_y;
			}
#if PARALLEL
			QtConcurrent::blockingMap<vector<ImageData> >(image[x & 1], prepare_extract);
#else
			for (int i = 0; i < image[x & 1].size(); i++)
				prepare_extract(image[x & 1][i]);
#endif
			if (debug_en == 1) {
				char filename[100];
				//sprintf(filename, "g0_M%d.jpg", 2 * (x - start_x));
				//imwrite(filename, abs(image[x & 1][0].v[0].d));
				//sprintf(filename, "g0_M%d.jpg", 2 * (x - start_x) + 1);
				//imwrite(filename, abs(image[x & 1][0].v[2].d) * 8);
			}

			total_load += (int) image[x & 1].size() - 1;
			progress = (float)total_load / (cpara.img_num_h * cpara.img_num_w);
			if (x != start_x) {
				if (row + img_load_num + 1 >= start_y + cpara.img_num_h)
					gdiff.resize(image[x & 1].size() * 2 - 1);
				else
					gdiff.resize(image[x & 1].size() * 2 - 2);
				for (int i = 0; i < gdiff.size(); i++) {					
					if (i & 1) { //for up-down image feature extract
						gdiff[i].e = &edge[0][(row -start_y + i / 2) * cpara.img_num_w + x - start_x - 1];
						CV_Assert(gdiff[i].e->edge_idx == MAKE_EDGE_IDX(x - start_x - 1, row - start_y + i / 2, 0));
						gdiff[i].img_d0 = &image[(x - 1) & 1][i / 2];
						gdiff[i].img_d1 = &image[(x - 1) & 1][i / 2 + 1];
						gdiff[i].overlap = image[0][0].v[0].d.rows * cpara.rescale - 
							(cpara.offset(row - start_y + i / 2 + 1, x - start_x - 1)[0] - cpara.offset(row - start_y + i / 2, x - start_x - 1)[0]);
						gdiff[i].shift = cpara.offset(row - start_y + i / 2 + 1, x - start_x - 1)[1] - cpara.offset(row - start_y + i / 2, x - start_x - 1)[1];
						gdiff[i].dir = 0;
						gdiff[i].cvar = &cpara;
						gdiff[i].tvar = &tpara;
					}
					else { //for left-right image feature extract
						gdiff[i].e = &edge[1][(row - start_y + i / 2) * (cpara.img_num_w - 1) + x - start_x - 1];
						CV_Assert(gdiff[i].e->edge_idx == MAKE_EDGE_IDX(x - start_x - 1, row - start_y + i / 2, 1));
						gdiff[i].img_d0 = &image[(x-1) & 1][i / 2];
						gdiff[i].img_d1 = &image[x & 1][i / 2];
						gdiff[i].overlap = image[0][0].v[0].d.cols * cpara.rescale - 
							(cpara.offset(row - start_y + i / 2, x - start_x)[1] - cpara.offset(row - start_y + i / 2, x - start_x - 1)[1]);
						gdiff[i].shift = cpara.offset(row - start_y + i / 2, x - start_x)[0] - cpara.offset(row - start_y + i / 2, x - start_x - 1)[0];
						gdiff[i].dir = 1;
						gdiff[i].cvar = &cpara;
						gdiff[i].tvar = &tpara;
					}
				}
				if (cpara.rescale <= 2) {
#if PARALLEL
					QtConcurrent::blockingMap<vector<ImageDiff> >(gdiff, extract_diff);
#else
					for (int i = 0; i < gdiff.size(); i++)
						extract_diff(gdiff[i]);
#endif										
				} else
#if PARALLEL
					QtConcurrent::blockingMap<vector<ImageDiff> >(gdiff, extract_diff1);
#else
				for (int i = 0; i < gdiff.size(); i++)
					extract_diff1(gdiff[i]);
#endif
			}
			if (x + 1 == start_x + cpara.img_num_w) {
				gdiff.resize(image[x&1].size() - 1);
				for (int i = 0; i < gdiff.size(); i++) {
					gdiff[i].e = &edge[0][(row - start_y + i) * cpara.img_num_w + x - start_x];
					gdiff[i].img_d0 = &image[x & 1][i];
					gdiff[i].img_d1 = &image[x & 1][i + 1];
					gdiff[i].overlap = image[0][0].v[0].d.rows * cpara.rescale - 
						(cpara.offset(row - start_y + i + 1, x - start_x)[0] - cpara.offset(row - start_y + i, x - start_x)[0]);
					gdiff[i].shift = cpara.offset(row - start_y + i + 1, x - start_x)[1] - cpara.offset(row - start_y + i, x - start_x)[1];
					gdiff[i].dir = 0;
					gdiff[i].cvar = &cpara;
					gdiff[i].tvar = &tpara;
				}
				if (cpara.rescale <= 2) {
#if PARALLEL
					QtConcurrent::blockingMap<vector<ImageDiff> >(gdiff, extract_diff);
#else
					for (int i = 0; i < gdiff.size(); i++)
						extract_diff(gdiff[i]);
#endif
				}
				else {
#if PARALLEL
					QtConcurrent::blockingMap<vector<ImageDiff> >(gdiff, extract_diff1);
#else
					for (int i = 0; i < gdiff.size(); i++)
						extract_diff1(gdiff[i]);
#endif
				}				
			}
		}
	}

	int diff_method = 0, pp_method = 0;
	for (auto & param : tpara.params) {
		int method = param.pi[1] >> 16 & 0xff;
		if (method == DIFF_NORMAL || method == DIFF_WEIGHT)
			diff_method = method;
		if (method == PP_COMPUTE_GRAD || method == PP_COMPUTE_CORNER)
			pp_method = method;
	}
	generate_post_process(diff_method, pp_method);
}

void FeatExt::generate_post_process(int diff_method, int pp_method)
{

}

void write(FileStorage& fs, const std::string&, const ConfigPara& x)
{
	x.write_file(fs);
}

void read(const FileNode& node, ConfigPara& x, const ConfigPara& default_value)
{
	if (node.empty())
		x = default_value;
	else
		x.read_file(node);
}
void write(FileStorage& fs, const std::string&, const TuningPara& x)
{
	x.write_file(fs);
}

void read(const FileNode& node, TuningPara& x, const TuningPara& default_value)
{
	if (node.empty())
		x = default_value;
	else
		x.read_file(node);
}

void write(FileStorage& fs, const std::string&, const EdgeDiff& x)
{
	x.write_file(fs);
}

void read(const FileNode& node, EdgeDiff& x, const EdgeDiff& default_value)
{
	if (node.empty())
		x = default_value;
	else
		x.read_file(node);
}

void FeatExt::write_diff_file(string filename)
{
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "version" << FEAT_EXT_VERSION;
	fs << "cpara" << cpara;	
	for (int y = 0; y < cpara.img_num_h - 1; y++)
		for (int x = 0; x < cpara.img_num_w; x++) {
			char name[30];
            sprintf(name, "d0ud_%d_%d", y, x);
			fs << name << edge[0][y* cpara.img_num_w + x];
		}

	for (int y = 0; y < cpara.img_num_h; y++)
		for (int x = 0; x < cpara.img_num_w - 1; x++) {
			char name[30];
            sprintf(name, "d1lr_%d_%d", y, x);
			fs << name << edge[1][y* (cpara.img_num_w - 1) + x];
		}
}

int FeatExt::read_diff_file(string filename)
{
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return -1;
	int version = fs["version"];
	if (version != FEAT_EXT_VERSION)
		return -1;
	fs["cpara"] >> cpara;
	edge[0].clear();
	edge[1].clear();
	edge[0].resize((cpara.img_num_h - 1) * cpara.img_num_w);
	edge[1].resize(cpara.img_num_h * (cpara.img_num_w - 1));

	for (int y = 0; y < cpara.img_num_h - 1; y++)
		for (int x = 0; x < cpara.img_num_w; x++) {
			char name[30];
			sprintf(name, "d0ud_%d_%d", y, x);
			fs[name] >> edge[0][y* cpara.img_num_w + x];
			EdgeDiff * pdiff = &edge[0][y* cpara.img_num_w + x];
			pdiff->edge_idx = MAKE_EDGE_IDX(x, y, 0);
			CV_Assert(pdiff->offset.x % cpara.rescale == 0 && pdiff->offset.y % cpara.rescale == 0);
		}

	for (int y = 0; y < cpara.img_num_h; y++)
		for (int x = 0; x < cpara.img_num_w - 1; x++) {
			char name[30];
			sprintf(name, "d1lr_%d_%d", y, x);
			fs[name] >> edge[1][y* (cpara.img_num_w - 1) + x];
			EdgeDiff * pdiff = &edge[1][y* (cpara.img_num_w - 1) + x];
			pdiff->edge_idx = MAKE_EDGE_IDX(x, y, 1);
			CV_Assert(pdiff->offset.x % cpara.rescale == 0 && pdiff->offset.y % cpara.rescale == 0);
		}
	return 0;
}

const EdgeDiff * FeatExt::get_edge(int i, int y, int x) const
{
	if (i == 0 && (y >= cpara.img_num_h - 1 || x >= cpara.img_num_w))
		return NULL;
	if (i == 1 && (y >= cpara.img_num_h || x >= cpara.img_num_w - 1))
		return NULL;
	if (x < 0 || y < 0)
		return NULL;
	if (i == 0)
		return &edge[0][y* cpara.img_num_w + x];
	else
		return &edge[1][y* (cpara.img_num_w - 1) + x];
}

const EdgeDiff * FeatExt::get_edge(int idx) const
{
	const EdgeDiff * diff = get_edge(EDGE_E(idx), EDGE_Y(idx), EDGE_X(idx));
	CV_Assert(diff->edge_idx == idx);
	return diff;
}

const EdgeDiff * FeatExt::get_edge(int y0, int x0, int y1, int x1) const
{
	if (x0 == x1) {
		if (y1 == y0 + 1)
			return get_edge(0, y0, x0);
		if (y0 == y1 + 1)
			return get_edge(0, y1, x1);
	}
	if (y0 == y1) {
		if (x1 == x0 + 1)
			return get_edge(1, y0, x0);
		if (x0 == x1 + 1)
			return get_edge(1, y1, x1);
	}
	return NULL;
}
