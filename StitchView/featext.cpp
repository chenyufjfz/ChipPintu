#define FEATEXT_C
#include "featext.h"
#include <iostream>
#include <vector>
#include <string>
#include <QtConcurrent>
#include "opencv2/highgui/highgui.hpp"

#ifdef QT_DEBUG
#ifdef Q_OS_WIN
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif

#define PARALLEL 0
#define TOTAL_PROP 500
#define MIN_OVERLAP 12

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



static int push_point(vector<Point3i> & vs, Point3i v, double mindistance, bool checkonly = false)
{
	for (auto & o : vs) {
		float dx = v.x - o.x;
		float dy = v.y - o.y;
		if (dx*dx + dy*dy < mindistance)
			return 0;
	}
	if (checkonly)
		return 1;
	vs.push_back(v);
	return 1;
}

template<typename T> struct greaterThanPtr
{
	bool operator()(const T* a, const T* b) const { return *a > *b; }
};

static void calc_eigenval(const Mat& _cov, Mat& _dst, Mat& _dst1, double th)
{
	int i, j;
	Size size = _cov.size();

	if (_cov.isContinuous() && _dst.isContinuous() && _dst1.isContinuous())
	{
		size.width *= size.height;
		size.height = 1;
	}

	for (i = 0; i < size.height; i++)
	{
		const float* cov = (const float*)(_cov.data + _cov.step*i);
		float* dst = (float*)(_dst.data + _dst.step*i);
		float* dst1 = (float*)(_dst1.data + _dst1.step*i);
		j = 0;
		for (; j < size.width; j++)
		{
			float a = cov[j * 3] * 0.5f;
			float b = cov[j * 3 + 1];
			float c = cov[j * 3 + 2] * 0.5f;
			if (a * 2 < th && c * 2 < th) {
				dst[j] = 0;
				dst1[j] = 0;
			}
			else {
				float d = std::sqrt((a - c)*(a - c) + b*b);
				float x0 = a + c - d; //dst[j] = a + c - d;
				float x1 = a + c + d;
				dst1[j] = x1;
				dst[j] = x0 * x0 / x1;
			}
		}
	}
}

static void corner_eigen_valsvecs(const Mat& src, Mat& eigenv0, Mat & eigenv1, Mat & cov, int block_size, int aperture_size, double th, int borderType = BORDER_DEFAULT)
{
	double scale = (double)(1 << ((aperture_size > 0 ? aperture_size : 3) - 1)) * sqrt(10);// *sqrt((double)(block_size + 1) / 2);
	if (aperture_size < 0)
		scale *= 2.;
	//if (depth == CV_8U)
	//scale *= 255.;
	scale = 1. / scale;

	CV_Assert(src.type() == CV_8UC1 || src.type() == CV_32FC1);

	Mat Dx, Dy;
	if (aperture_size > 0)
	{
		Mat kx, ky;
		Sobel(src, Dx, CV_32F, 1, 0, aperture_size, scale, 0, borderType);
		Sobel(src, Dy, CV_32F, 0, 1, aperture_size, scale, 0, borderType);
	}
	else
	{
		Scharr(src, Dx, CV_32F, 1, 0, scale, 0, borderType);
		Scharr(src, Dy, CV_32F, 0, 1, scale, 0, borderType);
	}

	Size size = src.size();
	cov.create(size, CV_32FC3);
	int i, j;

	for (i = 0; i < size.height; i++)
	{
		float* cov_data = (float*)(cov.data + i*cov.step);
		const float* dxdata = (const float*)(Dx.data + i*Dx.step);
		const float* dydata = (const float*)(Dy.data + i*Dy.step);

		for (j = 0; j < size.width; j++)
		{
			float dx = dxdata[j];
			float dy = dydata[j];

			cov_data[j * 3] = dx*dx;
			cov_data[j * 3 + 1] = dx*dy;
			cov_data[j * 3 + 2] = dy*dy;
		}
	}

	boxFilter(cov, cov, cov.depth(), Size(block_size, block_size),
		Point(-1, -1), false, borderType);

	calc_eigenval(cov, eigenv0, eigenv1, th);
}

static void corner_eigenval(InputArray _src, OutputArray _dst, Mat & dst1, Mat & cov, int blockSize, int ksize, double th, int borderType)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), CV_32F);
	dst1.create(src.size(), CV_32F);
	Mat dst = _dst.getMat();
	corner_eigen_valsvecs(src, dst, dst1, cov, blockSize, ksize, th, borderType);
}

static void find_good_corner(Mat & eig, vector<const float*> & pcorners, double th, int blocksize, bool need_dilate=true)
{
	Mat tmp;
	int bs = blocksize;
	if (need_dilate)
		dilate(eig, tmp, Mat());

	Size imgsize = eig.size();

	// collect list of pointers to features - put them into vector
	for (int y = bs; y < imgsize.height - bs; y++)
	{
		const float* eig_data = (const float*)eig.ptr(y);
		const float* tmp_data = need_dilate ? (const float*)tmp.ptr(y) : eig_data;

		for (int x = bs; x < imgsize.width - bs; x++)
		{
			float val = eig_data[x];
			if (val > th && (val == tmp_data[x] || y == bs || x == bs || y + 1 == imgsize.height - bs || x + 1 == imgsize.width - bs))
				pcorners.push_back(eig_data + x);
		}
	}

	sort(pcorners, greaterThanPtr<float>());
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

static void my_corner_eigenval(const Mat & image, Mat & eig0, Mat & eig1, int blockSize, int ksize, double & quality, int ox, int oy)
{
	CV_Assert(image.type() == CV_8UC1 && blockSize % 2 == 1);
	if (ox < 0)
		ox = image.cols / 2;
	if (oy < 0)
		oy = image.rows / 2;
	medianBlur(image, image, ksize);
	Mat eig2, shape;
	eig0.create(image.size(), CV_32FC1);
	eig1.create(image.size(), CV_32FC1);
	eig2.create(image.size(), CV_32FC1);
	shape.create(image.size(), CV_8UC1);
	eig0 = Scalar::all(0);
	eig1 = Scalar::all(0);
	eig2 = Scalar::all(0);
	shape = Scalar::all(100);

	Mat ig, iig;
	integral_square(image, ig, iig, Mat(), Mat(), false);

	vector<int> offset;
	vector<int> b;
	int bs2 = blockSize / 2;
	double area = 1.0 / (blockSize * blockSize);
	float a0 = 0, a1 = 0, a2 = 0, a3 = 0;

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
				_b = 1 << 5 | 8;
			else
			if (y > 0)
				_b = 4 << 5 | 5;
			else
				_b = 0;
		}
		else
		if (y == 0)
			_b = (x < 0) ? 6 << 5 | 7 : 2 << 5 | 3;
		else
		if (y == x)
			_b = (x < 0) ? 7 << 5 | 8 : 3 << 5 | 4;
		else
		if (y == -x)
			_b = (x < 0) ? 5 << 5 | 6 : 1 << 5 | 2;
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
	int offset_size = (int) offset.size();
	vector<int> stat[4];
	int total[4] = { 0 };
	for (int i = 0; i < 4; i++)
		stat[i].resize(1024, 0);
	for (int y = blockSize; y < image.rows - blockSize; y++) {
		int * pig = ig.ptr<int>(y - bs2);
		int * pig1 = ig.ptr<int>(y - bs2 + blockSize);
		int * piig = iig.ptr<int>(y - bs2);
		int * piig1 = iig.ptr<int>(y - bs2 + blockSize);
		float * peig1 = eig1.ptr<float>(y);
		float * peig2 = eig2.ptr<float>(y);
		for (int x = blockSize; x < image.cols - blockSize; x++) {
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

	float th[4] = { 16, 16, 16, 16};
	for (int j = 0; j < 4; j++) {
		int agg = 0;
		for (int i = 1023; i > 1; i--) {
			agg += stat[j][i];
			if (agg >= total[j] * quality) {
				th[j] = i * 8;
				break;
			}
		}
	}
	quality = min(min(th[0], th[1]), min(th[2], th[3])) * 2;
	qInfo("th0=%4f, th1=%4f, th2=%4f, th3=%4f, quality=%4f", th[0], th[1], th[2], th[3], quality);
	vector<Point> corner_pts;
	for (int y = blockSize; y < image.rows - blockSize; y++) {
		float * peig1 = eig1.ptr<float>(y);
		float * peig2 = eig2.ptr<float>(y);
		const uchar * pi = image.ptr<uchar>(y);
		uchar * pshape = shape.ptr<uchar>(y);
		int cor[31];
		for (int x = blockSize; x < image.cols - blockSize; x++) {
			if (y >= oy && y < image.rows - oy && x >= ox && x < image.cols - ox)
				x = image.cols - ox;			
			bool pass = (y < oy && peig1[x] > th[DIR_UP] || y >= image.rows - oy && peig1[x] > th[DIR_DOWN]
				|| x < ox && peig1[x] > th[DIR_LEFT] || x >= image.cols - ox && peig1[x] > th[DIR_RIGHT]);
			if (!pass) {
				peig2[x] = 0;
				continue;
			}

			uchar mi = 255, ma = 0;
			int s = peig2[x];
			int avg = s / offset_size;
			const uchar * p0 = pi + x;
			for (int i = 0; i < offset_size; i++) { //compute min, max avg
				uchar a = p0[offset[i]];
				mi = min(mi, a);
				ma = max(ma, a);
			}
			bool choose_min = (avg - mi < ma - avg); //choose min-adjust or max-adjust			
			int bs[9] = { 0 };
			int ss = 0;
			for (int i = 0; i < offset_size; i++) { //compute xiang guan
				int a = choose_min ? p0[offset[i]] - mi : ma - p0[offset[i]];
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
				cor[i] = bs[0] + bs[i + 1] + bs[i + 2];
			cor[7] = bs[0] + bs[8] + bs[1];
			for (int i = 8; i < 14; i++)
				cor[i] = bs[0] + bs[i - 7] + bs[i - 6] + bs[i - 5];
			cor[14] = bs[0] + bs[7] + bs[8] + bs[1];
			cor[15] = bs[0] + bs[8] + bs[1] + bs[2];
			for (int i = 16; i < 21; i++)
				cor[i] = bs[0] + bs[i - 15] + bs[i - 14] + bs[i - 13] + bs[i - 12];
			cor[21] = bs[0] + bs[6] + bs[7] + bs[8] + bs[1];
			cor[22] = bs[0] + bs[7] + bs[8] + bs[1] + bs[2];
			cor[23] = bs[0] + bs[8] + bs[1] + bs[2] + bs[3];
			float mc0 = 0, mc1 = 0, mc2 = 0, mc3 = 0, max_cor, max_cor0;
			for (int i = 0; i < 8; i++)
				mc0 = max(mc0, (float)bs[i]);
			mc0 = mc0 * mc0 / (a0 * ss);
			for (int i = 0; i < 8; i++)
				mc1 = max(mc1, (float)cor[i]);
			mc1 = mc1 * mc1 / (a1 * ss);
			for (int i = 8; i < 16; i++)
				mc2 = max(mc2, (float)cor[i]);
			mc2 = mc2 * mc2 / (a2 * ss);
			int _shape = 16;
			for (int i = 16; i < 24; i++) {
				if (mc3 < cor[i]) {
					mc3 = cor[i];
					_shape = i;
				}
			}
			mc3 = mc3 * mc3 / (a3 * ss);
			max_cor = max(mc1, mc2);
			max_cor0 = max(mc3, mc0);
			CV_Assert(max_cor <= 1);
			if (max_cor > 0.6667 && max_cor > max_cor0) {
				peig2[x] = max_cor;
				corner_pts.push_back(Point(x, y));
				pshape[x] = 10;
			}
			else {
				peig2[x] = max_cor0;
				pshape[x] = (_shape < 20) ? _shape - 16 : _shape - 20;
			}
		}
	}

	for (auto & c : corner_pts) {
		float * peig2 = eig2.ptr<float>(c.y, c.x);
		uchar * pshape = shape.ptr<uchar>(c.y, c.x);
		uchar * pshape1 = shape.ptr<uchar>(c.y + 1, c.x);
		uchar * pshape_1 = shape.ptr<uchar>(c.y - 1, c.x);
		bool in_line = (pshape[1] == 2 && pshape[-1] == 2) || (pshape1[0] == 0 && pshape_1[0] == 0)
			|| (pshape1[1] == 3 && pshape_1[-1] == 3) || (pshape1[-1] == 1 && pshape_1[1] == 1);
		bool in_void = (pshape[1] == 100 && pshape[-1] == 100) || (pshape1[0] == 100 && pshape_1[0] == 100)
			|| (pshape1[1] == 100 && pshape_1[-1] == 100) || (pshape1[-1] == 100 && pshape_1[1] == 100);
		if (!in_void)
		if (peig2[0] > peig2[1] && peig2[0] > peig2[-1] &&
			peig2[0] > eig2.at<float>(c.y - 1, c.x) && peig2[0] > eig2.at<float>(c.y + 1, c.x))
			eig0.at<float>(c) = 3000 * (peig2[0] - 0.6667);
		else
		if (!in_line)
			eig0.at<float>(c) = 1200 * (peig2[0] - 0.6667);
		else
			eig0.at<float>(c) = 500 * (peig2[0] - 0.6667);
	}
}

struct CornerInfo {
	vector<Point3i> corners[4];  //used as good_features_to_track output para
	int max_number; //used as good_features_to_track input para
	double th;  //used as good_features_to_track input para
	double minDistance;  //used as good_features_to_track input para
};
static void good_features_to_track(Mat & img, CornerInfo * cinfo, int scale, int ox, int oy, int method, bool debug_en)
{
	CV_Assert(cinfo[1].minDistance >= cinfo[0].minDistance);
	int blockSize, ksize;
	Mat eig[2], cov;
	vector<const float*> tmpCorners[2];
	if (ox < 0)
		ox = img.cols / 4;
	if (oy < 0)
		oy = img.rows / 4;
	qInfo("good feature to track");
	Mat image;
	if (method == 0) {
		cinfo[0].th *= cinfo[0].th; 
		cinfo[1].th *= cinfo[1].th;
		blockSize = (scale == 4) ? 5 : (scale == 2) ? 9 : 13;
		ksize = (scale == 4) ? 3 : (scale == 2) ? 5 : 7;
		image = img;
		corner_eigenval(image, eig[0], eig[1], cov, blockSize, ksize, cinfo[0].th, BORDER_DEFAULT);
		find_good_corner(eig[0], tmpCorners[0], cinfo[0].th, blockSize); //eig[0] stores corner
		find_good_corner(eig[1], tmpCorners[1], cinfo[1].th, blockSize); //eig[1] stores edge
	}
	else {
		blockSize = (scale == 4) ? 5 : 9;
		ksize = (scale == 4) ? 3 : 5;
		cinfo[1].th /= 100;
		if (scale == 1) {
			resize(img, image, Size(img.cols / 2, img.rows / 2));
			cinfo[0].minDistance /= 2;
			cinfo[1].minDistance /= 2;
			ox = ox / 2;
			oy = oy / 2;
		}
		else
			image = img;
		my_corner_eigenval(image, eig[0], eig[1], blockSize, ksize, cinfo[1].th, ox, oy);
		find_good_corner(eig[0], tmpCorners[0], cinfo[0].th, blockSize, false); //eig[0] stores corner
		find_good_corner(eig[1], tmpCorners[1], cinfo[1].th, blockSize, false); //eig[1] stores edge
	}

	for (int eig_idx = 0; eig_idx < 2; eig_idx++) {
		CornerInfo & c = cinfo[eig_idx];
		c.minDistance *= c.minDistance;
		for (int i = 0; i < (int)tmpCorners[eig_idx].size(); i++)
		{
			int ofs = (int)((const uchar*)tmpCorners[eig_idx][i] - eig[eig_idx].data);
			int y = (int)(ofs / eig[eig_idx].step);
			int x = (int)((ofs - y * eig[eig_idx].step) / sizeof(float));
			//push to different corner set
			Point3i v(x, y, eig[eig_idx].at<float>(y, x) + 0.5f);
			if (y < oy && c.corners[DIR_UP].size() < c.max_number)
				push_point(c.corners[DIR_UP], v, c.minDistance);
			if (y > image.rows - oy && c.corners[DIR_DOWN].size() < c.max_number)
				push_point(c.corners[DIR_DOWN], v, c.minDistance);
			if (x < ox && c.corners[DIR_LEFT].size() < c.max_number)
				push_point(c.corners[DIR_LEFT], v, c.minDistance);
			if (x > image.cols - ox && c.corners[DIR_RIGHT].size() < c.max_number)
				push_point(c.corners[DIR_RIGHT], v, c.minDistance);
		}
		if (eig_idx == 0) {
			for (int i = 0; i < 4; i++)
				cinfo[1].corners[i] = cinfo[0].corners[i];
			cinfo[1].max_number += cinfo[0].max_number;
		}
		else {
			for (int i = 0; i < 4; i++)
				cinfo[1].corners[i].erase(cinfo[1].corners[i].begin(), cinfo[1].corners[i].begin() + cinfo[0].corners[i].size());
			cinfo[1].max_number -= cinfo[0].max_number;
		}
	}

	if (debug_en)
	for (int eig_idx = 0; eig_idx < 2; eig_idx++)
	for (int i = 0; i < 4; i++) {
		char line[1000];
		int idx = 0;
		for (int j = 0; j < min((int)cinfo[eig_idx].corners[i].size(), 5); j++) {
			int x = cinfo[eig_idx].corners[i][j].x;
			int y = cinfo[eig_idx].corners[i][j].y;
			idx += sprintf(line + idx, "%4d,%4d,%5.0f,%5.0f  ", y, x,
				eig[0].at<float>(y, x), eig[1].at<float>(y, x));
		}
		qInfo(line);
	}

	if (scale == 1)
	for (int eig_idx = 0; eig_idx < 2; eig_idx++) {
		for (int i = 0; i < 4; i++)
		for (auto & v : cinfo[eig_idx].corners[i]) {
			v.x = v.x * 2;
			v.y = v.y * 2;
		}
	}
}

/*     31..24  23..16   15..8   7..0
opt0:					method layer
opt1: debug_opt      corner_num edge_num
opt2: corner_distance edge_distance corner_th edge_th
opt3: max_weight_corner(16) max_weight_edge(16)
opt4:          edge_reduce overlapx overlapy
*/
static void prepare_corner(ImageData & img_d, const ParamItem & param)
{
	int method = param.pi[0] >> 8 & 0xff;
	int debug_opt = param.pi[1] >> 24 & 0xff;
	int edge_reduce = param.pi[4] >> 16 & 0xff;
	CornerInfo cinfo[2];
	int scale = img_d.cvar->rescale;

	cinfo[0].max_number = param.pi[1] >> 8 & 0xff;
	cinfo[1].max_number = param.pi[1] & 0xff;
	cinfo[0].minDistance = param.pi[2] >> 24 & 0xff;
	cinfo[1].minDistance = param.pi[2] >> 16 & 0xff;
	cinfo[0].minDistance /= scale;
	cinfo[1].minDistance /= scale;

	cinfo[0].th = param.pi[2] >> 8 & 0xff;
	cinfo[1].th = param.pi[2] & 0xff;

	int weight_th[2] = { param.pi[3] >> 16 & 0xffff, param.pi[3] & 0xffff };
	int overlap_x = img_d.v[0].d.cols / (param.pi[4] >> 8 & 0xff);
	int overlap_y = img_d.v[0].d.rows / (param.pi[4] & 0xff);
	good_features_to_track(img_d.v[0].d, cinfo, scale, overlap_x, overlap_y, method, debug_opt & 2);

	if (debug_opt & 1) {
		Mat cf = img_d.v[0].d.clone();
		for (int i = 0; i < 4; i++) {
			for (auto & v : cinfo[0].corners[i])
				circle(cf, Point(v.x, v.y), 1, Scalar::all(255));
		}
		string filename = img_d.filename.substr(0, img_d.filename.find_last_of("."));
		filename += "_c.jpg";
		imwrite(filename, cf);
	}

	//limit weight
	for (int eig_idx = 0; eig_idx < 2; eig_idx++)
	for (int i = 0; i < 4; i++) {
		for (auto & v : cinfo[eig_idx].corners[i]) {
			if (method == 0) {
				if (v.z > weight_th[eig_idx])
					v.z = sqrt(v.z * weight_th[eig_idx]); //make corner weight smaller
				if (eig_idx == 1 && cinfo[0].corners[i].size() < cinfo[0].max_number) {
					v.z /= edge_reduce;
					cinfo[0].corners[i].push_back(v);
				}
			}
			else {
				if (eig_idx == 1 && cinfo[0].corners[i].size() < cinfo[0].max_number) {
					v.z = edge_reduce;
					cinfo[0].corners[i].push_back(v);
				}
			}
		}
		if (eig_idx == 1)
			img_d.v[i + 1].d = Mat(cinfo[0].corners[i], true); //change vector to mat
	}
}

static void prepare_gray_vec(Mat & m, vector<int> & offset, int scale)
{
	int d = (scale == 4) ? 2 : (scale == 2) ? 4 : 7; //should be same as get_gray_vec
	CV_Assert(m.type() == CV_8UC1);
	offset.clear();
	for (int y = -d; y <= d; y++)
	for (int x = -d; x <= d; x++)
		offset.push_back((y * (int)m.step.p[0] + x * (int)m.step.p[1]) / sizeof(uchar));

}
//return average gray
static bool get_gray_vec(const Mat & m, Point c, vector<float> & g, const vector<int> & offset, int scale)
{
	int d = (scale == 4) ? 2 : (scale == 2) ? 4 : 7; //should be same as prepare_gray_vec and abs_diff_gray
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
abs_diff_gray means get_gray_vec and abs_diff
*/
static float abs_diff_gray(const Mat & m, Point c, const vector<float> & g, const vector<int> & offset, int scale)
{
	int d = (scale == 4) ? 2 : (scale == 2) ? 4 : 7; //should be same as prepare_gray_vec
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


/*
      31..24  23..16   15..8   7..0
opt0:				   method layer
opt1: debug_opt          var_th(16)
opt2:    fenzi0(16)      fenmu0(16)
opt3:    		         
*/
static void compute_weight_diff(const ImageDiff & gd, const ParamItem & param)
{
	int debug_opt = param.pi[1] >> 24 & 0xff;
	int method = param.pi[0] >> 8 & 0xff;
	int fenzi0 = param.pi[2] >> 16 & 0xffff; //initial fenzi
	int fenmu0 = param.pi[2] & 0xffff;		//initial fenmu
	float var_th = param.pi[1] & 0xffff;
	int scale = gd.cvar->rescale;
	int x_shift = (gd.dir) ? gd.cvar->max_lr_xshift / scale : gd.cvar->max_ud_xshift / scale; //dif size x
	int y_shift = (gd.dir) ? gd.cvar->max_lr_yshift / scale : gd.cvar->max_ud_yshift / scale; //dif size y
	int x_org0 = (gd.dir) ? gd.img_d0->v[0].d.cols - gd.overlap / scale : gd.shift / scale; //image initial bias
	int y_org0 = (gd.dir) ? gd.shift / scale : gd.img_d0->v[0].d.rows - gd.overlap / scale;
	gd.e->offset.x = (x_org0 - x_shift) * scale;
	gd.e->offset.y = (y_org0 - y_shift) * scale;
	qDebug("compute_weight_diff for img1=%s, img2=%s", gd.img_d0->filename.c_str(), gd.img_d1->filename.c_str());
	gd.e->dif.create(2 * y_shift + 1, 2 * x_shift + 1); //start compute dif
	if (gd.img_d0->is_black_img || gd.img_d1->is_black_img || gd.overlap <= 0) {
		gd.e->img_num = 0;
		gd.e->dif = 0;
		gd.e->compute_score();
		return;
	}
	Mat img[2];
	img[0] = gd.img_d0->v[0].d, img[1] = gd.img_d1->v[0].d; //load image
	CV_Assert(img[0].type() == img[1].type() && img[0].size() == img[1].size() && img[0].type() == CV_8UC1);
	vector<Point3i> corners[2]; //load corner
	if (gd.dir) {
		corners[0] = Mat_<Point3i>(gd.img_d0->v[1 + DIR_RIGHT].d); //corners[0] store img[0] corner
		corners[1] = Mat_<Point3i>(gd.img_d1->v[1 + DIR_LEFT].d); //corners[1] store img[1] corner
	}
	else {
		corners[0] = Mat_<Point3i>(gd.img_d0->v[1 + DIR_DOWN].d);
		corners[1] = Mat_<Point3i>(gd.img_d1->v[1 + DIR_UP].d);
	}
	vector<int> offset;
	prepare_gray_vec(img[0], offset, scale);
	vector<vector<float> > corner_gray[2]; //corner_gray is corner nearby gray "feature vector"
	vector<float> corner_gray_abs[2]; //corner_gray_abs = sum{corner_gray^2}
	vector<float> corner_var_th[2];
	for (int i = 0; i < 2; i++) { //prepare corner_gray and corner_gray_abs
		for (const auto & c : corners[i]) {
			vector<float> g;
			if (!get_gray_vec(img[i], Point(c.x, c.y), g, offset, scale)) //get corner nearby feature
				CV_Assert(0);
			corner_gray[i].push_back(g);
			float var = abs(g);
			corner_gray_abs[i].push_back(var);
			var = var / g.size();			
			if (var > var_th)
				var = pow(var * var_th * var_th * var_th, 0.25);
			if (method == 0)
				corner_var_th[i].push_back(var * c.z * 0.25);  //suppose c.z peak is 400
			else
				corner_var_th[i].push_back(var * c.z * 0.08); //suppose c.z peak is 1000
		}
	}	

	for (int yy = -y_shift; yy <= y_shift; yy++)
	for (int xx = -x_shift; xx <= x_shift; xx++) {
		int y_org = y_org0 + yy; //(x_org, y_org) is shift between img[0] and img[1]
		int x_org = x_org0 + xx;
		int y_overlap = img[0].rows - abs(y_org);
		int x_overlap = img[0].cols - abs(x_org);
		if (x_overlap >= MIN_OVERLAP && y_overlap >= MIN_OVERLAP) {
			float fenzi[2], fenmu[2];
			for (int i = 0; i < 2; i++) {
				int xs = (i == 0) ? x_org : -x_org;
				int ys = (i == 0) ? y_org : -y_org;
				int j = 0;
				fenzi[i] = fenzi0;
				fenmu[i] = fenmu0;
				for (const auto & c : corners[1 - i]) {
					float w = abs_diff_gray(img[i], Point(c.x + xs, c.y + ys), corner_gray[1 - i][j], offset, scale);
					if (w < 0) {
						j++;
						continue;
					}
					w = w / corner_gray_abs[1 - i][j]; //compute similar
					fenzi[i] += w * corner_var_th[1 - i][j];
					fenmu[i] += corner_var_th[1 - i][j];
					j++;
				}
			}			
			gd.e->dif(yy + y_shift, xx + x_shift) = 5000 * fenzi[0] / fenmu[0] + 5000 * fenzi[1] / fenmu[1];
		}
		else
			gd.e->dif(yy + y_shift, xx + x_shift) = DIFF_NOT_CONTACT;
	}
	gd.e->compute_score();
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
			unsigned sum;
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
			unsigned sum;
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
