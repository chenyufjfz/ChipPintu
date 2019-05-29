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

#define PARALLEL 1
#define TOTAL_PROP 500

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

void edge_mixer(const Mat & img_in, const Mat & mask, Mat & img_out, char lut[], int lut_size)
{
    CV_Assert(img_in.type() == CV_16SC1 && mask.type() == CV_8UC1);
    Mat out(img_in.rows, img_in.cols, CV_8SC1);
    for (int y = 0; y < img_in.rows; y++) {
        const short * p_img_in = img_in.ptr<short>(y);
        const unsigned char * p_mask = mask.ptr<unsigned char>(y);
        char * p_img_out = out.ptr<char>(y);
        for (int x = 0; x < img_in.cols; x++)
		if (p_mask[x]) {
			p_img_out[x] = (abs(p_img_in[x]) >= lut_size) ? lut[lut_size - 1] : lut[abs(p_img_in[x])];
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
void prepare_grad(ImageData & img_d, const ParamItem & param)
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
		CV_Assert(out[i] > 0);
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
			}
		}
	}
	catch (std::exception & e) {
		qFatal("Error in prepare img %s, Exception %s.", img_dat.filename.c_str(), e.what());
	}
}
#define MIN_OVERLAP 3
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
	
	for (int row = start_y; row < cpara.img_num_h - 1; row += img_load_num) { //once load img_load_num
		for (int x = start_x; x < cpara.img_num_w; x++) {
			image[x & 1].resize(min(img_load_num + 1, cpara.img_num_h - row));
			for (int y = row; y < min(row + img_load_num + 1, cpara.img_num_h); y++)
				image[x & 1][y - row].filename = cpara.get_img_name(x, y);
#if PARALLEL
			QtConcurrent::blockingMap<vector<ImageData> >(image[x & 1], prepare_extract);
#else
			for (int i = 0; i < image[x & 1].size(); i++)
				prepare_extract(image[x & 1][i]);
#endif
			if (debug_en == 1) {
				char filename[100];
				sprintf(filename, "g0_M%d.jpg", 2 * (x - start_x));
				imwrite(filename, abs(image[x & 1][0].v[0].d));
				//sprintf(filename, "g0_M%d.jpg", 2 * (x - start_x) + 1);
				//imwrite(filename, abs(image[x & 1][0].v[2].d) * 8);
			}

			total_load += (int) image[x & 1].size() - 1;
			progress = (float)total_load / (cpara.img_num_h * cpara.img_num_w);
			if (x != start_x) {
				if (row + img_load_num + 1 >= cpara.img_num_h)
					gdiff.resize(image[x & 1].size() * 2 - 1);
				else
					gdiff.resize(image[x & 1].size() * 2 - 2);
				for (int i = 0; i < gdiff.size(); i++) {					
					if (i & 1) { //for up-down image feature extract
						gdiff[i].e = &edge[0][(row + i / 2) * cpara.img_num_w + x - 1];
						CV_Assert(gdiff[i].e->edge_idx == MAKE_EDGE_IDX(x - 1, row + i / 2, 0));
						gdiff[i].img_d0 = &image[(x - 1) & 1][i / 2];
						gdiff[i].img_d1 = &image[(x - 1) & 1][i / 2 + 1];
						gdiff[i].overlap = image[0][0].v[0].d.rows * cpara.rescale - (cpara.offset(row + i / 2 + 1, x - 1)[0] - cpara.offset(row + i / 2, x - 1)[0]);
						gdiff[i].shift = cpara.offset(row + i / 2 + 1, x - 1)[1] - cpara.offset(row + i / 2, x - 1)[1];
						gdiff[i].dir = 0;
						gdiff[i].cvar = &cpara;
						gdiff[i].tvar = &tpara;
					}
					else { //for left-right image feature extract
						gdiff[i].e = &edge[1][(row + i / 2) * (cpara.img_num_w - 1) + x - 1];
						CV_Assert(gdiff[i].e->edge_idx == MAKE_EDGE_IDX(x - 1, row + i / 2, 1));
						gdiff[i].img_d0 = &image[(x-1) & 1][i / 2];
						gdiff[i].img_d1 = &image[x & 1][i / 2];
						gdiff[i].overlap = image[0][0].v[0].d.cols * cpara.rescale - (cpara.offset(row + i / 2, x)[1] - cpara.offset(row + i / 2, x - 1)[1]);
						gdiff[i].shift = cpara.offset(row + i / 2, x)[0] - cpara.offset(row + i / 2, x - 1)[0];
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
			if (x + 1 == cpara.img_num_w) {
				gdiff.resize(image[x&1].size() - 1);
				for (int i = 0; i < gdiff.size(); i++) {
					gdiff[i].e = &edge[0][(row + i) * cpara.img_num_w + x];
					gdiff[i].img_d0 = &image[x & 1][i];
					gdiff[i].img_d1 = &image[x & 1][i + 1];
					gdiff[i].overlap = image[0][0].v[0].d.rows * cpara.rescale - (cpara.offset(row + i + 1, x)[0] - cpara.offset(row + i, x)[0]);
					gdiff[i].shift = cpara.offset(row + i + 1, x)[1] - cpara.offset(row + i, x)[1];
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
