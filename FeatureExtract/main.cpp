#include "featurewindow.h"
#include <QApplication>
#include <QtConcurrent>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include <emmintrin.h>
#include <iostream>
#include <vector>
#include <string>
using namespace cv;
using namespace std;

#define PARALLEL 1
#define TOTAL_PROP 1000


struct {
	vector<Mat> grad_x[2];
	vector<Mat> grad_y[2];
	vector<Mat> diff;
	string img_path;
	int clip_l, clip_r, clip_u, clip_d;
	int img_num_w, img_num_h;
	int rescale;
	int bfilt_w;
	int bfilt_csigma;
	int canny_high_th;
	int canny_low_th;
	int sobel_w;
	int max_lr_xshift, max_lr_yshift;
	int max_ud_xshift, max_ud_yshift;
	int init_overlap_w;
	int init_overlap_h;
	Mat_<Vec2i> offset;
} gvar;

void edge_mixer(const Mat & img_in, const Mat & mask, Mat & img_out)
{

	CV_Assert(img_in.type() == CV_16SC1 && mask.type() == CV_8UC1);
	Mat out(img_in.rows, img_in.cols, CV_8SC1);
    for (int y = 0; y < img_in.rows; y++) {
		const unsigned short * p_img_in = img_in.ptr<unsigned short>(y);
		const unsigned char * p_mask = mask.ptr<unsigned char>(y);
		unsigned char * p_img_out = out.ptr<unsigned char>(y);
        for (int x = 0; x < img_in.cols; x++)
        if (p_mask[x])
            p_img_out[x] = p_img_in[x] >> 4;
        else
            p_img_out[x] = 0;

    }
    img_out = out;
}

void prepare_grad(const Mat & img_in, Mat & grad_x, Mat & grad_y, int rescale = 4, int bfilt_w = 8, int bfilt_csigma = 23,
                  int canny_high_th=200, int canny_low_th=100, int sobel_w=3)
{
    Mat filt_mat, edge_mask;

	bilateralFilter(img_in(Rect(gvar.clip_l, gvar.clip_u, img_in.cols - gvar.clip_l- gvar.clip_r, img_in.rows -gvar.clip_u-gvar.clip_d)), 
		filt_mat, bfilt_w, bfilt_csigma, 0);
	resize(filt_mat, filt_mat, Size(filt_mat.cols / rescale, filt_mat.rows / rescale));
    Canny(filt_mat, edge_mask, canny_low_th, canny_high_th, 3);
    dilate(edge_mask, edge_mask, Mat());

	Sobel(filt_mat, grad_x, CV_16S, 1, 0, sobel_w);
    edge_mixer(grad_x, edge_mask, grad_x);

    Sobel(filt_mat, grad_y, CV_16S, 0, 1, sobel_w);
    edge_mixer(grad_y, edge_mask, grad_y);
}

void compute_diff(const Mat &grad_x0, const Mat &grad_y0, const Mat &grad_x1, const Mat & grad_y1, int xshift, int yshift, Mat &diff, int step=1)
{
	vector<int> out((2 * yshift + 1)*(2 * xshift + 1), 0);
	vector<int> num((2 * yshift + 1)*(2 * xshift + 1), 0);
	vector<double> normal_out((2 * yshift + 1)*(2 * xshift + 1));
    CV_Assert(grad_x0.type() == CV_8SC1 && grad_y0.type() == CV_8SC1 && grad_x0.size == grad_y0.size);
    CV_Assert(grad_x1.type() == CV_8SC1 && grad_y1.type() == CV_8SC1 && grad_x1.size == grad_y1.size);

    for (int y = 0; y < grad_x0.rows; y+=step) {
        const char * p0_grad_x = grad_x0.ptr<char>(y);
		const char * p0_grad_y = grad_y0.ptr<char>(y);
		for (int x = 0; x < grad_x0.cols; x+=step) {
			int y_org = y - yshift;
			int x_org = x - xshift;
			for (int yy = max(0, y_org); yy < min(grad_x1.rows, y + yshift+1); yy++) {
				int * pout = &out[(yy - y_org) * (2 * xshift + 1)];
				int * pnum = &num[(yy - y_org) * (2 * xshift + 1)];
				const char * p1_grad_x = grad_x1.ptr<char>(yy);
				const char * p1_grad_y = grad_y1.ptr<char>(yy);
				for (int xx = max(0, x_org); xx < min(grad_x1.cols, x + xshift + 1); xx++) {
					pout[xx - x_org] += abs(p0_grad_x[x] - p1_grad_x[xx]);
					pout[xx - x_org] += abs(p0_grad_y[x] - p1_grad_y[xx]);
					pnum[xx - x_org]++;
				}
			}
        }
    }

	double avg=0, dev=0;
	for (int i = 0; i < out.size(); i++) {
		normal_out[i] = (double)out[i] / num[i];
		avg += normal_out[i];
	}
		
	avg = avg / out.size() *1.03;
	for (int i = 0; i < out.size(); i++) {
		if (normal_out[i] > avg)
			normal_out[i] = 0;
		else
			normal_out[i] = (avg - normal_out[i]) * (avg - normal_out[i]);
		dev += normal_out[i];
	}
	for (int i = 0; i < out.size(); i++) {
		normal_out[i] = normal_out[i] * TOTAL_PROP / dev;
		if (normal_out[i]>255)
			normal_out[i] = 255;
	}

	diff.create(2 * yshift + 1, 2 * xshift + 1, CV_8U);
	for (int y = 0; y < 2 * yshift + 1; y++) {
		unsigned char * pdiff = diff.ptr<unsigned char>(y);
		double * pout = &normal_out[y*(2 * xshift + 1)];
		for (int x = 0; x < 2 * xshift + 1; x++)
			pdiff[x] = (unsigned char) pout[x];
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

struct Image2Grad {
	string filename;
	int grad_index;
	int grad_heap;
};

struct GradDiff {
	int gi0;
	int gh0;
	int gi1;
	int gh1;
	int dir;
	int overlap; //w - n+1.x + n.x
	int shift; //n+1.y - n.y
	int diff_index;
};

class ParallelExtractDiff {
public:	
	ParallelExtractDiff(int grad_num = 2, int diff_num = -1)
	{
		gvar.grad_x[0].resize(grad_num);
		gvar.grad_y[0].resize(grad_num);
		gvar.grad_x[1].resize(grad_num);
		gvar.grad_y[1].resize(grad_num);
		if (diff_num == -1)
			diff_num = grad_num * 2;
		gvar.diff.resize(diff_num);
	}

	~ParallelExtractDiff()
	{
		gvar.grad_x[0].clear();
		gvar.grad_y[0].clear();
		gvar.grad_x[1].clear();
		gvar.grad_y[1].clear();
		gvar.diff.clear();
	}

	void operator() (const Image2Grad & image)
	{
		Mat src = imread(image.filename, 0);
		CV_Assert(!src.empty() && image.grad_heap < 2 && image.grad_index < gvar.grad_x[0].size());
		prepare_grad(src, gvar.grad_x[image.grad_heap][image.grad_index], gvar.grad_y[image.grad_heap][image.grad_index],
			gvar.rescale, gvar.bfilt_w, gvar.bfilt_csigma, gvar.canny_high_th, gvar.canny_low_th, gvar.sobel_w);
	}

	void operator() (const GradDiff & gd)
	{
		CV_Assert(gd.gi0 < gvar.grad_x[0].size() && gd.gh0 < 2 && gd.gi1 < gvar.grad_x[0].size() && gd.gh1 < 2);
		CV_Assert(!gvar.grad_x[gd.gh0][gd.gi0].empty() && !gvar.grad_x[gd.gh1][gd.gi1].empty());
		if (gd.dir == 0) {
			int start_x = gvar.grad_x[gd.gh0][gd.gi0].cols - gd.overlap / gvar.rescale;
			compute_diff(
				gvar.grad_x[gd.gh0][gd.gi0](Rect(start_x, max(gd.shift, 0), gd.overlap / gvar.rescale, gvar.grad_x[gd.gh0][gd.gi0].rows - abs(gd.shift))),
				gvar.grad_y[gd.gh0][gd.gi0](Rect(start_x, max(gd.shift, 0), gd.overlap / gvar.rescale, gvar.grad_y[gd.gh0][gd.gi0].rows - abs(gd.shift))),
				gvar.grad_x[gd.gh1][gd.gi1](Rect(0, min(gd.shift, 0), gd.overlap / gvar.rescale, gvar.grad_x[gd.gh1][gd.gi1].rows - abs(gd.shift))),
				gvar.grad_y[gd.gh1][gd.gi1](Rect(0, min(gd.shift, 0), gd.overlap / gvar.rescale, gvar.grad_y[gd.gh1][gd.gi1].rows - abs(gd.shift))),
				gvar.max_lr_xshift / gvar.rescale, gvar.max_lr_yshift / gvar.rescale, gvar.diff[gd.diff_index]);
		}
		else {
			int start_y = gvar.grad_x[gd.gh0][gd.gi0].rows - gd.overlap / gvar.rescale;
			compute_diff(
				gvar.grad_x[gd.gh0][gd.gi0](Rect(max(gd.shift, 0), start_y, gvar.grad_x[gd.gh0][gd.gi0].cols - abs(gd.shift), gd.overlap / gvar.rescale)),
				gvar.grad_y[gd.gh0][gd.gi0](Rect(max(gd.shift, 0), start_y, gvar.grad_x[gd.gh0][gd.gi0].cols - abs(gd.shift), gd.overlap / gvar.rescale)),
				gvar.grad_x[gd.gh1][gd.gi1](Rect(min(gd.shift, 0), 0, gvar.grad_x[gd.gh0][gd.gi0].cols - abs(gd.shift), gd.overlap / gvar.rescale)),
				gvar.grad_y[gd.gh1][gd.gi1](Rect(min(gd.shift, 0), 0, gvar.grad_x[gd.gh0][gd.gi0].cols - abs(gd.shift), gd.overlap / gvar.rescale)),
				gvar.max_ud_xshift / gvar.rescale, gvar.max_ud_yshift / gvar.rescale, gvar.diff[gd.diff_index]);
		}		
	}
};

//before calling it, should initialize gvar
void compute_init_estimate(int dir, int line)
{
	vector<Image2Grad> image;
	vector<GradDiff> gdiff;
	int img_num = (dir == 0) ? gvar.img_num_w : gvar.img_num_h;
	ParallelExtractDiff ext_diff(img_num);

	image.resize(img_num);
	for (int i = 0; i < img_num; i++) {
		char filename[50];
		if (dir == 0)
			sprintf(filename, "%d_%d.jpg", line + 1, i + 1);
		else
			sprintf(filename, "%d_%d.jpg", i + 1, line + 1);
		image[i].filename = gvar.img_path + filename;
		image[i].grad_heap = 0;
		image[i].grad_index = i;
	}
#if PARALLEL
	QtConcurrent::blockingMap<vector<Image2Grad>>(image, ext_diff);
#else
	for (int i = 0; i < image.size(); i++)
		ext_diff(image[i]);
#endif

	gdiff.resize(img_num - 1);
	for (int i = 0; i < img_num - 1; i++) {
		gdiff[i].diff_index = i;
		gdiff[i].gh0 = 0;
		gdiff[i].gi0 = i;
		gdiff[i].gh1 = 0;
		gdiff[i].gi1 = i + 1;
		gdiff[i].overlap = (dir == 0) ? gvar.init_overlap_w : gvar.init_overlap_h;
		gdiff[i].shift = 0;
		gdiff[i].dir = dir;
	}

#if PARALLEL
	QtConcurrent::blockingMap<vector<GradDiff>>(gdiff, ext_diff);
#else
	for (int i = 0; i < gdiff.size(); i++)
		ext_diff(gdiff[i]);
#endif
}

void generate_init_offset()
{
	int right_bound = 0, bottom_bound = 0;
	gvar.clip_l = 6;
	gvar.clip_r = 0;
	gvar.clip_u = 0;
	gvar.clip_d = 12;
	gvar.rescale = 4;
	gvar.bfilt_w = 8;
	gvar.bfilt_csigma = 23;
	gvar.canny_high_th = 200;
	gvar.canny_low_th = 100;
	gvar.sobel_w = 3;
	gvar.max_lr_xshift = 144;
	gvar.max_lr_yshift = 100;
	gvar.max_ud_xshift = 100;
	gvar.max_ud_yshift = 128;
	gvar.init_overlap_w = 300;
	gvar.init_overlap_h = 260;
	gvar.img_path = "F:/chenyu/work/ChipStitch/data/m3/";
	gvar.img_num_w = 6;
	gvar.img_num_h = 6;
	gvar.offset.create(gvar.img_num_h, gvar.img_num_w);

#if 1
	for (int y = 0; y < gvar.img_num_h; y++) {
		for (int x = 0; x < gvar.img_num_w; x++) {
			gvar.offset(y, x)[1] = 1780 * x;
			gvar.offset(y, x)[0] = 1572 * y;
		}
	}
	right_bound = gvar.offset(0, gvar.img_num_w - 1)[1] + 2048;
	bottom_bound = gvar.offset(gvar.img_num_h - 1, 0)[0] + 1770;
#else
	compute_init_estimate(0, 2);
	
	for () {
		if (x == 0)
			for (int y = 0; y < gvar.img_num_h; y++)
				gvar.offset(y, 0)[1] = 0;
		else {
			double minval, maxval;
			Point minloc, maxloc;
			minMaxLoc(gvar.diff[x - 1], &minval, &maxval, &minloc, &maxloc);
			int overlap_x = gvar.init_overlap_w / gvar.rescale - gvar.max_lr_xshift / gvar.rescale + maxloc.x;
			for (int y = 0; y < gvar.img_num_h; y++) 
				gvar.offset(y, x)[1] = gvar.offset(y, x - 1)[1] + (gvar.grad_x[0][0].cols - overlap_x) * gvar.rescale;	
		}
	}
	right_bound = gvar.offset(0, gvar.img_num_w - 1)[1] + gvar.grad_x[0][0].cols * gvar.rescale;

	compute_init_estimate(1, 13);

	for (int y = 0; y < gvar.img_num_h; y++) {
		if (y == 0)
			for (int x = 0; x < gvar.img_num_w; x++)
				gvar.offset(0, x)[0] = 0;
		else {
			double minval, maxval;
			Point minloc, maxloc;
			minMaxLoc(gvar.diff[y - 1], &minval, &maxval, &minloc, &maxloc);
			int overlap_y = gvar.init_overlap_h / gvar.rescale - gvar.max_ud_yshift / gvar.rescale + maxloc.y;
			for (int x = 0; x < gvar.img_num_w; x++)
				gvar.offset(y, x)[0] = gvar.offset(y - 1, x)[0] + (gvar.grad_x[0][0].rows - overlap_y) * gvar.rescale;
		}
	}
	bottom_bound = gvar.offset(gvar.img_num_h - 1, 0)[0] + gvar.grad_x[0][0].rows * gvar.rescale;
#endif

	FileStorage fs(".\\coarse_layer_cfg.xml", FileStorage::WRITE);
	fs << "clip_left" << gvar.clip_l;
	fs << "clip_right" <<gvar.clip_r;
	fs << "clip_up" << gvar.clip_u;
	fs << "clip_down" << gvar.clip_d;
	fs << "image_path" << gvar.img_path;
	fs << "img_num_w" << gvar.img_num_w;
	fs << "img_num_h" << gvar.img_num_h;
	fs << "right_bound" << right_bound;
	fs << "bottom_bound" << bottom_bound;
	fs << "scale" << gvar.rescale;
	fs << "offset" << gvar.offset;
	fs.release();
}

//depend on gvar
void generate_feature_diff()
{	
	int img_load_num = 10;
	int total_load = 0;
	vector<Image2Grad> image;
	vector<GradDiff> gdiff;
	ParallelExtractDiff ext_diff(img_load_num + 1, img_load_num * 2 + 2);
	gvar.max_lr_xshift = max(gvar.max_lr_xshift/2, 1);
	gvar.max_lr_yshift = max(gvar.max_lr_yshift/2, 1);
	gvar.max_ud_xshift = max(gvar.max_ud_xshift/2, 1);
	gvar.max_ud_yshift = max(gvar.max_ud_yshift/2, 1);
	FileStorage fs(".\\feature.xml", FileStorage::WRITE);
	FILE *fp = fopen("feature.bin", "w");

	for (int row = 0; row < gvar.img_num_h - 1; row += img_load_num) {
		for (int x = 0; x < gvar.img_num_w; x++) {
			image.resize(min(img_load_num + 1, gvar.img_num_h - row));
			for (int y = row; y < min(row + img_load_num + 1, gvar.img_num_h); y++) {
				char filename[50];
				sprintf(filename, "%d_%d.jpg", y + 1, x + 1);
				image[y - row].filename = gvar.img_path + filename;
				image[y - row].grad_heap = x & 1;
				image[y - row].grad_index = y - row;
			}
			total_load += image.size() - 1;
#if PARALLEL
			QtConcurrent::blockingMap<vector<Image2Grad>>(image, ext_diff);
#else
			for (int i = 0; i < image.size(); i++)
				ext_diff(image[i]);
#endif
			
			if (x != 0) {
				if (row + img_load_num + 1 >= gvar.img_num_h)
					gdiff.resize(image.size() * 2 - 1);
				else
					gdiff.resize(image.size() * 2 - 2);
				for (int i = 0; i < gdiff.size(); i++) {
					gdiff[i].diff_index = i;
					if (i & 1) {
						gdiff[i].gh0 = (x - 1) & 1;
						gdiff[i].gi0 = i / 2;
						gdiff[i].gh1 = (x - 1) & 1;
						gdiff[i].gi1 = i / 2 + 1;
						gdiff[i].overlap = gvar.grad_x[0][0].rows*gvar.rescale - (gvar.offset(row + i / 2 + 1, x - 1)[0] - gvar.offset(row + i / 2, x - 1)[0]);
						gdiff[i].shift = gvar.offset(row + i / 2 + 1, x - 1)[1] - gvar.offset(row + i / 2, x - 1)[1];
						gdiff[i].dir = 1;
					}
					else {
						gdiff[i].gh0 = 0;
						gdiff[i].gi0 = i / 2;
						gdiff[i].gh1 = 1;
						gdiff[i].gi1 = i / 2;
						gdiff[i].overlap = gvar.grad_x[0][0].cols*gvar.rescale - (gvar.offset(row + i / 2, x)[1] - gvar.offset(row + i / 2, x - 1)[1]);
						gdiff[i].shift = gvar.offset(row + i / 2, x)[0] - gvar.offset(row + i / 2, x - 1)[0];
						gdiff[i].dir = 0;
					}
				}
#if PARALLEL
				QtConcurrent::blockingMap<vector<GradDiff>>(gdiff, ext_diff);
#else
				for (int i = 0; i < gdiff.size(); i++)
					ext_diff(gdiff[i]);
#endif
				printf("%d%%", total_load * 100 / (gvar.img_num_h * gvar.img_num_w));
				for (int i = 0; i < gdiff.size(); i++) {
					char name[50];
					sprintf(name, "d_%d_%d", i, x-1);
					fs << name << gvar.diff[i];
#if 1
					double minval, maxval;
					Point minloc, maxloc;
					minMaxLoc(gvar.diff[i], &minval, &maxval, &minloc, &maxloc);
					int overlap;
					if (i&1)
						overlap = gdiff[i].overlap / gvar.rescale - gvar.max_ud_yshift / gvar.rescale + maxloc.y;
					else
						overlap = gdiff[i].overlap / gvar.rescale - gvar.max_lr_xshift / gvar.rescale + maxloc.x;
					sprintf(name, "o_%d_%d", i, x - 1);
					fs << name << overlap;
#endif
					write_mat_binary(fp, i, x - 1, gvar.diff[i]);
				}
				if (x + 1 == gvar.img_num_w) {
					gdiff.resize(image.size() - 1);
					for (int i = 0; i < gdiff.size(); i++) {
						gdiff[i].gh0 = x & 1;
						gdiff[i].gi0 = i;
						gdiff[i].gh1 = x & 1;
						gdiff[i].gi1 = i + 1;
						gdiff[i].overlap = gvar.grad_x[0][0].rows*gvar.rescale - (gvar.offset(row + i+ 1, x)[0] - gvar.offset(row + i, x)[0]);
						gdiff[i].shift = gvar.offset(row + i + 1, x)[1] - gvar.offset(row + i, x)[1];
						gdiff[i].dir = 1;
					}
					for (int i = 0; i < gdiff.size(); i++) {
						char name[50];
						sprintf(name, "d_%d_%d", i * 2 + 1, x);
						fs << name << gvar.diff[i];
#if 1
						double minval, maxval;
						Point minloc, maxloc;
						minMaxLoc(gvar.diff[i], &minval, &maxval, &minloc, &maxloc);
						int overlap;
						overlap = gdiff[i].overlap / gvar.rescale - gvar.max_ud_yshift / gvar.rescale + maxloc.y;
						sprintf(name, "o_%d_%d", i * 2 + 1, x);
						fs << name << overlap;
#endif
						write_mat_binary(fp, i * 2 + 1, x, gvar.diff[i]);
					}
				}				
			}
		}
	}
	fs.release();
	fclose(fp);
}

int main(int argc, char** argv)
{
	/*Mat src0 = imread(filename, 0);
	Mat src1 = imread("F:/chenyu/work/ChipStitch/data/m3/10_49.jpg", 0);
    if(src0.empty() || src1.empty()) {
        cout << "can not open " << filename << endl;
        return -1;
    } else
        cout << "src row=" << src0.rows << ",col=" << src0.cols << ",depth=" <<src0.depth()<<
                ",channel=" << src0.channels() << '\n';
    Mat grad_x0, grad_y0, grad_x1, grad_y1, diff;
    prepare_grad(src0, grad_x0, grad_y0);
	prepare_grad(src1, grad_x1, grad_y1);
	compute_diff(grad_x0(Rect(420, 0, grad_x0.cols - 420, grad_x0.rows)), grad_y0(Rect(420, 0, grad_y0.cols - 420, grad_y0.rows)),
		grad_x1(Rect(0, 0, 100, grad_x1.rows)), grad_y1(Rect(0, 0, 100, grad_y1.rows)), 50, 50, diff);
	double min, max;
	Point min_loc, max_loc;
	minMaxLoc(diff, &min, &max, &min_loc, &max_loc);
	convertScaleAbs(grad_x0, grad_x0);
	FileStorage fs(".\\diff.xml", FileStorage::WRITE);
	fs << "diff" << diff;
	fs.release();*/
	/*
	generate_init_offset();
	generate_feature_diff();
	return 0;*/
	gvar.clip_l = 6;
	gvar.clip_r = 0;
	gvar.clip_u = 0;
	gvar.clip_d = 12;
	gvar.rescale = 4;
	gvar.bfilt_w = 8;
	gvar.bfilt_csigma = 23;
	gvar.canny_high_th = 200;
	gvar.canny_low_th = 100;
	gvar.sobel_w = 3;
	gvar.max_lr_xshift = 144;
	gvar.max_lr_yshift = 100;
	gvar.max_ud_xshift = 100;
	gvar.max_ud_yshift = 128;
	gvar.init_overlap_w = 300; 
	gvar.init_overlap_h = 260;

	ParallelExtractDiff ext_diff(2, 2);
	vector<Image2Grad> image(4);
	vector<GradDiff> gdiff(2);
	image[0].filename = "F:/chenyu/work/ChipStitch/data/m3/4_1.jpg";
	image[0].grad_heap = 0;
	image[0].grad_index = 0;
	image[1].filename = "F:/chenyu/work/ChipStitch/data/m3/4_2.jpg";
	image[1].grad_heap = 1;
	image[1].grad_index = 0;
	image[2].filename = "F:/chenyu/work/ChipStitch/data/m3/3_3.jpg";
	image[2].grad_heap = 0;
	image[2].grad_index = 1;
	image[3].filename = "F:/chenyu/work/ChipStitch/data/m3/4_3.jpg";
	image[3].grad_heap = 1;
	image[3].grad_index = 1;
#if PARALLEL
	QtConcurrent::blockingMap<vector<Image2Grad>>(image, ext_diff);
#else
	for (int i = 0; i < image.size(); i++)
		ext_diff(image[i]);
#endif
		

	gdiff[0].diff_index = 0;
	gdiff[0].gh0 = 0;
	gdiff[0].gi0 = 0;
	gdiff[0].gh1 = 1;
	gdiff[0].gi1 = 0;
	gdiff[0].dir = 0;
	gdiff[0].overlap = gvar.init_overlap_w;
	gdiff[0].shift = 0;
	gdiff[1].diff_index = 1;
	gdiff[1].gh0 = 0;
	gdiff[1].gi0 = 1;
	gdiff[1].gh1 = 1;
	gdiff[1].gi1 = 1;
	gdiff[1].dir = 1;
	gdiff[1].overlap = gvar.init_overlap_h;
	gdiff[1].shift = 0;
#if PARALLEL
	QtConcurrent::blockingMap<vector<GradDiff>>(gdiff, ext_diff);
#else
	for (int i = 0; i < gdiff.size(); i++)
		ext_diff(gdiff[i]);
#endif

	double minval, maxval;
	Point minloc, maxloc;
	minMaxLoc(gvar.diff[0], &minval, &maxval, &minloc, &maxloc);
	FileStorage fs(".\\diff.xml", FileStorage::WRITE);
	FILE * fp = fopen("diff.bin", "wb");

	fs << "diff0" << gvar.diff[0];
	fs << "maxloc0" << maxloc;
	int overlap_x = gvar.init_overlap_w / gvar.rescale - gvar.max_lr_xshift / gvar.rescale + maxloc.x;
	fs << "overlap_x" << overlap_x;
	imshow("x0", gvar.grad_y[0][0](Rect(gvar.grad_x[0][0].cols - overlap_x, 0, overlap_x, gvar.grad_x[0][0].rows)));
	imshow("x1", gvar.grad_y[1][0](Rect(0, 0, overlap_x, gvar.grad_x[1][0].rows)));
	write_mat_binary(fp, 2, 3, gvar.diff[0]);

	minMaxLoc(gvar.diff[1], &minval, &maxval, &minloc, &maxloc);
	fs << "diff1" << gvar.diff[1];
	fs << "maxloc1" << maxloc;	
	int overlap_y = gvar.init_overlap_h / gvar.rescale - gvar.max_ud_yshift / gvar.rescale + maxloc.y;
	fs << "overlap_y" << overlap_y;
	imshow("y0", gvar.grad_y[0][1](Rect(0, gvar.grad_x[0][1].rows - overlap_y, gvar.grad_x[0][1].cols, overlap_y)));
	imshow("y1", gvar.grad_y[1][1](Rect(0, 0, gvar.grad_x[0][1].cols, overlap_y)));
	write_mat_binary(fp, 4, 5, gvar.diff[1]);
	waitKey();
	fs.release();
	fclose(fp);
    return 0;
}

/*
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    FeatureWindow w;
    w.show();

    return a.exec();
}
*/
