#include "featext.h"
#include <iostream>
#include <vector>
#include <string>
#include <QtConcurrent>
#include "opencv2/highgui/highgui.hpp"

#define PARALLEL 1
#define TOTAL_PROP 1000

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

/*
Input img_in
Output grad_x
Output grad_y
grad_x and grad_y size is (img_in.cols -clip_l -clip_r, img_in.rows -clip_u -clip_d) / rescale
*/
void prepare_grad(const Mat & img_in, Mat & grad_x, Mat & grad_y, const TuningPara * tvar, const ConfigPara * cvar)
{
    Mat tailor_mat, rescaled_mat, filt_mat, edge_mask;

    tailor_mat = img_in(Rect(cvar->clip_l, cvar->clip_u, img_in.cols - cvar->clip_l - cvar->clip_r, img_in.rows - cvar->clip_u - cvar->clip_d));
	if (cvar->rescale != 1)
        resize(tailor_mat, rescaled_mat, Size(tailor_mat.cols / cvar->rescale, tailor_mat.rows / cvar->rescale));
    else
        rescaled_mat = tailor_mat;

    bilateralFilter(rescaled_mat, filt_mat, tvar->bfilt_w, tvar->bfilt_csigma, 0);
    
    Canny(filt_mat, edge_mask, tvar->canny_low_th, tvar->canny_high_th, 3);
    dilate(edge_mask, edge_mask, Mat());

    Sobel(filt_mat, grad_x, CV_16S, 1, 0, tvar->sobel_w);
    edge_mixer(grad_x, edge_mask, grad_x);

    Sobel(filt_mat, grad_y, CV_16S, 0, 1, tvar->sobel_w);
    edge_mixer(grad_y, edge_mask, grad_y);
}

/*
Input grad_x0, grad_y0, grad_x1, grad_y1, they are output from prepare_grad
Output diff (2 *yshift +1, 2 *xshift +1)
*/
int compute_diff(const Mat &grad_x0, const Mat &grad_y0, const Mat &grad_x1, const Mat & grad_y1, int xshift, int yshift, Mat &diff, int step=1)
{
#if 0
	static int idx = 0;
	string file_name = "gl0.jpg";
	file_name[2] = '0' + idx;
	imwrite(file_name, grad_x0);
	file_name = "gr0.jpg";
	file_name[2] = '0' + (idx++);
	imwrite(file_name, grad_x1);
#endif
    vector<int> out((2 * yshift + 1)*(2 * xshift + 1), 0);
    vector<int> num((2 * yshift + 1)*(2 * xshift + 1), 0);
    vector<double> normal_out((2 * yshift + 1)*(2 * xshift + 1));
    CV_Assert(grad_x0.type() == CV_8SC1 && grad_y0.type() == CV_8SC1 && grad_x0.size == grad_y0.size);
    CV_Assert(grad_x1.type() == CV_8SC1 && grad_y1.type() == CV_8SC1 && grad_x1.size == grad_y1.size);

    for (int y = 0; y < grad_x0.rows; y+=step) { //y,x is grad_x0, grad_y0
        const char * p0_grad_x = grad_x0.ptr<char>(y);
        const char * p0_grad_y = grad_y0.ptr<char>(y);
        for (int x = 0; x < grad_x0.cols; x+=step) {
            int y_org = y - yshift;
            int x_org = x - xshift;
            for (int yy = max(0, y_org); yy < min(grad_x1.rows, y + yshift+1); yy++) { //yy,xx is grad_x1, grad_y1
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
	double max=0;
    for (int i = 0; i < out.size(); i++) {
        normal_out[i] = normal_out[i] * TOTAL_PROP / dev;
        if (normal_out[i]>255)
            normal_out[i] = 255;
		if (normal_out[i]>max)
			max = normal_out[i];
    }

    diff.create(2 * yshift + 1, 2 * xshift + 1, CV_8U);
    for (int y = 0; y < 2 * yshift + 1; y++) {
		unsigned char * pdiff = diff.ptr<unsigned char>(2 * yshift - y);
        double * pout = &normal_out[y*(2 * xshift + 1)];
        for (int x = 0; x < 2 * xshift + 1; x++)
			pdiff[2 * xshift - x] = (unsigned char)(max - pout[x]);
    }
	return max;
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

struct Image2Grad {
    string filename;
	Mat * gx, *gy;
	TuningPara * tvar;
	ConfigPara * cvar;
};

struct GradDiff {
	Mat * gx0, *gy0;
	Mat * gx1, *gy1;
    int dir; //0 is up-down, 1 is left-right
    int overlap; //for left_right compare, image_width + n.x- n+1.x
    int shift; //for left_right compare, n+1.y - n.y
	EdgeDiff * e;
	TuningPara * tvar;
	ConfigPara * cvar;
};

void prepare_extract(const Image2Grad & image)
{
	Mat src = imread(image.filename, 0);
	CV_Assert(!src.empty());
	prepare_grad(src, *image.gx, *image.gy, image.tvar, image.cvar);
}

void extract_diff(GradDiff & gd)
{
	CV_Assert(!gd.gx0->empty() && !gd.gy0->empty() && !gd.gx1->empty() && !gd.gy1->empty());
	if (gd.dir) { //do left-right compare
		int start_x = gd.gx0->cols - gd.overlap / gd.cvar->rescale;		
		int shift = gd.shift / gd.cvar->rescale;
		gd.e->max = compute_diff(
			(*gd.gx0)(Rect(start_x, max(shift, 0), gd.overlap / gd.cvar->rescale, gd.gx0->rows - abs(shift))),
			(*gd.gy0)(Rect(start_x, max(shift, 0), gd.overlap / gd.cvar->rescale, gd.gy0->rows - abs(shift))),
			(*gd.gx1)(Rect(0, max(-shift, 0), gd.overlap / gd.cvar->rescale, gd.gx1->rows - abs(shift))),
			(*gd.gy1)(Rect(0, max(-shift, 0), gd.overlap / gd.cvar->rescale, gd.gy1->rows - abs(shift))),
			gd.cvar->max_lr_xshift / gd.cvar->rescale, gd.cvar->max_lr_yshift / gd.cvar->rescale, gd.e->diff);
		gd.e->offset_x = (start_x - gd.cvar->max_lr_xshift / gd.cvar->rescale) * gd.cvar->rescale;
		gd.e->offset_y = (shift - gd.cvar->max_lr_yshift / gd.cvar->rescale) * gd.cvar->rescale;
		gd.e->compute_score();
	}
	else { //do up-down compare
		int start_y = gd.gx0->rows - gd.overlap / gd.cvar->rescale;
		int shift = gd.shift / gd.cvar->rescale;
		gd.e->max = compute_diff(
			(*gd.gx0)(Rect(max(shift, 0), start_y, gd.gx0->cols - abs(shift), gd.overlap / gd.cvar->rescale)),
			(*gd.gy0)(Rect(max(shift, 0), start_y, gd.gy0->cols - abs(shift), gd.overlap / gd.cvar->rescale)),
			(*gd.gx1)(Rect(max(-shift, 0), 0, gd.gx1->cols - abs(shift), gd.overlap / gd.cvar->rescale)),
			(*gd.gy1)(Rect(max(-shift, 0), 0, gd.gy1->cols - abs(shift), gd.overlap / gd.cvar->rescale)),
			gd.cvar->max_ud_xshift / gd.cvar->rescale, gd.cvar->max_ud_yshift / gd.cvar->rescale, gd.e->diff);
		gd.e->offset_x = (shift - gd.cvar->max_ud_xshift / gd.cvar->rescale) * gd.cvar->rescale;
		gd.e->offset_y = (start_y - gd.cvar->max_ud_yshift / gd.cvar->rescale) * gd.cvar->rescale;
		gd.e->compute_score();
	}
}


FeatExt::FeatExt()
{
    tpara.bfilt_w = 8;
    tpara.bfilt_csigma = 23;
    tpara.canny_high_th = 200;
    tpara.canny_low_th = 100;
    tpara.sobel_w = 3;

    progress = 0;
}

void FeatExt::set_tune_para(const TuningPara & _tpara)
{
	tpara = _tpara;
}

void FeatExt::set_cfg_para(const ConfigPara & _cpara)
{
	cpara = _cpara;
}

void FeatExt::generate_feature_diff()
{
	vector<Mat> grad_x[2];
	vector<Mat> grad_y[2];

	progress = 0;
    int img_load_num = 15;
	int total_load = 0;
	vector<Image2Grad> image;
	vector<GradDiff> gdiff;

	grad_x[0].resize(img_load_num + 1);
	grad_y[0].resize(img_load_num + 1);
	grad_x[1].resize(img_load_num + 1);
	grad_y[1].resize(img_load_num + 1);
	edge[0].clear();
	edge[1].clear();
	edge[0].resize((cpara.img_num_h - 1) * cpara.img_num_w);
	edge[1].resize(cpara.img_num_h * (cpara.img_num_w - 1));

	for (int row = 0; row < cpara.img_num_h - 1; row += img_load_num) { //once load img_load_num
		for (int x = 0; x < cpara.img_num_w; x++) {
			image.resize(min(img_load_num + 1, cpara.img_num_h - row));
			for (int y = row; y < min(row + img_load_num + 1, cpara.img_num_h); y++) {
				char filename[50];
				sprintf(filename, "%d_%d.jpg", y + 1, x + 1); //image name postfix
				image[y - row].filename = cpara.img_path + filename;
				image[y - row].gx = &grad_x[x & 1][y - row];
				image[y - row].gy = &grad_y[x & 1][y - row];
				image[y - row].cvar = &cpara;
				image[y - row].tvar = &tpara;
			}			
#if PARALLEL
			QtConcurrent::blockingMap<vector<Image2Grad> >(image, prepare_extract);
#else
			for (int i = 0; i < image.size(); i++)
				prepare_extract(image[i]);
#endif
			total_load += (int) image.size() - 1;
			progress = (float)total_load / (cpara.img_num_h * cpara.img_num_w);
			if (x != 0) {
				if (row + img_load_num + 1 >= cpara.img_num_h)
					gdiff.resize(image.size() * 2 - 1);
				else
					gdiff.resize(image.size() * 2 - 2);
				for (int i = 0; i < gdiff.size(); i++) {					
					if (i & 1) { //for up-down image feature extract
						gdiff[i].e = &edge[0][(row + i / 2) * cpara.img_num_w + x - 1];
						gdiff[i].gx0 = &grad_x[(x - 1) & 1][i / 2];
						gdiff[i].gy0 = &grad_y[(x - 1) & 1][i / 2];
						gdiff[i].gx1 = &grad_x[(x - 1) & 1][i / 2 + 1];
						gdiff[i].gy1 = &grad_y[(x - 1) & 1][i / 2 + 1];
						gdiff[i].overlap = grad_x[0][0].rows * cpara.rescale - (cpara.offset(row + i / 2 + 1, x - 1)[0] - cpara.offset(row + i / 2, x - 1)[0]);
						gdiff[i].shift = cpara.offset(row + i / 2 + 1, x - 1)[1] - cpara.offset(row + i / 2, x - 1)[1];
						gdiff[i].dir = 0;
						gdiff[i].cvar = &cpara;
						gdiff[i].tvar = &tpara;
					}
					else { //for left-right image feature extract
						gdiff[i].e = &edge[1][(row + i / 2) * (cpara.img_num_w - 1) + x - 1];
						gdiff[i].gx0 = &grad_x[0][i / 2];
						gdiff[i].gy0 = &grad_y[0][i / 2];
						gdiff[i].gx1 = &grad_x[1][i / 2];
						gdiff[i].gy1 = &grad_y[1][i / 2];
						gdiff[i].overlap = grad_x[0][0].cols * cpara.rescale - (cpara.offset(row + i / 2, x)[1] - cpara.offset(row + i / 2, x - 1)[1]);
						gdiff[i].shift = cpara.offset(row + i / 2, x)[0] - cpara.offset(row + i / 2, x - 1)[0];
						gdiff[i].dir = 1;
						gdiff[i].cvar = &cpara;
						gdiff[i].tvar = &tpara;
					}
				}
#if PARALLEL
				QtConcurrent::blockingMap<vector<GradDiff> >(gdiff, extract_diff);
#else
				for (int i = 0; i < gdiff.size(); i++)
					extract_diff(gdiff[i]);
#endif										
			}
			if (x + 1 == cpara.img_num_w) {
				gdiff.resize(image.size() - 1);
				for (int i = 0; i < gdiff.size(); i++) {
					gdiff[i].e = &edge[0][(row + i) * cpara.img_num_w + x];
					gdiff[i].gx0 = &grad_x[x & 1][i];
					gdiff[i].gy0 = &grad_y[x & 1][i];
					gdiff[i].gx1 = &grad_x[x & 1][i + 1];
					gdiff[i].gy1 = &grad_y[x & 1][i + 1];
					gdiff[i].overlap = grad_x[0][0].rows * cpara.rescale - (cpara.offset(row + i + 1, x)[0] - cpara.offset(row + i, x)[0]);
					gdiff[i].shift = cpara.offset(row + i + 1, x)[1] - cpara.offset(row + i, x)[1];
					gdiff[i].dir = 0;
					gdiff[i].cvar = &cpara;
					gdiff[i].tvar = &tpara;
				}
#if PARALLEL
				QtConcurrent::blockingMap<vector<GradDiff> >(gdiff, extract_diff);
#else
				for (int i = 0; i < gdiff.size(); i++)
					extract_diff(gdiff[i]);
#endif
			}
		}
	}
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
			edge[0][y* cpara.img_num_w + x].compute_score();
		}

	for (int y = 0; y < cpara.img_num_h; y++)
		for (int x = 0; x < cpara.img_num_w - 1; x++) {
			char name[30];
			sprintf(name, "d1lr_%d_%d", y, x);
			fs[name] >> edge[1][y* (cpara.img_num_w - 1) + x];
			edge[1][y* (cpara.img_num_w - 1) + x].compute_score();
		}
	return 0;
}

Mat & FeatExt::get_diff0(int y, int x)
{
    CV_Assert(y < cpara.img_num_h - 1 && x < cpara.img_num_w);
    return edge[0][y* cpara.img_num_w + x].diff;
}

Mat & FeatExt::get_diff1(int y, int x)
{
    CV_Assert(y < cpara.img_num_h && x < cpara.img_num_w - 1);
    return edge[1][y* (cpara.img_num_w - 1) + x].diff;
}

Point FeatExt::get_diff_offset0(int y, int x)
{
    CV_Assert(y < cpara.img_num_h - 1 && x < cpara.img_num_w);
	return Point(edge[0][y* cpara.img_num_w + x].offset_x, edge[0][y* cpara.img_num_w + x].offset_y);
}

Point FeatExt::get_diff_offset1(int y, int x)
{
    CV_Assert(y < cpara.img_num_h && x < cpara.img_num_w - 1);
	return Point(edge[1][y* (cpara.img_num_w - 1) + x].offset_x, edge[1][y* (cpara.img_num_w - 1) + x].offset_y);
}

EdgeDiff * FeatExt::get_edge(int i, int y, int x)
{
	CV_Assert(i == 0 && y < cpara.img_num_h - 1 && x < cpara.img_num_w || i == 1 && y < cpara.img_num_h && x < cpara.img_num_w - 1);
	if (i == 0)
		return &edge[0][y* cpara.img_num_w + x];
	else
		return &edge[1][y* (cpara.img_num_w - 1) + x];
}
