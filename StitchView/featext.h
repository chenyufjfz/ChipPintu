#ifndef FEATEXT_H
#define FEATEXT_H

#include <string>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

class TuningPara {
public:
	int bfilt_w;
	int bfilt_csigma;
	int canny_high_th;
	int canny_low_th;
	int sobel_w;
public:
	void read_file(const FileNode& node) {
		bfilt_w = (int)node["bfilt_w"];
		bfilt_csigma = (int)node["bfilt_csigma"];
		canny_high_th = (int)node["canny_high_th"];
		canny_low_th = (int)node["canny_low_th"];
		sobel_w = (int)node["sobel_w"];
	}

	void write_file(FileStorage& fs) const {
		fs << "{" << "bfilt_w" << bfilt_w;
		fs << "bfilt_csigma" << bfilt_csigma;
		fs << "canny_high_th" << canny_high_th;
		fs << "canny_low_th" << canny_low_th;
		fs << "sobel_w" << sobel_w << "}";
	}
};

class ConfigPara {
public:
	string img_path;
	int clip_l, clip_r, clip_u, clip_d;
	int img_num_w, img_num_h;
	int rescale;
	int max_lr_xshift, max_lr_yshift; //LR Diff size = (max_lr_xshift / rescale, max_lr_yshift / rescale)
	int max_ud_xshift, max_ud_yshift; //UD Diff size = (max_lr_xshift / rescale, max_lr_yshift / rescale)
	Mat_<Vec2i> offset;

public:	 

	void read_file(const FileNode& node) {
		clip_l = (int) node["clip_left"];
		clip_r = (int) node["clip_right"];
		clip_u = (int) node["clip_up"];
		clip_d = (int) node["clip_down"];
		img_path = (string)node["image_path"];
		img_num_w = (int)node["img_num_w"];
		img_num_h = (int)node["img_num_h"];
		rescale = (int)node["scale"];
		max_lr_xshift = (int)node["max_lr_xshift"];
		max_lr_yshift = (int)node["max_lr_yshift"];
		max_ud_xshift = (int)node["max_ud_xshift"];
		max_ud_yshift = (int)node["max_ud_yshift"];
		read(node["offset"], offset);
	}

	void write_file(FileStorage& fs) const {
		fs << "{" << "clip_left" << clip_l;
		fs << "clip_right" << clip_r;
		fs << "clip_up" << clip_u;
		fs << "clip_down" << clip_d;
		fs << "image_path" << img_path;
		fs << "img_num_w" << img_num_w;
		fs << "img_num_h" << img_num_h;
		fs << "scale" << rescale;
		fs << "max_lr_xshift" << max_lr_xshift;
		fs << "max_lr_yshift" << max_lr_yshift;
		fs << "max_ud_xshift" << max_ud_xshift;
		fs << "max_ud_yshift" << max_ud_yshift;
		fs << "offset" << offset << "}";
	}

	int right_bound() const {
		return offset(0, img_num_w - 1)[1] + offset(0, 1)[1] * 3/2 - offset(0,0)[1];
	}
	
	int bottom_bound() const {
		return offset(img_num_h - 1, 0)[0] + offset(1, 0)[0] * 3/2 - offset(0,0)[0];
	}
};

class EdgeDiff {
public:
	Mat_<unsigned char> diff;
	unsigned char max;
	unsigned short score;
	int offset_y, offset_x;

public:
	void read_file(const FileNode& node) {
		offset_y = (int) node["oy"];
		offset_x = (int) node["ox"];
		max = (int)node["m"];
		read(node["diff"], diff);
	}

	void write_file(FileStorage& fs) const {
		fs << "{" << "oy" << offset_y;
		fs << "ox" << offset_x;
		fs << "m" << max;
		fs << "diff" << diff << "}";
	}

	void compute_score() {
		unsigned min = 1000, submin = 1000, thirdmin = 1000;
		for (int y = 0; y < diff.rows; y++) {
			unsigned char * pdiff = diff.ptr(y);
			for (int x = 0; x < diff.cols; x++) {
				if (pdiff[x] < min) {
					thirdmin = submin;
					submin = min;
					min = pdiff[x];
				} else
					if (pdiff[x] < submin) {
						thirdmin = submin;
						submin = pdiff[x];
					}
					else
						if (pdiff[x] < thirdmin)
							thirdmin = pdiff[x];
			}
		}
		score = (submin - min) + (thirdmin - submin) / 3;
	}
};

void write(FileStorage& fs, const std::string&, const TuningPara& x);
void read(const FileNode& node, TuningPara& x, const TuningPara& default_value = TuningPara());

void write(FileStorage& fs, const std::string&, const ConfigPara& x);
void read(const FileNode& node, ConfigPara& x, const ConfigPara& default_value = ConfigPara());

void write(FileStorage& fs, const std::string&, const EdgeDiff& x);
void read(const FileNode& node, EdgeDiff& x, const EdgeDiff& default_value = EdgeDiff());

class FeatExt
{
protected:
	vector<EdgeDiff> edge[2]; //edge[0] is for up-down compare, edge[1] is left-right compare
	TuningPara tpara;
	ConfigPara cpara;
	float progress;

public:
    FeatExt();
	void set_tune_para(const TuningPara & _tpara);
	void set_cfg_para(const ConfigPara & _cpara);
	/*
	Compute EdgeDiff based on cpara and tpara 
	*/
	void generate_feature_diff();
	void write_diff_file(string filename);	
	int read_diff_file(string filename);
	float get_progress() {
		return progress;
	}
    Mat & get_diff0(int y, int x);
    Mat & get_diff1(int y, int x);
    Point get_diff_offset0(int y, int x);
    Point get_diff_offset1(int y, int x);
	EdgeDiff * get_edge(int i, int y, int x);
};

#endif // FEATEXT_H
