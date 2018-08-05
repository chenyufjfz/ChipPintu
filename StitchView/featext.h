#ifndef FEATEXT_H
#define FEATEXT_H

#include <string>
#include <math.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "extractparam.h"

using namespace cv;
using namespace std;

class TuningPara {
public:
	vector<ParamItem> params;

public:
	TuningPara() {
	}

	void read(ExtractParam & ep, const string action_name) {
		ep.get_param(action_name, params);
	}

	TuningPara(ExtractParam & ep, const string action_name) {
		read(ep, action_name);
	}

	void read_file(const FileNode& node) {
		for (FileNodeIterator it = node.begin(); it != node.end(); it++) {
			ParamItem param;
			param.pi[0] = (int)(*it)["pi0"];
			param.pi[1] = (int)(*it)["pi1"];
			param.pi[2] = (int)(*it)["pi2"];
			param.pi[3] = (int)(*it)["pi3"];
			param.pi[4] = (int)(*it)["pi4"];
			param.pi[5] = (int)(*it)["pi5"];
			param.pi[6] = (int)(*it)["pi6"];
			param.pi[7] = (int)(*it)["pi7"];
			param.pi[8] = (int)(*it)["pi8"];
			param.pf = (float)(*it)["pf"];
			params.push_back(param);
		}
	}

	void write_file(FileStorage& fs) const {
		fs << "[";
		for (int i = 0; i < (int)params.size(); i++) {
			fs << "{" << "pi0" << params[i].pi[0];
			fs << "pi1" << params[i].pi[1];
			fs << "pi2" << params[i].pi[2];
			fs << "pi3" << params[i].pi[3];
			fs << "pi4" << params[i].pi[4];
			fs << "pi5" << params[i].pi[5];
			fs << "pi6" << params[i].pi[6];
			fs << "pi7" << params[i].pi[7];
			fs << "pi8" << params[i].pi[8] << "}";
		}
		fs << "]";
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
	ConfigPara() {
		clip_l = 0;
		clip_r = 0;
		clip_u = 0;
		clip_d = 0;
		img_num_w = 0;
		img_num_h = 0;
	}

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

#define MAKE_EDGE_IDX(x, y, e) ((e) << 31 | (y) << 16 | (x))
#define EDGE_Y(idx) ((idx) >> 16 & 0x7fff)
#define EDGE_X(idx) ((idx) & 0x7fff)
#define EDGE_E(idx) ((idx) >> 31 & 1)

#define MAKE_IMG_IDX(x, y) ((y) << 16 | (x))
#define IMG_Y(idx) ((idx) >> 16 & 0x7fff)
#define IMG_X(idx) ((idx) & 0x7fff)

class EdgeDiff {
public:
	Point offset; //it is nearby image top-left point - base image top-left point
	unsigned edge_idx;
	Mat_<unsigned char> diff;
	unsigned char maxd;
	unsigned score;	

public:
	void compute_score() {
		score = maxd;
	}

	void read_file(const FileNode& node) {
		offset.y = (int) node["oy"];
		offset.x = (int) node["ox"];
		maxd = (int)node["m"];
		read(node["diff"], diff);
		compute_score();
	}

	void write_file(FileStorage& fs) const {
		fs << "{" << "oy" << offset.y;
		fs << "ox" << offset.x;
		fs << "m" << maxd;
		fs << "diff" << diff << "}";
	}

	EdgeDiff * clone() const {
		EdgeDiff * new_diff = new EdgeDiff();
		new_diff->offset = offset;
		new_diff->edge_idx = edge_idx;
		new_diff->diff = diff.clone();
		new_diff->maxd = maxd;
		new_diff->score = score;
		return new_diff;
	}

	void get_img_idx(unsigned & img_idx0, unsigned & img_idx1) const
	{
		int x = EDGE_X(edge_idx);
		int y = EDGE_Y(edge_idx);
		if (EDGE_E(edge_idx)) {
			img_idx0 = MAKE_IMG_IDX(x, y);
			img_idx1 = MAKE_IMG_IDX(x + 1, y);
		}
		else {
			img_idx0 = MAKE_IMG_IDX(x, y);
			img_idx1 = MAKE_IMG_IDX(x, y + 1);
		}
	}

	unsigned char get_diff(Point o, int s) const
	{
		Point t = o - offset;
		t.y = t.y / s;
		t.x = t.x / s;
		if (t.y < 0 || t.x < 0 || t.y >= diff.rows || t.x >= diff.cols)
			return maxd;
		return diff.at<unsigned char>(t);
	}
	
	/*p1m1 is for this edge, p2m2 is for edge e2
	p1, p2 belong to same bundle; m1, m2 belong to same bundle*/
	void fw_merge(const EdgeDiff & e2, Point p1p2, Point m1m2, int s)
	{
		Point t = m1m2 - p1p2 + offset - e2.offset;
		t.y = t.y / s;
		t.x = t.x / s;
		maxd = max(maxd, e2.maxd);
		for (int y = 0; y < diff.rows; y++) {
			unsigned char * pd = diff.ptr<unsigned char>(y);
			int y1 = y + t.y;
			const unsigned char * pd1 = (y1 >= 0 && y1 < e2.diff.rows) ? e2.diff.ptr<unsigned char>(y1) : NULL;
			for (int x = 0; x < diff.cols; x++) {
				int x1 = x + t.x;
				if (pd1 && x1 >= 0 && x1 < e2.diff.cols)
					pd[x] = max(pd[x], pd1[x1]);
				else
					pd[x] = max(pd[x], e2.maxd);
			}
		}
		compute_score();
	}

	/*p1m1 is for this edge, m2p2 is for edge e2
	p1, p2 belong to same bundle; m1, m2 belong to same bundle*/
	void bw_merge(const EdgeDiff & e2, Point p1p2, Point m1m2, int s)
	{
		Point t = p1p2 - m1m2 - offset - e2.offset;
		t.y = t.y / s;
		t.x = t.x / s;
		maxd = max(maxd, e2.maxd);
		for (int y = 0; y < diff.rows; y++) {
			unsigned char * pd = diff.ptr<unsigned char>(y);
			int y1 = t.y - y;
			const unsigned char * pd1 = (y1 < 0 || y1 >= e2.diff.rows) ? NULL : e2.diff.ptr<unsigned char>(y1);
			for (int x = 0; x < diff.cols; x++) {
				int x1 = t.x - x;
				if (y1 >= 0 && y1 < e2.diff.rows && x1 >= 0 && x1 < e2.diff.cols)
					pd[x] = max(pd[x], pd1[x1]);
				else
					pd[x] = max(pd[x], e2.maxd);
			}
		}
		compute_score();
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
	bool is_valid() {
		return (cpara.img_num_h != 0 && cpara.img_num_w != 0 && !cpara.offset.empty());
	}
	/*
	Compute EdgeDiff based on cpara and tpara 
	*/
	void generate_feature_diff();
	void write_diff_file(string filename);	
	int read_diff_file(string filename);
	float get_progress() {
		return progress;
	}
	//i==0 means heng edge, i==1 means shu edge
	const EdgeDiff * get_edge(int i, int y, int x) const;
	const EdgeDiff * get_edge(int idx) const;
	const EdgeDiff * get_edge(int y0, int x0, int y1, int x1) const;
	ConfigPara get_config_para() const {
		return cpara;
	}
};

#endif // FEATEXT_H
