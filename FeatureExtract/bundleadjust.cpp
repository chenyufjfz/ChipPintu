#include "bundleadjust.h"
#include <stdio.h>
list<EdgeDiff> BundleAdjust::edge_diff0;
LayerInfo BundleAdjust::l_info;


int read_mat_binary(FILE * fp, unsigned short &para0, unsigned short &para1, Mat &m)
{
	unsigned short para[4];
	int row, col;

	if (feof(fp))
		return -1;
	if (fread(para, sizeof(para), 1, fp)!=1)
		return -1;
	para0 = para[0];
	para1 = para[1];
	row = para[2];
	col = para[3];
	m.create(row, col, CV_8UC1);
	fread(m.data, m.rows*m.cols, 1, fp);
	return 0;
}

BundleAdjust::BundleAdjust()
{

}

unsigned BundleAdjust::compute_score(Mat_<unsigned char> &diff)
{

}

int BundleAdjust::load_edge_diff0(string filename)
{
	FILE *fp;
	fp = fopen(filename.c_str(), "rb");
	while (1) {
		EdgeDiff ed;
		if (read_mat_binary(fp, ed.y, ed.x, ed.diff) != 0)
			break;
		ed.score = compute_score(ed.diff);
		edge_diff0.push_back(ed);
	}
	edge_diff0.sort(EdgeDiffCmp());
}

int BundleAdjust::load_config(string filename)
{
	FileStorage fs(filename, FileStorage::READ);
	if (fs.isOpened()) {
		fs["clip_left"] >> l_info.clip_l;
		fs["clip_right"] >> l_info.clip_r;
		fs["clip_up"] >> l_info.clip_u;
		fs["clip_down"] >> l_info.clip_d;
		fs["image_path"] >> l_info.img_path;
		fs["img_num_w"] >> l_info.img_num_w;
		fs["img_num_h"] >> l_info.img_num_h;
		fs["right_bound"] >> l_info.right_bound;
		fs["bottom_bound"] >> l_info.bottom_bound;
		fs["scale"] >> l_info.move_scale;
		fs["offset"] >> l_info.offset;
		CV_Assert(l_info.img_num_w == l_info.offset.cols && l_info.img_num_h == l_info.offset.rows);
		return 0;
	}
	return -1;
}