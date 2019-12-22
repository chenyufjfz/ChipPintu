#include "vwextract.h"

VWExtractML::VWExtractML()
{
	current_layer = -1;
}

int VWExtractML::set_train_param(int layer, int d, int clear, int, int, int, int, int, int, float)
{
	current_layer = layer;
	via_diameter = d;
	if (current_layer >= (int)vwf.size()) {
		vwf.resize(current_layer + 1);
		via_mark.resize(current_layer + 1);
	}
	if (clear) {
		vwf[current_layer].clear_feature();
		via_mark[current_layer].release();
	}
	return 0;
}

int VWExtractML::set_extract_param(int layer, int, int, int, int, int, int, int, int, float)
{
	current_layer = layer;
	if (current_layer >= (int)vwf.size()) {
		vwf.resize(current_layer + 1);
		via_mark.resize(current_layer + 1);
	}
	return 0;
}

Mat VWExtractML::get_mark(int layer)
{
	if (layer < via_mark.size())
		return via_mark[layer];
	return Mat();
}

int VWExtractML::train(string img_name, vector<MarkObj> & obj_sets)
{
	string file_name(img_name);
	file_name[file_name.length() - 5] = current_layer + '0';
	Mat img = imread(file_name, 0);
	if (current_layer >= (int)vwf.size()) {
		vwf.resize(current_layer + 1);
		via_mark.resize(current_layer + 1);
	}
	if (via_mark[current_layer].empty()) {
		via_mark[current_layer].create(img.rows, img.cols, CV_8U);
		via_mark[current_layer] = 0;
	}
	for (auto & m : obj_sets) {
		if (m.type == OBJ_POINT) {
			Point range;
			int label = (m.type2 == POINT_NO_VIA) ? 0 : 1;
			vector<Point> vs;
			label |= VIA_FEATURE;
			Point loc = Point(m.p0.x(), m.p0.y());
			bool ret = vwf[current_layer].add_feature(img, loc, loc, range,
				via_diameter, label, &vs);
			if (ret) {
				m.p0 = QPoint(loc.x, loc.y);
				for (auto p : vs)
					via_mark[current_layer].at<uchar>(p) = 255;
			}
		}
	}
	int loc = img_name.find_last_of("\\/");
	string project_path = img_name.substr(0, loc);
	vwf[current_layer].write_file(project_path, current_layer);
	return 0;
}

int VWExtractML::extract(string img_name, QRect rect, vector<MarkObj> & obj_sets)
{
	string file_name(img_name);
	file_name[file_name.length() - 5] = current_layer + '0';
	Mat img = imread(file_name, 0);
	if (current_layer >= (int)vwf.size()) {
		vwf.resize(current_layer + 1);
		via_mark.resize(current_layer + 1);
	}
	
	via_mark[current_layer].create(img.rows, img.cols, CV_8U);
	via_mark[current_layer] = 0;
	
	int loc = img_name.find_last_of("\\/");
	string project_path = img_name.substr(0, loc);
	if (!vwf[current_layer].read_file(project_path, current_layer))
		return;
	vwf[current_layer].via_search(img, via_mark[current_layer], obj_sets);
	return 0;
}