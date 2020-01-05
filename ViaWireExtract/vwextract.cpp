#include <QtConcurrent>
#include "vwextract.h"
#include <QDir>
#define SAVE_RST_TO_FILE	1
/*
input ic_layer
input scale, 0,1,2,3...
input rect, non-scale pixel unit
Return raw image
*/
Mat prepare_raw_img(ICLayerWrInterface * ic_layer, int scale, QRect rect)
{
	Mat img((rect.height() >> scale), (rect.width() >> scale), CV_8UC1);
	int xl = (rect.left() / ic_layer->getBlockWidth() >> scale) << scale; //xl is picture idx x
	int yt = (rect.top() / ic_layer->getBlockWidth() >> scale) << scale;
	int xr = (rect.right() / ic_layer->getBlockWidth() >> scale) << scale;
	int yb = (rect.bottom() / ic_layer->getBlockWidth() >> scale) << scale;
	bool right_hit = false, bottom_hit = false;
	for (int y = yt; y <= yb; y += 1 << scale) //both y & x is picture idx
	for (int x = xl; x <= xr; x += 1 << scale) {		
		vector<uchar> encode_img;
		qDebug("load img from database scale=%d, x=%d,y=%d", scale, x, y);
		if (ic_layer->getRawImgByIdx(encode_img, x, y, scale, 0, true) != 0) {
			qCritical("load image error at s=%d, (%d,%d)", scale, x, y);
			return Mat();
		}
		Mat raw_img = imdecode(Mat(encode_img), 0);
		
		//now raw_img is loaded, copy it to img;
		QRect raw_img_rect(x *ic_layer->getBlockWidth(), y *ic_layer->getBlockWidth(), ic_layer->getBlockWidth() << scale, ic_layer->getBlockWidth() << scale);
		QRect overlap_rect = raw_img_rect & rect; //overlap rect is the copy rect, unit is pixel
		QRect src = overlap_rect.translated(-raw_img_rect.topLeft());
		QRect tgt = overlap_rect.translated(-rect.topLeft());
		CV_Assert((tgt.left() >> scale) + (tgt.width() >> scale) <= img.cols &&
			(tgt.top() >> scale) + (tgt.height() >> scale) <= img.rows);
		CV_Assert((src.left() >> scale) + (src.width() >> scale) <= raw_img.cols &&
			(src.top() >> scale) + (src.height() >> scale) <= raw_img.rows);
		if ((tgt.left() >> scale) + (tgt.width() >> scale) == img.cols)
			right_hit = true;
		if ((tgt.top() >> scale) + (tgt.height() >> scale) == img.rows)
			bottom_hit = true;
		if ((src.width() >> scale) == 0 || (src.height() >> scale) == 0)
			continue;
		CV_Assert(raw_img.type() == img.type());
		raw_img(Rect(src.left() >> scale, src.top() >> scale, src.width() >> scale, src.height() >> scale)).copyTo(
			img(Rect(tgt.left() >> scale, tgt.top() >> scale, tgt.width() >> scale, tgt.height() >> scale)));
	}
	CV_Assert(right_hit && bottom_hit);
	return img;
}

struct ProcessData {
	int x0, y0;
	int layer;
	vector<ElementObj> eo;
};

struct ProcessImageData {
	Mat img;
	QPoint tl;
	VWfeature * vwf;
	ProcessData * lpd;
	ProcessData * upd;
	ProcessData * cpd;
	bool multi_thread;	
};

ProcessImageData process_img(const ProcessImageData & pi)
{
	qInfo("process x=%d, y=%d, l=%d", pi.cpd->x0, pi.cpd->y0, pi.cpd->layer);
	pi.vwf->via_search(pi.img, Mat(), pi.cpd->eo, pi.multi_thread);
	for (auto & o : pi.cpd->eo) {
		o.type3 = pi.cpd->layer;
		o.p0 = o.p0 + pi.tl;
	}
	return pi;
}

void merge_img_result(vector<ElementObj> & objs, const ProcessImageData & t)
{
	objs.insert(objs.end(), t.cpd->eo.begin(), t.cpd->eo.end());
}

void process_imgs(vector<ProcessImageData> & pis, vector<ElementObj> & obj_sets, bool parallel) {
	if (parallel) {
		//each thread process all layer on same tile
		vector<ElementObj> temp_vec;
		temp_vec = QtConcurrent::blockingMappedReduced<vector<ElementObj>, vector<ProcessImageData> >(pis, process_img, merge_img_result,
			QtConcurrent::OrderedReduce | QtConcurrent::SequentialReduce);
		obj_sets.insert(obj_sets.end(), temp_vec.begin(), temp_vec.end());
	}
	else
	for (int i = 0; i < pis.size(); i++) {
		process_img(pis[i]);
		//3 output result to obj_sets
		merge_img_result(obj_sets, pis[i]);
	}
}

VWExtractML::VWExtractML()
{
	layer_min = -1;
	layer_max = -1;
}

/*    31..24 23..16 15..8 7..0
type
d            clear dmax  dmin
clear
*/
int VWExtractML::set_train_param(int type, int d, int, int, int, int, int, int, int, float)
{
	if ((type & 0xff) == OBJ_POINT) {
		via_diameter_min = d & 0xff;
		via_diameter_max = d >> 8 & 0xff;
		del = d >> 16 & 0xff;
	}
	return 0;
}

int VWExtractML::set_extract_param(int layer, int, int, int, int, int, int, int, int, float)
{
	layer_min = layer & 0xff;
	layer_max = layer >> 8 & 0xff;
	if (layer_max >= (int)vwf.size()) {
		vwf.resize(layer_max + 1);
		via_mark.resize(layer_max + 1);
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
	layer_min = 200, layer_max = 0;
	for (auto & m : obj_sets) {
		layer_min = (layer_min > m.type3) ? m.type3 : layer_min;
		layer_max = (layer_max < m.type3) ? m.type3 : layer_max;
	}
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
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
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer);	
		for (auto & m : obj_sets) 
		if (!del) {
			if (m.type == OBJ_POINT && m.type3 == current_layer) {
				Point range;
				int label = (m.type2 == POINT_NO_VIA) ? 0 : 1;
				vector<Point> vs;
				label |= VIA_FEATURE;
				Point loc = Point(m.p0.x(), m.p0.y());
				bool ret = vwf[current_layer].add_feature(img, loc, loc, range,
					via_diameter_min, via_diameter_max, label, &vs);
				if (ret) {
					m.p0 = QPoint(loc.x, loc.y);
					m.p1 = QPoint(range.x, range.y);
					for (auto p : vs)
						via_mark[current_layer].at<uchar>(p) = 255;
				}
			}
		}		
		else 
			if (m.type == OBJ_POINT && m.type3 == current_layer) {
				Point loc = Point(m.p0.x(), m.p0.y());
				vwf[current_layer].del_feature(loc, via_diameter_max);
			}
		vwf[current_layer].write_file(project_path, current_layer);
	}
	return 0;
}

int VWExtractML::extract(string img_name, QRect rect, vector<MarkObj> & obj_sets)
{
	vector<ProcessImageData> pis;
	ProcessData ed[100];
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
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
			return -1;
		ProcessImageData pi;
		ed[current_layer].x0 = 0;
		ed[current_layer].y0 = 0;
		ed[current_layer].layer = current_layer;
		pi.img = img;
		pi.vwf = &vwf[current_layer];
		pi.lpd = NULL;
		pi.upd = NULL;
		pi.cpd = &ed[current_layer];
		pi.multi_thread = false;
		pis.push_back(pi);
	}
	vector<ElementObj> es;
	process_imgs(pis, es, false);
	convert_element_obj(es, obj_sets, 1);
	return 0;
}

#define PREPARE_IMG_RADIUS 60

int VWExtractML::train(vector<ICLayerWrInterface *> & ics, vector<MarkObj> & obj_sets)
{
	layer_min = 200, layer_max = 0;
	for (auto & m : obj_sets) {
		layer_min = (layer_min > m.type3) ? m.type3 : layer_min;
		layer_max = (layer_max < m.type3) ? m.type3 : layer_max;
	}
	int block_width = ics[0]->getBlockWidth();
	int scale = 32768 / block_width;
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		if (current_layer >= (int)vwf.size())
			vwf.resize(current_layer + 1);
		string img_name = ics[current_layer]->get_file_name();
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		
		vwf[current_layer].read_file(project_path, current_layer);
		for (auto & m : obj_sets) 
		if (!del) {				
			if (m.type == OBJ_POINT && m.type3 == current_layer) {
				Point range;
				int label = (m.type2 == POINT_NO_VIA) ? 0 : 1;
				label |= VIA_FEATURE;
				Point c = Point(m.p0.x() / scale, m.p0.y() / scale);
				Point loc(PREPARE_IMG_RADIUS, PREPARE_IMG_RADIUS);
				Mat img = prepare_raw_img(ics[current_layer], 0,
					QRect((c - loc).x, (c - loc).y, PREPARE_IMG_RADIUS * 2 + 1, PREPARE_IMG_RADIUS * 2 + 1));
				bool ret = vwf[current_layer].add_feature(img, loc, c, range,
					via_diameter_min, via_diameter_max, label, NULL);
				if (ret) {
					m.p0 = QPoint(c.x * scale, c.y * scale);
					m.p1 = QPoint(range.x, range.y);
				}
				else
					m.p1 = QPoint(0, 0);
			}
		}
		else {
			if (m.type == OBJ_POINT && m.type3 == current_layer) {
				Point c = Point(m.p0.x() / scale, m.p0.y() / scale);
				vwf[current_layer].del_feature(c, via_diameter_max);
			}
		}
		if (del)
			obj_sets.clear();
		vwf[current_layer].write_file(project_path, current_layer);			
	}
	return 0;
}

int VWExtractML::extract(vector<ICLayerWrInterface *> & ics, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets)
{
	QDir *qdir = new QDir;
	deldir("./DImg");
	bool exist = qdir->exists("./DImg");
	if (!exist) {
		bool ok = qdir->mkdir("./DImg");
		if (!ok)
			qCritical("mkdir failed");
	}
	delete qdir;

	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		if (current_layer >= (int)vwf.size())
			vwf.resize(current_layer + 1);
		string img_name = ics[current_layer]->get_file_name();
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer);
	}

	int block_x, block_y;
	ics[0]->getBlockNum(block_x, block_y);
	int block_width = ics[0]->getBlockWidth();
	int scale = 32768 / block_width;
	obj_sets.clear();

	for (int area_idx = 0; area_idx < area_.size(); area_idx++) {
		int extend = 18 * scale;

		QRect sr = area_[area_idx].rect.marginsAdded(QMargins(extend, extend, extend, extend));
		sr &= QRect(0, 0, block_x << 15, block_y << 15);
		if (sr.width() <= 0x10000 || sr.height() <= 0x10000) {
			vector<ProcessImageData> pis;
			ProcessData ed[100];
			for (int l = layer_min; l <= layer_max; l++) {
				if (!vwf[l].via_valid(false)) {
					qWarning("skip layer %d, its feature is invalid", l);
					continue;
				}
				ProcessImageData pi;
				ed[l].x0 = sr.left() >> 15;
				ed[l].y0 = sr.top() >> 15;
				ed[l].layer = l;
				QRect r(sr.left() / scale, sr.top() / scale, sr.width() / scale, sr.height() / scale);
				pi.img = prepare_raw_img(ics[l], 0, r);
				pi.vwf = &vwf[l];
				pi.lpd = NULL;
				pi.upd = NULL;
				pi.cpd = &ed[l];
				pi.multi_thread = false;
				pi.tl = r.topLeft();
				pis.push_back(pi);
			}
			vector<ElementObj> es;
			process_imgs(pis, es, false);
#if SAVE_RST_TO_FILE
			save_rst_to_file(es);
#endif
			convert_element_obj(es, obj_sets, scale);
		}
	}
	qInfo("VWExtractML Extract finished successfully");
	qDebug("*#*#DumpMessage#*#*");
	return 0;
}