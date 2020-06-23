#include <QtConcurrent>
#include "vwextract.h"
#include <QDir>
#define SAVE_RST_TO_FILE	1
#ifdef Q_OS_WIN
#ifdef QT_DEBUG
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif
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
	int img_pixel_x0, img_pixel_y0; //imgae left-top pixel
	int layer;
	Mat raw_img;
	Mat prob;
	Mat * via_mark_debug; //only for debug
	Mat * edge_mark_debug; //only for debug
	vector<ElementObj *> eo;
	QPolygon * poly;
	int ref_cnt;
	ProcessData() {
		ref_cnt = 0;
		x0 = -10000;
		y0 = -10000;
		img_pixel_x0 = -10000;
		img_pixel_y0 = -10000;
		layer = -1;
		via_mark_debug = NULL;
		edge_mark_debug = NULL;
	}
};

struct ProcessImageData {
	VWfeature * vwf;
	ProcessData * lpd;
	ProcessData * upd;
	ProcessData * cpd;
	bool multi_thread;
};

ProcessImageData process_img(const ProcessImageData & pi)
{
	qInfo("process_img (%d,%d,%d)", pi.cpd->x0, pi.cpd->y0, pi.cpd->layer);

	if (!pi.cpd->raw_img.empty()) {
		Mat empty;
		pi.vwf->via_search(pi.cpd->raw_img, (pi.cpd->via_mark_debug == NULL ? empty : *(pi.cpd->via_mark_debug)), pi.cpd->eo, pi.multi_thread);
		pi.vwf->edge_search(pi.cpd->raw_img, pi.cpd->prob, Mat(), (pi.cpd->edge_mark_debug == NULL ? empty : *(pi.cpd->edge_mark_debug)), pi.multi_thread);
		QPoint tl(pi.cpd->img_pixel_x0, pi.cpd->img_pixel_y0);
		for (auto & o : pi.cpd->eo) {
			o->type3 = pi.cpd->layer;
			o->p0 = o->p0 + tl;
		}
	}
	qInfo("process_img (%d,%d,%d) o=%d", pi.cpd->x0, pi.cpd->y0, pi.cpd->layer, pi.cpd->eo.size());
	return pi;
}

void merge_process_data(ProcessData * pd0, ProcessData * pd1)
{
	qInfo("merge_process_data (%d,%d,%d) o=%d and (%d,%d,%d) o=%d", pd0->x0, pd0->y0, pd0->layer, pd0->eo.size(),
		pd1->x0, pd1->y0, pd1->layer, pd1->eo.size());
	for (int i = 0; i < (int) pd0->eo.size(); i++)
	if (pd0->eo[i]->type == OBJ_POINT) {
		QPoint loc = pd0->eo[i]->p0;
		int layer = pd0->eo[i]->type3;
		for (int j = 0; j < (int)pd1->eo.size(); j++)
		if (pd1->eo[j]->type == OBJ_POINT && pd1->eo[j]->type3 == layer && pd0->eo[i] != pd1->eo[j]) {
			QPoint diff = loc - pd1->eo[j]->p0;
			if (abs(diff.x()) <= 2 && abs(diff.y()) <= 2) { //pd0 and pd1 point to same via, merge
				if (pd0->eo[i]->un.attach >= pd1->eo[j]->un.attach) {
					CV_Assert(pd1->eo[j]->un.attach == 1);
					pd0->eo[i]->un.attach++;
					delete pd1->eo[j];
					pd1->eo[j] = pd0->eo[i];
				}
				else {
					pd1->eo[j]->un.attach++;
					delete pd0->eo[i];
					pd0->eo[i] = pd1->eo[j];
				}
				break;
			}
		}
	}
	else {

	}
}

void get_result(ProcessData * pd, vector<ElementObj *> & objs)
{
	qInfo("get_result (%d,%d,%d) w=%d,h=%d,o=%d", pd->x0, pd->y0, pd->layer,
		pd->raw_img.cols, pd->raw_img.rows, pd->eo.size());
	for (auto & e : pd->eo) 
	if (e->type == OBJ_POINT) {
		if (e->un.attach == 1) {
			if (!pd->poly)
				objs.push_back(e);
			else
			if (pd->poly->containsPoint(e->p0, Qt::OddEvenFill))
				objs.push_back(e);
		}
		else
			e->un.attach--;
	}
}

void merge_img_result(vector<ElementObj *> & objs, const ProcessImageData & t)
{
	if (t.lpd)
		merge_process_data(t.lpd, t.cpd);
	if (t.upd)
		merge_process_data(t.upd, t.cpd);
	if (t.lpd) {
		if (t.lpd->ref_cnt-- == 1)
			get_result(t.lpd, objs);
	}
	if (t.upd) {
		if (t.upd->ref_cnt-- == 1)
			get_result(t.upd, objs);
	}
}

void process_imgs(vector<ProcessImageData> & pis, vector<ElementObj *> & obj_sets, bool parallel) {
	if (parallel) {
		//each thread process all layer on same tile
		vector<ElementObj *> temp_vec;
		temp_vec = QtConcurrent::blockingMappedReduced<vector<ElementObj *>, vector<ProcessImageData> >(pis, process_img, merge_img_result,
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

int VWExtractML::set_extract_param(int layer, int , int, int, int, int, int, int, int, float)
{
	layer_min = layer & 0xff;
	layer_max = layer >> 8 & 0xff;
	layer_max = max(layer_max, layer_min);
	if (layer_max >= (int)vwf.size()) {
		vwf.resize(layer_max + 1);
		via_mark.resize(layer_max + 1);
		edge_mark.resize(layer_max + 1);
	}
	return 0;
}

Mat VWExtractML::get_mark(int layer)
{
	if (layer < via_mark.size())
		return via_mark[layer];
	return Mat();
}

Mat VWExtractML::get_mark1(int layer)
{
	if (layer < edge_mark.size())
		return edge_mark[layer];
	return Mat();
}

int VWExtractML::train(string img_name, vector<MarkObj> & obj_sets)
{
	layer_min = 200, layer_max = 0;
	for (auto & m : obj_sets) {
		layer_min = (layer_min > m.type3) ? m.type3 : layer_min;
		layer_max = (layer_max < m.type3) ? m.type3 : layer_max;
	}
	if (layer_max >= (int)vwf.size()) {
		vwf.resize(layer_max + 1);
		via_mark.resize(layer_max + 1);
		edge_mark.resize(layer_max + 1);
	}
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		string file_name(img_name);
		file_name[file_name.length() - 5] = current_layer + '0';
		Mat img = imread(file_name, 0);
		if (via_mark[current_layer].empty()) {
			via_mark[current_layer].create(img.rows, img.cols, CV_8U);
            via_mark[current_layer] = Scalar::all(0);
		}
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer);
		for (auto & m : obj_sets) 
		if (!del) {
			if (m.type == OBJ_POINT && m.type3 == current_layer) {
				if (m.type2 == POINT_NO_VIA || m.type2 == POINT_NORMAL_VIA0) { // add via or no via
					Point range;
					int label = (m.type2 == POINT_NO_VIA) ? 0 : VIA_IS_VIA;
					vector<Point> vs;
					label |= VIA_FEATURE;
					Point loc = Point(m.p0.x(), m.p0.y());
					bool ret = vwf[current_layer].add_feature(img, loc, loc, range,
						via_diameter_min, via_diameter_max, label, &vs);
					if (ret) { //add success
						m.p0 = QPoint(loc.x, loc.y); //return location
						m.p1 = QPoint(range.x, range.y); //return diameter
						for (auto p : vs)
							via_mark[current_layer].at<uchar>(p) = 255;
					}
					else
						m.p1 = QPoint(0, 0); //add fail
				}
				if (m.type2 == POINT_WIRE_INSU || m.type2 == POINT_WIRE || m.type2 == POINT_INSU) {
					Point range = m.type2 == POINT_WIRE_INSU ? Point(2, 2) : Point(1, 1);
					Point loc = Point(m.p0.x(), m.p0.y());
					int label = EDGE_FEATURE;
					label |= (m.type2 == POINT_WIRE_INSU) ? EDGE_IS_WIRE_INSU :
						(m.type2 == POINT_WIRE) ? EDGE_IS_WIRE : EDGE_IS_INSU;
					bool ret = vwf[current_layer].add_feature(img, loc, loc, range,
						via_diameter_min, via_diameter_max, label, NULL);
					if (ret) { //add success
						m.p0 = QPoint(loc.x, loc.y); //return location
						m.p1 = QPoint(range.x, range.y); //return diameter
					}
					else
						m.p1 = QPoint(0, 0); //add fail
				}
			}
		}		
		else 
			if (m.type == OBJ_POINT && m.type3 == current_layer) { //delete via or no via
				Point loc = Point(m.p0.x(), m.p0.y());
				if (m.type2 == POINT_NO_VIA || m.type2 == POINT_NORMAL_VIA0) // del via or no via
					vwf[current_layer].del_feature(loc, via_diameter_max);
				if (m.type2 == POINT_WIRE_INSU || m.type2 == POINT_WIRE || m.type2 == POINT_INSU)
					vwf[current_layer].del_feature(loc, 2);
			}
		vwf[current_layer].write_file(project_path, current_layer);
	}
	return 0;
}

int VWExtractML::extract(string img_name, QRect rect, vector<MarkObj> & obj_sets)
{
	vector<ProcessImageData> pis;
	ProcessData ed[100];
	obj_sets.clear();
	if (layer_max >= (int)vwf.size()) {
		vwf.resize(layer_max + 1);
		via_mark.resize(layer_max + 1);
		edge_mark.resize(layer_max + 1);
	}
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		string file_name(img_name);
		file_name[file_name.length() - 5] = current_layer + '0';
		Mat img = imread(file_name, 0);
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		if (!vwf[current_layer].read_file(project_path, current_layer))
			return -1;
		ProcessImageData pi;
		ed[current_layer].x0 = 0;
		ed[current_layer].y0 = 0;
		ed[current_layer].img_pixel_x0 = 0;
		ed[current_layer].img_pixel_y0 = 0;
		ed[current_layer].layer = current_layer;
		ed[current_layer].raw_img = img;
		ed[current_layer].poly = NULL;
		via_mark[current_layer] = img.clone();
		cvtColor(img, edge_mark[current_layer], CV_GRAY2BGR);
		ed[current_layer].via_mark_debug = &(via_mark[current_layer]);
		ed[current_layer].edge_mark_debug = &(edge_mark[current_layer]);
		pi.vwf = &vwf[current_layer];
		pi.lpd = NULL;
		pi.upd = NULL;
		pi.cpd = &ed[current_layer];
		pi.multi_thread = false;
		pis.push_back(pi);
	}
	vector<ElementObj *> es;
	process_imgs(pis, es, false);
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++)
		get_result(&ed[current_layer], es);
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
				int label = (m.type2 == POINT_NO_VIA) ? 0 : VIA_IS_VIA;
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

struct SearchAreaPoly {
	QPolygon poly;
	int option;
	SearchAreaPoly(QPolygon p, int o) {
		poly = p;
		option = o;
	}
};
int VWExtractML::extract(vector<ICLayerWrInterface *> & ics, const vector<SearchArea> & area_rect, vector<MarkObj> & obj_sets)
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
	int BORDER_SIZE = 0;
	for (int current_layer = layer_min; current_layer <= layer_max; current_layer++) {
		if (current_layer >= (int)vwf.size())
			vwf.resize(current_layer + 1);
		string img_name = ics[current_layer]->get_file_name();
		int loc = img_name.find_last_of("\\/");
		string project_path = img_name.substr(0, loc);
		vwf[current_layer].read_file(project_path, current_layer);
		BORDER_SIZE = max(BORDER_SIZE, vwf[current_layer].get_max_d());
	}
	BORDER_SIZE += 8;

	vector<SearchAreaPoly> area_;
	QPolygon area_poly;
	int option;
	for (auto & ar : area_rect) {
		if (ar.option & OPT_POLYGON_SEARCH) {
			area_poly.push_back(ar.rect.topLeft());
			area_poly.push_back(QPoint(ar.rect.width(), ar.rect.height()));
			option = ar.option;
		}
		else {
			if (area_poly.size() > 2) {
				area_.push_back(SearchAreaPoly(area_poly, option));
				for (int i = 0; i < area_poly.size(); i += 2)
					qInfo("Poly (%x,%x) (%x,%x)", area_poly[i].x(), area_poly[i].y(), area_poly[i+1].x(), area_poly[i+1].y());
				area_poly.clear();
			}
			area_.push_back(SearchAreaPoly(QPolygon(ar.rect), ar.option));
		}
	}
	if (area_poly.size() > 2) {
		area_.push_back(SearchAreaPoly(area_poly, option));
		for (int i = 0; i < area_poly.size(); i += 2)
			qInfo("Poly (%x,%x) (%x,%x)", area_poly[i].x(), area_poly[i].y(), area_poly[i + 1].x(), area_poly[i + 1].y());
		area_poly.clear();
	}
	int block_x, block_y;
	ics[0]->getBlockNum(block_x, block_y);
	int block_width = ics[0]->getBlockWidth();
	int scale = 32768 / block_width;
	obj_sets.clear();

	for (int area_idx = 0; area_idx < area_.size(); area_idx++) {
		int extend = BORDER_SIZE * scale;
		int parallel_search = area_[area_idx].option & OPT_PARALLEL_SEARCH;
		QRect sr = area_[area_idx].poly.boundingRect().marginsAdded(QMargins(extend, extend, extend, extend));
		QPolygon cur_poly;
		for (int i = 0; i < area_[area_idx].poly.size(); i++)
			cur_poly.push_back(QPoint(area_[area_idx].poly[i].x() / scale, area_[area_idx].poly[i].y() / scale));
		
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
				ed[l].x0 = 0;
				ed[l].y0 = 0;
				ed[l].img_pixel_x0 = sr.left() / scale;
				ed[l].img_pixel_y0 = sr.top() / scale;
				ed[l].layer = l;
				QRect r(sr.left() / scale, sr.top() / scale, sr.width() / scale, sr.height() / scale);
				ed[l].raw_img = prepare_raw_img(ics[l], 0, r);
				pi.vwf = &vwf[l];
				pi.lpd = NULL;
				pi.upd = NULL;
				pi.cpd = &ed[l];
				pi.multi_thread = false;
				pis.push_back(pi);
			}
			vector<ElementObj *> es;
			process_imgs(pis, es, false);
			for (int l = layer_min; l <= layer_max; l++)
				get_result(&ed[l], es);
			convert_element_obj(es, obj_sets, scale);
		}
		else {
			//following same as vwextract2
			QRect sb(QPoint((sr.left() + 0x4000) >> 15, (sr.top() + 0x4000) >> 15),
				QPoint((sr.right() - 0x4000) >> 15, (sr.bottom() - 0x4000) >> 15));
			CV_Assert(sb.right() >= sb.left() && sb.bottom() >= sb.top());
			vector<vector<ProcessData> > diag_line[2];
			for (int i = 0; i < 2; i++) {
				diag_line[i].resize(layer_max + 1);
				for (int j = 0; j < (int)diag_line[i].size(); j++)
					diag_line[i][j].resize(max(sb.width() + 1, sb.height() + 1));
			}
			int lx = sr.left() - (sb.left() << 15); //do 4dec/5inc
			int rx = sr.right() - ((sb.right() + 1) << 15);
			int ty = sr.top() - (sb.top() << 15);
			int by = sr.bottom() - ((sb.bottom() + 1) << 15);
			int cl = 1;
			int cl_num;
			QPoint sr_tl_pixel = sr.topLeft() / scale;
			qInfo("extract Rect, lt=(%d,%d), rb=(%d,%d), multithread=%d", sr_tl_pixel.x(), sr_tl_pixel.y(), sr.right() / scale, sr.bottom() / scale, parallel_search);
			for (int xay = sb.left() + sb.top(); xay <= sb.right() + sb.bottom(); xay++) {
				/*Scan from top left to bottom right. One loop process one xie line /,
				For xie line /, process each ProcessData concurrenty*/
				cl = 1 - cl;
				cl_num = 0;
				for (int i = 0; i < (int) diag_line[cl].size(); i++) //release current diag_line memory
				for (int j = 0; j < (int)diag_line[cl][i].size(); j++) {
					CV_Assert(diag_line[cl][i][j].ref_cnt == 0);
					diag_line[cl][i][j].eo.clear();
				}
				//1 load image to diag_line
				for (int x0 = sb.left(); x0 <= sb.right(); x0++) {
					int y0 = xay - x0;
					if (sb.contains(x0, y0)) { //load image per Tile
						bool in_area = true;
						bool need_check_inpoly = false;
						cl_num++;
						QPoint p0(x0 << 15, y0 << 15);
						QPoint p1((x0 << 15) + 0x4000, (y0 << 15) + 0x4000);
						QPoint p2((x0 << 15) + 0x8000 + BORDER_SIZE * scale, (y0 << 15) + 0x8000 + BORDER_SIZE * scale);
						bool in0 = area_[area_idx].poly.containsPoint(p0, Qt::OddEvenFill);
						bool in1 = area_[area_idx].poly.containsPoint(p1, Qt::OddEvenFill);
						bool in2 = area_[area_idx].poly.containsPoint(p2, Qt::OddEvenFill);
						if (!in0 && !in1 &&	!in2 && !(x0 == sb.left() && y0 == sb.top()))
							in_area = false;
						if (!in0 || !in1 || !in2)
							need_check_inpoly = true;
						//1.1 compute load image source and destination
						int wide[3] = { 0 }, height[3] = { 0 }; //wide & height is image destination bound

						if (x0 == sb.left()) {
							if (lx < 0) {				//if left edge locates at less than half image picture, 
								wide[0] = -lx / scale;	//load one more image
								wide[1] = block_width;
							}
							else
								wide[1] = block_width - lx / scale;
						}
						if (x0 == sb.right()) {
							if (rx > 0) {				//if right edge locates at less than half image picture,  
								wide[1] = (wide[1] == 0) ? block_width : wide[1]; //load one more image
								wide[2] = rx / scale;
							}
							else
								wide[1] = block_width + rx / scale;
						}
						if (x0 != sb.left() && x0 != sb.right())
							wide[1] = block_width;

						if (y0 == sb.top()) {
							if (ty < 0) {				//if top edge locates at less than half image picture, 
								height[0] = -ty / scale;	//load one more image
								height[1] = block_width;
							}
							else
								height[1] = block_width - ty / scale;
						}
						if (y0 == sb.bottom()) {
							if (by > 0) {				//if bottom edge locates at less than half image picture, 
								height[1] = (height[1] == 0) ? block_width : height[1];	//load one more image
								height[2] = by / scale;
							}
							else
								height[1] = block_width + by / scale;
						}
						if (y0 != sb.top() && y0 != sb.bottom())
							height[1] = block_width;
						int ewide = (x0 == sb.left()) ? 0 : BORDER_SIZE * 2;
						int ehight = (y0 == sb.top()) ? 0 : BORDER_SIZE * 2;
						for (int l = layer_min; l <= layer_max; l++) {
							Mat img(height[0] + height[1] + height[2] + ehight, wide[0] + wide[1] + wide[2] + ewide, CV_8U);
							ProcessData * d = &diag_line[cl][l][cl_num - 1];
							d->layer = l;
							d->x0 = x0;
							d->y0 = y0;
							d->poly = need_check_inpoly ? &cur_poly : NULL;
							if (ewide == 0 && ehight == 0) {
								d->img_pixel_x0 = sr_tl_pixel.x();
								d->img_pixel_y0 = sr_tl_pixel.y();								
							}
							if (ewide != 0) { // copy left image from old diag_line
								Mat * s_img;
								if (diag_line[1 - cl][l][cl_num - 1].y0 == y0) {
									s_img = &diag_line[1 - cl][l][cl_num - 1].raw_img;
									d->img_pixel_x0 = diag_line[1 - cl][l][cl_num - 1].img_pixel_x0 + s_img->cols
										- BORDER_SIZE * 2;
									d->img_pixel_y0 = diag_line[1 - cl][l][cl_num - 1].img_pixel_y0;
								}
								else {
									CV_Assert(diag_line[1 - cl][l][cl_num - 2].y0 == y0);
									s_img = &diag_line[1 - cl][l][cl_num - 2].raw_img;
									d->img_pixel_x0 = diag_line[1 - cl][l][cl_num - 2].img_pixel_x0 + s_img->cols
										- BORDER_SIZE * 2;
									d->img_pixel_y0 = diag_line[1 - cl][l][cl_num - 2].img_pixel_y0;
								}
								if (in_area) {
									if (!s_img->empty()) {
										CV_Assert(s_img->rows == img.rows);
										(*s_img)(Rect(s_img->cols - BORDER_SIZE * 2, 0, BORDER_SIZE * 2, img.rows)).copyTo(img(Rect(0, 0, BORDER_SIZE * 2, img.rows)));
									}
									else
										img(Rect(0, 0, BORDER_SIZE * 2, img.rows)) = Scalar::all(0);
								}
							}

							if (ehight != 0) {  // copy upper image from old diag_line
								Mat * s_img;
								if (diag_line[1 - cl][l][cl_num - 1].x0 == x0) {
									s_img = &diag_line[1 - cl][l][cl_num - 1].raw_img;
									if (ewide != 0)
										CV_Assert(d->img_pixel_x0 == diag_line[1 - cl][l][cl_num - 1].img_pixel_x0 &&
										d->img_pixel_y0 == diag_line[1 - cl][l][cl_num - 1].img_pixel_y0 + s_img->rows
										- BORDER_SIZE * 2);
									d->img_pixel_x0 = diag_line[1 - cl][l][cl_num - 1].img_pixel_x0;
									d->img_pixel_y0 = diag_line[1 - cl][l][cl_num - 1].img_pixel_y0 + s_img->rows
										- BORDER_SIZE * 2;
								}
								else {
									CV_Assert(diag_line[1 - cl][l][cl_num].x0 == x0);
									s_img = &diag_line[1 - cl][l][cl_num].raw_img;
									if (ewide != 0)
										CV_Assert(d->img_pixel_x0 == diag_line[1 - cl][l][cl_num].img_pixel_x0 &&
										d->img_pixel_y0 == diag_line[1 - cl][l][cl_num].img_pixel_y0 + s_img->rows
										- BORDER_SIZE * 2);
									d->img_pixel_x0 = diag_line[1 - cl][l][cl_num].img_pixel_x0;
									d->img_pixel_y0 = diag_line[1 - cl][l][cl_num].img_pixel_y0 + s_img->rows
										- BORDER_SIZE * 2;
								}
								if (in_area) {
									if (!s_img->empty()) {
										CV_Assert(s_img->cols == img.cols);
										(*s_img)(Rect(0, s_img->rows - BORDER_SIZE * 2, img.cols, BORDER_SIZE * 2)).copyTo(img(Rect(0, 0, img.cols, BORDER_SIZE * 2)));
									}
									else
										img(Rect(0, 0, img.cols, BORDER_SIZE * 2)) = Scalar::all(0);
								}
							}
							d->raw_img.release();
							//When extract width(height) > 2 * image width
							//if loading image is not at the edge, load 1*1 image; if at the edge, load 1*1 or 1*2;
							//if at the corner, load 1*1 or 1*2, or 2*2.
							//When extract width(height) < 2 * image width
							//if loading image is at the edge, load 1*1, 1*2 or 1*3 image, if at the corner, load 1*1 or 1*2 or 2*2 or 2*3 or 3*3 
							if (in_area)
							for (int y = y0 - 1, img_y = ehight; y <= y0 + 1; y++) {
								int dy = y - y0;
								for (int x = x0 - 1, img_x = ewide; x <= x0 + 1; x++) {
									int dx = x - x0;
									if (wide[dx + 1] != 0 && height[dy + 1] != 0) {
										vector<uchar> encode_img;
										if (ics[l]->getRawImgByIdx(encode_img, x, y, 0, 0, false) != 0) {
											qCritical("load image error at l=%d, (%d,%d)", l, x, y);
											return -1;
										}
										Mat image = imdecode(Mat(encode_img), 0);
										if (image.rows != image.cols) {
											qCritical("load image rows!=cols at l=%d, (%d,%d)", l, x, y);
											return -1;
										}
										QRect s(0, 0, image.cols, image.rows);
										if (dx == -1)
											s.setLeft(image.cols - wide[0]);
										if (dx == 1)
											s.setRight(wide[2] - 1);
										if (dx == 0) {
											if (wide[0] == 0 && x0 == sb.left())
												s.setLeft(image.cols - wide[1]);
											if (wide[2] == 0 && x0 == sb.right())
												s.setRight(wide[1] - 1);
										}
										if (dy == -1)
											s.setTop(image.rows - height[0]);
										if (dy == 1)
											s.setBottom(height[2] - 1);
										if (dy == 0) {
											if (height[0] == 0 && y0 == sb.top())
												s.setTop(image.rows - height[1]);
											if (height[2] == 0 && y0 == sb.bottom())
												s.setBottom(height[1] - 1);
										}
										CV_Assert(s.width() == wide[dx + 1] && s.height() == height[dy + 1]);
										if (l == 0)
											qDebug("load image%d_%d, (x=%d,y=%d),(w=%d,h=%d) to (x=%d,y=%d)", y, x, s.left(), s.top(), s.width(), s.height(), img_x, img_y);
										image(Rect(s.left(), s.top(), s.width(), s.height())).copyTo(img(Rect(img_x, img_y, wide[dx + 1], height[dy + 1])));
									}
									img_x += wide[dx + 1];
								}
								img_y += height[dy + 1];
							}
							d->raw_img = img;
						}
					}
				}
				//2 now diag_line is loaded, process it
				vector<ProcessImageData> pis;
				for (int i = 0; i < cl_num; i++) 
				for (int l = layer_min; l <= layer_max; l++) {
					ProcessImageData pi;
					pi.vwf = &vwf[l];
					pi.cpd = &diag_line[cl][l][i];
					pi.lpd = &diag_line[1 - cl][l][i];
					pi.upd = &diag_line[1 - cl][l][i];
					pi.multi_thread = (pi.cpd->x0 == sb.left() && pi.cpd->y0 == sb.top()) ? false : parallel_search;
					if (pi.cpd->y0 > sb.top() && pi.cpd->x0 != pi.upd->x0)
						pi.upd = &diag_line[1 - cl][l][i + 1];
					if (pi.cpd->x0 > sb.left() && pi.cpd->y0 != pi.lpd->y0)
						pi.lpd = &diag_line[1 - cl][l][i - 1];
					if (pi.cpd->x0 > sb.left()) {
						CV_Assert(pi.cpd->y0 == pi.lpd->y0 && pi.cpd->layer == pi.lpd->layer);
						pi.lpd->ref_cnt++;
					}
					else
						pi.lpd = NULL;
					if (pi.cpd->y0 > sb.top()) {
						CV_Assert(pi.cpd->x0 == pi.upd->x0 && pi.cpd->layer == pi.upd->layer);
						pi.upd->ref_cnt++;
					}
					else
						pi.upd = NULL;
					pis.push_back(pi);
				}
				vector<ElementObj *> es;
				process_imgs(pis, es, parallel_search);
				convert_element_obj(es, obj_sets, scale);
			}
			vector<ElementObj *> es;
			for (int l = layer_min; l <= layer_max; l++)
				get_result(&diag_line[cl][l][0], es);
			convert_element_obj(es, obj_sets, scale);
		}
	}
#if SAVE_RST_TO_FILE
	save_rst_to_file(obj_sets, scale);
#endif
	qInfo("VWExtractML Extract finished successfully");
	qDebug("*#*#DumpMessage#*#*");
	return 0;
}
