#include "renderimage.h"
#include <QMutexLocker>
#include <QThread>
#include <algorithm>
#include <QColor>
#include <QtConcurrent>

#define PARALLEL 1

void write(FileStorage& fs, const std::string&, const MapXY & x)
{
	x.write_file(fs);
}

void read(const FileNode& node, MapXY& x, const MapXY& default_value)
{
	if (node.empty())
		x = default_value;
	else
		x.read_file(node);
}

Point TOPOINT(QPoint q) {
	return Point(q.x(), q.y());
}

QPoint TOQPOINT(Point p) {
	return QPoint(p.x, p.y);
}

QLineF clipline(QLineF line, QRectF box)
{
	QRectF lineBox(line.p1(), line.p2());
	if (!box.intersects(lineBox)) //not intersect
		return QLineF(QPointF(-1000000000000, -1000000000000), QPointF(-1000000000000, -1000000000000));

	bool b0 = box.contains(line.p1());
	bool b1 = box.contains(line.p2());
	if (b0 && b1)
		return line;
	else
	{
		QLineF line1(box.topLeft(), box.topRight());
		QLineF line2(box.topRight(), box.bottomRight());
		QLineF line3(box.bottomLeft(), box.bottomRight());
		QLineF line4(box.topLeft(), box.bottomLeft());

		QPointF pt1, pt2, pt3, pt4;
		bool bi1 = (QLineF::BoundedIntersection == line.intersect(line1, &pt1));
		bool bi2 = (QLineF::BoundedIntersection == line.intersect(line2, &pt2));
		bool bi3 = (QLineF::BoundedIntersection == line.intersect(line3, &pt3));
		bool bi4 = (QLineF::BoundedIntersection == line.intersect(line4, &pt4));

		if (b0)
		{
			if (bi1) return QLineF(line.p1(), pt1);
			if (bi2) return QLineF(line.p1(), pt2);
			if (bi3) return QLineF(line.p1(), pt3);
			if (bi4) return QLineF(line.p1(), pt4);
		}
		else if (b1)
		{
			if (bi1) return QLineF(line.p2(), pt1);
			if (bi2) return QLineF(line.p2(), pt2);
			if (bi3) return QLineF(line.p2(), pt3);
			if (bi4) return QLineF(line.p2(), pt4);
		}
		else
		{
			vector<QPointF> pts;
			if (bi1) pts.push_back(pt1);
			if (bi2) pts.push_back(pt2);
			if (bi3) pts.push_back(pt3);
			if (bi4) pts.push_back(pt4);

			if (pts.size() >= 2)
				return QLineF(pts.front(), pts.back());
			else
				return QLineF(QPointF(-1000000000000, -1000000000000), QPointF(-1000000000000, -1000000000000));
		}
	}
}

QImage mat2qimage(const Mat & mat)
{
	// 8-bits unsigned, NO. OF CHANNELS = 1
	if (mat.type() == CV_8UC1)
	{
		QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
		// Set the color table (used to translate colour indexes to qRgb values)
		image.setColorCount(256);
		for (int i = 0; i < 256; i++)
		{
			image.setColor(i, qRgb(i, i, i));
		}
		// Copy input Mat
		uchar *pSrc = mat.data;
		for (int row = 0; row < mat.rows; row++)
		{
			uchar *pDest = image.scanLine(row);
			memcpy(pDest, pSrc, mat.cols);
			pSrc += mat.step;
		}
		return image;
	}
	// 8-bits unsigned, NO. OF CHANNELS = 3
	else if (mat.type() == CV_8UC3)
	{
		// Copy input Mat
		const uchar *pSrc = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat
		QImage image(pSrc, mat.cols, mat.rows, (int) mat.step, QImage::Format_RGB888);
		return image.rgbSwapped();
	}
	else if (mat.type() == CV_8UC4)
	{
		qDebug() << "CV_8UC4";
		// Copy input Mat
		const uchar *pSrc = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat
		QImage image(pSrc, mat.cols, mat.rows, (int) mat.step, QImage::Format_ARGB32);
		return image.copy();
	}
	else
	{
		qDebug() << "ERROR: Mat could not be converted to QImage.";
		return QImage();
	}
}

class PremapTailorImg {
public:
	int draw_order;
	Rect_<double>  rect; //it is in pixel
	Point2d lt, rb;
	Mat m;
	PremapTailorImg() {}
	PremapTailorImg(const Mat _m, Rect_<double> & _r, unsigned _draw_order = 0) {
		lt = Point2d(_r.x + 0.5, _r.y + 0.5);
		rb = Point2d(_r.x + _r.width - 0.5, _r.y + _r.height - 0.5);
		m = _m;
		rect = _r;
		draw_order = _draw_order;
	}
};

bool compare_premap_img(const PremapTailorImg & l, const PremapTailorImg & r)
{
	return l.draw_order > r.draw_order;
}

/*
Input cpara
Input p, pixel unit
Input wh, src picture size
Input method, 0, left-up point compare. 1, right-bottom point compare
Return src map x, y
*/
Point find_src_map(const ConfigPara & cpara, const Point & p, const Size & wh, int method)
{
	int start_x = min(p.x / wh.width, cpara.img_num_w - 1);
	start_x = max(start_x, 0);
	int end_x = min(cpara.img_num_w - (cpara.right_bound() - p.x) / wh.width + 2, cpara.img_num_w);
	end_x = max(end_x, 1);
	int start_y = min(p.y / wh.height, cpara.img_num_h - 1);
	start_y = max(start_y, 0);
	int end_y = min(cpara.img_num_h - (cpara.bottom_bound() - p.y) / wh.height + 2, cpara.img_num_h);
	end_y = max(end_y, 1);

	for (int y = start_y; y < end_y; y++)
	for (int x = start_x; x < end_x; x++) {
		int yy = cpara.offset(y, x)[0];
		int xx = cpara.offset(y, x)[1];
		if (method == 0 &&
			(yy <= p.y &&
			(y + 1 == cpara.img_num_h || cpara.offset(y + 1, x)[0] > p.y)) &&
			(xx <= p.x &&
			(x + 1 == cpara.img_num_w || cpara.offset(y, x + 1)[1] > p.x)))
			return Point(x, y);
		if (method != 0 &&
			(yy + wh.height > p.y &&
			(y == 0 || yy <= p.y)) &&
			(xx + wh.width  > p.x &&
			(x == 0 || xx <= p.x)))
			return Point(x, y);
	}

	if (method != 0)
		return Point(100000000, 100000000);
	else
		return Point(-100000000, -100000000);
}

struct MapRequest {
	MapID id;
	int dst_w;
	Size src;
	const MapXY * pmapxy;
	const ConfigPara * cpara; //this is share with other thread_map_image
	PreMapCache * premap_cache; //this is share with other thread_map_image
	const vector<MapID> * draw_order;
	int load_flag;
	QImage img;
};

//It is running in global thread-pool
void thread_map_image(MapRequest & pr)
{
	int dst_w = pr.dst_w;
	int l = MAPID_L(pr.id);
	const MapXY * pxy = pr.pmapxy;

	//0 Following compute 4 dst vertex and src point
	Point lt, rb;
	Point2d src_lt, src_rb, src_lb, src_rt;

	Point dst_lt(dst_w * MAPID_X(pr.id), dst_w * MAPID_Y(pr.id));
	Point dst_rb(dst_lt.x + dst_w - 1, dst_lt.y + dst_w - 1);
	Point dst_lb(dst_lt.x, dst_rb.y);
	Point dst_rt(dst_rb.x, dst_lt.y);
	bool load_unmap_image = pxy->is_original();

	if (load_unmap_image) {
		src_lt = dst_lt; //src is same as dst for LOAD_RAW_IMAGE
		src_rb = dst_rb;
		src_lb = dst_lb;
		src_rt = dst_rt;
	} 
	else {
		src_lt = pxy->dst2src(dst_lt); //dst_lt map to src_lt, src_lt is not src left top.
		src_rb = pxy->dst2src(dst_rb);
		src_lb = pxy->dst2src(dst_lb);
		src_rt = pxy->dst2src(dst_rt);
	}

	double minx = min(min(src_lt.x, src_rb.x), min(src_lb.x, src_rt.x));
	double miny = min(min(src_lt.y, src_rb.y), min(src_lb.y, src_rt.y));
	double maxx = max(max(src_lt.x, src_rb.x), max(src_lb.x, src_rt.x));
	double maxy = max(max(src_lt.y, src_rb.y), max(src_lb.y, src_rt.y));
	//now Point(minx,miny) is src left top, Point (max, maxy) is src right bottom

	lt = find_src_map(*(pr.cpara), Point(minx, miny), pr.src, 1);
	rb = find_src_map(*(pr.cpara), Point(maxx + 1, maxy + 1), pr.src, 0);

	vector<PremapTailorImg> src_map;
	PreMapCache * pmc = pr.premap_cache;

	//1 load all needed premap into cache
	for (int y = lt.y; y <= rb.y; y++)
	for (int x = lt.x; x <= rb.x; x++) {
		MapID id = MAPID(l, x, y);
		PremapImg * pmap;
		int ret = pmc->find_reserve(id, &pmap);
		if (ret == NOT_FETCH) { //load from disk
			char file_name[200];
			sprintf(file_name, "%s%d_%d.jpg", pr.cpara->img_path.c_str(), y + 1, x + 1);
			qDebug("loadImage, %s", file_name);
			Mat raw_img = imread(file_name, pr.load_flag);
			//pmc->insert(id, raw_img, Point(pr.cpara->offset(y, x)[1], pr.cpara->offset(y, x)[0]), pr.src);
			pmc->insert(id, raw_img);
		}
		if (ret == ALREADY_EXIST)
			pmc->repush(id);
		//if ret==ALREADY_FETCH, other thread load it
	}
	//wait all premap loaded
	for (int y = lt.y; y <= rb.y; y++)
	for (int x = lt.x; x <= rb.x; x++) {
		MapID id = MAPID(l, x, y);
		PremapImg * pmap;
		while (1) {
			int ret = pmc->find_reserve(id, &pmap);
			if (ret == ALREADY_EXIST) {
				int order = y * 16 + x;
				for (int i = 0; i < (int)pr.draw_order->size(); i++)
				if ((*pr.draw_order)[i] == id)
					order += (i + 1) * 0x10000;				
				src_map.push_back(PremapTailorImg(pmap->m(Rect(pr.cpara->clip_l, pr.cpara->clip_u, pr.src.width, pr.src.height)), 
					Rect_<double>(pr.cpara->offset(y, x)[1] - 0.5, pr.cpara->offset(y, x)[0] - 0.5, pr.src.width, pr.src.height), order));				
				break;
			}
			CV_Assert(ret != NOT_FETCH);
			QThread::currentThread()->usleep(1000); //ret==ALREADY_FETCH, wait
		}
	}
	//now all premap is loaded
	sort(src_map.begin(), src_map.end(), compare_premap_img);

	QImage img(dst_w, dst_w, QImage::Format_Indexed8);
	// Set the color table (used to translate colour indexes to qRgb values)
	img.setColorCount(256);
	for (int i = 0; i < 256; i++)
		img.setColor(i, qRgb(i, i, i));
	
	if (src_map.empty()) {
		img.fill(QColor(0, 0, 0));
		pr.img = img;
	}

	//2 do remap for each dst point
	if (load_unmap_image) {
		for (int dy = 0; dy < dst_w; dy++) {
			uchar *pd = img.scanLine(dy);
			Point2d move = src_lt + Point2d(0, dy);
			for (int dx = 0; dx < dst_w; dx++, move += Point2d(1,0)) {
				int dst_gray = 0, dst_num = 0;
				//move is map point for dst[dy, dx]
				for (int i = 0; i < (int)src_map.size(); i++)
				if (src_map[i].rect.contains(move)) {
					Point2f offset = move - src_map[i].lt;
					uchar * plt = src_map[i].m.ptr<uchar>((int)offset.y, (int)offset.x);
					dst_gray += plt[0];
					dst_num++;
					if (pxy->get_merge_method() == MERGE_BY_DRAW_ORDER)
						break;
				}
				switch (dst_num) {
				case 0:
					pd[dx] = 0;
					break;
				case 1:
					pd[dx] = dst_gray;
					break;
				case 2:
					pd[dx] = dst_gray >> 1;
					break;
				default:
					pd[dx] = dst_gray / dst_num;
					break;
				}
			}
		}
		pr.img = img;
		return;
	}

	int src_row_step = (int) src_map[0].m.step.p[0] / sizeof(uchar);
	double dst_w_1 = 1.0 / (dst_w - 1);
	for (int dy = 0; dy < dst_w; dy++) {
		uchar *pd = img.scanLine(dy);
		double alpha = dy * dst_w_1;
		Point2d start = src_lt * (1 - alpha) + src_lb * alpha;
		Point2d end = src_rt * (1 - alpha) + src_rb * alpha;
		Point2d step = (end - start) * dst_w_1;
		Point2d move = start;
		for (int dx = 0; dx < dst_w; dx++, move += step) {
			int dst_gray = 0, dst_num = 0;
			//move is map point for dst[dy, dx]
			for (int i = 0; i < (int) src_map.size(); i++) 
			if (src_map[i].rect.contains(move)) {
				Point2f offset = move - src_map[i].lt;
				uchar * plt = src_map[i].m.ptr<uchar>((int)offset.y, (int)offset.x);
				int glt = plt[0];
				bool x_in_range = (offset.x > 0 && move.x < src_map[i].rb.x);
				bool y_in_range = (offset.y > 0 && move.y < src_map[i].rb.y);
				int grt = x_in_range ? plt[1] : glt;
				int glb = y_in_range ? plt[src_row_step] : glt;
				int grb = (x_in_range && y_in_range) ? plt[src_row_step + 1] :
					((!x_in_range && y_in_range) ? glb :
					((x_in_range && !y_in_range) ? grt : glt));
				float deltax = offset.x - (int)offset.x;
				float deltay = offset.y - (int)offset.y;
				dst_gray += (deltax * glt + (1 - deltax) * grt) * deltay + (deltax * glb + (1 - deltax) * grb) * (1 - deltay);
				dst_num++;
				if (pxy->get_merge_method() == MERGE_BY_DRAW_ORDER)
					break;
			}
			switch (dst_num) {
			case 0:
				qCritical("dst_num=0 at (x=%d,y=%d)", dx, dy);
				pd[dx] = 255;
				break;
			case 1:
				pd[dx] = dst_gray;
				break;
			case 2:
				pd[dx] = dst_gray >> 1;
				break;
			default:
				pd[dx] = dst_gray / dst_num;
				break;
			}
		}
	}

	pr.img = img;
}

void RenderImage::set_cfg_para(int layer, const ConfigPara & _cpara)
{
	if (layer > cpara.size() || layer < 0)
		return;
	if (layer == cpara.size()) {
		cpara.push_back(_cpara);
		if (layer == mapxy.size()) {
			mapxy.push_back(MapXY());
			src_img_size.push_back(Size(0, 0));
		}
	}
	else {
		if (cpara[layer].img_path != _cpara.img_path)
			premap_cache.clear(layer);
		cpara[layer] = _cpara;
		postmap_cache.clear(layer);
	}
	char file_name[200];
	sprintf(file_name, "%s%d_%d.jpg", _cpara.img_path.c_str(), 1, 1);
	qDebug("loadImage, %s", file_name);
	Mat raw_img = imread(file_name);
	src_img_size[layer] = Size(raw_img.cols - _cpara.clip_l - _cpara.clip_r, raw_img.rows - _cpara.clip_d - _cpara.clip_u);
	qInfo("set config, l=%d, nx=%d, ny=%d, img_w=%d, img_h=%d", layer,
		_cpara.img_num_w, _cpara.img_num_h, raw_img.cols, raw_img.rows);
}

ConfigPara RenderImage::get_cfg_para(int layer)
{
	if (layer >= cpara.size() || layer < 0)
		return ConfigPara();
	return cpara[layer];
}

Size RenderImage::get_src_img_size(int layer)
{
	if (layer >= cpara.size() || layer < 0)
		return Size(0, 0);
	return src_img_size[layer];
}
void RenderImage::set_mapxy(int layer, const MapXY & _mapxy)
{
	if (layer > mapxy.size() || layer < 0)
		return;
	if (layer == mapxy.size())
		mapxy.push_back(_mapxy);
	else {
		mapxy[layer] = _mapxy;
		postmap_cache.clear(layer);
	}
	qInfo("set mapxy, beta=%f, z0x=%f, z0y=%f", layer, _mapxy.get_beta(), _mapxy.get_default_zoomx(), _mapxy.get_default_zoomy());
}

MapXY RenderImage::get_mapxy(int layer)
{
	if (layer >= cpara.size() || layer < 0)
		return MapXY();
	return mapxy[layer];
}

bool RenderImage::is_mapxy_origin(int layer)
{
	if (layer >= cpara.size() || layer < 0)
		return true;
	return mapxy[layer].is_original();
}

void RenderImage::set_dst_wide(int wide)
{
	if (dst_w != wide) {
		dst_w = wide;
		postmap_cache.clear(-1);
	}
}

int RenderImage::get_dst_wide()
{
	return dst_w;
}

void RenderImage::render_img(const vector<MapID> & map_id, vector<QImage> & imgs, const vector<MapID> & draw_order, int load_flag)
{
	vector<MapRequest> mrs;
	
	if (load_flag != prev_load_flag) {
		prev_load_flag = load_flag;
		premap_cache.clear(-1);
		postmap_cache.clear(-1);
	}
	if (prev_draw_order.size() == draw_order.size()) {
		for (int i = 0; i < (int) draw_order.size(); i++)
		if (prev_draw_order[i] != draw_order[i]) {
			postmap_cache.clear(-1);
			prev_draw_order = draw_order;
			break;
		}
	} 
	else {
		postmap_cache.clear(-1);
		prev_draw_order = draw_order;
	}
	imgs.resize(map_id.size());
	for (int i = 0; i < (int)map_id.size(); i++) {
		PostmapImg * pimg;
		postmap_cache.find_repush(map_id[i], &pimg);
		if (pimg == NULL) {			
			int layer = MAPID_L(map_id[i]);
			if (layer < cpara.size()) {
				MapRequest mr;
				mr.id = map_id[i];
				mr.cpara = &cpara[layer];
				mr.dst_w = dst_w;
				mr.load_flag = load_flag;
				mr.pmapxy = &mapxy[layer];
				mr.premap_cache = &premap_cache;
				mr.src = src_img_size[layer];
				mr.draw_order = &draw_order;
				mrs.push_back(mr);
			}
			else
				qFatal("Not set layer %d cpara before render_img", layer);
		}
		else
			imgs[i] = pimg->data;
	}

#if PARALLEL
	QtConcurrent::blockingMap<vector<MapRequest> >(mrs, thread_map_image);
#else
	for (int i = 0; i < (int)mrs.size(); i++)
		thread_map_image(mrs[i]);
#endif

	for (int i = 0, j = 0; i < (int)map_id.size(); i++) {
		PostmapImg * pimg;
		postmap_cache.find_repush(map_id[i], &pimg);
		if (pimg == NULL) {
			CV_Assert(mrs[j].id == map_id[i]);
			imgs[i] = mrs[j].img;
			postmap_cache.insert(map_id[i], mrs[j].img);
			j++;
		}
	}

	premap_cache.remove_exceed();
	postmap_cache.remove_exceed();
}

RenderImage::RenderImage()
{
	dst_w = 1024;
	premap_cache.set_size(200);
	postmap_cache.set_size(30);
}

