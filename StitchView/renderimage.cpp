#include "renderimage.h"
#include <QMutexLocker>
#include <QThread>
#include <algorithm>
#include <QColor>
#include <QtConcurrent>

#define PARALLEL 1
#define PRINT_LOAD_IMAGE 0

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

//xy.first is x, xy.second is y, return (a,b), y= ax + b
pair<double, double> least_square(const vector<pair<int, int> >& xy) {
	double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
	for (int i = 0; i<xy.size(); ++i) {
		t1 += xy[i].first * xy[i].first;
		t2 += xy[i].first;
		t3 += xy[i].first * xy[i].second;
		t4 += xy[i].second;
	}
	double a = (t3*xy.size() - t2*t4) / (t1*xy.size() - t2*t2);
	double b = (t1*t4 - t2*t3) / (t1*xy.size() - t2*t2);
	return make_pair(a, b);
}

void MapXY::recompute_tp(vector<TurnPoint> & tp, double default_zoom)
{
	for (int i = 0; i < (int)tp.size() - 1; i++) {
		CV_Assert(tp[i + 1].sp != tp[i].sp && tp[i + 1].dp != tp[i].dp);
		tp[i].sz = (tp[i + 1].dp - tp[i].dp) / (tp[i + 1].sp - tp[i].sp);
		tp[i].dz = 1 / tp[i].sz;
	}
	tp.back().sz = default_zoom;
	tp.back().dz = 1 / default_zoom;
}

//add (s,d) to tp
static void add_turn_point(vector<TurnPoint> & tp, double s, double d)
{
    for (int i = 0; i < (int)tp.size(); i++) {
        if (tp[i].sp >= s) {
            if (tp[i].dp <= d || tp[i].sp == s)
                qCritical("add_turn_pointx error, dp=%f, dx=%f, sp=%f, sx=%f", tp[i].dp, d, tp[i].sp, s);

			if (i != 0) {
                if (tp[i - 1].dp >= d)
                    qFatal("add_turn_pointx err, dp=%f, dx=%f, sp=%f, sx=%f", tp[i - 1].dp, d, tp[i].sp, s);
			}
            tp.insert(tp.begin() + i, TurnPoint(s, d));
			return;
		}
	}
    tp.push_back(TurnPoint(s, d));
	//recompute_tp(tx, z0x);
}

/*
output tp, turn point result
inout nxy, nail's x or y nxy.first is mid, nxy.second is dst
output z, default slope rate
return 0 if good, else bad
*/
double MapXY::recompute_turn_point(vector<TurnPoint> & tp, vector<pair<int, int> > & nxy, double & z)
{
	double ret = 0;
    //1 sort and merge
	tp.clear();
    sort(nxy.begin(), nxy.end());
    for (int i = 0, weight = 1; i + 1 < (int)nxy.size();) {
        if (abs(nxy[i].first - nxy[i + 1].first) < merge_pt_distance ||
            abs(nxy[i].second - nxy[i + 1].second) < merge_pt_distance) {
            nxy[i].first = (weight * nxy[i].first + nxy[i + 1].first) / (weight + 1);
            nxy[i].second = (weight * nxy[i].second + nxy[i + 1].second) / (weight + 1);
            weight++;
            nxy.erase(nxy.begin() + i + 1);
        }
        else {
            i++;
            weight = 1;
        }
    }
	if (nxy.size() == 1) {
		add_turn_point(tp, nxy[0].first, nxy[0].second);
		return 0;
	}
    else {
        //2 compute nihe line
        pair<double, double> line = least_square(nxy);
        z = line.first;
        vector<double> nx_shift, errs;
        for (int i = 0; i < (int)nxy.size(); i++) {
            double err = nxy[i].second - nxy[i].first *line.first - line.second;
            //following do shift to move close to nihe line
            if (abs(err) < max_pt_error)
                err = 0;
            else
                err = (err > 0) ? err - max_pt_error : err + max_pt_error;
            nx_shift.push_back(nxy[i].first *line.first + line.second + err);
			errs.push_back(err);
        }

        //3 use cubic spline to compute turn point
        add_turn_point(tp, nxy[0].first, nx_shift[0]);
        int x1 = nxy[0].first, nx_idx=0;
        while (1) {
            x1 += 512;
            while (nx_idx + 1 < nxy.size() && x1 > nxy[nx_idx + 1].first)
                nx_idx++;
            if (nx_idx + 1 == nxy.size()) {
                add_turn_point(tp, nxy[nx_idx].first, nx_shift[nx_idx]);
                break;
            }
            double u = ((double) x1 - nxy[nx_idx].first) / (nxy[nx_idx + 1].first - nxy[nx_idx].first);
			CV_Assert(u >= 0 && u <= 1);
            int y1 = nx_shift[nx_idx] * (2 * u*u*u - 3 * u*u + 1) + nx_shift[nx_idx + 1] * (-2 * u*u*u + 3 * u*u) +
				line.first * (nxy[nx_idx + 1].first - nxy[nx_idx].first) * (2 * u*u*u - 3 * u*u + u);
            add_turn_point(tp, x1, y1);
        }

		//4 check if result is good
		for (int i = 0; i + 1 < (int)nxy.size(); i++) {
			double err_slope = (errs[i + 1] - errs[i]) / (nxy[i + 1].first - nxy[i].first);
			ret = max(ret, abs(err_slope));
		}		
    }
	return ret;
}

void MapXY::set_original()
{
	beta = 0; //set default rotation angle
	cos_beta = 1;
	sin_beta = 0;
	z0x = 1;
	z0y = 1;
	tx.clear();
	ty.clear();
	tx.push_back(TurnPoint(0, 0)); //push default zoom x 1
	ty.push_back(TurnPoint(0, 0)); //push default zoom y 1
	merge_method = MERGE_BY_DRAW_ORDER;
	merge_pt_distance = 512;
	max_pt_error = 1;
	max_slope = 0.001;
}

bool MapXY::is_original() const
{
	return (beta == 0 && z0x == 1 && z0y == 1 && tx.size() == 1 && ty.size() == 1
		&& (abs(tx[0].sp - tx[0].dp - (int)(tx[0].sp - tx[0].dp)) < 0.00001)
		&& (abs(ty[0].sp - ty[0].dp - (int)(ty[0].sp - ty[0].dp)) < 0.00001));
}

Point2d MapXY::mid2dst(Point2d mid) const
{
	Point2d dst;
	int i;
	for (i = 0; i < (int)tx.size(); i++) {
		if (mid.x < tx[i].sp) {
			if (i == 0)
				dst.x = tx[0].dp + z0x * (mid.x - tx[0].sp);
			else
				dst.x = tx[i - 1].dp + tx[i - 1].sz * (mid.x - tx[i - 1].sp);
			break;
		}
	}
	if (i == (int)tx.size())
		dst.x = tx[i - 1].dp + z0x * (mid.x - tx[i - 1].sp);

	for (i = 0; i < (int)ty.size(); i++) {
		if (mid.y < ty[i].sp) {
			if (i == 0)
				dst.y = ty[0].dp + z0y * (mid.y - ty[0].sp);
			else
				dst.y = ty[i - 1].dp + ty[i - 1].sz * (mid.y - ty[i - 1].sp);
			break;
		}
	}
	if (i == (int)ty.size())
		dst.y = ty[i - 1].dp + z0y * (mid.y - ty[i - 1].sp);
	return dst;
}

Point2d MapXY::dst2mid(Point2d dst) const
{
	Point2d mid;
	int i;
	for (i = 0; i < (int)tx.size(); i++) {
		if (dst.x < tx[i].dp) {
			if (i == 0)
				mid.x = tx[0].sp + (dst.x - tx[0].dp) / z0x;
			else
				mid.x = tx[i - 1].sp + tx[i - 1].dz * (dst.x - tx[i - 1].dp);
			break;
		}
	}
	if (i == (int)tx.size())
		mid.x = tx[i - 1].sp + (dst.x - tx[i - 1].dp) / z0x;

	for (i = 0; i < (int)ty.size(); i++) {
		if (dst.y < ty[i].dp) {
			if (i == 0)
				mid.y = ty[0].sp + (dst.y - ty[0].dp) / z0y;
			else
				mid.y = ty[i - 1].sp + ty[i - 1].dz * (dst.y - ty[i - 1].dp);
			break;
		}
	}
	if (i == (int)ty.size())
		mid.y = ty[i - 1].sp + (dst.y - ty[i - 1].dp) / z0y;
	return mid;
}

/*
	Return 0: success, 1 means x wrong, 2 means y wrong
*/
double MapXY::recompute(const vector<pair<Point, Point> > & nail)
{
	double ret = 0;
	if (nail.size() == 0)
		return ret;
	
	vector<pair<int, int> > nx, ny;
	for (int i = 0; i < nail.size(); i++) {
		Point mid = src2mid(nail[i].first);
		nx.push_back(make_pair(mid.x, nail[i].second.x));
		ny.push_back(make_pair(mid.y, nail[i].second.y));
	}

	ret = recompute_turn_point(tx, nx, z0x);
	recompute_tp(tx, z0x);
	ret = max(ret, recompute_turn_point(ty, ny, z0y));
	recompute_tp(ty, z0y);

	return ret;
}

class PremapTailorImg {
public:
	int draw_order;
	Rect_<double>  rect; //it is in pixel
	Point2d lt, rb;
	Mat m; //it is only a reference to PremapImg.m, it doesn't allocate memory
	PremapTailorImg() {}
	PremapTailorImg(const Mat _m, Rect_<double> & _r, unsigned _draw_order = 0) {
		lt = Point2d(_r.x + 0.5, _r.y + 0.5);
		rb = Point2d(_r.x + _r.width - 1.5, _r.y + _r.height - 1.5);
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
	QImage img;
};

//It is running in global thread-pool
void thread_map_image(MapRequest & pr)
{
	int dst_w = pr.dst_w;
	int l = MAPID_L(pr.id);
	int biggest_channel = 0;
	const MapXY * pxy = pr.pmapxy;

	//0 Following compute 4 dst vertex and src point
	Point lt, rb;
	Point2d src_lt, src_rb, src_lb, src_rt;

	Point dst_lt(dst_w * MAPID_X(pr.id), dst_w * MAPID_Y(pr.id));
	Point dst_rb(dst_lt.x + dst_w - 1, dst_lt.y + dst_w - 1);
	Point dst_lb(dst_lt.x, dst_rb.y);
	Point dst_rt(dst_rb.x, dst_lt.y);
	bool load_unmap_image = pxy->is_original();

	src_lt = pxy->dst2src(dst_lt); //dst_lt map to src_lt, src_lt is not src left top.
	src_rb = pxy->dst2src(dst_rb);
	src_lb = pxy->dst2src(dst_lb);
	src_rt = pxy->dst2src(dst_rt);

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
			sprintf(file_name, pr.cpara->get_img_name(x, y).c_str());
#if PRINT_LOAD_IMAGE
			qDebug("loadImage, %s", file_name);			
#endif
			Mat raw_img = imread(file_name, pr.cpara->load_flag);
			//pmc->insert(id, raw_img, Point(pr.cpara->offset(y, x)[1], pr.cpara->offset(y, x)[0]), pr.src);
			if (raw_img.empty())
				raw_img.create(1, 1, CV_8UC1);
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
				biggest_channel = max(biggest_channel, pmap->m.type());
				for (int i = 0; i < (int)pr.draw_order->size(); i++)
				if ((*pr.draw_order)[i] == id)
					order += (i + 1) * 0x10000;		
				if (pmap->m.cols >= pr.cpara->clip_l + pr.src.width && pmap->m.rows >= pr.cpara->clip_u + pr.src.height)
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

	QImage img;
	switch (biggest_channel) {
	case CV_8UC1: 
		{
			img = QImage(dst_w, dst_w, QImage::Format_Indexed8);
			// Set the color table (used to translate colour indexes to qRgb values)
			img.setColorCount(256);
			for (int i = 0; i < 256; i++)
				img.setColor(i, qRgb(i, i, i));

			if (src_map.empty()) {
				img.fill(QColor(0, 0, 0));
				pr.img = img;
				return;
			}
			break;
		}
	case CV_8UC3:
		img = QImage(dst_w, dst_w, QImage::Format_RGB888);
		break;
	}
	

	//2 do remap for each dst point
	if (load_unmap_image) {
		for (int dy = 0; dy < dst_w; dy++) {
			uchar *pd = img.scanLine(dy);
			Point2d move = src_lt + Point2d(0, dy);
			for (int dx = 0; dx < dst_w; dx++, move += Point2d(1,0)) {
				int dst_gray = 0, dst_num = 0, dst_gray1 = 0, dst_gray2 = 0;
				//move is map point for dst[dy, dx]
				for (int i = 0; i < (int)src_map.size(); i++)
				if (src_map[i].rect.contains(move)) {
					Point2f offset = move - src_map[i].lt;
					uchar * plt = src_map[i].m.ptr<uchar>((int)offset.y, (int)offset.x);
					dst_gray += plt[0];
					if (src_map[i].m.type() == CV_8UC3) {
						dst_gray1 += plt[1];
						dst_gray2 += plt[2];
					}
					dst_num++;
					if (pxy->get_merge_method() == MERGE_BY_DRAW_ORDER)
						break;
				}
				if (biggest_channel == CV_8UC1) {
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
				if (biggest_channel == CV_8UC3) {
					switch (dst_num) {
					case 0:
						pd[dx * 3] = 0;
						pd[dx * 3 + 1] = 0;
						pd[dx * 3 + 2] = 0;
						break;
					case 1:
						pd[dx* 3] = dst_gray2;
						pd[dx* 3 + 1] = dst_gray1;
						pd[dx* 3 + 2] = dst_gray;
						break;
					case 2:
						pd[dx * 3] = dst_gray2 >> 1;
						pd[dx * 3 + 1] = dst_gray1 >> 1;
						pd[dx * 3 + 2] = dst_gray >> 1;
						break;
					default:
						pd[dx * 3] = dst_gray2 / dst_num;
						pd[dx * 3 + 1] = dst_gray1 / dst_num;
						pd[dx * 3 + 2] = dst_gray / dst_num;
						break;
					}
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
			int dst_gray[3] = { 0, 0, 0 }, dst_num = 0;
			//move is map point for dst[dy, dx]
			for (int i = 0; i < (int) src_map.size(); i++) 
			if (src_map[i].rect.contains(move)) {
				Point2f offset = move - src_map[i].lt;
				uchar * plt = src_map[i].m.ptr<uchar>((int)offset.y, (int)offset.x);
				bool x_in_range = (offset.x > 0 && move.x < src_map[i].rb.x);
				bool y_in_range = (offset.y > 0 && move.y < src_map[i].rb.y);
				float deltax = 1 - (offset.x - (int)offset.x);
				float deltay = 1 - (offset.y - (int)offset.y);
				int map_type = src_map[i].m.type();
				if (map_type == CV_8UC1) {
					int glt = plt[0];					
					int grt = x_in_range ? plt[1] : glt;
					int glb = y_in_range ? plt[src_row_step] : glt;
					int grb = (x_in_range && y_in_range) ? plt[src_row_step + 1] :
						((!x_in_range && y_in_range) ? glb :
						((x_in_range && !y_in_range) ? grt : glt));
					dst_gray[0] += (deltax * glt + (1 - deltax) * grt) * deltay + (deltax * glb + (1 - deltax) * grb) * (1 - deltay);
				}
				if (map_type == CV_8UC3) {
					for (int j = 0; j < 3; j++) {
						int glt = plt[j];
						int grt = x_in_range ? plt[j+3] : glt;
						int glb = y_in_range ? plt[j+src_row_step] : glt;
						int grb = (x_in_range && y_in_range) ? plt[j+src_row_step + 3] :
							((!x_in_range && y_in_range) ? glb :
							((x_in_range && !y_in_range) ? grt : glt));
						dst_gray[j] += (deltax * glt + (1 - deltax) * grt) * deltay + (deltax * glb + (1 - deltax) * grb) * (1 - deltay);
					}
				}
				dst_num++;
				if (pxy->get_merge_method() == MERGE_BY_DRAW_ORDER)
					break;
			}
			if (biggest_channel == CV_8UC1) {
				switch (dst_num) {
				case 0:
					pd[dx] = 0;
					break;
				case 1:
					pd[dx] = dst_gray[0];
					break;
				case 2:
					pd[dx] = dst_gray[0] >> 1;
					break;
				default:
					pd[dx] = dst_gray[0] / dst_num;
					break;
				}
			}
			if (biggest_channel == CV_8UC3) {
				switch (dst_num) {
				case 0:
					pd[dx * 3] = 0;
					pd[dx * 3 + 1] = 0;
					pd[dx * 3 + 2] = 0;
					break;
				case 1:
					pd[dx * 3] = dst_gray[2];
					pd[dx * 3 + 1] = dst_gray[1];
					pd[dx * 3 + 2] = dst_gray[0];
					break;
				case 2:
					pd[dx * 3] = dst_gray[2] >> 1;
					pd[dx * 3 + 1] = dst_gray[1] >> 1;
					pd[dx * 3 + 2] = dst_gray[0] >> 1;
					break;
				default:
					pd[dx * 3] = dst_gray[2] / dst_num;
					pd[dx * 3 + 1] = dst_gray[1] / dst_num;
					pd[dx * 3 + 2] = dst_gray[0] / dst_num;
					break;
				}
			}
		}
	}

	pr.img = img;
}

void RenderImage::set_cfg_para(int layer, const ConfigPara * _cpara)
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
		if (cpara[layer]->img_path != _cpara->img_path || cpara[layer]->load_flag != _cpara->load_flag)
			premap_cache.clear(layer);
		cpara[layer] = _cpara;
		postmap_cache.clear(layer);
	}
	char file_name[200];
	sprintf(file_name, _cpara->get_img_name(1, 1).c_str());
	qDebug("loadImage, %s", file_name);
	Mat raw_img = imread(file_name);
	if (raw_img.empty()) {
		qCritical("please make sure (1,1) has valid image");
		src_img_size[layer] = Size(1, 1);
	} 
	else
		src_img_size[layer] = Size(raw_img.cols - _cpara->clip_l - _cpara->clip_r, raw_img.rows - _cpara->clip_d - _cpara->clip_u);
	qInfo("set config, l=%d, nx=%d, ny=%d, img_w=%d, img_h=%d", layer,
		_cpara->img_num_w, _cpara->img_num_h, raw_img.cols, raw_img.rows);
}

const ConfigPara * RenderImage::get_cfg_para(int layer)
{
	if (layer >= cpara.size() || layer < 0)
		return NULL;
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
	qInfo("set mapxy, l=%d, beta=%f, z0x=%f, z0y=%f", layer, _mapxy.get_beta(), _mapxy.get_default_zoomx(), _mapxy.get_default_zoomy());
}

MapXY RenderImage::get_mapxy(int layer)
{
	if (layer >= cpara.size() || layer < 0)
		return MapXY();
	return mapxy[layer];
}

int RenderImage::mapxy_merge_method(int layer)
{
	if (layer >= cpara.size() || layer < 0)
		return false;
	return mapxy[layer].get_merge_method();
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

void RenderImage::render_img(const vector<MapID> & map_id, vector<QImage> & imgs, const vector<MapID> & draw_order)
{
	vector<MapRequest> mrs;
	
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
				mr.cpara = cpara[layer];
				mr.dst_w = dst_w;
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
	premap_cache.set_size(36);
	postmap_cache.set_size(30);
}

