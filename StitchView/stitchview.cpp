#include "stitchview.h"
#include "iclayer.h"
#include <QPainter>
#include <algorithm>
#include <QtCore/QSharedPointer>
#include <fstream>
#include <QMessageBox>
#include <QDateTime>
#include <QApplication>

const int step_para = 3;
const int max_scale = 8;

enum MouseState {
	IDLE,
	PutFirstNail,
	PutSecondNail,
	ChangeNail
};

string thread_generate_diff(FeatExt * feature, int layer)
{
	feature->generate_feature_diff();
	QDateTime current = QDateTime::currentDateTime();
	string filename = current.toString("yy_MM_dd-hh.mm.ss-l").toStdString();
	char l = '0' + layer;
	filename = filename + l;
	filename = filename + ".xml";
	filename = qApp->applicationDirPath().toStdString() + "/WorkData/" + filename;
	qInfo("write to %s", filename.c_str());
	feature->write_diff_file(filename);
	return filename;
}

void thread_bundle_adjust(BundleAdjustInf * ba, FeatExt * feature, Mat_<Vec2i> *offset, Mat_<unsigned long long> * c_info, vector<FixEdge> * fe)
{
	ba->arrange(*feature, -1, -1, fe);
	*offset = ba->get_best_offset();
	*c_info = ba->get_corner();
}

StitchView::StitchView(QWidget *parent) : QWidget(parent)
{
    scale = 8;
	center = QPoint(0, 0);
    layer = -1;
	edge_cost = 0;
	minloc_shift = Point(0, 0);
	feature_layer = -1;
	draw_corner = 0;
	for (int i=0; i<3; i++)
		choose[i] = QPoint(-1, -1);
    resize(1800, 1000);
	ri.set_dst_wide(1024);
	setMouseTracking(true);
	setAutoFillBackground(false);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setAttribute(Qt::WA_NoSystemBackground, true);
	xgrid_size = 20;
	ygrid_size = 20;
	xoffset = 0;
	yoffset = 0;
	auto_save = 0;
	draw_grid = false;
	mouse_state = IDLE;
	cur_nail = Nail();
	ba = BundleAdjustInf::create_instance(2);
}

StitchView::~StitchView()
{
	for (int i = 0; i < lf.size(); i++)
		delete lf[i];
}

void StitchView::paintEvent(QPaintEvent *)
{
	if (lf.empty())
		return;
	Q_ASSERT(layer < (int) lf.size());
    QPoint ce(size().width()*scale/2, size().height()*scale/2);
    QSize s(size().width()*scale, size().height()*scale);
    view_rect = QRect(center - ce, s);
	if (view_rect.width() > lf[layer]->cpara.right_bound())
		view_rect.adjust(view_rect.width() - lf[layer]->cpara.right_bound(), 0, 0, 0);
	if (view_rect.height() > lf[layer]->cpara.bottom_bound())
		view_rect.adjust(0, view_rect.height() - lf[layer]->cpara.bottom_bound(), 0, 0);
    if (view_rect.left() < 0)
        view_rect.moveLeft(0);
	if (view_rect.right() > lf[layer]->cpara.right_bound() - 1)
		view_rect.moveRight(lf[layer]->cpara.right_bound() - 1);
    if (view_rect.top() < 0)
        view_rect.moveTop(0);
	if (view_rect.bottom() > lf[layer]->cpara.bottom_bound() - 1)
		view_rect.moveBottom(lf[layer]->cpara.bottom_bound() - 1);
	Q_ASSERT(view_rect.left() >= 0 && view_rect.right() < lf[layer]->cpara.right_bound() &&
		view_rect.top() >= 0 && view_rect.bottom() < lf[layer]->cpara.bottom_bound());

    QImage image(size(), QImage::Format_RGB32);
	image.fill(QColor(0, 0, 0));
    QPainter painter(&image);
	painter.setPen(QPen(Qt::green));
	painter.setBrush(QBrush(Qt::green));
	QRect screen_rect(0, 0, width(), height());	

	//1 Draw image
	//first loop, prepare for renderimage
	int dst_w = ri.get_dst_wide();
	int start_y = view_rect.top() / dst_w;
	int end_y = view_rect.bottom() / dst_w + 1;
	int start_x = view_rect.left() / dst_w;
	int end_x = view_rect.right() / dst_w + 1;
	vector<MapID> map_id;
	for (int y = start_y; y < end_y; y++)
		for (int x = start_x; x < end_x; x++)		
			map_id.push_back(MAPID(layer, x, y));
	vector<MapID> draw_order;
	for (int i = 0; i < 3; i++)
		draw_order.push_back(MAPID(layer, choose[i].x(), choose[i].y()));
	vector<QImage> imgs;
	ri.render_img(map_id, imgs, draw_order);

#if 0
	for (int i = 0; i < imgs.size(); i++) {
		char filename[100];
		sprintf(filename, "%d_%d.jpg", MAPID_Y(map_id[i]), MAPID_X(map_id[i]));
		imgs[i].save(filename);
	}
#endif
	//second loop, draw image
	int img_idx = 0;
	for (int y = start_y; y < end_y; y++)
		for (int x = start_x; x < end_x; x++)  {
			QImage subimg = imgs[img_idx].scaled(imgs[img_idx].size() / scale);
			QPoint offset(x * dst_w, y * dst_w);
			QRect img_rect((offset - view_rect.topLeft()) / scale, subimg.size());
			QRect target_rect = img_rect & screen_rect;
			QRect src_rect(target_rect.topLeft() - img_rect.topLeft(),
				target_rect.bottomRight() - img_rect.topLeft());
			painter.drawImage(target_rect, subimg, src_rect);
			qDebug("drawImage, (y=%d,x=%d) (l=%d,t=%d,w=%d, h=%d) -> (l=%d,t=%d,w=%d,h=%d)", y, x,
				src_rect.left(), src_rect.top(), src_rect.width(), src_rect.height(),
				target_rect.left(), target_rect.top(), target_rect.width(), target_rect.height());
			img_idx++;
		}

	//2 draw edge
	MapXY mxy = ri.get_mapxy(layer);
	Point src_lt = mxy.dst2src(TOPOINT(view_rect.topLeft())); //dst_lt map to src_lt, src_lt is not src left top.
	Point src_rb = mxy.dst2src(TOPOINT(view_rect.bottomRight()));
	Point src_lb = mxy.dst2src(TOPOINT(view_rect.bottomLeft()));
	Point src_rt = mxy.dst2src(TOPOINT(view_rect.topRight()));

	double minx = min(min(src_lt.x, src_rb.x), min(src_lb.x, src_rt.x));
	double miny = min(min(src_lt.y, src_rb.y), min(src_lb.y, src_rt.y));
	double maxx = max(max(src_lt.x, src_rb.x), max(src_lb.x, src_rt.x));
	double maxy = max(max(src_lt.y, src_rb.y), max(src_lb.y, src_rt.y));
	//now Point(minx,miny) is src left top, Point (max, maxy) is src right bottom

	Point lt = find_src_map(lf[layer]->cpara, Point(minx, miny), ri.get_src_img_size(layer), 1);
	Point rb = find_src_map(lf[layer]->cpara, Point(maxx + 1, maxy + 1), ri.get_src_img_size(layer), 0);
	//now lt is src left top map, rb is src right bottom map

	painter.setPen(QPen(Qt::green, 1));
	for (int y = lt.y; y <= rb.y; y++)
	for (int x = lt.x; x <= rb.x; x++) {
		Point src_corner(lf[layer]->cpara.offset(y, x)[1], lf[layer]->cpara.offset(y, x)[0]);
		Point src_corner1, src_corner2;
		if (y + 1 < lf[layer]->cpara.offset.rows)
			src_corner1 = Point(lf[layer]->cpara.offset(y + 1, x)[1], lf[layer]->cpara.offset(y + 1, x)[0]);
		else {
			src_corner1 = Point(lf[layer]->cpara.offset(y - 1, x)[1], lf[layer]->cpara.offset(y - 1, x)[0]);
			src_corner1 = 2 * src_corner - src_corner1;
		}
		if (x + 1 < lf[layer]->cpara.offset.cols)
			src_corner2 = Point(lf[layer]->cpara.offset(y, x + 1)[1], lf[layer]->cpara.offset(y, x + 1)[0]);
		else {
			src_corner2 = Point(lf[layer]->cpara.offset(y, x - 1)[1], lf[layer]->cpara.offset(y, x - 1)[0]);
			src_corner2 = 2 * src_corner - src_corner2;
		}
		Point src_edge_center[2] = { src_corner + src_corner2, src_corner + src_corner1};
		for (int i = 0; i < 2; i++) {
			src_edge_center[i].x = src_edge_center[i].x / 2;
			src_edge_center[i].y = src_edge_center[i].y / 2;
			QPoint dst_edge_center = TOQPOINT(mxy.src2dst(src_edge_center[i]));
			if (view_rect.contains(dst_edge_center)) {
				dst_edge_center = (dst_edge_center - view_rect.topLeft()) / scale + QPoint(0, -5);
				int fe = 0;
				if (y > 0 && i == 0)
					fe = lf[layer]->fix_edge[i](y - 1, x);
				if (x > 0 && i == 1)
					fe = lf[layer]->fix_edge[i](y, x - 1);
				//draw fix edge
				if (fe & BIND_Y_MASK)
					painter.drawLine(dst_edge_center - QPoint(0, 5), dst_edge_center + QPoint(0, 5));
				if (fe & BIND_X_MASK)
					painter.drawLine(dst_edge_center - QPoint(5, 0), dst_edge_center + QPoint(5, 0));
				if (draw_corner) {
					const EdgeDiff * ed = (i == 0) ? lf[layer]->feature.get_edge(0, y - 1, x) :
						lf[layer]->feature.get_edge(1, y, x - 1);
					if (ed) {
						Point src_corner3= (i==0) ? Point(lf[layer]->cpara.offset(y - 1, x)[1], lf[layer]->cpara.offset(y - 1, x)[0]) :
							Point(lf[layer]->cpara.offset(y, x - 1)[1], lf[layer]->cpara.offset(y, x - 1)[0]);
						Point oo = src_corner - src_corner3;
						Point shift = oo - ed->offset - ed->minloc * lf[layer]->cpara.rescale;
						int cost = ed->get_dif(oo, lf[layer]->cpara.rescale) - ed->mind;
						int val;
						switch (draw_corner) {
						case 1:
							val = (fe & BIND_X_MASK) ? 0 : shift.x;
							break;
						case 2:
							val = (fe & BIND_Y_MASK) ? 0 : shift.y;
							break;
						case 3:
							val = 0;
							break;
						case 4:
							val = cost;
							break;
						}
						painter.drawText(dst_edge_center + QPoint(-10, 15), QString::number(val));
					}					
				}
			}
		}
	}
	
	//3 Draw corner
	painter.setPen(QPen(Qt::red, 1));
	if (draw_corner) {
		for (int y = lt.y; y <= rb.y; y++)
		for (int x = lt.x; x <= rb.x; x++) {
			Point src_corner(lf[layer]->cpara.offset(y, x)[1], lf[layer]->cpara.offset(y, x)[0]);
			QPoint dst_corner = TOQPOINT(mxy.src2dst(src_corner));
			if (view_rect.contains(dst_corner)) {
				if (lf[layer]->corner_info.empty()) {
					painter.setPen(QPen(Qt::green));
					painter.setBrush(QBrush(Qt::green));
					painter.drawEllipse((dst_corner - view_rect.topLeft()) / scale, 3, 3);
				}
				else {
					unsigned long long info = lf[layer]->corner_info(y, x);
					short val;
					switch (draw_corner) {
					case 1:
						val = info & 0xffff;
						break;
					case 2:
						val = (info >> 16) & 0xffff;
						break;
					case 3:
						val = (info >> 32) & 0xffff;
						break;
					case 4:
						val = (info >> 48) & 0xffff;
						break;
					}
					QPoint center = (dst_corner - view_rect.topLeft()) / scale;
					painter.drawText(center - QPoint(10, 5), QString::number(val));
				}
			}
		}
	}

	//4 draw grid
	if (draw_grid) {
		painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
		int width = size().width();
		int height = size().height();
		if (ygrid_size >= 2)
		for (int i = (view_rect.top() - yoffset) / ygrid_size + 1; i <= (view_rect.bottom() - yoffset) / ygrid_size; i++) {
			int y = (i * ygrid_size + yoffset - view_rect.top()) / scale;
			painter.drawLine(0, y, width, y);
		}
		if (xgrid_size >= 2)
		for (int i = (view_rect.left() - xoffset) / xgrid_size + 1; i <= (view_rect.right() - xoffset) / xgrid_size; i++) {
			int x = (i * xgrid_size + xoffset - view_rect.left()) / scale;
			painter.drawLine(x, 0, x, height);
		}
	}

	//5 draw nail
	vector<Nail> ns;
	painter.setPen(Qt::yellow);
	painter.setBrush(QBrush());
	get_one_layer_nails(lf[layer], ns);
	for (int i = 0; i < (int)ns.size(); i++) {
		QPoint ns_point = TOQPOINT(mxy.src2dst(ns[i].p0));
		if (view_rect.contains(ns_point)) {
			ns_point = (ns_point - view_rect.topLeft()) / scale;
			painter.drawLine(ns_point + QPoint(-6, -6), ns_point + QPoint(6, 6));
			painter.drawLine(ns_point + QPoint(-6, 6), ns_point + QPoint(6, -6));
		}
		if (ns[i].lf1 == lf[layer]) {
			ns_point = TOQPOINT(ns[i].p1);
			ns_point = (ns_point - view_rect.topLeft()) / scale;
			painter.drawEllipse(ns_point, 3, 3);
		}
	}
	if (cur_nail.lf0 == lf[layer]) {
		QPoint ns_point = TOQPOINT(mxy.src2dst(cur_nail.p0));
		if (view_rect.contains(ns_point)) {
			ns_point = (ns_point - view_rect.topLeft()) / scale;
			painter.drawLine(ns_point + QPoint(-6, -6), ns_point + QPoint(6, 6));
			painter.drawLine(ns_point + QPoint(-6, 6), ns_point + QPoint(6, -6));
		}
	}

	//7 Copy image
	QPainter paint(this);
	paint.drawImage(QPoint(0, 0), image);
}

void StitchView::keyPressEvent(QKeyEvent *e)
{
	if (lf.empty())
		return;
	int step;
	switch (e->key()) {
	case Qt::Key_Left:		
		if (QApplication::queryKeyboardModifiers() == Qt::ControlModifier) {
			lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1] -= lf[layer]->cpara.rescale;
			if (lf[layer]->check_img_offset(MAKE_IMG_IDX(may_choose.x(), may_choose.y())))
				lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1] += lf[layer]->cpara.rescale;
			else
				ri.invalidate_cache(layer);
		}
		else 
		if (QApplication::queryKeyboardModifiers() == Qt::ShiftModifier) {
			vector <unsigned> imgs;
			imgs = lf[layer]->get_fix_img(MAKE_IMG_IDX(may_choose.x(), may_choose.y()), BIND_X_MASK);
			for (int i = 0; i < (int) imgs.size(); i++)
				lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[1] -= lf[layer]->cpara.rescale;
			ri.invalidate_cache(layer);
		}
		else {
			step = view_rect.width() * step_para / 10;
			view_rect.moveLeft(max(0, view_rect.left() - step));
			center = view_rect.center();
		}
		break;
	case Qt::Key_Up:		
		if (QApplication::queryKeyboardModifiers() == Qt::ControlModifier) {
			lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[0] -= lf[layer]->cpara.rescale;
			if (lf[layer]->check_img_offset(MAKE_IMG_IDX(may_choose.x(), may_choose.y())))
				lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[0] += lf[layer]->cpara.rescale;
			else
				ri.invalidate_cache(layer);
		}
		else
		if (QApplication::queryKeyboardModifiers() == Qt::ShiftModifier) {
			vector <unsigned> imgs;
			imgs = lf[layer]->get_fix_img(MAKE_IMG_IDX(may_choose.x(), may_choose.y()), BIND_Y_MASK);
			for (int i = 0; i < (int)imgs.size(); i++)
				lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[0] -= lf[layer]->cpara.rescale;
			ri.invalidate_cache(layer);
		}
		else {
			step = view_rect.height() * step_para / 10;
			view_rect.moveTop(max(0, view_rect.top() - step));
			center = view_rect.center();
		}
		break;
	case Qt::Key_Right:
		if (QApplication::queryKeyboardModifiers() == Qt::ControlModifier) {
			lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1] += lf[layer]->cpara.rescale;
			if (lf[layer]->check_img_offset(MAKE_IMG_IDX(may_choose.x(), may_choose.y())))
				lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1] -= lf[layer]->cpara.rescale;
			else
				ri.invalidate_cache(layer);
		}
		else
		if (QApplication::queryKeyboardModifiers() == Qt::ShiftModifier) {
			vector <unsigned> imgs;
			imgs = lf[layer]->get_fix_img(MAKE_IMG_IDX(may_choose.x(), may_choose.y()), BIND_X_MASK);
			for (int i = 0; i < (int)imgs.size(); i++)
				lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[1] += lf[layer]->cpara.rescale;
			ri.invalidate_cache(layer);
		}
		else {
			step = view_rect.width() * step_para / 10;
			view_rect.moveRight(min(lf[layer]->cpara.right_bound() - 1, view_rect.right() + step));
			center = view_rect.center();
		}
		break;
	case Qt::Key_Down:		
		if (QApplication::queryKeyboardModifiers() == Qt::ControlModifier) {
			lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[0] += lf[layer]->cpara.rescale;
			if (lf[layer]->check_img_offset(MAKE_IMG_IDX(may_choose.x(), may_choose.y())))
				lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[0] -= lf[layer]->cpara.rescale;
			else
				ri.invalidate_cache(layer);
		}
		else
		if (QApplication::queryKeyboardModifiers() == Qt::ShiftModifier) {
			vector <unsigned> imgs;
			imgs = lf[layer]->get_fix_img(MAKE_IMG_IDX(may_choose.x(), may_choose.y()), BIND_Y_MASK);
			for (int i = 0; i < (int)imgs.size(); i++)
				lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[0] += lf[layer]->cpara.rescale;
			ri.invalidate_cache(layer);
		}
		else {
			step = view_rect.height() * step_para / 10;
			view_rect.moveBottom(min(lf[layer]->cpara.bottom_bound() - 1, view_rect.bottom() + step));
			center = view_rect.center();
		}
		break;
	case Qt::Key_PageUp:
		if (scale < max_scale)
			scale = scale * 2;
		break;
	case Qt::Key_PageDown:
		if (scale > 0.4999)
			scale = scale / 2;
		break;
	case Qt::Key_0:
		if (lf.size() > 0)
			layer = 0;		
		break;
	case Qt::Key_1:
		if (lf.size() > 1)
			layer = 1;
		break;
	case Qt::Key_2:
		if (lf.size() > 2)
			layer = 2;
		break;
	case Qt::Key_3:
		if (lf.size() > 3)
			layer = 3;
		break;
	case Qt::Key_4:
		if (lf.size() > 4)
			layer = 4;
		break;
	case Qt::Key_5:
		if (lf.size() > 5)
			layer = 5;
		break;
	case Qt::Key_6:
		if (lf.size() > 6)
			layer = 6;
		break;
	case Qt::Key_7:
		if (lf.size() > 7)
			layer = 7;
		break;
	case Qt::Key_G:
		draw_grid = !draw_grid;
		break;
	case Qt::Key_C:
		draw_corner++;
		if (draw_corner == 5)
			draw_corner = 0;
		break; 
	case Qt::Key_M: {
			MapXY mxy = get_mapxy(layer);
			int merge = mxy.get_merge_method();
			mxy.set_merge_method(!merge);
			ri.set_mapxy(layer, mxy);
		}
		break;
	case Qt::Key_Delete:
	case Qt::Key_Escape: {
			switch (mouse_state) {
			case PutSecondNail:
				generate_mapxy();
			case PutFirstNail:
			case ChangeNail:
				cur_nail = Nail();
				mouse_state = IDLE;
				setCursor(Qt::ArrowCursor);
				break;			
			}
		}
	default:
		QWidget::keyPressEvent(e);
		return;
	}
	char title[100];
	char c = (draw_corner == 1) ? 'x' : (draw_corner == 2) ? 'y' : ((draw_corner == 0) ? ' ' : 'c');
	sprintf(title, "%d:%s s=%d %c", layer, lf[layer]->layer_name.c_str(), lf[layer]->cpara.rescale, c);
	emit title_change(QString::fromLocal8Bit(title));
	qDebug("Key=%d press, l=%d, s=%d, center=(%d,%d)", e->key(), layer, scale, center.x(), center.y());
	update();
}

void StitchView::mouseMoveEvent(QMouseEvent *event)
{
	if (lf.empty())
		return;
	QPoint mouse_point(event->localPos().x(), event->localPos().y());
	cur_mouse_point = mouse_point;
	Point offset;
	mouse_point = mouse_point * scale + view_rect.topLeft();

	QPoint prev_may_choose = may_choose;
	MapXY mxy = ri.get_mapxy(layer);
	Point src_point = mxy.dst2src(TOPOINT(mouse_point));
	may_choose = TOQPOINT(find_src_map(lf[layer]->cpara, src_point, ri.get_src_img_size(layer), 0));

	if (may_choose != prev_may_choose && lf[layer]->feature.is_valid()) {
		const EdgeDiff * ed = lf[layer]->feature.get_edge(may_choose.y(), may_choose.x(), prev_may_choose.y(), prev_may_choose.x());
		if (ed) {
			Point o0(lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1],
				lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[0]);
			Point o1(lf[layer]->cpara.offset(prev_may_choose.y(), prev_may_choose.x())[1],
				lf[layer]->cpara.offset(prev_may_choose.y(), prev_may_choose.x())[0]);
			Point oo = (o1.y + o1.x > o0.y + o0.x) ? o1 - o0 : o0 - o1;
			minloc_shift = oo - ed->offset - ed->minloc * lf[layer]->cpara.rescale;
			edge_cost = ed->get_dif(oo, lf[layer]->cpara.rescale) - ed->mind;
		}
	}
	char info[200];
    sprintf(info, "x=%d,y=%d,sx=%d,sy=%d,ix=%d,iy=%d,mx=%d,my=%d,c=%d", 
		mouse_point.x(), mouse_point.y(), src_point.x, src_point.y,
        may_choose.x() + 1, may_choose.y() + 1, minloc_shift.x, minloc_shift.y, edge_cost);
	emit MouseChange(info);
	QWidget::mouseMoveEvent(event);
}

void StitchView::mousePressEvent(QMouseEvent *event)
{
	if (lf.empty())
		return;
	QPoint mouse_point(event->localPos().x(), event->localPos().y());
	mouse_point = mouse_point * scale + view_rect.topLeft();
	switch (mouse_state) {
	case ChangeNail:
	{
		MapXY mxy = ri.get_mapxy(layer);
		cur_nail = search_nail(lf[layer], mxy.dst2src(TOPOINT(mouse_point)), 5);
		if (cur_nail.lf0 != NULL) {
			del_nail(cur_nail);
			if (cur_nail.lf1 != lf[layer])
				cur_nail.swap_order();
			cur_nail.lf1 = NULL;
			mouse_state = PutSecondNail;
			setCursor(Qt::CrossCursor);
		}
		break;
	}
	case PutFirstNail: 
	{
		cur_nail.lf0 = lf[layer];
		MapXY mxy = ri.get_mapxy(layer);
		cur_nail.p0 = mxy.dst2src(TOPOINT(mouse_point));
		mouse_state = PutSecondNail;
		break;
	}
	case PutSecondNail: 
	{
		cur_nail.lf1 = lf[layer];
		MapXY mxy = ri.get_mapxy(layer);
		cur_nail.p1 = mxy.dst2src(TOPOINT(mouse_point));
		add_nail(cur_nail);
		cur_nail = Nail();
		mouse_state = IDLE;
		setCursor(Qt::ArrowCursor);
		break;
	}
	case IDLE: 
	{
		if (QApplication::keyboardModifiers() == Qt::ControlModifier ||
			QApplication::keyboardModifiers() == Qt::ShiftModifier) {
			MapXY mxy = ri.get_mapxy(layer);
			Point src_point = mxy.dst2src(TOPOINT(mouse_point));
			int y = may_choose.y(), x = may_choose.x();
			Point src_corner(lf[layer]->cpara.offset(y, x)[1], lf[layer]->cpara.offset(y, x)[0]);
			Point src_corner1, src_corner2;
			if (y + 1 < lf[layer]->cpara.offset.rows)
				src_corner1 = Point(lf[layer]->cpara.offset(y + 1, x)[1], lf[layer]->cpara.offset(y + 1, x)[0]);
			else {
				src_corner1 = Point(lf[layer]->cpara.offset(y - 1, x)[1], lf[layer]->cpara.offset(y - 1, x)[0]);
				src_corner1 = 2 * src_corner - src_corner1;
			}
			if (x + 1 < lf[layer]->cpara.offset.cols)
				src_corner2 = Point(lf[layer]->cpara.offset(y, x + 1)[1], lf[layer]->cpara.offset(y, x + 1)[0]);
			else {
				src_corner2 = Point(lf[layer]->cpara.offset(y, x - 1)[1], lf[layer]->cpara.offset(y, x - 1)[0]);
				src_corner2 = 2 * src_corner - src_corner2;
			}
			Point src_corner3(src_corner2.x, src_corner1.y);
			Point src_edge_center[4] = { src_corner + src_corner1,
				src_corner + src_corner2,
				src_corner1 + src_corner3,
				src_corner2 + src_corner3
			};
			int choose = -1;
			int min_dis = 10000000;
			for (int i = 0; i <= 3; i++) {
				src_edge_center[i].x = src_edge_center[i].x / 2;
				src_edge_center[i].y = src_edge_center[i].y / 2;
				Point dis = src_point - src_edge_center[i];
				if (min_dis > abs(dis.x) + abs(dis.y)) {
					min_dis = abs(dis.x) + abs(dis.y);
					choose = i;
				}
			}
			unsigned edge_idx = 0xffffffff;
			unsigned new_fix;
			switch (choose) {
			case 0:
				if (x > 0) {
					new_fix = (lf[layer]->fix_edge[1](y, x - 1) + 1) & 3;
					edge_idx = MAKE_EDGE_IDX(x - 1, y, 1);
					lf[layer]->fix_edge[1](y, x - 1) = 0;
				}
				break;
			case 1:
				if (y > 0) {
					new_fix = (lf[layer]->fix_edge[0](y - 1, x) + 1) & 3;
					edge_idx = MAKE_EDGE_IDX(x, y - 1, 0);
					lf[layer]->fix_edge[0](y - 1, x) = 0;
				}
				break;
			case 2:
				new_fix = (lf[layer]->fix_edge[0](y, x) + 1) & 3;
				edge_idx = MAKE_EDGE_IDX(x, y, 0);
				lf[layer]->fix_edge[0](y, x) = 0;
				break;
			case 3:
				new_fix = (lf[layer]->fix_edge[1](y, x) + 1) & 3;
				edge_idx = MAKE_EDGE_IDX(x, y, 1);
				lf[layer]->fix_edge[1](y, x) = 0;
				break;
			}
			vector <unsigned> imgs_x;
			imgs_x = lf[layer]->get_fix_img(MAKE_IMG_IDX(EDGE_X(edge_idx), EDGE_Y(edge_idx)), BIND_X_MASK);
			vector <unsigned> imgs_y;
			imgs_y = lf[layer]->get_fix_img(MAKE_IMG_IDX(EDGE_X(edge_idx), EDGE_Y(edge_idx)), BIND_Y_MASK);
			qInfo("change fix_edge, l=%d, e=%d, x=%d, y=%d to %d", layer, EDGE_E(edge_idx), EDGE_X(edge_idx), EDGE_Y(edge_idx), new_fix);
			lf[layer]->fix_edge[EDGE_E(edge_idx)](EDGE_Y(edge_idx), EDGE_X(edge_idx)) = new_fix;
			int ret = lf[layer]->check_edge(edge_idx);
			if (ret)
				ri.invalidate_cache(layer);
			while (ret) {
				if (ret & 1) {
					for (int i = 0; i < (int)imgs_x.size(); i++)
						lf[layer]->cpara.offset(IMG_Y(imgs_x[i]), IMG_X(imgs_x[i]))[1] -= lf[layer]->cpara.rescale;
				}
				if (ret & 2) {
					for (int i = 0; i < (int)imgs_x.size(); i++)
						lf[layer]->cpara.offset(IMG_Y(imgs_x[i]), IMG_X(imgs_x[i]))[1] += lf[layer]->cpara.rescale;
				}
				if (ret & 4) {
					for (int i = 0; i < (int)imgs_y.size(); i++)
						lf[layer]->cpara.offset(IMG_Y(imgs_y[i]), IMG_X(imgs_y[i]))[0] -= lf[layer]->cpara.rescale;
				}
				if (ret & 8) {
					for (int i = 0; i < (int)imgs_y.size(); i++)
						lf[layer]->cpara.offset(IMG_Y(imgs_y[i]), IMG_X(imgs_y[i]))[0] += lf[layer]->cpara.rescale;
				}
				ret = lf[layer]->check_edge(edge_idx);
			}
		}
		else {
			if (choose[2] != may_choose) {
				choose[0] = choose[1];
				choose[1] = choose[2];
				choose[2] = may_choose;
			}
		}
		break;
	}
	}
	update();
	QWidget::mousePressEvent(event);
}

void StitchView::mouseReleaseEvent(QMouseEvent *event)
{
	QWidget::mousePressEvent(event);
}

void StitchView::timerEvent(QTimerEvent *e)
{
	if (e->timerId() == compute_feature_timer) {
		if (compute_feature.isFinished()) {
			killTimer(compute_feature_timer);
			emit notify_progress(0);			
			string filename = compute_feature.result();
			lf[feature_layer]->feature_file = filename;
			lf[feature_layer]->feature.read_diff_file(filename);
			QMessageBox::information(this, "Info", "Prepare finish");
		} else
			emit notify_progress(computing_feature.get_progress());
	}
}

bool StitchView::add_nail(Nail nail)
{
	for (int i = 0; i < (int)nails.size(); i++) {
		if (nails[i].within_range(nail, 1024)) {
			QMessageBox::information(this, "Add nail fail", "Nail add fail because of too near with other nail");
			return false;
		}
	}
	nails.push_back(nail);
	vector<double> slope =generate_mapxy();
	char a[300];
	for (int i = 0, j=0; i < slope.size(); i++)
		j += sprintf(a + j, "k%d=%7f,", i, slope[i]);	
	QMessageBox::information(this, "Add nail slope", QLatin1String(a));
	return true;
}

void StitchView::del_nail(Nail nail)
{
	for (int i = 0; i < (int)nails.size(); i++)
	if (nails[i] == nail) {
		nails.erase(nails.begin() + i);
		return;
	}
}

void StitchView::get_one_layer_nails(LayerFeature * lf, vector<Nail> &ns)
{
	for (int i = 0; i < (int)nails.size(); i++) {
		if (nails[i].lf0 == lf)
			ns.push_back(nails[i]);
		else
		if (nails[i].lf1 == lf) {
			Nail n;
			n.lf0 = nails[i].lf1;
			n.lf1 = nails[i].lf0;
			n.p0 = nails[i].p1;
			n.p1 = nails[i].p0;
			ns.push_back(n);
		}
	}
}

void StitchView::del_one_layer_nails(LayerFeature * lf)
{
	for (int i = 0; i < (int)nails.size(); i++)
	if (nails[i].lf0 == lf || nails[i].lf1 == lf) {
		nails[i] = nails.back();
		nails.pop_back();
	}	
}

Nail StitchView::search_nail(LayerFeature * lf, Point p, int range)
{
	for (int i = 0; i < (int)nails.size(); i++) {
		if (nails[i].within_range(lf, p, range))
			return nails[i];
	}
	return Nail();
}

vector<double> StitchView::generate_mapxy()
{
	vector<double> slope;
	vector< vector<Nail> > layer_nails(lf.size());
	for (int i = 0; i < (int)lf.size(); i++)
		get_one_layer_nails(lf[i], layer_nails[i]);
	vector<int> finish_map(lf.size(), 0); //if layer i finished, finish_map[i] < 0
	for (int l=0; l<(int) lf.size(); l++) {
		//1 choose which layer map
		int choose = 0;
		for (int i = 0; i < (int)layer_nails.size(); i++)
		if (finish_map[i] >= 0) {
			finish_map[i] = 0;
			for (int j = 0; j < (int)layer_nails[i].size(); j++)
			if (layer_nails[i][j].lf0 == layer_nails[i][j].lf1)
				finish_map[i]++;
			if (finish_map[i] > finish_map[choose]) {
				choose = i;
			}
		}
		qInfo("generate mapxy for layer %d, nails=%d", choose, finish_map[choose]);
		//2 set mxy for layer choose
		vector<pair<Point, Point> > abs_nails;
		for (int j = 0; j < (int)layer_nails[choose].size(); j++)
		if (layer_nails[choose][j].lf0 == layer_nails[choose][j].lf1)
			abs_nails.push_back(make_pair(layer_nails[choose][j].p0, layer_nails[choose][j].p1));
		CV_Assert(abs_nails.size() == finish_map[choose]);
		finish_map[choose] = -1;
		MapXY mxy = ri.get_mapxy(choose);
		slope.push_back(mxy.recompute(abs_nails));
		ri.set_mapxy(choose, mxy);

		//3 change nail with choose layer from src to dst
		for (int i = 0; i < (int)layer_nails.size(); i++) 
		if (finish_map[i] >= 0) {
			for (int j = 0; j < (int)layer_nails[i].size(); j++)
			if (layer_nails[i][j].lf1 == lf[choose]) {
				layer_nails[i][j].lf1 = layer_nails[i][j].lf0; //make it absolute
				layer_nails[i][j].p1 = mxy.src2dst(layer_nails[i][j].p1);
			}
		}
	}
	return slope;
}

int StitchView::which_layer(LayerFeature * l)
{
	for (int i = 0; i < (int)lf.size(); i++)
	if (lf[i] == l)
		return i;
	return -1;
}

//if _layer==-1, means current layer
int StitchView::set_config_para(int _layer, const ConfigPara & _cpara)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer > lf.size() || _layer < 0)
		return -1;
	if (_layer == lf.size()) {
		LayerFeature * layer_feature = new LayerFeature();
		if (lf.empty()) {
			ExtractParam ep;
			TuningPara _tpara;
			if (!ep.read_file("tune.xml"))
				QMessageBox::information(this, "Info", "not found tune.xml");
			else
				_tpara.read(ep, "Default");

			layer_feature->tpara = _tpara;
		}
		else
			layer_feature->tpara = lf[0]->tpara;		
		layer_feature->cpara = _cpara;
		layer_feature->fix_edge[0].create(_cpara.offset.rows - 1, _cpara.offset.cols);
		layer_feature->fix_edge[0] = 0;
		layer_feature->fix_edge[1].create(_cpara.offset.rows, _cpara.offset.cols - 1);
		layer_feature->fix_edge[1] = 0;
		lf.push_back(layer_feature);		
	}
	else {
		lf[_layer]->cpara = _cpara;
		lf[_layer]->fix_edge[0].create(_cpara.offset.rows - 1, _cpara.offset.cols);
		lf[_layer]->fix_edge[0] = 0;
		lf[_layer]->fix_edge[1].create(_cpara.offset.rows, _cpara.offset.cols - 1);
		lf[_layer]->fix_edge[1] = 0;
	}
	ri.set_cfg_para(_layer, &lf[_layer]->cpara);
	qInfo("set config, l=%d, s=%d, nx=%d, ny=%d", _layer, _cpara.rescale, _cpara.img_num_w, _cpara.img_num_h);
	return 0;
}

//if _layer==-1, means current layer
int StitchView::get_config_para(int _layer, ConfigPara & _cpara)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	_cpara = lf[_layer]->cpara;
	return 0;
}

//if _layer==-1, means current layer
int StitchView::set_tune_para(int _layer, const TuningPara & _tpara)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	lf[_layer]->tpara = _tpara;
	return 0;
}

//if _layer==-1, means current layer
int StitchView::get_tune_para(int _layer, TuningPara & _tpara)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	_tpara = lf[_layer]->tpara;
	return 0;
}

//if _layer==-1, means get tune of current layer
void StitchView::set_mapxy_dstw(int _layer, const MapXY & _mapxy, int _dst_w)
{
	if (_layer == -1)
		_layer = layer;
	ri.set_mapxy(_layer, _mapxy);
	ri.set_dst_wide(_dst_w);
	generate_mapxy();
	update();
}
//if _layer==-1, means get tune of current layer
MapXY StitchView::get_mapxy(int _layer)
{
	if (_layer == -1)
		_layer = layer;
	return ri.get_mapxy(_layer);
}

int StitchView::get_dst_wide()
{
	return ri.get_dst_wide();
}

void StitchView::set_grid(double _xoffset, double _yoffset, double _xgrid_size, double _ygrid_size)
{
	xoffset = _xoffset;
	yoffset = _yoffset;
	xgrid_size = _xgrid_size;
	ygrid_size = _ygrid_size;
	draw_grid = true;
	update();
}

void StitchView::get_grid(double & _xoffset, double & _yoffset, double & _xgrid_size, double & _ygrid_size)
{
	_xoffset = xoffset;
	_yoffset = yoffset;
	_xgrid_size = xgrid_size;
	_ygrid_size = ygrid_size;
}

int StitchView::compute_new_feature(int _layer)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is already running, wait it finish");
		return -2;
	}

	computing_feature.set_cfg_para(lf[_layer]->cpara);
	computing_feature.set_tune_para(lf[_layer]->tpara);
	feature_layer = _layer;
	compute_feature = QtConcurrent::run(thread_generate_diff, &computing_feature, _layer);
	compute_feature_timer = startTimer(1000);
	return 0;
}

int StitchView::optimize_offset(int _layer)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is running, wait it finish and redo optimize offset");
		return -2;
	}
	char name[100];
	sprintf(name, "/WorkData/autosave%d.xml", auto_save);
	auto_save = (auto_save + 1) % 5;

	string filename = qApp->applicationDirPath().toStdString() + name;
	write_file(filename);

	vector<FixEdge> fe;
	for (int i = 0; i < 2; i++) {
		for (int y = 0; y < lf[layer]->fix_edge[i].rows; y++)
		for (int x = 0; x < lf[layer]->fix_edge[i].cols; x++) {
			int a = lf[layer]->fix_edge[i](y, x);
			if (a != 0) {
				FixEdge new_fe;
				new_fe.idx = MAKE_EDGE_IDX(x, y, i);
				new_fe.bind_flag = a;
				if (i == 0) {
					new_fe.shift.y = lf[layer]->cpara.offset(y + 1, x)[0] - lf[layer]->cpara.offset(y, x)[0];
					new_fe.shift.x = lf[layer]->cpara.offset(y + 1, x)[1] - lf[layer]->cpara.offset(y, x)[1];
				}
				else {
					new_fe.shift.y = lf[layer]->cpara.offset(y, x + 1)[0] - lf[layer]->cpara.offset(y, x)[0];
					new_fe.shift.x = lf[layer]->cpara.offset(y, x + 1)[1] - lf[layer]->cpara.offset(y, x)[1];
				}
				fe.push_back(new_fe);
			}
		}
	}
	thread_bundle_adjust(ba, &lf[layer]->feature, &adjust_offset, &lf[layer]->corner_info, &fe);
	lf[layer]->cpara.offset = adjust_offset;
	ri.invalidate_cache(layer);
	update();

	return 0;
}

void StitchView::goto_xy(int x, int y)
{
	center = QPoint(x, y);
	qDebug("Goto s=%6.3f, c=(%d,%d), l=%d", scale, center.x(), center.y(), layer);
	update();
}

int StitchView::set_current_layer(int _layer) {
	if (_layer < get_layer_num() && _layer >= 0)
		layer = _layer;
	else
		return -1;
	update();
	return 0;
}

void StitchView::to_state_add_nail() {
	mouse_state = PutFirstNail;
	setCursor(Qt::CrossCursor);
}

void StitchView::to_state_change_nail() {
	mouse_state = ChangeNail;
}

int StitchView::output_layer(int _layer, string pathname) {
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	string filename = pathname + "/" + lf[layer]->layer_name + ".db";
	ICLayerWrInterface *ic = ICLayerWrInterface::create(filename, false, 1, 1, 0, 0, 0, 0, 0);
	ICLayerInterface *icl = ic->get_iclayer_inf();

	int dst_w = ri.get_dst_wide();
	int end_x = lf[layer]->cpara.right_bound() / dst_w + 1;
	int end_y = lf[layer]->cpara.bottom_bound() / dst_w + 1;
	icl->putBlockNumWidth(end_x, end_y, dst_w);
	vector<MapID> map_id;
	vector<MapID> draw_order;
	for (int i = 0; i < 3; i++)
		draw_order.push_back(MAPID(layer, choose[i].x(), choose[i].y()));
	float cnt = 0;
	for (int y = 0; y < end_y; y++)
	for (int x = 0; x < end_x; x++) {
		map_id.push_back(MAPID(layer, x, y));
		if (map_id.size() == 16 || (x + 1==end_x && y + 1==end_y)) {
			vector<QImage> imgs;
			ri.render_img(map_id, imgs, draw_order);
			CV_Assert(imgs.size() == map_id.size());			
			for (int i = 0; i < imgs.size(); i++) {
				QByteArray ba;
				QBuffer buffer(&ba);
				buffer.open(QIODevice::WriteOnly);
				imgs[i].save(&buffer, "JPG", 85);
				vector<uchar> buff;
				buff.resize(ba.size());
				memcpy(buff.data(), ba.data(), ba.size());
				icl->addRawImg(buff, MAPID_X(map_id[i]), MAPID_Y(map_id[i]), 0);
			}
			map_id.clear();
			cnt += 16;
			emit notify_progress(cnt * 0.8 / (end_y * end_x));
		}
	}
	CV_Assert(map_id.empty());
	delete ic;
	emit notify_progress(0);
}

int StitchView::delete_layer(int _layer)
{
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is running, can't delete layer");
		return -2;
	}
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	del_one_layer_nails(lf[_layer]);
	for (int i = _layer; i < (int)lf.size() - 1; i++) {
		MapXY mxy = ri.get_mapxy(i + 1);
		ri.set_mapxy(i, mxy);
		ri.set_cfg_para(i, &lf[i + 1]->cpara);
	}
	delete lf[_layer];
	lf.erase(lf.begin() + _layer);
	vector<double> slope = generate_mapxy();
	char a[300];
	for (int i = 0, j = 0; i < slope.size(); i++)
		j += sprintf(a + j, "k%d=%7f,", i, slope[i]);
	QMessageBox::information(this, "Add nail slope", QLatin1String(a));
	if (layer >= _layer)
		layer--;
	update();	
	return 0;
}

int StitchView::layer_up(int _layer)
{
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is running, can't delete layer");
		return -2;
	}
	if (_layer == -1)
		_layer = layer;
	if (_layer + 1 >= lf.size() || _layer < 0)
		return -1;
	MapXY mxy = ri.get_mapxy(_layer);
	MapXY mxy1 = ri.get_mapxy(_layer + 1);
	ri.set_mapxy(_layer, mxy1);
	ri.set_mapxy(_layer + 1, mxy);
	swap(lf[_layer], lf[_layer + 1]);
	ri.set_cfg_para(_layer, &lf[_layer]->cpara);
	ri.set_cfg_para(_layer + 1, &lf[_layer + 1]->cpara);
	generate_mapxy();
	if (layer == _layer)
		layer++;
	update();
	return 0;
}

int StitchView::layer_down(int _layer)
{
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is running, can't delete layer");
		return -2;
	}
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer <= 0)
		return -1;
	MapXY mxy = ri.get_mapxy(_layer);
	MapXY mxy1 = ri.get_mapxy(_layer - 1);
	ri.set_mapxy(_layer, mxy1);
	ri.set_mapxy(_layer - 1, mxy);
	swap(lf[_layer], lf[_layer - 1]);
	ri.set_cfg_para(_layer, &lf[_layer]->cpara);
	ri.set_cfg_para(_layer - 1, &lf[_layer - 1]->cpara);
	generate_mapxy();
	if (layer == _layer)
		layer--;
	update();
	return 0;
}

void StitchView::write_file(string file_name)
{
	FileStorage fs(file_name, FileStorage::WRITE);

	fs << "layer_num" << (int) lf.size();
	for (int i = 0; i < (int)lf.size(); i++) {
		char name[30];
		sprintf(name, "cpara%d", i);
		fs << name << lf[i]->cpara;
		sprintf(name, "tpara%d", i);
		fs << name << lf[i]->tpara;
		sprintf(name, "diff_file%d", i);
		fs << name << lf[i]->feature_file;
		sprintf(name, "mapxy%d", i);
		fs << name << ri.get_mapxy(i);
		sprintf(name, "fixedge%d_0", i);
		fs << name << lf[i]->fix_edge[0];
		sprintf(name, "fixedge%d_1", i);
		fs << name << lf[i]->fix_edge[1];
		sprintf(name, "name%d", i);
		fs << name << lf[i]->layer_name;
	}
	fs << "dst_w" << ri.get_dst_wide();
	fs << "Nails" << "[";
	for (int i = 0; i < (int)nails.size(); i++) {
		fs << "{" << "l0" << which_layer(nails[i].lf0);
		fs << "p0x" << nails[i].p0.x;
		fs << "p0y" << nails[i].p0.y;
		fs << "l1" << which_layer(nails[i].lf1);
		fs << "p1x" << nails[i].p1.x;
		fs << "p1y" << nails[i].p1.y << "}";
	}
	fs << "]";

	Point ct(center.x(), center.y());
	Point ce0(choose[0].x(), choose[0].y());
	Point ce1(choose[1].x(), choose[1].y());
	Point ce2(choose[2].x(), choose[2].y());
	fs << "layer" << layer;
	fs << "scale" << scale;
	fs << "center" << ct;
	fs << "choose" << ce0;
	fs << "choose1" << ce1;
	fs << "choose2" << ce2;
	fs << "draw_corner" << draw_corner;
	fs << "xoffset" << xoffset;
	fs << "yoffset" << yoffset;
	fs << "xgrid_size" << xgrid_size;
	fs << "ygrid_size" << ygrid_size;
	fs.release();
}

int StitchView::read_file(string file_name)
{
	FileStorage fs(file_name, FileStorage::READ);
	if (fs.isOpened()) {
		int layer_num;
		for (int i = 0; i < lf.size(); i++)
			delete lf[i];
		layer_num = (int)fs["layer_num"];
		lf.resize(layer_num);
		for (int i = 0; i < (int)lf.size(); i++) {
			lf[i] = new LayerFeature();
			char name[30];
			sprintf(name, "cpara%d", i);
			fs[name] >> lf[i]->cpara;
			ri.set_cfg_para(i, &lf[i]->cpara);
			sprintf(name, "tpara%d", i);
			fs[name] >> lf[i]->tpara;
			sprintf(name, "diff_file%d", i);
			fs[name] >> lf[i]->feature_file;
			if (!lf[i]->feature_file.empty())
				lf[i]->feature.read_diff_file(lf[i]->feature_file);
			MapXY mxy;
			sprintf(name, "mapxy%d", i);
			fs[name] >> mxy;
			ri.set_mapxy(i, mxy);
			sprintf(name, "fixedge%d_0", i);
			fs[name] >> lf[i]->fix_edge[0];
			if (lf[i]->fix_edge[0].empty()) {
				lf[i]->fix_edge[0].create(lf[i]->cpara.offset.rows - 1, lf[i]->cpara.offset.cols);
				lf[i]->fix_edge[0] = 0;
			}
			sprintf(name, "fixedge%d_1", i);
			fs[name] >> lf[i]->fix_edge[1];
			if (lf[i]->fix_edge[1].empty()) {
				lf[i]->fix_edge[1].create(lf[i]->cpara.offset.rows, lf[i]->cpara.offset.cols - 1);
				lf[i]->fix_edge[1] = 0;
			}
			sprintf(name, "name%d", i);
			fs[name] >> lf[i]->layer_name;
		}
		int dst_w;
		fs["dst_w"] >> dst_w;
		ri.set_dst_wide(dst_w);
		nails.clear();
		FileNode file_nails = fs["Nails"];
		for (FileNodeIterator it = file_nails.begin(); it != file_nails.end(); it++) {
			int l0 = (int)(*it)["l0"];
			int l1 = (int)(*it)["l1"];
			int p0x = (int)(*it)["p0x"];
			int p0y = (int)(*it)["p0y"];
			int p1x = (int)(*it)["p1x"];
			int p1y = (int)(*it)["p1y"];
			nails.push_back(Nail(lf[l0], lf[l1], Point(p0x, p0y), Point(p1x, p1y)));
		}		
		vector<double> slope = generate_mapxy();
        char text[300];
        for (int i = 0, j = 0; i < slope.size(); i++)
            j += sprintf(text + j, "k%d=%7f,", i, slope[i]);
        qInfo(text);
		Point ce0, ce1, ce2, ct;
		fs["layer"] >> layer;
		fs["scale"] >> scale;
		fs["center"] >> ct;
		fs["choose"] >> ce0;
		fs["choose1"] >> ce1;
		fs["choose2"] >> ce2;
		fs["draw_corner"] >> draw_corner;
		fs["xoffset"] >> xoffset;
		fs["yoffset"] >> yoffset;
		fs["xgrid_size"] >> xgrid_size;
		fs["ygrid_size"] >> ygrid_size;
		center = QPoint(ct.x, ct.y);
		choose[0] = QPoint(ce0.x, ce0.y);
		choose[1] = QPoint(ce1.x, ce1.y);
		choose[2] = QPoint(ce2.x, ce2.y);
		fs.release();
		feature_layer = -1;
		update();
		return 0;
	}
	return -1;
}
