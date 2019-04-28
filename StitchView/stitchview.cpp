#include "stitchview.h"
#include "iclayer.h"
#include <QPainter>
#include <algorithm>
#include <QtCore/QSharedPointer>
#include <fstream>
#include <QMessageBox>
#include <QDateTime>
#include <QApplication>
#include <QScopedPointer>

const int step_para = 3;
const int max_scale = 8;

#define PRINT_DRAW_IMAGE	0
#define IMAGE_OUTPUT_QUALITY 85
enum MouseState {
	IDLE,
	PutFirstNail,
	PutSecondNail,
	ChangeNail
};

string thread_generate_diff(FeatExt * feature, string project_path, int layer)
{
	feature->generate_feature_diff();
	QDateTime current = QDateTime::currentDateTime();
	string filename = current.toString("yy_MM_dd-hh.mm.ss.zzz-l").toStdString();
	char l = '0' + layer;
	filename = filename + l;
	filename = filename + ".xml";
	filename = project_path + "/WorkData/" + filename;
	qInfo("write to %s", filename.c_str());
	feature->write_diff_file(filename);
	return filename;
}

void thread_bundle_adjust(BundleAdjustInf * ba, FeatExt * feature, Mat_<Vec2i> *offset, Mat_<Vec2i> * c_info, vector<FixEdge> * fe, Size s, int option)
{
	ba->arrange(*feature, -1, -1, fe, s, option);
	*offset = ba->get_best_offset();
	Mat_<unsigned long long> corner = ba->get_corner();
	c_info->create(corner.rows, corner.cols);
	for (int y = 0; y < corner.rows; y++)
	for (int x = 0; x < corner.cols; x++) {
		unsigned long long c = corner(y, x);
		(*c_info)(y, x) = Vec2i(c & 0xffffffff, c >> 32);
	}
}

StitchView::StitchView(QWidget *parent) : QWidget(parent)
{
    scale = 8;
	center = QPoint(0, 0);
    layer = ABS_LAYER;
	edge_cost = 0;
	minloc_shift = Point(0, 0);
	feature_layer = -1;
	draw_corner = 0;
	for (int i=0; i<3; i++)
		choose[i] = QPoint(-1, -1);
    resize(1800, 1000);
	ri = new RenderImage();
	ri->set_dst_wide(1024);
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
	choose_edge = 0;
	cur_nail = Nail();
	
	ba = BundleAdjustInf::create_instance(2);
}

StitchView::~StitchView()
{
	for (int i = 0; i < lf.size(); i++)
		delete lf[i];
	delete ba;
}

void StitchView::paintEvent(QPaintEvent *)
{
	Q_ASSERT((layer < (int) lf.size() && layer >= 0) || layer == ABS_LAYER);
    QPoint ce(size().width()*scale/2, size().height()*scale/2);
    QSize s(size().width()*scale, size().height()*scale);
    view_rect = QRect(center - ce, s);	
	if (layer != ABS_LAYER) {
		Point bd = ri->src2dst(layer, Point(lf[layer]->cpara.right_bound(), lf[layer]->cpara.bottom_bound()));
		if (view_rect.width() > bd.x) {
			view_rect.adjust(view_rect.width() - bd.x, 0, 0, 0);
			center.setX(view_rect.center().x());
		}
		if (view_rect.height() > bd.y) {
			view_rect.adjust(0, view_rect.height() - bd.y, 0, 0);
			center.setY(view_rect.center().y());
		}

		if (view_rect.right() > bd.x - 1) {
			view_rect.moveRight(bd.x - 1);
			center.setX(view_rect.center().x());
		}

		if (view_rect.bottom() > bd.y - 1) {
			view_rect.moveBottom(bd.y - 1);
			center.setY(view_rect.center().y());
		}
		Q_ASSERT(view_rect.right() < bd.x && view_rect.bottom() < bd.y);
	}
	if (view_rect.left() < 0) {
		view_rect.moveLeft(0);
		center.setX(view_rect.center().x());
	}
	if (view_rect.top() < 0) {
		view_rect.moveTop(0);
		center.setY(view_rect.center().y());
	}

	Q_ASSERT(view_rect.left() >= 0 && view_rect.top() >= 0);

    QImage image(size(), QImage::Format_RGB32);
	image.fill(QColor(0, 0, 0));
    QPainter painter(&image);

	if (layer != ABS_LAYER) {
		painter.setPen(QPen(Qt::green));
		painter.setBrush(QBrush(Qt::green));
		QRect screen_rect(0, 0, width(), height());

		//1 Draw image
		//first loop, prepare for renderimage
		int dst_w = ri->get_dst_wide();
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
		ri->render_img(map_id, imgs, draw_order);

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
#if PRINT_DRAW_IMAGE
			qDebug("drawImage, (y=%d,x=%d) (l=%d,t=%d,w=%d, h=%d) -> (l=%d,t=%d,w=%d,h=%d)", y, x,
				src_rect.left(), src_rect.top(), src_rect.width(), src_rect.height(),
				target_rect.left(), target_rect.top(), target_rect.width(), target_rect.height());
#endif
			img_idx++;
		}

		//2 draw edge
		Point src_lt = ri->dst2src(layer, TOPOINT(view_rect.topLeft())); //dst_lt map to src_lt, src_lt is not src left top.
		Point src_rb = ri->dst2src(layer, TOPOINT(view_rect.bottomRight()));
		Point src_lb = ri->dst2src(layer, TOPOINT(view_rect.bottomLeft()));
		Point src_rt = ri->dst2src(layer, TOPOINT(view_rect.topRight()));

		double minx = min(min(src_lt.x, src_rb.x), min(src_lb.x, src_rt.x));
		double miny = min(min(src_lt.y, src_rb.y), min(src_lb.y, src_rt.y));
		double maxx = max(max(src_lt.x, src_rb.x), max(src_lb.x, src_rt.x));
		double maxy = max(max(src_lt.y, src_rb.y), max(src_lb.y, src_rt.y));
		//now Point(minx,miny) is src left top, Point (max, maxy) is src right bottom

		Point lt = find_src_map(lf[layer]->cpara, Point(minx, miny), ri->get_src_img_size(layer), 1);
		Point rb = find_src_map(lf[layer]->cpara, Point(maxx + 1, maxy + 1), ri->get_src_img_size(layer), 0);
		//now lt is src left top map, rb is src right bottom map
		//following code is same as below Key_E
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
			Point src_edge_center[2] = { src_corner + src_corner2, src_corner + src_corner1 };
			for (int i = 0; i < 2; i++) {
				src_edge_center[i].x = src_edge_center[i].x / 2;
				src_edge_center[i].y = src_edge_center[i].y / 2;
				QPoint dst_edge_center = TOQPOINT(ri->src2dst(layer, src_edge_center[i]));
				if (view_rect.contains(dst_edge_center)) {
					dst_edge_center = (dst_edge_center - view_rect.topLeft()) / scale;
					int fe = 0;
					if (y > 0 && i == 0)
						fe = lf[layer]->flagb[i](y - 1, x);
					if (x > 0 && i == 1)
						fe = lf[layer]->flagb[i](y, x - 1);
					//draw fix edge
					if (fe) {
						if (FIX_EDGE_IDEA_POS(fe))
							painter.setPen(QPen(Qt::yellow, 1));
						else
						if (FIX_EDGE_BINDY(fe))
							painter.setPen(QPen(Qt::red, 1));
						else
							painter.setPen(QPen(Qt::green, 1));
						painter.drawLine(dst_edge_center - QPoint(0, 5), dst_edge_center + QPoint(0, 5));
						if (FIX_EDGE_IDEA_POS(fe))
							painter.setPen(QPen(Qt::yellow, 1));
						else
						if (FIX_EDGE_BINDX(fe))
							painter.setPen(QPen(Qt::red, 1));
						else
							painter.setPen(QPen(Qt::green, 1));
						painter.drawLine(dst_edge_center - QPoint(5, 0), dst_edge_center + QPoint(5, 0));
					}
					if (draw_corner) {
						const EdgeDiff * ed = (i == 0) ? lf[layer]->feature.get_edge(0, y - 1, x) :
							lf[layer]->feature.get_edge(1, y, x - 1);
						if (ed && ed->img_num > 0) {
							Point src_corner3 = (i == 0) ? Point(lf[layer]->cpara.offset(y - 1, x)[1], lf[layer]->cpara.offset(y - 1, x)[0]) :
								Point(lf[layer]->cpara.offset(y, x - 1)[1], lf[layer]->cpara.offset(y, x - 1)[0]);
							Point oo = src_corner - src_corner3;
							Point idea_pos = (FIX_EDGE_IDEA_POS(fe)) ? 
								ed->offset + Point(FIX_EDGE_IDEA_DX(fe), FIX_EDGE_IDEA_DY(fe)) * lf[layer]->cpara.rescale :
								ed->offset + ed->minloc * lf[layer]->cpara.rescale;
							Point shift = oo - idea_pos;
							int cost = ed->get_dif(oo, lf[layer]->cpara.rescale) - ed->mind;
							int val_x = FIX_EDGE_BINDX(fe) ? 0 : shift.x;
							int val_y = FIX_EDGE_BINDY(fe) ? 0 : shift.y;
							char val[50];
							Point t = oo - ed->offset;
							t.y = t.y / lf[layer]->cpara.rescale;
							t.x = t.x / lf[layer]->cpara.rescale;
							if (t.y < 0 || t.x < 0 || t.y >= ed->dif.rows || t.x >= ed->dif.cols)
								painter.setPen(QPen(Qt::red, 1));
							else
								painter.setPen(QPen(Qt::green, 1));
							sprintf(val, "%d,%d", val_x, val_y);
							if (i == 1)
								painter.drawText(dst_edge_center + QPoint(-10, -30), val);
							else
								painter.drawText(dst_edge_center + QPoint(-60, -8), val);
						}
					}
				}
			}
		}

		//3 Draw corner
		painter.setPen(QPen(Qt::yellow, 1));
		if (draw_corner) {
			char val[50];
			for (int y = lt.y; y <= rb.y; y++)
			for (int x = lt.x; x <= rb.x; x++) {
				Point src_corner(lf[layer]->cpara.offset(y, x)[1], lf[layer]->cpara.offset(y, x)[0]);
				QPoint dst_corner = TOQPOINT(ri->src2dst(layer, src_corner));
				if (view_rect.contains(dst_corner)) {
					if (lf[layer]->corner_info.empty()) {
						painter.setPen(QPen(Qt::green));
						painter.setBrush(QBrush(Qt::green));
						painter.drawEllipse((dst_corner - view_rect.topLeft()) / scale, 3, 3);
					}
					else {
						unsigned long long info = lf[layer]->corner_info(y, x)[1];
						info = info << 32 | lf[layer]->corner_info(y, x)[0];
						short val_x = info & 0xffff;
						short val_y = (info >> 16) & 0xffff;
						sprintf(val, "%d,%d", val_x, val_y);
						QPoint center = (dst_corner - view_rect.topLeft()) / scale;
						painter.drawText(center - QPoint(10, 5), val);
					}
				}
			}
		}
	}
	//4 draw grid
	if (draw_grid) {
		int width = size().width();
		int height = size().height();
		if (ygrid_size >= 2)
		for (int i = (view_rect.top() - yoffset) / ygrid_size + 1; i <= (view_rect.bottom() - yoffset) / ygrid_size; i++) {
			int y = (i * ygrid_size + yoffset - view_rect.top()) / scale;
			switch (i & 3) {
			case 0:
				painter.setPen(QPen(Qt::red, 1, Qt::DotLine));
				break;
			case 1:
				painter.setPen(QPen(Qt::yellow, 1, Qt::DotLine));
				break;
			case 2:
				painter.setPen(QPen(Qt::blue, 1, Qt::DotLine));
				break;
			case 3:
				painter.setPen(QPen(Qt::green, 1, Qt::DotLine));
				break;
			}
			painter.drawLine(0, y, width, y);
		}
		if (xgrid_size >= 2)
		for (int i = (view_rect.left() - xoffset) / xgrid_size + 1; i <= (view_rect.right() - xoffset) / xgrid_size; i++) {
			int x = (i * xgrid_size + xoffset - view_rect.left()) / scale;
			switch (i & 3) {
			case 0:
				painter.setPen(QPen(Qt::red, 1, Qt::DotLine));
				break;
			case 1:
				painter.setPen(QPen(Qt::yellow, 1, Qt::DotLine));
				break;
			case 2:
				painter.setPen(QPen(Qt::blue, 1, Qt::DotLine));
				break;
			case 3:
				painter.setPen(QPen(Qt::green, 1, Qt::DotLine));
				break;
			}
			painter.drawLine(x, 0, x, height);
		}
	}

	//5 draw nail
	if (layer == ABS_LAYER) {
		vector<Nail> ns;
		painter.setBrush(Qt::NoBrush);
		painter.setPen(Qt::red);
		get_absolute_nails(ns);
		for (int i = 0; i < (int)ns.size(); i++) {
			QPoint ns_point = TOQPOINT(ns[i].p1);
			if (view_rect.contains(ns_point)) {
				ns_point = (ns_point - view_rect.topLeft()) / scale;
				painter.drawLine(ns_point + QPoint(-6, -6), ns_point + QPoint(6, 6));
				painter.drawLine(ns_point + QPoint(-6, 6), ns_point + QPoint(6, -6));
			}
		}
	}
	else {
		vector<Nail> ns;
		painter.setBrush(Qt::NoBrush);
		get_one_layer_nails(lf[layer], ns);
		for (int i = 0; i < (int)ns.size(); i++) {
			QPoint ns_point = TOQPOINT(ri->src2dst(layer, ns[i].p0));
			if (view_rect.contains(ns_point)) {
				if (layer > 0 && lf[layer - 1] == ns[i].lf1 || ns[i].lf0 == ns[i].lf1)
					painter.setPen(Qt::blue);
				else
					painter.setPen(Qt::red);
				ns_point = (ns_point - view_rect.topLeft()) / scale;
				painter.drawLine(ns_point + QPoint(-6, -6), ns_point + QPoint(6, 6));
				painter.drawLine(ns_point + QPoint(-6, 6), ns_point + QPoint(6, -6));
			}
		}
		if (cur_nail.lf0 == lf[layer]) {
			painter.setPen(Qt::red);
			QPoint ns_point = TOQPOINT(ri->src2dst(layer, cur_nail.p0));
			if (view_rect.contains(ns_point)) {
				ns_point = (ns_point - view_rect.topLeft()) / scale;
				painter.drawLine(ns_point + QPoint(-6, -6), ns_point + QPoint(6, 6));
				painter.drawLine(ns_point + QPoint(-6, 6), ns_point + QPoint(6, -6));
			}
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
	int prev_layer = layer;
	QPoint may_choose = point2choose(cur_mouse_point);
	int step;
	qInfo("StitchView press key %x, modifier=%x", e->key(), QApplication::queryKeyboardModifiers());
	switch (e->key()) {
	case Qt::Key_Left:		
        if (QApplication::queryKeyboardModifiers() == Qt::ControlModifier && layer != ABS_LAYER &&
                abs(may_choose.x()) < 10000000) {
			lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1] -= lf[layer]->cpara.rescale;
			/*
			//check image offset
			if (lf[layer]->check_img_offset(MAKE_IMG_IDX(may_choose.x(), may_choose.y())))
				lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1] += lf[layer]->cpara.rescale;
			else
			*/
				ri->invalidate_cache(layer);
		}
		else 
        if (QApplication::queryKeyboardModifiers() == Qt::ShiftModifier && layer != ABS_LAYER  &&
                abs(may_choose.x()) < 10000000) {
			vector <unsigned> imgs;
			imgs = lf[layer]->get_fix_img(MAKE_IMG_IDX(may_choose.x(), may_choose.y()), BIND_X_MASK | IDEA_POS_MASK);
			for (int i = 0; i < (int) imgs.size(); i++)
				lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[1] -= lf[layer]->cpara.rescale;
			ri->invalidate_cache(layer);
		}
		else {
			step = view_rect.width() * step_para / 10;
			center += QPoint(-step, 0);
		}
		break;
	case Qt::Key_Up:		
        if (QApplication::queryKeyboardModifiers() == Qt::ControlModifier && layer != ABS_LAYER &&
                abs(may_choose.x()) < 10000000) {
			lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[0] -= lf[layer]->cpara.rescale;
				ri->invalidate_cache(layer);
		}
		else
        if (QApplication::queryKeyboardModifiers() == Qt::ShiftModifier && layer != ABS_LAYER &&
                abs(may_choose.x()) < 10000000) {
			vector <unsigned> imgs;
			imgs = lf[layer]->get_fix_img(MAKE_IMG_IDX(may_choose.x(), may_choose.y()), BIND_Y_MASK | IDEA_POS_MASK);
			for (int i = 0; i < (int)imgs.size(); i++)
				lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[0] -= lf[layer]->cpara.rescale;
			ri->invalidate_cache(layer);
		}
		else {
			step = view_rect.height() * step_para / 10;
			center += QPoint(0, -step);
		}
		break;
	case Qt::Key_Right:
        if (QApplication::queryKeyboardModifiers() == Qt::ControlModifier && layer != ABS_LAYER &&
                abs(may_choose.x()) < 10000000) {
			lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1] += lf[layer]->cpara.rescale;
				ri->invalidate_cache(layer);
		}
		else
        if (QApplication::queryKeyboardModifiers() == Qt::ShiftModifier && layer != ABS_LAYER &&
                abs(may_choose.x()) < 10000000) {
			vector <unsigned> imgs;
			imgs = lf[layer]->get_fix_img(MAKE_IMG_IDX(may_choose.x(), may_choose.y()), BIND_X_MASK | IDEA_POS_MASK);
			for (int i = 0; i < (int)imgs.size(); i++)
				lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[1] += lf[layer]->cpara.rescale;
			ri->invalidate_cache(layer);
		}
		else {
			step = view_rect.width() * step_para / 10;
			center += QPoint(step, 0);
		}
		break;
	case Qt::Key_Down:		
        if (QApplication::queryKeyboardModifiers() == Qt::ControlModifier && layer != ABS_LAYER &&
                abs(may_choose.x()) < 10000000) {
			lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[0] += lf[layer]->cpara.rescale;
				ri->invalidate_cache(layer);
		}
		else
        if (QApplication::queryKeyboardModifiers() == Qt::ShiftModifier && layer != ABS_LAYER &&
                abs(may_choose.x()) < 10000000) {
			vector <unsigned> imgs;
			imgs = lf[layer]->get_fix_img(MAKE_IMG_IDX(may_choose.x(), may_choose.y()), BIND_Y_MASK | IDEA_POS_MASK);
			for (int i = 0; i < (int)imgs.size(); i++)
				lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[0] += lf[layer]->cpara.rescale;
			ri->invalidate_cache(layer);
		}
		else {
			step = view_rect.height() * step_para / 10;
			center += QPoint(0, step);
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
	case Qt::Key_Z:
		if (QApplication::keyboardModifiers() == Qt::ShiftModifier) {
			if (scale < max_scale)
				scale = scale * 2;
		}
		else
		if (QApplication::keyboardModifiers() == Qt::ControlModifier) {
			if (scale > 0.4999)
				scale = scale / 2;
		}
		break;
	case Qt::Key_0:
		layer = ABS_LAYER;
		break;
	case Qt::Key_1:
		if (lf.size() > 0)
			layer = 0;
		break;
	case Qt::Key_2:
		if (lf.size() > 1)
			layer = 1;
		break;
	case Qt::Key_3:
		if (lf.size() > 2)
			layer = 2;
		break;
	case Qt::Key_4:
		if (lf.size() > 3)
			layer = 3;
		break;
	case Qt::Key_5:
		if (lf.size() > 4)
			layer = 4;
		break;
	case Qt::Key_6:
		if (lf.size() > 5)
			layer = 5;
		break;
	case Qt::Key_7:
		if (lf.size() > 6)
			layer = 6;
		break;
	case Qt::Key_8:
		if (lf.size() > 7)
			layer = 7;
		break;
	case Qt::Key_9:
		if (lf.size() > 8)
			layer = 8;
		break;
	case Qt::Key_A:
		if (lf.size() > 9)
			layer = 9;
		break;
	case Qt::Key_B:
		if (lf.size() > 10)
			layer = 10;
		break;
	case Qt::Key_C:
		if (lf.size() > 11)
			layer = 11;
		break;
	case Qt::Key_D:
		if (lf.size() > 12)
			layer = 12;
		break;
	case Qt::Key_E:
		if (lf.size() > 13)
			layer = 13;
		break;
	case Qt::Key_F:
		if (lf.size() > 14)
			layer = 14;
		break;
	case Qt::Key_Minus:
		if (layer > 0 && layer != ABS_LAYER)
			layer--;
		else
		if (layer == 0)
			layer = ABS_LAYER;
		break;
	case Qt::Key_Plus:
	case Qt::Key_Equal:
		if (layer == ABS_LAYER)
			layer = 0;
		else
		if (layer + 1 < lf.size())
			layer++;
		break;	
	case Qt::Key_F1:
		if (layer != ABS_LAYER)
			draw_corner = 1 - draw_corner;
		break; 
	case Qt::Key_F2: 
		if (layer != ABS_LAYER) {
			QScopedPointer<MapXY> mxy(get_mapxy(layer));
			int merge = mxy->get_merge_method();
			mxy->set_merge_method(!merge);
			ri->set_mapxy(layer, mxy.data());
		}
		break;
	case Qt::Key_F3: 
		if (layer != ABS_LAYER) {			
			Point loc(-1, -1);
			if (mouse_state == ChangeNail) {
				vector<Nail> ns;
				get_one_layer_nails(lf[layer], ns);
				if (ns.size() > 0) {
					lf[layer]->find_next = lf[layer]->find_next % ns.size();
					loc = ns[lf[layer]->find_next].p0;
					loc = ri->src2dst(layer, loc);
					lf[layer]->find_next++;
				}
			}
			else 
			if (draw_corner > 0) {
				ceview->goto_next_corner();
			}
			if (loc.x > 0)
				goto_xy(loc.x, loc.y);
		}
		else {
			Point loc(-1, -1);
			if (mouse_state == ChangeNail) {
				vector<Nail> ns;
				get_absolute_nails(ns);
				if (ns.size() > 0) {
					lf[0]->find_next = lf[0]->find_next % ns.size();
					loc = ns[lf[0]->find_next].p1;
					lf[0]->find_next++;
				}
			}
			if (loc.x > 0)
				goto_xy(loc.x, loc.y);
		}		
		break;
	case Qt::Key_F4:	
		if (layer != ABS_LAYER) {
			ceview->goto_next_edge();
		}
		break;
	case Qt::Key_F5:
		draw_grid = !draw_grid;
		break;

	case Qt::Key_Delete:
	case Qt::Key_Escape: {
			switch (mouse_state) {
			case PutSecondNail:
				generate_mapxy();
				if (layer != ABS_LAYER)
					notify_nail_info(layer);
			case PutFirstNail:
			case ChangeNail:
				cur_nail = Nail();
				mouse_state = IDLE;
				setCursor(Qt::ArrowCursor);
				break;			
			}
		}
		break;
	default:
		QWidget::keyPressEvent(e);
		return;
	}
	if (prev_layer != layer && layer != ABS_LAYER) {
		ceview->set_layer_info(lf[layer]);
		notify_nail_info(layer);
	}
	update_title();
	qDebug("Key=%d press, l=%d, s=%d, center=(%d,%d)", e->key(), layer, scale, center.x(), center.y());
	update();
	
	QPoint ce(size().width()*scale / 2, size().height()*scale / 2);
	QSize s(size().width()*scale, size().height()*scale);
	view_rect = QRect(center - ce, s);
	update_nview();
	self_check_offset(layer);
	char info[200];
	QPoint mouse_point = cur_mouse_point * scale + view_rect.topLeft();
	Point src_point = ri->dst2src(layer, TOPOINT(mouse_point));
	sprintf(info, "x=%d,y=%d,sx=%d,sy=%d,ix=%d,iy=%d,mx=%d,my=%d,c=%d",
		mouse_point.x(), mouse_point.y(), src_point.x, src_point.y,
		may_choose.x() + 1, may_choose.y() + 1, minloc_shift.x, minloc_shift.y, edge_cost);
	emit MouseChange(info);
}

void StitchView::mouseMoveEvent(QMouseEvent *event)
{
	setFocus();
	static Point oo;
	if (lf.empty())
		return;
	QPoint mouse_point(event->localPos().x(), event->localPos().y());
	QPoint prev_may_choose = point2choose(cur_mouse_point);
	cur_mouse_point = mouse_point;
	QPoint may_choose = point2choose(cur_mouse_point);

    if (abs(may_choose.x()) >= 10000000 || abs(prev_may_choose.x()) >= 10000000)
        return;

	mouse_point = mouse_point * scale + view_rect.topLeft();
	Point src_point = ri->dst2src(layer, TOPOINT(mouse_point));

	if (layer != ABS_LAYER && may_choose != prev_may_choose && lf[layer]->feature.is_valid()) {
		const EdgeDiff * ed = lf[layer]->feature.get_edge(may_choose.y(), may_choose.x(), prev_may_choose.y(), prev_may_choose.x());
		if (ed) {
			Point o0(lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[1],
				lf[layer]->cpara.offset(may_choose.y(), may_choose.x())[0]);
			Point o1(lf[layer]->cpara.offset(prev_may_choose.y(), prev_may_choose.x())[1],
				lf[layer]->cpara.offset(prev_may_choose.y(), prev_may_choose.x())[0]);
			oo = (o1.y + o1.x > o0.y + o0.x) ? o1 - o0 : o0 - o1;
			minloc_shift = oo - ed->offset - ed->minloc * lf[layer]->cpara.rescale;
			edge_cost = ed->get_dif(oo, lf[layer]->cpara.rescale) - ed->mind;
		}
	}
	char info[200];
    sprintf(info, "x=%d,y=%d,sx=%d,sy=%d,ix=%d,iy=%d,ox=%d,oy=%d,mx=%d,my=%d,c=%d", 
		mouse_point.x(), mouse_point.y(), src_point.x, src_point.y,
		may_choose.x() + 1, may_choose.y() + 1, oo.x, oo.y, minloc_shift.x, minloc_shift.y, edge_cost);
	emit MouseChange(info);
	QWidget::mouseMoveEvent(event);
}

void StitchView::mousePressEvent(QMouseEvent *event)
{
	setFocus();
	if (lf.empty())
		return;
	QPoint mouse_point(event->localPos().x(), event->localPos().y());
	cur_mouse_point = mouse_point;
	QPoint may_choose = point2choose(cur_mouse_point);
	mouse_point = mouse_point * scale + view_rect.topLeft();
    if (abs(may_choose.x()) >= 10000000)
        return;
	qInfo("receive mousePress, state=%d, (x=%d,y=%d)", mouse_state, may_choose.x(), may_choose.y());
	switch (mouse_state) {
	case ChangeNail:
	{
		if (layer == ABS_LAYER) {
			cur_nail = Nail();
			vector<Nail> ns;
			get_absolute_nails(ns);
			for (int i = 0; i < (int)ns.size(); i++) {
				if (abs(ns[i].p1.x - mouse_point.x()) < 25 && abs(ns[i].p1.y - mouse_point.y()) < 25)
					cur_nail = ns[i];
			}
		} else
			cur_nail = search_nail(lf[layer], ri->dst2src(layer, TOPOINT(mouse_point)), 25);
		if (cur_nail.lf0 != NULL) {
			del_nail(cur_nail);
			if (layer != ABS_LAYER && cur_nail.lf1 != lf[layer])
				cur_nail.swap_order();
			cur_nail.lf1 = NULL;
			mouse_state = PutSecondNail;
			setCursor(Qt::CrossCursor);
		}
		break;
	}
	case PutFirstNail: 
	{
		if (layer == ABS_LAYER) {
			QMessageBox::information(this, "Info", "click actual layer first");
			return;
		}			
		cur_nail.lf0 = lf[layer];
		cur_nail.p0 = ri->dst2src(layer, TOPOINT(mouse_point));
		mouse_state = PutSecondNail;
		break;
	}
	case PutSecondNail: 
	{
		if (layer == ABS_LAYER) 
			cur_nail.lf1 = cur_nail.lf0;
		else
			cur_nail.lf1 = lf[layer];
		if (cur_nail.lf0 == cur_nail.lf1)
			layer = ABS_LAYER;
		cur_nail.p1 = ri->dst2src(layer, TOPOINT(mouse_point));
		add_nail(cur_nail);
		cur_nail = Nail();
		mouse_state = IDLE;
		setCursor(Qt::ArrowCursor);
		break;
	}
	case IDLE: 
	if (layer != ABS_LAYER) {
		Point src_point = ri->dst2src(layer, TOPOINT(mouse_point));
		if (QApplication::keyboardModifiers() == Qt::ShiftModifier || QApplication::keyboardModifiers() == Qt::ControlModifier) {
			int y = may_choose.y(), x = may_choose.x();
			//1 find nearest edge to fix
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
			int choose = -1; //choose decide which edge
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
			if (QApplication::keyboardModifiers() == Qt::ShiftModifier) {
				unsigned edge_idx = 0xffffffff;
				unsigned new_fix;
				switch (choose) {
				case 0:
					if (x > 0) {
						new_fix = lf[layer]->flagb[1](y, x - 1);
						edge_idx = MAKE_EDGE_IDX(x - 1, y, 1);
						lf[layer]->flagb[1](y, x - 1) = 0;
					}
					break;
				case 1:
					if (y > 0) {
						new_fix = lf[layer]->flagb[0](y - 1, x);
						edge_idx = MAKE_EDGE_IDX(x, y - 1, 0);
						lf[layer]->flagb[0](y - 1, x) = 0;
					}
					break;
				case 2:
					new_fix = lf[layer]->flagb[0](y, x);
					edge_idx = MAKE_EDGE_IDX(x, y, 0);
					lf[layer]->flagb[0](y, x) = 0;
					break;
				case 3:
					new_fix = lf[layer]->flagb[1](y, x);
					edge_idx = MAKE_EDGE_IDX(x, y, 1);
					lf[layer]->flagb[1](y, x) = 0;
					break;
				}
				if (edge_idx == 0xffffffff)
					break;
				//now nearest edge is alrady found
				if (new_fix == 0) {
					const EdgeDiff * ed = lf[layer]->feature.get_edge(edge_idx);
					unsigned i0, i1;
					ed->get_img_idx(i0, i1);
					Point img0(lf[layer]->cpara.offset(IMG_Y(i0), IMG_X(i0))[1], lf[layer]->cpara.offset(IMG_Y(i0), IMG_X(i0))[0]);
					Point img1(lf[layer]->cpara.offset(IMG_Y(i1), IMG_X(i1))[1], lf[layer]->cpara.offset(IMG_Y(i1), IMG_X(i1))[0]);
					Point shift = img1 - img0;
					shift -= ed->offset;
					if (shift.x >= 0 && shift.y >= 0 && shift.x / lf[layer]->cpara.rescale < ed->dif.cols 
						&& shift.y / lf[layer]->cpara.rescale < ed->dif.rows)
						new_fix = MAKE_FIX_EDGE(0, lf[layer]->cpara.rescale, 1, shift.x / lf[layer]->cpara.rescale, shift.y / lf[layer]->cpara.rescale);
					else
						new_fix = MAKE_FIX_EDGE(0, lf[layer]->cpara.rescale, 0, 0, 0);
				}
				else {
					if (FIX_EDGE_IDEA_POS(new_fix))
						new_fix = MAKE_FIX_EDGE(0, lf[layer]->cpara.rescale, 0, 0, 0);
					else 
					if (!FIX_EDGE_BINDX(new_fix) || !FIX_EDGE_BINDY(new_fix))
						new_fix++;
					else
						new_fix = 0;
					
				}
				qInfo("change fix_edge, l=%d, e=%d, x=%d, y=%d to %d", layer, EDGE_E(edge_idx), EDGE_X(edge_idx), EDGE_Y(edge_idx), new_fix);
				lf[layer]->flagb[EDGE_E(edge_idx)](EDGE_Y(edge_idx), EDGE_X(edge_idx)) = new_fix;
			}
			else { //ControlModifier
				const EdgeDiff * ed;
				switch (choose) {
				case 0:
					ed = lf[layer]->feature.get_edge(1, y, x - 1);
					break;
				case 1:
					ed = lf[layer]->feature.get_edge(0, y - 1, x);
					break;
				case 2:
					ed = lf[layer]->feature.get_edge(0, y, x);
					break;
				case 3:
					ed = lf[layer]->feature.get_edge(1, y, x);
					break;
				}
				if (ed == NULL)
					return;
				unsigned i0, i1;
				ed->get_img_idx(i0, i1);
				vector <unsigned> imgs;
				imgs = lf[layer]->get_fix_img(i1, BIND_X_MASK | BIND_Y_MASK | IDEA_POS_MASK);
				if (std::find(imgs.begin(), imgs.end(), i0) != imgs.end())
					return;
				Point img0(lf[layer]->cpara.offset(IMG_Y(i0), IMG_X(i0))[1], lf[layer]->cpara.offset(IMG_Y(i0), IMG_X(i0))[0]);
				Point img1(lf[layer]->cpara.offset(IMG_Y(i1), IMG_X(i1))[1], lf[layer]->cpara.offset(IMG_Y(i1), IMG_X(i1))[0]);
				Point shift = img1 - img0;
				Point bestshift = ed->offset + ed->minloc * lf[layer]->cpara.rescale;
				for (int i = 0; i < (int)imgs.size(); i++) {
					lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[1] += (bestshift - shift).x;
					lf[layer]->cpara.offset(IMG_Y(imgs[i]), IMG_X(imgs[i]))[0] += (bestshift - shift).y;
				}
				ri->invalidate_cache(layer);
			}
		}
		else
		{
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

void StitchView::mouseDoubleClickEvent(QMouseEvent * event)
{
	setFocus();
	QWidget::mouseDoubleClickEvent(event);
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
			update_title();
		} else
			emit notify_progress(computing_feature.get_progress());
	}
}

void StitchView::update_nview()
{
	vector<Point> nsp;
	vector<int> ns_state;
	vector<Nail> ns;
	if (layer == ABS_LAYER) {
		get_absolute_nails(ns);
		for (int i = 0; i < (int)ns.size(); i++) {
			nsp.push_back(ns[i].p1);
			ns_state.push_back(0);
		}
	}
	else {
		Point bd = ri->src2dst(layer, Point(lf[layer]->cpara.right_bound(), lf[layer]->cpara.bottom_bound()));
		nview->set_boundary(bd.x, bd.y);
		get_one_layer_nails(lf[layer], ns);
		for (int i = 0; i < (int)ns.size(); i++) {
			nsp.push_back(ri->src2dst(layer, ns[i].p0));
			if (layer > 0 && lf[layer - 1] == ns[i].lf1 || ns[i].lf0 == ns[i].lf1)
				ns_state.push_back(1);
			else
				ns_state.push_back(0);
		}
	}
	nview->set_nails(nsp, ns_state);
	QPoint ce(size().width()*scale / 2, size().height()*scale / 2);
	QSize s(size().width()*scale, size().height()*scale);
	QRect view_rect = QRect(center - ce, s);
	nview->set_viewrect(view_rect);	
}

/*
method = 0, means convert nsrc to nimg
method = 1, means convert nimg to nsrc
nabs is  31..16   15..0
        img_idx point_offset
*/
void StitchView::convert_nails(vector<Nail> & nsrc, vector<Nail> & nimg, int method)
{
	if (method == 0) {
        nimg = nsrc;
		for (int i = 0; i < (int)nsrc.size(); i++) {
            int l0 = which_layer(nimg[i].lf0);
            Point idx0 = find_src_map(nimg[i].lf0->cpara, nimg[i].p0, ri->get_src_img_size(l0), 0);
			if (idx0.x >= 32768 || idx0.y >= 32768 || idx0.x < 0 || idx0.y < 0)
                nimg[i].p0 = -nimg[i].p0;
			else {
                Point offset0(nimg[i].lf0->cpara.offset(idx0.y, idx0.x)[1], nimg[i].lf0->cpara.offset(idx0.y, idx0.x)[0]);
                offset0 = nimg[i].p0 - offset0;
				CV_Assert(offset0.x >= 0 && offset0.y >= 0);
                nimg[i].p0 = idx0 * 65536 + offset0;
			}
			if (nimg[i].lf0 == nimg[i].lf1) //absolute nail, not modify p1
				continue;
            int l1 = which_layer(nimg[i].lf1);
            Point idx1 = find_src_map(nimg[i].lf1->cpara, nimg[i].p1, ri->get_src_img_size(l1), 0);
			if (idx1.x >= 32768 || idx1.y >= 32768 || idx1.x < 0 || idx1.y < 0)
                nimg[i].p1 = -nimg[i].p1;
			else {
                Point offset1(nimg[i].lf1->cpara.offset(idx1.y, idx1.x)[1], nimg[i].lf1->cpara.offset(idx1.y, idx1.x)[0]);
                offset1 = nimg[i].p1 - offset1;
				CV_Assert(offset1.x >= 0 && offset1.y >= 0);
                nimg[i].p1 = idx1 * 65536 + offset1;
			}
		}
	}
	else {
        nsrc = nimg;
		for (int i = 0; i < (int)nsrc.size(); i++) {
			if (nsrc[i].p0.x < 0 || nsrc[i].p0.y < 0)
				nsrc[i].p0 = -nsrc[i].p0;
			else {
				Point idx0(nsrc[i].p0.x / 65536, nsrc[i].p0.y / 65536);
				Point offset0(nsrc[i].p0.x % 65536, nsrc[i].p0.y % 65536);
				nsrc[i].p0 = Point(nsrc[i].lf0->cpara.offset(idx0.y, idx0.x)[1], nsrc[i].lf0->cpara.offset(idx0.y, idx0.x)[0]);
				nsrc[i].p0 += offset0;
			}
			if (nsrc[i].lf0 == nsrc[i].lf1) //absolute nail, not modify p1
				continue;
			if (nsrc[i].p1.x < 0 || nsrc[i].p1.y < 0)
				nsrc[i].p1 = -nsrc[i].p1;
			else {
				Point idx1(nsrc[i].p1.x / 65536, nsrc[i].p1.y / 65536);
				Point offset1(nsrc[i].p1.x % 65536, nsrc[i].p1.y % 65536);
				nsrc[i].p1 = Point(nsrc[i].lf1->cpara.offset(idx1.y, idx1.x)[1], nsrc[i].lf1->cpara.offset(idx1.y, idx1.x)[0]);
				nsrc[i].p1 += offset1;
			}
		}
	}
}

void StitchView::update_center(QPoint center)
{
	if (lf.empty())
		return;
	goto_xy(center.x(), center.y());
}

void StitchView::goto_corner(unsigned corner_idx)
{
	int y = CORNER_Y(corner_idx);
	int x = CORNER_X(corner_idx);
	Point src_corner(lf[layer]->cpara.offset(y, x)[1], lf[layer]->cpara.offset(y, x)[0]);
	Point loc = ri->src2dst(layer, src_corner);
	goto_xy(loc.x, loc.y);
}

void StitchView::goto_edge(unsigned edge_idx)
{
	int y = EDGE_Y(edge_idx);
	int x = EDGE_X(edge_idx);
	if (EDGE_E(edge_idx))
		x++;
	else
		y++;
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
	Point src_edge_center[2] = { src_corner + src_corner2, src_corner + src_corner1 };
	int e = EDGE_E(edge_idx);
	Point loc = ri->src2dst(layer, Point(src_edge_center[e].x / 2, src_edge_center[e].y / 2));
	goto_xy(loc.x, loc.y);
}

void StitchView::goto_nail(unsigned nail_idx)
{
	vector<Nail> ns;
	get_one_layer_nails(lf[layer], ns);
	if (ns.size() > nail_idx) {
		Point loc = ri->src2dst(layer, ns[nail_idx].p0);
		goto_xy(loc.x, loc.y);
	}
}

bool StitchView::add_nail(Nail nail)
{
	nails.push_back(nail);
	vector<double> slope =generate_mapxy();
	char a[300];
	for (int i = 0, j=0; i < slope.size(); i++)
		j += sprintf(a + j, "k%d=%7f,", i, slope[i]);	
	QMessageBox::information(this, "Add nail slope", QLatin1String(a));
	update_nview();
	if (layer != ABS_LAYER)
		notify_nail_info(layer);
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

void StitchView::get_absolute_nails(vector<Nail> &ns)
{
	ns.clear();
	for (int i = 0; i < (int)nails.size(); i++) 
	if (nails[i].lf0 == nails[i].lf1)
		ns.push_back(nails[i]);
}

void StitchView::get_one_layer_nails(LayerFeature * lf, vector<Nail> &ns)
{
	ns.clear();
    vector<Nail> n0, n1, n2;
    int l0 = which_layer(lf);
    for (int i = 0; i < (int)nails.size(); i++) {
        Nail n;
        int l1;
        if (nails[i].lf0 == lf) {
            n = nails[i];
            l1 = which_layer(nails[i].lf1);
        }
		else
		if (nails[i].lf1 == lf) {
			n.lf0 = nails[i].lf1;
			n.lf1 = nails[i].lf0;
			n.p0 = nails[i].p1;
			n.p1 = nails[i].p0;
            l1 = which_layer(nails[i].lf0);
        }
        if (n.lf0) {
            if (l0 == l1)
                n0.push_back(n);
            if (l0 > l1)
                n1.push_back(n);
            if (l0 < l1)
                n2.push_back(n);
        }
	}
    ns.insert(ns.end(), n0.begin(), n0.end());
    ns.insert(ns.end(), n1.begin(), n1.end());
    ns.insert(ns.end(), n2.begin(), n2.end());
}

void StitchView::del_one_layer_nails(LayerFeature * lf)
{
	for (int i = 0; i < (int)nails.size(); i++)
	if (nails[i].lf0 == lf || nails[i].lf1 == lf) {
		nails[i] = nails.back();
		nails.pop_back();
        i--;
	}	
}

void StitchView::notify_nail_info(int layer)
{
	vector<Nail> ns;
	get_one_layer_nails(lf[layer], ns);
	vector<Point2f> nail;
	vector<Point> bias;
	vector<int> dir;
	for (int i = 0; i < (int)ns.size(); i++) {
		Point2d nxy = ri->src2dst(layer, ns[i].p0);
		Point2d nxy1;
		if (layer > 0 && lf[layer - 1] == ns[i].lf1) {
			dir.push_back(1);
			nxy1 = ri->src2dst(layer - 1, ns[i].p1);
		}
		else
		if (ns[i].lf0 == ns[i].lf1) {
			dir.push_back(2);
			nxy1 = ns[i].p1;
		}
		else {
			int l = which_layer(ns[i].lf1);
			dir.push_back(0);
			nxy1 = ri->src2dst(l, ns[i].p1);
		}
		bias.push_back(nxy1 - nxy);
		Point2f nxy0;
		nxy0.x = (double) ns[i].p0.x / lf[layer]->cpara.right_bound();
		nxy0.y = (double) ns[i].p0.y / lf[layer]->cpara.bottom_bound();
		nail.push_back(nxy0);
	}
	ceview->set_nail_info(nail, bias, dir);
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
	qInfo("generate_mapxy, total_nail=%d", (int) nails.size());
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
		QScopedPointer<MapXY> mxy(get_mapxy(choose));
		slope.push_back(mxy->recompute(abs_nails));		
		ri->set_mapxy(choose, mxy.data());
		//3 change nail with choose layer from src to dst
		for (int i = 0; i < (int)layer_nails.size(); i++) 
		if (finish_map[i] >= 0) {
			for (int j = 0; j < (int)layer_nails[i].size(); j++)
			if (layer_nails[i][j].lf1 == lf[choose]) {
				layer_nails[i][j].lf1 = layer_nails[i][j].lf0; //make it absolute
				layer_nails[i][j].p1 = mxy->src2dst(layer_nails[i][j].p1);
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

void StitchView::self_check_offset(int _layer)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer == ABS_LAYER)
		return;
	for (int y = 0; y < lf[_layer]->cpara.offset.rows; y++)
	for (int x = 0; x < lf[_layer]->cpara.offset.cols; x++)
	if (lf[_layer]->cpara.offset(y, x)[0] % lf[_layer]->cpara.rescale != 0 ||
		lf[_layer]->cpara.offset(y, x)[1] % lf[_layer]->cpara.rescale != 0) {
		qCritical("offset[%d, %d] s=%d, (y=%d, x=%d) wrong!", y, x, lf[_layer]->cpara.rescale, 
			lf[_layer]->cpara.offset(y, x)[0], lf[_layer]->cpara.offset(y, x)[1]);
		CV_Assert(0);
	}
}

//if _layer==-1, means current layer
int StitchView::set_config_para(int _layer, const ConfigPara & _cpara)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer > lf.size() || _layer < 0)
		return -1;
	if (lf.empty()) {
		int loc = (int) _cpara.img_path.find_last_of("\\/");
		int loc2 = (int) _cpara.img_path.find_last_of("\\/", loc - 1);
		project_path = _cpara.img_path.substr(0, loc2);
		QFile file(QString::fromStdString(project_path));
		if (!file.exists()) {
			QMessageBox::information(this, "Info", QString::fromStdString(project_path + " not exist"));
			return -2;
		}
		QDir work_dir(QString::fromStdString(project_path + "/WorkData"));
		if (!work_dir.exists()) {
			bool ret = work_dir.mkdir(QString::fromStdString(project_path + "/WorkData"));
			if (!ret) {
				qFatal("Unable to create work dir %s", work_dir.absolutePath().toStdString().c_str());
				return -1;
			}
		}
	}
	convert_nails(nails, nails, 0);
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
		layer_feature->cpara.offset = _cpara.offset.clone();
		layer_feature->flagb[0].create(_cpara.offset.rows - 1, _cpara.offset.cols);
		layer_feature->flagb[0] = 0;
		layer_feature->flagb[1].create(_cpara.offset.rows, _cpara.offset.cols - 1);
		layer_feature->flagb[1] = 0;
		lf.push_back(layer_feature);
		self_check_offset(lf.size() - 1);
	}
	else {
		if (lf[_layer]->cpara.img_path == _cpara.img_path && lf[_layer]->cpara.img_num_w == _cpara.img_num_w &&
			lf[_layer]->cpara.img_num_h == _cpara.img_num_h) {
			if (lf[_layer]->cpara.rescale > _cpara.rescale) { //only inherit free-move fix edge, call in prepare_next_iter 
				for (int y = 0; y < _cpara.offset.rows - 1; y++)
				for (int x = 0; x < _cpara.offset.cols; x++) 
				if (FIX_EDGE_BINDX(lf[_layer]->flagb[0](y, x)) || FIX_EDGE_BINDY(lf[_layer]->flagb[0](y, x)))
					lf[_layer]->flagb[0](y, x) = 0;
				for (int y = 0; y < _cpara.offset.rows; y++)
				for (int x = 0; x < _cpara.offset.cols - 1; x++)
				if (FIX_EDGE_BINDX(lf[_layer]->flagb[1](y, x)) || FIX_EDGE_BINDY(lf[_layer]->flagb[1](y, x)))
					lf[_layer]->flagb[1](y, x) = 0;
			}
			lf[_layer]->cpara = _cpara;
			lf[_layer]->cpara.offset = _cpara.offset.clone();
		}
		else {
			lf[_layer]->cpara = _cpara;
			lf[_layer]->cpara.offset = _cpara.offset.clone();
			lf[_layer]->flagb[0].create(_cpara.offset.rows - 1, _cpara.offset.cols);
			lf[_layer]->flagb[0] = 0;
			lf[_layer]->flagb[1].create(_cpara.offset.rows, _cpara.offset.cols - 1);
			lf[_layer]->flagb[1] = 0;
		}
		self_check_offset(_layer);
	}
	ri->set_cfg_para(_layer, &lf[_layer]->cpara);
	convert_nails(nails, nails, 1);
	if (_layer == layer)
		update_nview();
	qInfo("set_config_para, l=%d, s=%d, nx=%d, ny=%d", _layer, _cpara.rescale, _cpara.img_num_w, _cpara.img_num_h);
	return 0;
}

//if _layer==-1, means current layer
int StitchView::get_config_para(int _layer, ConfigPara & _cpara) const
{
	qInfo("get_config_para, l=%d", _layer);
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	_cpara = lf[_layer]->cpara;
	_cpara.offset = _cpara.offset.clone();
	return 0;
}

//if _layer==-1, means current layer
int StitchView::set_tune_para(int _layer, const TuningPara & _tpara)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	qInfo("set_tune_para, l=%d", _layer);
	lf[_layer]->tpara = _tpara;
	return 0;
}

//if _layer==-1, means current layer
int StitchView::get_tune_para(int _layer, TuningPara & _tpara) const
{	
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	qInfo("get_tune_para, l=%d", _layer);
	_tpara = lf[_layer]->tpara;
	return 0;
}

//if _layer==-1, means get tune of current layer
void StitchView::set_mapxy_dstw(int _layer, const MapXY * _mapxy, int _dst_w)
{
	if (_layer == -1)
		_layer = layer;
    if (_layer == ABS_LAYER) {
        double zx = _mapxy->get_default_zoomx();
        double zy = _mapxy->get_default_zoomy();
        if (zx < 0.2 || zy < 0.2)
            return;
        for (int i = 0; i < (int)nails.size(); i++)
        if (nails[i].lf0 == nails[i].lf1) {
            nails[i].p1.x = nails[i].p1.x * zx;
            nails[i].p1.y = nails[i].p1.y * zy;
        }
    }
    else{
        if (_layer >= lf.size() || _layer < 0)
            return;
        ri->set_mapxy(_layer, _mapxy);
    }
	ri->set_dst_wide(_dst_w);
	generate_mapxy();
	qInfo("set_mapxy_dstw, l=%d, dst_w=%d", _layer, _dst_w);
	update();
}
//if _layer==-1, means get tune of current layer
MapXY * StitchView::get_mapxy(int _layer)
{
	if (_layer == -1)
		_layer = layer;
    if (_layer == ABS_LAYER) {
        MapXY * ret = ri->get_mapxy(0);
        ret->set_default_zoomx(1);
        ret->set_default_zoomy(1);
        ret->set_beta(0);
        return ret;
    }
	if (_layer >= lf.size() || _layer < 0)
		return NULL;
	qInfo("get_mapxy, l=%d", _layer);
	return ri->get_mapxy(_layer);
}

int StitchView::get_dst_wide()
{
	return ri->get_dst_wide();
}

void StitchView::set_grid(double _xoffset, double _yoffset, double _xgrid_size, double _ygrid_size)
{
	xoffset = _xoffset;
	yoffset = _yoffset;
	xgrid_size = _xgrid_size;
	ygrid_size = _ygrid_size;
	draw_grid = true;
	qInfo("set_grid, xoffset=%d, yoffset=%d, xgrid=%d, ygrid=%d", _xoffset, _yoffset, _xgrid_size, _ygrid_size);
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
	qInfo("compute_new_feature l=%d", _layer);
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is already running, wait it finish");
		return -2;
	}
	char name[100];
	sprintf(name, "/WorkData/autosave%d.xml", auto_save);
	auto_save = (auto_save + 1) % 5;

	string filename = project_path + name;
	write_file(filename);

	computing_feature.set_cfg_para(lf[_layer]->cpara);
	computing_feature.set_tune_para(lf[_layer]->tpara);
	feature_layer = _layer;
	compute_feature = QtConcurrent::run(thread_generate_diff, &computing_feature, project_path, _layer);
	compute_feature_timer = startTimer(1000);
	return 0;
}

int StitchView::optimize_offset(int _layer, int optimize_option)
{
	Mat_<Vec2i> adjust_offset;
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

	string filename = project_path + name;
	write_file(filename);
	vector<Nail> ns;
	get_one_layer_nails(lf[_layer], ns);
	if (!ns.empty())
		convert_nails(nails, nails, 0);
	vector<FixEdge> fe;
	for (int i = 0; i < 2; i++) {
		for (int y = 0; y < lf[_layer]->flagb[i].rows; y++)
		for (int x = 0; x < lf[_layer]->flagb[i].cols; x++) {
			int a = lf[_layer]->flagb[i](y, x);
			if (a != 0) {
				FixEdge new_fe;
				new_fe.idx = MAKE_EDGE_IDX(x, y, i);
				new_fe.bind_flag = a;
				const EdgeDiff * ped = lf[_layer]->feature.get_edge(new_fe.idx);
				Point idea_pos = ped->offset + ped->minloc * lf[layer]->cpara.rescale;
				if (FIX_EDGE_IDEA_POS(a)) {					
					new_fe.shift = ped->offset + Point(FIX_EDGE_IDEA_DX(a), FIX_EDGE_IDEA_DY(a)) * lf[_layer]->cpara.rescale;
				}
				else {
					if (i == 0) {
						if (FIX_EDGE_BINDY(a))
							new_fe.shift.y = lf[_layer]->cpara.offset(y + 1, x)[0] - lf[_layer]->cpara.offset(y, x)[0];
						else
							new_fe.shift.y = idea_pos.y;
						if (FIX_EDGE_BINDX(a))
							new_fe.shift.x = lf[_layer]->cpara.offset(y + 1, x)[1] - lf[_layer]->cpara.offset(y, x)[1];
						else
							new_fe.shift.x = idea_pos.x;
					}
					else {
						if (FIX_EDGE_BINDY(a))
							new_fe.shift.y = lf[_layer]->cpara.offset(y, x + 1)[0] - lf[_layer]->cpara.offset(y, x)[0];
						else
							new_fe.shift.y = idea_pos.y;
						if (FIX_EDGE_BINDX(a))
							new_fe.shift.x = lf[_layer]->cpara.offset(y, x + 1)[1] - lf[_layer]->cpara.offset(y, x)[1];
						else
							new_fe.shift.x = idea_pos.x;
					}
				}
				fe.push_back(new_fe);
			}
		}
	}
	thread_bundle_adjust(ba, &lf[_layer]->feature, &adjust_offset, &lf[_layer]->corner_info, &fe, ri->get_src_img_size(layer), optimize_option);
	lf[_layer]->cpara.offset = adjust_offset;
	self_check_offset(_layer);
	adjust_offset.release();
	if (!ns.empty()) {
		convert_nails(nails, nails, 1);
		generate_mapxy();
	}
	ri->invalidate_cache(_layer);
	if (_layer == layer) {
		ceview->set_layer_info(lf[layer]);
		notify_nail_info(layer);
	}
	update();
	return 0;
}

void StitchView::goto_xy(int x, int y)
{
	center = QPoint(x, y);
	qDebug("Goto s=%6.3f, c=(%d,%d), l=%d", scale, center.x(), center.y(), layer);
	update();
    QPoint ce(size().width()*scale / 2, size().height()*scale / 2);
    QSize s(size().width()*scale, size().height()*scale);
    view_rect = QRect(center - ce, s);
    nview->set_viewrect(view_rect);
}

void StitchView::clear_fix_edge(int _layer)
{
    if (_layer == -1)
        _layer = layer;
    if (_layer >= lf.size() || _layer < 0)
        return;
	qInfo("clear_fix_edge l=%d", _layer);
	lf[_layer]->flagb[0] = 0;
    lf[_layer]->flagb[1] = 0;
}

void StitchView::clear_red_fix_edge(int _layer)
{
    if (_layer == -1)
        _layer = layer;
    if (_layer >= lf.size() || _layer < 0)
        return;
    qInfo("clear_red_fix_edge l=%d", _layer);
	for (int i = 0; i < 2; i++) {
		for (int y = 0; y < lf[_layer]->flagb[i].rows; y++)
		for (int x = 0; x < lf[_layer]->flagb[i].cols; x++) {
			int a = lf[_layer]->flagb[i](y, x);
			if (FIX_EDGE_BINDX(a) || FIX_EDGE_BINDY(a))
				lf[_layer]->flagb[i](y, x) = 0;
		}
	}
}

void StitchView::clear_yellow_fix_edge(int _layer)
{
    if (_layer == -1)
        _layer = layer;
    if (_layer >= lf.size() || _layer < 0)
        return;
    qInfo("clear_yellow_fix_edge l=%d", _layer);
	for (int i = 0; i < 2; i++) {
		for (int y = 0; y < lf[_layer]->flagb[i].rows; y++)
		for (int x = 0; x < lf[_layer]->flagb[i].cols; x++) {
			int a = lf[_layer]->flagb[i](y, x);
			if (FIX_EDGE_IDEA_POS(a))
				lf[_layer]->flagb[i](y, x) = 0;
		}
	}

}

int StitchView::set_current_layer(int _layer) {
	if (_layer < get_layer_num() && _layer >= 0)
		layer = _layer;
	else
		return -1;
	update();
	update_nview();
	qInfo("set_current_layer l=%d", _layer);
	return 0;
}

string StitchView::get_layer_name(int _layer) {
	if (_layer == -1)
		_layer = layer;
	qInfo("get_layer_name l=%d", _layer);
	if (_layer >= lf.size() || _layer < 0)
		return string();
	else
		return lf[_layer]->layer_name;
}

void StitchView::set_layer_name(int _layer, string name) {
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return;
	else
		lf[_layer]->layer_name = name;
	qInfo("set_layer_name l=%d, name=%s", _layer, name.c_str());
	update_title();
}

void StitchView::set_nview(NavigateView * _nview) {
	nview = _nview;
	connect(nview, SIGNAL(update_center(QPoint)), this, SLOT(update_center(QPoint)));
}

void StitchView::set_ceview(CornerEdge * _ceview) {
	ceview = _ceview;
	connect(ceview, SIGNAL(goto_corner(unsigned)), this, SLOT(goto_corner(unsigned)));
	connect(ceview, SIGNAL(goto_edge(unsigned)), this, SLOT(goto_edge(unsigned)));
	connect(ceview, SIGNAL(goto_nail(unsigned)), this, SLOT(goto_nail(unsigned)));
}

void StitchView::to_state_add_nail() {
	qInfo("to_state_add_nail");
	mouse_state = PutFirstNail;
	setCursor(Qt::CrossCursor);
}

void StitchView::to_state_change_nail() {
	qInfo("to_state_change_nail");
	mouse_state = ChangeNail;
}

int StitchView::output_layer(int _layer, string pathname) {
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is running, can't output layer");
		return -2;
	}
	if (_layer == -1)
		_layer = layer;
	if (_layer >= lf.size() || _layer < 0)
		return -1;
	string filename = pathname + "/" + lf[_layer]->layer_name + ".db";
	ICLayerWrInterface *ic = ICLayerWrInterface::create(filename, false, 1, 1, 0, 0, 0, 0, 0);
	ICLayerInterface *icl = ic->get_iclayer_inf();

	int dst_w = ri->get_dst_wide();
	Point max_bd(0, 0);
	for (int l = 0; l < lf.size(); l++) {
		Point bd = ri->src2dst(l, Point(lf[l]->cpara.right_bound(), lf[l]->cpara.bottom_bound()));
		max_bd.x = max(bd.x, max_bd.x);
		max_bd.y = max(bd.y, max_bd.y);
	}
	int end_x = max_bd.x / dst_w + 1;
	int end_y = max_bd.y / dst_w + 1;
	qInfo("output_layer l=%d, file=%s, x=%d, y=%d", _layer, filename.c_str(), end_x, end_y);
	icl->putBlockNumWidth(end_x, end_y, dst_w);
	vector<MapID> map_id;
	vector<MapID> draw_order;
	for (int i = 0; i < 3; i++)
        draw_order.push_back(MAPID(_layer, 0xffff, 0xffff));
    QScopedPointer<MapXY> mxy(get_mapxy(_layer));
    mxy->set_merge_method(0);
    ri->set_mapxy(_layer, mxy.data());
	float cnt = 0;
	for (int y = 0; y < end_y; y++)
	for (int x = 0; x < end_x; x++) {
		map_id.push_back(MAPID(_layer, x, y));
		if (map_id.size() == 16 || (x + 1==end_x && y + 1==end_y)) {
			vector<QImage> imgs;
			ri->render_img(map_id, imgs, draw_order);
			CV_Assert(imgs.size() == map_id.size());			
			for (int i = 0; i < imgs.size(); i++) {
				QByteArray ba;
				QBuffer buffer(&ba);
				buffer.open(QIODevice::WriteOnly);
				imgs[i].save(&buffer, "JPG", IMAGE_OUTPUT_QUALITY);
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
    return 0;
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
	qInfo("delete_layer l=%d", _layer);
	del_one_layer_nails(lf[_layer]);
	for (int i = _layer; i < (int)lf.size() - 1; i++) {
		QScopedPointer<MapXY> mxy(get_mapxy(i + 1));
		ri->set_mapxy(i, mxy.data());
		ri->set_cfg_para(i, &lf[i + 1]->cpara);
	}
	delete lf[_layer];
	lf.erase(lf.begin() + _layer);
	vector<double> slope = generate_mapxy();
	if (layer >= _layer && layer >= 0) {
		layer--;
		update_nview();
        notify_nail_info(layer);
	}
	update_title();
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
	qInfo("layer_up l=%d", _layer);
	QScopedPointer<MapXY> mxy(get_mapxy(_layer));
	QScopedPointer<MapXY> mxy1(get_mapxy(_layer + 1));
	ri->set_mapxy(_layer, mxy1.data());
	ri->set_mapxy(_layer + 1, mxy.data());
	swap(lf[_layer], lf[_layer + 1]);
	ri->set_cfg_para(_layer, &lf[_layer]->cpara);
	ri->set_cfg_para(_layer + 1, &lf[_layer + 1]->cpara);
	generate_mapxy();
	if (layer == _layer) {
		layer++;
		update_nview();
    }
    update_title();
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
	qInfo("layer_down l=%d", _layer);
	QScopedPointer<MapXY> mxy(get_mapxy(_layer));
	QScopedPointer<MapXY> mxy1(get_mapxy(_layer - 1));
	ri->set_mapxy(_layer, mxy1.data());
	ri->set_mapxy(_layer - 1, mxy.data());
	swap(lf[_layer], lf[_layer - 1]);
	ri->set_cfg_para(_layer, &lf[_layer]->cpara);
	ri->set_cfg_para(_layer - 1, &lf[_layer - 1]->cpara);
	generate_mapxy();
	if (layer == _layer) {
		layer--;
		update_nview();
	}
    update_title();
	update();
	return 0;
}

void StitchView::update_title()
{
	char title[100];
	
	if (layer == ABS_LAYER)
		sprintf(title, "%d:%s s=%d", 0, "ABS", 1);
	else {
		string m = ri->mapxy_merge_method(layer) ? "Merge" : "";
		sprintf(title, "%d:%s s=%d %s", layer + 1, lf[layer]->layer_name.c_str(), lf[layer]->cpara.rescale, m.c_str());
	}
	emit title_change(QString::fromLocal8Bit(title));
}

QPoint StitchView::point2choose(QPoint mouse_point)
{
	if (layer == ABS_LAYER)
		return QPoint(10000, 10000);
	mouse_point = mouse_point * scale + view_rect.topLeft();
	Point src_point = ri->dst2src(layer, TOPOINT(mouse_point));
	return TOQPOINT(find_src_map(lf[layer]->cpara, src_point, ri->get_src_img_size(layer), 0));
}

void StitchView::write_file(string file_name)
{	
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is running, can't write file");
		return;
	}
	int loc = (int) file_name.find_last_of("\\/");
	string path = file_name.substr(0, loc);
	QFile file(QString::fromStdString(path));
	if (!file.exists()) {
		QMessageBox::information(this, "Info", QString::fromStdString(path + " not exist"));
		return;
	}
	if (lf.size() == 0)
		return;
	qInfo("write_file name=%s", file_name.c_str());
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
		fs << name << *get_mapxy(i);
		sprintf(name, "fixedge%d_0", i);
		fs << name << lf[i]->flagb[0];
		sprintf(name, "fixedge%d_1", i);
		fs << name << lf[i]->flagb[1];
		sprintf(name, "name%d", i);
		fs << name << lf[i]->layer_name;
		sprintf(name, "corner%d", i);
		fs << name << lf[i]->corner_info;
		sprintf(name, "check_edge0_%d", i);
		fs << name << lf[i]->checked_edge_offset[0];
		sprintf(name, "check_edge1_%d", i);
		fs << name << lf[i]->checked_edge_offset[1];
	}
	fs << "dst_w" << ri->get_dst_wide();
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

int StitchView::read_file(string file_name, bool import)
{
	qInfo("read_file name=%s", file_name.c_str());
	FileStorage fs(file_name, FileStorage::READ);
	if (fs.isOpened()) {
		int loc = (int) file_name.find_last_of("\\/");
		int loc2 = (int) file_name.find_last_of("\\/", loc - 1);
		if (!import)
			project_path = file_name.substr(0, loc2);
		else
		if (project_path != file_name.substr(0, loc2))
			return -1;
		int layer_num;
		layer_num = (int)fs["layer_num"];
		int begin_layer;
		if (!import) {	
			for (int i = 0; i < lf.size(); i++)
				delete lf[i];			
			lf.resize(layer_num);
			delete ri;
			ri = new RenderImage();
			begin_layer = 0;
		}
		else {
			begin_layer = (int) lf.size();
			lf.resize(layer_num + lf.size());
		}
		for (int i = begin_layer; i < (int)lf.size(); i++) {
			lf[i] = new LayerFeature();
			char name[30];
			sprintf(name, "cpara%d", i - begin_layer);
			fs[name] >> lf[i]->cpara;
			if (!lf[i]->cpara.img_path.empty()) {
				int loc = (int) lf[i]->cpara.img_path.find_last_of("\\/");
				string path = lf[i]->cpara.img_path.substr(0, loc);
				QFile file(QString::fromStdString(path));
				if (!file.exists()) { //may change path
					int loc2 = (int) lf[i]->cpara.img_path.find_last_of("\\/", loc - 1);
					string another_path = project_path + lf[i]->cpara.img_path.substr(loc2, loc - loc2);
					QFile file1(QString::fromStdString(another_path));
					if (!file1.exists()) {
						QMessageBox::information(this, "Info", QString::fromStdString(path + " not exist"));
						lf[i]->cpara.img_path.clear();
					}
					else {
						lf[i]->cpara.img_path = project_path + lf[i]->cpara.img_path.substr(loc2);
					}
						
				}
			}
			ri->set_cfg_para(i, &lf[i]->cpara);
			self_check_offset(i);
			sprintf(name, "tpara%d", i - begin_layer);
			fs[name] >> lf[i]->tpara;
			sprintf(name, "diff_file%d", i - begin_layer);
			fs[name] >> lf[i]->feature_file;
			if (!lf[i]->feature_file.empty()) {
				QFile file(QString::fromStdString(lf[i]->feature_file));
				if (!file.exists()) { //may change path
					int loc = (int) lf[i]->feature_file.find_last_of("\\/"); //try project path
					int loc2 = (int) lf[i]->feature_file.find_last_of("\\/", loc - 1);
					string another_path = project_path + lf[i]->feature_file.substr(loc2);
					QFile file1(QString::fromStdString(another_path));
					if (!file1.exists()) {
						QMessageBox::information(this, "Info", QString::fromStdString(lf[i]->feature_file + " not exist"));
						lf[i]->feature_file.clear();
					}
					else
						lf[i]->feature_file = another_path;
				}
				if (!lf[i]->feature_file.empty())
					lf[i]->feature.read_diff_file(lf[i]->feature_file);
			}
				
			MapXY0 mxy0;
			sprintf(name, "mapxy%d", i - begin_layer);
			fs[name] >> mxy0;
			QScopedPointer<MapXY> mxy(MapXY::create_mapxy()); //stupid method, but what else I can do?
			mxy->copy_base(mxy0);
			ri->set_mapxy(i, mxy.data());
			sprintf(name, "fixedge%d_0", i - begin_layer);
			fs[name] >> lf[i]->flagb[0];
			if (lf[i]->flagb[0].empty()) {
				lf[i]->flagb[0].create(lf[i]->cpara.offset.rows - 1, lf[i]->cpara.offset.cols);
				lf[i]->flagb[0] = 0;
			}
			sprintf(name, "fixedge%d_1", i - begin_layer);
			fs[name] >> lf[i]->flagb[1];
			if (lf[i]->flagb[1].empty()) {
				lf[i]->flagb[1].create(lf[i]->cpara.offset.rows, lf[i]->cpara.offset.cols - 1);
				lf[i]->flagb[1] = 0;
			}
			sprintf(name, "name%d", i - begin_layer);
			fs[name] >> lf[i]->layer_name;

			sprintf(name, "corner%d", i - begin_layer);
			fs[name] >> lf[i]->corner_info;			
			if (lf[i]->corner_info.empty()) {
				lf[i]->corner_info.create(lf[i]->cpara.offset.rows, lf[i]->cpara.offset.cols);
				lf[i]->corner_info = 0;
			}
			else {
				lf[i]->find_next = 0;
			}
			sprintf(name, "check_edge0_%d", i - begin_layer);
			fs[name] >> lf[i]->checked_edge_offset[0];
			sprintf(name, "check_edge1_%d", i - begin_layer);
			fs[name] >> lf[i]->checked_edge_offset[1];
		}
		int dst_w;
		fs["dst_w"] >> dst_w;
		ri->set_dst_wide(dst_w);
		nails.clear();
		FileNode file_nails = fs["Nails"];
		for (FileNodeIterator it = file_nails.begin(); it != file_nails.end(); it++) {
			int l0 = (int)(*it)["l0"];
			int l1 = (int)(*it)["l1"];
			int p0x = (int)(*it)["p0x"];
			int p0y = (int)(*it)["p0y"];
			int p1x = (int)(*it)["p1x"];
			int p1y = (int)(*it)["p1y"];
			nails.push_back(Nail(lf[l0 + begin_layer], lf[l1 + begin_layer], Point(p0x, p0y), Point(p1x, p1y)));
		}		
		vector<double> slope = generate_mapxy();
        char text[300];
        for (int i = 0, j = 0; i < slope.size(); i++)
            j += sprintf(text + j, "k%d=%7f,", i, slope[i]);
        qInfo(text);
		Point ce0, ce1, ce2, ct;
		int nview_visible;
		fs["layer"] >> layer;
		if (layer != ABS_LAYER)
			layer += begin_layer;
		fs["nview_visble"] >> nview_visible;
		fs["scale"] >> scale;
		fs["center"] >> ct;
		fs["choose"] >> ce0;
		fs["choose1"] >> ce1;
		fs["choose2"] >> ce2;
		fs["draw_corner"] >> draw_corner;
		draw_corner = draw_corner & 1;
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
		update_title();
		QPoint ce(size().width()*scale / 2, size().height()*scale / 2);
		QSize s(size().width()*scale, size().height()*scale);
		view_rect = QRect(center - ce, s);
		update_nview();
		if (layer != ABS_LAYER) {
			ceview->set_layer_info(lf[layer]);
			notify_nail_info(layer);
		}
		return 0;
	}
	return -1;
}
