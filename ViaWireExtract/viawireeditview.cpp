#include "viawireeditview.h"
#include <QPainter>
#include <QDebug>
#include <string>
#include <algorithm>
#include <QMessageBox>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;

static unsigned int mark_color_table[] = {
	0xff000000,
	0xff0000a0,
	0xff00a000,
    0xff000030,
    0xff003000,
	0xff202020,
	0xff008080,
	0xffa00000,
    0xff300000,
	0xff202020,
	0xff0000a0,
	0xff00a000
};
static double distance_p2p(const QPoint & p1, const QPoint & p2)
{
    double ret;
    double t = p1.x() -p2.x();

    ret = t * t;
    t = p1.y() -p2.y();
    ret += t * t;
    return ret;
}

static double distance_p2l(const QPoint & p1, const QPoint & p2, const QPoint & p, int & wp)
{
    double a2, b2, c2;

    a2 = distance_p2p(p1, p2);
    b2 = distance_p2p(p1, p);
    c2 = distance_p2p(p2, p);

    if (a2<=1) { //p1 & p2 is one point
        wp = 3;
        return b2;
    }

    if (c2 >= a2 + b2) { //p1 is dun jiao
        wp = 1;
        return b2;
    }

    if (b2 >= a2 + c2) { //p2 is dun jiao
        wp=2;
        return c2;
    }

    wp =3;
    double s = c2*a2 - (c2+a2-b2) * (c2+a2-b2) /4;

    return s/a2;
}

void adjust_wire_point(const QPoint & pbase, QPoint & padjust)
{
	QPoint ano_point = pbase;
	if (abs(pbase.x() - padjust.x()) > abs(pbase.y() - padjust.y()))
		ano_point.setX(padjust.x());
	else
		ano_point.setY(padjust.y());
	padjust = ano_point;
}

bool ViaWireEditView::load_objects(QString file_path, std::vector <MarkObj> & obj_set, int & wire_width, int & via_radius)
{
	FileStorage fs(file_path.toStdString(), FileStorage::READ);
	if (fs.isOpened()) {
		int num = -1;
		fs["obj_num"] >> num;
		fs["wire_width"] >> wire_width;
		fs["via_radius"] >> via_radius;
		if (num > 0) {
			obj_set.resize(num);
			for (unsigned i = 0; i < obj_set.size(); i++) {
				char sh[30];
				Point p;
				obj_set[i].select_state = 0;
				sprintf(sh, "t%d", i);
				fs[sh] >> obj_set[i].type;
				obj_set[i].type2 = obj_set[i].type >> 16;
				obj_set[i].type = obj_set[i].type & 0xffff;
				sprintf(sh, "p0_%d", i);
				fs[sh] >> p;
				obj_set[i].p0.setX(p.x);
				obj_set[i].p0.setY(p.y);
				sprintf(sh, "p1_%d", i);
				fs[sh] >> p;
				obj_set[i].p1.setX(p.x);
				obj_set[i].p1.setY(p.y);
			}
		}

		return true;
	}
	return false;
}

ViaWireEditView::ViaWireEditView(QWidget *parent) : QWidget(parent)
{
    resize(600, 400);
    grid_high = 0;
    grid_width = 0;
    offset_y = 0;
    offset_x = 0;
	wire_width = 10;
	via_radius = 9;
    scale = 1;
    mark_state = SELECT_OBJ;
    current_obj.type = OBJ_NONE;
	setMouseTracking(true);
	mouse_press = false;
}

void ViaWireEditView::draw_obj(QPainter &painter, const MarkObj & obj)
{
    switch (obj.type) {
    case OBJ_NONE:
        break;
    case OBJ_AREA:
		if (obj.select_state != 4) {
			QPoint top_left(min(obj.p0.x(), obj.p1.x()), min(obj.p0.y(), obj.p1.y()));
			QPoint bot_right(max(obj.p0.x(), obj.p1.x()), max(obj.p0.y(), obj.p1.y()));
			if (obj.select_state == 3)
				painter.setPen(QPen(Qt::red, 1));
			else
				if (obj.type2 == AREA_LEARN)
					painter.setPen(QPen(Qt::white, 1));
				else
					painter.setPen(QPen(Qt::blue, 1));
			if (obj.type2 == AREA_LEARN)
				painter.setBrush(QBrush(Qt::NoBrush));
			else
				painter.setBrush(QBrush(Qt::blue));
			painter.drawRect(QRect(top_left, bot_right));
			if (obj.select_state != 0) {
				painter.setPen(QPen(Qt::red, 1));
				painter.setBrush(QBrush(Qt::red));
				QPoint top_right(obj.p1.x(), obj.p0.y());
				QPoint bot_left(obj.p0.x(), obj.p1.y());
				if (obj.select_state == 1)
					painter.drawEllipse(obj.p0, 2, 2);
				if (obj.select_state == 2)
					painter.drawEllipse(obj.p1, 2, 2);
				if (obj.select_state == 5)
					painter.drawEllipse(top_right, 2, 2);
				if (obj.select_state == 6)
					painter.drawEllipse(bot_left, 2, 2);				
			}
			//draw metal rect
			if (obj.type2 == AREA_METAL) {
				painter.setPen(QPen(Qt::blue, 1, Qt::DotLine));
				painter.setBrush(QBrush(Qt::NoBrush));
				int rw = wire_width - wire_width / 2 - 1;
				QPoint lt(min(obj.p0.x(), obj.p1.x()) - wire_width / 2, min(obj.p0.y(), obj.p1.y()) - wire_width / 2);
				QPoint rb(max(obj.p0.x(), obj.p1.x()) + rw, max(obj.p0.y(), obj.p1.y()) + rw);
                painter.drawRect(QRect(lt, rb -QPoint(1,1)));
			}
		}
        break;
    case OBJ_WIRE:
		if (obj.select_state != 4) {
			if (obj.select_state == 3)
				painter.setPen(QPen(Qt::red, 1));
			else
				painter.setPen(QPen(Qt::blue, 1));

			painter.drawLine(QLine(obj.p0, obj.p1));
			if (obj.select_state != 0) {
				painter.setPen(QPen(Qt::red, 1));
				painter.setBrush(QBrush(Qt::red));
				if (obj.select_state == 1)
					painter.drawEllipse(obj.p0, 2, 2);
				if (obj.select_state == 2)
					painter.drawEllipse(obj.p1, 2, 2);
			}
			//draw metal rect			
			painter.setPen(QPen(Qt::blue, 1, Qt::DotLine));
			painter.setBrush(QBrush(Qt::NoBrush));
			int rw = wire_width - wire_width / 2 - 1;
			QPoint lt(min(obj.p0.x(), obj.p1.x()) - wire_width / 2, min(obj.p0.y(), obj.p1.y()) - wire_width / 2);
			QPoint rb(max(obj.p0.x(), obj.p1.x()) + rw, max(obj.p0.y(), obj.p1.y()) + rw);
            painter.drawRect(QRect(lt, rb-QPoint(1,1)));
			
		}
        break;
    case OBJ_VIA:
        if (obj.select_state !=4) {
            if (obj.select_state == 0) {
                painter.setPen(QPen(Qt::green, 1));
                painter.setBrush(QBrush(Qt::green));
            } else {
                painter.setPen(QPen(Qt::red, 1));
                painter.setBrush(QBrush(Qt::red));
            }
            painter.drawEllipse(obj.p1, 1, 1);

			//draw via circle
			painter.setPen(QPen(Qt::green, 1, Qt::DotLine));
			painter.setBrush(QBrush(Qt::NoBrush));
			painter.drawEllipse(obj.p1, via_radius, via_radius);
        }
        break;
    }
}

void ViaWireEditView::paintEvent(QPaintEvent *e)
{    
    if (bk_img_mask.isNull())
        return;
    QImage image = bk_img_mask.copy();
    QPainter painter_mem(&image);

    painter_mem.setPen(QPen(Qt::red, 1));
    painter_mem.setBrush(QBrush(Qt::red));
    if (grid_high!=0 && grid_width!=0) {
        for (double y=offset_y; y<height(); y+=grid_high)
            for (double x=offset_x; x<width(); x+=grid_width)
                painter_mem.drawRect((int)x, (int)y, 1, 1);
    }
	    
    for (unsigned i=0; i<obj_set.size(); i++)
        draw_obj(painter_mem, obj_set[i]);
    draw_obj(painter_mem, current_obj);

    QPainter painter(this);
    painter.save();
    painter.scale(scale, scale);
    painter.drawImage(QPoint(0, 0), image);
    painter.restore();
}

void ViaWireEditView::load_bk_image(QString file_path)
{
	if (!bk_img.load(file_path)) {
		QMessageBox::warning(this, "Warning", "file is not a image");
		return;
	}
	img_name = file_path.toStdString();
	qDebug("load image %s, format=%d", file_path.toStdString().c_str(), bk_img.format());
    if (size() != bk_img.size())
        resize(bk_img.size()*scale);
	bk_img_mask = bk_img;
    update();
}

void ViaWireEditView::set_grid_size(double high, double width)
{
    qDebug("Grid high=%f, width=%f", high, width);
    if (grid_high!=high || grid_width!=width) {
        grid_high = high;
        grid_width = width;
        update();
    }
}

void ViaWireEditView::set_offset(double oy, double ox)
{
    qDebug("offset_y=%f, offset_x=%f", oy, ox);
    if (offset_y != oy || offset_x !=ox) {
        offset_y = oy;
        offset_x = ox;
        update();
    }
}

void ViaWireEditView::set_mark(int ms, int type2)
{
    qDebug("Set mark %d", ms);
    mark_state = ms;
	mark_type2 = type2;
    if (ms == SELECT_OBJ)
        setFocus();
}

void ViaWireEditView::set_scale(int _scale)
{
    if (_scale<=16 && _scale>=1)
        scale = _scale;
    else
        return;

    if (!bk_img.isNull())
        resize(bk_img.size()*scale);
    update();
}

int ViaWireEditView::get_scale()
{
    return scale;
}

void ViaWireEditView::save_objects(QString file_path)
{
	qDebug("save objects %s", file_path.toStdString().c_str());
	if (obj_set.size() == 0)
		return;
    FileStorage fs(file_path.toStdString(), FileStorage::WRITE);
    fs << "obj_num" << (int) obj_set.size();
    fs << "wire_width" << wire_width;
    fs << "via_radius" << via_radius;
    for (unsigned i=0; i<obj_set.size(); i++) {
        char sh[30];
        Point p0(obj_set[i].p0.x(), obj_set[i].p0.y());
		Point p1(obj_set[i].p1.x(), obj_set[i].p1.y());

		sprintf(sh, "t%d", i);
		fs << sh << (obj_set[i].type | (obj_set[i].type2<<16));
		sprintf(sh, "p0_%d", i);
		fs << sh << p0;
		sprintf(sh, "p1_%d", i);
		fs << sh << p1;
    }
	fs.release();
}

void ViaWireEditView::load_objects(QString file_path)
{
	qDebug("load objects %s", file_path.toStdString().c_str());
	if (load_objects(file_path, obj_set, wire_width, via_radius))
		update();
}

void ViaWireEditView::erase_all_objects()
{
	obj_set.clear();
	current_obj.type = OBJ_NONE;
	update();
}

void ViaWireEditView::start_train(int _feature, int _iter_num, float _param1,float _param2, float _param3)
{
	if (obj_set.empty()) {
		QMessageBox::warning(this, "Warning", "draw or load wire via first!");
		return;
	}

    vwe.set_param(wire_width, via_radius, _iter_num, _param1, _param2, _param3);
    vwe.train(img_name, obj_set, _feature);

	//vwe.extract(img_name, QRect(900, 600, 1000, 700), obj_sets);
}

void ViaWireEditView::extract()
{
    vector<MarkObj> obj_sets;
    vwe.extract(img_name, QRect(50, 50, 550, 250), obj_sets);
}

void ViaWireEditView::set_mark(unsigned mark_mask)
{
	Mat mark = vwe.get_mark();
	if (bk_img.width() != mark.cols || bk_img.height() != mark.rows) {
		QMessageBox::warning(this, "Warning", "click train or extract first!");
		return;
	}
	if (mark_mask == 0) {
		bk_img_mask = bk_img;
		return;
	}
	else
		bk_img_mask = bk_img.copy();
	
	for (int y = 0; y < mark.rows; y++) {		
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = 0; x < mark.cols; x++)
			if ((mark_mask >> p_mark[x]) & 1)
				bk_img_mask.setPixel(x, y, mark_color_table[p_mark[x]]);
	}
	update();
}

void ViaWireEditView::mousePressEvent(QMouseEvent *event)
{
    if (mark_state != OBJ_NONE) {
        mp_point = event->pos() / scale;
		if (mark_state == SELECT_OBJ) {
			if (select_idx != -1) {
				mouse_press = true;
				current_obj = obj_set[select_idx];
				obj_set[select_idx].select_state = 4;
			}
		}
		else {
			mouse_press = true;
			current_obj.type = mark_state;
			current_obj.type2 = mark_type2;
			current_obj.select_state = 0;
            current_obj.p0 = event->pos() / scale;
            current_obj.p1 = event->pos() / scale;
		}        
    }
}

void ViaWireEditView::mouseReleaseEvent(QMouseEvent *event)
{
    if (mouse_press) {
        mouse_press = false;
		if (mark_state == SELECT_OBJ) {
#if 0
			if (current_obj.select_state == 1 || current_obj.select_state == 3)
				current_obj.p0 = obj_set[select_idx].p0 + event->pos() - mp_point;
			if (current_obj.select_state == 2 || current_obj.select_state == 3)
				current_obj.p1 = obj_set[select_idx].p1 + event->pos() - mp_point;
			if (current_obj.select_state == 1)
				adjust_wire_point(current_obj.p1, current_obj.p0);
			if (current_obj.select_state == 2)
				adjust_wire_point(current_obj.p0, current_obj.p1);
#endif
			current_obj.select_state = 0;
			obj_set[select_idx] = current_obj;
			current_obj.type = OBJ_NONE;
		}
		else {
            current_obj.p1 = event->pos() /scale;
			
			if (mark_state == OBJ_WIRE) {
				adjust_wire_point(current_obj.p0, current_obj.p1);
				QPoint p2;
				if (current_obj.p0.x() > current_obj.p1.x() || current_obj.p0.y() > current_obj.p1.y()) {
					p2 = current_obj.p0;
					current_obj.p0 = current_obj.p1;
					current_obj.p1 = p2;
				}
			}				
			if (mark_state == OBJ_VIA)
				current_obj.p0 = current_obj.p1;
			if (mark_state == OBJ_AREA) {
				QPoint top_left(min(current_obj.p0.x(), current_obj.p1.x()), min(current_obj.p0.y(), current_obj.p1.y()));
				QPoint bot_right(max(current_obj.p0.x(), current_obj.p1.x()), max(current_obj.p0.y(), current_obj.p1.y()));
				current_obj.p0 = top_left;
				current_obj.p1 = bot_right;
			}
			obj_set.push_back(current_obj);
			
			current_obj.type = OBJ_NONE;
		}
        
        update();
    }
}

void ViaWireEditView::mouseMoveEvent(QMouseEvent *event)
{
    if (mouse_press) {
		if (mark_state == SELECT_OBJ) {
			if (current_obj.select_state == 1 || current_obj.select_state == 3)
                current_obj.p0 = obj_set[select_idx].p0 + event->pos() / scale - mp_point;
			if (current_obj.select_state == 2 || current_obj.select_state == 3)
                current_obj.p1 = obj_set[select_idx].p1 + event->pos() / scale - mp_point;
			if (current_obj.type == OBJ_WIRE) {
				if (current_obj.select_state == 1)
					adjust_wire_point(current_obj.p1, current_obj.p0);
				if (current_obj.select_state == 2)
					adjust_wire_point(current_obj.p0, current_obj.p1);
			}
			if (current_obj.type == OBJ_AREA) {
                QPoint offset = event->pos() / scale - mp_point;
				if (current_obj.select_state == 5) {
					current_obj.p0.setY(obj_set[select_idx].p0.y() + offset.y());
					current_obj.p1.setX(obj_set[select_idx].p1.x() + offset.x());
				}
				if (current_obj.select_state == 6) {
					current_obj.p0.setX(obj_set[select_idx].p0.x() + offset.x());
					current_obj.p1.setY(obj_set[select_idx].p1.y() + offset.y());
				}
			}
		}
		else {
            current_obj.p1 = event->pos() / scale;
			if (mark_state == OBJ_WIRE)
				adjust_wire_point(current_obj.p0, current_obj.p1);
			if (mark_state == OBJ_VIA)
				current_obj.p0 = current_obj.p1;
		}			
        update();
    } else {
        if (mark_state == SELECT_OBJ) {
			double min_dis = 10000;
			int min_wp;
            for (int i=0; i<obj_set.size(); i++) {
                int wp;
				double dis;
                obj_set[i].select_state = 0;
				if (obj_set[i].type == OBJ_WIRE || obj_set[i].type == OBJ_VIA) {
                    dis = distance_p2l(obj_set[i].p0, obj_set[i].p1, event->pos() / scale, wp);
					if (min_dis > dis) {
						min_dis = dis;
						min_wp = wp;
						select_idx = i;
					}
				}					
				if (obj_set[i].type == OBJ_AREA) {
					QPoint top_right(obj_set[i].p1.x(), obj_set[i].p0.y());
					QPoint bot_left(obj_set[i].p0.x(), obj_set[i].p1.y());
                    dis = distance_p2l(obj_set[i].p0, top_right, event->pos() / scale, wp);
					if (min_dis > dis) {
						min_dis = dis;
						min_wp = (wp == 2) ? 5 : wp;
						select_idx = i;
					}
                    dis = distance_p2l(top_right, obj_set[i].p1, event->pos() / scale, wp);
					if (min_dis > dis) {
						min_dis = dis;
						min_wp = (wp == 1) ? 5 : wp;
						select_idx = i;
					}
                    dis = distance_p2l(bot_left, obj_set[i].p1, event->pos() / scale, wp);
					if (min_dis > dis) {
						min_dis = dis;
						min_wp = (wp == 1) ? 6 : wp;
						select_idx = i;
					}
                    dis = distance_p2l(obj_set[i].p0, bot_left, event->pos() / scale, wp);
					if (min_dis > dis) {
						min_dis = dis;
						min_wp = (wp == 2) ? 6 : wp;
						select_idx = i;
					}
				}				
            }

			if (min_dis < 9.1)
				obj_set[select_idx].select_state = min_wp;
			else
				select_idx = -1;
			update();
        }
    }
    emit mouse_change(event->pos() / scale);
}

void ViaWireEditView::keyPressEvent(QKeyEvent *e)
{
	
	if (mark_state == SELECT_OBJ && select_idx != -1) {
		MarkObj select_obj = obj_set[select_idx];
		int d;
		switch (e->key()) {
		case Qt::Key_Delete:
			obj_set.erase(obj_set.begin() + select_idx);
			break;
		case Qt::Key_Up:
		case Qt::Key_Down:
			d = (e->key() == Qt::Key_Up) ? -1 : 1;
			if (select_obj.select_state == 1 || select_obj.select_state == 3)
				select_obj.p0.setY(select_obj.p0.y() + d);
			if (select_obj.select_state == 2 || select_obj.select_state == 3)
				select_obj.p1.setY(select_obj.p1.y() + d);
			if (select_obj.select_state == 1)
				adjust_wire_point(select_obj.p1, select_obj.p0);
			if (select_obj.select_state == 2)
				adjust_wire_point(select_obj.p0, select_obj.p1);
			obj_set[select_idx] = select_obj;
			break;	
		case Qt::Key_Left:
		case Qt::Key_Right:
			d = (e->key() == Qt::Key_Left) ? -1 : 1;
			if (select_obj.select_state == 1 || select_obj.select_state == 3)
				select_obj.p0.setX(select_obj.p0.x() + d);
			if (select_obj.select_state == 2 || select_obj.select_state == 3)
				select_obj.p1.setX(select_obj.p1.x() + d);
			if (select_obj.select_state == 1)
				adjust_wire_point(select_obj.p1, select_obj.p0);
			if (select_obj.select_state == 2)
				adjust_wire_point(select_obj.p0, select_obj.p1);
			obj_set[select_idx] = select_obj;
			break;        
		}

		update();
    }
}
