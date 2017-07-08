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

static unsigned int mark_color_table[][2] = {
		{ 0xff000000, M_UNKNOW},
		{ 0xff0000a0, M_W },
		{ 0xff00a000, M_V },
		{ 0xff000030, M_W_I },
		{ 0xff003000, M_V_I },
		{ 0xff202020, M_I },
		{ 0xff008080, M_W_V },
		{ 0xffa00000, M_V_I_W },
		{ 0xff300000, M_V_I_V },
		{ 0xff800000, M_EDGE},
		{ 0xff202020, M_NOEDGE },
		{ 0xff202020, M_INL },
		{ 0xff0000a0, M_WNL },
		{ 0xff00a000, M_VNL },
		{ 0xff000030, M_W_INL },
		{ 0xff003000, M_V_INL }
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

ViaWireEditView::ViaWireEditView(QWidget *parent) : QWidget(parent)
{
    resize(600, 400);
    grid_high = 0;
    grid_width = 0;
    offset_y = 0;
    offset_x = 0;
	layer = 0;
    scale = 1;
    mark_state = SELECT_OBJ;
    current_obj.type = OBJ_NONE;
	setMouseTracking(true);
	mouse_press = false;
	mark_mask = 0;
	show_debug_en = false;
	hide_obj = false;
	mark_color.resize(sizeof(mark_color_table) / (sizeof(unsigned) * 2));
	for (int i = 0; i < mark_color.size(); i++)
		mark_color[mark_color_table[i][1]] = mark_color_table[i][0];
	cele = new CellExtract();
	current_train = NULL;
	vwe = NULL;
}

ViaWireEditView::~ViaWireEditView()
{
	if (cele != NULL)
		delete cele;
	if (vwe != NULL)
		delete vwe;
}
void ViaWireEditView::draw_obj(QPainter &painter, const MarkObj & obj)
{
	int wire_width = (obj.type == OBJ_LINE) ? 10 : 0;
	int via_radius = (obj.type == OBJ_POINT) ? 8 : 0;

    switch (obj.type) {
    case OBJ_NONE:
        break;
    case OBJ_AREA:
		if (obj.type2 == AREA_LEARN || obj.type2 == AREA_CELL || obj.type2==AREA_EXTRACT)
			if (obj.state != 4) {
				QPoint top_left(min(obj.p0.x(), obj.p1.x()), min(obj.p0.y(), obj.p1.y()));
				QPoint bot_right(max(obj.p0.x(), obj.p1.x()), max(obj.p0.y(), obj.p1.y()));
				if (obj.state == 3)
					painter.setPen(QPen(Qt::red, 1));
				else
					if (obj.type2 == AREA_LEARN)
						painter.setPen(QPen(Qt::white, 1));
					else
						if (obj.type2 == AREA_CELL)
							painter.setPen(QPen(Qt::yellow, 2));
						else
							painter.setPen(QPen(Qt::blue, 1));
				if (obj.type2 == AREA_LEARN || obj.type2 == AREA_CELL)
					painter.setBrush(QBrush(Qt::NoBrush));
				else
					painter.setBrush(QBrush(Qt::blue));
				painter.drawRect(QRect(top_left, bot_right));
				if (obj.state != 0) {
					painter.setPen(QPen(Qt::red, 1));
					painter.setBrush(QBrush(Qt::red));
					QPoint top_right(obj.p1.x(), obj.p0.y());
					QPoint bot_left(obj.p0.x(), obj.p1.y());
					if (obj.state == 1)
						painter.drawEllipse(obj.p0, 2, 2);
					if (obj.state == 2)
						painter.drawEllipse(obj.p1, 2, 2);
					if (obj.state == 5)
						painter.drawEllipse(top_right, 2, 2);
					if (obj.state == 6)
						painter.drawEllipse(bot_left, 2, 2);
				}
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
		if (obj.type2 == AREA_CHECK_ERR && obj.type3 == layer) {
			painter.setPen(QPen(QColor(200 * obj.prob, 0, 0), 1, Qt::DotLine));
			painter.setBrush(QBrush(Qt::NoBrush));
			QPoint lt(obj.p0.x(), obj.p0.y());
			QPoint rb(obj.p1.x(), obj.p1.y());
			painter.drawRect(QRect(lt, rb));
		}
        break;
    case OBJ_LINE:
		if (obj.state != 4 && obj.type3 == layer) {
			if (obj.state == 3)
				painter.setPen(QPen(Qt::red, 1));
			else
				painter.setPen(QPen(Qt::blue, 1));

			painter.drawLine(QLine(obj.p0, obj.p1));
			if (obj.state != 0) {
				painter.setPen(QPen(Qt::red, 1));
				painter.setBrush(QBrush(Qt::red));
				if (obj.state == 1)
					painter.drawEllipse(obj.p0, 2, 2);
				if (obj.state == 2)
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
    case OBJ_POINT:
        if (obj.state !=4 && obj.type3 == layer) {
            if (obj.state == 0) {
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

void ViaWireEditView::paintEvent(QPaintEvent *)
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
	
	if (!hide_obj) {
		for (unsigned i = 0; i < obj_set.size(); i++)
			draw_obj(painter_mem, obj_set[i]);
	}
    draw_obj(painter_mem, current_obj);

    QPainter painter(this);
    painter.save();
    painter.scale(scale, scale);
    painter.drawImage(QPoint(0, 0), image);
    painter.restore();
}

void ViaWireEditView::load_bk_image(QString file_path)
{
	string file_name = file_path.toStdString();
	unsigned char ll = file_name[file_name.size() - 5];
	while (1) {
		file_name[file_name.size() - 5] = ll++;
		QImage img;
		if (!img.load(QString::fromStdString(file_name)))
			break;
		qDebug("load image %s, format=%d", file_name.c_str(), img.format());
        bk_img.push_back(img.convertToFormat(QImage::Format_ARGB32));
	}
	if (bk_img.empty()) {
		QMessageBox::warning(this, "Warning", "file is not a image");
		return;
	}

	img_name = file_path.toStdString();	
	mark_mask = 0;
	show_debug_en = false;
	layer = 0;
	if (size() != bk_img[layer].size()*scale)
		resize(bk_img[layer].size()*scale);
	bk_img_mask = bk_img[layer];
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

	if (!bk_img.empty()) {
		resize(bk_img[layer].size()*scale);
		update();
	}
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
    for (unsigned i=0; i<obj_set.size(); i++) {
        char sh[30];
        Point p0(obj_set[i].p0.x(), obj_set[i].p0.y());
		Point p1(obj_set[i].p1.x(), obj_set[i].p1.y());

		sprintf(sh, "t%d", i);
        fs << sh << ((int) obj_set[i].type | ((int) obj_set[i].type2<<16) | ((int) obj_set[i].type3<<24));
		sprintf(sh, "p0_%d", i);
		fs << sh << p0;
		sprintf(sh, "p1_%d", i);
		fs << sh << p1;
    }
	fs.release();
}

bool ViaWireEditView::load_objects(QString file_path)
{
	qDebug("load objects %s", file_path.toStdString().c_str());
	FileStorage fs(file_path.toStdString(), FileStorage::READ);
	if (fs.isOpened()) {
		int num = -1;
		fs["obj_num"] >> num;
		if (num > 0) {
			obj_set.resize(num);
			for (unsigned i = 0; i < obj_set.size(); i++) {
				char sh[30];
				Point p;
				obj_set[i].state = 0;
				sprintf(sh, "t%d", i);
				int type;
				fs[sh] >> type;
				obj_set[i].type3 = (type >> 24) & 0xff;
				obj_set[i].type2 = (type >> 16) & 0xff;
				obj_set[i].type = type & 0xffff;
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
		update();
		return true;
	}
	return false;		
}

void ViaWireEditView::erase_all_objects()
{
	obj_set.clear();
	current_obj.type = OBJ_NONE;
	update();
}

void ViaWireEditView::start_cell_train(int , int , int , float _param1, float _param2, float _param3)
{
	cele->set_train_param(0, 0, 0, 0, 0, 0, _param1 * 100, _param2 * 100, _param3 * 100, 0);
	for (int i = 0; i < obj_set.size(); i++)
		if (obj_set[i].type2 == AREA_CELL && obj_set[i].type == OBJ_AREA)
			obj_set[i].type3 = POWER_UP;
	cele->train(img_name, obj_set);
	current_train = cele;
}

void ViaWireEditView::set_wire_para(ExtractParam * ep, string action_name)
{
	if (vwe != NULL)
		delete vwe;
	qInfo("Set param for action %s", action_name.c_str());
	vwe = VWExtract::create_extract(0);
	vector<ParamItem> params;
	ep->get_param(action_name, params);
	int ll = img_name[img_name.size() - 5] - '0';

	for (int i = 0; i < params.size(); i++) {
		if (params[i].pi[0] >= 0) {
			params[i].pi[0] -= ll;
			if (params[i].pi[0] < 0)
				continue;
		}
		qInfo("Set params %x, %x, %x, %x, %x, %x, %x, %x, %x, %f", params[i].pi[0], params[i].pi[1], params[i].pi[2],
			params[i].pi[3], params[i].pi[4], params[i].pi[5], params[i].pi[6], params[i].pi[7], params[i].pi[8], params[i].pf);
		vwe->set_extract_param(params[i].pi[0], params[i].pi[1], params[i].pi[2], params[i].pi[3], params[i].pi[4],
			params[i].pi[5], params[i].pi[6], params[i].pi[7], params[i].pi[8], params[i].pf);
	}
	current_train = vwe;
}

void ViaWireEditView::extract()
{
	if (current_train == NULL)
		return;
    obj_set.clear();
    current_obj.type = OBJ_NONE;
	current_train->extract(img_name, QRect(0, 0, 1020, 1020), obj_set);
    update();
}

void ViaWireEditView::show_debug(unsigned _mark_mask, bool _show_debug_en)
{
	if (current_train == NULL || bk_img.empty())
		return;
	mark_mask = _mark_mask;	
	show_debug_en = _show_debug_en;
	
	if (!_show_debug_en || _mark_mask > 3) {
		bk_img_mask = bk_img[layer];		
	}
	else {
		Mat mark;
		switch (_mark_mask) {
		case 0:
			mark = current_train->get_mark(layer);
			break;
		case 1:
			mark = current_train->get_mark1(layer);
			break;
		case 2:
			mark = current_train->get_mark2(layer);
			break;
		case 3:
			mark = current_train->get_mark3(layer);
			break;
		} 
		if (bk_img[layer].width() != mark.cols || bk_img[layer].height() != mark.rows) {
			QMessageBox::warning(this, "Warning", "click train or extract first!");
			return;
		}
		bk_img_mask = bk_img[layer].copy();
		for (int y = 0; y < mark.rows; y++) {
			unsigned char * p_mark = mark.ptr<unsigned char>(y);
			for (int x = 0; x < mark.cols; x++) {
				unsigned bc = p_mark[x];
				QRgb nc = bc + (bc << 8) + (bc << 16);
				nc += 0xff000000;
				bk_img_mask.setPixel(x, y, nc);
			}
		}
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
                obj_set[select_idx].state = 4;
			}
		}
		else {
			mouse_press = true;
			current_obj.type = mark_state;
			current_obj.type2 = mark_type2;
			current_obj.type3 = layer;
			if (mark_type2 == AREA_CELL)
				current_obj.type3 = POWER_UP;
			current_obj.state = 0;
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
			current_obj.state = 0;
			obj_set[select_idx] = current_obj;
			current_obj.type = OBJ_NONE;
		}
		else {
            current_obj.p1 = event->pos() /scale;
			
			if (mark_state == OBJ_LINE) {
				adjust_wire_point(current_obj.p0, current_obj.p1);
				QPoint p2;
				if (current_obj.p0.x() > current_obj.p1.x() || current_obj.p0.y() > current_obj.p1.y()) {
					p2 = current_obj.p0;
					current_obj.p0 = current_obj.p1;
					current_obj.p1 = p2;
				}
			}				
			if (mark_state == OBJ_POINT)
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
			if (current_obj.state == 1 || current_obj.state == 3)
                current_obj.p0 = obj_set[select_idx].p0 + event->pos() / scale - mp_point;
			if (current_obj.state == 2 || current_obj.state == 3)
                current_obj.p1 = obj_set[select_idx].p1 + event->pos() / scale - mp_point;
			if (current_obj.type == OBJ_LINE) {
				if (current_obj.state == 1)
					adjust_wire_point(current_obj.p1, current_obj.p0);
				if (current_obj.state == 2)
					adjust_wire_point(current_obj.p0, current_obj.p1);
			}
			if (current_obj.type == OBJ_AREA) {
                QPoint offset = event->pos() / scale - mp_point;
				if (current_obj.state == 5) {
					current_obj.p0.setY(obj_set[select_idx].p0.y() + offset.y());
					current_obj.p1.setX(obj_set[select_idx].p1.x() + offset.x());
				}
				if (current_obj.state == 6) {
					current_obj.p0.setX(obj_set[select_idx].p0.x() + offset.x());
					current_obj.p1.setY(obj_set[select_idx].p1.y() + offset.y());
				}
			}
		}
		else {
            current_obj.p1 = event->pos() / scale;
			if (mark_state == OBJ_LINE)
				adjust_wire_point(current_obj.p0, current_obj.p1);
			if (mark_state == OBJ_POINT)
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
                obj_set[i].state = 0;
				if (obj_set[i].type == OBJ_LINE || obj_set[i].type == OBJ_POINT) {
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
                obj_set[select_idx].state = min_wp;
			else
				select_idx = -1;
			update();
        }
    }
	if (bk_img.empty() || !bk_img[layer].valid(event->pos() / scale))
		return;
	QRgb color = bk_img_mask.pixel(event->pos() / scale);
	char s[200];
	if (current_train == NULL) 
		sprintf(s, "c=%x", color);	
	else {		
		int len = 0;
		vector<float> feature;
		current_train->get_feature(layer, (event->pos()).x() / scale, (event->pos()).y() / scale, feature);
		if (!feature.empty()) {
			for (int i = 0; i < feature.size(); i++) {
				len += sprintf(s + len, "f%d=%f,", i, feature[i]);
			}
		}
		
		sprintf(s, "%s, c=%x", s, color);
	}
    emit mouse_change(event->pos() / scale, QString(s));
}

void ViaWireEditView::keyPressEvent(QKeyEvent *e)
{
	int new_layer = layer;
	switch (e->key()) {
	case Qt::Key_0:
		if (bk_img.size() > 0)
			new_layer = 0;
		break;
	case Qt::Key_1:
		if (bk_img.size() > 1)
			new_layer = 1;
		break;
	case Qt::Key_2:
		if (bk_img.size() > 2)
			new_layer = 2;
		break;
	case Qt::Key_3:
		if (bk_img.size() > 3)
			new_layer = 3;
		break;
	case Qt::Key_4:
		if (bk_img.size() > 4)
			new_layer = 4;
		break;
	case Qt::Key_5:
		if (bk_img.size() > 5)
			new_layer = 5;
		break;
	case Qt::Key_6:
		if (bk_img.size() > 6)
			new_layer = 6;
		break;
	case Qt::Key_7:
		if (bk_img.size() > 7)
			new_layer = 7;
		break;
	case Qt::Key_8:
		if (bk_img.size() > 8)
			new_layer = 8;
		break;
	case Qt::Key_9:
		if (bk_img.size() > 9)
			new_layer = 9;
		break;
	case Qt::Key_PageUp:
		if (bk_img.size() > layer + 1)
			new_layer = layer + 1;
		break;
	case Qt::Key_PageDown:
		if (layer > 0)
			new_layer = layer - 1;
		break;
	}
	if (new_layer != layer) {
		layer = new_layer;
		bk_img_mask = bk_img[layer];
		if (show_debug_en)
            show_debug(mark_mask, show_debug_en);
		update();
		return;
	}
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
			if (select_obj.state == 1 || select_obj.state == 3)
				select_obj.p0.setY(select_obj.p0.y() + d);
			if (select_obj.state == 2 || select_obj.state == 3)
				select_obj.p1.setY(select_obj.p1.y() + d);
			if (select_obj.state == 1)
				adjust_wire_point(select_obj.p1, select_obj.p0);
			if (select_obj.state == 2)
				adjust_wire_point(select_obj.p0, select_obj.p1);
			obj_set[select_idx] = select_obj;
			break;	
		case Qt::Key_Left:
		case Qt::Key_Right:
			d = (e->key() == Qt::Key_Left) ? -1 : 1;
			if (select_obj.state == 1 || select_obj.state == 3)
				select_obj.p0.setX(select_obj.p0.x() + d);
			if (select_obj.state == 2 || select_obj.state == 3)
				select_obj.p1.setX(select_obj.p1.x() + d);
			if (select_obj.state == 1)
				adjust_wire_point(select_obj.p1, select_obj.p0);
			if (select_obj.state == 2)
				adjust_wire_point(select_obj.p0, select_obj.p1);
			obj_set[select_idx] = select_obj;
			break;   
		case Qt::Key_H:
			hide_obj = !hide_obj;
			break;
		}
		update();
    }
}
