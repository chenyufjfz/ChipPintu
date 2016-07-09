#include "stitchview.h"
#include <QPainter>
#include <algorithm>

const int step_para = 3;
const int min_scale = 1;
const int max_scale = 8;
StitchView::StitchView(QWidget *parent) : QWidget(parent)
{
    scale = 1;
	center = QPoint(0, 0);
    cache_size = 100;
    layer = 0;
	choose = QPoint(-1, -1);
    for (int i=0; i<cache_size; i++)
        cache_list.push_back(INVALID_MAP_ID);
    resize(1800, 1000);
	load_layer_info("C:/chenyu/work/ChipPintu/FeatureExtract/release/coarse_layer_cfg.xml");
	setMouseTracking(true);
	setAutoFillBackground(false);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setAttribute(Qt::WA_NoSystemBackground, true);
}

int StitchView::load_layer_info(string layer_cfg)
{
	LayerInfo layer_info;

	FileStorage fs(layer_cfg, FileStorage::READ);
	if (fs.isOpened()) {
		fs["clip_left"] >> layer_info.clip_l;
		fs["clip_right"] >> layer_info.clip_r;
		fs["clip_up"] >> layer_info.clip_u;
		fs["clip_down"] >> layer_info.clip_d;
		fs["image_path"] >> layer_info.img_path;
		fs["img_num_w"] >> layer_info.img_num_w;
		fs["img_num_h"] >> layer_info.img_num_h;
		fs["right_bound"] >> layer_info.right_bound;
		fs["bottom_bound"] >> layer_info.bottom_bound;
		fs["scale"] >> layer_info.move_scale;
		fs["offset"] >> layer_info.offset;
		l_info.push_back(layer_info);
		Q_ASSERT(layer_info.img_num_w == layer_info.offset.cols && layer_info.img_num_h == layer_info.offset.rows);
		return l_info.size() - 1;
	}	
	return -1;
}

void StitchView::paintEvent(QPaintEvent *e)
{
    QPoint ce(size().width()*scale/2, size().height()*scale/2);
    QSize s(size().width()*scale, size().height()*scale);
    view_rect = QRect(center - ce, s);
	if (view_rect.width() > l_info[layer].right_bound)
		view_rect.adjust(view_rect.width() - l_info[layer].right_bound, 0, 0, 0);
	if (view_rect.height() > l_info[layer].bottom_bound)
		view_rect.adjust(0, view_rect.height() - l_info[layer].bottom_bound, 0, 0);
    if (view_rect.left() < 0)
        view_rect.moveLeft(0);
	if (view_rect.right() > l_info[layer].right_bound - 1)
		view_rect.moveRight(l_info[layer].right_bound - 1);
    if (view_rect.top() < 0)
        view_rect.moveTop(0);
	if (view_rect.bottom() > l_info[layer].bottom_bound - 1)
		view_rect.moveBottom(l_info[layer].bottom_bound - 1);
	Q_ASSERT(view_rect.left() >= 0 && view_rect.right() < l_info[layer].right_bound &&
		view_rect.top() >= 0 && view_rect.bottom() < l_info[layer].bottom_bound);

    QImage image(size(), QImage::Format_RGB32);
    QPainter painter_mem(&image);
	painter_mem.setPen(QPen(Qt::red, 1));
	painter_mem.setBrush(QBrush(Qt::red));
	QRect screen_rect(0, 0, width(), height());
	for (int draw_order = 0; draw_order < 2; draw_order++)
    for (int y = 0; y<l_info[layer].img_num_h; y++)
		for (int x = 0; x<l_info[layer].img_num_w; x++) {
		if ((l_info[layer].offset(y, x)[0] < view_rect.bottom() && 
			(y + 1 == l_info[layer].img_num_h || l_info[layer].offset(y + 1, x)[0] > view_rect.top())) &&
			(l_info[layer].offset(y, x)[1] < view_rect.right() && 
			(x + 1 == l_info[layer].img_num_w || l_info[layer].offset(y, x + 1)[1] > view_rect.left())) &&
			QPoint(y,x) !=choose) {
				if (draw_order==1) {
					if (choose != QPoint(-1, -1)) {
						y = choose.y();
						x = choose.x();
					}
					else {
						y = l_info[layer].img_num_h;
						x = l_info[layer].img_num_w;
						break;
					}
				}
				MapID map_id = sxy2mapid(0, 0, x, y);				
				map<MapID, Bkimg>::iterator map_it;
				QImage *img;
				if ((map_it=cache_map.find(map_id)) == cache_map.end()) {
					char file_name[200];
					sprintf(file_name, "%s%d_%d.jpg", l_info[layer].img_path.c_str(), y + 1, x + 1);
					qDebug("loadImage, %s", file_name);
					img = new QImage(file_name);
					remove_cache_front();
					Bkimg bk_img;
					cache_list.push_back(map_id);
					bk_img.plist = cache_list.end();
					bk_img.plist--;
					bk_img.data = img;
					cache_map[map_id] = bk_img;
				}
				else {
					img = map_it->second.data;
					Q_ASSERT(*(map_it->second.plist) == map_id);
					cache_list.erase(map_it->second.plist);
					cache_list.push_back(map_id);
					map_it->second.plist = cache_list.end();
					map_it->second.plist--;
				}
				QRect valid_rect(l_info[layer].clip_l, l_info[layer].clip_u, 
					img->width() - l_info[layer].clip_l - l_info[layer].clip_r, 
					img->height() - l_info[layer].clip_u - l_info[layer].clip_d);
				QImage subimg_s = (img->copy(valid_rect)).scaled(img->width() / scale, img->height() / scale);
				QPoint offset(l_info[layer].offset(y, x)[1], l_info[layer].offset(y, x)[0]);
				QRect img_rect((offset - view_rect.topLeft()) / scale, subimg_s.size());
				QRect target_rect = img_rect & screen_rect;
				QRect src_rect(target_rect.topLeft() - img_rect.topLeft(), 
					target_rect.bottomRight() - img_rect.topLeft());
				painter_mem.drawImage(target_rect, subimg_s, src_rect);		
				qDebug("drawImage, (%d,%d) (%d, %d, w=%d, h=%d) -> (%d, %d, w=%d,h=%d)", y, x,
					src_rect.top(), src_rect.left(), src_rect.width(), src_rect.height(),
					target_rect.top(), target_rect.left(), target_rect.width(), target_rect.height());
				if (view_rect.contains(offset)) 
					painter_mem.drawEllipse(target_rect.topLeft(), 3, 3);
				if (draw_order == 1) {
					y = l_info[layer].img_num_h;
					x = l_info[layer].img_num_w;
					break;
				}
            }
        }

	QPainter painter(this);
	painter.drawImage(QPoint(0, 0), image);
}

void StitchView::keyPressEvent(QKeyEvent *e)
{
	int step;
	switch (e->key()) {
	case Qt::Key_Left:
		step = view_rect.width() * step_para / 10;
		view_rect.moveLeft(max(0, view_rect.left() - step));
		center = view_rect.center();
		break;
	case Qt::Key_Up:
		step = view_rect.height() * step_para / 10;
		view_rect.moveTop(max(0, view_rect.top() - step));
		center = view_rect.center();
		break;
	case Qt::Key_Right:
		step = view_rect.width() * step_para / 10;
		view_rect.moveRight(min(l_info[layer].right_bound-1, view_rect.right() + step));
		center = view_rect.center();
		break;
	case Qt::Key_Down:
		step = view_rect.height() * step_para / 10;
		view_rect.moveBottom(min(l_info[layer].bottom_bound - 1, view_rect.bottom() + step));
		center = view_rect.center();
		break;
	case Qt::Key_PageUp:
		if (scale<max_scale)
			scale = scale * 2;
		break;
	case Qt::Key_PageDown:
		if (scale>min_scale)
			scale = scale / 2;
		break;
	default:
		QWidget::keyPressEvent(e);
		return;
	}

	qDebug("Key press, s=%d, center=(%d,%d)", scale, center.x(), center.y());
	update();
}

void StitchView::mouseMoveEvent(QMouseEvent *event)
{
	QPoint mouse_point(event->localPos().x(), event->localPos().y());
	mouse_point = mouse_point * scale + view_rect.topLeft();
	for (int y = 0; y<l_info[layer].img_num_h; y++)
		for (int x = 0; x<l_info[layer].img_num_w; x++)
			if (l_info[layer].offset(y, x)[0] < mouse_point.y() && l_info[layer].offset(y + 1, x)[0] > mouse_point.y() &&
				l_info[layer].offset(y, x)[1] < mouse_point.x() && l_info[layer].offset(y, x + 1)[1] > mouse_point.x()) {
				may_choose.setX(x);
				may_choose.setY(y);
				break;
			}
	emit MouseChange(may_choose);
	QWidget::mouseMoveEvent(event);
}

void StitchView::mouseReleaseEvent(QMouseEvent *event)
{
	if (choose == may_choose)
		choose = QPoint(-1, -1);
	else
		choose = may_choose;
	update();
	QWidget::mouseReleaseEvent(event);
}

void StitchView::remove_cache_front()
{
	MapID id = cache_list.front();
	cache_list.erase(cache_list.begin());
	if (id == INVALID_MAP_ID)
		return;

	map<MapID, Bkimg>::iterator it = cache_map.find(id);
	Q_ASSERT(it != cache_map.end());
	delete it->second.data;
	cache_map.erase(it);
}