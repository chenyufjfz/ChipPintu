#include "stitchview.h"
#include <QPainter>
#include <algorithm>
#include <QtCore/QSharedPointer>
#include <fstream>
#include <QMessageBox>
#include <QDateTime>

const int step_para = 3;
const int max_scale = 8;

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

void thread_bundle_adjust(BundleAdjust * ba, FeatExt * feature, Mat_<Vec2i> *offset)
{
	*offset = ba->arrange(*feature, -1, -1);
}

StitchView::StitchView(QWidget *parent) : QWidget(parent)
{
    scale = 8;
	center = QPoint(0, 0);
    layer = -1;
	edge_cost = 0;
	feature_layer = -1;
	draw_corner = true;
	for (int i=0; i<3; i++)
		choose[i] = QPoint(-1, -1);
    resize(1800, 1000);
	ri.set_dst_wide(1024);
	setMouseTracking(true);
	setAutoFillBackground(false);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setAttribute(Qt::WA_NoSystemBackground, true);
}

void StitchView::paintEvent(QPaintEvent *)
{
	if (cpara.empty())
		return;
	Q_ASSERT(layer < (int) cpara.size());
    QPoint ce(size().width()*scale/2, size().height()*scale/2);
    QSize s(size().width()*scale, size().height()*scale);
    view_rect = QRect(center - ce, s);
	if (view_rect.width() > cpara[layer].right_bound())
		view_rect.adjust(view_rect.width() - cpara[layer].right_bound(), 0, 0, 0);
	if (view_rect.height() > cpara[layer].bottom_bound())
		view_rect.adjust(0, view_rect.height() - cpara[layer].bottom_bound(), 0, 0);
    if (view_rect.left() < 0)
        view_rect.moveLeft(0);
	if (view_rect.right() > cpara[layer].right_bound() - 1)
		view_rect.moveRight(cpara[layer].right_bound() - 1);
    if (view_rect.top() < 0)
        view_rect.moveTop(0);
	if (view_rect.bottom() > cpara[layer].bottom_bound() - 1)
		view_rect.moveBottom(cpara[layer].bottom_bound() - 1);
	Q_ASSERT(view_rect.left() >= 0 && view_rect.right() < cpara[layer].right_bound() &&
		view_rect.top() >= 0 && view_rect.bottom() < cpara[layer].bottom_bound());

    QImage image(size(), QImage::Format_RGB32);
	image.fill(QColor(0, 0, 0));
    QPainter painter(&image);
	painter.setPen(QPen(Qt::red, 1));
	painter.setBrush(QBrush(Qt::red));
	QRect screen_rect(0, 0, width(), height());	

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
	ri.render_img(map_id, imgs, draw_order, load_img_opt[layer]);

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
	
	if (draw_corner) {
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

		Point lt = find_src_map(cpara[layer], Point(minx, miny), ri.get_src_img_size(layer), 1);
		Point rb = find_src_map(cpara[layer], Point(maxx + 1, maxy + 1), ri.get_src_img_size(layer), 0);

		for (int y = lt.y; y <= rb.y; y++)
		for (int x = lt.x; x <= rb.x; x++) {
			Point src_corner(cpara[layer].offset(y, x)[1], cpara[layer].offset(y, x)[0]);
			QPoint dst_corner = TOQPOINT(mxy.src2dst(src_corner));
			if (view_rect.contains(dst_corner)) 
				painter.drawEllipse((dst_corner - view_rect.topLeft()) / scale, 2, 2);
		}
	}
	QPainter paint(this);
	paint.drawImage(QPoint(0, 0), image);
}

void StitchView::keyPressEvent(QKeyEvent *e)
{
	if (cpara.empty())
		return;
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
		view_rect.moveRight(min(cpara[layer].right_bound() - 1, view_rect.right() + step));
		center = view_rect.center();
		break;
	case Qt::Key_Down:
		step = view_rect.height() * step_para / 10;
		view_rect.moveBottom(min(cpara[layer].bottom_bound() - 1, view_rect.bottom() + step));
		center = view_rect.center();
		break;
	case Qt::Key_PageUp:
		if (scale < max_scale)
			scale = scale * 2;
		break;
	case Qt::Key_PageDown:
		if (scale * 2 > cpara[layer].rescale)
			scale = scale / 2;
		break;
	case Qt::Key_0:
		if (cpara.size() > 0)
			layer = 0;		
		break;
	case Qt::Key_1:
		if (cpara.size() > 1)
			layer = 1;
		break;
	case Qt::Key_2:
		if (cpara.size() > 2)
			layer = 2;
		break;
	case Qt::Key_3:
		if (cpara.size() > 3)
			layer = 3;
		break;
	case Qt::Key_4:
		if (cpara.size() > 4)
			layer = 4;
		break;
	case Qt::Key_5:
		if (cpara.size() > 5)
			layer = 5;
		break;
	case Qt::Key_6:
		if (cpara.size() > 6)
			layer = 6;
		break;
	case Qt::Key_7:
		if (cpara.size() > 7)
			layer = 7;
		break;
	default:
		QWidget::keyPressEvent(e);
		return;
	}

	qDebug("Key press, l=%d, s=%d, center=(%d,%d)", layer, scale, center.x(), center.y());
	update();
}

void StitchView::mouseMoveEvent(QMouseEvent *event)
{
	if (cpara.empty())
		return;
	QPoint mouse_point(event->localPos().x(), event->localPos().y());
	Point offset;
	mouse_point = mouse_point * scale + view_rect.topLeft();

	QPoint prev_may_choose = may_choose;	
	MapXY mxy = ri.get_mapxy(layer);
	Point src_point = mxy.dst2src(TOPOINT(mouse_point));
	may_choose = TOQPOINT(find_src_map(cpara[layer], src_point, ri.get_src_img_size(layer), 0));

	if (may_choose != prev_may_choose && feature.is_valid()) {
		const EdgeDiff * ed = feature.get_edge(may_choose.y(), may_choose.x(), prev_may_choose.y(), prev_may_choose.x());
		if (ed) {
			Point o0(cpara[layer].offset(may_choose.y(), may_choose.x())[1], 
				cpara[layer].offset(may_choose.y(), may_choose.x())[0]);
			Point o1(cpara[layer].offset(prev_may_choose.y(), prev_may_choose.x())[1],
				cpara[layer].offset(prev_may_choose.y(), prev_may_choose.x())[0]);
			Point oo = (o1.y + o1.x > o0.y + o0.x) ? o1 - o0 : o0 - o1;
			edge_cost = ed->get_diff(oo, cpara[layer].rescale);
		}
	}
	char info[200];
	sprintf(info, "x=%d,y=%d,sx=%d,sy=%d,ix=%d,iy=%d", mouse_point.x(),
		mouse_point.y(), src_point.x, src_point.y, may_choose.x(), may_choose.y());
	emit MouseChange(info);
	QWidget::mouseMoveEvent(event);
}

void StitchView::mouseReleaseEvent(QMouseEvent *event)
{
	if (cpara.empty())
		return;
	if (choose[2] != may_choose) {
		choose[0] = choose[1];
		choose[1] = choose[2];
		choose[2] = may_choose;
	}
	update();
	QWidget::mouseReleaseEvent(event);
}

void StitchView::timerEvent(QTimerEvent *e)
{
	if (e->timerId() == compute_feature_timer) {
		if (compute_feature.isFinished()) {
			killTimer(compute_feature_timer);
			emit notify_progress(0);			
			string filename = compute_feature.result();
			feature_file[feature_layer] = filename;
			QMessageBox::information(this, "Info", "Prepare finish");
		} else
			emit notify_progress(feature.get_progress());
	}

	if (e->timerId() == bundle_adjust_timer) {
		if (bundle_adjust_future.isFinished()) {
			killTimer(bundle_adjust_timer);
			emit notify_progress(0);
			cpara[feature_layer].offset = adjust_offset;
			QMessageBox::information(this, "Info", "Optimize offset finish");
			update();
		} else
			emit notify_progress(ba.get_progress());
	}
}

//if _layer==-1, means current layer
int StitchView::set_config_para(int _layer, const ConfigPara & _cpara)
{
	Q_ASSERT(cpara.size() == tpara.size() && cpara.size() == feature_file.size());
	if (_layer == -1)
		_layer = layer;
	if (_layer > cpara.size() || _layer < 0)
		return -1;
	if (_layer == cpara.size()) {
		cpara.push_back(_cpara);
		feature_file.push_back(string());
		load_img_opt.push_back(CV_LOAD_IMAGE_UNCHANGED);
		if (tpara.empty()) {
			TuningPara _tpara;
			tpara.push_back(_tpara);
		}
		else
			tpara.push_back(tpara[0]);
	}
	else
		cpara[_layer] = _cpara;
	ri.set_cfg_para(_layer, _cpara);
	qInfo("set config, l=%d, s=%d, nx=%d, ny=%d", _layer, _cpara.rescale, _cpara.img_num_w, _cpara.img_num_h);
	return 0;
}

//if _layer==-1, means current layer
int StitchView::get_config_para(int _layer, ConfigPara & _cpara)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer >= cpara.size() || _layer < 0)
		return -1;
	_cpara = cpara[_layer];
	return 0;
}

//if _layer==-1, means current layer
int StitchView::set_tune_para(int _layer, const TuningPara & _tpara)
{
	Q_ASSERT(cpara.size() == tpara.size() && cpara.size() == feature_file.size());
	if (_layer == -1)
		_layer = layer;
	if (_layer >= tpara.size() || _layer < 0)
		return -1;
	tpara[_layer] = _tpara;
	return 0;
}

//if _layer==-1, means current layer
int StitchView::get_tune_para(int _layer, TuningPara & _tpara)
{
	if (_layer == -1)
		_layer = layer;
	if (_layer >= tpara.size() || _layer < 0)
		return -1;
	_tpara = tpara[_layer];
	return 0;
}

int StitchView::compute_new_feature(int _layer)
{
	Q_ASSERT(cpara.size() == tpara.size() && cpara.size() == feature_file.size());
	if (_layer == -1)
		_layer = layer;
	if (_layer >= cpara.size() || _layer < 0)
		return -1;
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is already running, wait it finish");
		return -2;
	}
	if (bundle_adjust_future.isRunning()) {
		QMessageBox::information(this, "Info", "Optimize offset is running, wait it finish and redo Prepare");
		return -3;
	}

	feature.set_cfg_para(cpara[_layer]);
	feature.set_tune_para(tpara[_layer]);
	feature_layer = _layer;
	compute_feature = QtConcurrent::run(thread_generate_diff, &feature, _layer);
	compute_feature_timer = startTimer(2000);
	return 0;
}

int StitchView::optimize_offset(int _layer)
{
	Q_ASSERT(cpara.size() == tpara.size() && cpara.size() == feature_file.size());
	if (_layer == -1)
		_layer = layer;
	if (_layer >= cpara.size() || _layer < 0)
		return -1;
	if (compute_feature.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is running, wait it finish and redo optimize offset");
		return -2;
	}
	if (bundle_adjust_future.isRunning()) {
		QMessageBox::information(this, "Info", "Optimize offset is running, wait it finish");
		return -3;
	}
	if (feature_layer != _layer) {
		if (feature_file[_layer].size() < 2) {//empty
			QMessageBox::information(this, "Info", "feature is empty, click Prepare first");
			return -4;
		}
		feature.read_diff_file(feature_file[_layer]);
		feature_layer = _layer;
	}
	
#if 0
	bundle_adjust_future = QtConcurrent::run(thread_bundle_adjust, &ba, &feature, &adjust_offset);
	bundle_adjust_timer = startTimer(2000);
#else
	thread_bundle_adjust(&ba, &feature, &adjust_offset);
	cpara[feature_layer].offset = adjust_offset;
	ri.set_cfg_para(feature_layer, cpara[feature_layer]);
	if (feature_layer == layer)
		update();
#endif
	return 0;
}

int StitchView::set_current_layer(int _layer) {
	if (_layer < get_layer_num() && _layer >= 0)
		layer = _layer;
	else
		return -1;
	update();
	return 0;
}

int StitchView::delete_layer(int _layer)
{
	Q_ASSERT(cpara.size() == tpara.size() && cpara.size() == feature_file.size());
	if (compute_feature.isRunning() || bundle_adjust_future.isRunning()) {
		QMessageBox::information(this, "Info", "Prepare is running, can't delete layer");
		return -2;
	}
	if (_layer == -1)
		_layer = layer;
	if (_layer >= cpara.size() || _layer < 0)
		return -1;
	cpara.erase(cpara.begin() + _layer);
	tpara.erase(tpara.begin() + _layer);
	feature_file.erase(feature_file.begin() + _layer);
	if (layer >= _layer) {
		layer--;
		update();
	}
	return 0;
}

void StitchView::write_file(string file_name)
{
	Q_ASSERT(cpara.size() == feature_file.size());
	FileStorage fs(file_name, FileStorage::WRITE);

	fs << "layer_num" << (int) cpara.size();
	for (int i = 0; i < (int)cpara.size(); i++) {
		char name[30];
		sprintf(name, "cpara%d", i);
		fs << name << cpara[i];
		sprintf(name, "tpara%d", i);
		fs << name << tpara[i];
		sprintf(name, "diff_file%d", i);
		fs << name << feature_file[i];
		sprintf(name, "load_img_opt%d", i);
		fs << name << load_img_opt[i];
		sprintf(name, "mapxy%d", i);
		fs << name << ri.get_mapxy(i);
	}
	fs << "dst_w" << ri.get_dst_wide();
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
	fs.release();
}

int StitchView::read_file(string file_name)
{
	FileStorage fs(file_name, FileStorage::READ);
	if (fs.isOpened()) {
		int layer_num;
		layer_num = (int)fs["layer_num"];
		cpara.clear();
		cpara.resize(layer_num);
		tpara.resize(layer_num);
		feature_file.resize(layer_num);
		for (int i = 0; i < (int)cpara.size(); i++) {
			char name[30];
			sprintf(name, "cpara%d", i);
			fs[name] >> cpara[i];
			ri.set_cfg_para(i, cpara[i]);
			sprintf(name, "tpara%d", i);
			fs[name] >> tpara[i];
			sprintf(name, "diff_file%d", i);
			fs[name] >> feature_file[i];
			sprintf(name, "load_img_opt%d", i);
			fs[name] >> load_img_opt[i];
			MapXY mxy;
			sprintf(name, "mapxy%d", i);
			fs[name] >> mxy;
			ri.set_mapxy(i, mxy);
		}
		int dst_w;
		fs["dst_w"] >> dst_w;
		ri.set_dst_wide(dst_w);
		Point ce0, ce1, ce2, ct;
		fs["layer"] >> layer;
		fs["scale"] >> scale;
		fs["center"] >> ct;
		fs["choose"] >> ce0;
		fs["choose1"] >> ce1;
		fs["choose2"] >> ce2;
		feature.read_diff_file(feature_file[layer]);
		center = QPoint(ct.x, ct.y);
		choose[0] = QPoint(ce0.x, ce0.y);
		choose[1] = QPoint(ce1.x, ce1.y);
		choose[2] = QPoint(ce2.x, ce2.y);
		fs.release();
		feature_layer = -1;
		return 0;
	}
	return -1;
}