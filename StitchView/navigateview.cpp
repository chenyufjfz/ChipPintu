#include "navigateview.h"
#include <QPainter>
#include <QKeyEvent>

NavigateView::NavigateView(QWidget *parent) : QWidget(parent)
{
	QPalette pal(palette());
	setAutoFillBackground(false);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setAttribute(Qt::WA_NoSystemBackground, true);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint | Qt::Tool);
	setAttribute(Qt::WA_ShowWithoutActivating, true);
	setFocusPolicy(Qt::NoFocus);
	width = 0;
	height = 0;
}

void NavigateView::paintEvent(QPaintEvent *)
{
	QImage image(size(), QImage::Format_RGB32);
	image.fill(QColor(0, 0, 0));
	if (width != 0 && height != 0) {
		QPainter paint(&image);
		double w = size().width();
		double h = size().height();
		double zr = min(w / width, h / height);
		paint.setBrush(QBrush(Qt::gray));
		paint.drawRect(QRect(0, 0, width*zr, height * zr));
		//draw nail
		for (int i = 0; i < (int)ns.size(); i++) {
			if (ns_state[i])
				paint.setPen(QPen(Qt::blue, 1));
			else
				paint.setPen(QPen(Qt::red, 1));
			QPoint a((long long)ns[i].x * zr, (long long)ns[i].y * zr);
			paint.drawLine(a - QPoint(2, 2), a + QPoint(2, 2));
			paint.drawLine(a - QPoint(-2, 2), a + QPoint(-2, 2));
		}
		//draw view rect
		paint.setPen(QPen(Qt::yellow, 1));
		paint.setBrush(Qt::NoBrush);
		w = view_rect.width() * zr;
		h = view_rect.height() * zr;
		QRect rect(view_rect.x() * zr, view_rect.y() * zr, max(1, (int) w), max(1, (int) h));
		paint.drawRect(rect);
	}
	QPainter painter(this);
	painter.drawImage(QPoint(0, 0), image);
}

void NavigateView::mousePressEvent(QMouseEvent *event)
{
	if (width == 0 || height == 0)
		return;
	double w = size().width();
	double h = size().height();
	double zr = min(w / width, h / height);
	Point center(event->localPos().x(), event->localPos().y());
	center.x = center.x / zr;
	center.y = center.y / zr;
	qInfo("NView move center to (%d,%d)", center.x, center.y);
	update_center(QPoint(center.x, center.y));
	QWidget::mousePressEvent(event);
}

void NavigateView::set_boundary(int _width, int _height) //in pixel
{
	width = _width;
	height = _height;
	update();
}

void NavigateView::set_nails(const vector<Point> & _ns, const vector<int> & _state)
{
	CV_Assert(_ns.size() == _state.size());
	ns = _ns;
	ns_state = _state;
	update();
}
	
void NavigateView::set_viewrect(QRect _rect)
{
	view_rect = _rect;
	update();
}