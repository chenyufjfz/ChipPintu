#ifndef NAVIGATEVIEW_H
#define NAVIGATEVIEW_H

#include <QWidget>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
using namespace cv;

class NavigateView : public QWidget
{
    Q_OBJECT
protected:
	int width, height;
	vector<Point> ns;
	vector<int> ns_state;
	QRect view_rect;

protected:
	void paintEvent(QPaintEvent *e);
	void mousePressEvent(QMouseEvent *event);

public:
    explicit NavigateView(QWidget *parent = 0);
	void set_boundary(int _width, int _height); //in pixel
	void set_nails(const vector<Point> & _ns, const vector<int> & _state);
	void set_viewrect(QRect _rect);

signals:
	void update_center(QPoint center);

public slots:
};

#endif // NAVIGATEVIEW_H
