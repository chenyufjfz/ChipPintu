#ifndef STITCHVIEW_H
#define STITCHVIEW_H

#include <QWidget>
#include <vector>
#include <QImage>
#include <map>
#include <list>
#include <string>
#include <QKeyEvent>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;

typedef unsigned long long MapID;
#define INVALID_MAP_ID 0xffffffffffffffff


typedef struct {
	list <MapID>::iterator plist;
	QImage * data;
} Bkimg;

struct LayerInfo {
	int img_num_w, img_num_h;
	int clip_l, clip_r, clip_u, clip_d;
	int move_scale;
	int right_bound, bottom_bound;
	string img_path;
	Mat_<Vec2i> offset;
};

class StitchView : public QWidget
{
    Q_OBJECT
public:
    explicit StitchView(QWidget *parent = 0);
    static MapID sxy2mapid(unsigned char layer, unsigned char scale, unsigned short x, unsigned short y) {
        MapID ret;
        ret = layer;
        ret = ret <<8;
        ret |= scale;
        ret = ret <<16;
        ret |= x;
        ret = ret <<16;
        ret |= y;
        return ret;
    }

    static void mapid2sxy(MapID m, unsigned char &layer, unsigned char & scale, unsigned short & x, unsigned short &y)
    {
        layer = (m >>40) & 0xff;
        scale = (m >>32) & 0xff;
        x = (m>>16) & 0xffff;
        y = m & 0xffff;
    }

signals:
	void MouseChange(QPoint point);

public slots:

protected:
    void paintEvent(QPaintEvent *e);
    void keyPressEvent(QKeyEvent *e);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	int load_layer_info(string layer_cfg);
	void remove_cache_front();

protected:
	vector<LayerInfo> l_info;
	int scale, cache_size;
    unsigned char layer;
	QPoint choose, may_choose;
    QRect view_rect;
    QPoint center;
    list<MapID> cache_list;
	map<MapID, Bkimg> cache_map;
};

#endif // STITCHVIEW_H
