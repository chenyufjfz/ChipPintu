#ifndef STITCHVIEW_H
#define STITCHVIEW_H

#include <QWidget>
#include <vector>
#include <QImage>
#include <map>
#include <list>
#include <string>
#include <QKeyEvent>
#include <QtConcurrent>
#include "featext.h"
#include "bundleadjust.h"
using namespace std;
using namespace cv;

typedef unsigned long long MapID;
typedef unsigned long long MapID2;
#define INVALID_MAP_ID 0xffffffffffffffff


struct BkEncimg {
	list <MapID>::iterator plist;
	vector<uchar> data;
};

struct BkDecimg {
	list <MapID2>::iterator plist;
	QImage data;
	BkDecimg() {}
	BkDecimg(list <MapID2>::iterator _plist, QImage _data) {
		plist = _plist;
		data = _data;
	}
};

class StitchView : public QWidget
{
    Q_OBJECT
public:
    explicit StitchView(QWidget *parent = 0);
    static MapID lxy2mapid(unsigned char layer, unsigned short x, unsigned short y) {
        MapID ret;
        ret = layer;
        ret = ret <<16;
        ret |= x;
        ret = ret <<16;
        ret |= y;
        return ret;
    }

    static void mapid2lxy(MapID m, unsigned char &layer, unsigned short & x, unsigned short &y)
    {
        layer = (m >>32) & 0xff;
        x = (m>>16) & 0xffff;
        y = m & 0xffff;
    }

	static MapID2 slxy2mapid(unsigned scale, unsigned char layer, unsigned short x, unsigned short y) {
		MapID ret;
		ret = scale;
		ret = ret << 8;
		ret |= layer;
		ret = ret << 16;
		ret |= x;
		ret = ret << 16;
		ret |= y;
		return ret;
	}

	static void mapid2slxy(MapID2 m, unsigned & scale, unsigned char &layer, unsigned short & x, unsigned short &y)
	{
		scale = (m >> 40) & 0xff;
		layer = (m >> 32) & 0xff;
		x = (m >> 16) & 0xffff;
		y = m & 0xffff;
	}

signals:
	void MouseChange(QPoint point);
	void notify_progress(float progress);

public slots:

protected:
    void paintEvent(QPaintEvent *e);
    void keyPressEvent(QKeyEvent *e);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void remove_cache_front();
	void timerEvent(QTimerEvent *e);

protected:
	vector<ConfigPara> cpara;

	//Following is for drawing layer and rect
	double scale; //current scale, should be bigger than cpara.rescale /2
    unsigned char layer; //current layer, use cpara[layer] for drawing
	QPoint choose, may_choose;
	QRect view_rect; //view_rect unit is pixel in original file image
	QPoint center;
	//upper is for drawing layer and rect

	//Following is for encoder and decode image cache
	map <MapID2, BkDecimg> preimg_map; //store decode image, its number is small, because each BkDecimg is big
	list<MapID2> preimg_list;
	map<MapID, BkEncimg> cache_map; //store encode image,  its number is big, because each BkEncimg is small
    list<MapID> cache_list;		
	int cache_size, preimg_size;
	//Upper is for  encoder and decode image cache

	//Following is for feature compute and store
	FeatExt feature;
	vector<TuningPara> tpara;
	vector<string> feature_file; //feature file name in disk
	int feature_layer; //feature storts which layer
	QFuture<string> compute_feature;
	int compute_feature_timer;
	//upper is for feature compute and store

	//Following is for bundleadjust
	int bundle_adjust_timer;
	BundleAdjust ba;
	QFuture<void> bundle_adjust_future;
	Mat_<Vec2i> adjust_offset;
	//upper is for bundleadjust
public:
	//if _layer==-1, means current layer, if _layer==get_layer_num(), add new layer
	int set_config_para(int _layer, const ConfigPara & _cpara);
	//if _layer==-1, means get config of current layer
	int get_config_para(int _layer, ConfigPara & _cpara);
	//if _layer==-1, means set tune of current layer
	int set_tune_para(int _layer, const TuningPara & _tpara);
	//if _layer==-1, means get tune of current layer
	int get_tune_para(int _layer, TuningPara & _tpara);
	//From cpara, tpara, generate new FeatExt. if _layer==-1, means get tune of current layer
	int compute_new_feature(int _layer);
	//From FeatExt, compute new cpara.offset
	int optimize_offset(int _layer);
	int get_layer_num() { return (int)cpara.size(); }
	int get_current_layer() { return layer; }
	int set_current_layer(int _layer);
	//if _layer==-1, means erase current layer
	int delete_layer(int _layer);
	void write_file(string file_name);
	int read_file(string file_name);
};

#endif // STITCHVIEW_H
