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
#include "renderimage.h"

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

signals:
	void MouseChange(QString info);
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
	QPoint choose[3], may_choose;
	QRect view_rect; //view_rect unit is pixel for dst file image
	QPoint center;
	int edge_cost;
	vector<int> load_img_opt;
	RenderImage ri;
	bool draw_corner;
	//upper is for drawing layer and rect
	
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
