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
#include "bundleadjustinf.h"
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

struct LayerFeature {
	ConfigPara cpara;
	TuningPara tpara;
	FeatExt feature;
	string feature_file; //feature file name in disk
    string layer_name;
	Mat_<unsigned long long> corner_info;
	Mat_<int> fix_edge[2];
	vector<unsigned> get_fix_img(unsigned img_idx, int bind_flag) {
		vector<unsigned> ret;
		vector<unsigned> queue;
		
		queue.push_back(img_idx);
		while (!queue.empty()) {
			unsigned idx = queue.back();
			ret.push_back(idx);
			queue.pop_back();
			int y = IMG_Y(idx);
			int x = IMG_X(idx);
			if (y > 0 && fix_edge[0](y - 1, x) & bind_flag) { 
				unsigned idx1 = MAKE_IMG_IDX(x, y - 1);
				if (find(ret.begin(), ret.end(), idx1) == ret.end())
					queue.push_back(idx1);
			}
			if (y + 1 < cpara.img_num_h && fix_edge[0](y, x) & bind_flag) {
				unsigned idx1 = MAKE_IMG_IDX(x, y + 1);
				if (find(ret.begin(), ret.end(), idx1) == ret.end())
					queue.push_back(idx1);
			}
			if (x > 0 && fix_edge[1](y, x - 1) & bind_flag) {
				unsigned idx1 = MAKE_IMG_IDX(x - 1, y);
				if (find(ret.begin(), ret.end(), idx1) == ret.end())
					queue.push_back(idx1);
			}
			if (x + 1 < cpara.img_num_w && fix_edge[1](y, x) & bind_flag) {
				unsigned idx1 = MAKE_IMG_IDX(x + 1, y);
				if (find(ret.begin(), ret.end(), idx1) == ret.end())
					queue.push_back(idx1);
			}
		}
		return ret;
	}

	/*if fix edge is in bound return 0, else return direction
	1: x<0
	2: x>bound
	4: y<0
	8: y>bound*/
	int check_edge(unsigned edge_idx) {
		int ret = 0;
		if (!feature.is_valid())
			return 0;
		int x = EDGE_X(edge_idx);
		int y = EDGE_Y(edge_idx);
		int e = EDGE_E(edge_idx);
		if (e == 0 && (y >= cpara.img_num_h - 1 || x >= cpara.img_num_w))
			return 0;
		if (e == 1 && (y >= cpara.img_num_h || x >= cpara.img_num_w - 1))
			return 0;
		if (fix_edge[e](y, x) == 0)
			return 0;
		Point o0(cpara.offset(y, x)[1], cpara.offset(y, x)[0]);
		Point o1(cpara.offset(y + 1, x)[1], cpara.offset(y + 1, x)[0]);
		Point o2(cpara.offset(y, x + 1)[1], cpara.offset(y, x + 1)[0]);
		Point oo = (e == 0) ? o1 - o0 : o2 - o0;
		const EdgeDiff * ed = (e == 0) ? feature.get_edge(y, x, y + 1, x) : feature.get_edge(y, x, y, x + 1);
		Point shift = oo - ed->offset;
		CV_Assert(shift.x % cpara.rescale == 0 && shift.y % cpara.rescale == 0);
		shift.x = shift.x / cpara.rescale;
		shift.y = shift.y / cpara.rescale;

		if (shift.x < 0)
			ret |= 1;
		if (shift.x >= ed->dif.cols)
			ret |= 2;
		if (shift.y < 0)
			ret |= 4;
		if (shift.y >= ed->dif.rows)
			ret |= 8;

		return ret;
	}
	int check_img_offset(unsigned img_idx) {
		if (check_edge(img_idx) || check_edge(img_idx | 0x80000000))
			return 1;
		int x = IMG_X(img_idx);
		int y = IMG_Y(img_idx);
		if (y > 0 && check_edge(MAKE_EDGE_IDX(x, y - 1, 0)))
			return 1;
		if (x > 0 && check_edge(MAKE_EDGE_IDX(x - 1, y, 1)))
			return 1;
		return 0;
	}
};

class Nail {
public:
	LayerFeature * lf0;
	LayerFeature * lf1;
	Point p0, p1;
	Nail() {
		lf0 = NULL;
		lf1 = NULL;
	}
	Nail(LayerFeature * _lf0, LayerFeature * _lf1, Point _p0, Point _p1) {
		lf0 = _lf0;
		lf1 = _lf1;
		p0 = _p0;
		p1 = _p1;
	}
	bool within_range(LayerFeature *lf, Point p, int range) {
		if (lf0 == lf && abs(p0.x - p.x) < range && abs(p0.y - p.y) < range)
			return true;
		if (lf1 == lf && abs(p1.x - p.x) < range && abs(p1.y - p.y) < range)
			return true;
		return false;
	}
	bool within_range(Nail n, int range) {
		if (lf0 == n.lf0 && lf1 == n.lf1 &&
			abs(p0.x - n.p0.x) < range && abs(p0.y - n.p0.y) < range &&
			abs(p1.x - n.p1.x) < range && abs(p1.y - n.p1.y) < range)
			return true;
		if (lf0 == n.lf1 && lf1 == n.lf0 &&
			abs(p0.x - n.p1.x) < range && abs(p0.y - n.p1.y) < range &&
			abs(p1.x - n.p0.x) < range && abs(p1.y - n.p0.y) < range)
			return true;
		return false;
	}
	void swap_order() {
		swap(lf0, lf1);
		swap(p0, p1);
	}

	bool operator==(const Nail & n) {
		return (lf0 == n.lf0 && lf1 == n.lf1 && p0 == n.p0 && p1 == n.p1 ||
			lf0 == n.lf1 && lf1 == n.lf0 && p0 == n.p1 && p1 == n.p0);
	}
};

class StitchView : public QWidget
{
    Q_OBJECT
public:
    explicit StitchView(QWidget *parent = 0);
	~StitchView();
signals:
	void MouseChange(QString info);
	void notify_progress(float progress);
	void title_change(QString title);

public slots:

protected:
    void paintEvent(QPaintEvent *e);
    void keyPressEvent(QKeyEvent *e);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void remove_cache_front();
	void timerEvent(QTimerEvent *e);

protected:
	//Following is for drawing layer and rect
	double scale; //current scale, should be bigger than cpara.rescale /2
    unsigned char layer; //current layer, use cpara[layer] for drawing
	QPoint choose[3], may_choose;
	QRect view_rect; //view_rect unit is pixel for dst file image
	QPoint center;
	int edge_cost;
	Point minloc_shift;
	RenderImage ri;
	int draw_corner;
	//upper is for drawing layer and rect
	
	//Following is for feature compute and store
	FeatExt computing_feature;
	vector<LayerFeature *> lf;
	int feature_layer; //computing_feature is for which layer
	QFuture<string> compute_feature;
	int compute_feature_timer;
	//upper is for feature compute and store

	//following is for grid drawing
	double xoffset, yoffset, xgrid_size, ygrid_size;
	bool draw_grid;
	//upper is for grid drawing

	//Following is for bundleadjust
	BundleAdjustInf * ba;
	Mat_<Vec2i> adjust_offset;
	int auto_save;
	//upper is for bundleadjust

	//Following is for nail
	vector<Nail> nails;
	bool add_nail(Nail nail);
	void del_nail(Nail nail);
	void get_one_layer_nails(LayerFeature * lf, vector<Nail> &ns);
	void del_one_layer_nails(LayerFeature * lf);
	Nail search_nail(LayerFeature * lf, Point p, int range);
	vector<double> generate_mapxy();
	int which_layer(LayerFeature * l);
	Nail cur_nail;
	//Upper is for nail

	//follow is for draw mouse
	int mouse_state;
	QPoint cur_mouse_point;
	//upper is for draw mouse
public:
	//if _layer==-1, means current layer, if _layer==get_layer_num(), add new layer
	int set_config_para(int _layer, const ConfigPara & _cpara);
	//if _layer==-1, means get config of current layer
	int get_config_para(int _layer, ConfigPara & _cpara);
	//if _layer==-1, means set tune of current layer
	int set_tune_para(int _layer, const TuningPara & _tpara);
	//if _layer==-1, means get tune of current layer
	int get_tune_para(int _layer, TuningPara & _tpara);
	//if _layer==-1, means get tune of current layer
	void set_mapxy_dstw(int _layer, const MapXY & _mapxy, int _dst_w);
	//if _layer==-1, means get tune of current layer
	MapXY get_mapxy(int _layer);
	int get_dst_wide();
	void set_grid(double _xoffset, double _yoffset, double _xgrid_size, double _ygrid_size);
	void get_grid(double & _xoffset, double & _yoffset, double & _xgrid_size, double & _ygrid_size);
	//From cpara, tpara, generate new FeatExt. if _layer==-1, means get tune of current layer
	int compute_new_feature(int _layer);
	//From FeatExt, compute new cpara.offset
	int optimize_offset(int _layer);
	int get_layer_num() { return (int)lf.size(); }
	void goto_xy(int x, int y);
	int get_current_layer() { return layer; }
	int set_current_layer(int _layer);
	string get_layer_name(int _layer) {
		if (_layer == -1)
			_layer = layer;
		if (_layer >= lf.size() || _layer < 0)
			return string();
		else
			return lf[_layer]->layer_name;
	}
	void set_layer_name(int _layer, string name) {
		if (_layer == -1)
			_layer = layer;
		if (_layer >= lf.size() || _layer < 0)
			return;
		else
			lf[_layer]->layer_name = name;
	}
	void to_state_add_nail();
	void to_state_change_nail();
	int output_layer(int _layer, string pathname);
	//if _layer==-1, means erase current layer
	int delete_layer(int _layer);
	int layer_up(int _layer);
	int layer_down(int _layer);
	void write_file(string file_name);
	int read_file(string file_name);
};

#endif // STITCHVIEW_H
