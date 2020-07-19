#ifndef STITCHVIEW_H
#define STITCHVIEW_H

#include <QWidget>
#include <vector>
#include <QImage>
#include <map>
#include <set>
#include <list>
#include <string>
#include <functional>
#include <algorithm>
#include <QKeyEvent>
#include <QtConcurrent>
#include "featext.h"
#include "bundleadjustinf.h"
#include "renderimage.h"
#include "navigateview.h"
struct LayerFeature;
#include "corneredge.h"

using namespace std;
using namespace cv;

typedef unsigned long long MapID;
typedef unsigned long long MapID2;
#define INVALID_MAP_ID 0xffffffffffffffff

#define FUNCTION_MASK				0xffffffff
#define ALLOW_DISPLAY_CORNER		0x00000001
#define ALLOW_YELLOW_EDGE			0x00000002
#define ALLOW_IMPORT				0x00000004
#define ALLOW_FAST_OPTIMIZE			0x00000008
#define ALLOW_TUNE					0x00000010
#define ALLOW_DISPLAY_SPAN_TREE		0x00000020
#define ALLOW_OUTPUT_DB				0x00000040
#define ALLOW_FOREVER				0x80000000
#define ALLOW_ANY_LICENSE			0x00000080

#define OUTPUT_DB			1
#define OUTPUT_PIC			0

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
	Mat_<Vec2i> corner_info; //y_bias is in vec0 31..16, x_bias is in vec0 15..0
	Mat_<int> flagb[2];
	Mat_<Vec2i> checked_edge_offset[2];  //used by corneredge
	int find_next;
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
			if (y > 0 && flagb[0](y - 1, x) & bind_flag) { 
				unsigned idx1 = MAKE_IMG_IDX(x, y - 1);
				if (find(ret.begin(), ret.end(), idx1) == ret.end() &&
					find(queue.begin(), queue.end(), idx1) == queue.end())
					queue.push_back(idx1);
			}
			if (y + 1 < cpara.img_num_h && flagb[0](y, x) & bind_flag) {
				unsigned idx1 = MAKE_IMG_IDX(x, y + 1);
				if (find(ret.begin(), ret.end(), idx1) == ret.end() &&
					find(queue.begin(), queue.end(), idx1) == queue.end())
					queue.push_back(idx1);
			}
			if (x > 0 && flagb[1](y, x - 1) & bind_flag) {
				unsigned idx1 = MAKE_IMG_IDX(x - 1, y);
				if (find(ret.begin(), ret.end(), idx1) == ret.end() &&
					find(queue.begin(), queue.end(), idx1) == queue.end())
					queue.push_back(idx1);
			}
			if (x + 1 < cpara.img_num_w && flagb[1](y, x) & bind_flag) {
				unsigned idx1 = MAKE_IMG_IDX(x + 1, y);
				if (find(ret.begin(), ret.end(), idx1) == ret.end() &&
					find(queue.begin(), queue.end(), idx1) == queue.end())
					queue.push_back(idx1);
			}
		}
		return ret;
	}

	LayerFeature() {
		find_next = 0;
	}
};

class Nail {
public:
	LayerFeature * lf0; //lf0==lf1 means absolute nail
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
	void update_center(QPoint center);
	void goto_corner(unsigned corner_idx);
	void goto_edge(unsigned edge_idx);
	void goto_nail(unsigned nail_idx);

protected:
    void paintEvent(QPaintEvent *e);
    void keyPressEvent(QKeyEvent *e);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseDoubleClickEvent(QMouseEvent * event);
	void timerEvent(QTimerEvent *e);
	void update_nview();
	void convert_nails(vector<Nail> & nsrc, vector<Nail> & nimg, int method);
	
protected:
	string project_path;
	string license;
	int choose_edge;
	//Following is for drawing layer and rect
	double scale; //current scale, should be bigger than cpara.rescale /2
    unsigned char layer; //current layer, use cpara[layer] for drawing
	QPoint choose[3];
	QRect view_rect; //view_rect unit is pixel for dst file image
	QPoint center;
	Point edge_cost, minloc_shift, edge_type;
	RenderImage * ri;
	int draw_corner;
	QRect output_rect;
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
	int auto_save;
	int span_tree_dir;
	//upper is for bundleadjust

	//Following is for nail
	vector<Nail> nails;
	bool add_nail(Nail nail);
	void del_nail(Nail nail);
	void get_absolute_nails(vector<Nail> &ns);
	void get_one_layer_nails(LayerFeature * lf, vector<Nail> &ns);
	void del_one_layer_nails(LayerFeature * lf);
	void notify_nail_info(int layer);
	Nail search_nail(LayerFeature * lf, Point p, int range);
	vector<double> generate_mapxy();
	set<unsigned> span_tree_edges;
	int which_layer(LayerFeature * l);
	void self_check_offset(int _layer);
	Nail cur_nail;
	//Upper is for nail

	//following is for navigate view
	NavigateView * nview;
	CornerEdge * ceview;
	//Upper is for navigate view

	//following is for draw mouse
	int mouse_state;
	int new_nail_num;
	QPoint cur_mouse_point;
	//upper is for draw mouse
public:
	void set_license(string _license);
	//if _layer==-1, means current layer, if _layer==get_layer_num(), add new layer
	int set_config_para(int _layer, const ConfigPara & _cpara);
	//if _layer==-1, means get config of current layer
	int get_config_para(int _layer, ConfigPara & _cpara) const;
	//if _layer==-1, means set tune of current layer
	int set_tune_para(int _layer, const TuningPara & _tpara);
	//if _layer==-1, means get tune of current layer
	int get_tune_para(int _layer, TuningPara & _tpara) const;
	//if _layer==-1, means get tune of current layer
	void set_mapxy_dstw(int _layer, const MapXY * _mapxy, int _dst_w);
	//if _layer==-1, means get tune of current layer
	MapXY * get_mapxy(int _layer);
	int get_dst_wide();
	void set_grid(double _xoffset, double _yoffset, double _xgrid_size, double _ygrid_size);
	void get_grid(double & _xoffset, double & _yoffset, double & _xgrid_size, double & _ygrid_size);
	//From cpara, tpara, generate new FeatExt. if _layer==-1, means get tune of current layer
	int compute_new_feature(int _layer);
	//From FeatExt, compute new cpara.offset
	int optimize_offset(int _layer, int optimize_option);
	int get_layer_num() { return (int)lf.size(); }
	void goto_xy(int x, int y);
    void clear_fix_edge(int _layer);
    void clear_red_fix_edge(int _layer);
    void clear_yellow_fix_edge(int _layer);
	string get_project_path() {
		return project_path;
	}
	QRect get_output_rect() {
		return output_rect;
	}
	void set_output_rect(QRect & r) {
		output_rect = r;
	}
	int get_current_layer() { return layer; }
	int set_current_layer(int _layer);
	string get_layer_name(int _layer);
	void set_layer_name(int _layer, string name);
	void set_nview(NavigateView * _nview);
	void set_ceview(CornerEdge * _ceview);
	void to_state_add_nail();
	void to_state_change_nail();
	int output_layer(int _layer, string pathname, int output_format=1, int qua=75);
	//if _layer==-1, means erase current layer
	int delete_layer(int _layer);
	int layer_up(int _layer);
	int layer_down(int _layer);
	void find_span_tree(int edge_idx, int compute_dir);
    double refilter_edge(int _layer, int w0, int w1, int option);
	void update_title();
	QPoint point2choose(QPoint mouse_point);
	bool modified();
	void write_file(string file_name);
	void auto_save_file();
	int read_file(string file_name, bool import=false);
};

#endif // STITCHVIEW_H
