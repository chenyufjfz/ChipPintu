#ifndef VIAWIREEDITVIEW_H
#define VIAWIREEDITVIEW_H

#include <QWidget>
#include <vector>
#include <QMouseEvent>
#include <string>
#include "vwextract2.h"
#include "cellextract.h"
#include "extractparam.h"

struct SingleWireExtParam {
	int wmin;
	int wmax;
	int opt;
	int shape_mask;
	int sep;
	int gray_th;
};

struct BrickShapeExtParam {
	int w_wide;
	double i_wide;
	double w_wide1;
	int gray_i;
	int gray_w;
};

class ViaWireEditView : public QWidget
{
    Q_OBJECT
public:
    explicit ViaWireEditView(QWidget *parent = 0);
	~ViaWireEditView();
    double grid_high, grid_width, offset_y, offset_x;
	int mark_state, mark_type2, layer;
	unsigned mark_mask, show_debug_en;
	bool hide_obj;
	SingleWireExtParam swe_param;
	BrickShapeExtParam bse_param;

signals:
	void mouse_change(QPoint pos, QString msg);

public:
    void load_bk_image(QString file_path);
    void set_grid_size(double high, double width);
    void set_offset(double oy, double ox);
    void set_mark(int ms, int type2);
    void save_objects(QString file_path);
    bool load_objects(QString file_path);
	void erase_all_objects();
    void start_cell_train(int , int , int , float _param1,float _param2, float _param3);
    void extract();
	void show_debug(unsigned _mark_mask, bool _show_debug_en);
	void set_wire_para(ExtractParam * ep, string action_name);
    void set_single_wire_ext_para(int wmin, int wmax, int opt, int gray_th, int sep, int shape_mask);
    void get_single_wire_ext_para(int &wmin, int &wmax, int &opt, int &gray_th, int &sep, int & shape_mask);
	void get_brick_shape_ext_para(int &ww, double &iw, double &ww1, int &gi, int &gw);
	void set_brick_shape_ext_para(int ww, double iw, double ww1, int gi, int gw);
	void get_via_diameter(int & _dia0, int & _dia1, int & _dia2, int & _dia3, int & _dia4, int & _dia5, int & _dia6, int & _dia7);
	void set_via_diameter(int _dia0, int _dia1, int _dia2, int _dia3, int _dia4, int _dia5, int _dia6, int _dia7);
    void set_scale(int _scale);
    int get_scale();

protected:
    vector <MarkObj> obj_set;
	int dia[8];
	string img_name;
	vector<QImage> bk_img;
	QImage bk_img_mask;
    bool mouse_press;
	QPoint mp_point;
    int select_idx, scale;
    MarkObj current_obj;
	VWExtract * vwe;	
	VWExtract * vwe_single;
	VWExtract * vwe_ml;
	CellExtract * cele;
	ObjExtract * current_train;

protected:
    void paintEvent(QPaintEvent *e);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
	void keyPressEvent(QKeyEvent *e);
    void draw_obj(QPainter & painter, const MarkObj & obj);
	void timerEvent(QTimerEvent *event);
};

#endif // VIAWIREEDITVIEW_H
