#ifndef VIAWIREEDITVIEW_H
#define VIAWIREEDITVIEW_H

#include <QWidget>
#include <vector>
#include <QMouseEvent>
#include <string>
#include "vwextract2.h"
#include "cellextract.h"


class ViaWireEditView : public QWidget
{
    Q_OBJECT
public:
    explicit ViaWireEditView(QWidget *parent = 0);
    double grid_high, grid_width, offset_y, offset_x;
	int mark_state, mark_type2, layer;
	unsigned mark_mask, show_debug_en;
	bool hide_obj;

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
	void show_mark(unsigned _mark_mask);
	void show_debug(bool _show_debug_en);
	void set_wire_para(int _layer, int type, int opt0, int opt1, int opt2, int opt3, 
		int opt4, int opt5, int opt6, float opt_f0);

    void set_scale(int _scale);
    int get_scale();

protected:
    vector <MarkObj> obj_set;
	string img_name;
	vector<unsigned int> mark_color;
	vector<QImage> bk_img;
	QImage bk_img_mask;
    bool mouse_press;
	QPoint mp_point;
    int select_idx, scale;
    MarkObj current_obj;
	VWExtract * vwe;	
	CellExtract * cele;
	ObjExtract * current_train;

protected:
    void paintEvent(QPaintEvent *e);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
	void keyPressEvent(QKeyEvent *e);
    void draw_obj(QPainter & painter, const MarkObj & obj);
};

#endif // VIAWIREEDITVIEW_H
