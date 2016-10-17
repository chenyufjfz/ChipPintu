#ifndef VIAWIREEDITVIEW_H
#define VIAWIREEDITVIEW_H

#include <QWidget>
#include <vector>
#include <QMouseEvent>
#include <string>
#include "vwextract.h"
#include "cellextract.h"

class ViaWireEditView : public QWidget
{
    Q_OBJECT
public:
    explicit ViaWireEditView(QWidget *parent = 0);
    double grid_high, grid_width, offset_y, offset_x;
    int mark_state, mark_type2;
    int wire_width, via_radius, insu_gap, grid_size;

signals:
	void mouse_change(QPoint pos, QString msg);

public:
    void load_bk_image(QString file_path);
    void set_grid_size(double high, double width);
    void set_offset(double oy, double ox);
    void set_mark(int ms, int type2);
    void save_objects(QString file_path);
    void load_objects(QString file_path);
	static bool load_objects(QString file_path, std::vector <MarkObj> & obj_set, int & wire_width, int & via_radius);
	void erase_all_objects();
    void start_train(int train_what, int _feature, int _iter_num, float _param1,float _param2, float _param3);
    void extract();
	void set_mark(unsigned mark_mask);
	void show_edge(bool show);
    void set_para(int _wire_width, int _via_radius, int _insu_gap, int _grid_size) {
        wire_width = _wire_width;
        via_radius = _via_radius;
        insu_gap = _insu_gap;
        grid_size = _grid_size;
    }

    void set_scale(int _scale);
    int get_scale();

protected:
    std::vector <MarkObj> obj_set;
	std::string img_name;
	std::vector<unsigned int> mark_color;
	QImage bk_img, bk_img_mask;
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
