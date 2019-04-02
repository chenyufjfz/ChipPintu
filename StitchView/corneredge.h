#ifndef CORNEREDGE_H
#define CORNEREDGE_H

#include <QDialog>
class CornerEdge;
#include "stitchview.h"

namespace Ui {
class CornerEdge;
}

class CornerEdge : public QDialog
{
    Q_OBJECT

public:
    explicit CornerEdge(QWidget *parent = 0);
    ~CornerEdge();
	void set_layer_info(LayerFeature * _lf);
	void set_nail_info(vector<Point2f> ns, vector<Point> bias, vector<int> dir);
	void goto_next_corner();
	void goto_next_edge();

public slots:
	void corner0_click(int row, int column);
	void edge0_click(int row, int column);
	void corner0_double_click(int row, int column);
	void edge0_double_click(int row, int column);
	void nail0_click(int row, int column);

signals:
	void goto_corner(unsigned corner_idx);
	void goto_edge(unsigned edge_idx);
	void goto_nail(unsigned idx);

protected:
	int find_edge(unsigned idx);
	int find_corner(unsigned idx);
	bool is_edge_changed(unsigned idx);
	bool is_corner_changed(unsigned idx);
	void review_edge(unsigned idx);
	void review_corner(unsigned idx);

private:
    Ui::CornerEdge *ui;
	LayerFeature * lf;
	vector<unsigned> corner_idx;
	vector<unsigned> edge_idx;
    unsigned reviewed_corner_idx, reviewed_edge_idx;
};

#endif // CORNEREDGE_H
