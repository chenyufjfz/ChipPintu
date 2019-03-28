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
	void goto_next_corner();
	void goto_next_edge();

public slots:
	void corner0_click(int row, int column);
	void corner1_click(int row, int column);
	void edge0_click(int row, int column);
	void edge1_click(int row, int column);
	void corner0_double_click(int row, int column);
	void corner1_double_click(int row, int column);
	void edge0_double_click(int row, int column);
	void edge1_double_click(int row, int column);

signals:
	void goto_corner(unsigned corner_idx);
	void goto_edge(unsigned edge_idx);

protected:
	int find_edge(unsigned idx);
	int find_corner(unsigned idx);
private:
    Ui::CornerEdge *ui;
	LayerFeature * lf;
	vector<unsigned> corner_idx;
	vector<unsigned> edge_idx;
	unsigned corner1_idx[2];
	unsigned edge1_idx[4];
};

#endif // CORNEREDGE_H
