#include "corneredge.h"
#include "ui_corneredge.h"
#include "stitchview.h"

static unsigned get_edge_idx(int idx, int dir) {
	if (idx == 0xffffffff)
		return 0xffffffff;
	switch (dir) {
	case DIR_LEFT:
		if (idx >= 0x10001)
			return idx - 0x10001;
		break;

	case DIR_UP:
		if (idx >= 0x10001)
			return (idx - 0x10001) | 0x80000000;
		break;

	case DIR_RIGHT:
		if (idx >= 0x10000)
			return idx - 0x10000;
		break;

	case DIR_DOWN:
		if (idx >= 1)
			return (idx - 1) | 0x80000000;
		break;
	default:
		CV_Assert(0);
	}
	return 0xffffffff;
}

static unsigned get_corner_idx(int idx, int dir) {
	if (idx == 0xffffffff)
		return 0xffffffff;
	unsigned first, second;
	if (EDGE_E(idx)) {
		first = (idx + 1) & 0x7fffffff;
		second = (idx + 0x10001) & 0x7fffffff;
	}
	else {
		first = idx + 0x10000;
		second = idx + 0x10001;
	}
	return dir == 0 ? first : second;
}

CornerEdge::CornerEdge(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CornerEdge)
{
    ui->setupUi(this);

	ui->corner_tbl0->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui->corner_tbl1->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui->edge_tb0->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui->edge_tbl1->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui->corner_tbl1->setRowCount(2);
	ui->edge_tbl1->setRowCount(4);

	connect(ui->corner_tbl0, SIGNAL(cellClicked(int, int)), this, SLOT(corner0_click(int, int)));
	connect(ui->corner_tbl1, SIGNAL(cellClicked(int, int)), this, SLOT(corner1_click(int, int)));
	connect(ui->edge_tb0, SIGNAL(cellClicked(int, int)), this, SLOT(edge0_click(int, int)));
	connect(ui->edge_tbl1, SIGNAL(cellClicked(int, int)), this, SLOT(edge1_click(int, int)));
	connect(ui->corner_tbl0, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(corner0_double_click(int, int)));
	connect(ui->corner_tbl1, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(corner1_double_click(int, int)));
	connect(ui->edge_tb0, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(edge0_double_click(int, int)));
	connect(ui->edge_tbl1, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(edge1_double_click(int, int)));

	ui->corner_tbl0->resizeColumnsToContents();
	ui->corner_tbl1->resizeColumnsToContents();
	ui->edge_tb0->resizeColumnsToContents();
	ui->edge_tbl1->resizeColumnsToContents();

	for (int i = 0; i < 2; i++)
		corner1_idx[i] = 0xffffffff;
	for (int i = 0; i < 4; i++)
		edge1_idx[i] = 0xffffffff;
}

CornerEdge::~CornerEdge()
{
    delete ui;
}

void CornerEdge::corner0_click(int row, int col)
{
	qInfo("corner0 click %d, %d", row, col);
	unsigned idx = corner_idx[row];
	for (int dir = 0; dir < 4; dir++) {
		unsigned e_idx = get_edge_idx(idx, dir);
		int row = find_edge(e_idx);
		if (row == 0xffffffff)
			continue;
		ui->edge_tbl1->setItem(dir, 0, ui->edge_tb0->item(row, 0)->clone());
		ui->edge_tbl1->setItem(dir, 1, ui->edge_tb0->item(row, 1)->clone());
		edge1_idx[dir] = e_idx;
	}
	update();
}

void CornerEdge::corner1_click(int row, int col)
{
	qInfo("corner1 click %d, %d", row, col);
	unsigned idx = corner1_idx[row];
	for (int dir = 0; dir < 4; dir++) {
		unsigned e_idx = get_edge_idx(idx, dir);
		int row = find_edge(e_idx);
		if (row == 0xffffffff)
			continue;
		ui->edge_tbl1->setItem(dir, 0, ui->edge_tb0->item(row, 0)->clone());
		ui->edge_tbl1->setItem(dir, 1, ui->edge_tb0->item(row, 1)->clone());
		edge1_idx[dir] = e_idx;
	}
	update();
}

void CornerEdge::edge0_click(int row, int col)
{
	qInfo("edge0 click %d, %d", row, col);
	unsigned idx = edge_idx[row];
	for (int dir = 0; dir < 2; dir++) {
		unsigned c_idx = get_corner_idx(idx, dir);
		int row = find_corner(c_idx);
		if (row == 0xffffffff)
			continue;
		ui->corner_tbl1->setItem(dir, 0, ui->corner_tbl0->item(row, 0)->clone());
		ui->corner_tbl1->setItem(dir, 1, ui->corner_tbl0->item(row, 1)->clone());
		corner1_idx[dir] = c_idx;
	}
	update();
}

void CornerEdge::edge1_click(int row, int col)
{
	qInfo("edge1 click %d, %d", row, col);
	unsigned idx = edge1_idx[row];
	for (int dir = 0; dir < 2; dir++) {
		unsigned c_idx = get_corner_idx(idx, dir);
		int row = find_corner(c_idx);
		if (row == 0xffffffff)
			continue;
		ui->corner_tbl1->setItem(dir, 0, ui->corner_tbl0->item(row, 0)->clone());
		ui->corner_tbl1->setItem(dir, 1, ui->corner_tbl0->item(row, 1)->clone());
		corner1_idx[dir] = c_idx;
	}
	update();
}

void CornerEdge::corner0_double_click(int row, int col)
{
	qInfo("corner0 double click %d, %d", row, col);
	emit goto_corner(corner_idx[row]);
}

void CornerEdge::corner1_double_click(int row, int col)
{
	qInfo("corner1 double click %d, %d", row, col);
	emit goto_corner(corner1_idx[row]);
}

void CornerEdge::edge0_double_click(int row, int col)
{
	qInfo("edge0 double click %d, %d", row, col);
	emit goto_edge(edge_idx[row]);
}

void CornerEdge::edge1_double_click(int row, int col)
{
	qInfo("edge1 double click %d, %d", row, col);
	emit goto_edge(edge1_idx[row]);
}

int CornerEdge::find_edge(unsigned idx)
{
	for (int i = 0; i < (int)edge_idx.size(); i++)
	if (edge_idx[i] == idx)
		return i;
	return 0xffffffff;
}

int CornerEdge::find_corner(unsigned idx)
{
	for (int i = 0; i < (int)edge_idx.size(); i++)
	if (corner_idx[i] == idx)
		return i;
	return 0xffffffff;
}

void CornerEdge::set_layer_info(LayerFeature * _lf)
{
	lf =  _lf;
	vector<unsigned long long> ce_set;
	//same as compute_unsure_corner
	for (int y = 0; y < lf->corner_info.rows; y++) {
		for (int x = 0; x < lf->corner_info.cols; x++) {
			unsigned long long info = lf->corner_info(y, x)[1];
			info = info << 32 | (unsigned) lf->corner_info(y, x)[0];
			unsigned long long c = info >> 32 & 0xffff;
			short val0, val1;
			val0 = info & 0xffff;
			val0 = val0 / lf->cpara.rescale;
			val0 = max(min(val0, (short) 127), (short)-127);
			val1 = info >> 16 & 0xffff;
			val1 = val1 / lf->cpara.rescale;
			val1 = max(min(val1, (short) 127), (short)-127);
			c = c << 16 | ((unsigned)val1 & 0xff) << 8 | (unsigned)val0 & 0xff;
			ce_set.push_back(c << 32 | MAKE_CORNER_IDX(x, y));
		}
	}
	sort(ce_set.begin(), ce_set.end(), greater<unsigned long long>());
	corner_idx.resize(ce_set.size());
	ui->corner_tbl0->setRowCount((int) ce_set.size());
	for (int i = 0; i < (int)corner_idx.size(); i++) {
		corner_idx[i] = ce_set[i] & 0xffffffff;
		char str[100];
		sprintf(str, "%d,%d", CORNER_X(corner_idx[i]), CORNER_Y(corner_idx[i]));
		ui->corner_tbl0->setItem(i, 0, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		int val0 = (ce_set[i] >> 32) & 0xff;
		int val1 = (ce_set[i] >> 40) & 0xff;
		val0 = val0 > 0x7f ? val0 - 0x100 : val0;
		val1 = val1 > 0x7f ? val1 - 0x100 : val1;
		val0 = val0 * lf->cpara.rescale;
		val1 = val1 * lf->cpara.rescale;
		sprintf(str, "%d,%d", val0, val1);
		ui->corner_tbl0->setItem(i, 1, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		int cost = (ce_set[i] >> 48) & 0xffff;
		sprintf(str, "%d", cost);
		ui->corner_tbl0->setItem(i, 2, new QTableWidgetItem(QString::fromLocal8Bit(str)));
	}

	ce_set.clear();
	for (int y = 0; y < lf->cpara.img_num_h; y++)
	for (int x = 0; x < lf->cpara.img_num_w; x++) 
	for (int i = 0; i < 2; i++) {
		Point src_corner(lf->cpara.offset(y, x)[1], lf->cpara.offset(y, x)[0]);
		int fe = 0;
		if (y > 0 && i == 0)
			fe = lf->flagb[i](y - 1, x);
		if (x > 0 && i == 1)
			fe = lf->flagb[i](y, x - 1);
		const EdgeDiff * ed = (i == 0) ? lf->feature.get_edge(0, y - 1, x) :
			lf->feature.get_edge(1, y, x - 1);
		if (ed && ed->img_num > 0) {
			Point src_corner3 = (i == 0) ? Point(lf->cpara.offset(y - 1, x)[1], lf->cpara.offset(y - 1, x)[0]) :
				Point(lf->cpara.offset(y, x - 1)[1], lf->cpara.offset(y, x - 1)[0]);
			Point oo = src_corner - src_corner3;
			Point idea_pos = (FIX_EDGE_IDEA_POS(fe)) ?
				ed->offset + Point(FIX_EDGE_IDEA_DX(fe), FIX_EDGE_IDEA_DY(fe)) * lf->cpara.rescale :
				ed->offset + ed->minloc * lf->cpara.rescale;
			Point shift = oo - idea_pos;
			int val_x = fe ? 0 : shift.x;
			int val_y = fe ? 0 : shift.y;
			unsigned long long c = (abs(val_x) + abs(val_y)) / lf->cpara.rescale;
			if (c > 255)
				c = 255;
			c = c << 24 | (int)(val_y & 0xfff) << 12 | val_x & 0xfff;
			
			if (i == 0)
				ce_set.push_back(c << 32 | MAKE_EDGE_IDX(x, y - 1, i));
			else
				ce_set.push_back(c << 32 | MAKE_EDGE_IDX(x - 1, y, i));
		}
	}

	sort(ce_set.begin(), ce_set.end(), greater<unsigned long long>());
	edge_idx.resize(ce_set.size());
	ui->edge_tb0->setRowCount((int) ce_set.size());
	for (int i = 0; i < (int)edge_idx.size(); i++) {
		edge_idx[i] = ce_set[i] & 0xffffffff;
		char str[100];
		sprintf(str, "%d,%d,%d", EDGE_X(edge_idx[i]), EDGE_Y(edge_idx[i]), EDGE_E(edge_idx[i]));
		ui->edge_tb0->setItem(i, 0, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		int val_x = (ce_set[i] >> 32) & 0xfff;
		int val_y = (ce_set[i] >> 44) & 0xfff;
		val_x = val_x > 0x7ff ? val_x - 0x1000 : val_x;
		val_y = val_y > 0x7ff ? val_y - 0x1000 : val_y;
		sprintf(str, "%d,%d", val_x, val_y);
		ui->edge_tb0->setItem(i, 1, new QTableWidgetItem(QString::fromLocal8Bit(str)));
	}

	update();
}