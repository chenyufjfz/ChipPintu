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
	ui->edge_tb0->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui->nail_tbl0->setEditTriggers(QAbstractItemView::NoEditTriggers);

	connect(ui->corner_tbl0, SIGNAL(cellClicked(int, int)), this, SLOT(corner0_click(int, int)));
	connect(ui->edge_tb0, SIGNAL(cellClicked(int, int)), this, SLOT(edge0_click(int, int)));
    //connect(ui->corner_tbl0, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(corner0_double_click(int, int)));
    //connect(ui->edge_tb0, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(edge0_double_click(int, int)));
	connect(ui->nail_tbl0, SIGNAL(cellClicked(int, int)), this, SLOT(nail0_click(int, int)));

	ui->corner_tbl0->resizeColumnsToContents();
	ui->edge_tb0->resizeColumnsToContents();
	ui->nail_tbl0->resizeColumnsToContents();
	ui->corner_tbl0->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui->edge_tb0->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui->nail_tbl0->setSelectionBehavior(QAbstractItemView::SelectRows);
	reviewed_corner_idx = 0xffffffff;
	reviewed_edge_idx = 0xffffffff;
}

CornerEdge::~CornerEdge()
{
    delete ui;
}

void CornerEdge::corner0_click(int row, int )
{
    bool shift_on = QApplication::queryKeyboardModifiers() == Qt::ShiftModifier;
    qInfo("corner0 click %d, shift=%d", row, shift_on);
	emit goto_corner(corner_idx[row]);
    if (shift_on && reviewed_corner_idx == corner_idx[row]) {
        review_corner(reviewed_corner_idx);
        ui->corner_tbl0->item(row, 0)->setForeground(QBrush(QColor(0, 0, 0)));
        ui->corner_tbl0->item(row, 1)->setForeground(QBrush(QColor(0, 0, 0)));
        ui->corner_tbl0->item(row, 2)->setForeground(QBrush(QColor(0, 0, 0)));
    }
	reviewed_corner_idx = corner_idx[row];
	update();
}

void CornerEdge::edge0_click(int row, int )
{
    bool shift_on = QApplication::queryKeyboardModifiers() == Qt::ShiftModifier;
    qInfo("edge0 click %d, shift=%d", row, shift_on);
	emit goto_edge(edge_idx[row]);
    if (shift_on && reviewed_edge_idx == edge_idx[row]) {
        review_edge(reviewed_edge_idx);
        ui->edge_tb0->item(row, 0)->setForeground(QBrush(QColor(0, 0, 0)));
		ui->edge_tb0->item(row, 1)->setForeground(QBrush(QColor(0, 0, 0)));
    }
	reviewed_edge_idx = edge_idx[row];
	update();
}

void CornerEdge::corner0_double_click(int row, int col)
{
	qInfo("corner0 double click %d, %d", row, col);
	if (reviewed_corner_idx != 0xffffffff) {
		review_corner(reviewed_corner_idx);
		int review_row = find_corner(reviewed_corner_idx);
		ui->corner_tbl0->item(review_row, 0)->setForeground(QBrush(QColor(0, 0, 0)));
		ui->corner_tbl0->item(review_row, 1)->setForeground(QBrush(QColor(0, 0, 0)));
		ui->corner_tbl0->item(review_row, 2)->setForeground(QBrush(QColor(0, 0, 0)));
	}
	emit goto_corner(corner_idx[row]);
	reviewed_corner_idx = corner_idx[row];
	update();
}

void CornerEdge::edge0_double_click(int row, int col)
{
	qInfo("edge0 double click %d, %d", row, col);
	if (reviewed_edge_idx != 0xffffffff) {
		review_edge(reviewed_edge_idx);
		int review_row = find_edge(reviewed_edge_idx);
		ui->edge_tb0->item(review_row, 0)->setForeground(QBrush(QColor(0, 0, 0)));
		ui->edge_tb0->item(review_row, 1)->setForeground(QBrush(QColor(0, 0, 0)));
		ui->edge_tb0->item(review_row, 2)->setForeground(QBrush(QColor(0, 0, 0)));
	}
	emit goto_edge(edge_idx[row]);
	reviewed_edge_idx = edge_idx[row];
	update();
}

void CornerEdge::nail0_click(int row, int col)
{
	qInfo("nail0 click %d, %d", row, col);
	emit goto_nail(row);
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
	for (int i = 0; i < (int)corner_idx.size(); i++)
	if (corner_idx[i] == idx)
		return i;
	return 0xffffffff;
}

bool CornerEdge::is_edge_changed(unsigned idx)
{
	int e = EDGE_E(idx);
	int x = EDGE_X(idx);
	int y = EDGE_Y(idx);
	int flagb = lf->flagb[e](y, x);
	if (FIX_EDGE_ISBIND(flagb))
		return false;
	Vec2i oo = e ? lf->cpara.offset(y, x + 1) - lf->cpara.offset(y, x):
		lf->cpara.offset(y + 1, x) - lf->cpara.offset(y, x);
	Vec2i review_oo = lf->checked_edge_offset[e](y, x);
	int err = abs(review_oo[0] - oo[0]) + abs(review_oo[1] - oo[1]);
	return err > lf->cpara.rescale;
}

bool CornerEdge::is_corner_changed(unsigned idx)
{	
	return is_edge_changed(get_edge_idx(idx, DIR_UP)) || 
		is_edge_changed(get_edge_idx(idx, DIR_LEFT)) ||
		is_edge_changed(get_edge_idx(idx, DIR_DOWN)) || 
		is_edge_changed(get_edge_idx(idx, DIR_RIGHT));
}

void CornerEdge::review_edge(unsigned idx)
{
	int e = EDGE_E(idx);
	int x = EDGE_X(idx);
	int y = EDGE_Y(idx);
	Vec2i oo = e ? lf->cpara.offset(y, x + 1) - lf->cpara.offset(y, x) :
		lf->cpara.offset(y + 1, x) - lf->cpara.offset(y, x);
	lf->checked_edge_offset[e](y, x) = oo;
}

void CornerEdge::review_corner(unsigned idx)
{
	review_edge(get_edge_idx(idx, DIR_UP));
	review_edge(get_edge_idx(idx, DIR_LEFT));
	review_edge(get_edge_idx(idx, DIR_DOWN));
	review_edge(get_edge_idx(idx, DIR_RIGHT));
}

void CornerEdge::set_layer_info(LayerFeature * _lf)
{
	lf = _lf;
	if (lf->checked_edge_offset[0].empty() || lf->checked_edge_offset[0].rows != lf->cpara.img_num_h - 1
		|| lf->checked_edge_offset[0].cols != lf->cpara.img_num_w) {
		lf->checked_edge_offset[0].create(lf->cpara.img_num_h - 1, lf->cpara.img_num_w);
		lf->checked_edge_offset[0] = 0;
		lf->checked_edge_offset[1].create(lf->cpara.img_num_h, lf->cpara.img_num_w - 1);
		lf->checked_edge_offset[1] = 0;
	}
	vector<unsigned long long> ce_set;
	//same as compute_unsure_corner
	for (int y = 1; y < lf->corner_info.rows; y++) {
		for (int x = 1; x < lf->corner_info.cols; x++) {
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
		bool use_red = is_corner_changed(corner_idx[i]);
		QBrush mybrush(QColor(use_red ? 255 : 0, 0, 0));
		char str[100];
		sprintf(str, "%d,%d", CORNER_X(corner_idx[i]), CORNER_Y(corner_idx[i]));
		ui->corner_tbl0->setItem(i, 0, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		ui->corner_tbl0->item(i, 0)->setForeground(mybrush);
		int val0 = (ce_set[i] >> 32) & 0xff;
		int val1 = (ce_set[i] >> 40) & 0xff;
		val0 = val0 > 0x7f ? val0 - 0x100 : val0;
		val1 = val1 > 0x7f ? val1 - 0x100 : val1;
		val0 = val0 * lf->cpara.rescale;
		val1 = val1 * lf->cpara.rescale;
		sprintf(str, "%d,%d", val0, val1);
		ui->corner_tbl0->setItem(i, 1, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		ui->corner_tbl0->item(i, 1)->setForeground(mybrush);
		int cost = (ce_set[i] >> 48) & 0xffff;
		sprintf(str, "%d", cost);
		ui->corner_tbl0->setItem(i, 2, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		ui->corner_tbl0->item(i, 2)->setForeground(mybrush);
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
			int val_x = FIX_EDGE_BINDX(fe) ? 0 : shift.x;
			int val_y = FIX_EDGE_BINDY(fe) ? 0 : shift.y;
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
		bool use_red = is_edge_changed(edge_idx[i]);
		QBrush mybrush(QColor(use_red ? 255 : 0, 0, 0));
		char str[100];
		sprintf(str, "%d,%d,%d", EDGE_X(edge_idx[i]), EDGE_Y(edge_idx[i]), EDGE_E(edge_idx[i]));
		ui->edge_tb0->setItem(i, 0, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		ui->edge_tb0->item(i, 0)->setForeground(mybrush);
		int val_x = (ce_set[i] >> 32) & 0xfff;
		int val_y = (ce_set[i] >> 44) & 0xfff;
		val_x = val_x > 0x7ff ? val_x - 0x1000 : val_x;
		val_y = val_y > 0x7ff ? val_y - 0x1000 : val_y;
		sprintf(str, "%d,%d", val_x, val_y);
		ui->edge_tb0->setItem(i, 1, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		ui->edge_tb0->item(i, 1)->setForeground(mybrush);
	}
	ui->corner_tbl0->resizeColumnsToContents();
	ui->edge_tb0->resizeColumnsToContents();
	update();
	reviewed_corner_idx = 0xffffffff;
	reviewed_edge_idx = 0xffffffff;
}

void CornerEdge::set_nail_info(vector<Point2f> ns, vector<Point> bias, vector<int> dir)
{
	CV_Assert(ns.size() == bias.size() && ns.size() == dir.size());
	ui->nail_tbl0->setRowCount((int) ns.size());
	for (int i = 0; i < (int)ns.size(); i++) {
		char str[100];
		sprintf(str, "%5.3f,%5.3f", ns[i].x, ns[i].y);
		ui->nail_tbl0->setItem(i, 0, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		sprintf(str, "%d,%d", bias[i].x, bias[i].y);
		ui->nail_tbl0->setItem(i, 1, new QTableWidgetItem(QString::fromLocal8Bit(str)));
		sprintf(str, "%c", dir[i]==2 ? '=' : (dir[i] ? '!' : '^'));
		ui->nail_tbl0->setItem(i, 2, new QTableWidgetItem(QString::fromLocal8Bit(str)));
	}
	ui->nail_tbl0->resizeColumnsToContents();
	update();
}

void CornerEdge::goto_next_corner()
{
	int row = ui->corner_tbl0->currentRow();
	row++;
	if (row == ui->corner_tbl0->rowCount())
		row = 0;
	ui->corner_tbl0->setCurrentCell(row, 0);
    corner0_click(row, 0);
}

void CornerEdge::goto_next_edge()
{
	int row = ui->edge_tb0->currentRow();
	row++;
	if (row == ui->edge_tb0->rowCount())
		row = 0;
	ui->edge_tb0->setCurrentCell(row, 0);
    edge0_click(row, 0);
}
