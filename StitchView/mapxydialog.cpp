#include "mapxydialog.h"
#include "ui_mapxydialog.h"

MapxyDialog::MapxyDialog(double _beta, double _zx, double _zy, int _merge, int _dst_w, int max_pt_err, int _merge_distance, QRect _output_rect, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MapxyDialog)
{
    ui->setupUi(this);
	ui->merge->setText(QString::number(_merge));
	ui->rotate->setText(QString::number(_beta, 'f', 10));
	ui->zoomx->setText(QString::number(_zx, 'f', 10));
	ui->zoomy->setText(QString::number(_zy, 'f', 10));
	ui->max_pt_err->setText(QString::number(max_pt_err));
	ui->max_slope_err->setText(QString::number(_merge_distance));
	ui->dstw->setText(QString::number(_dst_w));
	ui->output_top->setText(QString::number(_output_rect.top()));
	ui->output_left->setText(QString::number(_output_rect.left()));
	ui->output_bottom->setText(QString::number(_output_rect.bottom()));
	ui->output_right->setText(QString::number(_output_rect.right()));
}

MapxyDialog::~MapxyDialog()
{
    delete ui;
}

void MapxyDialog::on_buttonBox_accepted()
{
	merge = ui->merge->text().toInt();
	beta = ui->rotate->text().toDouble();
	zx = ui->zoomx->text().toDouble();
	zy = ui->zoomy->text().toDouble();
	dst_w = ui->dstw->text().toInt();
	max_pt_err = ui->max_pt_err->text().toInt();
	merge_distance = ui->max_slope_err->text().toInt();
	int output_top = ui->output_top->text().toInt();
	int output_left = ui->output_left->text().toInt();
	int output_bottom = ui->output_bottom->text().toInt();
	int output_right = ui->output_right->text().toInt();
	output_rect.setTopLeft(QPoint(output_left, output_top));
	output_rect.setBottomRight(QPoint(output_right, output_bottom));
}
