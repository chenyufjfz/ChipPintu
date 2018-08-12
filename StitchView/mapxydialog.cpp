#include "mapxydialog.h"
#include "ui_mapxydialog.h"

MapxyDialog::MapxyDialog(double _beta, double _zx, double _zy, int _merge, int _dst_w, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MapxyDialog)
{
    ui->setupUi(this);
	ui->merge->setText(QString::number(_merge));
	ui->rotate->setText(QString::number(_beta, 'f', 10));
	ui->zoomx->setText(QString::number(_zx, 'f', 10));
	ui->zoomy->setText(QString::number(_zy, 'f', 10));
	ui->dstw->setText(QString::number(_dst_w));
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
}