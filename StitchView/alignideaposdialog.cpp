#include "alignideaposdialog.h"
#include "ui_alignideaposdialog.h"

AlignIdeaPosDialog::AlignIdeaPosDialog(QWidget *parent, int _w0, int _w1) :
    QDialog(parent),
    ui(new Ui::AlignIdeaPosDialog)
{
    w0 = _w0;
    w1 = _w1;
    ui->setupUi(this);
	ui->lrSlider->setValue(w1);
	ui->udSlider->setValue(w0);
}

AlignIdeaPosDialog::~AlignIdeaPosDialog()
{
    delete ui;
}

void AlignIdeaPosDialog::on_buttonBox_accepted()
{
    w0 = ui->udSlider->value();
    w1 = ui->lrSlider->value();
}
