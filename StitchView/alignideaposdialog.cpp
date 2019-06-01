#include "alignideaposdialog.h"
#include "ui_alignideaposdialog.h"

AlignIdeaPosDialog::AlignIdeaPosDialog(QWidget *parent, int _w0, int _w1, int _option) :
    QDialog(parent),
    ui(new Ui::AlignIdeaPosDialog)
{
    w0 = _w0;
    w1 = _w1;
    option = _option;
    ui->setupUi(this);
	ui->lrSlider->setValue(w1);
	ui->udSlider->setValue(w0);
    ui->FirstColGood->setChecked(option & 1);
    ui->LastColGood->setChecked(option & 2);
    ui->FirstRowGood->setChecked(option & 4);
    ui->LastRowGood->setChecked(option & 8);
}

AlignIdeaPosDialog::~AlignIdeaPosDialog()
{
    delete ui;
}

void AlignIdeaPosDialog::on_buttonBox_accepted()
{
    w0 = ui->udSlider->value();
    w1 = ui->lrSlider->value();
    int fc = ui->FirstColGood->isChecked() ? 1 : 0;
    int lc = ui->LastColGood->isChecked() ? 2 : 0;
    int fr = ui->FirstRowGood->isChecked() ? 4 : 0;
    int lr = ui->LastRowGood->isChecked() ? 8 : 0;
    option = fc | lc | fr | lr;
}
