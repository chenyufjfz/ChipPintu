#include "tparadialog.h"
#include "ui_tparadialog.h"

TParaDialog::TParaDialog(TuningPara _tpara, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TParaDialog)
{
    tpara = _tpara;
    ui->setupUi(this);
    ui->bfilt_w->setText(QString::number(tpara.bfilt_w));
    ui->bfilt_csigma->setText(QString::number(tpara.bfilt_csigma));
    ui->canny_high_th->setText(QString::number(tpara.canny_high_th));
    ui->canny_low_th->setText(QString::number(tpara.canny_low_th));
    ui->sobel_w->setText(QString::number(tpara.sobel_w));
}

TParaDialog::~TParaDialog()
{
    delete ui;
}

void TParaDialog::on_buttonBox_accepted()
{
    tpara.bfilt_w = ui->bfilt_w->text().toInt();
    tpara.bfilt_csigma = ui->bfilt_csigma->text().toInt();
    tpara.canny_high_th = ui->canny_high_th->text().toInt();
    tpara.canny_low_th = ui->canny_low_th->text().toInt();
    tpara.sobel_w = ui->sobel_w->text().toInt();
}
