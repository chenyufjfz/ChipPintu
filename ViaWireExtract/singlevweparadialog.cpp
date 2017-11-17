#include "singlevweparadialog.h"
#include "ui_singlevweparadialog.h"

SingleVWEParaDialog::SingleVWEParaDialog(QWidget *parent, unsigned _wmin, unsigned _wmax, unsigned _opt,
                     unsigned _gw, unsigned _gi, unsigned _ww, double _ww1, double _iw) :
    QDialog(parent),
    ui(new Ui::SingleVWEParaDialog)
{
    ui->setupUi(this);
    ui->wmin->setText(QString::number(_wmin));
    ui->wmax->setText(QString::number(_wmax));
    ui->gray_w->setText(QString::number(_gw));
    ui->gray_i->setText(QString::number(_gi));
    ui->w_wide->setText(QString::number(_ww));
    ui->w_wide1->setText(QString::number(_ww1));
    ui->i_wide->setText(QString::number(_iw));
    switch (_opt) {
    case 1:
        ui->straight_ext->setChecked(true);
        break;
    case 2:
        ui->single_ext->setChecked(true);
        break;
    case 3:
        ui->net_ext->setChecked(true);
        break;
    }
}

SingleVWEParaDialog::~SingleVWEParaDialog()
{

    delete ui;
}

void SingleVWEParaDialog::on_buttonBox_accepted()
{
	wmin = ui->wmin->text().toInt();
	wmax = ui->wmax->text().toInt();
    gray_i = ui->gray_i->text().toInt();
    gray_w = ui->gray_w->text().toInt();
    w_wide = ui->w_wide->text().toInt();
    w_wide1 = ui->w_wide1->text().toDouble();
    i_wide = ui->i_wide->text().toDouble();
	if (ui->straight_ext->isChecked())
		opt = 1;
	if (ui->single_ext->isChecked())
		opt = 2;
	if (ui->net_ext->isChecked())
		opt = 3;
}
