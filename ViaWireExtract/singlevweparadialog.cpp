#include "singlevweparadialog.h"
#include "ui_singlevweparadialog.h"

SingleVWEParaDialog::SingleVWEParaDialog(QWidget *parent, unsigned _wmin, unsigned _wmax, unsigned _gray_th, unsigned _sep,
	unsigned _opt, unsigned _gw, unsigned _gi, unsigned _ww, double _ww1, double _iw, int _shape_mask,
	int _dia0, int _dia1, int _dia2, int _dia3, int _dia4, int _dia5, int _dia6, int _dia7) :
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
    ui->gray_th->setText(QString::number(_gray_th));
    ui->sep->setText(QString::number(_sep));
	ui->shape_mask->setText(QString::number(_shape_mask));
    switch (_opt) {
    case 0:
        ui->straight_ext->setChecked(true);
        break;
    case 1:
        ui->net_ext->setChecked(true);
        break;
    }
	ui->dia0->setText(QString::number(_dia0));
	ui->dia1->setText(QString::number(_dia1));
	ui->dia2->setText(QString::number(_dia2));
	ui->dia3->setText(QString::number(_dia3));
	ui->dia4->setText(QString::number(_dia4));
	ui->dia5->setText(QString::number(_dia5));
	ui->dia6->setText(QString::number(_dia6));
	ui->dia7->setText(QString::number(_dia7));
}

SingleVWEParaDialog::~SingleVWEParaDialog()
{

    delete ui;
}

void SingleVWEParaDialog::on_buttonBox_accepted()
{
	wmin = ui->wmin->text().toInt();
	wmax = ui->wmax->text().toInt();
    gray_th = ui->gray_th->text().toInt();
    sep = ui->sep->text().toInt();
    gray_i = ui->gray_i->text().toInt();
    gray_w = ui->gray_w->text().toInt();
    w_wide = ui->w_wide->text().toInt();
	shape_mask = ui->shape_mask->text().toInt();
    w_wide1 = ui->w_wide1->text().toDouble();
    i_wide = ui->i_wide->text().toDouble();
	if (ui->straight_ext->isChecked())
        opt = 0;
	if (ui->net_ext->isChecked())
        opt = 1;
	dia0 = ui->dia0->text().toInt();
	dia1 = ui->dia1->text().toInt();
	dia2 = ui->dia2->text().toInt();
	dia3 = ui->dia3->text().toInt();
	dia4 = ui->dia4->text().toInt();
	dia5 = ui->dia5->text().toInt();
	dia6 = ui->dia6->text().toInt();
	dia7 = ui->dia7->text().toInt();
}
