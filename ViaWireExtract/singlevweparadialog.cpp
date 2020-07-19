#include "singlevweparadialog.h"
#include "ui_singlevweparadialog.h"

SingleVWEParaDialog::SingleVWEParaDialog(QWidget *parent, unsigned _wmin, unsigned _wmax, unsigned _gray_th, unsigned _sep,
	unsigned _opt, unsigned _gw, unsigned _gi, unsigned _ww, double _ww1, double _iw, int _shape_mask,
	int _dia0, int _dia1, int _dia2, int _dia3, int _dia4, int _dia5, int _dia6, int _dia7, 
	int _wi0, int _wi1, int _wi2, int _wi3, int _wi4, int _wi5, int _wi6, int _wi7, int _via_at_center) :
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
	ui->insu0->setText(QString::number(_wi0 & 0xff));
	ui->insu1->setText(QString::number(_wi1 & 0xff));
	ui->insu2->setText(QString::number(_wi2 & 0xff));
	ui->insu3->setText(QString::number(_wi3 & 0xff));
	ui->insu4->setText(QString::number(_wi4 & 0xff));
	ui->insu5->setText(QString::number(_wi5 & 0xff));
	ui->insu6->setText(QString::number(_wi6 & 0xff));
	ui->insu7->setText(QString::number(_wi7 & 0xff));
	ui->wmin0->setText(QString::number(_wi0 >> 8 & 0xff));
	ui->wmin1->setText(QString::number(_wi1 >> 8 & 0xff));
	ui->wmin2->setText(QString::number(_wi2 >> 8 & 0xff));
	ui->wmin3->setText(QString::number(_wi3 >> 8 & 0xff));
	ui->wmin4->setText(QString::number(_wi4 >> 8 & 0xff));
	ui->wmin5->setText(QString::number(_wi5 >> 8 & 0xff));
	ui->wmin6->setText(QString::number(_wi6 >> 8 & 0xff));
	ui->wmin7->setText(QString::number(_wi7 >> 8 & 0xff));
	ui->wmax0->setText(QString::number(_wi0 >> 16 & 0xff));
	ui->wmax1->setText(QString::number(_wi1 >> 16 & 0xff));
	ui->wmax2->setText(QString::number(_wi2 >> 16 & 0xff));
	ui->wmax3->setText(QString::number(_wi3 >> 16 & 0xff));
	ui->wmax4->setText(QString::number(_wi4 >> 16 & 0xff));
	ui->wmax5->setText(QString::number(_wi5 >> 16 & 0xff));
	ui->wmax6->setText(QString::number(_wi6 >> 16 & 0xff));
	ui->wmax7->setText(QString::number(_wi7 >> 16 & 0xff));
	ui->viacenter0->setChecked(_via_at_center & 1);
	ui->viacenter1->setChecked(_via_at_center & 2);
	ui->viacenter2->setChecked(_via_at_center & 4);
	ui->viacenter3->setChecked(_via_at_center & 8);
	ui->viacenter4->setChecked(_via_at_center & 16);
	ui->viacenter5->setChecked(_via_at_center & 32);
	ui->viacenter6->setChecked(_via_at_center & 64);
	ui->viacenter7->setChecked(_via_at_center & 128);
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
	via_at_center = 0;
	via_at_center |= ui->viacenter0->isChecked() ? 1 : 0;
	via_at_center |= ui->viacenter1->isChecked() ? 2 : 0;
	via_at_center |= ui->viacenter2->isChecked() ? 4 : 0;
	via_at_center |= ui->viacenter3->isChecked() ? 8 : 0;
	via_at_center |= ui->viacenter4->isChecked() ? 16 : 0;
	via_at_center |= ui->viacenter5->isChecked() ? 32 : 0;
	via_at_center |= ui->viacenter6->isChecked() ? 64 : 0;
	via_at_center |= ui->viacenter7->isChecked() ? 128 : 0;
	wi0 = ui->insu0->text().toInt();
	wi1 = ui->insu1->text().toInt();
	wi2 = ui->insu2->text().toInt();
	wi3 = ui->insu3->text().toInt();
	wi4 = ui->insu4->text().toInt();
	wi5 = ui->insu5->text().toInt();
	wi6 = ui->insu6->text().toInt();
	wi7 = ui->insu7->text().toInt();
	wi0 |= ui->wmin0->text().toInt() << 8;
	wi1 |= ui->wmin1->text().toInt() << 8;
	wi2 |= ui->wmin2->text().toInt() << 8;
	wi3 |= ui->wmin3->text().toInt() << 8;
	wi4 |= ui->wmin4->text().toInt() << 8;
	wi5 |= ui->wmin5->text().toInt() << 8;
	wi6 |= ui->wmin6->text().toInt() << 8;
	wi7 |= ui->wmin7->text().toInt() << 8;
}
