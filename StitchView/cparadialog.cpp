#include "cparadialog.h"
#include "ui_cparadialog.h"

CparaDialog::CparaDialog(ConfigPara _cpara, bool _new_layer, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CparaDialog)
{
    cpara = _cpara;
	new_layer = _new_layer;
    ui->setupUi(this);

	ui->rescale->setText(QString::number(cpara.rescale));
	ui->max_lr_xshift->setText(QString::number(cpara.max_lr_xshift));
	ui->max_lr_yshift->setText(QString::number(cpara.max_lr_yshift));
	ui->max_ud_xshift->setText(QString::number(cpara.max_ud_xshift));
	ui->max_ud_yshift->setText(QString::number(cpara.max_ud_yshift));

	if (new_layer) {
		ui->clip_l->setText(QString::number(cpara.clip_l));
		ui->clip_r->setText(QString::number(cpara.clip_r));
		ui->clip_d->setText(QString::number(cpara.clip_d));
		ui->clip_u->setText(QString::number(cpara.clip_u));
		ui->img_num_w->setText(QString::number(cpara.img_num_w));
		ui->img_num_h->setText(QString::number(cpara.img_num_h));
		ui->img_path->setText(QString::fromStdString(cpara.img_path));
		ui->new_layer->setChecked(new_layer);
		ui->init_offset_x->setText(QString::number(cpara.offset(0, 1)[1] - cpara.offset(0, 0)[1]));
		ui->init_offset_y->setText(QString::number(cpara.offset(1, 0)[0] - cpara.offset(0, 0)[0]));
		ui->init_offset_udx->setText(QString::number(cpara.offset(1, 0)[1] - cpara.offset(0, 0)[1]));
		ui->init_offset_lry->setText(QString::number(cpara.offset(0, 1)[0] - cpara.offset(0, 0)[0]));
	}
	else {
		ui->clip_l->setEnabled(false);
		ui->clip_r->setEnabled(false);
		ui->clip_d->setEnabled(false);
		ui->clip_u->setEnabled(false);
		ui->img_num_w->setEnabled(false);
		ui->img_num_h->setEnabled(false);
		ui->img_path->setEnabled(false);
		ui->new_layer->setEnabled(false);
		ui->init_offset_x->setEnabled(false);
		ui->init_offset_y->setEnabled(false);
		ui->init_offset_udx->setEnabled(false);
		ui->init_offset_lry->setEnabled(false);
	}

}

CparaDialog::~CparaDialog()
{
    delete ui;
}

void CparaDialog::on_buttonBox_accepted()
{
	cpara.rescale = ui->rescale->text().toInt();
	cpara.max_lr_xshift = ui->max_lr_xshift->text().toInt();
	cpara.max_lr_yshift = ui->max_lr_yshift->text().toInt();
	cpara.max_ud_xshift = ui->max_ud_xshift->text().toInt();
	cpara.max_ud_yshift = ui->max_ud_yshift->text().toInt();

	if (new_layer) {
		cpara.clip_l = ui->clip_l->text().toInt();
		cpara.clip_r = ui->clip_r->text().toInt();
		cpara.clip_u = ui->clip_u->text().toInt();
		cpara.clip_d = ui->clip_d->text().toInt();
		cpara.img_num_w = ui->img_num_w->text().toInt();
		cpara.img_num_h = ui->img_num_h->text().toInt();
		cpara.img_path = ui->img_path->text().toStdString();
		cpara.offset(0, 1)[1] = ui->init_offset_x->text().toInt() + cpara.offset(0, 0)[1];
		cpara.offset(0, 1)[0] = ui->init_offset_lry->text().toInt() + cpara.offset(0, 0)[0];
		cpara.offset(1, 0)[0] = ui->init_offset_y->text().toInt() + cpara.offset(0, 0)[0];
		cpara.offset(1, 0)[1] = ui->init_offset_udx->text().toInt() + cpara.offset(0, 0)[1];
		new_layer = ui->new_layer->isChecked();
	}
}
