#include "cparadialog.h"
#include "ui_cparadialog.h"
#include <iostream>
#include <fstream>
#include <QMessageBox>
#include <QFileDialog>

CparaDialog::CparaDialog(ConfigPara _cpara, bool _new_layer, bool _update_scale_en, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CparaDialog)
{
    cpara = _cpara;
	new_layer = _new_layer;
	update_scale_en = _update_scale_en;
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
		ui->ConfigButton->setEnabled(false);
		ui->config_file->setEnabled(false);
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
		if (config_file.empty()) {
			int init_offset_x = ui->init_offset_x->text().toInt();
			int init_offset_lry = ui->init_offset_lry->text().toInt();
			int init_offset_y = ui->init_offset_y->text().toInt();
			int init_offset_udx = ui->init_offset_udx->text().toInt();
			init_offset_x = (init_offset_x + cpara.rescale / 2) / cpara.rescale * cpara.rescale;
			init_offset_lry = (init_offset_lry + cpara.rescale / 2) / cpara.rescale * cpara.rescale;
			init_offset_y = (init_offset_y + cpara.rescale / 2) / cpara.rescale * cpara.rescale;
			init_offset_udx = (init_offset_udx + cpara.rescale / 2) / cpara.rescale * cpara.rescale;
			qInfo("UI: config init, path=%s, ox=%d,oy=%d, nw=%d, nh=%d", cpara.img_path.c_str(),
				init_offset_x, init_offset_y, cpara.img_num_w, cpara.img_num_h);
			cpara.offset.create(cpara.img_num_h, cpara.img_num_w);
			for (int y = 0; y < cpara.img_num_h; y++) {
				for (int x = 0; x < cpara.img_num_w; x++) {
					cpara.offset(y, x)[1] = init_offset_x * x + init_offset_udx * y;
					cpara.offset(y, x)[0] = init_offset_y * y + init_offset_lry * x;
				}
			}
		}
		new_layer = ui->new_layer->isChecked();
	}
}

void CparaDialog::on_config_file_textChanged(const QString &arg1)
{
	config_file = arg1.toStdString();
	if (config_file.length() < 2) {
		ui->init_offset_x->setEnabled(true);
		ui->init_offset_y->setEnabled(true);
		ui->init_offset_udx->setEnabled(true);
		ui->init_offset_lry->setEnabled(true);
		ui->max_lr_xshift->setEnabled(true);
		ui->max_lr_yshift->setEnabled(true);
		ui->max_ud_xshift->setEnabled(true);
		ui->max_ud_yshift->setEnabled(true);
		ui->img_num_w->setEnabled(true);
		ui->img_num_h->setEnabled(true);
		ui->rescale->setEnabled(true);
		return;
	}
	string item;
	ifstream fr(config_file);
	vector<Point> cx, cy;
	if (!fr) {
		config_file.clear();
		return;
	}
		
	while (getline(fr, item)) {
		int state = 0;
		string item_head = item.substr(0, 50);
		if (item_head.find("LR err distribute") != string::npos)
			state = 1;
		if (item_head.find("UD err distribute") != string::npos)
			state = 2;
		if (item_head.find("img_num") != string::npos) {
			if (sscanf(item_head.c_str(), "img_num, w=%d h=%d s=%d", &cpara.img_num_w, &cpara.img_num_h, &cpara.rescale) != 3) {
				config_file.clear();
				break;
			}
		}
		if (state == 1) {
			int v1, v2, v3, v4, v5, v6;
			if (sscanf(item.c_str(), "LR err distribute,%d %d,%d %d,%d %d,", &v1, &v2, &v3, &v4, &v5, &v6) != 6) {
				config_file.clear();
				break;
			}
			cpara.max_lr_xshift = v5; //97% cover
			cpara.max_lr_yshift = v6;
			cx.push_back(Point(0, 0));
			for (int i = 0; i < cpara.img_num_w - 1; i++) {
				getline(fr, item, ',');
				int x, y;
				if (sscanf(item.c_str(), "%d %d", &x, &y) == 2)
					cx.push_back(cx.back() + Point(x, y)); //cx[n] = cx[n-1]+offset
				else {
					config_file.clear();
					break;
				}
			}
			qInfo("UI: read cpara config, cx1=(%d,%d), ncx1=(%d,%d), cxn=(%d,%d)", cx[1].x, cx[1].y, 
				cx[1].x * (cpara.img_num_w - 1), cx[1].y * (cpara.img_num_w - 1), cx.back().x, cx.back().y);
		}
		if (state == 2) {
			int v1, v2, v3, v4, v5, v6;
			if (sscanf(item.c_str(), "UD err distribute,%d %d,%d %d,%d %d,", &v1, &v2, &v3, &v4, &v5, &v6) != 6) {
				config_file.clear();
				break;
			}
			cpara.max_ud_xshift = v5;
			cpara.max_ud_yshift = v6;
			cy.push_back(Point(0, 0));
			for (int i = 0; i < cpara.img_num_h - 1; i++) {
				getline(fr, item, ',');
				int x, y;
				if (sscanf(item.c_str(), "%d %d", &x, &y) == 2)
					cy.push_back(cy.back() + Point(x, y)); //cy[n] = cy[n-1]+offset
				else {
					config_file.clear();
					break;
				}
			}
			qInfo("UI: read cpara config, cy1=(%d,%d), ncy1=(%d,%d), cyn=(%d,%d)", cy[1].x, cy[1].y, 
				cy[1].x * (cpara.img_num_h - 1), cy[1].y * (cpara.img_num_h - 1), cy.back().x, cy.back().y);
		}
		if (config_file.empty())
			break;
	}

	if (config_file.empty()) {
		QMessageBox::information(this, "Error", "Config file wrong format!");
        ui->config_file->setText("");
	}
	else {
		cpara.offset.create(cpara.img_num_h, cpara.img_num_w);
		for (int y = 0; y < cpara.img_num_h; y++) {
			for (int x = 0; x < cpara.img_num_w; x++) {
				Point xy = cx[x] + cy[y]; //this can make up-down offset as cy[y]-cy[y-1], left-right offset as cx[x]-cx[x-1]
				cpara.offset(y, x)[1] = (xy.x + cpara.rescale / 2) / cpara.rescale * cpara.rescale;
				cpara.offset(y, x)[0] = (xy.y + cpara.rescale / 2) / cpara.rescale * cpara.rescale;
			}
		}
		ui->max_lr_xshift->setText(QString::number(cpara.max_lr_xshift));
		ui->max_lr_yshift->setText(QString::number(cpara.max_lr_yshift));
		ui->max_ud_xshift->setText(QString::number(cpara.max_ud_xshift));
		ui->max_ud_yshift->setText(QString::number(cpara.max_ud_yshift));
		ui->img_num_w->setText(QString::number(cpara.img_num_w));
		ui->img_num_h->setText(QString::number(cpara.img_num_h));
		ui->rescale->setText(QString::number(cpara.rescale));
		ui->init_offset_x->setEnabled(false);
		ui->init_offset_y->setEnabled(false);
		ui->init_offset_udx->setEnabled(false);
		ui->init_offset_lry->setEnabled(false);
		ui->max_lr_xshift->setEnabled(false);
		ui->max_lr_yshift->setEnabled(false);
		ui->max_ud_xshift->setEnabled(false);
		ui->max_ud_yshift->setEnabled(false);
		ui->img_num_w->setEnabled(false);
		ui->img_num_h->setEnabled(false);
		ui->rescale->setEnabled(false);
		ui->new_layer->setChecked(false);
	}
}

void CparaDialog::on_ConfigButton_clicked()
{
    QString dirname = QString::fromStdString("");
    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"),
        dirname,
        tr("Project (*.csv)"));
    if (!filename.isEmpty()) {
        ui->config_file->setText(filename);
    }
}

void CparaDialog::on_max_lr_xshift_textChanged(const QString &arg1)
{
	if (update_scale_en)
		update_scale();
}

void CparaDialog::on_max_lr_yshift_textChanged(const QString &arg1)
{
	if (update_scale_en)
		update_scale();
}

void CparaDialog::on_max_ud_xshift_textChanged(const QString &arg1)
{
	if (update_scale_en)
		update_scale();
}

void CparaDialog::on_max_ud_yshift_textChanged(const QString &arg1)
{
	if (update_scale_en)
		update_scale();
}

void CparaDialog::update_scale()
{
	int max_lr_xshift = ui->max_lr_xshift->text().toInt();
	int max_lr_yshift = ui->max_lr_yshift->text().toInt();
	int max_ud_xshift = ui->max_ud_xshift->text().toInt();
	int max_ud_yshift = ui->max_ud_yshift->text().toInt();
	
	int s = max(max_lr_xshift * max_lr_yshift, max_ud_xshift * max_ud_yshift);
	if (s <= 960)
		ui->rescale->setText(QString::number(1));
	else
	if (s <= 3900)
		ui->rescale->setText(QString::number(2));
	else
		ui->rescale->setText(QString::number(4));
}
