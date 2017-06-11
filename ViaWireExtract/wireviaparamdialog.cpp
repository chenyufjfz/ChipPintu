#include "wireviaparamdialog.h"
#include "ui_wireviaparamdialog.h"

WireViaParamDialog::WireViaParamDialog(QWidget *parent, int _layer, int _type, int _opt0, int _opt1,
						int _opt2, int _opt3, int _opt4, int _opt5, int _opt6, float _opt_f0) :
    QDialog(parent),
    ui(new Ui::WireViaParamDialog)
{
    ui->setupUi(this);
    ui->layer->setText(QString::number(_layer));
	ui->type->setText(QString::number(_type));
    ui->opt0->setText(QString::number(_opt0));    
    ui->opt1->setText(QString::number(_opt1));
    ui->opt2->setText(QString::number(_opt2));
	ui->opt3->setText(QString::number(_opt3));
    ui->opt4->setText(QString::number(_opt4));
    ui->opt5->setText(QString::number(_opt5));
    ui->opt6->setText(QString::number(_opt6));
	ui->opt_f0->setText(QString::number(_opt_f0));
}

WireViaParamDialog::~WireViaParamDialog()
{
    delete ui;
}

void WireViaParamDialog::on_buttonBox_accepted()
{
    layer = ui->layer->text().toUInt();
	type = ui->type->text().toUInt();
    opt0 = ui->opt0->text().toUInt();    
    opt1 = ui->opt1->text().toUInt();
    opt2 = ui->opt2->text().toUInt();
	opt3 = ui->opt3->text().toUInt();
	opt4 = ui->opt4->text().toUInt();
	opt5 = ui->opt5->text().toUInt();
	opt6 = ui->opt6->text().toUInt();
	opt_f0 = ui->opt_f0->text().toFloat();
}
