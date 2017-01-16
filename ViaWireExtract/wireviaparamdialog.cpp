#include "wireviaparamdialog.h"
#include "ui_wireviaparamdialog.h"

WireViaParamDialog::WireViaParamDialog(QWidget *parent, int _layer, int _wire_width, int _via_radius, int _grid_size,
						int _rule, int _warn_rule, float _param1, float _param2, float _param3, float _param4) :
    QDialog(parent),
    ui(new Ui::WireViaParamDialog)
{
    ui->setupUi(this);
    ui->layer->setText(QString::number(_layer));
    ui->wire_width->setText(QString::number(_wire_width));
    ui->via_radius->setText(QString::number(_via_radius));    
    ui->grid_size->setText(QString::number(_grid_size));
    ui->rule->setText(QString::number(_rule));
	ui->warn_rule->setText(QString::number(_warn_rule));
    ui->param1->setText(QString::number(_param1));
    ui->param2->setText(QString::number(_param2));
    ui->param3->setText(QString::number(_param3));
	ui->param4->setText(QString::number(_param4));
}

WireViaParamDialog::~WireViaParamDialog()
{
    delete ui;
}

void WireViaParamDialog::on_buttonBox_accepted()
{
    layer = ui->layer->text().toUInt();
    wire_width = ui->wire_width->text().toUInt();
    via_radius = ui->via_radius->text().toUInt();    
    grid_size = ui->grid_size->text().toUInt();
    rule = ui->rule->text().toUInt();
	warning_rule = ui->warn_rule->text().toUInt();
    param1 = ui->param1->text().toFloat();
    param2 = ui->param2->text().toFloat();
    param3 = ui->param3->text().toFloat();
	param4 = ui->param4->text().toFloat();
}
