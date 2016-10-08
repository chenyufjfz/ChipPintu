#include "wireviaparamdialog.h"
#include "ui_wireviaparamdialog.h"

WireViaParamDialog::WireViaParamDialog(QWidget *parent, int _wire_width, int _via_radius, int _insu_gap, int _grid_size) :
    QDialog(parent),
    ui(new Ui::WireViaParamDialog)
{
    ui->setupUi(this);
    ui->wire_width->setText(QString::number(_wire_width));
    ui->via_radius->setText(QString::number(_via_radius));
    ui->insu_gap->setText(QString::number(_insu_gap));
    ui->grid_size->setText(QString::number(_grid_size));
}

WireViaParamDialog::~WireViaParamDialog()
{
    delete ui;
}

void WireViaParamDialog::on_buttonBox_accepted()
{
    wire_width = ui->wire_width->text().toUInt();
    via_radius = ui->via_radius->text().toUInt();
    insu_gap = ui->insu_gap->text().toUInt();
    grid_size = ui->grid_size->text().toUInt();
}
