#include "wireviaparamdialog.h"
#include "ui_wireviaparamdialog.h"

WireViaParamDialog::WireViaParamDialog(QWidget *parent, int _wire_width, int _via_radius) :
    QDialog(parent),
    ui(new Ui::WireViaParamDialog)
{
    ui->setupUi(this);
    ui->wire_width->setText(QString::number(_wire_width));
    ui->via_radius->setText(QString::number(_via_radius));
}

WireViaParamDialog::~WireViaParamDialog()
{
    delete ui;
}

void WireViaParamDialog::on_buttonBox_accepted()
{
    wire_width = ui->wire_width->text().toUInt();
    via_radius = ui->via_radius->text().toUInt();
}
