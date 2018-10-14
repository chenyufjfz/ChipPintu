#include "griddialog.h"
#include "ui_griddialog.h"

GridDialog::GridDialog(QWidget *parent, double gh, double gw, double oy, double ox) :
    QDialog(parent),
    ui(new Ui::GridDialog)
{
    ui->setupUi(this);
    ui->grid_high->setText(QString::number(gh));
    ui->grid_width->setText(QString::number(gw));
    ui->offset_y->setText(QString::number(oy));
    ui->offset_x->setText(QString::number(ox));
}

GridDialog::~GridDialog()
{
    delete ui;
}

void GridDialog::on_buttonBox_accepted()
{
    grid_high = ui->grid_high->text().toDouble();
    grid_width = ui->grid_width->text().toDouble();
    offset_y = ui->offset_y->text().toDouble();
    offset_x = ui->offset_x->text().toDouble();
}
