#include "xydialog.h"
#include "ui_xydialog.h"

XYDialog::XYDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::XYDialog)
{
    ui->setupUi(this);
    ui->gox->setText(QString::number(0));
    ui->goy->setText(QString::number(0));
}

XYDialog::~XYDialog()
{
    delete ui;
}

void XYDialog::on_buttonBox_accepted()
{
    y = ui->goy->text().toInt();
    x = ui->gox->text().toInt();
}
