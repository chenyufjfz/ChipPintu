#include "traindialog.h"
#include "ui_traindialog.h"

TrainDialog::TrainDialog(QWidget *parent, int _train_what, int _feature, int _iter_num, float _param1,
                         float _param2, float _param3) :
    QDialog(parent),
    ui(new Ui::TrainDialog)
{
    ui->setupUi(this);
    ui->param2->setText(QString::number(_param2));
    ui->iter_num->setText(QString::number(_iter_num));
    ui->param1->setText(QString::number(_param1));
    ui->param3->setText(QString::number(_param3));
}

TrainDialog::~TrainDialog()
{
    delete ui;
}

void TrainDialog::on_buttonBox_accepted()
{
    param2 = ui->param2->text().toFloat();
    param3 = ui->param3->text().toFloat();
    iter_num = ui->iter_num->text().toUInt();
    param1 = ui->param1->text().toFloat();

    feature =1;
}
