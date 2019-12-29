#include "showmaskdialog.h"
#include "ui_showmaskdialog.h"
#include "vwextract.h"

ShowMaskDialog::ShowMaskDialog(QWidget *parent, unsigned _mask) :
    QDialog(parent),
    ui(new Ui::ShowMaskDialog)
{
    ui->setupUi(this);

}

ShowMaskDialog::~ShowMaskDialog()
{
    delete ui;
}

void ShowMaskDialog::on_buttonBox_accepted()
{
    mask = 0;

}
