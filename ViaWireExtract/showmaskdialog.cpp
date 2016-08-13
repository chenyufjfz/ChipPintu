#include "showmaskdialog.h"
#include "ui_showmaskdialog.h"
#include "vwextract.h"

ShowMaskDialog::ShowMaskDialog(QWidget *parent, unsigned _mask) :
    QDialog(parent),
    ui(new Ui::ShowMaskDialog)
{
    ui->setupUi(this);
    if ((_mask >> M_W) & 1)
        ui->wire->setChecked(true);
    if ((_mask >> M_V) & 1)
        ui->via->setChecked(true);
    if ((_mask >> M_W_I) & 1)
        ui->wire_edge->setChecked(true);
    if ((_mask >> M_V_I) & 1)
        ui->via_edge->setChecked(true);
    if ((_mask >> M_I) & 1)
        ui->insu->setChecked(true);
    if ((_mask >> M_W_V) & 1)
        ui->wire_via->setChecked(true);
    if ((_mask >> M_V_I_W) & 1)
        ui->wire_insu_via->setChecked(true);
    if ((_mask >> M_V_I_V) & 1)
        ui->via_insu_via->setChecked(true);
}

ShowMaskDialog::~ShowMaskDialog()
{
    delete ui;
}

void ShowMaskDialog::on_buttonBox_accepted()
{
    mask = 0;
    if (ui->wire->isChecked())
        mask |= 1 << M_W;
    if (ui->via->isChecked())
        mask |= 1 << M_V;
    if (ui->wire_edge->isChecked())
        mask |= 1 << M_W_I;
    if (ui->via_edge->isChecked())
        mask |= 1 << M_V_I;
    if (ui->insu->isChecked())
        mask |= 1 << M_I;
    if (ui->wire_via->isChecked())
        mask |= 1 << M_W_V;
    if (ui->wire_insu_via->isChecked())
        mask |= 1 << M_V_I_W;
    if (ui->via_insu_via->isChecked())
        mask |= 1 << M_V_I_V;
}
