#ifndef GRIDCFGDIALOG_H
#define GRIDCFGDIALOG_H

#include <QDialog>

namespace Ui {
class GridCfgDialog;
}

class GridCfgDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GridCfgDialog(QWidget *parent = 0, double gh=0, double gw=0, double oy=0, double ox=0);
    ~GridCfgDialog();
    double grid_high, grid_width;
    double offset_y, offset_x;
private slots:
    void on_buttonBox_accepted();

private:
    Ui::GridCfgDialog *ui;
};

#endif // GRIDCFGDIALOG_H
