#ifndef GRIDDIALOG_H
#define GRIDDIALOG_H

#include <QDialog>

namespace Ui {
class GridDialog;
}

class GridDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GridDialog(QWidget *parent, double gh, double gw, double oy, double ox);
    ~GridDialog();
    double grid_high, grid_width;
    double offset_y, offset_x;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::GridDialog *ui;
};

#endif // GRIDDIALOG_H
