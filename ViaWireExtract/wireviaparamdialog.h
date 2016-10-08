#ifndef WIREVIAPARAMDIALOG_H
#define WIREVIAPARAMDIALOG_H

#include <QDialog>

namespace Ui {
class WireViaParamDialog;
}

class WireViaParamDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WireViaParamDialog(QWidget *parent = 0, int _wire_width=9, int _via_radius=13, int _insu_gap=8, int _grid_size=19);
    ~WireViaParamDialog();
    int wire_width;
    int via_radius;
    int insu_gap;
    int grid_size;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::WireViaParamDialog *ui;
};

#endif // WIREVIAPARAMDIALOG_H
