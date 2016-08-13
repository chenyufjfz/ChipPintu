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
    explicit WireViaParamDialog(QWidget *parent = 0, int _wire_width=9, int _via_radius=13);
    ~WireViaParamDialog();
    int wire_width;
    int via_radius;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::WireViaParamDialog *ui;
};

#endif // WIREVIAPARAMDIALOG_H
