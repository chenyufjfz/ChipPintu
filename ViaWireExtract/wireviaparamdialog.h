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
    explicit WireViaParamDialog(QWidget *parent, int _layer, int _wire_width, int _via_radius, int _grid_size,
                                int _rule, int _warn_rule, float _param1, float _param2, float _param3, float _param4);
    ~WireViaParamDialog();
    int wire_width, via_radius, layer, grid_size, rule, warning_rule;
    float param1, param2, param3, param4;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::WireViaParamDialog *ui;
};

#endif // WIREVIAPARAMDIALOG_H
