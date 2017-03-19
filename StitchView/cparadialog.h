#ifndef CPARADIALOG_H
#define CPARADIALOG_H

#include <QDialog>
#include "featext.h"

namespace Ui {
class CparaDialog;
}

class CparaDialog : public QDialog
{
    Q_OBJECT

public:
    ConfigPara cpara;
    bool new_layer;
    explicit CparaDialog(ConfigPara _cpara, bool _new_layer, QWidget *parent = 0);
    ~CparaDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::CparaDialog *ui;
};

#endif // CPARADIALOG_H
