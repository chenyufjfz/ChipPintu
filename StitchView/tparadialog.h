#ifndef TPARADIALOG_H
#define TPARADIALOG_H

#include <QDialog>
#include "featext.h"

namespace Ui {
class TParaDialog;
}

class TParaDialog : public QDialog
{
    Q_OBJECT

public:
    TuningPara tpara;
    explicit TParaDialog(TuningPara _tpara, QWidget *parent = 0);
    ~TParaDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::TParaDialog *ui;
};

#endif // TPARADIALOG_H
