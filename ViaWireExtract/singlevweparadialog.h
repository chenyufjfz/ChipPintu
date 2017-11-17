#ifndef SINGLEVWEPARADIALOG_H
#define SINGLEVWEPARADIALOG_H

#include <QDialog>

namespace Ui {
class SingleVWEParaDialog;
}

class SingleVWEParaDialog : public QDialog
{
    Q_OBJECT

public:
    unsigned wmin, wmax;
    unsigned opt;
    unsigned gray_w, gray_i, w_wide;
    double w_wide1, i_wide;
    explicit SingleVWEParaDialog(QWidget *parent = 0, unsigned _wmin = 1, unsigned _wmax=50, unsigned _opt=1,
             unsigned _gw=70, unsigned _gi=20, unsigned _ww=10, double _ww1=0.3, double _iw=0.4);
    ~SingleVWEParaDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::SingleVWEParaDialog *ui;
};

#endif // SINGLEVWEPARADIALOG_H
