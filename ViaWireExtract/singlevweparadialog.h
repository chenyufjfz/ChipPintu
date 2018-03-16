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
    unsigned opt, gray_th, sep;
    unsigned gray_w, gray_i, w_wide;
    double w_wide1, i_wide;
    explicit SingleVWEParaDialog(QWidget *parent, unsigned _wmin, unsigned _wmax, unsigned _gray_th, unsigned _sep,
             unsigned _opt, unsigned _gw, unsigned _gi, unsigned _ww, double _ww1, double _iw);
    ~SingleVWEParaDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::SingleVWEParaDialog *ui;
};

#endif // SINGLEVWEPARADIALOG_H
