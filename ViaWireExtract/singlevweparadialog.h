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
	int shape_mask;
	int dia0, dia1, dia2, dia3, dia4, dia5, dia6, dia7;
	int via_at_center;
    explicit SingleVWEParaDialog(QWidget *parent, unsigned _wmin, unsigned _wmax, unsigned _gray_th, unsigned _sep,
		unsigned _opt, unsigned _gw, unsigned _gi, unsigned _ww, double _ww1, double _iw, int _shape_mask,
		int _dia0, int _dia1, int _dia2, int _dia3, int _dia4, int _dia5, int _dia6, int _dia7, int _via_at_center);
    ~SingleVWEParaDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::SingleVWEParaDialog *ui;
};

#endif // SINGLEVWEPARADIALOG_H
