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
	explicit WireViaParamDialog(QWidget *parent, int _layer, int _type, int _opt0, int _opt1,
		int _opt2, int _opt3, int _opt4, int _opt5, int _opt6, float _opt_f0);
    ~WireViaParamDialog();
    int layer, type, opt0, opt1, opt2, opt3, opt4, opt5, opt6;
	float opt_f0;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::WireViaParamDialog *ui;
};

#endif // WIREVIAPARAMDIALOG_H
