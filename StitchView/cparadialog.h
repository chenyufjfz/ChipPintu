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
    bool new_layer; //As input, it is to distinguish init or next. As output, it is for new layer 
	string config_file;
    explicit CparaDialog(ConfigPara _cpara, bool _new_layer, QWidget *parent = 0);
    ~CparaDialog();

private slots:
    void on_buttonBox_accepted();

    void on_config_file_textChanged(const QString &arg1);

    void on_ConfigButton_clicked();

private:
    Ui::CparaDialog *ui;
};

#endif // CPARADIALOG_H
