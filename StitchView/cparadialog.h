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
	bool update_scale_en; //when lr up shift changed, auto update scale
	string config_file;
	explicit CparaDialog(ConfigPara _cpara, bool _new_layer, bool _update_scale_en, QWidget *parent = 0);
    ~CparaDialog();

private slots:
    void on_buttonBox_accepted();

    void on_config_file_textChanged(const QString &arg1);

    void on_ConfigButton_clicked();

    void on_max_lr_xshift_textChanged(const QString &arg1);

    void on_max_lr_yshift_textChanged(const QString &arg1);

    void on_max_ud_xshift_textChanged(const QString &arg1);

    void on_max_ud_yshift_textChanged(const QString &arg1);

private:
	void update_scale();
    Ui::CparaDialog *ui;
};

#endif // CPARADIALOG_H
