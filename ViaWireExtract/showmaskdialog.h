#ifndef SHOWMASKDIALOG_H
#define SHOWMASKDIALOG_H

#include <QDialog>

namespace Ui {
class ShowMaskDialog;
}

class ShowMaskDialog : public QDialog
{
    Q_OBJECT

public:
    unsigned int mask;
    explicit ShowMaskDialog(QWidget *parent = 0, unsigned _mask =0);
    ~ShowMaskDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::ShowMaskDialog *ui;
};

#endif // SHOWMASKDIALOG_H
