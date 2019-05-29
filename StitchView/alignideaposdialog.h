#ifndef ALIGNIDEAPOSDIALOG_H
#define ALIGNIDEAPOSDIALOG_H

#include <QDialog>

namespace Ui {
class AlignIdeaPosDialog;
}

class AlignIdeaPosDialog : public QDialog
{
    Q_OBJECT

public:
    int w0, w1;
    explicit AlignIdeaPosDialog(QWidget *parent = 0, int _w0=2, int _w1=2);
    ~AlignIdeaPosDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::AlignIdeaPosDialog *ui;
};

#endif // ALIGNIDEAPOSDIALOG_H
