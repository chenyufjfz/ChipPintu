#ifndef XYDIALOG_H
#define XYDIALOG_H

#include <QDialog>

namespace Ui {
class XYDialog;
}

class XYDialog : public QDialog
{
    Q_OBJECT

public:
    int x, y;
    explicit XYDialog(QWidget *parent = 0);
    ~XYDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::XYDialog *ui;
};

#endif // XYDIALOG_H
