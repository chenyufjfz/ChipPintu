#ifndef TRAINDIALOG_H
#define TRAINDIALOG_H

#include <QDialog>

namespace Ui {
class TrainDialog;
}

class TrainDialog : public QDialog
{
    Q_OBJECT

public:
    int iter_num;
    float param1;
    int feature;
    float param2;
    float param3;

public:
    explicit TrainDialog(QWidget *parent = 0, int _feature=1, int _iter_num=1000, float _param1=0.1,
                          float _param2=0, float _param3=0);
    ~TrainDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::TrainDialog *ui;
};

#endif // TRAINDIALOG_H
