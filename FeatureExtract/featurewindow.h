#ifndef FEATUREWINDOW_H
#define FEATUREWINDOW_H

#include <QMainWindow>

namespace Ui {
class FeatureWindow;
}

class FeatureWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit FeatureWindow(QWidget *parent = 0);
    ~FeatureWindow();

private:
    Ui::FeatureWindow *ui;
};

#endif // FEATUREWINDOW_H
