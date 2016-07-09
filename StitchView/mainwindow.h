#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "stitchview.h"
#include <QLabel>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected slots:
	void mouse_change(QPoint pos);

private:
    Ui::MainWindow *ui;
	QLabel *status_label;
    StitchView * stitch_view;
};

#endif // MAINWINDOW_H
