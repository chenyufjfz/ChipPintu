#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "stitchview.h"
#include <QLabel>
#include <QProgressBar>

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
	void mouse_change(QString info);
	void notify_progress(float progress);

private slots:
    void on_actionConfig_Para_triggered();

    void on_actionTune_Para_triggered();

    void on_actionPrepare_Next_Iter_triggered();

    void on_actionQuick_Save_triggered();

    void on_actionQuick_Load_triggered();

    void on_actionOptimize_Offset_triggered();

private:
    Ui::MainWindow *ui;
	QLabel *status_label;
	QProgressBar * pProgressBar;
    StitchView * stitch_view;
};

#endif // MAINWINDOW_H
