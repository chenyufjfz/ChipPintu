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
	void title_change(QString title);

private slots:
    void on_actionConfig_Para_triggered();

    void on_actionTune_Para_triggered();

    void on_actionPrepare_Next_Iter_triggered();

    void on_actionQuick_Save_triggered();

    void on_actionQuick_Load_triggered();

    void on_actionOptimize_Offset_triggered();

    void on_actionTransForm_triggered();

    void on_actionLoad_triggered();

    void on_actionSave_as_triggered();

    void on_actionOutput_layer_triggered();

    void on_actionOutput_all_triggered();

    void on_actionDrawGrid_triggered();

    void on_actionAddNail_triggered();

    void on_actionDelete_layer_triggered();

    void on_actionSelectNail_triggered();

    void on_actionLayer_Up_triggered();

    void on_actionLayer_Down_triggered();

private:
    Ui::MainWindow *ui;
	QLabel *status_label;
	QProgressBar * pProgressBar;
    StitchView * stitch_view;
};

#endif // MAINWINDOW_H
