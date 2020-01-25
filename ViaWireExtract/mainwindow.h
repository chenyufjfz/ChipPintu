#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QScrollArea>
#include <QLabel>
#include "viawireeditview.h"

namespace Ui {
class MainWindow;
}

struct TrainParam {
	int train_what;
    int feature;
    int iter_num;
    float param1;
    float param2;
    float param3;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionLoad_Image_triggered();

    void on_actionGenerate_Grid_triggered();

    void on_actionMark_Insulator_triggered();

    void on_actionMark_Wire_triggered();

    void on_actionMark_Via_triggered();

    void on_actionSelect_triggered();

    void on_actionSave_Objects_triggered();

    void on_actionLoad_Objects_triggered();

    void on_actionClear_All_triggered();

    void mouse_change(QPoint pos, QString msg);

    void on_actionStart_Train_triggered();

    void on_actionShow_Wire_triggered();

    void on_actionShow_Via_triggered();

    void on_actionShow_Wire_Edge_triggered();

    void on_actionShow_Via_Edge_triggered();

	void on_actionShow_Via_Wire_Edge_triggered();

    void on_actionZoom_in_triggered();

    void on_actionZoom_out_triggered();

    void on_actionShow_Select_triggered();

    void on_actionSet_Param_triggered();

    void on_actionExtract_triggered();

    void on_actionMark_Cell_triggered();

    void on_actionSingle_Wire_Para_triggered();

    void on_actionMark_NoVia_triggered();

    void on_actionMark_Via_Wire_triggered();

    void on_actionMark_Via_NoWire_triggered();

private:
    QLabel *status_label;
    ViaWireEditView * vw_view;
    QScrollArea *scroll_view;
    Ui::MainWindow *ui;
    QString image_file_name;
    unsigned int vw_view_mask;
    TrainParam train_param;
};

#endif // MAINWINDOW_H
