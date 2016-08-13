#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QDebug>
#include "gridcfgdialog.h"
#include <QMessageBox>
#include <QScrollBar>
#include "showmaskdialog.h"
#include "traindialog.h"
#include "wireviaparamdialog.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    vw_view = new ViaWireEditView(this);

    scroll_view = new QScrollArea;
    scroll_view->setBackgroundRole(QPalette::Dark);
    scroll_view->setWidget(vw_view);
    setCentralWidget(scroll_view);
	vw_view->setFocus();

    status_label = new QLabel;
    status_label->setMinimumSize(200, 20);
    ui->statusbar->addWidget(status_label);

    connect(vw_view, SIGNAL(mouse_change(QPoint)), this, SLOT(mouse_change(QPoint)));

    QActionGroup* mark_action_group = new QActionGroup(this);
    mark_action_group->addAction(ui->actionMark_Insulator);
    mark_action_group->addAction(ui->actionMark_Via);
    mark_action_group->addAction(ui->actionMark_Wire);
    mark_action_group->addAction(ui->actionSelect);
    ui->actionSelect->setChecked(true);

    ui->actionShow_Via->setChecked(false);
    ui->actionShow_Via_Edge->setChecked(false);
    ui->actionShow_Via_Wire_Edge->setChecked(false);
    ui->actionShow_Wire->setChecked(false);
    ui->actionShow_Wire_Edge->setChecked(false);
    vw_view_mask = 0;
    train_param.feature = 1;
    train_param.iter_num = 1000;
    train_param.param1 = 1;
    train_param.param2 = 0;
    train_param.param3 = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionLoad_Image_triggered()
{
    image_file_name = QFileDialog::getOpenFileName( this,
                        "open file",
                        "C:/chenyu/work/ChipPintu/images/",
                        "Images (*.png *.xpm *.jpg)");

	vw_view->load_bk_image(image_file_name);
}

void MainWindow::on_actionGenerate_Grid_triggered()
{
    GridCfgDialog grid_cfg_dlg(this, vw_view->grid_high, vw_view->grid_width,
                               vw_view->offset_y, vw_view->offset_x);
    if (grid_cfg_dlg.exec() == QDialog::Accepted) {
		vw_view->set_grid_size(grid_cfg_dlg.grid_high, grid_cfg_dlg.grid_width);
		vw_view->set_offset(grid_cfg_dlg.offset_y, grid_cfg_dlg.offset_x);
    }
    /*while (grid_cfg_dlg.grid_high==0 || grid_cfg_dlg.grid_width==0) {
        int ret = QMessageBox::warning(this, "Warning", "Invalid Grid high or Grid width, reconfig?",
                             QMessageBox::Ok, QMessageBox::Cancel);
        if (ret==QMessageBox::Ok)
            grid_cfg_dlg.exec();
        else
            break;
    }*/
}

void MainWindow::on_actionMark_Insulator_triggered()
{
    vw_view->set_mark(OBJ_AREA, AREA_LEARN);
}

void MainWindow::on_actionMark_Wire_triggered()
{
	vw_view->set_mark(OBJ_WIRE, 0);
}

void MainWindow::on_actionMark_Via_triggered()
{
	vw_view->set_mark(OBJ_VIA, 0);
}

void MainWindow::on_actionSelect_triggered()
{
	vw_view->set_mark(SELECT_OBJ, 0);
}

void MainWindow::on_actionSave_Objects_triggered()
{
    std::string file_name = image_file_name.toStdString();
    file_name.erase(file_name.size()-4);
	file_name.append(".xml");
	
	vw_view->save_objects(QString::fromStdString(file_name));
}

void MainWindow::on_actionLoad_Objects_triggered()
{
    std::string file_name = image_file_name.toStdString();
    file_name.erase(file_name.size()-4);
    file_name.append(".xml");

	vw_view->load_objects(QString::fromStdString(file_name));
}

void MainWindow::on_actionClear_All_triggered()
{
	vw_view->erase_all_objects();
}

void MainWindow::mouse_change(QPoint pos)
{
    char s[100];
    sprintf(s, "x:%d,y:%d", pos.x(), pos.y());
    status_label->setText(s);
}

void MainWindow::on_actionStart_Train_triggered()
{
    TrainDialog train_dlg(this, train_param.feature, train_param.iter_num,
                          train_param.param1, train_param.param2, train_param.param3);
    if (train_dlg.exec() == QDialog::Accepted) {
        train_param.feature = train_dlg.feature;
        train_param.iter_num = train_dlg.iter_num;
        train_param.param1 = train_dlg.param1;
        train_param.param2 = train_dlg.param2;
        train_param.param3 = train_dlg.param3;
        vw_view->start_train(train_param.feature, train_param.iter_num,
                             train_param.param1, train_param.param2, train_param.param3);
    }
}

void MainWindow::on_actionShow_Wire_triggered(bool checked)
{
    if (checked)
        vw_view_mask |= (1<<M_W);
    else
        vw_view_mask &= ~(1<<M_W);
    vw_view->set_mark(vw_view_mask);
}

void MainWindow::on_actionShow_Via_triggered(bool checked)
{
    if (checked)
        vw_view_mask |= (1<<M_V);
    else
        vw_view_mask &= ~(1<<M_V);
    vw_view->set_mark(vw_view_mask);
}

void MainWindow::on_actionShow_Wire_Edge_triggered(bool checked)
{
    if (checked)
        vw_view_mask |= (1<<M_W_I);
    else
        vw_view_mask &= ~(1<<M_W_I);
    vw_view->set_mark(vw_view_mask);
}

void MainWindow::on_actionShow_Via_Edge_triggered(bool checked)
{
    if (checked)
        vw_view_mask |= (1<<M_V_I) | (1<<M_W_V);
    else
        vw_view_mask &= ~((1<<M_V_I) | (1<<M_W_V));
    vw_view->set_mark(vw_view_mask);
}

void MainWindow::on_actionShow_Via_Wire_Edge_triggered(bool checked)
{
    if (checked)
        vw_view_mask |= (1<<M_V_I_V) | (1<<M_V_I_W);
    else
        vw_view_mask &= ~((1<<M_V_I_V) | (1<<M_V_I_W));
    vw_view->set_mark(vw_view_mask);
}


void MainWindow::on_actionShow_Select_triggered()
{
    ShowMaskDialog show_mask_dlg(this, vw_view_mask);
    if (show_mask_dlg.exec() == QDialog::Accepted) {
        vw_view_mask = show_mask_dlg.mask;
        vw_view->set_mark(vw_view_mask);
    }
}

void MainWindow::on_actionZoom_in_triggered()
{
    int scale = vw_view->get_scale();
    int x = scroll_view->horizontalScrollBar()->value() / scale;
    int y = scroll_view->verticalScrollBar()->value() / scale;
    vw_view->set_scale(scale * 2);
    scale = vw_view->get_scale();
    scroll_view->horizontalScrollBar()->setValue(x*scale);
    scroll_view->verticalScrollBar()->setValue(y*scale);
}

void MainWindow::on_actionZoom_out_triggered()
{
    int scale = vw_view->get_scale();
    int x = scroll_view->horizontalScrollBar()->value() / scale;
    int y = scroll_view->verticalScrollBar()->value() / scale;
    vw_view->set_scale(scale / 2);
    scale = vw_view->get_scale();
    scroll_view->horizontalScrollBar()->setValue(x*scale);
    scroll_view->verticalScrollBar()->setValue(y*scale);
}

void MainWindow::on_actionSet_Param_triggered()
{
    WireViaParamDialog wv_dlg(this, vw_view->wire_width, vw_view->via_radius);
    if (wv_dlg.exec() == QDialog::Accepted)
        vw_view->set_para(wv_dlg.wire_width, wv_dlg.via_radius);

}

void MainWindow::on_actionExtract_triggered()
{
    vw_view->extract();
}
