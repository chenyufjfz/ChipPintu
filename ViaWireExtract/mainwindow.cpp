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
	scroll_view->setFocusPolicy(Qt::NoFocus);
    setCentralWidget(scroll_view);
	vw_view->setFocus();

    status_label = new QLabel;
    status_label->setMinimumSize(200, 20);
    ui->statusbar->addWidget(status_label);

    connect(vw_view, SIGNAL(mouse_change(QPoint, QString)), this, SLOT(mouse_change(QPoint, QString)));

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
    train_param.iter_num = 10;
    train_param.param1 = 0.5f;
    train_param.param2 = 0.5f;
    train_param.param3 = 0.5f;
	train_param.train_what = 0;
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
	vw_view->set_wire_para(0, 4, 9, 16, RULE_END_WITH_VIA, RULE_END_WITH_VIA, 0.5, 0.5, 1, 0);
	vw_view->set_wire_para(1, 10, 9, 16, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		RULE_NO_hCONN, 0.5, 0.5, 2, 0);
	vw_view->set_wire_para(2, 12, 10, 16, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		RULE_NO_hCONN, 0.5, 0.5, 2, 0);
	vw_view->set_wire_para(3, 12, 10, 16, RULE_NO_LOOP | RULE_NO_hCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		0, 0.5, 0.5, 1, 0);
}

void MainWindow::on_actionGenerate_Grid_triggered()
{
    GridCfgDialog grid_cfg_dlg(this, vw_view->grid_high, vw_view->grid_width,
                               vw_view->offset_y, vw_view->offset_x);
    if (grid_cfg_dlg.exec() == QDialog::Accepted) {
		vw_view->set_grid_size(grid_cfg_dlg.grid_high, grid_cfg_dlg.grid_width);
		vw_view->set_offset(grid_cfg_dlg.offset_y, grid_cfg_dlg.offset_x);
    }

}

void MainWindow::on_actionMark_Insulator_triggered()
{
    vw_view->set_mark(OBJ_AREA, AREA_LEARN);
}

void MainWindow::on_actionMark_Wire_triggered()
{
	vw_view->set_mark(OBJ_LINE, LINE_NORMAL_WIRE0);
}

void MainWindow::on_actionMark_Cell_triggered()
{
    vw_view->set_mark(OBJ_AREA, AREA_CELL);
}

void MainWindow::on_actionMark_Via_triggered()
{
    vw_view->set_mark(OBJ_POINT, POINT_NORMAL_VIA0);
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

void MainWindow::mouse_change(QPoint pos, QString msg)
{
    char s[200];
    sprintf(s, "x:%d,y:%d, %s", pos.x(), pos.y(), msg.toStdString().c_str());
    status_label->setText(s);
}

void MainWindow::on_actionStart_Train_triggered()
{
	TrainDialog train_dlg(this, train_param.train_what, train_param.feature, train_param.iter_num,
                          train_param.param1, train_param.param2, train_param.param3);
    if (train_dlg.exec() == QDialog::Accepted) {
        train_param.feature = train_dlg.feature;
        train_param.iter_num = train_dlg.iter_num;
        train_param.param1 = train_dlg.param1;
        train_param.param2 = train_dlg.param2;
        train_param.param3 = train_dlg.param3;
		train_param.train_what = train_dlg.train_what;
		vw_view->start_cell_train(0, 0, 0,
                             train_param.param1, train_param.param2, train_param.param3);
    }
}

void MainWindow::on_actionShow_Wire_triggered(bool checked)
{
	if (checked)
		vw_view_mask |= (1 << M_W);
	else
		vw_view_mask &= ~(1 << M_W);
	vw_view->show_mark(vw_view_mask);
	
}

void MainWindow::on_actionShow_Via_triggered(bool checked)
{
    if (checked)
        vw_view_mask |= (1<<M_V);
    else
        vw_view_mask &= ~(1<<M_V);
    vw_view->show_mark(vw_view_mask);
}

void MainWindow::on_actionShow_Wire_Edge_triggered(bool checked)
{
	vw_view->show_debug(checked);
}

void MainWindow::on_actionShow_Via_Edge_triggered(bool checked)
{
    if (checked)
        vw_view_mask |= (1<<M_V_I) | (1<<M_W_V);
    else
        vw_view_mask &= ~((1<<M_V_I) | (1<<M_W_V));
	vw_view->show_mark(vw_view_mask);
}

void MainWindow::on_actionShow_Via_Wire_Edge_triggered(bool checked)
{
    if (checked)
        vw_view_mask |= (1<<M_V_I_V) | (1<<M_V_I_W);
    else
        vw_view_mask &= ~((1<<M_V_I_V) | (1<<M_V_I_W));
	vw_view->show_mark(vw_view_mask);
}


void MainWindow::on_actionShow_Select_triggered()
{
    ShowMaskDialog show_mask_dlg(this, vw_view_mask);
    if (show_mask_dlg.exec() == QDialog::Accepted) {
        vw_view_mask = show_mask_dlg.mask;
		vw_view->show_mark(vw_view_mask);
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
	WireViaParamDialog wv_dlg(this, 0, 10, 9, 16, RULE_NO_LOOP | RULE_NO_UCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA, 0, 
		0.5f, 0.5f, 0.5f, 0);
    if (wv_dlg.exec() == QDialog::Accepted)
		vw_view->set_wire_para(wv_dlg.layer, wv_dlg.wire_width, wv_dlg.via_radius, wv_dlg.grid_size, wv_dlg.rule, wv_dlg.warning_rule,
			wv_dlg.param1, wv_dlg.param2, wv_dlg.param3, wv_dlg.param4);

}

void MainWindow::on_actionExtract_triggered()
{
    vw_view->extract();
}

