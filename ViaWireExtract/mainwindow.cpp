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
#include "singlevweparadialog.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    vw_view = new ViaWireEditView(NULL);

    scroll_view = new QScrollArea;
    scroll_view->setBackgroundRole(QPalette::Dark);
    scroll_view->setWidget(vw_view);
	scroll_view->setFocusPolicy(Qt::NoFocus);
	scroll_view->horizontalScrollBar()->setFocusPolicy(Qt::NoFocus);
	scroll_view->verticalScrollBar()->setFocusPolicy(Qt::NoFocus);
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
	mark_action_group->addAction(ui->actionMark_NoVia);
	mark_action_group->addAction(ui->actionMark_Wire_Insu);
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

	if (image_file_name.length()>0)
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

}

void MainWindow::on_actionMark_Insulator_triggered()
{
	vw_view->set_mark(OBJ_POINT, POINT_INSU);
}

void MainWindow::on_actionMark_Wire_triggered()
{
	vw_view->set_mark(OBJ_POINT, POINT_WIRE);
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
	if (file_name.empty())
		return;
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

void MainWindow::on_actionShow_Wire_triggered()
{
	vw_view->show_debug(0, true);
	
}

void MainWindow::on_actionShow_Via_triggered()
{
	vw_view->show_debug(1, true);
}

void MainWindow::on_actionShow_Wire_Edge_triggered()
{
	vw_view->show_debug(2, true);
}

void MainWindow::on_actionShow_Via_Edge_triggered()
{
	vw_view->show_debug(3, true);
}

void MainWindow::on_actionShow_Via_Wire_Edge_triggered()
{
	vw_view->show_debug(0, false);
}


void MainWindow::on_actionShow_Select_triggered()
{
    ShowMaskDialog show_mask_dlg(this, vw_view_mask);
    if (show_mask_dlg.exec() == QDialog::Accepted) {
        vw_view_mask = show_mask_dlg.mask;
		vw_view->show_debug(vw_view_mask, false);
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
	if (image_file_name.length() == 0)
		return;
	std::string file_name = image_file_name.toStdString();
	file_name.erase(file_name.find_last_of("/\\"));
	file_name.append("/action.xml");
	ExtractParam ep;
	ep.read_file(file_name);
	WireViaParamDialog wv_dlg(this, &ep);
	if (wv_dlg.exec() == QDialog::Accepted)
		vw_view->set_wire_para(&ep, wv_dlg.action_name);
	
}

void MainWindow::on_actionExtract_triggered()
{
    vw_view->extract();
}


void MainWindow::on_actionSingle_Wire_Para_triggered()
{
    int wmin, wmax, opt, gray_i, gray_w, w_wide, gray_th, sep, shape_mask;
	double i_wide, w_wide1;
	int d0, d1, d2, d3, d4, d5, d6, d7;
	int wi0, wi1, wi2, wi3, wi4, wi5, wi6, wi7;
	int via_at_center;
    vw_view->get_single_wire_ext_para(wmin, wmax, opt, gray_th, sep, shape_mask);
	vw_view->get_brick_shape_ext_para(w_wide, i_wide, w_wide1, gray_i, gray_w);
	vw_view->get_via_diameter(d0, d1, d2, d3, d4, d5, d6, d7);
	vw_view->get_wid(wi0, wi1, wi2, wi3, wi4, wi5, wi6, wi7);
	via_at_center = vw_view->get_via_center();
	SingleVWEParaDialog sd(this, wmin, wmax, gray_th, sep, opt, gray_w, gray_i, w_wide, w_wide1, i_wide, shape_mask,
		d0, d1, d2, d3, d4, d5, d6, d7, wi0, wi1, wi2, wi3, wi4, wi5, wi6, wi7, via_at_center);
	if (sd.exec() == QDialog::Accepted) {
        vw_view->set_single_wire_ext_para(sd.wmin, sd.wmax, sd.opt, sd.gray_th, sd.sep, sd.shape_mask);
		vw_view->set_brick_shape_ext_para(sd.w_wide, sd.i_wide, sd.w_wide1, sd.gray_i, sd.gray_w);
		vw_view->set_via_diameter(sd.dia0, sd.dia1, sd.dia2, sd.dia3, sd.dia4, sd.dia5, sd.dia6, sd.dia7);
		vw_view->set_via_center(sd.via_at_center);
		vw_view->set_wid(sd.wi0, sd.wi1, sd.wi2, sd.wi3, sd.wi4, sd.wi5, sd.wi6, sd.wi7);
	}
}

void MainWindow::on_actionMark_NoVia_triggered()
{
	vw_view->set_mark(OBJ_POINT, POINT_NO_VIA);
}

void MainWindow::on_actionMark_Via_Wire_triggered()
{
	vw_view->set_mark(OBJ_POINT, POINT_WIRE_V);
}

void MainWindow::on_actionMark_Via_NoWire_triggered()
{
	vw_view->set_mark(OBJ_POINT, POINT_INSU_V);
}

void MainWindow::on_actionMark_Learn_Area_triggered()
{
	vw_view->set_mark(OBJ_AREA, AREA_LEARN);
}

void MainWindow::on_actionMark_Wire_Insu_triggered()
{
	vw_view->set_mark(OBJ_POINT, POINT_WIRE_INSU);
}

void MainWindow::on_actionMark_Via_Wire_Insu_triggered()
{
    vw_view->set_mark(OBJ_POINT, POINT_WIRE_INSU_V);
}

void MainWindow::on_actionShow_mark4_triggered()
{
	vw_view->show_debug(4, true);
}

void MainWindow::on_actionShow_mark5_triggered()
{
	vw_view->show_debug(5, true);
}

void MainWindow::on_actionShow_mark6_triggered()
{
	vw_view->show_debug(6, true);
}
