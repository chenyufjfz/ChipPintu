#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "cparadialog.h"
#include "tparadialog.h"
#include "mapxydialog.h"
#include "griddialog.h"
#include "xydialog.h"
#include "alignideaposdialog.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QScopedPointer>
#include <QSplitter>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    stitch_view = new StitchView();	
	nview = new NavigateView();
	ce_dlg = new CornerEdge();
	stitch_view->set_nview(nview);
	stitch_view->set_ceview(ce_dlg);
	QSplitter *splitter = new QSplitter(Qt::Horizontal);
	QSplitter *vsplitter = new QSplitter(Qt::Vertical);
	vsplitter->addWidget(nview);
	vsplitter->addWidget(ce_dlg);
	vsplitter->setStretchFactor(0, 1);
	vsplitter->setStretchFactor(1, 3);
	setCentralWidget(splitter);
	splitter->addWidget(vsplitter);
	splitter->addWidget(stitch_view);
	splitter->setStretchFactor(0, 1);
	splitter->setStretchFactor(1, 4);

	stitch_view->setFocus();

	status_label = new QLabel();
    status_label->setMaximumSize(800, 20);
	ui->statusBar->addWidget(status_label);

	pProgressBar = new QProgressBar();
	pProgressBar->setRange(0, 100);
	pProgressBar->setValue(0);
	pProgressBar->setMaximumSize(200, 20);
	ui->statusBar->addPermanentWidget(pProgressBar);

	connect(stitch_view, SIGNAL(MouseChange(QString)), this, SLOT(mouse_change(QString)));
	connect(stitch_view, SIGNAL(notify_progress(float)), this, SLOT(notify_progress(float)));
	connect(stitch_view, SIGNAL(title_change(QString)), this, SLOT(title_change(QString)));

	speed = BUNDLE_ADJUST_SPEED_NORMAL;
	use_default_compare = true;
	ui->actionFast_But_Bad->setChecked(false);
	ui->actionMiddle_And_Normal->setChecked(true);
	ui->actionSlow_And_Good->setChecked(false);
#if !(FUNCTION_MASK & ALLOW_YELLOW_EDGE)
	ui->actionClear_Yellow_Fix_Edge->setVisible(false);
	ui->actionClear_Red_Fix_edge->setVisible(false);
#endif
#if !(FUNCTION_MASK & ALLOW_IMPORT)
	ui->actionImport->setVisible(false);
#endif
#if !(FUNCTION_MASK & ALLOW_FAST_OPTIMIZE)
	ui->actionFast_But_Bad->setVisible(false);
	ui->actionSlow_And_Good->setVisible(false);
	ui->menuOptimize_Speed->menuAction()->setVisible(false);
#endif
#if !(FUNCTION_MASK & ALLOW_TUNE)
	ui->actionTune_Para->setVisible(false);
#endif
	ui->actionChipVision_DB->setChecked(true);
#if FUNCTION_MASK & ALLOW_OUTPUT_DB
	output_format = OUTPUT_DB;
#else
	output_format = OUTPUT_PIC;
	ui->actionChipVision_DB->setVisible(false);
	ui->actionSeperate_Image->setVisible(false);
	ui->menuOutputFormat->menuAction()->setVisible(false);
#endif
#if !(FUNCTION_MASK & ALLOW_FOREVER)
	QDateTime current = QDateTime::currentDateTime();
	QDateTime filetime = QDateTime::fromString("2020_05_30", "yyyy_MM_dd");
	if (current > filetime) {
		QMessageBox::warning(this, "Time check error", "Time check error");
		exit(1);
	}
#endif
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::mouse_change(QString info)
{
	status_label->setText(info);
}

void MainWindow::notify_progress(float pos)
{
	pProgressBar->setValue(100 * pos);
	pProgressBar->repaint();
}

void MainWindow::title_change(QString title)
{
	setWindowTitle(title);
}

void MainWindow::on_actionConfig_Para_triggered()
{
	ConfigPara cpara;
	int ret = stitch_view->get_config_para(-1, cpara);
	if (ret < 0) {
		cpara.clip_l = 0;
		cpara.clip_r = 0;
		cpara.clip_u = 0;
		cpara.clip_d = 0;
		cpara.rescale = 8;
		cpara.max_lr_xshift = 100;
		cpara.max_lr_yshift = 60;
		cpara.max_ud_xshift = 100;
		cpara.max_ud_yshift = 100;
		cpara.img_path = "c:/chenyu/data/M4_1/M3_";
		cpara.img_num_w = 26;
		cpara.img_num_h = 9;
		cpara.offset.create(2, 2);
		cpara.offset(0, 0) = Vec2i(0, 0);
		cpara.offset(0, 1) = Vec2i(0, 1350);
		cpara.offset(1, 0) = Vec2i(850, 0);
		cpara.offset(1, 1) = Vec2i(850, 1350);
	}
	cpara.rescale = 4;

	CparaDialog para_dlg(cpara, true, use_default_compare, this);
	if (para_dlg.exec() == QDialog::Accepted) {
		if (cpara.rescale != 1 && cpara.rescale != 2 && cpara.rescale != 4 && cpara.rescale != 8) {
			QMessageBox::information(this, "Info", "rescale must be 1, 2, 4 or 8");
			return;
		}
		cpara = para_dlg.cpara;
		if (para_dlg.new_layer) {
			stitch_view->set_config_para(stitch_view->get_layer_num(), cpara);
			stitch_view->set_current_layer(stitch_view->get_layer_num() - 1);
			string filename = cpara.img_path;
			int loc = (int) filename.find_last_of("\\/");
			int loc2 = (int) filename.find_last_of("\\/", loc - 1);
			string layer_name = filename.substr(loc2 + 1, loc - loc2 - 1);
			stitch_view->set_layer_name(-1, layer_name);
		}
		else
			stitch_view->set_config_para(-1, cpara);
	}
}

void MainWindow::on_actionTune_Para_triggered()
{
    TuningPara tpara;
    int ret = stitch_view->get_tune_para(-1, tpara);

	TParaDialog para_dlg("tune.xml", this);
	if (para_dlg.exec() == QDialog::Accepted) {
		use_default_compare = para_dlg.is_default;
		stitch_view->set_tune_para(-1, para_dlg.tpara);
	}
}

void MainWindow::on_actionPrepare_Next_Iter_triggered()
{
	ConfigPara cpara;
	int ret = stitch_view->get_config_para(-1, cpara);
	if (ret < 0) {
		cpara.clip_l = 0;
		cpara.clip_r = 0;
		cpara.clip_u = 0;
		cpara.clip_d = 0;
		cpara.rescale = 8;
		cpara.max_lr_xshift = 36;
		cpara.max_lr_yshift = 16;
		cpara.max_ud_xshift = 16;
		cpara.max_ud_yshift = 36;
		cpara.img_path = "C:/chenyu/data/A01/M1/M1_";
		cpara.img_num_w = 3;
		cpara.img_num_h = 3;
		cpara.offset.create(2, 2);
		cpara.offset(0, 0) = Vec2i(0, 0);
		cpara.offset(0, 1) = Vec2i(0, 1817);
		cpara.offset(1, 0) = Vec2i(1558, 0);
		cpara.offset(0, 1) = Vec2i(1558, 1817);
	}

	CparaDialog para_dlg(cpara, false, use_default_compare, this);
	if (para_dlg.exec() == QDialog::Accepted) {
		if (para_dlg.cpara.rescale != 1 && para_dlg.cpara.rescale != 2 && para_dlg.cpara.rescale != 4 && para_dlg.cpara.rescale != 8) {
			QMessageBox::information(this, "Info", "rescale must be 1, 2, 4 or 8");
			return;
		}
		if (para_dlg.cpara.rescale > cpara.rescale) {
			QMessageBox::information(this, "Info", "Reduce rescale");
			return;			
		}
		qInfo("UI: prepare next, scale=%d, lrx=%d, lry=%d, udx=%d, udy=%d", cpara.rescale,
			cpara.max_lr_xshift, cpara.max_lr_yshift, cpara.max_ud_xshift, cpara.max_ud_yshift);
		cpara = para_dlg.cpara;
		stitch_view->set_config_para(-1, cpara);
		stitch_view->compute_new_feature(-1);
	}	
}

void MainWindow::on_actionQuick_Save_triggered()
{
	string filename = stitch_view->get_project_path() + "/WorkData/quicksave.xml";
	qInfo("UI: quick save %s", filename.c_str());
	stitch_view->write_file(filename);
}

void MainWindow::on_actionQuick_Load_triggered()
{
	string filename = stitch_view->get_project_path() + "/WorkData/quicksave.xml";
	qInfo("UI: quick load %s", filename.c_str());
	stitch_view->read_file(filename);
}

void MainWindow::on_actionOptimize_Offset_triggered()
{
	qInfo("UI: optimize offset");
    stitch_view->optimize_offset(-1, speed);
}

void MainWindow::on_actionTransForm_triggered()
{
	QScopedPointer<MapXY> mxy(stitch_view->get_mapxy(-1));
	if (mxy.isNull())
		return;
	int dst_w = stitch_view->get_dst_wide();
	QRect output_rect = stitch_view->get_output_rect();
	MapxyDialog mxy_dlg(mxy->get_beta(), mxy->get_default_zoomx(), mxy->get_default_zoomy(), 
		mxy->get_merge_method(), dst_w, mxy->get_max_pt_error(), mxy->get_merge_pt_distance(), output_rect, this);
	if (mxy_dlg.exec() == QDialog::Accepted) {
		qInfo("UI: deg=%f, zx=%f, zy=%f,dst_w=%d, pterr=%d, md=%d", mxy_dlg.beta, 
			mxy_dlg.zx, mxy_dlg.zy, mxy_dlg.dst_w, mxy_dlg.max_pt_err, mxy_dlg.merge_distance);
		mxy->set_beta(mxy_dlg.beta);
		mxy->set_default_zoomx(mxy_dlg.zx);
		mxy->set_default_zoomy(mxy_dlg.zy);
		mxy->set_merge_method(mxy_dlg.merge);
		mxy->set_max_pt_error(mxy_dlg.max_pt_err);
		mxy->set_merge_pt_distance(mxy_dlg.merge_distance);
		dst_w = mxy_dlg.dst_w;
		stitch_view->set_mapxy_dstw(-1, mxy.data(), dst_w);
		stitch_view->set_output_rect(mxy_dlg.output_rect);
	}
}

void MainWindow::on_actionLoad_triggered()
{
	QString dirname = QString::fromStdString(stitch_view->get_project_path() + "/WorkData/");
	QString filename = QFileDialog::getOpenFileName(this, tr("Open File"),
		dirname,
		tr("Project (*.xml)"));
	if (!filename.isEmpty()) {
		qInfo("UI: load %s", filename.toStdString().c_str());
        stitch_view->read_file(filename.toStdString(), false);
	}
}

void MainWindow::on_actionSave_as_triggered()
{
	QString dirname = QString::fromStdString(stitch_view->get_project_path() + "/WorkData/autosave.xml");
	QString filename = QFileDialog::getSaveFileName(this, tr("Open File"),
		dirname,
		tr("Project (*.xml)"));
	if (!filename.isEmpty()) {
		qInfo("UI: save as %s", filename.toStdString().c_str());
		stitch_view->write_file(filename.toStdString());
	}
}

void MainWindow::on_actionOutput_layer_triggered()
{
	QString dirname = QString::fromStdString(stitch_view->get_project_path());
	QString pathname = QFileDialog::getExistingDirectory(this, tr("choose path"), dirname);
	if (!pathname.isEmpty()) {
		qInfo("UI: Output layer%d, %s", stitch_view->get_current_layer(), pathname.toStdString().c_str());
		stitch_view->output_layer(-1, pathname.toStdString(), output_format);
	}
}

void MainWindow::on_actionOutput_all_triggered()
{
	QString dirname = QString::fromStdString(stitch_view->get_project_path());
    QString pathname = QFileDialog::getExistingDirectory(this, tr("choose path"), dirname);
    if (!pathname.isEmpty()) {
        int layer_num = stitch_view->get_layer_num();
		for (int l = 0; l < layer_num; l++) {
			qInfo("UI: Output layer%d, %s", l, pathname.toStdString().c_str());
			stitch_view->output_layer(l, pathname.toStdString(), output_format);
		}
    }
}

void MainWindow::on_actionDrawGrid_triggered()
{
    double gh, gw, oy, ox;
    stitch_view->get_grid(ox, oy, gw, gh);
    GridDialog grid_dlg(this, gh, gw, oy, ox);
    if (grid_dlg.exec() == QDialog::Accepted) {
		qInfo("UI: draw grid ox=%d, oy=%d, gw=%d, gh=%d", grid_dlg.offset_x, grid_dlg.offset_y, grid_dlg.grid_width, grid_dlg.grid_high);
		stitch_view->set_grid(grid_dlg.offset_x, grid_dlg.offset_y, grid_dlg.grid_width, grid_dlg.grid_high);
    }

}

void MainWindow::on_actionAddNail_triggered()
{
    stitch_view->to_state_add_nail();
}

void MainWindow::on_actionDelete_layer_triggered()
{
	QMessageBox::StandardButton ret = QMessageBox::warning(NULL, "Delete Layer", "Are you sure delete this layer",
		QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
	if (ret == QMessageBox::Ok)
		stitch_view->delete_layer(-1);
}

void MainWindow::on_actionSelectNail_triggered()
{
    stitch_view->to_state_change_nail();
}

void MainWindow::on_actionLayer_Up_triggered()
{
	stitch_view->layer_up(-1);
}

void MainWindow::on_actionLayer_Down_triggered()
{
	stitch_view->layer_down(-1);
}

void MainWindow::on_actionOptimize_Offset_NB_triggered()
{
	qInfo("UI: optimize offset neglect border");
	stitch_view->optimize_offset(-1, speed | BUNDLE_ADJUST_WEAK_ORDER);
}

void MainWindow::on_actionFast_But_Bad_triggered()
{
	speed = BUNDLE_ADJUST_SPEED_FAST;
	ui->actionFast_But_Bad->setChecked(true);
	ui->actionMiddle_And_Normal->setChecked(false);
	ui->actionSlow_And_Good->setChecked(false);
}

void MainWindow::on_actionMiddle_And_Normal_triggered()
{
	speed = BUNDLE_ADJUST_SPEED_NORMAL;
	ui->actionFast_But_Bad->setChecked(false);
	ui->actionMiddle_And_Normal->setChecked(true);
	ui->actionSlow_And_Good->setChecked(false);
}

void MainWindow::on_actionSlow_And_Good_triggered()
{
	speed = BUNDLE_ADJUST_SPEED_SLOW;
	ui->actionFast_But_Bad->setChecked(false);
	ui->actionMiddle_And_Normal->setChecked(false);
	ui->actionSlow_And_Good->setChecked(true);
}

void MainWindow::on_actionClear_All_Fix_Edge_triggered()
{
    stitch_view->clear_fix_edge(-1);
}

void MainWindow::on_actionClear_Red_Fix_edge_triggered()
{
    stitch_view->clear_red_fix_edge(-1);
}

void MainWindow::on_actionClear_Yellow_Fix_Edge_triggered()
{
    stitch_view->clear_yellow_fix_edge(-1);
}

void MainWindow::on_actionGoto_triggered()
{
    XYDialog dlg(this);
    if (dlg.exec() == QDialog::Accepted) {
        stitch_view->goto_xy(dlg.x, dlg.y);
    }
}

void MainWindow::on_actionImport_triggered()
{
    QString dirname = QString::fromStdString(stitch_view->get_project_path() + "/WorkData/");
    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"),
        dirname,
        tr("Project (*.xml)"));
    if (!filename.isEmpty()) {
        qInfo("UI: load %s", filename.toStdString().c_str());
        stitch_view->read_file(filename.toStdString(), true);
    }
}

void MainWindow::on_actionAlign_IdeaPos_triggered()
{
    AlignIdeaPosDialog dlg(this, 2, 2, 0xff);
    if (dlg.exec() == QDialog::Accepted) {
        qInfo("Align Ideapos w0=%d, w1=%d, option=0x%x", dlg.w0, dlg.w1, dlg.option);
        double ret = stitch_view->refilter_edge(-1, dlg.w0, dlg.w1, dlg.option);
		if (ret >= 0) {
			char message[50];
			sprintf(message, "Align success, err=%6.3f", ret);
			QMessageBox::information(this, "Info", message);
		}
        else
            QMessageBox::information(this, "Info", "Align fail");
    }
}

void MainWindow::closeEvent(QCloseEvent * e)
{
	if (stitch_view->modified())
	if (QMessageBox::question(this, "Quit", "Do you want to quicksave?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes) {
		on_actionQuick_Save_triggered();
	}
}

void MainWindow::on_actionSeperate_Image_triggered()
{
	output_format = OUTPUT_PIC;
	ui->actionChipVision_DB->setChecked(false);
	ui->actionSeperate_Image->setChecked(true);
}

void MainWindow::on_actionChipVision_DB_triggered()
{
	output_format = OUTPUT_DB;
	ui->actionChipVision_DB->setChecked(true);
	ui->actionSeperate_Image->setChecked(false);
}
