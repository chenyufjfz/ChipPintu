#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "cparadialog.h"
#include "tparadialog.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    stitch_view = new StitchView(this);
	setCentralWidget(stitch_view);
	stitch_view->setFocus();

	status_label = new QLabel();
    status_label->setMaximumSize(180, 20);
	ui->statusBar->addWidget(status_label);

	pProgressBar = new QProgressBar();
	pProgressBar->setRange(0, 100);
	pProgressBar->setValue(0);
	pProgressBar->setMaximumSize(300, 20);
	ui->statusBar->addPermanentWidget(pProgressBar);

	connect(stitch_view, SIGNAL(MouseChange(QPoint)), this, SLOT(mouse_change(QPoint)));
	connect(stitch_view, SIGNAL(notify_progress(float)), this, SLOT(notify_progress(float)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::mouse_change(QPoint pos)
{
	char s[100];
	sprintf(s, "x:%d,y:%d", pos.x(), pos.y());
	status_label->setText(s);
}

void MainWindow::notify_progress(float pos)
{
	pProgressBar->setValue(100 * pos);
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
		cpara.max_lr_xshift = 80;
		cpara.max_lr_yshift = 16;
		cpara.max_ud_xshift = 16;
		cpara.max_ud_yshift = 80;
		cpara.img_path = "C:/chenyu/data/A01/M1/M1_";
		cpara.img_num_w = 3;
		cpara.img_num_h = 3;
		cpara.offset.create(2, 2);
		cpara.offset(0, 0) = Vec2i(0, 0);
		cpara.offset(0, 1) = Vec2i(0, 1817);
		cpara.offset(1, 0) = Vec2i(1558, 0);
		cpara.offset(0, 1) = Vec2i(1558, 1817);
	}
	bool new_layer = (stitch_view->get_layer_num() == 0);
	CparaDialog para_dlg(cpara, new_layer, this);
	if (para_dlg.exec() == QDialog::Accepted) {
		cpara = para_dlg.cpara;
		int init_offset_x = cpara.offset(0, 1)[1] - cpara.offset(0, 0)[1];
		int init_offset_y = cpara.offset(1, 0)[0] - cpara.offset(0, 0)[0];
		cpara.offset.create(cpara.img_num_h, cpara.img_num_w);
		for (int y = 0; y < cpara.img_num_h; y++) {
			for (int x = 0; x < cpara.img_num_w; x++) {
				cpara.offset(y, x)[1] = init_offset_x * x;
				cpara.offset(y, x)[0] = init_offset_y * y;
			}
		}
		if (para_dlg.new_layer) {
			stitch_view->set_config_para(stitch_view->get_layer_num(), cpara);
			stitch_view->set_current_layer(stitch_view->get_layer_num() - 1);
		}
		else
			stitch_view->set_config_para(-1, cpara);
	}
}

void MainWindow::on_actionTune_Para_triggered()
{
    TuningPara tpara;
    int ret = stitch_view->get_tune_para(-1, tpara);

	TParaDialog para_dlg(tpara, this);
	if (para_dlg.exec() == QDialog::Accepted) {
		stitch_view->set_tune_para(-1, para_dlg.tpara);
	}
}

void MainWindow::on_actionPrepare_Next_Iter_triggered()
{
	stitch_view->compute_new_feature(-1);
}

void MainWindow::on_actionQuick_Save_triggered()
{
	string filename = qApp->applicationDirPath().toStdString() + "/WorkData/quicksave.xml";
	stitch_view->write_file(filename);
}

void MainWindow::on_actionQuick_Load_triggered()
{
	string filename = qApp->applicationDirPath().toStdString() + "/WorkData/quicksave.xml";
	stitch_view->read_file(filename);
}

void MainWindow::on_actionOptimize_Offset_triggered()
{
    stitch_view->optimize_offset(-1);
}
