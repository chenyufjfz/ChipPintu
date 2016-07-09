#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    stitch_view = new StitchView(this);
	setCentralWidget(stitch_view);
	stitch_view->setFocus();

	status_label = new QLabel;
	status_label->setMinimumSize(160, 20);
	ui->statusBar->addWidget(status_label);

	connect(stitch_view, SIGNAL(MouseChange(QPoint)), this, SLOT(mouse_change(QPoint)));
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