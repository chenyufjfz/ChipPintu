#include "featurewindow.h"
#include "ui_featurewindow.h"

FeatureWindow::FeatureWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FeatureWindow)
{
    ui->setupUi(this);
}

FeatureWindow::~FeatureWindow()
{
    delete ui;
}
