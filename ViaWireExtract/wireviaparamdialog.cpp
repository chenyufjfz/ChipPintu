#include "wireviaparamdialog.h"
#include "ui_wireviaparamdialog.h"

WireViaParamDialog::WireViaParamDialog(QWidget *parent, ExtractParam * _ep) :
    QDialog(parent),
    ui(new Ui::WireViaParamDialog)
{
	ep = _ep;
    ui->setupUi(this);
	vector<string> actions;
	ep->get_param_set_list(actions);
	for (int i = 0; i < actions.size(); i++)
		ui->actions->addItem(QString::fromStdString(actions[i]));
}

WireViaParamDialog::~WireViaParamDialog()
{
    delete ui;
}

void WireViaParamDialog::on_buttonBox_accepted()
{
	action_name = ui->actions->currentItem()->text().toStdString();
}

void WireViaParamDialog::on_actions_itemClicked(QListWidgetItem *item)
{
	QListWidget * param = ui->param_items;
	int count = param->count();
	for (int i = 0; i < count; i++)
		delete param->takeItem(0);
	vector<string> param_items;
	ep->get_param_sets(item->text().toStdString(), param_items);
	for (int i = 0; i < param_items.size(); i++)
		param->addItem(QString::fromStdString(param_items[i]));
}

void WireViaParamDialog::on_param_items_itemClicked(QListWidgetItem *item)
{
	vector<ParamItem> param_items;
	ep->get_param(item->text().toStdString(), param_items);
	if (param_items.size() == 1) {
		int layer = param_items[0].pi[0];
		int type = param_items[0].pi[1];
		int opt0 = param_items[0].pi[2];
		int opt1 = param_items[0].pi[3];
		int opt2 = param_items[0].pi[4];
		int opt3 = param_items[0].pi[5];
		int opt4 = param_items[0].pi[6];
		int opt5 = param_items[0].pi[7];
		int opt6 = param_items[0].pi[8];
		float opt_f0 = param_items[0].pf;
		ui->layer->setText(QString::number(layer, 16));
		ui->type->setText(QString::number(type, 16));
		ui->opt0->setText(QString::number(opt0, 16));
		ui->opt1->setText(QString::number(opt1, 16));
		ui->opt2->setText(QString::number(opt2, 16));
		ui->opt3->setText(QString::number(opt3, 16));
		ui->opt4->setText(QString::number(opt4, 16));
		ui->opt5->setText(QString::number(opt5, 16));
		ui->opt6->setText(QString::number(opt6, 16));
		ui->opt_f0->setText(QString::number(opt_f0));
	}
}
