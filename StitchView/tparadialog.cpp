#include "tparadialog.h"
#include "ui_tparadialog.h"

TParaDialog::TParaDialog(string filename, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TParaDialog)
{
	vector<string> actions;
    ui->setupUi(this);
	ep.read_file(filename);
	ep.get_param_set_list(actions);
	for (int i = 0; i < actions.size(); i++)
		ui->action_list->addItem(QString::fromStdString(actions[i]));
}

TParaDialog::~TParaDialog()
{
    delete ui;
}

void TParaDialog::on_buttonBox_accepted()
{

}

void TParaDialog::on_action_list_itemClicked(QListWidgetItem *item)
{
	tpara.read(ep, item->text().toStdString());
}
