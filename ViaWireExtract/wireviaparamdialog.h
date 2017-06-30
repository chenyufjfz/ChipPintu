#ifndef WIREVIAPARAMDIALOG_H
#define WIREVIAPARAMDIALOG_H

#include <QDialog>
#include <QListWidgetItem>
#include "extractparam.h"

namespace Ui {
class WireViaParamDialog;
}

class WireViaParamDialog : public QDialog
{
    Q_OBJECT
protected:
	ExtractParam * ep;

public:
	explicit WireViaParamDialog(QWidget *parent, ExtractParam * _ep);
    ~WireViaParamDialog();
	string action_name;

private slots:
    void on_buttonBox_accepted();

    void on_actions_itemClicked(QListWidgetItem *item);

    void on_param_items_itemClicked(QListWidgetItem *item);

private:
    Ui::WireViaParamDialog *ui;
};

#endif // WIREVIAPARAMDIALOG_H
