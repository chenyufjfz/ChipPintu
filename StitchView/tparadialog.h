#ifndef TPARADIALOG_H
#define TPARADIALOG_H

#include <QDialog>
#include <QListWidgetItem>
#include "featext.h"

namespace Ui {
class TParaDialog;
}

class TParaDialog : public QDialog
{
    Q_OBJECT

public:
    TuningPara tpara;
	ExtractParam ep;
	bool is_default;
    explicit TParaDialog(string filename, QWidget *parent = 0);
    ~TParaDialog();

private slots:
    void on_buttonBox_accepted();

    void on_action_list_itemClicked(QListWidgetItem *item);

private:
    Ui::TParaDialog *ui;
};

#endif // TPARADIALOG_H
