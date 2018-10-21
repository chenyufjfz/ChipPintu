#ifndef MAPXYDIALOG_H
#define MAPXYDIALOG_H
#include <QDialog>

namespace Ui {
class MapxyDialog;
}

class MapxyDialog : public QDialog
{
    Q_OBJECT
public:
	double beta;
	double zx;
	double zy;
	int merge;
	int dst_w;
	int max_pt_err;
	double max_slope_err;
public:
	explicit MapxyDialog(double _beta, double _zx, double _zy, int _merge, int _dst_w, int _max_pt_err, double _max_slope_err, QWidget *parent = 0);
    ~MapxyDialog();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::MapxyDialog *ui;
};

#endif // MAPXYDIALOG_H
