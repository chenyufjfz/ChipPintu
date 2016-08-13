#ifndef VWEXTRACT_H
#define VWEXTRACT_H
#include <vector>
#include <QRect>
#include <QLine>
#include <QPoint>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
using namespace cv;

struct MarkObj {
    int type;
	int type2;
    int select_state;
    QPoint p0, p1;
};


enum {
    OBJ_NONE=0,
    OBJ_AREA,
    OBJ_WIRE,
    OBJ_VIA,
    SELECT_OBJ
};

enum {
	AREA_LEARN=0,
	AREA_METAL,
};

enum {
	M_UNKNOW=0,	
	M_W,
	M_V,
	M_W_I,	
	M_V_I,
	M_I,
	M_W_V,
	M_V_I_W,
	M_V_I_V,
	M_INL, //must be first unlearn
	M_WNL,
	M_VNL,
	M_W_INL,
	M_V_INL
};

enum {
    FEA_GRADXY_HIST_6x6=0,
	FEA_GRADXY_HIST_7x7,
	FEA_GRADXY_HIST_9x9
};

enum {
	LEARN_SVM,
};
struct LearnContainer {
	QRect learn_rect;
	vector<QRect> wires;
	vector<QPoint> vias;
	LearnContainer(QRect &rect) {
		learn_rect = rect;
	}
};

class VWExtract
{
protected:
    int wire_wd;
    int via_rd;
	int feature_method;
    Mat img;
	Mat mark;
	vector<LearnContainer> l_areas;
	CvSVM svm;
    int iter_num;
    float param1, param2, param3;
public:
	VWExtract();
    void set_param(int width, int r, int _iter_num, float _param1, float _param2, float _param3) {
        wire_wd = width;
        via_rd = r;
        iter_num = _iter_num;
        param1 = _param1;
        param2 = _param2;
        param3 = _param3;
    }

	void train(string file_name, const std::vector<MarkObj> & obj_sets, 
		int _feature_method = FEA_GRADXY_HIST_7x7, int learn_method = LEARN_SVM);
    void extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	Mat get_mark() {
		return mark;
	}
    
protected:
	int fill_mark(const std::vector<MarkObj> & obj_sets);
};

#endif // VWEXTRACT_H
