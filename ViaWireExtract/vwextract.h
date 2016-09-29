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
	M_EDGE,
	M_NOEDGE,
	M_INL, //must be first unlearn
	M_WNL,
	M_VNL,
	M_W_INL,
	M_V_INL
};

enum {
    FEA_GRADXY_HIST_6x6=0,
	FEA_GRADXY_HIST_7x7,
	FEA_GRADXY_HIST_9x9,
	FEA_GRADXY_HIST_9x5,
	FEA_GRADXY_9x5,
};

enum {
	LEARN_SVM,
	LEARN_BAYES
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
	int insu_wd;
    int wire_wd;
    int via_rd;
	int feature_method;
	int learn_method;
    Mat img;
	Mat mark, mark1, mark2;
    int iter_num;
    float param1, param2, param3;
public:
	VWExtract();
    virtual void set_param(int width, int r, int _iter_num, float _param1, float _param2, float _param3, int insu_width) {
        wire_wd = width;
        via_rd = r;
        iter_num = _iter_num;
        param1 = _param1;
        param2 = _param2;
        param3 = _param3;
		insu_wd = insu_width;
    }

	virtual void train(string file_name, const std::vector<MarkObj> & obj_sets, 
		int _feature_method, int _learn_method) = 0;
    virtual void extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets) = 0;
	virtual void get_feature(int x, int y, vector<float> & feature) = 0;
	Mat get_mark() {
		return mark;
	}
	Mat get_mark1() {
		return mark1;
	}
	Mat get_mark2() {
		return mark2;
	}
	virtual ~VWExtract() {

	}
};

class VWExtractClasic : public VWExtract {
protected:
	vector<LearnContainer> l_areas;
	CvSVM svm;
	float via_th;
	CvNormalBayesClassifier bayes;
	int fill_mark(const std::vector<MarkObj> & obj_sets);
public:
	void train(string file_name, const std::vector<MarkObj> & obj_sets,
		int _feature_method, int _learn_method);
	void extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	void get_feature(int x, int y, vector<float> & feature);
};

class VWExtractStat : public VWExtract {
protected:
	vector<LearnContainer> l_areas;
	float via_feature_th, edge_feature_th1, edge_feature_th2, insu_feature_th;
	bool use_ratio;
public:
	void train(string file_name, const std::vector<MarkObj> & obj_sets,
		int _feature_method, int _learn_method);
	void extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	void get_feature(int x, int y, vector<float> & feature);
};
#endif // VWEXTRACT_H
