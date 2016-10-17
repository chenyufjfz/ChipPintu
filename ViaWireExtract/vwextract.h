#ifndef VWEXTRACT_H
#define VWEXTRACT_H
#include "opencv2/ml/ml.hpp"
#include "objextract.h"
using namespace cv;

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

#define RULE_TREE				1
#define RULE_NO_4CONN			2
#define RULE_NO_3CONN_PAIR		4
#define RULE_MINIMUM_LEN		8

struct LearnContainer {
	QRect learn_rect;
	vector<QRect> wires;
	vector<QPoint> vias;
	LearnContainer(QRect &rect) {
		learn_rect = rect;
	}
};


class VWExtract : public ObjExtract
{
protected:
	int insu_wd; //insulator width
    int wire_wd; //wire width
    int via_rd; //via radius
	int grid_wd; //grid width
	int feature_method;
	int learn_method;
    Mat img;
	Mat mark, mark1, mark2, mark3;
    int iter_num;
    float param1, param2, param3;
public:
	VWExtract();
    virtual void set_param(int width, int r, int _iter_num, float _param1, float _param2, float _param3, int insu_width, int grid_width) {
        wire_wd = width;
        via_rd = r;
        iter_num = _iter_num;
        param1 = _param1;
        param2 = _param2;
        param3 = _param3;
		insu_wd = insu_width;
		grid_wd = grid_width;
    }

	Mat get_mark() {
		return mark;
	}
	Mat get_mark1() {
		return mark1;
	}
	Mat get_mark2() {
		return mark2;
	}
	Mat get_mark3() {
		return mark3;
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
	float via_feature_th, edge_feature_th1[2], edge_feature_th2[2], insu_feature_th[2];
	bool use_ratio[2];
public:
	void train(string file_name, const std::vector<MarkObj> & obj_sets,
		int _feature_method, int _learn_method);
	void extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	void get_feature(int x, int y, vector<float> & feature);
};
#endif // VWEXTRACT_H
