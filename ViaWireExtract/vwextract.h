#ifndef VWEXTRACT_H
#define VWEXTRACT_H
#include "opencv2/ml/ml.hpp"
#include "objextract.h"

using namespace std;
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
	virtual int set_train_param(int width, int r, int _iter_num, int , int grid_width, float _param1, float _param2, float _param3) {
        wire_wd = width;
        via_rd = r;
        iter_num = _iter_num;
        param1 = _param1;
        param2 = _param2;
        param3 = _param3;
		grid_wd = grid_width;
		return 0;
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

class VWExtractStat : public VWExtract {
public:
	int train(string, const std::vector<MarkObj> &) { return 0; }
	int extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	int set_extract_param(int width, int r, int _iter_num, int , int grid_width, float _param1, float _param2, float _param3, float) {
		wire_wd = width;
		via_rd = r;
		iter_num = _iter_num;
		param1 = _param1;
		param2 = _param2;
		param3 = _param3;
		grid_wd = grid_width;
		return 0; 
	}
	int train(ICLayerWr *, const std::vector<MarkObj> & ) { return 0; }
	int extract(ICLayerWr * ic_layer, const std::vector<SearchArea> & area_, std::vector<MarkObj> & obj_sets) {
		return 0;
	}
	void get_feature(int x, int y, vector<float> & feature);
};
#endif // VWEXTRACT_H
