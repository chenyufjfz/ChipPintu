#ifndef CELLEXTRACT_H
#define CELLEXTRACT_H
#include "objextract.h"

using namespace cv;
using namespace std;

class CellFeature {
protected:
	int cmp_vector(const vector<int> & tn0, const vector<int> & tn1, int & max_score);
public:
	int wunit, hunit, width, height;
	vector<int> tn[4];
	int valid_area;
	CellFeature();
	void cal_feature(int * ig, int lsize, int _width, int _height, int _wunit, int _hunit);
	float cmp(int *ig, int lsize, int & dir, float & m_score);
};

class CellExtract : public ObjExtract
{
protected:
	Mat img, mark, mark1, mark2, mark3;
	float param1, param2;
	CellFeature feature[3];
public:
    CellExtract();
	void set_param(int, int, int, float param1_, float param2_, float, int, int) {
		param1 = param1_;
		param2 = param2_;
	}
	void train(string file_name, const vector<MarkObj> & obj_sets, int _feature_method, int _learn_method);
	void extract(string file_name, QRect rect, vector<MarkObj> & obj_sets);
	void get_feature(int , int , std::vector<float> & ) {}
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
};

#endif // CELLEXTRACT_H
