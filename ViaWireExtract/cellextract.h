#ifndef CELLEXTRACT_H
#define CELLEXTRACT_H
#include "objextract.h"

using namespace std;
using namespace cv;

class CellFeature {
protected:
	int cmp_vector(const vector<int> & tn0, const vector<int> & tn1, int & max_score);
public:
	int wunit, hunit;  //thumb size in pixel
	int width, height; //cell width and height in pixel
	vector<int> tn[8]; //0,1 for power up. 2,3 for power down. 4,5 for power left. 6, 7 for power right
	int valid_area;
	CellFeature();
	void cal_feature(int * ig, int lsize, int _width, int _height, int _wunit, int _hunit, int dir);
	float cmp(int *ig, int lsize, int & dir, float & m_score);
};

class CellFeatures {
public:
	CellFeature feature[2];
	int cell_type;
};

class CellExtract : public ObjExtract
{
protected:
	Mat img, mark, mark1, mark2, mark3;
	float param1, param2, param3;
	vector<CellFeatures> cell;
public:
	CellExtract() {
		param1 = 0.1f;
		param2 = 2;	
		param3 = 0.5f;
	}
	int set_train_param(int, int, int, int, int, float param1_, float param2_, float param3_) {
		if (param1_ < 0 || param1_ >= 0.5 || param3_ >= 1 || param2_>4) {
			qWarning("CellExtract invalid param %f, %f, %f", param1_, param2_, param3_);
            return -1;
		}			
		param1 = param1_;
		param2 = param2_;
		param3 = param3_;
        return 0;
	}
	int train(string file_name, const vector<MarkObj> & obj_sets);
	int extract(string file_name, QRect rect, vector<MarkObj> & obj_sets);
    int set_extract_param(int, int, int, int, int, float param1_, float param2_, float param3_, float)
    {
        if (param1_ < 0 || param1_ >= 0.5 || param3_ >= 1 || param2_>4) {
            qWarning("CellExtract invalid param %f, %f, %f", param1_, param2_, param3_);
            return -1;
        }
        param1 = param1_;
        param2 = param2_;
        param3 = param3_;
        return 0;
    }
    int train(vector<ICLayerWr *> & ic_layer, const vector<MarkObj> & obj_sets);
    int extract(vector<ICLayerWr *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets);
	void get_feature(int, int , int , vector<float> & ) {}
	Mat get_mark(int) {
		return mark;
	}
	Mat get_mark1(int) {
		return mark1;
	}
	Mat get_mark2(int) {
		return mark2;
	}
	Mat get_mark3(int) {
		return mark3;
	}
};

#endif // CELLEXTRACT_H
