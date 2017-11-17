#ifndef VWEXTRACT3_H
#define VWEXTRACT3_H
#include "vwextract.h"

struct DetectWirePara {
	int w_min, w_max;
	int gray_i, gray_w;
	int dir_mask;
};


class VWExtractAnt : public VWExtract
{
protected:
	DetectWirePara dw0;
	Point abs_org0;
	int layer0;
	int search_opt;

public:
	VWExtractAnt();
	~VWExtractAnt() {}
	int set_train_param(int , int , int , int , int , int , int , int , int , float ) { return 0; }
	int set_extract_param(int layer, int type, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, float pf1);
	Mat get_mark(int layer) { return Mat(); }
	Mat get_mark1(int ) { return Mat(); }
	Mat get_mark2(int ) { return Mat(); }
	Mat get_mark3(int ) { return Mat(); }
	int train(string, const std::vector<MarkObj> &) { return 0; }
	int extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	int train(vector<ICLayerWrInterface *> &, const std::vector<MarkObj> &) { return 0; }
	int extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets);
	void get_feature(int, int, int, std::vector<float> &, std::vector<int> &);
};
#endif // VWEXTRACT3_H

