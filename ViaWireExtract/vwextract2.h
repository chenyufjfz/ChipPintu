#ifndef VWEXTRACT2_H
#define VWEXTRACT2_H
#include "vwextract.h"

struct ProcessParameter {
	int layer;
	int method;
	int method_opt;
	int opt0, opt1, opt2, opt3, opt4, opt5, opt6;
	float opt_f0;
};

class VWExtractPipe : public VWExtract
{
protected:
	vector<ProcessParameter> vwp;
	void * private_data;

public:
	VWExtractPipe();
	~VWExtractPipe();
	int set_train_param(int layer, int type, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, float pf1);
	int set_extract_param(int layer, int type, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, float pf1);
	Mat get_mark(int layer);
	Mat get_mark1(int layer);
	Mat get_mark2(int layer);
	Mat get_mark3(int layer);
	int train(string, const std::vector<MarkObj> &) { return 0; }
	int extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	int train(vector<ICLayerWrInterface *> &, const std::vector<MarkObj> &) { return 0; }
	int extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets);
	void get_feature(int, int, int, std::vector<float> &) { return; }
};
#endif // VWEXTRACT2_H

