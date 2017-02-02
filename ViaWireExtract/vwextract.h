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

struct LayerParam{
	int wire_wd; //wire width
	int via_rd; //via radius
	int grid_wd; //grid width	
	float param1; //via th, close to 1, higher threshold
	float param2; //wire th, close to 1, higher threshold
	float param3; //via_cred vs wire_cred, if via_cred> wire_cred, >1; else <1
	float param4; //via density
	unsigned long long rule; //rule affect bbfm
	unsigned long long warning_rule; //rule affect bbfm
};


class VWExtract : public ObjExtract
{
protected:
	vector<LayerParam> lpm;

public:
	VWExtract() {}
	static VWExtract * create_extract(int method);
	virtual int set_train_param(int layer, int width, int r, int rule_low, int warning_rule_low, int grid_width, float _param1, float _param2, float _param3, float _param4) {
        if (layer==0)
            lpm.clear();
		if (layer > (int) lpm.size())
			return -1;
		if (layer == (int) lpm.size())
			lpm.push_back(LayerParam());
		lpm[layer].wire_wd = width;
		lpm[layer].via_rd = r;
		lpm[layer].rule = rule_low;
		lpm[layer].warning_rule = warning_rule_low;
		lpm[layer].grid_wd = grid_width;
		lpm[layer].param1 = _param1;
		lpm[layer].param2 = _param2;
		lpm[layer].param3 = _param3;
		lpm[layer].param4 = _param4;
		return 0;
    }	
	virtual int set_extract_param(int layer, int width, int r, int rule_low, int warning_rule_low, int grid_width, float _param1, float _param2, float _param3, float _param4) {
        if (layer==0)
            lpm.clear();
		if (layer > (int) lpm.size())
			return -1;
		if (layer == (int) lpm.size())
			lpm.push_back(LayerParam());
		lpm[layer].wire_wd = width;
		lpm[layer].via_rd = r;
		lpm[layer].rule = rule_low;
		lpm[layer].warning_rule = warning_rule_low;
		lpm[layer].grid_wd = grid_width;
		lpm[layer].param1 = _param1;
		lpm[layer].param2 = _param2;
		lpm[layer].param3 = _param3;
		lpm[layer].param4 = _param4;
		return 0;
	}

	virtual ~VWExtract() {

	}
};

#endif // VWEXTRACT_H
