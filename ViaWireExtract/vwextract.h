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


#define RULE_NO_LOOP			1
#define RULE_NO_UCONN			2
#define RULE_NO_hCONN			4
#define RULE_NO_HCONN			8
#define RULE_NO_FCONN			0x10
#define RULE_NO_fCONN			0x20
#define RULE_NO_TT_CONN			0x40
#define RULE_NO_XT_CONN			0x80
#define RULE_NO_XX_CONN			0x100
#define RULE_NO_X_POINT			0x200
#define RULE_NO_T_POINT			0x400
#define RULE_MINIUM_3_POINT		0x800
#define RULE_END_WITH_VIA		0x80000000

struct LearnContainer {
	QRect learn_rect;
	vector<QRect> wires;
	vector<QPoint> vias;
	LearnContainer(QRect &rect) {
		learn_rect = rect;
	}
};

struct LayerParam{
	int wire_wd; //wire width
	int via_rd; //via radius
	int grid_wd; //grid width	
	float param1; //via th, close to 1, higher threshold
	float param2; //wire th, close to 1, higher threshold
	float param3; //via_cred vs wire_cred, if via_cred> wire_cred, beta>1; else <1
	unsigned long long rule; //rule affect bbfm
};

struct TileLayerData {	
	Mat mark, mark1;
	Mat img;
	Mat conet;
	vector<int> gl_x, gl_y;
};

struct TileData {
	vector<TileLayerData> d;
};
class VWExtract : public ObjExtract
{
protected:
	vector<LayerParam> lpm;
	vector<TileData> ts;

public:
	VWExtract();
	virtual int set_train_param(int layer, int width, int r, int rule_low, int grid_width, float _param1, float _param2, float _param3) {
		if (layer > lpm.size())
			return -1;
		if (layer == lpm.size())
			lpm.push_back(LayerParam());
		lpm[layer].wire_wd = width;
		lpm[layer].via_rd = r;
		lpm[layer].rule = rule_low;
		lpm[layer].grid_wd = grid_width;
		lpm[layer].param1 = _param1;
		lpm[layer].param2 = _param2;
		lpm[layer].param3 = _param3;
		
		return 0;
    }	
	virtual int set_extract_param(int layer, int width, int r, int rule_low, int grid_width, float _param1, float _param2, float _param3, float) {
		if (layer > lpm.size())
			return -1;
		if (layer == lpm.size())
			lpm.push_back(LayerParam());
		lpm[layer].wire_wd = width;
		lpm[layer].via_rd = r;
		lpm[layer].rule = rule_low;
		lpm[layer].grid_wd = grid_width;
		lpm[layer].param1 = _param1;
		lpm[layer].param2 = _param2;
		lpm[layer].param3 = _param3;
		return 0;
	}

	Mat get_mark(int layer) {
		if (ts.empty())
			return Mat();
		CV_Assert(layer < lpm.size());
		return ts[0].d[layer].mark;
	}
	Mat get_mark1(int layer) {
		if (ts.empty())
			return Mat();
		CV_Assert(layer < lpm.size());
		return ts[0].d[layer].mark1;
	}
	Mat get_mark2(int layer) {
		return Mat();
	}
	Mat get_mark3(int layer) {
		CV_Assert(layer < lpm.size());
		return Mat();
	}
	virtual ~VWExtract() {

	}
};

class VWExtractStat : public VWExtract {
public:
	int train(string, const std::vector<MarkObj> &) { return 0; }
	int extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	int train(ICLayerWr *, const std::vector<MarkObj> & ) { return 0; }
	int extract(ICLayerWr * ic_layer, const std::vector<SearchArea> & area_, std::vector<MarkObj> & obj_sets) {
		return 0;
	}
	void get_feature(int, int, int, vector<float> & ) {}
};
#endif // VWEXTRACT_H
