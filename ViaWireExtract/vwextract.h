#ifndef VWEXTRACT_H
#define VWEXTRACT_H
#include "opencv2/ml/ml.hpp"
#include "objextract.h"

using namespace std;
using namespace cv;

enum {
	M_UNKNOW = 0,
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

class VWExtract : public ObjExtract
{
public:
	VWExtract() {}
	static VWExtract * create_extract(int method);

	virtual ~VWExtract() {

	}
};

#endif // VWEXTRACT_H
