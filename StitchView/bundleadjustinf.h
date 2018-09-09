#ifndef BUNDLEADJUSTINF_H
#define BUNDLEADJUSTINF_H
#include "featext.h"

#define BIND_X_MASK		2
#define BIND_Y_MASK		1

struct FixEdge {
	int idx;
	Point shift;
	int bind_flag;
};
class BundleAdjustInf
{
public:
    static BundleAdjustInf * create_instance(int method);
    virtual Mat_<Vec2i> get_best_offset() = 0;
	/*31..24 23..16 15..8 7..0
					edge  corner
	*/
	virtual Mat_<int> get_corner() = 0;
    /*input fet, fet.diff
    input _img_num_h, if < 0, img_num_h = fet.cpara.img_num_h
    input _img_num_w, if < 0, img_num_w = fet.cpara.img_num_w*/
    virtual int arrange(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe) = 0;
    virtual ~BundleAdjustInf() {}
};

#endif // BUNDLEADJUSTINF_H
