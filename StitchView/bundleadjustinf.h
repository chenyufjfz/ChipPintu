#ifndef BUNDLEADJUSTINF_H
#define BUNDLEADJUSTINF_H
#include "featext.h"

//BIND_X_MASK & BIND_Y_MASK can't be change, because in stitchview.cpp new_fix = new_fix + 1;
#define BIND_X_MASK		2
#define BIND_Y_MASK		1
#define IDEA_POS_MASK	0x80
#define FIX_EDGE_SCALE(e) ((e) >> 2 & 0x1f)
#define FIX_EDGE_BINDX(e) (((e) & BIND_X_MASK) >> 1)
#define FIX_EDGE_BINDY(e) ((e) & BIND_Y_MASK)
#define FIX_EDGE_IDEA_POS(e) (((e) & IDEA_POS_MASK) >> 7)
#define FIX_EDGE_ISBIND(e) ((e) & 0x7f)
#define FIX_EDGE_IDEA_DX(e) ((e) >> 8 & 0x7ff)
#define FIX_EDGE_IDEA_DY(e) ((e) >> 19 & 0x7ff)
#define MAKE_FIX_EDGE(bxy, s, idea, idx, idy) ((s) << 2 & 0x7c | (bxy) & 3 | (idea) << 7 & 0x80 | ((idx) & 0x7ff) << 8 | ((idy) & 0x7ff) << 19)

#define BUNDLE_ADJUST_WEAK_ORDER		1
#define BUNDLE_ADJUST_SPEED_FAST		0
#define BUNDLE_ADJUST_SPEED_NORMAL		0x2
#define BUNDLE_ADJUST_SPEED_SLOW		0x4
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
	virtual Mat_<unsigned long long> get_corner() = 0;
    /*input fet, fet.diff
    input _img_num_h, if < 0, img_num_h = fet.cpara.img_num_h
    input _img_num_w, if < 0, img_num_w = fet.cpara.img_num_w*/
    virtual int arrange(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe, Size s, int weak_border) = 0;
    virtual ~BundleAdjustInf() {}
};

#endif // BUNDLEADJUSTINF_H
