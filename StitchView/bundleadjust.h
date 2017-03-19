#ifndef BUNDLEADJUST_H
#define BUNDLEADJUST_H
#include <vector>
#include <list>
#include "featext.h"
using namespace std;

struct EdgeDiffCmp {
	bool operator()(const EdgeDiff & e1, const EdgeDiff & e2) {
		return e1.score > e2.score;
	}
};
struct Edge {
	unsigned next_edge_idx[2], prev_edge_idx[2];
	short dx, dy;
	list<EdgeDiff>::iterator pdiff;
};

struct LayerInfo {
	int img_num_w, img_num_h;
	int clip_l, clip_r, clip_u, clip_d;
	int move_scale;
	int right_bound, bottom_bound;
	string img_path;
	Mat_<Vec2i> offset;
};

/*
Algorithm
1 init edge
*/
class BundleAdjust
{
public:
    BundleAdjust();
	static unsigned compute_score(Mat_<unsigned char> &diff);
	static int load_edge_diff0(string filename);
	static int load_config(string filename);
	static unsigned xy2edge(unsigned short y, unsigned short x);
protected:
	vector<Edge> edge; //size= n*(m-1) + m*(n-1), size doesn't decrease during merge
	list<EdgeDiff> edge_diff; //size decrease during merge
	static list<EdgeDiff> edge_diff0;
	static LayerInfo l_info;
};

#endif // BUNDLEADJUST_H
