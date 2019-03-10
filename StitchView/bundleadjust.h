#ifndef BUNDLEADJUST_H
#define BUNDLEADJUST_H
#include <vector>
#include <list>
#include <map>
#include "featext.h"
#include "bundleadjustinf.h"
using namespace std;


struct ImgMeta {
	unsigned img_idx;				//it is fixed after init
	unsigned bundle_idx;			//bundle_idx is bundle's top-left img_idx
	Point offset;					//This image top-left offset to bundle's top-left (in pixel)
	unsigned state;					//NOT VISIT or VISIT
	/*up is 0, right is 1, down is 2, left is 3, return edge index which can be used by get_edge*/
	unsigned get_edge_idx(int dir) {
		switch (dir) {
		case DIR_UP:
			if (img_idx >= 0x10000)
				return img_idx - 0x10000;
			break;
			
		case DIR_RIGHT:
			return img_idx | 0x80000000;

		case DIR_DOWN:
			return img_idx;	
			
		case DIR_LEFT:
			if (img_idx & 0x7fff)
				return (img_idx - 1) | 0x80000000;
			break;
		}
		return 0xffffffff;
	}
	/*up is 0, right is 1, down is 2, left is 3, return image index which can be used by get_img_meta*/
	unsigned get_img_idx(int dir) {
		switch (dir) {
		case DIR_UP:
			if (img_idx >= 0x10000)
				return img_idx - 0x10000;
			break;

		case DIR_RIGHT:
			return img_idx + 1;

		case DIR_DOWN:
			return img_idx + 0x10000;

		case DIR_LEFT:
			if (img_idx & 0x7fff)
				return img_idx - 1;
			break;
		}
		return 0xffffffff;
	}
};

struct Edge {
	const EdgeDiff * diff;
	unsigned base_score;
	short state; //MERGED, SHARED, FREE
    unsigned cost_or_dir;
	Edge() {
		base_score = 0;
	}
};

/*
Algorithm
1 init edge
*/
class BundleAdjust : public BundleAdjustInf
{
protected:
	vector<Edge> eds[2]; //Originally eds.diff point to FeatExt, during merge, it point to new_eds
	vector<ImgMeta> imgs; //row * col raw image
	list<EdgeDiff *> new_eds; //new EdgeDiff during merge, it need to free when finish
	list<Edge *> edge_mqueue; //merge edge priority queue, Edge * point to eds item.
	int img_num_h, img_num_w, scale;

protected:
	Edge * get_edge(int i, int y, int x);
	Edge * get_edge(int idx);
	ImgMeta * get_img_meta(int idx);
	void push_mqueue(Edge * e);
	void release_new_eds();
	void init(const FeatExt & fet, int _img_num_h, int _img_num_w);
	void reinit(const FeatExt & fet, const Mat_<Vec2i> & offset, bool replace_offset);
	int merge(const EdgeDiff * edge, const FeatExt & fet);
	unsigned long long split(int m, int n, const FeatExt & fet);
	Mat_<Vec2i> best_offset;
	Edge * pick_mq_edge();

public:
    BundleAdjust();
	~BundleAdjust();
	Mat_<Vec2i> get_best_offset() {
		return best_offset.clone();
	}
	Mat_<unsigned long long> get_corner() {
		Mat_<unsigned long long> corner(best_offset.size());
		return corner;
	}
	int arrange(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe, Size, bool);
};

#endif // BUNDLEADJUST_H
