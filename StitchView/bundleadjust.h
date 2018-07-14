#ifndef BUNDLEADJUST_H
#define BUNDLEADJUST_H
#include <vector>
#include <list>
#include <map>
#include "featext.h"
using namespace std;


#define DIR_UP				0
#define DIR_RIGHT			1
#define DIR_DOWN			2
#define DIR_LEFT			3

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
	EdgeDiff * diff;
	short state; //MERGED, SHARED, FREE
	short score;	
};

/*
Algorithm
1 init edge
*/
class BundleAdjust
{
protected:
	vector<Edge> eds[2]; //Originally eds.diff point to FeatExt, during merge, it point to new_eds
	vector<ImgMeta> imgs; //row * col raw image
	list<EdgeDiff *> new_eds; //new diff during merge
	list<Edge *> edge_mqueue; //merge edge priority queue
	int img_num_h, img_num_w, scale;
	float progress;

protected:
	Edge * get_edge(int i, int y, int x);
	Edge * get_edge(int idx);
	ImgMeta * get_img_meta(int idx);
	void push_mqueue(Edge * e);
	void release_new_eds();
	void init(FeatExt & fet, int _img_num_h, int _img_num_w);
	void merge(EdgeDiff * edge, FeatExt & fet);

public:
    BundleAdjust();
	Mat_<Vec2i> arrange(FeatExt & fet, int _img_num_h = 0, int _img_num_w = 0);
	float get_progress() {
		return progress;
	}
};

#endif // BUNDLEADJUST_H
