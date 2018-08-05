#include "bundleadjust.h"
#include <stdio.h>
#include <QtGlobal>

#ifdef QT_DEBUG
#ifdef Q_OS_WIN
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif


//state for img
#define NOT_VISIT	0
#define VISIT		1

//state for edge, FREE means it can move with bundle freely, MERGED means it is bundle internal edge
//SHARED means it is absorbed into other edge
#define FREE	0
#define MERGED	1
#define SHARED	2


struct EdgeDiffCmp {
	bool operator()(const Edge * e1, const Edge * e2) {
		return e1->diff->score > e2->diff->score;
	}
};

/*
Input diff
Output minval, min_loc
*/
void found_min(const Mat &diff, vector<int> & minval, vector<Point> & min_loc)
{
	minval.resize(3);
	min_loc.resize(minval.size());
	for (int i = 0; i < minval.size(); i++)
		minval[i] = 1000;
	for (int y = 0; y < diff.rows; y++) {
		const unsigned char * pdiff = diff.ptr(y);
		for (int x = 0; x < diff.cols; x++) {
			for (int i = 0; i < minval.size(); i++)
			if (pdiff[x] < minval[i]) {
				minval.insert(minval.begin() + i, pdiff[x]);
				min_loc.insert(min_loc.begin() + i, Point(x, y));
				minval.pop_back();
				min_loc.pop_back();
			}
		}
	}
}

BundleAdjust::BundleAdjust()
{

}

BundleAdjust::~BundleAdjust()
{
	release_new_eds();
}
Edge * BundleAdjust::get_edge(int i, int y, int x)
{
	if (i == 0 && (y >= img_num_h - 1 || x >= img_num_w))
		return NULL;
	if (i == 1 && (y >= img_num_h || x >= img_num_w - 1))
		return NULL;
	if (i == 0)
		return &eds[0][y* img_num_w + x];
	else
		return &eds[1][y* (img_num_w - 1) + x];
}

//if idx is invalid, return NULL
Edge * BundleAdjust::get_edge(int idx)
{
	Edge * ed = get_edge(EDGE_E(idx), EDGE_Y(idx), EDGE_X(idx));
	CV_Assert(ed==NULL || ed->diff->edge_idx == idx);
	return ed;
}

//If idx is invalid, return NULL
ImgMeta * BundleAdjust::get_img_meta(int idx)
{
	int y = IMG_Y(idx);
	int x = IMG_X(idx);
	if (y >= img_num_h || x >= img_num_w)
		return NULL;
	ImgMeta * pimg = &imgs[y * img_num_w + x];
	CV_Assert(pimg->img_idx == idx);
	return pimg;
}

void BundleAdjust::push_mqueue(Edge * e)
{
	for (list<Edge *>::iterator iter = edge_mqueue.begin(); iter != edge_mqueue.end(); iter++)
		if (e->diff->score + e->base_score >= (*iter)->diff->score + (*iter)->base_score) {
			edge_mqueue.insert(iter, e);
			return;
		}
	edge_mqueue.push_back(e);
}

void BundleAdjust::release_new_eds()
{
	while (!new_eds.empty()) {
		EdgeDiff * ne = new_eds.front();
		new_eds.pop_front();
		delete ne;
	}
}

/*
input fet,
input _img_num_h, if < 0, img_num_h = fet.cpara.img_num_h
input _img_num_w, if < 0, img_num_w = fet.cpara.img_num_w
*/
void BundleAdjust::init(const FeatExt & fet, int _img_num_h, int _img_num_w)
{
	ConfigPara cpara = fet.get_config_para();
	img_num_h = _img_num_h >= 0 ? _img_num_h : cpara.img_num_h;
	img_num_w = _img_num_w >= 0 ? _img_num_w : cpara.img_num_w;
	scale = cpara.rescale;
	//init imgs as seperate free move state
	imgs.clear();
	imgs.resize(img_num_h * img_num_w);
	for (int y = 0; y < img_num_h; y++)
		for (int x = 0; x < img_num_w; x++) {
		ImgMeta *pimg = &imgs[y * img_num_w + x];
		pimg->img_idx = MAKE_IMG_IDX(x, y);
		pimg->bundle_idx = pimg->img_idx;
		pimg->offset = Point(0, 0);
		pimg->state = NOT_VISIT;
		}
	release_new_eds();
	edge_mqueue.clear();
	eds[0].clear();
	eds[0].resize((img_num_h - 1) * img_num_w);
	for (int y = 0; y < img_num_h - 1; y++)
		for (int x = 0; x < img_num_w; x++) {
		eds[0][y * img_num_w + x].diff = fet.get_edge(0, y, x);
		eds[0][y * img_num_w + x].state = FREE;
		eds[0][y * img_num_w + x].diff_or_dir = 0;
		push_mqueue(&eds[0][y * img_num_w + x]);
		}

	eds[1].clear();
	eds[1].resize(img_num_h * (img_num_w - 1));
	for (int y = 0; y < img_num_h; y++)
		for (int x = 0; x < img_num_w - 1; x++) {
		eds[1][y * (img_num_w - 1) + x].diff = fet.get_edge(1, y, x);
		eds[1][y * (img_num_w - 1) + x].state = FREE;
		eds[1][y * (img_num_w - 1) + x].diff_or_dir = 0;
		push_mqueue(&eds[1][y * (img_num_w - 1) + x]);
		}
}

/*
Input fet
Input ed
Add new edge diff to edge_mqueue
*/
void BundleAdjust::merge(const EdgeDiff * ed, const FeatExt & fet)
{
	map<unsigned, Edge *> share_edges;
	list<ImgMeta *> visit_img;
	list<ImgMeta *> merge_img;
	unsigned killer_bdx, dead_bdx;	
	
	vector<Point> minloc;
	vector<int> minval;	

	found_min(ed->diff, minval, minloc);

	unsigned img_idx0, img_idx1;	
	ed->get_img_idx(img_idx0, img_idx1);
	ImgMeta * img0 = get_img_meta(img_idx0);
	ImgMeta * img1 = get_img_meta(img_idx1);
	Point oo = ed->offset + minloc[0] * scale; //img1 offset to img0
	oo += img0->offset - img1->offset; //img1's origin offset to img0's origin
	if (img0->bundle_idx > img1->bundle_idx) {	//little bundle_idx kill large bundle_idx
		oo = -oo;								//if img1 kill img0, reverse oo
		img0->state = VISIT;
		img0->offset += oo;
		visit_img.push_back(img0);				//img0 is dead
	}
	else {
		img1->state = VISIT;
		img1->offset += oo;
		visit_img.push_back(img1);				//img1 is dead
	}
	CV_Assert(oo.y >= -100);

	killer_bdx = min(img0->bundle_idx, img1->bundle_idx);
	dead_bdx = max(img0->bundle_idx, img1->bundle_idx);
	qDebug("merge (y=%d,x=%d) to (y=%d,x=%d), (oy=%d,ox=%d), cost=%d", IMG_Y(dead_bdx), IMG_X(dead_bdx), 
		IMG_Y(killer_bdx), IMG_X(killer_bdx), oo.y, oo.x, minval[0]);
	short max_merge_ed = 0;
	while (!visit_img.empty()) { //first go through dead img chain
		ImgMeta * img = visit_img.front();
		visit_img.pop_front();
		merge_img.push_back(img);	//push to merge_img for change bundle_idx later
		for (int dir = 0; dir <= 3; dir++) {
			ImgMeta * img_d = get_img_meta(img->get_img_idx(dir));
			if (img_d == NULL)
				continue;
			unsigned ei2 = img->get_edge_idx(dir);
			Edge * e2 = get_edge(ei2); //e2 is between img and img_d
			CV_Assert(e2 != NULL);
			if (img_d->bundle_idx == dead_bdx) {
				if (img_d->state == NOT_VISIT) { //At first all image state is NOT_VISIT, 
					img_d->state = VISIT;	//merge img_d
					img_d->offset += oo;		//recompute dead img offset based on new bundle
					visit_img.push_back(img_d);
					CV_Assert(e2->state == MERGED);
				}
			} else
				if (img_d->bundle_idx == killer_bdx) {
					CV_Assert(img_d->state == NOT_VISIT);
					e2->state = MERGED; 
					//following compute merge cost
					const EdgeDiff * oed = fet.get_edge(ei2);
					Point edge_o = img_d->offset - img->offset;
					if (dir == DIR_UP || dir == DIR_LEFT)
						edge_o = -edge_o;
					CV_Assert(edge_o.y + edge_o.x > 0);
                    e2->diff_or_dir = oed->get_diff(edge_o, scale); //e2 score is e2 merge cost, big is bad
					e2->base_score += e2->diff_or_dir;
                    max_merge_ed = max(e2->diff_or_dir, max_merge_ed); //compute max_merge_ed for confirm edge merge
				}
				else {
					if (e2->state == FREE) {
                        e2->diff_or_dir = dir;
						share_edges[img_d->bundle_idx] = e2;
					}
				}
		}
	}
	if ((max_merge_ed != (short)minval[0])) 
		qFatal("max_merge_ed(%d) != minval(%d) (x=%d,y=%d)", max_merge_ed, minval[0], minloc[0].x, minloc[0].y);	

	if (img0->bundle_idx > img1->bundle_idx) { //little bundle_idx kill large bundle_idx
		img1->state = VISIT;
		visit_img.push_back(img1);	//img1 is killer
	}
	else {
		img0->state = VISIT;
		visit_img.push_back(img0);	//img0 is killer
	}

	while (!visit_img.empty()) { //Then go through killer img chain
		ImgMeta * img = visit_img.front();
		visit_img.pop_front();
		merge_img.push_back(img);	//push to merge_img for change state later
		for (int dir = 0; dir <= 3; dir++) {
			ImgMeta * img_d = get_img_meta(img->get_img_idx(dir));
			if (img_d == NULL)
				continue;
			unsigned ei2 = img->get_edge_idx(dir);
			Edge * e1 = get_edge(ei2); //e1 is between img_d and img
			CV_Assert(e1 != NULL);
			if (img_d->bundle_idx == killer_bdx) {
				if (img_d->state == NOT_VISIT) {
					img_d->state = VISIT;
					visit_img.push_back(img_d);
					CV_Assert(e1->state == MERGED);
				}
			} else
				if (img_d->bundle_idx == dead_bdx) {
					CV_Assert(img_d->state == VISIT);
				}
				else 
					if (e1->state == FREE) {
						map<unsigned, Edge *>::iterator iter = share_edges.find(img_d->bundle_idx);
						if (iter != share_edges.end()) { //e1 has share edge in dead image, recompute edge diff
							Edge * e2 = iter->second;	

							EdgeDiff * new_ed = e1->diff->clone();
							e1->diff = new_ed;
							new_eds.push_back(new_ed);
							
							bool forward_merge;
							Point p1, p2, m1, m2;
							if (dir == DIR_UP || dir == DIR_LEFT) {								
                                forward_merge = (e2->diff_or_dir == DIR_UP || e2->diff_or_dir == DIR_LEFT);
								m1 = img->offset;
								p1 = img_d->offset;
								unsigned i2, i2d;
								e2->diff->get_img_idx(i2, i2d);
								if (forward_merge) {
									ImgMeta * ano_img = get_img_meta(i2d);
									CV_Assert(ano_img->bundle_idx == dead_bdx);
									m2 = ano_img->offset;
									p2 = get_img_meta(i2)->offset;
								}
								else {
									ImgMeta * ano_img = get_img_meta(i2);
									CV_Assert(ano_img->bundle_idx == dead_bdx);
									m2 = ano_img->offset;
									p2 = get_img_meta(i2d)->offset;
								}
							} 
							else {
                                forward_merge = (e2->diff_or_dir == DIR_DOWN || e2->diff_or_dir == DIR_RIGHT);
								m1 = img_d->offset;
								p1 = img->offset;
								unsigned i2, i2d;
								e2->diff->get_img_idx(i2, i2d);
								if (forward_merge) {
									ImgMeta * ano_img = get_img_meta(i2);
									CV_Assert(ano_img->bundle_idx == dead_bdx);
									m2 = get_img_meta(i2d)->offset;
									p2 = ano_img->offset;
								}
								else {
									ImgMeta * ano_img = get_img_meta(i2d);
									CV_Assert(ano_img->bundle_idx == dead_bdx);
									m2 = get_img_meta(i2)->offset;
									p2 = ano_img->offset;
								}
							}
							e2->state = SHARED;
                            e2->diff_or_dir = 0;
							if (forward_merge)
								new_ed->fw_merge(*(e2->diff), p2 - p1, m2 - m1, scale);
							else
								new_ed->bw_merge(*(e2->diff), p2 - p1, m2 - m1, scale);
							push_mqueue(e1);
						}
					}
		}
	}

	for (list<ImgMeta *>::iterator iter = merge_img.begin(); iter != merge_img.end(); iter++) {
		ImgMeta * img = *iter;
		if (img->bundle_idx == dead_bdx)
			img->bundle_idx = killer_bdx;
		img->state = NOT_VISIT;
	}
}

Mat_<Vec2i> BundleAdjust::arrange(const FeatExt & fet, int _img_num_h, int _img_num_w)
{
	int merge_num = 0;

	progress = 0;
	init(fet, _img_num_h, _img_num_w);
	while (!edge_mqueue.empty()) {
		Edge * me;
		do {
			me = edge_mqueue.front();
			edge_mqueue.pop_front();
		} while (me->state != FREE && !edge_mqueue.empty());
		if (me->state != FREE)
			break;
		merge_num++;
		progress = (float)merge_num / (img_num_h * img_num_w - 1);
		merge(me->diff, fet);
	}
	CV_Assert(merge_num == img_num_h * img_num_w - 1);
	release_new_eds();

	Mat_<Vec2i> ret;
	ret.create(img_num_h, img_num_w);
	for (int y = 0; y < img_num_h; y++) {
		for (int x = 0; x < img_num_w; x++) {
			Point offset = imgs[y*img_num_w + x].offset;
			ret(y, x) = Vec2i(offset.y, offset.x);
		}
	}
	return ret;
}
