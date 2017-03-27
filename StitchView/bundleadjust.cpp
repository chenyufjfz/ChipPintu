#include "bundleadjust.h"
#include <stdio.h>
#include <QtGlobal>

//state for img
#define NOT_VISIT	0
#define VISIT		1

//state for edge
#define FREE	0
#define MERGED	1
#define SHARED	2


struct EdgeDiffCmp {
	bool operator()(const Edge * e1, const Edge * e2) {
		return e1->diff->score > e2->diff->score;
	}
};
struct EdgeCmp {
	bool operator()(const Edge * e1, const Edge * e2) {
		return e1->score > e2->score;
	}
};

BundleAdjust::BundleAdjust()
{

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
		if (e->diff->score >= (*iter)->diff->score) {
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

void BundleAdjust::init(FeatExt & fet, int _img_num_h, int _img_num_w)
{
	ConfigPara cpara = fet.get_config_para();
	img_num_h = _img_num_h >= 0 ? _img_num_h : cpara.img_num_h;
	img_num_w = _img_num_w >= 0 ? _img_num_w : cpara.img_num_w;
	scale = cpara.rescale;
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
		eds[0][y * img_num_w + x].score = 0;
		edge_mqueue.push_back(&eds[0][y * img_num_w + x]);
		}

	eds[1].clear();
	eds[1].resize(img_num_h * (img_num_w - 1));
	for (int y = 0; y < img_num_h; y++)
		for (int x = 0; x < img_num_w - 1; x++) {
		eds[1][y * (img_num_w - 1) + x].diff = fet.get_edge(1, y, x);
		eds[1][y * (img_num_w - 1) + x].state = FREE;
		eds[1][y * (img_num_w - 1) + x].score = 0;
		edge_mqueue.push_back(&eds[1][y * (img_num_w - 1) + x]);
		}
	edge_mqueue.sort();
}

void BundleAdjust::merge(EdgeDiff * ed, FeatExt & fet)
{
	map<unsigned, Edge *> share_edges;
	list<ImgMeta *> visit_img;
	list<ImgMeta *> merge_img;
	unsigned killer_bdx, dead_bdx;	
	
	double minval, maxval;
	Point minloc, maxloc;
	minMaxLoc(ed->diff, &minval, &maxval, &minloc, &maxloc);
	
	unsigned img_idx0, img_idx1;	
	ed->get_img_idx(img_idx0, img_idx1);
	ImgMeta * img0 = get_img_meta(img_idx0);
	ImgMeta * img1 = get_img_meta(img_idx1);
	Point oo = ed->offset + minloc * scale; //img1 offset to img0
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
	qDebug("merge (y=%d,x=%d) to (y=%d,x=%d), (oy=%d,ox=%d), cost=%f", IMG_Y(dead_bdx), IMG_X(dead_bdx), 
		IMG_Y(killer_bdx), IMG_X(killer_bdx), oo.y, oo.x, minval);
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
			Edge * e2 = get_edge(ei2);
			CV_Assert(e2 != NULL);
			if (img_d->bundle_idx == dead_bdx) {
				if (img_d->state == NOT_VISIT) {
					img_d->state = VISIT;
					img_d->offset += oo;		//recompute dead img offset 
					visit_img.push_back(img_d);
					CV_Assert(e2->state == MERGED);
				}
			} else
				if (img_d->bundle_idx == killer_bdx) {
					CV_Assert(img_d->state == NOT_VISIT);
					e2->state = MERGED;
					EdgeDiff * oed = fet.get_edge(ei2);
					Point edge_o = img_d->offset - img->offset;
					if (dir == DIR_UP || dir == DIR_LEFT)
						edge_o = -edge_o;
					e2->score = oed->get_diff(edge_o, scale);
					max_merge_ed = max(e2->score, max_merge_ed);
				}
				else {
					if (e2->state == FREE) {
						e2->score = dir;
						share_edges[img_d->bundle_idx] = e2;
					}
				}
		}
	}
	CV_Assert(max_merge_ed == (short) minval);

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
			Edge * e1 = get_edge(ei2);
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
						if (iter != share_edges.end()) {
							Edge * e2 = iter->second;							
							if (e1->diff == fet.get_edge(e1->diff->edge_idx)) {
								e1->diff = e1->diff->clone();
								new_eds.push_back(e1->diff);
							}
							bool forward_merge;
							Point p1, p2, m1, m2;
							if (dir == DIR_UP || dir == DIR_LEFT) {								
								forward_merge = (e2->score == DIR_UP || e2->score == DIR_LEFT);
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
								forward_merge = (e2->score == DIR_DOWN || e2->score == DIR_RIGHT);
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
							e2->score = 0;
							if (forward_merge)
								e1->diff->fw_merge(*(e2->diff), p2 - p1, m2 - m1, scale);
							else
								e1->diff->bw_merge(*(e2->diff), p2 - p1, m2 - m1, scale);
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

Mat_<Vec2i> BundleAdjust::arrange(FeatExt & fet, int _img_num_h, int _img_num_w)
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