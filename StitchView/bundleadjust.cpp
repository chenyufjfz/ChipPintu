#include "bundleadjust.h"
#include <stdio.h>
#include <QtGlobal>
#include <algorithm>
#include <stdlib.h>

#ifdef QT_DEBUG
#ifdef Q_OS_WIN
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif


//state for img, ImgMeta state is NOT_VISIT before and after calling merge
#define NOT_VISIT	0
//ImgMeta state is VISIT during merge
#define VISIT		1

//state for edge, FREE means it can move with bundle freely, MERGED means it is bundle internal edge
//SHARED means it is absorbed into other edge
#define FREE	0
#define MERGED	1
#define SHARED	2


bool edge_cost_cmp(const Edge * e1, const Edge * e2) {
	return e1->cost_or_dir > e2->cost_or_dir;
}


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
	if (y < 0 || x < 0)
		return NULL;
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

//edge_mqueue is sorted from small to big
void BundleAdjust::push_mqueue(Edge * e)
{
	for (list<Edge *>::iterator iter = edge_mqueue.begin(); iter != edge_mqueue.end(); iter++)
		//if (e->diff->score + e->base_score >= (*iter)->diff->score + (*iter)->base_score) {
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

/*
input fet, fet.diff
input _img_num_h, if < 0, img_num_h = fet.cpara.img_num_h
input _img_num_w, if < 0, img_num_w = fet.cpara.img_num_w
Init each img as seperate free
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
		eds[0][y * img_num_w + x].cost_or_dir = 0;
		push_mqueue(&eds[0][y * img_num_w + x]);
		}

	eds[1].clear();
	eds[1].resize(img_num_h * (img_num_w - 1));
	for (int y = 0; y < img_num_h; y++)
		for (int x = 0; x < img_num_w - 1; x++) {
		eds[1][y * (img_num_w - 1) + x].diff = fet.get_edge(1, y, x);
		eds[1][y * (img_num_w - 1) + x].state = FREE;
		eds[1][y * (img_num_w - 1) + x].cost_or_dir = 0;
		push_mqueue(&eds[1][y * (img_num_w - 1) + x]);
		}
}

/*
input fet
input offset
Init all imgs as one bundle with offset
*/
void BundleAdjust::reinit(const FeatExt & fet, const Mat_<Vec2i> & offset, bool replace_offset)
{
	if (replace_offset && (img_num_h != offset.rows || img_num_w != offset.cols)) {
		imgs.clear();
		img_num_h = offset.rows;
		img_num_w = offset.cols;
		imgs.resize(img_num_h * img_num_w);
		eds[0].clear();
		eds[0].resize((img_num_h - 1) * img_num_w);
		eds[1].clear();
		eds[1].resize(img_num_h * (img_num_w - 1));
	}
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w; x++) {
		ImgMeta *pimg = &imgs[y * img_num_w + x];
		pimg->img_idx = MAKE_IMG_IDX(x, y);
		pimg->bundle_idx = 0; //only one bundle	
		pimg->state = NOT_VISIT;
		if (replace_offset)
			pimg->offset = Point(offset(y, x)[1], offset(y, x)[0]);
	}
	release_new_eds();
	edge_mqueue.clear();

	for (int y = 0; y < img_num_h - 1; y++)
	for (int x = 0; x < img_num_w; x++) {
		const EdgeDiff * ed = fet.get_edge(0, y, x);
		eds[0][y * img_num_w + x].diff = ed;
		eds[0][y * img_num_w + x].state = MERGED;
		if (!replace_offset)
			continue;
		Point edge_o = Point(offset(y + 1, x)[1], offset(y + 1, x)[0]) - Point(offset(y, x)[1], offset(y, x)[0]);		
		eds[0][y * img_num_w + x].cost_or_dir = ed->get_dif(edge_o, scale) - ed->mind;
	}

	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w - 1; x++) {
		const EdgeDiff * ed = fet.get_edge(1, y, x);
		eds[1][y * (img_num_w - 1) + x].diff = ed;
		eds[1][y * (img_num_w - 1) + x].state = MERGED;
		if (!replace_offset)
			continue;
		Point edge_o = Point(offset(y, x + 1)[1], offset(y, x + 1)[0]) - Point(offset(y, x)[1], offset(y, x)[0]);		
		eds[1][y * (img_num_w - 1) + x].cost_or_dir = ed->get_dif(edge_o, scale) - ed->mind;
	}
}
/*
Input fet
Input ed
Add new edge diff to edge_mqueue
Return merge cost
*/
int BundleAdjust::merge(const EdgeDiff * ed, const FeatExt & fet)
{
	map<unsigned, Edge *> share_edges;
	list<ImgMeta *> visit_img;
	list<ImgMeta *> merge_img;
	unsigned killer_bdx, dead_bdx;	
	
	//vector<Point> minloc;
	//vector<int> minval;	

	//found_min(ed->diff, minval, minloc);

	unsigned img_idx0, img_idx1;	
	ed->get_img_idx(img_idx0, img_idx1);
	ImgMeta * img0 = get_img_meta(img_idx0);
	ImgMeta * img1 = get_img_meta(img_idx1);
	Point oo = ed->offset + Point(ed->minloc.x, ed->minloc.y) * scale; //img1 offset to img0
	oo += img0->offset - img1->offset; //img1's origin offset to img0's origin
	CV_Assert(img0->bundle_idx != img1->bundle_idx);
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

	killer_bdx = min(img0->bundle_idx, img1->bundle_idx);
	dead_bdx = max(img0->bundle_idx, img1->bundle_idx);
	unsigned max_merge_ed = 0, cost_total = 0;
	Edge * most_big_edge = NULL;
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
					e2->cost_or_dir = oed->get_dif(edge_o, scale) - oed->mind; //e2 score is e2 merge cost, big is bad
					if (e2->cost_or_dir >= max_merge_ed) {//compute max_merge_ed for confirm edge merge
						max_merge_ed = e2->cost_or_dir;
						most_big_edge = e2;
					}
					cost_total += e2->cost_or_dir;
				}
				else {
					if (e2->state == FREE) {
                        e2->cost_or_dir = dir;
						share_edges[img_d->bundle_idx] = e2;
					}
				}
		}
	}
	CV_Assert(most_big_edge!=NULL);
	unsigned img3_idx, img4_idx;
	most_big_edge->diff->get_img_idx(img3_idx, img4_idx);
	qDebug("merge (y=%d,x=%d) to (y=%d,x=%d) (%d,%d) to (%d,%d), (oy=%d,ox=%d), score=%d, min=%d, smin=%d, num=%d, cost=%d", 
		IMG_Y(img3_idx), IMG_X(img3_idx), IMG_Y(img4_idx), IMG_X(img4_idx),
		IMG_Y(dead_bdx), IMG_X(dead_bdx), IMG_Y(killer_bdx), IMG_X(killer_bdx), 
		oo.y, oo.x, ed->score, ed->mind, ed->submind, ed->img_num, max_merge_ed);

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
                                forward_merge = (e2->cost_or_dir == DIR_UP || e2->cost_or_dir == DIR_LEFT);
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
                                forward_merge = (e2->cost_or_dir == DIR_DOWN || e2->cost_or_dir == DIR_RIGHT);
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
                            e2->cost_or_dir = 0;
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

	return cost_total;
}

/*
input m, must broken edge
input n, keep edge
*/
unsigned long long BundleAdjust::split(int m, int n, const FeatExt & fet)
{
	vector<Edge*> es; //point to eds
	vector<pair<unsigned, unsigned> > forbid_edge;
	list<ImgMeta *> visit_img;
	unsigned long long cost_total = 0;
	CV_Assert(n <= eds[0].size() && n <= eds[1].size());

	for (int i = 0; i < (int)imgs.size(); i++) {
		imgs[i].bundle_idx = imgs[i].img_idx;
		imgs[i].state = NOT_VISIT;
	}
	edge_mqueue.clear();
	//1 sort the edge to find forbiden edge
	for (int i = 0; i < 2; i++)
	for (int j = 0; j < eds[i].size(); j++)
		es.push_back(&eds[i][j]);
	sort(es.begin(), es.end(), edge_cost_cmp);

	for (int i = 0; i < m; i++) {
		unsigned img0, img1;
		es[i]->diff->get_img_idx(img0, img1);
		forbid_edge.push_back(make_pair(img0, img1));
	}

	int keep_edge = 0;
	while (keep_edge < n) {
		if (es.size() <= m)
			break;
		//2 pick edge
		int rand_max = min((int)es.size() - m, 5);
		int pick = rand() % rand_max;
		Edge * pick_edge = es[es.size() - 1 - pick];
		es.erase(es.begin() + es.size() - 1 - pick);
		unsigned img_idx0, img_idx1;
		pick_edge->diff->get_img_idx(img_idx0, img_idx1);
		ImgMeta * img0 = get_img_meta(img_idx0);
		ImgMeta * img1 = get_img_meta(img_idx1);
		//2.1 check redundant and forbid rule
		if (img0->bundle_idx == img1->bundle_idx) //redundant edge
			continue;
		bool check_forbid = true;
		for (int i = 0; i<m; i++) 
		if (forbid_edge[i].first == img0->bundle_idx && forbid_edge[i].second == img1->bundle_idx ||
			forbid_edge[i].second == img0->bundle_idx && forbid_edge[i].first == img1->bundle_idx) {
			check_forbid = false;
			break;
		}
		if (!check_forbid)
			continue;

		//3 now edge is pick, merge
		unsigned killer_bdx = min(img0->bundle_idx, img1->bundle_idx);
		unsigned dead_bdx = max(img0->bundle_idx, img1->bundle_idx);
		//3.1 change forbid rule
		for (int i = 0; i<m; i++) {
			if (forbid_edge[i].first == dead_bdx)
				forbid_edge[i].first = killer_bdx;
			if (forbid_edge[i].second == dead_bdx)
				forbid_edge[i].second = killer_bdx;
		}
		//3.2 go through dead bundle and change its bundle idx
		if (img0->bundle_idx > img1->bundle_idx) { //little bundle_idx kill large bundle_idx
			visit_img.push_back(img0);
			img0->bundle_idx = killer_bdx;
		}
		else {
			visit_img.push_back(img1);
			img1->bundle_idx = killer_bdx;
		}

		while (!visit_img.empty()) {
			ImgMeta * img = visit_img.front();
			visit_img.pop_front();
			for (int dir = 0; dir <= 3; dir++) {
				ImgMeta * img_d = get_img_meta(img->get_img_idx(dir));
				if (img_d == NULL)
					continue;
				if (img_d->bundle_idx == dead_bdx) {
					visit_img.push_back(img_d);
					img_d->bundle_idx = killer_bdx;
				}
			}
		}
		keep_edge++;
	}

	//4 Split is done modify edge	

	for (int i = 0; i < (int)imgs.size(); i++) {		
		if (imgs[i].state == VISIT)
			continue;
		map<unsigned, Edge *> free_edges; //contain all free edges
		list<ImgMeta *> merge_img;
		CV_Assert(imgs[i].bundle_idx == imgs[i].img_idx);
		visit_img.push_back(&imgs[i]);
		imgs[i].state = VISIT;
		while (!visit_img.empty()) {
			ImgMeta * img = visit_img.front();
			visit_img.pop_front();
			merge_img.push_back(img);
			for (int dir = 0; dir <= 3; dir++) {
				ImgMeta * img_d = get_img_meta(img->get_img_idx(dir));
				if (img_d == NULL)
					continue;
				Edge * e2 = get_edge(img->get_edge_idx(dir)); //e2 is between img and img_d
				CV_Assert(e2 != NULL);
				if (img_d->bundle_idx == img->bundle_idx) {
					if (img_d->state == VISIT)
						continue;
					const EdgeDiff * oed = fet.get_edge(img->get_edge_idx(dir));
					Point edge_o = img_d->offset - img->offset;
					if (dir == DIR_UP || dir == DIR_LEFT)
						edge_o = -edge_o;
					CV_Assert(edge_o.y + edge_o.x > 0);
					CV_Assert(e2->cost_or_dir == oed->get_dif(edge_o, scale) - oed->mind);
					cost_total += e2->cost_or_dir;
					img_d->state = VISIT;
					CV_Assert(e2->state == MERGED); //originally all edge state is merged
					visit_img.push_back(img_d);
				}
				else 
				if (img_d->bundle_idx > img->bundle_idx) {
					map<unsigned, Edge *>::iterator iter = free_edges.find(img_d->bundle_idx);
					if (iter == free_edges.end()) { //this edge is free edge
						e2->state = FREE;
						e2->cost_or_dir = dir;
						free_edges[img_d->bundle_idx] = e2;
					}
					else {
						Edge * e1 = iter->second;
						CV_Assert(e1->state == FREE);
						EdgeDiff * new_ed = e1->diff->clone();
						e1->diff = new_ed;
						new_eds.push_back(new_ed);

						bool forward_merge;
						Point p1, p2, m1, m2;
						unsigned i1, i1d;
						e1->diff->get_img_idx(i1, i1d);
						if (dir == DIR_UP || dir == DIR_LEFT) {
							forward_merge = (e1->cost_or_dir == DIR_UP || e1->cost_or_dir == DIR_LEFT);
							m2 = img->offset;
							p2 = img_d->offset;							
							if (forward_merge) {
								ImgMeta * ano_img = get_img_meta(i1d);
								CV_Assert(ano_img->bundle_idx == img->bundle_idx);
								m1 = ano_img->offset;
								ano_img = get_img_meta(i1);
								CV_Assert(ano_img->bundle_idx == img_d->bundle_idx);
								p1 = ano_img->offset;
							}
							else {
								ImgMeta * ano_img = get_img_meta(i1);
								CV_Assert(ano_img->bundle_idx == img->bundle_idx);
								m1 = ano_img->offset;
								ano_img = get_img_meta(i1d);
								CV_Assert(ano_img->bundle_idx == img_d->bundle_idx);
								p1 = ano_img->offset;
							}
						}
						else {
							forward_merge = (e1->cost_or_dir == DIR_DOWN || e1->cost_or_dir == DIR_RIGHT);
							m2 = img_d->offset;
							p2 = img->offset;
							if (forward_merge) {
								ImgMeta * ano_img = get_img_meta(i1);
								CV_Assert(ano_img->bundle_idx == img->bundle_idx);
								p1 = ano_img->offset;
								ano_img = get_img_meta(i1d);
								CV_Assert(ano_img->bundle_idx == img_d->bundle_idx);
								m1 = ano_img->offset;								
							}
							else {
								ImgMeta * ano_img = get_img_meta(i1d);
								CV_Assert(ano_img->bundle_idx == img->bundle_idx);
								p1 = ano_img->offset;
								ano_img = get_img_meta(i1);
								CV_Assert(ano_img->bundle_idx == img_d->bundle_idx);
								m1 = ano_img->offset;								
							}
						}
						e2->state = SHARED;
						e2->cost_or_dir = 0;
						if (forward_merge)
							new_ed->fw_merge(*(e2->diff), p2 - p1, m2 - m1, scale);
						else
							new_ed->bw_merge(*(e2->diff), p2 - p1, m2 - m1, scale);
					}
				}
			}
		}
		Point oo = merge_img.front()->offset;
		for (list<ImgMeta *>::iterator iter = merge_img.begin(); iter != merge_img.end(); iter++) {
			ImgMeta * img = *iter;
			img->offset -= oo;
		}
		for (map<unsigned, Edge *>::iterator iter = free_edges.begin(); iter != free_edges.end(); iter++)
			push_mqueue(iter->second);
	}
	for (int i = 0; i < (int)imgs.size(); i++)
		imgs[i].state = NOT_VISIT;
	return cost_total;
}

Edge * BundleAdjust::pick_mq_edge()
{
	Edge * me = NULL;
	/*
	do {
	me = edge_mqueue.front();
	edge_mqueue.pop_front();
	} while (me->state != FREE && !edge_mqueue.empty());
	if (me->state != FREE)
	break;*/
	int score = -1000000;
	int rand_max = 0;
	for (list<Edge *>::iterator iter = edge_mqueue.begin(); iter != edge_mqueue.end(); iter++) {
		if ((*iter)->state != FREE)
			continue;
		if (score == -1000000)
			score = (*iter)->diff->score;
		if ((*iter)->diff->score == score)
			rand_max++;
	}
	if (rand_max == 0)
		return NULL;
	int pick = rand() % rand_max;
	for (list<Edge *>::iterator iter = edge_mqueue.begin(); iter != edge_mqueue.end(); iter++) {
		if ((*iter)->state != FREE)
			continue;
		if (pick == 0) {
			me = *iter;
			edge_mqueue.erase(iter);
			break;
		}
		pick--;
	}
	CV_Assert(me != NULL);
	return me;
}
int BundleAdjust::arrange(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> *, bool)
{
	int merge_num = 0;
    unsigned long long cost = 0, min_cost;
	srand(1);
	init(fet, _img_num_h, _img_num_w);
	while (!edge_mqueue.empty()) {
		Edge * me = pick_mq_edge();
		if (me == NULL)
			break;
		merge_num++;
        cost += merge(me->diff, fet);
	}
	qDebug("cost=%lld", cost);
	CV_Assert(merge_num == img_num_h * img_num_w - 1);
	best_offset.create(img_num_h, img_num_w);
	for (int y = 0; y < img_num_h; y++) {
		for (int x = 0; x < img_num_w; x++) {
			Point offset = imgs[y*img_num_w + x].offset;
			best_offset(y, x) = Vec2i(offset.y, offset.x);
		}
	}
	min_cost = cost;
	if (img_num_h * img_num_w < 5)
		return min_cost;
	int m, n = (img_num_h * img_num_w - 1) / 2;
	if (img_num_h * img_num_w < 10)
		m = 1;
	else
		m = 2;

	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 3; j++) {
			reinit(fet, best_offset, false);
			cost = split(m, n, fet);
			merge_num = n;
			while (!edge_mqueue.empty()) {
				Edge * me = pick_mq_edge();
				if (me == NULL)
					break;
				merge_num++;
				cost += merge(me->diff, fet);
			}
			qDebug("cost=%lld, min_cost=%lld", cost, min_cost);
			CV_Assert(merge_num == img_num_h * img_num_w - 1);

			if (cost < min_cost) {
				min_cost = cost;
				for (int y = 0; y < img_num_h; y++) {
					for (int x = 0; x < img_num_w; x++) {
						Point offset = imgs[y*img_num_w + x].offset;
						best_offset(y, x) = Vec2i(offset.y, offset.x);
					}
				}
			}
		}
		reinit(fet, best_offset, true);
	}
	release_new_eds();
	edge_mqueue.clear();
    return min_cost;
}
