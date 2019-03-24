#include "bundleadjust2.h"
#include <algorithm> 
#include <queue>
#include <functional>
#include <QtConcurrent>

enum {
	NOT_VALID_4CORNER,
	GOOD_4CORNER,
	BAD_4CORNER_HAS_MATE,
	BAD_4CORNER_NO_MATE
};

const int dxy[4][2] = {
	//y , x
	{ -1, 0 }, //up
	{ 0, 1 }, //right
	{ 1, 0 }, //down
	{ 0, -1 }, //left
};

#define SGN(x) ((x)>0 ? 1 : ((x<0) ? -1 : 0))

int Edge2::image_width;
int Edge2::image_height;

struct BundleShape {
	int len;
	int d[4][2];
	int c[4];
	int left;
} bundle_shape[] = {
	//len,  y1,x1,    y2,x2,    y3,x3
	{ 2, { { 0, 1 }, { 0, 0 }, { 0, 0 }, { 0, 0 } }, { 0, 0, 0, 0 }, 1 },
	{ 2, { { 1, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } }, { 0, 0, 0, 0 }, 1 },
	{ 3, { { 0, 1 }, { 0, 2 }, { 0, 0 }, { 0, 0 } }, { 0, 1, 0, 0 }, 1 },
	{ 3, { { 1, 0 }, { 1, 1 }, { 0, 0 }, { 0, 0 } }, { 0, 1, 0, 0 }, 1 },
	{ 3, { { 0, 1 }, { 1, 1 }, { 0, 0 }, { 0, 0 } }, { 0, 1, 0, 0 }, 1 },
	{ 3, { { 1, 0 }, { 2, 0 }, { 0, 0 }, { 0, 0 } }, { 0, 1, 0, 0 }, 1 },
	{ 3, { { 1, 0 }, { 1, -1 }, { 0, 0 }, { 0, 0 } }, { 0, 1, 0, 0 }, 1 },
	{ 3, { { 0, -1 }, { 1, -1 }, { 0, 0 }, { 0, 0 } }, { 0, 1, 0, 0 }, 1 },
	{ 4, { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 0, 1 }, { 0, 2 }, { 1, 2 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 0, 1 }, { 1, 1 }, { 1, 2 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 0, 1 }, { 1, 1 }, { 2, 1 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 1, 0 }, { 1, 1 }, { 1, 2 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 1, 0 }, { 1, 1 }, { 2, 1 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 1, 0 }, { 2, 0 }, { 2, 1 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 1, 0 }, { 2, 0 }, { 2, -1 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 1, 0 }, { 1, -1 }, { 1, -2 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 1, 0 }, { 1, -1 }, { 2, -1 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 0, -1 }, { 1, -1 }, { 2, -1 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 0, -1 }, { 1, -1 }, { 1, -2 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 0, -1 }, { 0, -2 }, { 1, -2 }, { 0, 0 } }, { 0, 0, 1, 0 }, 2 },
	{ 4, { { 1, 0 }, { 2, 0 }, { 1, 1 }, { 0, 0 } }, { 0, 1, 1, 0 }, 1 },
	{ 4, { { 0, 1 }, { 1, 1 }, { 0, 2 }, { 0, 0 } }, { 0, 1, 1, 0 }, 1 },
	{ 4, { { 1, 0 }, { 2, 0 }, { 1, -1 }, { 0, 0 } }, { 0, 1, 1, 0 }, 1 },
	{ 4, { { 1, 0 }, { 1, -1 }, { 1, 1 }, { 0, 0 } }, { 0, 1, 1, 0 }, 1 },
	{ 5, { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 } }, { 0, 0, 0, 1 }, 3 }, // |
	{ 5, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 4, 0 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 3 } }, { 0, 0, 0, 1 }, 3 }, // L
	{ 5, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 3, -1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, -1 }, { 0, -2 }, { 0, -3 }, { -1, -3 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, -1 }, { 1, -1 }, { 2, -1 }, { 3, -1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 0, 2 }, { 0, 3 }, { -1, 3 } }, { 0, 0, 0, 1 }, 3 }, // L
	{ 5, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 3, 1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, -1 }, { 0, -2 }, { 0, -3 }, { 1, -3 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 1, 1 }, { 2, 1 }, { 3, 1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 0, 2 }, { 1, 2 }, { 1, 3 } }, { 0, 0, 0, 1 }, 3 }, // lighting~
	{ 5, { { 1, 0 }, { 2, 0 }, { 2, -1 }, { 3, -1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 1, 1 }, { 1, 2 }, { 1, 3 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 1, 0 }, { 1, -1 }, { 2, -1 }, { 3, -1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 0, 2 }, { -1, 2 }, { -1, 3 } }, { 0, 0, 0, 1 }, 3 }, // lighting~
	{ 5, { { 1, 0 }, { 2, 0 }, { 2, 1 }, { 3, 1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { -1, 1 }, { -1, 2 }, { -1, 3 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 1, 0 }, { 1, 1 }, { 2, 1 }, { 3, 1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 0, 2 }, { 1, 2 }, { 0, 3 } }, { 0, 0, 1, 1 }, 2 }, //
	{ 5, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 2, -1 } }, { 0, 0, 1, 1 }, 2 },
	{ 5, { { 0, -1 }, { 0, -2 }, { -1, -2 }, { 0, -3 } }, { 0, 0, 1, 1 }, 2 },
	{ 5, { { -1, 0 }, { -2, 0 }, { -3, 0 }, { -2, 1 } }, { 0, 0, 1, 1 }, 2 },
	{ 5, { { 0, 1 }, { 0, 2 }, { -1, 2 }, { 0, 3 } }, { 0, 0, 1, 1 }, 2 }, //
	{ 5, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 2, 1 } }, { 0, 0, 1, 1 }, 2 },
	{ 5, { { 0, -1 }, { 0, -2 }, { 1, -2 }, { 0, -3 } }, { 0, 0, 1, 1 }, 2 },
	{ 5, { { -1, 0 }, { -2, 0 }, { -3, 0 }, { -2, -1 } }, { 0, 0, 1, 1 }, 2 },
	{ 5, { { 0, 1 }, { 0, 2 }, { -1, 2 }, { 1, 2 } }, { 0, 0, 1, 1 }, 2 }, // T
	{ 5, { { 1, 0 }, { 2, 0 }, { 2, -1 }, { 2, 1 } }, { 0, 0, 1, 1 }, 2 },
	{ 5, { { 0, -1 }, { 0, -2 }, { -1, -2 }, { 1, -2 } }, { 0, 0, 1, 1 }, 2 },
	{ 5, { { -1, 0 }, { -2, 0 }, { -2, -1 }, { -2, 1 } }, { 0, 0, 1, 1 }, 2 }
};


#define MAKE_BUNDLE(c, changeid, idx, shape, queue, xok, yok, sx0, sx1, sy) make_pair( (((unsigned long long) ((c) * 100) << 32) + (changeid)), \
	((unsigned long long) (idx) << 32) + ((shape) << 8) + ((xok) << 16) + ((yok) << 18) + ((sx0) << 20) + ((sx1) << 24) + ((sy) << 28) + (queue))
#define BUNDLE_CHANGE_ID(b) ((unsigned)(b.first & 0xffffffff))
#define BUNDLE_CORNER_IDX(b) ((unsigned)(b.second >> 32))
#define BUNDLE_SHAPE(b) ((unsigned)(b.second >> 8 & 0xff))
#define BUNDLE_QUEUE(b) ((unsigned)(b.second & 0xff))
#define BUNDLE_XOK(b) ((unsigned)(b.second >> 16 & 3))
#define BUNDLE_YOK(b) ((unsigned)(b.second >> 18 & 3))
#define BUNDLE_SX0(b) ((unsigned)(b.second >> 20 & 0xf))
#define BUNDLE_SX1(b) ((unsigned)(b.second >> 24 & 0xf))
#define BUNDLE_SY(b) ((unsigned)(b.second >> 28 & 0xf))

#define BUNDLE_SHAPE_SQUARE 254
#define BUNDLE_SHAPE_WIDTH  6
#define BUNDLE_SHAPE_INVALID 255

#define PRINT_ADJUST_PATH 1
#define CHECK_LOOP_NUM 5
//#define MERGE_ALL_SUM_TH 25
#define MERGE_ALL_GRID0 10
#define MERGE_ALL_GRID1 15
#define MERGE_ALL_GRID2 22
#define PARALLEL 1
#define VALID_COST_DIFF 50

struct MergeSquarePara {
	BundleAdjust2 * ba;
	Rect r;
	float cost_th;
	MergeSquarePara(BundleAdjust2 * _ba, Rect & _r, float _cost_th) {
		ba = _ba;
		r = _r;
		cost_th = _cost_th;
	}
};

static void thread_merge_square(MergeSquarePara & ms)
{
	ms.ba->merge_square_area(ms.r, ms.r, ms.r, 9, 9, ms.cost_th);
}

/*
input total
input unit
output grid
e.g. total = 50, unit =12, grid = {-1, 11, 23, 35, 50}
total = 50, unit =18, grid = {-1, 17, 35, 50}
*/
static void split(int total, int unit, vector<int> & grid)
{
	grid.clear();
	grid.push_back(-1);

	for (int i = 0;; i++) {
		grid.push_back(grid[i] + unit);
		if (grid.back() >= total - 3) {
			grid.pop_back();
			grid.push_back(total);
			return;
		}
	}
}

template < class T >
void clear_vec(vector< T >& vt)
{
	vector< T > vtTemp;
	veTemp.swap(vt);
}

unsigned queue_number(int shape_len, int dx, int dy)
{
	if (dx == 0 && dy == 0) {
		if (shape_len < 5)
			return shape_len - 2;
		else
		if (shape_len < 10)
			return 2;
		else
		if (shape_len < 17)
			return 3;
		else
			return 4;
	}
	if (abs(dx) + abs(dy) == 1) {
		if (shape_len < 5)
			return shape_len + 1;
		else
		if (shape_len < 10)
			return 5;
		else
		if (shape_len < 17)
			return 6;
		else
			return 7;
	}

	if (dx == 0 || dy == 0) {
		if (shape_len < 5)
			return shape_len + 6;
		else
		if (shape_len < 10)
			return 10;
		else
		if (shape_len < 17)
			return 11;
		else
			return 12;
	}
	if (abs(dx) == 1 || abs(dy) == 1) {
		if (shape_len < 5)
			return shape_len + 9;
		else
		if (shape_len < 10)
			return 13;
		else
		if (shape_len < 17)
			return 14;
		else
			return 15;
	}

	return 1000;
}

/*
Compute pe->cost and pe->hard_score
*/
void BundleAdjust2::compute_edge_cost(Edge2 * pe, float global_avg, bool weak)
{
	float mind = pe->diff->mind;
	float area = pe->diff->dif.rows * pe->diff->dif.cols;
	float beta = (area < 30) ? 1 : ((area >= 130) ? 0.7 : 1 - (area - 30) * 0.003);
	float avg = pe->diff->avg * beta + pe->diff->submind * (1 - beta);
	float alpha = (avg - mind < 29) ? 0 : sqrt(pe->diff->avg / global_avg);

	if (avg - mind < 29) {
		qWarning("edge(%x) cost is all 0 because avg(%f) - mind(%f) is small", pe->diff->edge_idx, avg, mind);
		avg = pe->diff->avg + 20; //make avg bigger to let cost all zero
	}
	else
	if (avg - mind < VALID_COST_DIFF) {
		qWarning("increase edge(%x) avg from %f to %f", pe->diff->edge_idx, avg, mind + VALID_COST_DIFF);
		avg = mind + VALID_COST_DIFF;
	}
	pe->cost.create(pe->diff->dif.size());
	int nearby = FIX_EDGE_SCALE(pe->flagb) / scale;
	int valid_x = FIX_EDGE_BINDX(pe->flagb) ? nearby / 2 : 100000;
	int valid_y = FIX_EDGE_BINDY(pe->flagb) ? nearby / 2 : 100000;
	if (FIX_EDGE_ISBIND(pe->flagb)) { //make fix edge cost euqal to nearly 0
		alpha = alpha / 8;
		avg = pe->diff->avg * 2;
	}
	else
	if (weak) { //make cost smaller
		alpha = alpha * 0.7;
		avg = pe->diff->avg * 1.1;
	}
	Point idea_pos = pe->idea_pos - pe->diff->offset;
	idea_pos.x = idea_pos.x / scale;
	idea_pos.y = idea_pos.y / scale;
	
	pe->hard_score = COST_BIND;
	for (int y = 0; y < pe->cost.rows; y++) {
		const int * pdif_1 = (y>0) ? pe->diff->dif.ptr<int>(y - 1) : NULL;
		const int * pdif = pe->diff->dif.ptr<int>(y);
		const int * pdif1 = (y + 1 < pe->cost.rows) ? pe->diff->dif.ptr<int>(y + 1) : NULL;
		float * pcost = pe->cost.ptr<float>(y);
		for (int x = 0; x < pe->cost.cols; x++) {
			float z = pdif[x];
			if (abs(y - idea_pos.y) <= nearby / 2 && abs(x - idea_pos.x) <= nearby / 2)
				pcost[x] = 0;
			else
			if (abs(y - idea_pos.y) >valid_y || abs(x - idea_pos.x) > valid_x)
				pcost[x] = COST_BIND + z;
			else
			if (z > avg - 1) {
				pcost[x] = COST_BIGER_THAN_AVG + z - avg + 1;
				pcost[x] = min(pcost[x], (float)COST_BOUNDARY - 2);
			}
			else {
				pcost[x] = alpha * (z - mind) / (avg - z);
				pcost[x] = min(pcost[x], (float)COST_BIGER_THAN_AVG);
			}
			if (y > 0 && x > 0 && y + 1 < pe->cost.rows && x + 1 < pe->cost.cols && pdif[x] != mind &&
				pdif[x] < pdif_1[x] && pdif[x] < pdif1[x] && pdif[x] < pdif[x - 1] && pdif[x] < pdif[x + 1] &&
				pdif[x] < pdif_1[x - 1] && pdif[x] < pdif_1[x + 1] && pdif[x] < pdif1[x - 1] && pdif[x] < pdif1[x + 1])
				pe->hard_score = min(pe->hard_score, pcost[x]);
		}
	}
	if (pe->hard_score >= COST_BIND - 10)
		pe->hard_score = alpha;
	if (FIX_EDGE_ISBIND(pe->flagb) && FIX_EDGE_BINDX(pe->flagb) && FIX_EDGE_BINDY(pe->flagb))
		pe->hard_score = COST_BIGER_THAN_AVG;
	if (FIX_EDGE_ISBIND(pe->flagb) && !FIX_EDGE_BINDX(pe->flagb) && !FIX_EDGE_BINDY(pe->flagb))
		pe->hard_score = 0;
	return;
}

Edge2 * BundleAdjust2::get_edge(int i, int y, int x)
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

Edge2 * BundleAdjust2::get_edge(FourCorner * pc0, FourCorner * pc1)
{
	if (CORNER_X(pc0->idx) == CORNER_X(pc1->idx)) {
		if (CORNER_Y(pc0->idx) > CORNER_Y(pc1->idx))
			return get_edge(pc0->get_edge_idx(DIR_UP));
		else
			return get_edge(pc0->get_edge_idx(DIR_DOWN));
	}

	if (CORNER_X(pc0->idx) > CORNER_X(pc1->idx))
		return get_edge(pc0->get_edge_idx(DIR_LEFT));
	else
		return get_edge(pc0->get_edge_idx(DIR_RIGHT));

}

//if idx is invalid, return NULL
Edge2 * BundleAdjust2::get_edge(int idx)
{
	Edge2 * ed = get_edge(EDGE_E(idx), EDGE_Y(idx), EDGE_X(idx));
	CV_Assert(ed == NULL || ed->diff->edge_idx == idx);
	return ed;
}

FourCorner * BundleAdjust2::get_4corner(int y, int x)
{
	if (y > img_num_h || x > img_num_w || x < 0 || y < 0)
		return NULL;
	FourCorner * pcorner = &fc[y * (img_num_w + 1) + x];
	return pcorner;
}

FourCorner * BundleAdjust2::get_4corner(int idx)
{
	int y = CORNER_Y(idx);
	int x = CORNER_X(idx);
	FourCorner * pcorner = get_4corner(y, x);
	CV_Assert(pcorner == NULL || pcorner->idx == idx);
	return pcorner;
}

//print corner res_sft stat
void BundleAdjust2::print_4corner_stat()
{
	vector<double> statx(10, 0), staty(10, 0);
	int total = 0;
	for (int y = 0; y <= img_num_h; y++)
	for (int x = 0; x <= img_num_w; x++) {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		if (pc->bd <= 1)
			continue;
		if (abs(pc->res_sft[1]) < 9)
			statx[abs(pc->res_sft[1])] += 1;
		else
			statx[9] += 1;
		if (abs(pc->res_sft[0]) < 9)
			staty[abs(pc->res_sft[0])] += 1;
		else
			staty[9] += 1;
		total++;
	}
	qDebug("staty: %5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,",
		staty[0], staty[1], staty[2], staty[3], staty[4],
		staty[5], staty[6], staty[7], staty[8], staty[9]);
	qDebug("statx: %5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,%5.0f,",
		statx[0], statx[1], statx[2], statx[3], statx[4],
		statx[5], statx[6], statx[7], statx[8], statx[9]);

	for (int i = 0; i < 10; i++) {
		staty[i] = staty[i] * 100 / total;
		statx[i] = statx[i] * 100 / total;
	}

	qDebug("staty: %4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%",
		staty[0], staty[1], staty[2], staty[3], staty[4],
		staty[5], staty[6], staty[7], staty[8], staty[9]);
	qDebug("statx: %4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%",
		statx[0], statx[1], statx[2], statx[3], statx[4],
		statx[5], statx[6], statx[7], statx[8], statx[9]);
}

void BundleAdjust2::init(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe, Size s, bool week_border)
{
	ConfigPara cpara = fet.get_config_para();
	Edge2::image_width = s.width;
	Edge2::image_height = s.height;
	img_num_h = _img_num_h >= 0 ? _img_num_h : cpara.img_num_h;
	img_num_w = _img_num_w >= 0 ? _img_num_w : cpara.img_num_w;
	qInfo("Init img_height=%d, image_width=%d, img_num_h=%d, img_num_w=%d", s.height, s.width, img_num_h, img_num_w);
	scale = cpara.rescale;
	//1 Init edge diff, mls, idea_pos, flag
	double avg[2] = { 0, 0 };
	eds[0].clear();
	eds[0].resize((img_num_h - 1) * img_num_w);
	for (int y = 0; y < img_num_h - 1; y++)
	for (int x = 0; x < img_num_w; x++) {
		Edge2 * pe = &eds[0][y * img_num_w + x];
		pe->diff = fet.get_edge(0, y, x);
		pe->mls[0] = 0;
		pe->mls[1] = 0;
		pe->flagb = 0;
		pe->idea_pos = pe->diff->minloc * scale + pe->diff->offset;
		CV_Assert(pe->idea_pos.x % scale == 0 && pe->idea_pos.y % scale == 0);
		avg[0] += pe->diff->avg;
	}
	avg[0] = avg[0] / ((img_num_h - 1) * img_num_w);

	eds[1].clear();
	eds[1].resize(img_num_h * (img_num_w - 1));
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w - 1; x++) {
		Edge2 * pe = &eds[1][y * (img_num_w - 1) + x];
		pe->diff = fet.get_edge(1, y, x);
		pe->mls[0] = 0;
		pe->mls[1] = 0;
		pe->flagb = 0;
		pe->idea_pos = pe->diff->minloc * scale + pe->diff->offset;
		CV_Assert(pe->idea_pos.x % scale == 0 && pe->idea_pos.y % scale == 0);
		avg[1] += pe->diff->avg;
	}
	avg[1] = avg[1] / (img_num_h * (img_num_w - 1));

	if (fe != NULL) {
		const vector<FixEdge> & vfe = *fe;
		for (int i = 0; i < vfe.size(); i++) {
			qInfo("BundleAdjust2 apply fixedge ix=%d,iy=%d,e=%d, offset=(%d,%d), bind=%d", EDGE_X(vfe[i].idx),
				EDGE_Y(vfe[i].idx), EDGE_E(vfe[i].idx), vfe[i].shift.x, vfe[i].shift.y, vfe[i].bind_flag);
			Edge2 * pe = get_edge(vfe[i].idx);
			pe->flagb = vfe[i].bind_flag;
			Point idea_pos = vfe[i].shift - pe->diff->offset;
			CV_Assert(idea_pos.x % scale == 0 && idea_pos.y % scale == 0);
			pe->idea_pos = idea_pos + pe->diff->offset;
		}
	}

	//2 Init fourcorner
	fc.resize((img_num_h + 1) * (img_num_w + 1));
	queue<FourCorner *> bd_update;
	//2.1 init res_sft and bd
	corner_info.create(img_num_h, img_num_w);
	for (int y = 0; y <= img_num_h; y++)
	for (int x = 0; x <= img_num_w; x++) {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		pc->idx = MAKE_CORNER_IDX(x, y);
		pc->bd = min(min(x, y), min(img_num_w - x, img_num_h - y));
		Point res_sft(0, 0);
		if (pc->bd) {
			int bind_mask = 0;
			for (int i = 0; i < 4; i++) {
				Edge2 * pe = get_edge(pc->get_edge_idx(i));
				CV_Assert(pe != NULL);
				if (i < 2)
					res_sft += pe->idea_pos;
				else
					res_sft -= pe->idea_pos;
				if (FIX_EDGE_ISBIND(pe->flagb)) {
					if (!FIX_EDGE_BINDX(pe->flagb))
						bind_mask |= BIND_X_MASK;
					if (!FIX_EDGE_BINDY(pe->flagb))
						bind_mask |= BIND_Y_MASK;
				}
			}
			corner_info(y, x) = (res_sft.x & 0xffff) | (res_sft.y & 0xffff) << 16;
			/*if (bind_mask & BIND_X_MASK) //has at least one green fix x
				corner_info(y, x) = res_sft.y << 16;
			if (bind_mask & BIND_Y_MASK)
				corner_info(y, x) = res_sft.x & 0xffff; //has at least one green fix y*/
			corner_info(y, x) &= 0xffffffff;
		}
		else {
			if (y < img_num_h && x < img_num_w)
				corner_info(y, x) = 0;
			bd_update.push(pc);
		}

		CV_Assert(res_sft.x % scale == 0 && res_sft.y % scale == 0);
		res_sft.x = res_sft.x / scale;
		res_sft.y = res_sft.y / scale;
		pc->res_sft[0] = res_sft.y;
		pc->res_sft[1] = res_sft.x;
		pc->change_id = 0;
		pc->change_x = (res_sft.x == 0) ? 0 : 65536;
		pc->change_y = (res_sft.y == 0) ? 0 : 65536;
	}
	//2.2 second loop init bd
	while (!bd_update.empty()) {
		FourCorner * pc = bd_update.front();
		bd_update.pop();
		for (int dir = 0; dir < 4; dir++) {
			FourCorner * pc1 = get_4corner(pc->get_4corner_idx(dir));
			if (pc1 == NULL || pc1->bd <= pc->bd)
				continue;
			if (pc->bd == 0) {
				Edge2 * pe = get_edge(pc->get_edge_idx(dir));
				CV_Assert(pe != NULL);
				if (pe->diff->img_num == 0) {
					pc1->bd = 0;
					corner_info(CORNER_Y(pc1->idx), CORNER_X(pc1->idx)) = 0;
					pc1->res_sft[0] = 0;
					pc1->res_sft[1] = 0;
					pc1->change_x = 0;
					pc1->change_y = 0;
					bd_update.push(pc1);
					qInfo("Add new border x=%d, y=%d", CORNER_X(pc1->idx), CORNER_Y(pc1->idx));
				}
			}
			if (pc1->bd > pc->bd + 1) {
				pc1->bd = pc->bd + 1;
				bd_update.push(pc1);
			}
		}
	}
	//2.3 init edge cost
	for (int i = 0; i < 2; i++)
	for (int j = 0; j < (int)eds[i].size(); j++) {
		bool weak = false;
		if (week_border) {
			unsigned img0, img1;
			eds[i][j].get_4corner_idx(img0, img1);
			if (get_4corner(img0)->bd == 0 || get_4corner(img1)->bd == 0)
				weak = true; //it is border
		}
		compute_edge_cost(&eds[i][j], avg[i], weak);
	}
	
	//3 init best offset
	best_offset.create(img_num_h, img_num_w);
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w; x++)
		best_offset(y, x) = cpara.offset(y, x);
}

void BundleAdjust2::undo(const UndoPatch & patch)
{
	for (map<unsigned, pair<int, int> >::const_iterator pep = patch.edge_patch.begin(); pep != patch.edge_patch.end(); pep++) {
		Edge2 * pe = get_edge(pep->first);
		pe->mls[0] = pep->second.first;
		pe->mls[1] = pep->second.second;
	}
	for (map<unsigned, FourCornerPatch>::const_iterator pcp = patch.corner_patch.begin(); pcp != patch.corner_patch.end(); pcp++) {
		FourCorner * pc = get_4corner(pcp->first);
		pc->res_sft[0] = pcp->second.res_sft[0];
		pc->res_sft[1] = pcp->second.res_sft[1];
		pc->change_id = pcp->second.change_id;
		pc->change_x = pcp->second.change_x;
		pc->change_y = pcp->second.change_y;
	}
}
/*
Input FourCorner * ps,
Input FourCorner * pt,  ps==pt means loop, else not loop
Input sidx, source[sidx]
Input modify,
output rq
output patch, for undo
input change_id
input cost_th, reset cs
border_is_source
if true, border is used as source, so not update border.
if false, border is used as dest, so update border
It depend on source, adjust edge from ps to pt with modify
*/
void BundleAdjust2::adjust_edge_mls(FourCorner * ps, FourCorner * pt, int sidx, int modify, queue<unsigned> & rq, vector<SourceInfo> & source, int change_id, UndoPatch * patch, float cost_th, bool border_is_source)
{
	//following change path FourCorner update_state to 2
	int isloop = (ps == pt) ? 1 : 0;
	FourCorner * pa = pt;
	vector<FourCorner *> invalid_queue;
	vector<unsigned> invalid_source;
#if PRINT_ADJUST_PATH
	char path[100];
	int path_len = sprintf(path, "adjust_edge_mls %c %d, (x=%d,y=%d)", source[sidx].isx ? 'x' : 'y',
		source[sidx].sign *modify, CORNER_X(pa->idx), CORNER_Y(pa->idx));
#endif
	while (1) {
		FourCorner * pb = get_4corner(pa->cs[sidx][modify].fa);
		//following change path FourCorner update_state to 2
		if (!border_is_source || pa->bd > 0) {
			for (int i = 0; i < pa->cs.size(); i++) {
				bool enqueue_invalid_source = false;
				for (int j = 1; j <= source[i].cap; j++) {
					if (source[i].idx == pa->idx) { //pa is source[i]
						if (ps != pt) {//not loop, no update
							CV_Assert(pa->cs[i][j].fa == pa->idx && abs(pa->cs[i][j].cost) < 0.001);
							continue;
						}
						if (pa->cs[i][j].fa != pa->idx)
							enqueue_invalid_source = true;
						else {
							CV_Assert(abs(pa->cs[i][j].cost) < 0.001);
							continue;
						}
					}
					pa->cs[i][j].reset(cost_th + 1);
					pa->cs[i][j].update_state = 2;
					if (pa->bd != 0 && !pa->already_inqueue) {
						rq.push(pa->idx);
						pa->already_inqueue = true;
					}
				}
				if (enqueue_invalid_source)
					invalid_source.push_back(i);
			}
			invalid_queue.push_back(pa);
		}
		if (pa == ps) {
			if (isloop)
				isloop = 0;
			else
				break;
		}
		//following change edge mls
		if (pb == NULL)
			qFatal("pa->idx=%x, sidx=%d, modify=%d", pa->idx, sidx, modify);
		int dir = pa->get_dir(pb->idx);
		Edge2 * pe = get_edge(pa->get_edge_idx(dir));
		CV_Assert(pe != NULL);
		int dmls = (dir < 2) ? source[sidx].sign * modify : -source[sidx].sign *modify;
		if (patch != NULL)
			patch->add_patch(pe->diff->edge_idx, pe);
		if (source[sidx].isx)
			pe->mls[1] += dmls;
		else
			pe->mls[0] += dmls;
		if (change_id > 0) {
			if (patch != NULL)
				patch->add_patch(pb->idx, pb);
			pb->change_id = change_id;
		}
#if PRINT_ADJUST_PATH
		path_len += sprintf(path + path_len, "<-%d->(%d,%d)", dmls, CORNER_X(pb->idx), CORNER_Y(pb->idx));
		if (path_len > 60) {
			qDebug("%s", path);
			path_len = 0;
		}
#endif
		pa = pb;
	}
#if PRINT_ADJUST_PATH
	if (path_len > 0)
		qDebug("%s", path);
#endif

	if (ps != pt) { //not loop
		if (change_id > 0) {
			if (patch != NULL)
				patch->add_patch(pt->idx, pt);
			pt->change_id = change_id;
			if (source[sidx].isx) {
				if (ps->res_sft[1] != 0 && ps->change_x)
					ps->change_x = change_id;
				if (pt->res_sft[1] != 0 && pt->change_x)
					pt->change_x = change_id;
			}
			else {
				if (ps->res_sft[0] != 0 && ps->change_y)
					ps->change_y = change_id;
				if (pt->res_sft[0] != 0 && pt->change_y)
					pt->change_y = change_id;
			}
		}
		if (source[sidx].isx) {
			if (ps->bd)
				ps->res_sft[1] -= source[sidx].sign *modify;
			if (pt->bd)
				pt->res_sft[1] += source[sidx].sign *modify;
		}
		else {
			if (ps->bd)
				ps->res_sft[0] -= source[sidx].sign *modify;
			if (pt->bd)
				pt->res_sft[0] += source[sidx].sign *modify;
		}
	}
	while (!invalid_queue.empty()) { //propogate more cs update_state to 2
		pa = invalid_queue.back();
		invalid_queue.pop_back();
		for (int dir = 0; dir < 4; dir++) {
			FourCorner * pb = get_4corner(pa->get_4corner_idx(dir));
			if (pb == NULL || pb->cs.size() == 0)
				continue;
			if (border_is_source && pb->bd == 0)
				continue;
			bool enqueue = false;
			for (int i = 0; i < pa->cs.size(); i++) {
				bool enqueue_invalid_source = false;
				for (int j = source[i].cap; j >0; j--)
				if (pa->cs[i][j].update_state == 2 && pb->cs[i][j].fa == pa->idx) {
					if (ps != pt) //not loop, pb is not source
						CV_Assert(source[i].idx != pb->idx);
					if (source[i].idx == pb->idx) //pb is source[i]
						enqueue_invalid_source = true;
					pb->cs[i][j].reset(cost_th + 1);
					pb->cs[i][j].update_state = 2;
					enqueue = true;
					if (pb->bd != 0 && !pb->already_inqueue) {
						rq.push(pb->idx);
						pb->already_inqueue = true;
					}
				}
				if (enqueue_invalid_source)
					invalid_source.push_back(i);
			}
			if (enqueue)
				invalid_queue.push_back(pb);
		}
	}
	if (border_is_source)
		CV_Assert(invalid_source.empty());
	for (int k = 0; k < invalid_source.size(); k++) {
		int i = invalid_source[k];
		pa = get_4corner(source[i].idx);
		CV_Assert(pa->bd);
		for (int j = 1; j <= source[i].cap; j++) {
			if (pa->cs[i][j].update_state == 2) {
				pa->cs[i][j].cost = 0;
				pa->cs[i][j].fa = pa->idx;
				pa->cs[i][j].update_state = 1;
				CV_Assert(pa->already_inqueue);
			}
		}
	}
}

/*
Inout FourCorner * pc, if pc->cs[i].update==1, propogate nearby cost. if pc->cs[i].update==2, update pc cost
Input range, relax Corner is in Rect range
Ouput rq, relax queue
output patch, for undo
input cost_th, pass to adjust_edge_mls
input border_is_source,
if true, border is used as source, so not update border.
if false, border is used as dest, so update border
It depend on source
*/
void BundleAdjust2::relax(FourCorner * pc, const Rect & range, queue<unsigned> & rq, vector<SourceInfo> & source, UndoPatch * patch, float cost_th, bool border_is_source)
{
	CV_Assert(pc->already_inqueue && pc->cs.size() == source.size());
	CV_Assert(border_is_source || pc->bd > 0);
	pc->already_inqueue = false;
	int max_shift_num = 0;
	for (int i = 0; i < pc->cs.size(); i++) {
		max_shift_num = max(max_shift_num, (int)pc->cs[i].size());
		CV_Assert((int)pc->cs[i].size() > source[i].cap);
	}
	for (int dir = 0; dir < 4; dir++) {
		FourCorner * pc1 = get_4corner(pc->get_4corner_idx(dir));
		if (pc1 == NULL)
			continue;
		Edge2 * pe = get_edge(pc->get_edge_idx(dir));
		if (pe == NULL)
			continue;
		if (!range.contains(Point(CORNER_X(pc1->idx), CORNER_Y(pc1->idx)))) //pc1 not in range
			continue;
		Point cur_edge_point = pe->get_current_point(scale);
		float cur_edge_cost = pe->get_point_cost(cur_edge_point, scale);
		int sign = dir < 2 ? -1 : 1;
		vector <float> new_edge_cost_cache(max_shift_num * 4);
		vector <int> new_cost_cache_valid(max_shift_num * 4, 0);
		for (int i = 0; i < pc->cs.size(); i++) {
			for (int j = source[i].cap; j >0; j--)
			if (pc->cs[i][j].update_state == 1) {	//propogate cs[i][j] from pc to pc1
				CV_Assert(pc->cs[i][j].fa != 0xffffffff);
				if (pc1->idx == pc->cs[i][j].fa || border_is_source && pc1->bd == 0)
					continue;
				int cache_idx = j * 4 + (source[i].sign + 1) + source[i].isx;
				float new_edge_cost; //compute new edge cost to decide if pc1 need to be updated
				if (new_cost_cache_valid[cache_idx])
					new_edge_cost = new_edge_cost_cache[cache_idx];
				else {
					Point new_edge_point = cur_edge_point;
					if (source[i].isx)
						new_edge_point.x += sign * source[i].sign * j;
					else
						new_edge_point.y += sign * source[i].sign * j;
					new_edge_cost = pe->get_point_cost(new_edge_point, scale);
					new_edge_cost_cache[cache_idx] = new_edge_cost;
					new_cost_cache_valid[cache_idx] = 1;
				}
				float delta_cost = new_edge_cost - cur_edge_cost;
				double dcost = pc1->cs[i][j].cost - pc->cs[i][j].cost - delta_cost;
				if (dcost > 0.000001 && new_edge_cost < COST_BIND / 2) { //update pc1, check >0.000001 is to avoid double add error
					pc1->cs[i][j].cost = pc->cs[i][j].cost + delta_cost;
					pc1->cs[i][j].fa = pc->idx;
/*
//for debug purpose enable following
					if (i == 0 && j == 8 && pc->idx == 0x000c001a && pc1->idx == 0x000c0019) {
						pc->cs[i][j].update_num = pc->cs[i][j].update_num * 2 - pc->cs[i][j].update_num;
					}
					if (i == 0 && j == 8 && pc1->idx == 0x000c001a && pc->idx == 0x000d001a) {
						pc->cs[i][j].update_num = pc->cs[i][j].update_num * 2 - pc->cs[i][j].update_num;
					}

*/
					if (pc1->cs[i][j].update_state != 2) //if pc1 state is 2, should still hold as 2
						pc1->cs[i][j].update_state = 1;
					pc1->cs[i][j].update_num++;
					if (pc1->cs[i][j].update_num > CHECK_LOOP_NUM) { //check if loop exist
						FourCorner * pa = pc1;
						pa->visited = true;
						int cnt = 1;
						while (pa->idx != pa->cs[i][j].fa && pa->cs[i][j].fa != 0xffffffff) {
							pa = get_4corner(pa->cs[i][j].fa);
							if (pa->visited) //find loop
								break;
							pa->visited = true;
							cnt++;
						}
						bool find_loop = (pa->idx != pa->cs[i][j].fa && pa->cs[i][j].fa != 0xffffffff);
						//following clear visited, no matter loop exist or not, clear update_num
						FourCorner * pb = pc1;
						for (; cnt>0; cnt--) {
							pb->visited = false;
							pb->cs[i][j].update_num = 0;
							pb = get_4corner(pb->cs[i][j].fa);
						}
						pa->visited = false;
						pa->cs[i][j].update_num = 0;
						if (find_loop) {//if not reach root, find loop
							qDebug("relax find loop shift %c=%d", source[i].isx ? 'x' : 'y', j);
							adjust_edge_mls(pa, pa, i, j, rq, source, -1, patch, cost_th, border_is_source);
							return;
						}
					}
					if (pc1->bd != 0 && !pc1->already_inqueue) { //push pc1 to queue					
						rq.push(pc1->idx);
						pc1->already_inqueue = true;
					}
				}
			}
			else
			if (pc->cs[i][j].update_state == 2) { //propogate cs[i][j] from pc1 to pc1
				if (border_is_source)
					CV_Assert(pc->bd != 0);
				if (pc1->cs[i][j].update_state == 2 || pc1->cs[i][j].fa == 0xffffffff)
					continue;
				if (!border_is_source && pc1->bd == 0)
					continue;
				if (pc1->cs[i][j].fa == pc->idx) {
					CV_Assert(0);
				}
				int cache_idx = j * 4 + (-source[i].sign + 1) + source[i].isx;
				float new_edge_cost; //compute new edge cost to decide if pc need to be updated
				if (new_cost_cache_valid[cache_idx])
					new_edge_cost = new_edge_cost_cache[cache_idx];
				else {
					Point new_edge_point = cur_edge_point;
					if (source[i].isx)
						new_edge_point.x += -sign * source[i].sign * j;
					else
						new_edge_point.y += -sign * source[i].sign * j;
					new_edge_cost = pe->get_point_cost(new_edge_point, scale);
					new_edge_cost_cache[cache_idx] = new_edge_cost;
					new_cost_cache_valid[cache_idx] = 1;
				}
				float delta_cost = new_edge_cost - cur_edge_cost;
				if (pc->cs[i][j].cost > pc1->cs[i][j].cost + delta_cost && new_edge_cost < COST_BIND / 2) { //update pc
					pc->cs[i][j].cost = pc1->cs[i][j].cost + delta_cost;
					pc->cs[i][j].fa = pc1->idx;
					pc->cs[i][j].update_num++;
/*
//for debug purpose enable following
					if (i == 0 && j == 8 && pc1->idx == 0x000c001a && pc->idx == 0x000c0019) {
						pc->cs[i][j].update_num = pc->cs[i][j].update_num * 2 - pc->cs[i][j].update_num;
					}
					if (i == 0 && j == 8 && pc->idx == 0x000c001a && pc1->idx == 0x000d001a) {
						pc->cs[i][j].update_num = pc->cs[i][j].update_num * 2 - pc->cs[i][j].update_num;
					}

*/
				}
			}
		}
	}
	for (int i = 0; i < pc->cs.size(); i++)
	for (int j = 1; j <= source[i].cap; j++) {
		if (pc->cs[i][j].update_state == 1)
			pc->cs[i][j].update_state = 0;
		if (pc->cs[i][j].update_state == 2) {
			if (pc->cs[i][j].fa != 0xffffffff) {
				pc->cs[i][j].update_state = 1;
				if (!pc->already_inqueue) {
					rq.push(pc->idx);
					pc->already_inqueue = true;
				}
			}
			else
				pc->cs[i][j].update_state = 0;
		}
	}
}

void BundleAdjust2::check_relax(const Rect & rect, vector<SourceInfo> & source, float)
{
	for (int y = rect.y; y < rect.y + rect.height; y++)
	for (int x = rect.x; x < rect.x + rect.width; x++)  {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		if (pc->bd == 0) {
			CV_Assert(pc->res_sft[0] == 0 && pc->res_sft[1] == 0);
			continue;
		}
		CV_Assert(!pc->already_inqueue && !pc->visited);
		for (int dir = 0; dir < 4; dir++) {
			FourCorner * pc1 = get_4corner(pc->get_4corner_idx(dir));
			Edge2 * pe = get_edge(pc->get_edge_idx(dir));
			CV_Assert(pc1 != NULL && pe != NULL);
			if (!rect.contains(Point(CORNER_X(pc1->idx), CORNER_Y(pc1->idx)))) //pc1 not in range
				continue;
			if (pe->diff->img_num == 0)
				continue;
			Point cur_edge_point = pe->get_current_point(scale);
			float cur_edge_cost = pe->get_point_cost(cur_edge_point, scale);
			int sign = dir < 2 ? -1 : 1;
			for (int i = 0; i < pc->cs.size(); i++)
			for (int j = 1; j <= source[i].cap; j++) {
				CV_Assert(pc->cs[i][j].update_state == 0);
				Point new_edge_point = cur_edge_point;
				if (source[i].isx)
					new_edge_point.x += -sign * source[i].sign * j;
				else
					new_edge_point.y += -sign * source[i].sign * j;
				float new_edge_cost = pe->get_point_cost(new_edge_point, scale);
				float delta_cost = new_edge_cost - cur_edge_cost; //from pc1 to pc
				if (new_edge_cost > COST_BIND / 2) {
					CV_Assert(pc1->idx != pc->cs[i][j].fa);
					continue;
				}
				if (pc1->idx == pc->cs[i][j].fa) {
					if (abs(pc->cs[i][j].cost - pc1->cs[i][j].cost - delta_cost) > 0.001) { //pc.cost[i][j]=pc1.cost[i][j]+delta_cost
						qCritical("check_relax error, cost(x=%d,%d)=%f, facost(%d,%d)=%f, source=(%d,%d) delta=%f, err=%f",
							x, y, pc->cs[i][j].cost, CORNER_X(pc1->idx), CORNER_Y(pc1->idx), pc1->cs[i][j].cost,
							CORNER_X(source[i].idx), CORNER_Y(source[i].idx),
							delta_cost, abs(pc->cs[i][j].cost - pc1->cs[i][j].cost - delta_cost));
						CV_Assert(0);
					}
				}
				else
				if (pc1->cs[i][j].fa != pc->idx && pc1->cs[i][j].fa != 0xffffffff && pc1->bd!=0)
					CV_Assert(pc->cs[i][j].cost <= pc1->cs[i][j].cost + delta_cost + 0.001); //pc.cost[i][j]<=pc1.cost[i][j]+delta_cost
				if (pc->cs[i][j].fa == pc->idx)
					CV_Assert(source[i].idx == pc->idx);
				if (source[i].idx == pc->idx)
					CV_Assert(pc->cs[i][j].fa == pc->idx && abs(pc->cs[i][j].cost) < 0.001);
			}
		}
	}

}

void BundleAdjust2::check_fix_edge()
{
	for (int i = 0; i < 2; i++)
	for (int j = 0; j < eds[i].size(); j++)
	if (FIX_EDGE_ISBIND(eds[i][j].flagb)) {
		Edge2 * pe = &eds[i][j];
		Point cur_edge_point = pe->get_current_point(scale);
		float cur_edge_cost = pe->get_point_cost(cur_edge_point, scale);
		if (cur_edge_cost > COST_BIND / 2) {
			Point pos = pe->idea_pos - pe->diff->offset;
			pos.x = pos.x / scale;
			pos.y = pos.y / scale;
			qCritical("internal error, edge (x=%d,y=%d,e=%d,f=0x%x), idea (%d,%d), actual (%d,%d)", EDGE_X(pe->diff->edge_idx),
				EDGE_Y(pe->diff->edge_idx), EDGE_E(pe->diff->edge_idx), pe->flagb, pos.x, pos.y, cur_edge_point.x, cur_edge_point.y);
		}
	}
}

/*
Input src_rect, source FourCorner rect
Input dst_rect, dest FourCorner rect
Input outer, route boundary
src_posx =-2, merge negative x as source, if can't find path, no undo
src_posx =-1, merge negative x as source, if can't find path, undo
src_posx =1,  merge positive x as source, if can't find path, undo
src_posx =2,  merge positive x as source, if can't find path, no undo
src_posx =0, not merge x
src_posx =9, choos negative or positive x as source based on number
src_posy =-2,  merge negative y as source, if can't find path, no undo
src_posy =-1,  merge negative y as source, if can't find path, undo
src_posy =1,  merge positive y as source, if can't find path, undo
src_posy =2,  merge positive y as source, if can't find path, no undo
src_posy =0, not merge y
src_posy =9, choos negative or positive x as source based on number
cost_th,
return: how mange time it adjust edge
*/
int BundleAdjust2::merge_square_area(const Rect & src_rect, const Rect & tgt_rect, const Rect & outer, int src_posx, int src_posy, float cost_th)
{
	//1 init following 4 queue
	CV_Assert(cost_th > COST_BIGER_THAN_AVG - 1);
	vector<pair<unsigned, int> > pos_x, pos_y, neg_x, neg_y; //first is idx, second is res_sft
	UndoPatch patch;
	UndoPatch * p_patch = (abs(src_posx >= 2) || abs(src_posy) >= 2) ? NULL : &patch;
	if (abs(src_posx) == 2)
		src_posx = src_posx / 2;
	if (abs(src_posy) == 2)
		src_posy = src_posy / 2;
	//1.1 search FourCorner res_sft src_rect
	int sumx = 0, sumy = 0, abs_sumx = 0, abs_sumy = 0;
	for (int y = src_rect.y; y < src_rect.y + src_rect.height; y++)
	for (int x = src_rect.x; x < src_rect.x + src_rect.width; x++)  {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		if (pc->res_sft[0] > 0 && src_posy > 0)
			pos_y.push_back(make_pair(pc->idx, pc->res_sft[0]));
		if (pc->res_sft[0] < 0 && src_posy < 0)
			neg_y.push_back(make_pair(pc->idx, -pc->res_sft[0]));
		if (pc->res_sft[1] > 0 && src_posx > 0)
			pos_x.push_back(make_pair(pc->idx, pc->res_sft[1]));
		if (pc->res_sft[1] < 0 && src_posx < 0)
			neg_x.push_back(make_pair(pc->idx, -pc->res_sft[1]));
		sumy += pc->res_sft[0];
		sumx += pc->res_sft[1];
		abs_sumy += abs(pc->res_sft[0]);
		abs_sumx += abs(pc->res_sft[1]);
	}
	qDebug("merge_square_area ir=(x=%d,y=%d,%d,%d), sumx=%d, sumy=%d, absx=%d, absy=%d, posx=%d, posy=%d", src_rect.x, src_rect.y,
		src_rect.x + src_rect.width - 1, src_rect.y + src_rect.height - 1, sumx, sumy, abs_sumx, abs_sumy, src_posx, src_posy);
	//1.2 search FourCorner res_sft tgt_rect
	vector<SourceInfo> source, dest;
	for (int y = tgt_rect.y; y < tgt_rect.y + tgt_rect.height; y++)
	for (int x = tgt_rect.x; x < tgt_rect.x + tgt_rect.width; x++)  {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		if (pc->bd == 0) {
			dest.push_back(SourceInfo(pc->idx, 0, 0, 30000));
			continue;
		}
		if (pc->res_sft[0] > 0 && src_posy < 0)
			pos_y.push_back(make_pair(pc->idx, pc->res_sft[0]));
		if (pc->res_sft[0] < 0 && src_posy > 0)
			neg_y.push_back(make_pair(pc->idx, -pc->res_sft[0]));
		if (pc->res_sft[1] > 0 && src_posx < 0)
			pos_x.push_back(make_pair(pc->idx, pc->res_sft[1]));
		if (pc->res_sft[1] < 0 && src_posx > 0)
			neg_x.push_back(make_pair(pc->idx, -pc->res_sft[1]));
	}
	if (abs_sumx == 0 && abs_sumy == 0) {
		qInfo("merge_square_area abs sumx and abs sumy=0, no need merge");
		return 0;
	}
	//2 init Source Info and dest
	bool has_border = !dest.empty();
	if (abs_sumy != 0 && (!neg_y.empty() && !pos_y.empty() || has_border)) {
		if (src_posy == 9) {//choose source based on number
			if (has_border)
				src_posy = (sumy < 0) ? -1 : 1; //sumy < 0 means pos_y is less, choose neg_y as source, has_border choose more
			else
				src_posy = (sumy < 0) ? 1 : -1; //sumy < 0 means pos_y is less, choose pos_y as source
		}
		if (src_posy == -1) { //merge negative y
			for (int i = 0; i < (int)neg_y.size(); i++)
				source.push_back(SourceInfo(neg_y[i].first, -1, 0, neg_y[i].second));
			for (int i = 0; i < (int)pos_y.size(); i++)
				dest.push_back(SourceInfo(pos_y[i].first, 1, 0, pos_y[i].second));
		}
		if (src_posy == 1) { //merge positive y
			for (int i = 0; i < (int)pos_y.size(); i++)
				source.push_back(SourceInfo(pos_y[i].first, 1, 0, pos_y[i].second));
			for (int i = 0; i < (int)neg_y.size(); i++)
				dest.push_back(SourceInfo(neg_y[i].first, -1, 0, neg_y[i].second));
		}
	}
	if (abs_sumx != 0 && (!neg_x.empty() && !pos_x.empty() || has_border)) {
		if (src_posx == 9) { //choose source based on number
			if (has_border)
				src_posx = (sumx < 0) ? -1 : 1; //sumx < 0 means pos_x is less, choose neg_x as source,has_border choose more
			else
				src_posx = (sumx < 0) ? 1 : -1; //sumx < 0 means pos_x is less, choose pos_x as source
		}
		if (src_posx == -1) { //merge negative x
			for (int i = 0; i < (int)neg_x.size(); i++)
				source.push_back(SourceInfo(neg_x[i].first, -1, 1, neg_x[i].second));
			for (int i = 0; i < (int)pos_x.size(); i++)
				dest.push_back(SourceInfo(pos_x[i].first, 1, 1, pos_x[i].second));
		}
		if (src_posx == 1) { //merge positive x
			for (int i = 0; i < (int)pos_x.size(); i++)
				source.push_back(SourceInfo(pos_x[i].first, 1, 1, pos_x[i].second));
			for (int i = 0; i < (int)neg_x.size(); i++)
				dest.push_back(SourceInfo(neg_x[i].first, -1, 1, neg_x[i].second));
		}
	}
	if (source.empty() || dest.empty()) {
		qInfo("merge_square_area source=%d dest=%d, no need merge", source.size(), dest.size());
		return 0;
	}

	//3 init FourCorner SourceCost inside outer
	queue<unsigned> rq;
	for (int y = outer.y; y < outer.y + outer.height; y++)
	for (int x = outer.x; x < outer.x + outer.width; x++)  {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		CV_Assert(pc->cs.empty());
		pc->cs.resize(source.size());
		pc->already_inqueue = false;
		pc->visited = false;
		for (int i = 0; i < source.size(); i++) {
			pc->cs[i].resize(source[i].cap + 1);
			for (int j = 1; j <= source[i].cap; j++)
			if (source[i].idx != pc->idx)
				pc->cs[i][j].reset(cost_th + 1);
			else {
				pc->cs[i][j].cost = 0;
				pc->cs[i][j].fa = pc->idx;
				pc->cs[i][j].update_num = 0;
				pc->cs[i][j].update_state = 1;
				if (pc->bd != 0 && !pc->already_inqueue) {
					rq.push(pc->idx);
					pc->already_inqueue = true;
				}
			}
		}
	}
	int left_source = (int)source.size();
	int left_dest = (int)dest.size();
	//5 adjust path one by one
	int cnt = 0;
	while (left_source > 0 && left_dest > 0) {
		//5.1 do relax
		for (int y = outer.y; y < outer.y + outer.height; y++)
		for (int x = outer.x; x < outer.x + outer.width; x++)
		for (int i = 0; i < source.size(); i++)
		for (int j = 1; j <= source[i].cap; j++)
			fc[y * (img_num_w + 1) + x].cs[i][j].update_num = 0;
		while (!rq.empty()) {
			FourCorner * pc = get_4corner(rq.front());
			rq.pop();
			relax(pc, outer, rq, source, p_patch, cost_th, false);
		}
		check_relax(outer, source, cost_th);
		//5.2 find min unit cost
		FourCorner * p_best_src = NULL;
		FourCorner *p_best_dst = NULL;
		int best_src, best_dst, best_modify = 0;
		COST_TYPE min_unit_cost = COST_BIND;
		for (int k = 0; k < (int)dest.size(); k++)
		if (dest[k].cap > 0) {
			FourCorner * pc = get_4corner(dest[k].idx);
			for (int i = 0; i < pc->cs.size(); i++)
			if ((source[i].isx == dest[k].isx && source[i].sign == -dest[k].sign)
				|| dest[k].sign == 0) {
				for (int j = min(source[i].cap, dest[k].cap); j>0; j--) {
					COST_TYPE cost = pc->cs[i][j].cost / j;
					if (cost < min_unit_cost && pc->cs[i][j].cost < cost_th) {
						min_unit_cost = cost;
						best_src = i;
						best_dst = k;
						best_modify = j;
						p_best_dst = pc;
						p_best_src = get_4corner(source[i].idx);
					}
				}
			}
		}
		if (min_unit_cost > cost_th) {
			if (p_patch != NULL) {
				cnt = 0;
				qDebug("merge_square_area min_unit_cost=%f undo", min_unit_cost);
				undo(patch);
			}
			break;
		}
		//5.3 adjust
		CV_Assert(p_best_src != NULL);
		dest[best_dst].cap -= best_modify;
		if (dest[best_dst].cap == 0)
			left_dest--;
		source[best_src].cap -= best_modify;
		if (source[best_src].cap == 0)
			left_source--;
		adjust_edge_mls(p_best_src, p_best_dst, best_src, best_modify, rq, source, change_id, p_patch, cost_th, false);
		cnt++;
		check_fix_edge();
	}
	source.clear();
	for (int y = outer.y; y < outer.y + outer.height; y++)
	for (int x = outer.x; x < outer.x + outer.width; x++)  {
		fc[y * (img_num_w + 1) + x].cs.clear();
	}
	return cnt;
}

Bundle BundleAdjust2::search_bundle(FourCorner * pc, int len_limit, int width_limit)
{
	CV_Assert((pc->res_sft[0] != 0 || pc->res_sft[1] != 0) && pc->bd>0);
	int y0 = CORNER_Y(pc->idx);
	int x0 = CORNER_X(pc->idx);

	Bundle best = MAKE_BUNDLE(COST_BIND, 0, 0, BUNDLE_SHAPE_INVALID, 100, 0, 0, 0, 0, 0);
	float best_cost = COST_BIND;
	//Top loop check each shape
	for (int i = 0; i < sizeof(bundle_shape) / sizeof(bundle_shape[0]); i++) {
		if (BUNDLE_QUEUE(best) == 0 && bundle_shape[i].len > 2)
			break;
		if (BUNDLE_QUEUE(best) == 1 && bundle_shape[i].len > 3)
			break;
		if (bundle_shape[i].len > len_limit)
			break;
		int dx = pc->res_sft[1]; //dx = Sum(res_sft[1]) in one bundle
		int dy = pc->res_sft[0]; //dy = Sum(res_sft[0]) in one bundle
		int sum_res_x = abs(pc->res_sft[1]); //Sum(|res_sft[1]|) in one bundle
		int sum_res_y = abs(pc->res_sft[0]); //Sum(|res_sft[0]|) in one bundle
		int xok = 1, yok = 1;
		FourCorner * pc0 = pc;
		//First loop is to check if dx and dy are ok
		for (int j = 0; j < bundle_shape[i].len - 1; j++) {
			int y = y0 + bundle_shape[i].d[j][0];
			int x = x0 + bundle_shape[i].d[j][1];
			FourCorner * pc1 = get_4corner(y, x);
			if (pc1->bd == 0) { //corner in boundary, directly pass
				xok = 0;
				yok = 0;
				break;
			}
			Edge2 * pe = get_edge(pc1, pc0);
			CV_Assert(pe != NULL);
			dx += pc1->res_sft[1];
			dy += pc1->res_sft[0];
			sum_res_x += abs(pc1->res_sft[1]);
			sum_res_y += abs(pc1->res_sft[0]);
			pc0 = pc1;
		}

		if (!xok) //if meet bounder
			dx = 100;
		if (!yok) //if meet bounder
			dy = 100;
		if (abs(dx) > 1)
			xok = 0;
		else
			xok = (dx == 0) ? 2 : 1;
		if (abs(dy) > 1)
			yok = 0;
		else
			yok = (dy == 0) ? 2 : 1;
		if (!xok && !yok)
			continue;
		if (xok && !yok && (abs(dx) == sum_res_x)) //not reduce, pass
			continue;

		if (!xok && yok && (abs(dy) == sum_res_y)) //not reduce, pass
			continue;

		if (xok && yok && (abs(dx) == sum_res_x) && (abs(dy) == sum_res_y))
			continue;

		unsigned queue = queue_number(bundle_shape[i].len, dx, dy); //queue is valid when dx<=1 || dy<=1
		if (queue > BUNDLE_QUEUE(best)) //Already has better answer, direct pass
			continue;
		//second loop compute cost and change_id
		pc0 = pc;
		float cost = 0;
		unsigned max_change_id = pc0->change_id;
		dx = pc->res_sft[1];
		dy = pc->res_sft[0];
		for (int j = 0; j < bundle_shape[i].len - 1; j++) {
			int y = y0 + bundle_shape[i].d[j][0];
			int x = x0 + bundle_shape[i].d[j][1];
			FourCorner * pc1 = get_4corner(y, x);
			max_change_id = max(max_change_id, pc1->change_id);
			Edge2 * pe = get_edge(pc1, pc0);
			CV_Assert(pe != NULL);

			int sign = 1;
			Point newpos;
			Point oldpos = pe->get_current_point(scale);
			if (bundle_shape[i].c[j] == 0) {
				if (CORNER_X(pc0->idx) == CORNER_X(pc1->idx)) {
					if (CORNER_Y(pc0->idx) > CORNER_Y(pc1->idx)) //pc0 is under pc1
						sign = -1;
				}
				else
				if (CORNER_X(pc0->idx) < CORNER_X(pc1->idx)) //pc0 is left pc1
					sign = -1;
				newpos = (pe->idea_pos - pe->diff->offset); //newpos is cost matrix zuobiao
				newpos.x = newpos.x / scale + pe->mls[1] + sign * dx;
				newpos.y = newpos.y / scale + pe->mls[0] + sign * dy;
			}
			else {
				if (CORNER_X(pc0->idx) == CORNER_X(pc1->idx)) {
					if (CORNER_Y(pc1->idx) > CORNER_Y(pc0->idx))
						sign = -1;
				}
				else
				if (CORNER_X(pc1->idx) < CORNER_X(pc0->idx))
					sign = -1;
				newpos = (pe->idea_pos - pe->diff->offset);
				newpos.x = newpos.x / scale + pe->mls[1] + sign * pc1->res_sft[1];
				newpos.y = newpos.y / scale + pe->mls[0] + sign * pc1->res_sft[0];
			}

			if (xok && pe->get_point_cost(Point(newpos.x, oldpos.y), scale) >= COST_BOUNDARY - 1) { //out of cost matrix, fail
				cost += COST_BOUNDARY;
				break;
			}

			if (yok && pe->get_point_cost(Point(oldpos.x, newpos.y), scale) >= COST_BOUNDARY - 1) { //one move out of cost matrix, fail
				cost += COST_BOUNDARY;
				break;
			}

			if ((xok && sum_res_y <= 1 || yok && sum_res_x <= 1) &&
				pe->get_point_cost(newpos, scale) >= COST_BIGER_THAN_AVG - 1) { //Low than average, fail
				cost += COST_BIGER_THAN_AVG;
				break;
			}

			cost += (queue < 6) ? pe->get_point_cost(newpos, scale) : pe->hard_score;

			dx += pc1->res_sft[1];
			dy += pc1->res_sft[0];
			pc0 = pc1;
		}
		CV_Assert(cost < COST_BIND / 2);
		if (cost >= COST_BIGER_THAN_AVG) //if total err bigger than average, fail
			continue;
		if (queue < BUNDLE_QUEUE(best) || cost < best_cost) {
			best_cost = cost;
			best = MAKE_BUNDLE(cost, max_change_id, pc->idx, i, queue, xok, yok, 0, 0, 0);
		}
	}

	if (len_limit > 5 && BUNDLE_SHAPE(best) == BUNDLE_SHAPE_INVALID) {
		vector <int> lsx(width_limit, 0), rsx(width_limit, 0); //left sum and right sum 
		vector <int> lsy(width_limit, 0), rsy(width_limit, 0); //left sum and right sum 
		vector <int> lsabsx(width_limit, 0), rsabsx(width_limit, 0); //left sum abs and right sum abs
		vector <int> lsabsy(width_limit, 0), rsabsy(width_limit, 0); //left sum abs and right sum abs
		vector <int> lsnzx(width_limit, 0), rsnzx(width_limit, 0); //left non-zero number and right non-zero number
		vector <int> lsnzy(width_limit, 0), rsnzy(width_limit, 0); //left non-zero number and right non-zero number
		vector <unsigned> lcid(width_limit, 0), rcid(width_limit, 0); //left change id and right change id
		for (int y = 0; y < width_limit; y++) {
			int sx = 0, sy = 0, sabsx = 0, sabsy = 0, snzx = 0, snzy = 0;
			for (int x = 0; x < width_limit; x++) {
				FourCorner * pc1 = get_4corner(y + y0, x + x0);
				if (pc1 == NULL || pc1->bd == 0) { //bypass boundary directly
					sx += 0x10000;
					sy += 0x10000;
					sabsx += 0x10000;
					sabsy += 0x10000;
				}
				else {
					sx += pc1->res_sft[1]; //Sum(res_sft[1]) from x = 0 to width_limit
					sy += pc1->res_sft[0]; //Sum(res_sft[0]) from x = 0 to width_limit
					sabsx += abs(pc1->res_sft[1]); //Sum(abs(res_sft[1])) from x = 0 to width_limit
					sabsy += abs(pc1->res_sft[0]); //Sum(abs(res_sft[0])) from x = 0 to width_limit
					if (pc1->res_sft[1] != 0) //sum(non_zero(res_sft[1])) from x = 0 to width_limit
						snzx++;
					if (pc1->res_sft[0] != 0) //sum(non_zero(res_sft[0])) from x = 0 to width_limit
						snzy++;
					rcid[x] = max(rcid[x], pc1->change_id);
					if (x > 0)
						rcid[x] = max(rcid[x], rcid[x - 1]);
				}
				rsx[x] += sx; //Sum(res_sft[1]) (0..y) * (0..x)
				rsy[x] += sy; //Sum(res_sft[0]) (0..y) * (0..x)
				rsabsx[x] += sabsx; //Sum(abs(res_sft[1])) (0..y) * (0..x)
				rsabsy[x] += sabsy; //Sum(abs(res_sft[0])) (0..y) * (0..x)
				rsnzx[x] += snzx; //Sum(non_zero(res_sft[1])) (0..y) * (0..x)
				rsnzy[x] += snzy; //Sum(non_zero(res_sft[0])) (0..y) * (0..x)
			}
			sx = 0, sy = 0, sabsx = 0, sabsy = 0, snzx = 0, snzy = 0;
			for (int x = 1; x < width_limit; x++) {
				FourCorner * pc1 = get_4corner(y + y0, x0 - x);
				if (pc1 == NULL || pc1->bd == 0) {
					sx += 0x10000;
					sy += 0x10000;
					sabsx += 0x10000;
					sabsy += 0x10000;
				}
				else {
					sx += pc1->res_sft[1]; //Sum(res_sft[1]) from x = -1 to -width_limit
					sy += pc1->res_sft[0]; //Sum(res_sft[0]) from x = -1 to -width_limit
					sabsx += abs(pc1->res_sft[1]); //Sum(abs(res_sft[1])) from x = -1 to -width_limit
					sabsy += abs(pc1->res_sft[0]); //Sum(abs(res_sft[0])) from x = -1 to -width_limit
					if (pc1->res_sft[1] != 0) //sum(non_zero(res_sft[1])) from x = 0 to width_limit
						snzx++;
					if (pc1->res_sft[0] != 0) //sum(non_zero(res_sft[0])) from x = 0 to width_limit
						snzy++;
					lcid[x] = max(lcid[x], pc1->change_id);
					lcid[x] = max(lcid[x], lcid[x - 1]);
				}
				lsx[x] += sx; //Sum(res_sft[1]) (0..y) * (-1..-x)
				lsy[x] += sy; //Sum(res_sft[0]) (0..y) * (-1..-x)
				lsabsx[x] += sabsx; //Sum(abs(res_sft[1])) (0..y) * (-1..-x)
				lsabsy[x] += sabsy; //Sum(abs(res_sft[0])) (0..y) * (-1..-x)
				lsnzx[x] += snzx; //Sum(non_zero(res_sft[1])) (0..y) * (-1..-x)
				lsnzy[x] += snzy; //Sum(non_zero(res_sft[0])) (0..y) * (-1..-x)
			}
			if (y == 0) {
				for (int x = 1; x < width_limit; x++)
				if (lsabsx[x] || lsabsy[x]) { //avoid search same bundle
					lsx[x] += 0x10000;
					lsy[x] += 0x10000;
				}
			}

			for (int x1 = 0; x1 < width_limit; x1++)
			for (int x2 = 0; x2 < width_limit; x2++) {
				if (x1 + x2 + 1 > width_limit || (x1 + x2 + 1)*(y + 1) > len_limit)
					break;
				if ((x1 + x2 + 1)*(y + 1) <= 3)
					continue;
				int dx = lsx[x1] + rsx[x2]; //Sum(res_sft[1]) (0..y) * (-x1..x2)
				int dy = lsy[x1] + rsy[x2]; //Sum(res_sft[0]) (0..y) * (-x1..x2)
				if (dx > 0x8000) //(0..y) * (-x1..x2) contain invalid FourCorner
					break;
				int sum_res_x = lsabsx[x1] + rsabsx[x2]; //Sum(abs(res_sft[1])) (0..y) * (-x1..x2)
				int sum_res_y = lsabsy[x1] + rsabsy[x2]; //Sum(abs(res_sft[0])) (0..y) * (-x1..x2)
				unsigned max_change_id = max(lcid[x1], rcid[x2]);
				int xok, yok;
				if (abs(dx) > 1)
					xok = 0;
				else
					xok = (dx == 0) ? 2 : 1;
				if (abs(dy) > 1)
					yok = 0;
				else
					yok = (dy == 0) ? 2 : 1;
				if (!xok && !yok)
					continue;
				if (xok && !yok && (abs(dx) == sum_res_x)) //not reduce, pass
					continue;
				if (!xok && yok && (abs(dy) == sum_res_y)) //not reduce, pass
					continue;
				if (xok && yok && (abs(dx) == sum_res_x) && (abs(dy) == sum_res_y))
					continue;
				unsigned queue = queue_number((x1 + x2 + 1)*(y + 1), dx, dy); //queue is valid when dx<=1 || dy<=1
				if (queue > BUNDLE_QUEUE(best)) //Already has better answer, direct pass
					continue;
				unsigned nonzero = (xok && yok) ? max(lsnzx[x1] + rsnzx[x2], lsnzy[x1] + rsnzy[x2]) :
					(xok ? lsnzx[x1] + rsnzx[x2] : lsnzy[x1] + rsnzy[x2]);
				unsigned cost = nonzero * 5 + (x1 + x2 + 1)*(y + 1);
				best = MAKE_BUNDLE(cost, max_change_id, pc->idx, BUNDLE_SHAPE_SQUARE, queue, xok, yok, x1, x2, y);
			}
		}
	}
	return best;
}

bool BundleAdjust2::merge_one_bundle(Bundle b)
{
	FourCorner * pc = get_4corner(BUNDLE_CORNER_IDX(b));
	int y0 = CORNER_Y(pc->idx);
	int x0 = CORNER_X(pc->idx);
	int xok = BUNDLE_XOK(b);
	int yok = BUNDLE_YOK(b);
	int shape = BUNDLE_SHAPE(b);
	int dx = pc->res_sft[1];
	int dy = pc->res_sft[0];
	FourCorner * pc0 = pc;
	if (pc0->change_id > BUNDLE_CHANGE_ID(b))
		return false;
	if (shape == BUNDLE_SHAPE_SQUARE) {
		int lx = BUNDLE_SX0(b);
		int rx = BUNDLE_SX1(b);
		int by = BUNDLE_SY(b);
		Rect r(x0 - lx, y0, lx + rx + 1, by + 1);
		for (int y = r.y; y < r.y + r.height; y++)
		for (int x = r.x; x < r.x + r.width; x++) {
			if (fc[y * (img_num_w + 1) + x].change_id > BUNDLE_CHANGE_ID(b))
				return false;
		}
		return merge_square_area(r, r, r, (xok > 0) ? 1 : 0, (yok > 0) ? 1 : 0, COST_BIGER_THAN_AVG);
	}

	//first loop is for Assert and check change id
	for (int j = 0; j < bundle_shape[shape].len - 1; j++) {
		int y = y0 + bundle_shape[shape].d[j][0];
		int x = x0 + bundle_shape[shape].d[j][1];
		FourCorner * pc1 = get_4corner(y, x);
		CV_Assert(pc1->bd != 0);
		Edge2 * pe = get_edge(pc1, pc0);
		CV_Assert(pe != NULL);
		dx += pc1->res_sft[1];
		dy += pc1->res_sft[0];
		pc0 = pc1;
		if (pc0->change_id > BUNDLE_CHANGE_ID(b)) //b is expire
			return false;
	}
	CV_Assert(dx == 0 && xok == 2 || xok == 1 && (abs(dx) == 1) || xok == 0);
	CV_Assert(dy == 0 && yok == 2 || yok == 1 && (abs(dy) == 1) || yok == 0);
	//second loop update mls
	pc0 = pc;
	dx = pc->res_sft[1];
	dy = pc->res_sft[0];
	pc0->change_id = change_id;
	for (int j = 0; j < bundle_shape[shape].len - 1; j++) {
		int y = y0 + bundle_shape[shape].d[j][0];
		int x = x0 + bundle_shape[shape].d[j][1];
		FourCorner * pc1 = get_4corner(y, x);
		Edge2 * pe = get_edge(pc1, pc0);
		int sign = 1;
		if (bundle_shape[shape].c[j] == 0) {
			if (CORNER_X(pc0->idx) == CORNER_X(pc1->idx)) {
				if (CORNER_Y(pc0->idx) > CORNER_Y(pc1->idx))
					sign = -1;
			}
			else
			if (CORNER_X(pc0->idx) < CORNER_X(pc1->idx))
				sign = -1;
			if (xok)
				pe->mls[1] += sign * dx;
			if (yok)
				pe->mls[0] += sign * dy;
		}
		else {
			if (CORNER_X(pc0->idx) == CORNER_X(pc1->idx)) {
				if (CORNER_Y(pc1->idx) > CORNER_Y(pc0->idx)) {
					sign = -1;
					pc0 = get_4corner(pc1->get_4corner_idx(DIR_UP));
				}
				else
					pc0 = get_4corner(pc1->get_4corner_idx(DIR_DOWN));
			}
			else
			if (CORNER_X(pc1->idx) < CORNER_X(pc0->idx)) {
				sign = -1;
				pc0 = get_4corner(pc1->get_4corner_idx(DIR_RIGHT));
			}
			else
				pc0 = get_4corner(pc1->get_4corner_idx(DIR_LEFT));
			if (xok)
				pe->mls[1] += sign * pc1->res_sft[1];
			if (yok)
				pe->mls[0] += sign * pc1->res_sft[0];
		}
		dx += pc1->res_sft[1];
		dy += pc1->res_sft[0];
		if (xok) {
			if (pc0->res_sft[1] != 0 && pc0->change_x)
				pc0->change_x = change_id;
			if (pc1->res_sft[1] != 0 && pc1->change_x)
				pc1->change_x = change_id;
		}
		if (yok) {
			if (pc0->res_sft[0] != 0 && pc0->change_y)
				pc0->change_y = change_id;
			if (pc1->res_sft[0] != 0 && pc1->change_y)
				pc1->change_y = change_id;
		}
		if (bundle_shape[shape].c[j] == 0) {
			if (xok) {
				pc1->res_sft[1] = dx;
				pc0->res_sft[1] = 0;
			}
			if (yok) {
				pc1->res_sft[0] = dy;
				pc0->res_sft[0] = 0;
			}
		}
		else {
			if (xok) {
				pc0->res_sft[1] = dx;
				pc1->res_sft[1] = 0;
			}
			if (yok) {
				pc0->res_sft[0] = dy;
				pc1->res_sft[0] = 0;
			}
		}
		pc0 = pc1;
		pc0->change_id = change_id;
	}
	int y = y0 + bundle_shape[shape].d[bundle_shape[shape].left - 1][0];
	int x = x0 + bundle_shape[shape].d[bundle_shape[shape].left - 1][1];
	FourCorner * pc1 = get_4corner(y, x);
	if (xok)
		CV_Assert(pc1->res_sft[1] == dx);
	if (yok)
		CV_Assert(pc1->res_sft[0] == dy);
	return true;
}

void BundleAdjust2::merge_bundles()
{
	change_id = 1;
	for (int i = 0; i < 16; i++) {
		priority_queue<Bundle, vector<Bundle>, greater<Bundle> > bundle_queue[16];
		int change_queue[sizeof(bundle_queue) / sizeof(bundle_queue[0])] = { 0 };
		int prev_change_id = change_id;
		unsigned exp_queue, len_limit;
		switch (i) {
		case 0:
			exp_queue = 0;
			len_limit = 2;
			break;
		case 1:
			exp_queue = 1;
			len_limit = 3;
			break;
		case 2:
			exp_queue = 2;
			len_limit = 4;
			break;
		case 3:
			exp_queue = 3;
			len_limit = 4;
			break;
		case 4:
			exp_queue = 4;
			len_limit = 4;
			break;
		case 5:
			exp_queue = 5;
			len_limit = 4;
			break;
		case 6:
			exp_queue = 5;
			len_limit = 5;
			break;
		case 7:
			exp_queue = 4;
			len_limit = 8;
			break;
		case 8:
			exp_queue = 5;
			len_limit = 8;
			break;
		case 9:
			exp_queue = 9;
			len_limit = 8;
			break;
		case 10:
			exp_queue = 10;
			len_limit = 8;
			break;
		case 11:
			exp_queue = 13;
			len_limit = 8;
			break;
		case 12:
			exp_queue = 15;
			len_limit = 10;
			break;
		case 13:
			exp_queue = 15;
			len_limit = 16;
			break;
		case 14:
			exp_queue = 15;
			len_limit = 36;
			break;
		case 15:
			exp_queue = 15;
			len_limit = 64;
			break;
		default:
			exp_queue = 15;
			len_limit = 100;
			break;
		}
		qInfo("Merge_bundles %d round, change_id=%d, queue_limit=%d, len_limit=%d", i, change_id, exp_queue, len_limit);
		print_4corner_stat();
		for (int y = 0; y <= img_num_h; y++)
		for (int x = 0; x <= img_num_w; x++) {
			FourCorner * pc = &fc[y * (img_num_w + 1) + x];
			if ((pc->res_sft[0] != 0 || pc->res_sft[1] != 0) && pc->bd >0) {
				Bundle b = search_bundle(pc, len_limit, BUNDLE_SHAPE_WIDTH);
				if (BUNDLE_QUEUE(b) <= exp_queue)
					bundle_queue[BUNDLE_QUEUE(b)].push(b);
			}
		}
		qInfo("Ready 0..7: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d", bundle_queue[0].size(), bundle_queue[1].size(),
			bundle_queue[2].size(), bundle_queue[3].size(), bundle_queue[4].size(),
			bundle_queue[5].size(), bundle_queue[6].size(), bundle_queue[7].size());
		qInfo("Ready 8.15: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d", bundle_queue[8].size(), bundle_queue[9].size(),
			bundle_queue[10].size(), bundle_queue[11].size(), bundle_queue[12].size(),
			bundle_queue[13].size(), bundle_queue[14].size(), bundle_queue[15].size());

		for (int j = 0; j < sizeof(bundle_queue) / sizeof(bundle_queue[0]); j++)
		while (!bundle_queue[j].empty()) {
			Bundle b = bundle_queue[j].top();
			bundle_queue[j].pop();
			if (merge_one_bundle(b)) {
				change_id++;
				change_queue[j]++;
			}
		}
		qInfo("Merge 0..7: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d", change_queue[0], change_queue[1],
			change_queue[2], change_queue[3], change_queue[4],
			change_queue[5], change_queue[6], change_queue[7]);
		qInfo("Merge 8.15: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d", change_queue[8], change_queue[9],
			change_queue[10], change_queue[11], change_queue[12],
			change_queue[13], change_queue[14], change_queue[15]);
		if (change_id == prev_change_id && i > 15)
			break;
		check_fix_edge();
	}
	print_4corner_stat();
}

struct FourCornerBdStat {
	int sum_posx, sum_posy, sum_negx, sum_negy;
	FourCornerBdStat() {
		sum_posx = 0;
		sum_negx = 0;
		sum_posy = 0;
		sum_negy = 0;
	}
};
/*
void BundleAdjust2::merge_all()
{
int prev_src_bd = -1;
float cost_th;
while (1) {
vector<FourCornerBdStat> fcbs;
fcbs.resize(min(img_num_h / 2 + 1, img_num_w / 2 + 1));
int ag_sum_posx = 0, ag_sum_negx = 0, ag_sum_posy = 0, ag_sum_negy = 0;
for (int i = 0; i < fc.size(); i++)
if ((fc[i].res_sft[0] != 0 || fc[i].res_sft[1] != 0) && fc[i].bd > 0) {
int x = CORNER_X(fc[i].idx);
int y = CORNER_Y(fc[i].idx);
int bd = min(min(x, y), min(img_num_w - x, img_num_h - y));
if (fc[i].res_sft[1] > 0)
fcbs[bd].sum_posx += fc[i].res_sft[1];
else
fcbs[bd].sum_negx += -fc[i].res_sft[1];
if (fc[i].res_sft[0] > 0)
fcbs[bd].sum_posy += fc[i].res_sft[0];
else
fcbs[bd].sum_negy += -fc[i].res_sft[0];
}

int src_bd = 1, dst_bd = 0;
for (int i = (int) fcbs.size() - 1; i > 0; i--) {
ag_sum_posx += fcbs[i].sum_posx;
ag_sum_negx += fcbs[i].sum_negx;
ag_sum_posy += fcbs[i].sum_posy;
ag_sum_negy += fcbs[i].sum_negy;
if (max(ag_sum_posx, ag_sum_negx) + max(ag_sum_posy, ag_sum_negy) > MERGE_ALL_SUM_TH) {
src_bd = i;
break;
}
}
int src_posx = (ag_sum_posx > ag_sum_negx) ? 2 : -2;
int src_posy = (ag_sum_posy > ag_sum_negy) ? 2 : -2;
int ag_sum = max(ag_sum_posx, ag_sum_negx) + max(ag_sum_posy, ag_sum_negy);
if (ag_sum == 0)
return;
for (int i = src_bd - 1; i > 0; i--) {
ag_sum_posx += fcbs[i].sum_posx;
ag_sum_negx += fcbs[i].sum_negx;
ag_sum_posy += fcbs[i].sum_posy;
ag_sum_negy += fcbs[i].sum_negy;
if (min(ag_sum_posx, ag_sum_negx) + min(ag_sum_posy, ag_sum_negy) > ag_sum) {
dst_bd = i;
break;
}
}

if (prev_src_bd == src_bd) {
dst_bd = max(dst_bd - 1, 0);
cost_th = cost_th * 3;
if (cost_th > 10 * COST_BIGER_THAN_AVG)
return;
}
else
cost_th = COST_BIGER_THAN_AVG;
qInfo("merge_all, src_bd=%d, dst_bd=%d, ag_sum=%d, cost_th=%f", src_bd, dst_bd, ag_sum, cost_th);
Rect src_rect(src_bd, src_bd, img_num_w - 2 * src_bd + 1, img_num_h - 2 * src_bd + 1);
Rect tgt_rect(dst_bd, dst_bd, img_num_w - 2 * dst_bd + 1, img_num_h - 2 * dst_bd + 1);
int cnt = merge_square_area(src_rect, tgt_rect, tgt_rect, src_posx, src_posy, cost_th);
if (!cnt)
change_id++;
prev_src_bd = src_bd;
}
}*/

void BundleAdjust2::merge_all()
{
	int unit[] = { MERGE_ALL_GRID0, MERGE_ALL_GRID1, MERGE_ALL_GRID2 };
	for (int i = 0; i < sizeof(unit) / sizeof(unit[0]); i++) {
		vector<int> gy, gx;
		vector<MergeSquarePara> ms;
		qInfo("merge_all unit%d=%d", i, unit[i]);
		split(img_num_h + 1, unit[i], gy);
		split(img_num_w + 1, unit[i], gx);
		for (int y = 0; y < gy.size() - 1; y++)
		for (int x = 0; x < gx.size() - 1; x++)
			ms.push_back(MergeSquarePara(this, Rect(gx[x] + 1, gy[y] + 1, gx[x + 1] - gx[x] - 1, gy[y + 1] - gy[y] - 1), COST_BIGER_THAN_AVG * 1.5));
#if PARALLEL
		QtConcurrent::blockingMap<vector<MergeSquarePara> >(ms, thread_merge_square);
#else
		for (int j = 0; j < ms.size(); j++)
			thread_merge_square(ms[j]);
#endif
		change_id++;
	}
}

void BundleAdjust2::optimize_corner(unsigned corner_idx, int max_shift_x, int max_shift_y)
{
	qInfo("optimize_corner idx=%x, shift_x=%d, shifty=%d", corner_idx, max_shift_x, max_shift_y);
	//1 init Source Info and dest
	vector<SourceInfo> source;
	source.push_back(SourceInfo(corner_idx, -1, 0, max_shift_y));
	source.push_back(SourceInfo(corner_idx, 1, 0, max_shift_y));
	source.push_back(SourceInfo(corner_idx, -1, 1, max_shift_x));
	source.push_back(SourceInfo(corner_idx, 1, 1, max_shift_x));
	queue<unsigned> rq;

	//2 init FourCorner SourceCost
	for (int k = 0; k < fc.size(); k++) {
		FourCorner * pc = &fc[k];
		pc->cs.resize(source.size());
		pc->already_inqueue = false;
		pc->visited = false;
		for (int i = 0; i < source.size(); i++) {
			pc->cs[i].resize(source[i].cap + 1);
			for (int j = 1; j <= source[i].cap; j++)
			if (pc->idx == corner_idx) {
				pc->cs[i][j].cost = 0;
				pc->cs[i][j].fa = pc->idx;
				pc->cs[i][j].update_num = 0;
				pc->cs[i][j].update_state = 1;
				CV_Assert(pc->bd != 0);
				if (!pc->already_inqueue) {
					rq.push(pc->idx);
					pc->already_inqueue = true;
				}
			}
			else
				pc->cs[i][j].reset(COST_BIND * 10);
		}
	}
	Rect outer(0, 0, img_num_w + 1, img_num_h + 1);
	//3 search loop
	while (!rq.empty()) {
		FourCorner * pc = get_4corner(rq.front());
		rq.pop();
		relax(pc, outer, rq, source, NULL, COST_BIND * 10, false);
	}
	check_relax(outer, source, COST_BIND * 10);

	//4 clear
	source.clear();
	for (int y = outer.y; y < outer.y + outer.height; y++)
	for (int x = outer.x; x < outer.x + outer.width; x++)  {
		fc[y * (img_num_w + 1) + x].cs.clear();
	}
}

void BundleAdjust2::optimize_corner(int max_shift_x, int max_shift_y)
{
	vector<SourceInfo> source, dest;
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w; x++) {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		if (pc->res_sft[0] != 0) {
			CV_Assert(pc->bd);
			max_shift_y = max(max_shift_y, abs(pc->res_sft[0]));
			dest.push_back(SourceInfo(pc->idx, SGN(pc->res_sft[0]), 0, abs(pc->res_sft[0])));
		}
		if (pc->res_sft[1] != 0) {
			CV_Assert(pc->bd);
			max_shift_x = max(max_shift_x, abs(pc->res_sft[1]));
			dest.push_back(SourceInfo(pc->idx, SGN(pc->res_sft[1]), 1, abs(pc->res_sft[1])));
		}
	}
	qInfo("optimize_corner max_shift_x=%d, max_shifty=%d, dest_num=%d", max_shift_x, max_shift_y, dest.size());

	//1 init Source Info and dest
	source.push_back(SourceInfo(0, -1, 0, max_shift_y));
	source.push_back(SourceInfo(0, 1, 0, max_shift_y));
	source.push_back(SourceInfo(0, -1, 1, max_shift_x));
	source.push_back(SourceInfo(0, 1, 1, max_shift_x));
	queue<unsigned> rq;

	//2 init FourCorner SourceCost
	for (int k = 0; k < fc.size(); k++) {
		FourCorner * pc = &fc[k];
		pc->cs.resize(source.size());
		pc->already_inqueue = false;
		pc->visited = false;
		for (int i = 0; i < source.size(); i++) {
			pc->cs[i].resize(source[i].cap + 1);
			for (int j = 1; j <= source[i].cap; j++)
			if (pc->bd == 0) {
				pc->cs[i][j].cost = 0;
				pc->cs[i][j].fa = 0; //note set it to 0
				pc->cs[i][j].update_num = 0;
				pc->cs[i][j].update_state = 1;
				if (!pc->already_inqueue) {
					rq.push(pc->idx);
					pc->already_inqueue = true;
				}
			}
			else
				pc->cs[i][j].reset(COST_BIND * 10);
		}
	}

	Rect outer(0, 0, img_num_w + 1, img_num_h + 1);
	bool res_all0 = dest.empty();
	//3 search loop
	while (1) {
		//3.1 do relax
		for (int y = outer.y; y < outer.y + outer.height; y++)
		for (int x = outer.x; x < outer.x + outer.width; x++)
		for (int i = 0; i < source.size(); i++)
		for (int j = 1; j <= source[i].cap; j++)
			fc[y * (img_num_w + 1) + x].cs[i][j].update_num = 0;
		while (!rq.empty()) {
			FourCorner * pc = get_4corner(rq.front());
			rq.pop();
			relax(pc, outer, rq, source, NULL, COST_BIND * 10, true);
		}
		check_relax(outer, source, COST_BIND * 10);

		//3.2 merge all non-zero pc
		if (!res_all0) {
			COST_TYPE min_unit_cost = COST_BIND;
			FourCorner * p_best = NULL;
			int best_source, best_dst, best_modify = 0;
			for (int k = 0; k < (int)dest.size(); k++)
			if (dest[k].cap > 0) {
				FourCorner * pc = get_4corner(dest[k].idx);
				for (int i = 0; i < pc->cs.size(); i++) //only one i will be selected
				if ((source[i].isx == dest[k].isx && source[i].sign == -dest[k].sign)) {
					for (int j = dest[k].cap; j > 0; j--) {
						COST_TYPE cost = pc->cs[i][j].cost / j;
						if (cost < min_unit_cost && pc->cs[i][j].cost < COST_BIND - 1) {
							min_unit_cost = cost;
							p_best = pc;
							best_source = i;
							best_dst = k;
							best_modify = j;
						}
					}
					break;
				}
			}
			if (p_best != NULL) {
				dest[best_dst].cap -= best_modify;
				FourCorner * p_src = p_best;
				while (p_src->bd) //find the border
					p_src = get_4corner(p_src->cs[best_source][best_modify].fa);
				adjust_edge_mls(p_src, p_best, best_source, best_modify, rq, source, change_id, NULL, COST_BIND * 10, true);
				change_id++;
				check_fix_edge();
				continue;
			}
			else {
				qInfo("optimize_corner finish merge non-zero");
				res_all0 = true;
			}
		}

		//3.3 find border cost-negative loop
		COST_TYPE min_loop_cost = -0.00001;
		FourCorner * p_best = NULL;
		int best_source, best_modify = 0;
		for (int k = 0; k < fc.size(); k++) {
			FourCorner * pc = &fc[k];
			if (pc->bd == 1) {
				for (int i = 0; i < source.size(); i += 2)
				for (int j = 1; j <= source[i].cap; j++)
				if (pc->cs[i][j].cost + pc->cs[i + 1][j].cost < min_loop_cost) {
					FourCorner * p_fa0 = get_4corner(pc->cs[i][j].fa);
					FourCorner * p_fa1 = get_4corner(pc->cs[i + 1][j].fa);
					if (p_fa0->bd && p_fa1->bd || p_fa0 == p_fa1)
						continue;
					min_loop_cost = pc->cs[i][j].cost + pc->cs[i + 1][j].cost;
					best_source = i;
					best_modify = j;
					p_best = pc;
				}
			}
		}

		if (p_best == NULL) //no negative loop, return
			break;

		FourCorner * p_src[2];
		int len[2] = { 0, 0 };
		for (int i = 0; i < 2; i++) {
			p_src[i] = p_best;
			while (p_src[i]->bd) { //find the border
				p_src[i] = get_4corner(p_src[i]->cs[best_source + i][best_modify].fa);
				len[i]++;
			}
		}
		CV_Assert(len[0] == 1 || len[1] == 1);
		int cho = (len[0] == 1) ? 1 : 0;
		p_src[1 - cho]->cs[best_source + cho][best_modify].fa = p_best->idx;
		qInfo("optimize_corner corner=0x%x, shift%c %d, cost=%f", p_best->idx, best_source ? 'x' : 'y', cho ? best_modify : -best_modify, min_loop_cost);
		adjust_edge_mls(p_src[cho], p_src[1 - cho], best_source + cho, best_modify, rq, source, change_id, NULL, COST_BIND * 10, true);
		p_src[1 - cho]->cs[best_source + cho][best_modify].fa = 0;
		change_id++;
		check_fix_edge();
	}

	//4 clear
	source.clear();
	for (int y = outer.y; y < outer.y + outer.height; y++)
	for (int x = outer.x; x < outer.x + outer.width; x++)  {
		fc[y * (img_num_w + 1) + x].cs.clear();
	}
}

void BundleAdjust2::output()
{
	//check all 4corner satisfy res_sft==0
	int abs_err = 0;
	Mat_<int> sft(img_num_h, img_num_w);
	sft = 0;
	for (int j = 0; j < fc.size(); j++)
	if (fc[j].bd > 0) {
		Point res_sft(0, 0);
		int bind_mask = 0;
		for (int dir = 0; dir < 4; dir++) {
			Edge2 * pe = get_edge(fc[j].get_edge_idx(dir));
			CV_Assert(pe != NULL);
			if (dir < 2)
				res_sft += pe->idea_pos + Point(pe->mls[1], pe->mls[0]) * scale;
			else
				res_sft -= pe->idea_pos + Point(pe->mls[1], pe->mls[0]) * scale;
			if (FIX_EDGE_ISBIND(pe->flagb)) {
				if (!FIX_EDGE_BINDX(pe->flagb))
					bind_mask |= BIND_X_MASK;
				if (!FIX_EDGE_BINDY(pe->flagb))
					bind_mask |= BIND_Y_MASK;
			}		
		}
		CV_Assert(res_sft.x == fc[j].res_sft[1] * scale && res_sft.y == fc[j].res_sft[0] * scale);
		int y = CORNER_Y(fc[j].idx);
		int x = CORNER_X(fc[j].idx);
		if (bind_mask & BIND_X_MASK) { //has at least one green fix x
			corner_info(y, x) &= 0xffff0000; //overwrite with res_sft.x
			corner_info(y, x) |= res_sft.x & 0xffff; 
		}
		if (bind_mask & BIND_Y_MASK) {//has at least one green fix y
			corner_info(y, x) &= 0x0000ffff; //overwrite with res_sft.y
			corner_info(y, x) |= res_sft.y << 16;
		}
		corner_info(y, x) &= 0xffffffff;
		sft(CORNER_Y(fc[j].idx), CORNER_X(fc[j].idx)) = min(2, abs(res_sft.x) + abs(res_sft.y));
		abs_err += abs(res_sft.x) + abs(res_sft.y);
	}
	qInfo("BundleAdjust2 Ouput err=%d", abs_err);
	//compute need_visit, 0 means no need, 1 means need, 2 means already visit
	CV_Assert(corner_info.rows == img_num_h && corner_info.cols == img_num_w);

	//1 compute each edge cost
	for (int compute_dir = 0; compute_dir < 2; compute_dir++) {
		vector<pair<float, unsigned>> edge_cost;
		for (int i = 0; i < 2; i++)
		for (int j = 0; j < (int)eds[i].size(); j++) {
			Point cur_edge_point = eds[i][j].get_current_point(scale);
			float cur_edge_cost = eds[i][j].get_point_cost(cur_edge_point, scale);
			float cur_hard_score = eds[i][j].hard_score;
			int bind = (compute_dir == DIR_UP) ? FIX_EDGE_BINDY(eds[i][j].flagb) : FIX_EDGE_BINDX(eds[i][j].flagb);
			if (bind)
				cur_hard_score = COST_BOUNDARY; //for red fix edge, sort it in front of edge_cost
			else {
				if (eds[i][j].diff->img_num == 0)
					cur_hard_score = -COST_BOUNDARY; //for black image, sort it at back of edge_cost
				else
				if (FIX_EDGE_ISBIND(eds[i][j].flagb) && !bind)
					cur_hard_score = -COST_BOUNDARY; //for green fix edge, sort it at back of edge_cost
			}
			cur_edge_cost -= cur_hard_score;
			edge_cost.push_back(make_pair(cur_edge_cost, eds[i][j].diff->edge_idx));
		}
		//sort edge_cost from small to big
		sort(edge_cost.begin(), edge_cost.end());
		vector<int> imgfa(img_num_h * img_num_w);
#define IMGFA(img_idx) imgfa[IMG_Y(img_idx) * img_num_w + IMG_X(img_idx)]
#define IMGROOT(img_idx, img_root) do {\
		img_root = img_idx; \
		while (1) {		\
		unsigned fa = IMGFA(img_root); \
		if (fa == img_root) \
		break; \
		else \
		img_root = fa; \
		}} while (0)

		for (int y = 0; y < img_num_h; y++)
		for (int x = 0; x < img_num_w; x++)
			imgfa[y * img_num_w + x] = MAKE_IMG_IDX(x, y);

		//2 pick img_num_h * img_num_w -1 edge which satisfy lowest cost and no loop, like minimum span-tree
		int connect_num = 0;
		double total_cost = 0;
		int check_state = 0; //only for check print, red fix edge is in front of normal edge, normal edge is in front of green fix edge
		for (int i = 0; i < (int)edge_cost.size(); i++) {
			Edge2 * pe = get_edge(edge_cost[i].second);
			int bind = (compute_dir == DIR_UP) ? FIX_EDGE_BINDY(pe->flagb) : FIX_EDGE_BINDX(pe->flagb);
			switch (check_state) {
			case 0: //0 means red fix edge
				if (FIX_EDGE_ISBIND(pe->flagb) == 0)
					check_state = 1;
				break;
			case 1:
				if (bind)
					qCritical("Red fix edge cost=%f is behind normal edge.", edge_cost[i].first);
				if (FIX_EDGE_ISBIND(pe->flagb) && !bind)
					check_state = 2;
				break;
			case 2:
				if (FIX_EDGE_ISBIND(pe->flagb) == 0 || bind)
					qCritical("normal edge cost=%f is behind green fix", edge_cost[i].first);
				break;
			}
			unsigned img0, img1, img0_root, img1_root;
			pe->diff->get_img_idx(img0, img1);
			IMGROOT(img0, img0_root); //find root to check if img0 and img1 belong to same root
			IMGROOT(img1, img1_root);
			if (img0_root == img1_root) //img0 and img1 belong to different set
				continue;
			else { //pick pe, and bind img0 and img1 to same set
				total_cost += edge_cost[i].first;
				pe->flagb |= 0x80000000;
				if (img0_root < img1_root)
					IMGFA(img1_root) = img0_root;
				else
					IMGFA(img0_root) = img1_root;
				connect_num++;
				if (connect_num == img_num_h * img_num_w - 1) {
					CV_Assert(img0_root == 0 || img1_root == 0);
					break;
				}
			}
		}
		qInfo("BundleAdjust2 total_cost=%f", total_cost / 32);
		//3 connect all selected edge
		vector<unsigned> search_queue;
		search_queue.push_back(0);
		while (!search_queue.empty()) {
			int idx = search_queue.back();
			search_queue.pop_back();
			connect_num--;
			int y0 = IMG_Y(idx);
			int x0 = IMG_X(idx);
			for (int dir = 0; dir < 4; dir++) {
				int y = y0 + dxy[dir][0];
				int x = x0 + dxy[dir][1];
				if (x >= 0 && y >= 0 && x < img_num_w && y < img_num_h) {
					Edge2 * pe;
					Point offset;
					switch (dir) {
					case DIR_UP:
						pe = get_edge(0, y0 - 1, x0);
						if (pe->diff->img_num == 0) //black edge
							offset = -pe->idea_pos - Point(pe->diff->dif.cols / 2, pe->diff->dif.rows / 2) * scale;
						else
							offset = -pe->idea_pos - Point(pe->mls[1], pe->mls[0]) * scale;
						break;
					case DIR_DOWN:
						pe = get_edge(0, y0, x0);
						if (pe->diff->img_num == 0)
							offset = pe->idea_pos + Point(pe->diff->dif.cols / 2, pe->diff->dif.rows / 2) * scale;
						else
							offset = pe->idea_pos + Point(pe->mls[1], pe->mls[0]) * scale;
						break;
					case DIR_LEFT:
						pe = get_edge(1, y0, x0 - 1);
						if (pe->diff->img_num == 0)
							offset = -pe->idea_pos - Point(pe->diff->dif.cols / 2, pe->diff->dif.rows / 2) * scale;
						else
							offset = -pe->idea_pos - Point(pe->mls[1], pe->mls[0]) * scale;
						break;
					case DIR_RIGHT:
						pe = get_edge(1, y0, x0);
						if (pe->diff->img_num == 0)
							offset = pe->idea_pos + Point(pe->diff->dif.cols / 2, pe->diff->dif.rows / 2) * scale;
						else
							offset = pe->idea_pos + Point(pe->mls[1], pe->mls[0]) * scale;
						break;
					}
					if (pe->flagb & 0x80000000) {
						pe->flagb &= 0x7fffffff;						
						search_queue.push_back(MAKE_IMG_IDX(x, y));
						if (compute_dir == DIR_UP) {
							offset.y += best_offset(y0, x0)[0];
							best_offset(y, x)[0] = offset.y;
						}
						else {
							offset.x += best_offset(y0, x0)[1];
							best_offset(y, x)[1] = offset.x;
						}						
					}
				}
			}
		}
		CV_Assert(connect_num == -1);
		for (int i = 0; i < 2; i++)
		for (int j = 0; j < eds[i].size(); j++)
			CV_Assert((eds[i][j].flagb & 0x80000000) == 0);
	}
	//4 compute cost
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w; x++) {
		FourCorner * pc = get_4corner(y, x);
		unsigned long long cost = max(pc->change_x, pc->change_y);
		short val0, val1;
		val0 = corner_info(y, x) & 0xffff;
		val1 = corner_info(y, x) >> 16 & 0xffff;
		int res = max(abs(val0) / scale, abs(val1) / scale);
		cost += min(res, 3) * 2000;
		/*if (pc->bd > 0) {
			Point pos0(best_offset(y - 1, x - 1)[1], best_offset(y - 1, x - 1)[0]);
			Point pos1(best_offset(y - 1, x)[1], best_offset(y - 1, x)[0]);
			Point pos2(best_offset(y, x - 1)[1], best_offset(y, x - 1)[0]);
			Point pos3(best_offset(y, x)[1], best_offset(y, x)[0]);
			Point pos_dif[4] = { pos1 - pos0, pos3 - pos1, pos3 - pos2, pos2 - pos0 };
			for (int dir = 0; dir < 4; dir++) {
				Edge2 * pe = get_edge(pc->get_edge_idx(dir));				
				Point pos = pos_dif[dir] - pe->diff->offset;
				pos.x = pos.x / scale;
				pos.y = pos.y / scale;
				cost += pe->get_point_cost(pos, scale);
			}
		}*/
		cost = min(cost, 0x1fffULL);
		cost = (cost << 32) | ((unsigned long long) sft(y, x) << 45);
		corner_info(y, x) |= cost;
	}

}

int BundleAdjust2::arrange(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe, Size s, int option)
{
	bool week_border = option & BUNDLE_ADJUST_WEAK_ORDER;
	init(fet, _img_num_h, _img_num_w, fe, s, week_border);
	bool need_merge_all = option & (BUNDLE_ADJUST_SPEED_NORMAL | BUNDLE_ADJUST_SPEED_SLOW);
	bool need_optimize_corner = option & BUNDLE_ADJUST_SPEED_SLOW;
	merge_bundles();
	if (need_merge_all)
		merge_all();
	if (need_optimize_corner)
		optimize_corner(10, 10);
	output();
	return 0;
}
