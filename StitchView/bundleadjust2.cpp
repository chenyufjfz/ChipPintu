#include "bundleadjust2.h"
#include <algorithm> 
#include <queue>
#include <functional>

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
#define COST_BIGER_THAN_AVG 1000000
#define COST_BIND 10000000

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
	{ 4, { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 0, 0 } }, { 0, 0, 0, 0 }, 3 },
	{ 4, { { 1, 0 }, { 1, 1 }, { 0, 1 }, { 0, 0 } }, { 0, 0, 0, 0 }, 3 },
	{ 4, { { 0, -1 }, { 1, -1 }, { 1, 0 }, { 0, 0 } }, { 0, 0, 0, 0 }, 3 },
	{ 4, { { -1, 0 }, { -1, 1 }, { 0, 1 }, { 0, 0 } }, { 0, 0, 0, 0 }, 3 },
	{ 5, { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 } }, { 0, 0, 0, 1 }, 3 }, // |
	{ 5, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 4, 0 } }, { 0, 0, 0, 1 }, 3 }, 
	{ 5, { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 3 } }, { 0, 0, 0, 1 }, 3 }, // J
	{ 5, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 3, -1 } }, { 0, 0, 0, 1 }, 3 }, 
	{ 5, { { 0, -1 }, { 0, -2 }, { 0, -3 }, { -1, -3 } }, { 0, 0, 0, 1 }, 3 }, 
	{ 5, { { 0, -1 }, { 1, -1 }, { 2, -1 }, { 3, -1 } }, { 0, 0, 0, 1 }, 3 }, 
	{ 5, { { 0, 1 }, { 0, 2 }, { 0, 3 }, { -1, 3 } }, { 0, 0, 0, 1 }, 3 }, // L
	{ 5, { { 1, 0 }, { 2, 0 }, { 3, 0 }, { 3, 1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, -1 }, { 0, -2 }, { 0, -3 }, { 1, -3 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 1, 1 }, { 2, 1 }, { 3, 1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 0, 2 }, { 1, 2 }, { 1, 3 } }, { 0, 0, 0, 1 }, 3 }, //
	{ 5, { { 1, 0 }, { 2, 0 }, { 2, -1 }, { 3, -1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 1, 1 }, { 1, 2 }, { 1, 3 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 1, 0 }, { 1, -1 }, { 2, -1 }, { 3, -1 } }, { 0, 0, 0, 1 }, 3 },
	{ 5, { { 0, 1 }, { 0, 2 }, { -1, 2 }, { -1, 3 } }, { 0, 0, 0, 1 }, 3 }, //
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
	{ 5, { { 0, 1 }, { 1, 1 }, { 2, 1 }, { 0, 2 } }, { 0, 0, 1, 1 }, 2 }
};


#define MAKE_BUNDLE(c, changeid, idx, shape, queue, xok, yok, sx0, sx1, sy) make_pair( (((unsigned long long) ((c) * 100) << 32) + (changeid)), \
	((unsigned long long) (idx) << 32) + ((shape) << 8) + ((xok) << 16) + ((yok) << 18) + ((sx0) << 20) + ((sx1) << 24) + ((sy) << 28) +(queue))
#define BUNDLE_CHANGE_ID(b) ((unsigned)(b.first & 0xffffffff))
#define BUNDLE_CORNER_IDX(b) ((unsigned)(b.second >> 32))
#define BUNDLE_SHAPE(b) ((unsigned)(b.second >> 8 & 0xff))
#define BUNDLE_QUEUE(b) ((unsigned)(b.second & 0xff))
#define BUNDLE_XOK(b) ((unsigned)(b.second >> 16 & 3))
#define BUNDLE_YOK(b) ((unsigned)(b.second >> 18 & 3))
#define BUNDLE_SX0(b) ((unsigned)(b.second >> 20 & 0xf))
#define BUNDLE_SX1(b) ((unsigned)(b.second >> 24 & 0xf))
#define BUNDLE_SY(b) ((unsigned)(b.second >> 28 & 0xf))

#define BUNDLE_X_Y_0	0
#define BUNDLE_X_P_Y_1	1
#define BUNDLE_X_0		2
#define BUNDLE_Y_0		3
#define BUNDLE_X_1		4
#define BUNDLE_Y_1		5

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
			return shape_len + 4;
		else
		if (shape_len < 10)
			return 8;
		else
		if (shape_len < 17)
			return 9;
		else
			return 10;
	}
	if (abs(dx) == 1 || abs(dy) == 1) {
		if (shape_len < 5)
			return shape_len + 7;
		else
		if (shape_len < 10)
			return 11;
		else
		if (shape_len < 17)
			return 12;
		else
			return 13;
	}

	return 1000;
}

bool less_bd_4corner(const FourCorner * c1, const FourCorner * c2) {
	return c1->bd < c2->bd;
}


void BundleAdjust2::compute_edge_cost(Edge2 * pe, float global_avg)
{
	float mind = pe->diff->mind;
	float min2 = (pe->diff->mind + pe->diff->submind) / 2;
	float avg = pe->diff->avg * 0.6 + pe->diff->submind * 0.4;
	float alpha = (avg - mind < 9) ? 0 : 10 * sqrt(pe->diff->avg / global_avg);

	if (avg - mind < 9)
		qWarning("edge(%x) cost is all 0 because avg(%f) - mind(%f) is small", pe->diff->edge_idx, avg, mind);
	pe->cost.create(pe->diff->dif.size());
	if (pe->flag == 0) {
		for (int y = 0; y < pe->cost.rows; y++) {
			const int * pdif = pe->diff->dif.ptr<int>(y);
			float * pcost = pe->cost.ptr<float>(y);
			for (int x = 0; x < pe->cost.cols; x++) {
				float z = pdif[x];
				if (z > avg - 1)
					pcost[x] = COST_BIGER_THAN_AVG;
				else {
					pcost[x] = alpha * (z - mind) * (z - min2) / ((avg - z) * (avg - z));
					pcost[x] = min(pcost[x], (float) COST_BIGER_THAN_AVG);
				}
			}
		}
		pe->hard_score = alpha * (pe->diff->submind - mind) * (pe->diff->submind - min2) / ((avg - pe->diff->submind) * (avg - pe->diff->submind));
		return;
	}
	Point idea_pos = pe->idea_pos - pe->diff->offset;
	idea_pos.x = idea_pos.x / scale;
	idea_pos.y = idea_pos.y / scale;
	CV_Assert(idea_pos.y >= 0 && idea_pos.y < pe->cost.rows && idea_pos.x >= 0 && idea_pos.x < pe->cost.cols);
	if (pe->flag == (BIND_X_MASK | BIND_Y_MASK)) {
		pe->cost = COST_BIND;
		pe->cost(idea_pos) = 0;
		pe->hard_score = COST_BIND;
		return;
	}
	if (pe->flag == BIND_X_MASK) {
		pe->hard_score = COST_BIND;
		for (int y = 0; y < pe->cost.rows; y++) {
			const int * pdif = pe->diff->dif.ptr<int>(y);
			float * pcost = pe->cost.ptr<float>(y);
			for (int x = 0; x < pe->cost.cols; x++) {
				float z = pdif[x];
				if (x != idea_pos.x)
					pe->cost = COST_BIND;
				else
					if (z > avg - 1)
						pcost[x] = COST_BIGER_THAN_AVG;
					else {
						pcost[x] = alpha * (z - mind) * (z - min2) / ((avg - z) * (avg - z));
						pcost[x] = min(pcost[x], (float)COST_BIGER_THAN_AVG);
						if (pcost[x] > 0.001) //z!=mind
							pe->hard_score = min(pe->hard_score, pcost[x]);
					}
			}
		}
		if (pe->hard_score >= COST_BIND - 2)
			pe->hard_score = 0;
		return;
	}
	if (pe->flag == BIND_Y_MASK) {
		pe->hard_score = COST_BIND;
		for (int y = 0; y < pe->cost.rows; y++) {
			const int * pdif = pe->diff->dif.ptr<int>(y);
			float * pcost = pe->cost.ptr<float>(y);
			for (int x = 0; x < pe->cost.cols; x++) {
				float z = pdif[x];
				if (y != idea_pos.y)
					pe->cost = COST_BIND;
				else
					if (z > avg - 1)
						pcost[x] = COST_BIGER_THAN_AVG;
					else {
						pcost[x] = alpha * (z - mind) * (z - min2) / ((avg - z) * (avg - z));
						pcost[x] = min(pcost[x], (float)COST_BIGER_THAN_AVG);
						if (pcost[x] > 0.001) //z!=mind
							pe->hard_score = min(pe->hard_score, pcost[x]);
					}
			}
		}
		if (pe->hard_score >= COST_BIND - 2)
			pe->hard_score = 0;
		return;
	}
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
	CV_Assert(pcorner->idx == idx);
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
	qDebug("staty: %5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,",
		staty[0], staty[1], staty[2], staty[3], staty[4],
		staty[5], staty[6], staty[7], staty[8], staty[9]);
	qDebug("statx: %5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,%5f,",
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

void BundleAdjust2::init(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe)
{
	ConfigPara cpara = fet.get_config_para();
	img_num_h = _img_num_h >= 0 ? _img_num_h : cpara.img_num_h;
	img_num_w = _img_num_w >= 0 ? _img_num_w : cpara.img_num_w;
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
		pe->flag = 0;
		pe->idea_pos = pe->diff->minloc * scale + pe->diff->offset;
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
		pe->flag = 0;
		pe->idea_pos = pe->diff->minloc * scale + pe->diff->offset;
		avg[1] += pe->diff->avg;
	}
	avg[1] = avg[1] / (img_num_h * (img_num_w - 1));

	if (fe != NULL) {
		const vector<FixEdge> & vfe = *fe;
		for (int i = 0; i < vfe.size(); i++) {
			Edge2 * pe = get_edge(vfe[i].idx);
			pe->flag = vfe[i].bind_flag;
			Point idea_pos = vfe[i].shift - pe->diff->offset;
			idea_pos.x = idea_pos.x / scale;
			idea_pos.y = idea_pos.y / scale;
			pe->idea_pos = idea_pos * scale + pe->diff->offset;
		}
	}
	//1.2 init edge cost
	for (int i = 0; i < 2; i++)
	for (int j = 0; j < (int)eds[i].size(); j++)
		compute_edge_cost(&eds[i][j], avg[i]);
	//2 Init fourcorner
	fc.resize((img_num_h + 1) * (img_num_w + 1));
	queue<FourCorner *> bd_update;
	//2.1 init res_sft and bd
	for (int y = 0; y <= img_num_h; y++)
	for (int x = 0; x <= img_num_w; x++) {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		pc->idx = MAKE_CORNER_IDX(x, y);
		pc->bd = min(min(x, y), min(img_num_w - x, img_num_h - y));
		Point res_sft(0, 0);
		if (pc->bd)
		for (int i = 0; i < 4; i++) {
			Edge2 * pe = get_edge(pc->get_edge_idx(i));
			CV_Assert(pe != NULL);
			if (pe->diff->img_num == 0) {
				pc->bd = 0;
				res_sft = Point(0, 0);
				bd_update.push(pc);
				break;
			}
			if (i < 2)
				res_sft += pe->idea_pos;
			else
				res_sft -= pe->idea_pos;
		}
		CV_Assert(res_sft.x % scale == 0 && res_sft.y % scale == 0);
		res_sft.x = res_sft.x / scale;
		res_sft.y = res_sft.y / scale;
		pc->res_sft[0] = res_sft.y;
		pc->res_sft[1] = res_sft.x;
		pc->change_id = 0;
	}
	//2.2 second loop init bd
	while (!bd_update.empty()) {
		FourCorner * pc = bd_update.front();
		bd_update.pop();
		for (int dir = 0; dir < 4; dir++) {
			FourCorner * pc1 = get_4corner(pc->get_4corner_idx(dir));
			CV_Assert(pc1 != NULL);
			if (pc1->bd > pc->bd + 1) {
				pc1->bd = pc->bd + 1;
				bd_update.push(pc1);
			}
		}
	}
	//2.3 init type and adjust queue

	//4 init best offset
	best_offset.create(img_num_h, img_num_w);
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w; x++)
		best_offset(y, x) = cpara.offset(y, x);

	/*5 init corner info
	corner_info.create(img_num_h, img_num_w);
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w; x++) {
		int info = 0;
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		switch (pc->type[0]) {
		case BAD_4CORNER_HAS_MATE:
			info |= 1;
			break;
		case BAD_4CORNER_NO_MATE:
			info |= 2;
			break;
		}
		switch (pc->type[1]) {
		case BAD_4CORNER_HAS_MATE:
			info |= 0x4;
			break;
		case BAD_4CORNER_NO_MATE:
			info |= 0x8;
			break;
		}
		
		if (y == 0 || x == 0) {
			corner_info(y, x) = info;
			continue;
		}
		for (int i = 0; i < 4; i++) {
			Edge2 * pe = get_edge(pc->get_edge_idx(i));
			if (pe->ca[0] < 0.999) {
				if (pe->ca[0] > 0.74)
					info |= 1 << (8 + i * 4);
				else
				if (pe->ca[0] > 0.24)
					info |= 2 << (8 + i * 4);
				else
					info |= 3 << (8 + i * 4);
			}
			if (pe->ca[1] < 0.999) {
				if (pe->ca[1] > 0.74)
					info |= 1 << (10 + i * 4);
				else
				if (pe->ca[1] > 0.24)
					info |= 2 << (10 + i * 4);
				else
					info |= 3 << (10 + i * 4);
			}
		}
		corner_info(y, x) = info;
	}*/
}

Bundle BundleAdjust2::search_bundle(FourCorner * pc, int len_limit)
{
	CV_Assert(pc->res_sft[0] != 0 || pc->res_sft[1] != 0);
	int y0 = CORNER_Y(pc->idx);
	int x0 = CORNER_X(pc->idx);

	Bundle best = MAKE_BUNDLE(COST_BIND, 0, 0, 0, 255, 0, 0, 0, 0, 0);
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
			if (pe->flag & BIND_X_MASK)
				xok = 0;
			if (pe->flag & BIND_Y_MASK)
				yok = 0;
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
		//second loop compute cost
		pc0 = pc;
		float cost = 0;
		unsigned change_id = pc0->change_id;
		dx = pc->res_sft[1];
		dy = pc->res_sft[0];
		for (int j = 0; j < bundle_shape[i].len - 1; j++) {
			int y = y0 + bundle_shape[i].d[j][0];
			int x = x0 + bundle_shape[i].d[j][1];
			FourCorner * pc1 = get_4corner(y, x);
			change_id = max(change_id, pc1->change_id);
			Edge2 * pe = get_edge(pc1, pc0);
			CV_Assert(pe != NULL);
			
			int sign = 1;
			Point newpos;
			if (bundle_shape[i].c[j] == 0) {
				if (CORNER_X(pc0->idx) == CORNER_X(pc1->idx)) {
					if (CORNER_Y(pc0->idx) > CORNER_Y(pc1->idx))
						sign = -1;
				}
				else
				if (CORNER_X(pc0->idx) < CORNER_X(pc1->idx))
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
			if (xok && (newpos.x < 0 || newpos.x >= pe->cost.cols)) { //out of cost matrix, fail
				cost += COST_BIND / 4;
				break;
			}
				
			if (yok && (newpos.y < 0 || newpos.y >= pe->cost.rows)) { //out of cost matrix, fail
				cost += COST_BIND / 4;
				break;
			}
				
			if ((xok && sum_res_y <= 1 || yok && sum_res_x <= 1) &&
				(newpos.x >= 0 && newpos.x < pe->cost.cols) &&
				(newpos.y >= 0 && newpos.y < pe->cost.rows) &&
				(pe->cost(newpos) >= COST_BIGER_THAN_AVG - 1)) { //Low than average, fail
				cost += COST_BIGER_THAN_AVG;
				break;
			}
							
			cost += (queue < 6) ? pe->cost(newpos) : pe->hard_score;

			dx += pc1->res_sft[1];
			dy += pc1->res_sft[0];
			pc0 = pc1;
		}
		CV_Assert(cost < COST_BIND / 2);
		if (cost >= COST_BIGER_THAN_AVG)
			continue;
		if (queue < BUNDLE_QUEUE(best) || cost < best_cost) {
			best_cost = cost;
			best = MAKE_BUNDLE(cost, change_id, pc->idx, i, queue, xok, yok, 0, 0, 0);
		}
	}

	if (len_limit > 5) {

	}
	return best;
}

bool BundleAdjust2::merge_one_bundle(Bundle b, int change_id)
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

	//first loop is for Assert and check change id
	for (int j = 0; j < bundle_shape[shape].len - 1; j++) {
		int y = y0 + bundle_shape[shape].d[j][0];
		int x = x0 + bundle_shape[shape].d[j][1];
		FourCorner * pc1 = get_4corner(y, x);
		CV_Assert(pc1->bd != 0);
		Edge2 * pe = get_edge(pc1, pc0);
		CV_Assert(pe != NULL);
		if (xok)
			CV_Assert(!(pe->flag & BIND_X_MASK));
		if (yok)
			CV_Assert(!(pe->flag & BIND_Y_MASK));
		dx += pc1->res_sft[1];
		dy += pc1->res_sft[0];
		pc0 = pc1;
		if (pc0->change_id > BUNDLE_CHANGE_ID(b))
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
				} else
					pc0 = get_4corner(pc1->get_4corner_idx(DIR_LEFT));
			if (xok)
				pe->mls[1] += sign * pc1->res_sft[1];			
			if (yok) 
				pe->mls[0] += sign * pc1->res_sft[0];
		}
		dx += pc1->res_sft[1];
		dy += pc1->res_sft[0];
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
	int change_id = 1;
	for (int i = 0; i < 50; i++) {
		qDebug("Merge_bundles %d round", i);
		print_4corner_stat();
		priority_queue<Bundle, vector<Bundle>, greater<Bundle> > bundle_queue[14];
		int change_queue[12] = { 0 };
		int prev_change_id = change_id;
		unsigned exp_queue, len_limit;
		switch (i) {
		case 0:
			exp_queue = 1;
			len_limit = 3;
			break;
		case 1:
			exp_queue = 2;
			len_limit = 4;
			break;
		case 2:
			exp_queue = 4;
			len_limit = 4;
			break;
		case 3:
			exp_queue = 5;
			len_limit = 4;
			break;
		case 4:
			exp_queue = 7;
			len_limit = 4;
			break;
		case 5:
			exp_queue = 8;
			len_limit = 5;
			break;
		case 6:
			exp_queue = 11;
			len_limit = 5;
			break;
		case 7:
			exp_queue = 11;
			len_limit = 9;
			break;
		case 8:
			exp_queue = 12;
			len_limit = 16;
			break;
		default:
			exp_queue = 13;
			len_limit = 255;
			break;
		}
		for (int y = 0; y <= img_num_h; y++)
		for (int x = 0; x <= img_num_w; x++) {
			FourCorner * pc = &fc[y * (img_num_w + 1) + x];
			if (pc->res_sft[0] != 0 || pc->res_sft[1] != 0) {
				Bundle b = search_bundle(pc, len_limit);
				if (BUNDLE_QUEUE(b) <= exp_queue)
					bundle_queue[BUNDLE_QUEUE(b)].push(b);
			}
		}
		qDebug("Ready 0..5: %3d, %3d, %3d, %3d, %3d, %3d", bundle_queue[0].size(), bundle_queue[1].size(),
			bundle_queue[2].size(), bundle_queue[3].size(), bundle_queue[4].size(), bundle_queue[5].size());
		qDebug("Ready 6.11: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d", bundle_queue[6].size(), bundle_queue[7].size(),
			bundle_queue[8].size(), bundle_queue[9].size(), bundle_queue[10].size(),
			bundle_queue[11].size(), bundle_queue[12].size(), bundle_queue[13].size());

		for (int j = 0; j < sizeof(bundle_queue) /sizeof(bundle_queue[0]); j++)
		while (!bundle_queue[j].empty()) {
			Bundle b = bundle_queue[j].top();
			bundle_queue[j].pop();
			if (merge_one_bundle(b, change_id)) {
				change_id++;
				change_queue[j]++;
			}
		}
		qDebug("Merge 0..5: %3d, %3d, %3d, %3d, %3d, %3d", change_queue[0], change_queue[1],
			change_queue[2], change_queue[3], change_queue[4], change_queue[5]);
		qDebug("Merge 6.11: %3d, %3d, %3d, %3d, %3d, %3d", change_queue[6], change_queue[7],
			change_queue[8], change_queue[9], change_queue[10], change_queue[11]);
		if (change_id == prev_change_id && i > 8)
			break;
	}
}

void BundleAdjust2::adjust_edge_mls(FourCorner * pc, int dim, int modify)
{
	FourCorner * start = pc;
	while (pc->fa != pc->idx) {
		int dir = pc->get_dir(pc->fa);
		Edge2 * pe = get_edge(pc->get_edge_idx(dir));
		CV_Assert(pe != NULL);
		int dmls = (dir < 2) ? modify: -modify;
		pe->mls[dim] += dmls;
		pc = get_4corner(pc->fa);
		if (pc == start)
			break;
	}
}

void BundleAdjust2::adjust_one_4corner(int dim, FourCorner * root)
{
#define MAKE_EXPAND(pc) (((unsigned long long) (pc->cost * 1000 + 1000000000) << 32) + pc->idx)
	/*
	int sign = SGN(root->res_sft[dim]);
	while (root->res_sft[dim] != 0) {
		vector<FourCorner *> reach_queue;
		FourCorner * best = NULL;
		priority_queue<unsigned long long, vector<unsigned long long>, greater<unsigned long long> > expand_queue;
		root->fa = root->idx;
		root->cost = 0;
		root->depth = 0;
		expand_queue.push(MAKE_EXPAND(root));
		int find_answer = 0; //0 means not found, 1 means found path to reduce res_sft, 2 means found loop reduce cost (res_sft not reduce)
		while (!expand_queue.empty()) {
			unsigned long long top = expand_queue.top();
			expand_queue.pop();
			FourCorner * pc = get_4corner(top & 0xffffffff);
			if (top > MAKE_EXPAND(pc)) //already have better path
				continue;
			reach_queue.push_back(pc);
			if (best != NULL && best->cost + 3 < pc->cost) { //search enough 4corner, already find best, quit expand
				find_answer = 1;
				break;
			}
			
			for (int dir = 0; dir < 4; dir++) {
				FourCorner * pc1 = get_4corner(pc->get_4corner_idx(dir));
				Edge2 * pe = get_edge(pc->get_edge_idx(dir));
				CV_Assert(pc1 != NULL && pe != NULL);

				if (pc1->idx == pc->fa) //not search father
					continue;
				if (best != NULL) { //check distance
					int dy0 = CORNER_Y(best->idx) - CORNER_Y(root->idx);
					int dx0 = CORNER_X(best->idx) - CORNER_X(root->idx);
					int dy1 = CORNER_Y(pc1->idx) - CORNER_Y(root->idx);
					int dx1 = CORNER_X(pc1->idx) - CORNER_X(root->idx);
					if (abs(dy0) + abs(dx0) + 2 < abs(dy1) + abs(dx1))
						continue;
				}

				float old_edge_cost = COST_BETA(pe->mls[dim]);
				int dmls = (dir < 2) ? -sign : sign;
				float new_edge_cost = COST_BETA(pe->mls[dim] + dmls);
				float delta_edge_cost = pe->ca[dim] * (new_edge_cost - old_edge_cost);
				if (pc1->cost > pc->cost + delta_edge_cost + 0.01) { //update cost	
#if 1
					float delta = pc1->cost - pc->cost - delta_edge_cost;
#endif
					pc1->cost = pc->cost + delta_edge_cost;
					pc1->fa = pc->idx;
					if (pc->depth >= pc1->depth + 3) {
						FourCorner * pc2 = pc;
						while (pc2 != root) { //not root
							pc2 = get_4corner(pc2->fa);
							if (pc2 == pc1) {
								find_answer = 2;
								best = pc;
								break;
							}
						}
					}
					if (find_answer)
						break;
					pc1->depth = pc->depth + 1;
					if (pc1->bd != 0)
						expand_queue.push(MAKE_EXPAND(pc1));
					else
						reach_queue.push_back(pc1);
					if (pc1->bd == 0 || pc1->res_sft[dim] * sign < 0) { //find answer
						if (best == NULL || best->cost > pc1->cost) //check if it is best answer
							best = pc1;
						if (best->cost <= 0)
							find_answer = 1;
					}
				}
			}
			if (find_answer)
				break;
		}
		CV_Assert(best != NULL);
		if (find_answer == 2) {
			adjust_edge_mls(best, dim, sign);
		}
		else {
			find_answer = 1;
			//compute path's cap
			int cap = abs(root->res_sft[dim]);
			if (best->bd != 0)
				cap = min(cap, abs(best->res_sft[dim]));
			FourCorner * pc = best;
			while (pc != root) { //not reach root
				int dir = pc->get_dir(pc->fa);
				Edge2 * pe = get_edge(pc->get_edge_idx(dir));
				CV_Assert(pe != NULL);
				if (pe->ca[dim] != 0) {
					CV_Assert(pe->ca[dim] < 999);
					int dmls = (dir < 2) ? sign : -sign;
					if (dmls * pe->mls[dim] < 0)
						cap = min(cap, abs(pe->mls[dim]));
					else
						cap = 1;
				}
				pc = get_4corner(pc->fa);
			}
			CV_Assert(cap >= 1);
			//second loop, update edge.mls
			adjust_edge_mls(best, dim, cap * sign);
			root->res_sft[dim] -= sign * cap;
			if (best->bd != 0)
				best->res_sft[dim] += sign * cap;
		}
		//reset cost
		for (int i = 0; i < reach_queue.size(); i++) {
			reach_queue[i]->cost = 1000000;
			reach_queue[i]->fa = 0;
			reach_queue[i]->depth = 1000000;
		}
		while (!expand_queue.empty()) {
			unsigned long long top = expand_queue.top();
			expand_queue.pop();
			FourCorner * pc = get_4corner(top & 0xffffffff);
			pc->cost = 1000000;
			pc->fa = 0;
			pc->depth = 1000000;
		}
	}*/
}

void BundleAdjust2::adjust()
{
	//init adjust_queue
	adjust_queue[0].clear();
	adjust_queue[1].clear();
	for (int y = 0; y <= img_num_h; y++)
	for (int x = 0; x <= img_num_w; x++) {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		for (int i = 0; i < 2; i++)
		if (pc->res_sft[i] != 0) {
			CV_Assert(pc->bd > 0);
			adjust_queue[i].push_back(pc);
		}
	}
	sort(adjust_queue[0].begin(), adjust_queue[0].end(), less_bd_4corner);
	sort(adjust_queue[1].begin(), adjust_queue[1].end(), less_bd_4corner);

	for (int i = 0; i < fc.size(); i++) {
		fc[i].cost = 1000000;
		fc[i].fa = 0;
		fc[i].depth = 1000000;
	}
	for (int dim = 0; dim < 2; dim++) {
		while (!adjust_queue[dim].empty()) {
			FourCorner * pc = adjust_queue[dim].back();
			adjust_queue[dim].pop_back();
			adjust_one_4corner(dim, pc);
		}
	}

	//check all 4corner satisfy res_sft==0
	for (int j = 0; j < fc.size(); j++) 
	if (fc[j].bd != 0) {
		Point res_sft(0, 0);
		for (int dir = 0; dir < 4; dir++) {
			Edge2 * pe = get_edge(fc[j].get_edge_idx(dir));
			CV_Assert(pe != NULL && pe->diff->img_num != 0);
			if (dir < 2)
				res_sft += pe->idea_pos + Point(pe->mls[1], pe->mls[0]) * scale;
			else
				res_sft -= pe->idea_pos + Point(pe->mls[1], pe->mls[0]) * scale;
		}
		CV_Assert(res_sft.x == 0 && res_sft.y == 0);
	}

	//compute need_visit, 0 means no need, 1 means need, 2 means already visit
	vector<int> need_visit(img_num_h * img_num_w, 0);
	for (int i = 0; i < 2; i++)
	for (int j = 0; j < eds[i].size(); j++) {
		if (eds[i][j].diff->img_num != 0) {
			int y = EDGE_Y(eds[i][j].diff->edge_idx);
			int x = EDGE_X(eds[i][j].diff->edge_idx);
			if (i == 0) {
				need_visit[y * img_num_w + x] = 1;
				need_visit[(y + 1) * img_num_w + x] = 1;
			}
			else {
				need_visit[y * img_num_w + x] = 1;
				need_visit[y * img_num_w + x + 1] = 1;
			}
		}
	}
	for (int j = 0; j < need_visit.size(); j++)
	if (need_visit[j] == 1) {
		vector<int> search_queue;
		search_queue.push_back(j);
		while (!search_queue.empty()) {
			int idx = search_queue.back();
			search_queue.pop_back();
			int y0 = idx / img_num_w;
			int x0 = idx % img_num_w;
			for (int dir = 0; dir < 4; dir++) {
				int y = y0 + dxy[dir][0];
				int x = x0 + dxy[dir][1];
				if (x >= 0 && y >= 0 && x < img_num_w && y < img_num_h &&
					need_visit[y * img_num_w + x] == 1) {
					search_queue.push_back(y * img_num_w + x);
					need_visit[y * img_num_w + x] = 2;
					Edge2 * pe;
					Point offset;
					switch (dir) {
					case DIR_UP:
						pe = get_edge(0, y0 - 1, x0);
						offset = -pe->idea_pos - Point(pe->mls[1], pe->mls[0]) * scale;
						break;
					case DIR_DOWN:
						pe = get_edge(0, y0, x0);
						offset = pe->idea_pos + Point(pe->mls[1], pe->mls[0]) * scale;
						break;
					case DIR_LEFT:
						pe = get_edge(1, y0, x0 - 1);
						offset = -pe->idea_pos - Point(pe->mls[1], pe->mls[0]) * scale;
						break;
					case DIR_RIGHT:
						pe = get_edge(1, y0, x0);
						offset = pe->idea_pos + Point(pe->mls[1], pe->mls[0]) * scale;
						break;
					}
					offset.y += best_offset(y0, x0)[0];
					offset.x += best_offset(y0, x0)[1];
					best_offset(y, x) = Vec2i(offset.y, offset.x);
				}
			}
		}
	}
}

int BundleAdjust2::arrange(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe)
{
	init(fet, _img_num_h, _img_num_w, fe);
	merge_bundles();
	return 0;
}
