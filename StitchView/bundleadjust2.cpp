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
const float cost_beta[] = { 0, 1, 2.5, 6, 12};

bool less_bd_4corner(const FourCorner * c1, const FourCorner * c2) {
	return c1->bd < c2->bd;
}

struct cmp_cost {
	bool operator()(const FourCorner * c1, const FourCorner * c2) {
		return c1->cost > c2->cost;
	}
};

float COST_BETA(int s) {
	s = abs(s);
	return (s >= sizeof(cost_beta) / sizeof(cost_beta[0]) ? s* (s - 1) : cost_beta[s]);
}
int get_4corner_threshold(int scale) {
	if (scale == 1)
		return 4;
	else
		return 3;
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

//if idx is invalid, return NULL
Edge2 * BundleAdjust2::get_edge(int idx)
{
	Edge2 * ed = get_edge(EDGE_E(idx), EDGE_Y(idx), EDGE_X(idx));
	CV_Assert(ed == NULL || ed->diff->edge_idx == idx);
	return ed;
}

FourCorner * BundleAdjust2::get_4corner(int idx)
{
	int y = IMG_Y(idx);
	int x = IMG_X(idx);
	if (y > img_num_h || x > img_num_w || x < 0 || y < 0)
		return NULL;
	FourCorner * pcorner = &fc[y * (img_num_w + 1) + x];
	CV_Assert(pcorner->idx == idx);
	return pcorner;
}

void BundleAdjust2::compute_edge_cost_ratio(Edge2 * pe, int dim)
{
	unsigned cidx0, cidx1;
	const EdgeDiff * pdiff = pe->diff;
	int r = abs(pdiff->minloc.y - pdiff->subminloc.y) + abs(pdiff->minloc.x - pdiff->subminloc.x);
	pe->get_4corner_idx(cidx0, cidx1);
	FourCorner * pc0 = get_4corner(cidx0);
	FourCorner * pc1 = get_4corner(cidx1);
	
	if (pc0->type[dim] > pc1->type[dim])
		swap(pc0, pc1);
	switch (pc0->type[dim]) {
	case NOT_VALID_4CORNER:
	case GOOD_4CORNER:
		if (pc1->type[dim] == GOOD_4CORNER) {
			pe->ca[dim] = 1;
		}
		else
		if (pc1->type[dim] == BAD_4CORNER_HAS_MATE)
			pe->ca[dim] = (r <= 1) ? 0.8 : 0.75;
		else
			pe->ca[dim] = (r <= 1) ? 0.3 : 0.25;
		break;
	case BAD_4CORNER_HAS_MATE:
	case BAD_4CORNER_NO_MATE:
		pe->ca[dim] = 0;
		break;
	default:
		CV_Assert(0);
	}
	
}
void BundleAdjust2::init(const FeatExt & fet, int _img_num_h, int _img_num_w, const vector<FixEdge> * fe)
{
	ConfigPara cpara = fet.get_config_para();
	img_num_h = _img_num_h >= 0 ? _img_num_h : cpara.img_num_h;
	img_num_w = _img_num_w >= 0 ? _img_num_w : cpara.img_num_w;
	scale = cpara.rescale;
	int th = get_4corner_threshold(scale);
	//1 Init edge diff, mls, idea_pos
	eds[0].clear();
	eds[0].resize((img_num_h - 1) * img_num_w);
	for (int y = 0; y < img_num_h - 1; y++)
	for (int x = 0; x < img_num_w; x++) {
		Edge2 * pe = &eds[0][y * img_num_w + x];
		pe->diff = fet.get_edge(0, y, x);
		pe->mls[0] = 0;
		pe->mls[1] = 0;
		pe->idea_pos = pe->diff->minloc * scale + pe->diff->offset;
		pe->ca[0] = 0;
		pe->ca[1] = 0;
	}

	eds[1].clear();
	eds[1].resize(img_num_h * (img_num_w - 1));
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w - 1; x++) {
		Edge2 * pe = &eds[1][y * (img_num_w - 1) + x];
		pe->diff = fet.get_edge(1, y, x);
		pe->mls[0] = 0;
		pe->mls[1] = 0;
		pe->idea_pos = pe->diff->minloc * scale + pe->diff->offset;
		pe->ca[0] = 0;
		pe->ca[1] = 0;
	}
	if (fe != NULL) {
		const vector<FixEdge> & vfe = *fe;

		for (int i = 0; i < vfe.size(); i++) {
			Edge2 * pe = get_edge(vfe[i].idx);
			pe->idea_pos = vfe[i].shift;
			if (vfe[i].bind_flag & BIND_Y_MASK)
				pe->ca[0] = 1000;
			if (vfe[i].bind_flag & BIND_X_MASK)
				pe->ca[1] = 1000;
		}
	}
	//2 Init fourcorner
	vector<double> statx(10, 0), staty(10, 0);
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
		if (abs(res_sft.x) < 9)
			statx[abs(res_sft.x)] += 1;
		else
			statx[9] += 1;
		if (abs(res_sft.y) < 9)
			staty[abs(res_sft.y)] += 1;
		else
			staty[9] += 1;
	}

	for (int i = 0; i < 10; i++) {
		staty[i] = staty[i] * 100 / ((img_num_h + 1) * (img_num_w + 1));
		statx[i] = statx[i] * 100 / ((img_num_h + 1) * (img_num_w + 1));
	}
	qDebug("staty: %4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%",
		staty[0], staty[1], staty[2], staty[3], staty[4],
		staty[5], staty[6], staty[7], staty[8], staty[9]);
	qDebug("statx: %4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%,%4.1f%%",
		statx[0], statx[1], statx[2], statx[3], statx[4],
		statx[5], statx[6], statx[7], statx[8], statx[9]);
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
	adjust_queue[0].clear();
	adjust_queue[1].clear();
	for (int y = 0; y <= img_num_h; y++)
	for (int x = 0; x <= img_num_w; x++) {
		FourCorner * pc = &fc[y * (img_num_w + 1) + x];
		for (int i = 0; i < 2; i++) {
			if (pc->bd == 0)
				pc->type[i] = NOT_VALID_4CORNER;
			else 
				if (abs(pc->res_sft[i]) <= th)
					pc->type[i] = GOOD_4CORNER;
				else {
					pc->type[i] = (pc->bd == 1) ? BAD_4CORNER_HAS_MATE : BAD_4CORNER_NO_MATE;
					if (pc->bd > 1)
					for (int dir = 0; dir < 4; dir++) {
						FourCorner * pc1 = get_4corner(pc->get_4corner_idx(dir));
						if (abs(pc1->res_sft[i]) > th && pc1->res_sft[i] * pc->res_sft[i] < 0)
							pc->type[i] = BAD_4CORNER_HAS_MATE;
					}
				}				
		}
		for (int i = 0; i < 2; i++)
		if (pc->res_sft[i] != 0) {
			CV_Assert(pc->bd > 0);
			adjust_queue[i].push_back(pc);
		}
	}
	sort(adjust_queue[0].begin(), adjust_queue[0].end(), less_bd_4corner);
	sort(adjust_queue[1].begin(), adjust_queue[1].end(), less_bd_4corner);

	//3 continue init edge ca
	for (int y = 0; y < img_num_h - 1; y++)
	for (int x = 0; x < img_num_w; x++) {
		Edge2 * pe = &eds[0][y * img_num_w + x];
		if (pe->ca[0] < 0.001) //==0
			compute_edge_cost_ratio(pe, 0);
		if (pe->ca[1] < 0.001) //==0
			compute_edge_cost_ratio(pe, 1);
	}

	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w - 1; x++) {
		Edge2 * pe = &eds[1][y * (img_num_w - 1) + x];
		if (pe->ca[0] < 0.001) //==0
			compute_edge_cost_ratio(pe, 0);
		if (pe->ca[1] < 0.001) //==0
			compute_edge_cost_ratio(pe, 1);
	}

	//4 init best offset
	best_offset.create(img_num_h, img_num_w);
	for (int y = 0; y < img_num_h; y++)
	for (int x = 0; x < img_num_w; x++)
		best_offset(y, x) = cpara.offset(y, x);

	//5 init corner info
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
	}
}

void BundleAdjust2::adjust()
{
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
	adjust();
	return 0;
}
