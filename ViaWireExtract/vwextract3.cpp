#include "vwextract3.h"
#include "vwextract_public.h"

#define S_SCORE(p) ((int)((p) >> 16 & 0xffff))
#define S_TYPE(p) ((unsigned)((p) >> 8 & 0xff))
#define S_SHAPE(p) ((unsigned)((p) & 0xff))
#define MAKE_S(score, type, shape) ((unsigned)(score) << 16 | (unsigned)(type) << 8 | (shape))
#define DETECT_GRAY_RADIUS 64
#define DETECT_WIRE_RADIUS 64
#define DETECT_GRAY2_WIDTH 7
/*
input: img
input: rect
input: step
output: bins
bins is gray statistics for img[rect]
*/
static int cal_bins(Mat &img, QRect rect, vector<unsigned> &bins, int step = 1)
{
	CV_Assert(img.type() == CV_8UC1 && step >= 1 && rect.y() >=0 && rect.bottom() < img.rows && rect.x()>=0 && rect.right() < img.cols);
	int total = 0;
	bins.assign(256, 0);
	for (int y = rect.y(); y < rect.y() + rect.height(); y += step) {
		unsigned char * p_img = img.ptr<unsigned char>(y);
		for (int x = rect.x(); x < rect.x() + rect.width(); x += step) {
			bins[p_img[x]]++;
			total++;
		}
	}
	return total;
}

/*
Use 2 julei
input: bins
input: init_num 2 junlei init_num normally is 0.5
input: cut_via_ratio, cut via bins
output: th
*/
static void cal_threshold(vector<unsigned> bins, vector<unsigned> & th, float init_ratio = 0.5, float cut_via_ratio = 0.01, float cut_insu_ratio = 0.00)
{
	CV_Assert(cut_via_ratio < 0.25);
	int total = 0, total1 = 0, sep, total2, old_sep = 0xffffffff;
	for (sep = 0; sep < bins.size(); sep++)
		total += bins[sep];

	if (cut_via_ratio != 0) {
		total1 = total * cut_via_ratio;
		for (sep = (int)bins.size() - 1; sep >= 0; sep--) {
			total1 -= bins[sep];
			if (total1 < 0)
				break;
			total -= bins[sep];
			bins[sep] = 0;
		}
		total1 = 0;
	}

	if (cut_insu_ratio != 0) {
		total1 = total * cut_insu_ratio;
		for (sep = 0; sep < bins.size(); sep++) {
			total1 -= bins[sep];
			if (total1 < 0)
				break;
			total -= bins[sep];
			bins[sep] = 0;
		}
		total1 = 0;
	}
	int init_num = total * init_ratio;
	for (sep = 0; sep < bins.size(); sep++) {
		total1 += bins[sep];
		if (total1 > init_num)
			break;
	}

	CV_Assert(sep < bins.size());
	double m1_l = 0, m1_r = 0;
	while (sep != old_sep) {
		old_sep = sep;
		m1_l = 0, m1_r = 0;
		total1 = 0, total2 = 0;
		for (int j = 0; j <= sep; j++) {
			m1_l += bins[j] * j;
			total1 += bins[j];
		}
		for (int j = sep + 1; j < bins.size(); j++) {
			m1_r += bins[j] * j;
			total2 += bins[j];
		}
		CV_Assert(total1 != 0 && total2 != 0);
		m1_l = m1_l / total1;
		m1_r = m1_r / total2;
		sep = (m1_l + m1_r) / 2;
	}
	th.clear();
	th.push_back(m1_l);
	th.push_back(m1_r);
}

static struct Wire25ShapeConst3 {
	double ratio[25];
	int shape;
} wire_25_shape[] = {
	//   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
	{ { -1, -1, 00, -1, -1, -1, 00, 00, 00, -1, 00, 00, 00, 00, 00, -1, 00, 00, 00, -1, -1, -1, 00, -1, -1 }, BRICK_NO_WIRE },
	{ { -1, -1, 01, -1, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_I_0 },
	{ { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, 01, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_I_90 },
	{ { -1, -1, 01, -1, -1, -1, 01, 01, 00, -1, -1, 01, 01, 00, -1, -1, 01, 01, 00, -1, -1, -1, 01, -1, -1 }, BRICK_II_0 },
	{ { -1, -1, -1, -1, -1, -1, 01, 01, 01, -1, 01, 01, 01, 01, 01, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_II_90 },
	{ { -1, -1, 01, -1, -1, -1, 00, 01, 01, -1, -1, 00, 01, 01, -1, -1, 00, 01, 01, -1, -1, -1, 01, -1, -1 }, BRICK_II_180 },
	{ { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, 01, 01, 01, 01, 01, -1, 01, 01, 01, -1, -1, -1, -1, -1, -1 }, BRICK_II_270 },
	/*
	{ { -1, -1, .1, -1, -1, -1, 00, .5, 01, -1, .1, .5, .9, .5, .1, -1, 01, .5, 00, -1, -1, -1, .1, -1, -1 }, BRICK_Z_0 },
	{ { -1, -1, .1, -1, -1, -1, 01, .5, 00, -1, .1, .5, .9, .5, .1, -1, 00, .5, 01, -1, -1, -1, .1, -1, -1 }, BRICK_Z_90 },
	{ { -1, 00, 01, 00, -1, -1, 00, 01, 00, 00, -1, .5, 01, 01, 01, -1, 01, .5, 00, 00, 01, -1, -1, -1, -1 }, BRICK_Y_45 },
	{ { 01, -1, -1, -1, -1, -1, 01, .5, 00, 00, -1, .5, 01, 01, 01, -1, 00, 01, 00, 00, -1, 00, 01, 00, -1 }, BRICK_Y_135 },
	{ { -1, -1, -1, -1, 01, 00, 00, .5, 01, -1, 01, 01, 01, .5, -1, 00, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_Y_225 },
	{ { -1, 00, 01, 00, -1, 00, 00, 01, 00, -1, 01, 01, 01, .5, -1, 00, 00, .5, 01, -1, -1, -1, -1, -1, 01 }, BRICK_Y_315 }*/
	//   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
};

struct WireParameter3 {
	int type;
	int w_wide, w_wide1, w_high, w_high1; //normally w_high1 <w_high & w_wide1 <w_wide, these is for wire
	int i_wide, i_high;	//this is for insu
	int gray_w, gray_i; //this is not set by ProcessParameter
	WireParameter3() {
		w_wide = 0;
		w_high = 0;
		w_wide1 = 0;
		w_high1 = 0;
		i_wide = 0;
		i_high = 0;
	}
	WireParameter3(int w, int gw, int gi, int _type=-1) {
		w_high = w;
		w_wide = w;
		i_high = 4;
		i_wide = 4;
		w_high1 = 4;
		w_wide1 = 4;
		if (_type < 0)
			type = w;
		else
			type = _type;
		gray_i = gi;
		gray_w = gw;
	}
};

struct WirePartStat3 { //each shape's part expect stat value
	int part, sum; //These para are compute during prepare, and after prepare they are fixed
	int err; //this is update by compute, it means this part error
	bool operator == (const WirePartStat3 & ano) {
		return (part == ano.part && sum == ano.sum);
	}
};

struct ShapeDeviation3 {
	vector<int> si; //WirePartStat index, each ShapeDeviation have 13 or 17 or 25 part(si)
	int w25_shape_idx; //index of wire_25_shape
};

class Wire_25RectCompute3 {
public:
	WireParameter3 wp;
protected:
	vector<int> offset, dx, dy; //offset is compute based on dx and dy	
	vector<WirePartStat3> stats; //each part expected value
	vector<ShapeDeviation3> sds; //each shape may have one or multipul ShapeDeviation
	int mode;	
	double arf;
public:
	Wire_25RectCompute3() {
		offset.resize(36);
		dx.resize(36);
		dy.resize(36);
	}
	int prepare(WireParameter3 & _wp, const Mat & ig, const Mat & iig) {
		CV_Assert(ig.size() == iig.size() && ig.type()==CV_32SC1);
		wp = _wp;
		//following compute offset
		dx[0] = -wp.w_wide1 - wp.i_wide - wp.w_wide / 2;
		dx[1] = -wp.i_wide - wp.w_wide / 2;
		dx[2] = -wp.w_wide / 2;
		dx[3] = wp.w_wide - wp.w_wide / 2;
		dx[4] = wp.w_wide - wp.w_wide / 2 + wp.i_wide;
		dx[5] = wp.w_wide - wp.w_wide / 2 + wp.i_wide + wp.w_wide1;		
		for (int i = 6; i < 36; i++)
			dx[i] = dx[i - 6];

		dy[0] = -wp.w_high1 - wp.i_high - wp.w_high / 2;
		dy[6] = -wp.i_high - wp.w_high / 2;
		dy[12] = -wp.w_high / 2;
		dy[18] = wp.w_high - wp.w_high / 2;
		dy[24] = wp.w_high - wp.w_high / 2 + wp.i_high;
		dy[30] = wp.w_high - wp.w_high / 2 + wp.i_high + wp.w_high1;
		for (int i = 0; i < 36; i++)
			dy[i] = dy[i / 6 * 6];
		int area[25];
		for (int i = 0, j = 0; i < 25; i++, j++) {
			if (dx[j + 1] - dx[j] < 0)
				j++;
			area[i] = (dx[j + 1] - dx[j]) * (dy[j + 6] - dy[j]);
		}
		for (int i = 0; i < 36; i++)
			offset[i] = (dy[i] * (int) ig.step.p[0] + dx[i] * (int) ig.step.p[1]) / sizeof(unsigned);
		//following compute expected stats and init each shape part
		stats.clear();
		sds.clear();
		mode = 0;
		//stats 0..24 means each part minimum error
		for (int i = 0; i < 25; i++) {
			WirePartStat3 stat;
			stat.part = i;
			stat.sum = -1;
			stats.push_back(stat);
		}
		for (int j = 0; j < sizeof(wire_25_shape) / sizeof(wire_25_shape[0]); j++) {
			ShapeDeviation3 sd;
			sd.w25_shape_idx = j;
			for (int k = 0; k < 25; k++) { //following add wire_25_shape[j] part stat to shape sd
				if (wire_25_shape[j].ratio[k] < 0) { //<0 means don't care, choose minimum error to balance
					sd.si.push_back(k);
					continue;
				}
				//compute stat and check if it already exist, save computing
				WirePartStat3 stat;
				stat.part = k;
				stat.sum = round((wp.gray_i * (1 - wire_25_shape[j].ratio[k]) + wp.gray_w * wire_25_shape[j].ratio[k]) * area[k]);
				CV_Assert(stat.sum >= 0);
				
				vector<WirePartStat3>::iterator it = find(stats.begin(), stats.end(), stat); //check if stat already exit
				if (it == stats.end()) {
					sd.si.push_back((int)stats.size());
					stats.push_back(stat);
					if ((k == 0 || k == 4 || k == 20 || k == 24) && mode != 2)
						mode = 1;
					if (k == 1 || k == 3 || k == 5 || k == 9 || k == 15 || k == 19 || k == 21 || k == 23)
						mode = 2;
				}
				else
					sd.si.push_back(it - stats.begin());
				CV_Assert(stats[sd.si.back()] == stat);
			}
			sds.push_back(sd);
		}
		//for mode2, stats[sds[i].si[0]].part=0,stats[sds[i].si[1]].part=1,stats[sds[i].si[2]].part=2, 
		if (mode == 1) { //for mode1, stats[sds[i].si[0]].part=0,stats[sds[i].si[1]].part=2,stats[sds[i].si[2]].part=4
			for (int i = 0; i < (int)sds.size(); i++) {
				sds[i].si.erase(sds[i].si.begin() + 23);
				sds[i].si.erase(sds[i].si.begin() + 21);
				sds[i].si.erase(sds[i].si.begin() + 19);
				sds[i].si.erase(sds[i].si.begin() + 15);
				sds[i].si.erase(sds[i].si.begin() + 9);
				sds[i].si.erase(sds[i].si.begin() + 5);
				sds[i].si.erase(sds[i].si.begin() + 3);
				sds[i].si.erase(sds[i].si.begin() + 1);
			}
		}

		if (mode == 0) { //for mode0, stats[sds[i].si[0]].part=2,stats[sds[i].si[1]].part=6,stats[sds[i].si[2]].part=7
			for (int i = 0; i < (int)sds.size(); i++) {
				sds[i].si.erase(sds[i].si.begin() + 24);
				sds[i].si.erase(sds[i].si.begin() + 23);
				sds[i].si.erase(sds[i].si.begin() + 21);
				sds[i].si.erase(sds[i].si.begin() + 20);
				sds[i].si.erase(sds[i].si.begin() + 19);
				sds[i].si.erase(sds[i].si.begin() + 15);
				sds[i].si.erase(sds[i].si.begin() + 9);
				sds[i].si.erase(sds[i].si.begin() + 5);
				sds[i].si.erase(sds[i].si.begin() + 4);
				sds[i].si.erase(sds[i].si.begin() + 3);
				sds[i].si.erase(sds[i].si.begin() + 1);
				sds[i].si.erase(sds[i].si.begin());
			}
		}
		qInfo("Wire_25RectCompute3 Prepare w=%d,h=%d,w1=%d,h1=%d,i_w=%d,i_h=%d,mode=%d", wp.w_wide, wp.w_high,
			wp.w_wide1, wp.w_high1, wp.i_wide, wp.i_high, mode);
		int tot_area = 0;
		tot_area = area[12] + 2 * area[7] + 2 * area[11] + 4 * area[6] + 2 * area[2] + 2 * area[10];
		if (mode > 0)
			tot_area += 4 * area[0];
		if (mode > 1)
			tot_area += 4 * area[1] + 4 * area[5];
		arf = 256.0 / (sqrt(tot_area) * (wp.gray_w - wp.gray_i));
		return 0;
	}
	
	bool compute_valid(Point p0, const Mat & ig) {
		int x0 = p0.x, y0 = p0.y;
		if (x0 + dx[0] < 0 || y0 + dy[0] < 0 || x0 + dx[5] >= ig.cols || y0 + dy[30] >= ig.rows)
			return false;
		else
			return true;
	}
	//return S(y0, x0)
	unsigned compute(Point p0, const Mat & ig, const Mat & ) {
		int x0 = p0.x, y0 = p0.y;
		if (!compute_valid(p0, ig))
			return 0xffffffff;
		unsigned s[36];
		const unsigned *p_ig = ig.ptr<unsigned>(y0, x0);
		for (int i = 0; i < 36; i++)
			s[i] = p_ig[offset[i]];
			
		int sum[25], e[30];
		e[2] = s[8] - s[2];
		e[3] = s[9] - s[3];
		e[7] = s[13] - s[7];
		e[8] = s[14] - s[8];
		e[9] = s[15] - s[9];
		e[10] = s[16] - s[10];
		e[12] = s[18] - s[12];
		e[13] = s[19] - s[13];
		e[14] = s[20] - s[14];
		e[15] = s[21] - s[15];
		e[16] = s[22] - s[16];
		e[17] = s[23] - s[17];
		e[19] = s[25] - s[19];
		e[20] = s[26] - s[20];
		e[21] = s[27] - s[21];
		e[22] = s[28] - s[22];
		e[26] = s[32] - s[26];
		e[27] = s[33] - s[27];

		sum[2] = e[3] - e[2];
		sum[6] = e[8] - e[7];
		sum[7] = e[9] - e[8];
		sum[8] = e[10] - e[9];
		sum[10] = e[13] - e[12];
		sum[11] = e[14] - e[13];
		sum[12] = e[15] - e[14];
		sum[13] = e[16] - e[15];
		sum[14] = e[17] - e[16];
		sum[16] = e[20] - e[19];
		sum[17] = e[21] - e[20];
		sum[18] = e[22] - e[21];
		sum[22] = e[27] - e[26];
#ifdef QT_DEBUG
		CV_Assert(sum[2] >= 0 && sum[6] >= 0 && sum[7] >= 0 && sum[8] >= 0);
		CV_Assert(sum[10] >= 0 && sum[11] >= 0 && sum[12] >= 0);
		CV_Assert(sum[13] >= 0 && sum[14] >= 0 && sum[16] >= 0);
		CV_Assert(sum[17] >= 0 && sum[18] >= 0 && sum[22] >= 0);
#endif
		if (mode > 0) {
			e[0] = s[6] - s[0];
			e[1] = s[7] - s[1];
			e[4] = s[10] - s[4];
			e[5] = s[11] - s[5];
			e[6] = s[12] - s[6];
			e[11] = s[17] - s[11];
			e[18] = s[24] - s[18];
			e[23] = s[29] - s[23];
			e[24] = s[30] - s[24];
			e[25] = s[31] - s[25];
			e[28] = s[34] - s[28];
			e[29] = s[35] - s[29];

			sum[0] = e[1] - e[0];
			sum[4] = e[5] - e[4];
			sum[20] = e[25] - e[24];
			sum[24] = e[29] - e[28];
#ifdef QT_DEBUG
			CV_Assert(sum[0] >= 0 && sum[4] >= 0 && sum[20] >= 0 && sum[24] >= 0);
#endif
			if (mode == 2) {
				sum[1] = e[2] - e[1];
				sum[3] = e[4] - e[3];
				sum[5] = e[7] - e[6];
				sum[9] = e[11] - e[10];
				sum[15] = e[19] - e[18];
				sum[19] = e[23] - e[22];
				sum[21] = e[26] - e[25];
				sum[23] = e[28] - e[27];
#ifdef QT_DEBUG
				CV_Assert(sum[1] >= 0 && sum[3] >= 0 && sum[5] >= 0);
				CV_Assert(sum[9] >= 0 && sum[15] >= 0 && sum[19] >= 0);
				CV_Assert(sum[21] >= 0 && sum[23] >= 0);
#endif
			}
		}
		for (int i = 0; i < 25; i++)
			stats[i].err = -1;
		for (int i = 25; i < (int)stats.size(); i++) {
			stats[i].err = abs(stats[i].sum - sum[stats[i].part]);
			if (stats[stats[i].part].err < 0)
				stats[stats[i].part].err = stats[i].err;
			else
				stats[stats[i].part].err = min(stats[stats[i].part].err, stats[i].err);
		}
#if 0
		for (int i = 0; i < (int)stats.size(); i++)
		if (stats[i].err > 0x20000000 || stats[i].err < 0)
			qCritical("stat[%d], part=%d, espect_sum=%d, sum=%d", i, stats[i].part, stats[i].sum, sum[stats[i].part]);
#endif
		int minimum = 0x7fffffff, minimum2 = 0x7fffffff;
		int bestshape = 0, bestshape2 = 0;
		for (int i = 0; i < (int)sds.size(); i++) {
			int tot_err = 0;
			for (int j = 0; j < (int)sds[i].si.size(); j++) {
				tot_err += stats[sds[i].si[j]].err;
			}
			if (tot_err < minimum) {
				minimum2 = minimum;
				bestshape2 = bestshape;
				minimum = tot_err;
				bestshape = i;
			}
			else
			if (tot_err < minimum2) {
				minimum2 = tot_err;
				bestshape2 = i;
			}
		}
		CV_Assert(minimum >= -8 && minimum2 >= -8);
		int overlap_min = minimum, overlap_min2 = minimum2;
		for (int j = 0; j < (int)sds[bestshape].si.size(); j++) {
			CV_Assert(stats[sds[bestshape].si[j]].part == stats[sds[bestshape2].si[j]].part);
			int shape_need = wire_25_shape[sds[bestshape].w25_shape_idx].ratio[stats[sds[bestshape2].si[j]].part];
			int shape_need2 = wire_25_shape[sds[bestshape2].w25_shape_idx].ratio[stats[sds[bestshape].si[j]].part];
			if (shape_need < 0 || shape_need2 < 0) {
				overlap_min -= stats[sds[bestshape].si[j]].err;
				overlap_min2 -= stats[sds[bestshape2].si[j]].err;
			}
		}

		if (overlap_min > overlap_min2) {
			swap(minimum, minimum2);
			swap(bestshape, bestshape2);
		}

		minimum = minimum * arf;
		minimum2 = minimum2 * arf;
		minimum = max(MIN_SCORE, min(minimum, 65535));
		minimum2 = max(MIN_SCORE, min(minimum2, 65535));
		return MAKE_S(minimum, wp.type, wire_25_shape[sds[bestshape].w25_shape_idx].shape);
	}
};

struct AntStart {
	int dir;
	Point abs_org; //origin non-scale pixel unit
	int w_wide, i_high; //scale image size
	AntStart(int _dir, Point _abs_org, int w, int h) {
		dir = _dir;
		abs_org = _abs_org;
		w_wide = w;
		i_high = h;
	}
};

/*
  Input: ig, iig
  Input: dw
  Input: org
  Output: w_best, best wire parameter
  Output: org_best, best ant point
  scan all possible wire parameter to find best one
  Return best s
*/
unsigned detect_wire_para(Mat & ig, Mat & iig, const DetectWirePara & dw, int & w_best, Point & org_best)
{
	//following compute wp_best and org_best 
	unsigned best_s = 0xffffffff;
	const Point &org = dw.abs_org0;
	Wire_25RectCompute3 wc;
	for (int w = dw.w_min; w <= dw.w_max; w+=3) {		
		WireParameter3 wp(w, dw.gray_w, dw.gray_i, 1);
		wc.prepare(wp, ig, iig);
		for (int dir = 0; dir < 8; dir++) {
			if (dw.dir_mask >> dir & 1)
			for (int d = 0; d <= w / 2 + 1; d++) {
				Point sp = org + Point(dxy[dir][1] * d, dxy[dir][0] * d);
				unsigned s = wc.compute(sp, ig, iig);
				if (s < best_s && (S_SHAPE(s) == BRICK_I_0 || S_SHAPE(s) == BRICK_I_90 || S_SHAPE(s) == BRICK_Z_0 || S_SHAPE(s) == BRICK_Z_90)) {
					best_s = s;
					w_best = w;
					org_best = sp;
				}
			}
		}
	}
	if (best_s != 0xffffffff) {
		int w0 = w_best - 1, w1 = w_best + 1;
		Point org_b = org_best;
		for (int w = w0; w <= w1; w += 2) {
			WireParameter3 wp(w, dw.gray_w, dw.gray_i, 1);
			wc.prepare(wp, ig, iig);
			for (int dir = 0; dir < 8; dir++) {
				if (dw.dir_mask >> dir & 1)
				for (int d = 0; d <= 2; d++) {
					Point sp = org_b + Point(dxy[dir][1] * d, dxy[dir][0] * d);
					unsigned s = wc.compute(sp, ig, iig);
					if (s < best_s && (S_SHAPE(s) == BRICK_I_0 || S_SHAPE(s) == BRICK_I_90 || S_SHAPE(s) == BRICK_Z_0 || S_SHAPE(s) == BRICK_Z_90)) {
						best_s = s;
						w_best = w;
						org_best = sp;
					}
				}
			}
		}
	}
	return best_s;
}

/*
Input: img
Inout: dw, 
  input dw.w_max, dw.org output dw.gray_i, dw.gray_w
*/
void detect_gray(Mat & img,  DetectWirePara & dw)
{
	int ws = max(dw.w_max, DETECT_GRAY_RADIUS);
	Point org = dw.abs_org0;
	QRect dr(org.x - ws, org.y - ws, ws * 2, ws * 2);
	QRect img_rect(0, 0, img.cols, img.rows);
	dr &= img_rect;

	vector<unsigned> bins, th;
	cal_bins(img, dr, bins, 2);
	cal_threshold(bins, th, 0.5, 0.1, 0.1);
	dw.gray_i = th[0];
	dw.gray_w = th[1];
}


/*
Input: img
Inout: dw,
input dw.w_max, dw.org output dw.gray_i, dw.gray_w
*/
void detect_gray2(Mat & img, DetectWirePara & dw)
{
	Mat ig, iig, temp1, temp2;
	integral(img, ig);
	int max_gray = 0;
	int act_len = (DETECT_GRAY2_WIDTH % 2 == 0) ? DETECT_GRAY2_WIDTH : DETECT_GRAY2_WIDTH - 1;
	for (int dir = 0; dir < 4; dir++) {
		int d2 = dir_2[dir];
		Point p0(dw.abs_org0.x - dxy[d2][1] * (DETECT_GRAY2_WIDTH / 2), dw.abs_org0.y - dxy[d2][0] * (DETECT_GRAY2_WIDTH / 2));
		Point p1(dw.abs_org0.x + dxy[d2][1] * (DETECT_GRAY2_WIDTH / 2), dw.abs_org0.y + dxy[d2][0] * (DETECT_GRAY2_WIDTH / 2));
		Point p2 = p0 + Point(dxy[dir][1], dxy[dir][0]);
		Point p3 = p1 + Point(dxy[dir][1], dxy[dir][0]);
		int s0 = abs(ig.at<int>(p0) +ig.at<int>(p3) -ig.at<int>(p1) -ig.at<int>(p2));
		for (int d = 1; d < dw.w_max; d++) {
			p0 = p2;
			p1 = p3;
			p2 = p0 + Point(dxy[dir][1], dxy[dir][0]);
			p3 = p1 + Point(dxy[dir][1], dxy[dir][0]);
			if (p2.x < 0 || p2.y< 0 || p2.x >= ig.cols || p2.y >= ig.rows)
				continue;
			if (p3.x < 0 || p3.y< 0 || p3.x >= ig.cols || p3.y >= ig.rows)
				continue;
			int s1 = abs(ig.at<int>(p0) +ig.at<int>(p3) -ig.at<int>(p1) -ig.at<int>(p2));
			if (max_gray < s0 - s1) {
				max_gray = s0 - s1;
				dw.gray_i = s1 / act_len;
				dw.gray_w = s0 / act_len;
			}
			s0 = s1;
		}
	}
}

class Ant {
public:
	int ant_idx;
protected:
	vector<Point> trails; //non-scaled trail

public:
	Ant(int idx) {
		ant_idx = idx;
	}

	/*
	input as, as.abs_org is start point; as.w_wide is rect width
	input img_width
	input scale
	Return non-scale search rect
	*/
	QRect get_search_rect(const AntStart & as, int img_width, int scale) {
		img_width = img_width << scale;
		int xl = as.abs_org.x / img_width * img_width;
		int yt = as.abs_org.y / img_width * img_width;
		int xr = xl + img_width - (1 << scale);
		int yb = yt + img_width - (1 << scale);
		CV_Assert(as.abs_org.x <= xr && as.abs_org.y <= yb);
		switch (as.dir) {
		case DIR_UP:
			if (as.abs_org.y - yt < 50 << scale)
				yt -= img_width;
			return QRect(QPoint(as.abs_org.x - (as.w_wide << scale), yt),
				QPoint(as.abs_org.x + (as.w_wide << scale), as.abs_org.y + (1<< scale)));
			
		case DIR_DOWN:
			if (yb - as.abs_org.y < 50 << scale)
				yb += img_width;
			return QRect(QPoint(as.abs_org.x - (as.w_wide << scale), as.abs_org.y - (1<<scale)),
				QPoint(as.abs_org.x + (as.w_wide << scale), yb));

		case DIR_LEFT:
			if (as.abs_org.x - xl < 50 << scale)
				xl -= img_width;
			return QRect(QPoint(xl, as.abs_org.y - (as.w_wide << scale)),
				QPoint(as.abs_org.x  + (1<<scale), as.abs_org.y + (as.w_wide << scale)));

		case DIR_RIGHT:
			if (xr - as.abs_org.x < 50 << scale)
				xr += img_width;
			return QRect(QPoint(as.abs_org.x - (1<<scale), as.abs_org.y - (as.w_wide << scale)), 
				QPoint(xr, as.abs_org.y + (as.w_wide << scale)));
		default:
			return QRect(QPoint(xl, yt), QPoint(xr, yb)); //TODO fixme
		}
	}

	/*
	Input ig, iig
	Input lt, lt is img left-top xy non-scale pixel unit
	Input dw, input dw.gray_i, dw.gray_w, dw.gray_th
	input as, input as.w_wide, as.i_high as.abs_org as.dir
	input scale 
	Return brick
	*/
	unsigned go(const Mat & ig, const Mat & iig, Point lt, const DetectWirePara & dw, const AntStart & as) {
		CV_Assert(as.dir < 8);
		int scale = dw.scale;
		trails.clear();
		//1 compute gray_i and gray_w, and then do preprocess
		Point org = as.abs_org - lt;
		org.x = org.x >> scale;
		org.y = org.y >> scale;
		qDebug("Ant%d begin at (x=%d,y=%d), gray_i=%d, gray_w=%d,w=%d,i=%d", ant_idx, as.abs_org.x, as.abs_org.y, 
			dw.gray_i, dw.gray_w, as.w_wide, as.i_high);
		Point oxy[3][2];
		int offset[3][4];
		switch (as.dir) {
		case DIR_UP:
			oxy[0][0] = Point(-as.w_wide / 2, -as.i_high); //oxy[0] is wire
			oxy[0][1] = Point(0, 0);
			oxy[1][0] = Point(0, -as.i_high);
			oxy[1][1] = Point(as.w_wide - as.w_wide / 2, 0);
			oxy[2][0] = Point(-as.w_wide / 4, -as.i_high);
			oxy[2][1] = Point(as.w_wide / 2 - as.w_wide / 4, 0);
			break;
		case DIR_DOWN:
			oxy[0][0] = Point(-as.w_wide / 2, 0);
			oxy[0][1] = Point(0, as.i_high);
			oxy[1][0] = Point(0, 0);
			oxy[1][1] = Point(as.w_wide - as.w_wide / 2, as.i_high);
			oxy[2][0] = Point(-as.w_wide / 4, 0);
			oxy[2][1] = Point(as.w_wide / 2 - as.w_wide / 4, as.i_high);
			break;
		case DIR_LEFT:
			oxy[0][0] = Point(-as.i_high, -as.w_wide / 2);
			oxy[0][1] = Point(0, 0);
			oxy[1][0] = Point(-as.i_high, 0);
			oxy[1][1] = Point(0, as.w_wide - as.w_wide / 2);
			oxy[2][0] = Point(-as.i_high, -as.w_wide / 4);
			oxy[2][1] = Point(0, as.w_wide / 2 - as.w_wide / 4);
			break;
		case DIR_RIGHT:
			oxy[0][0] = Point(0, -as.w_wide / 2);
			oxy[0][1] = Point(as.i_high, 0);
			oxy[1][0] = Point(0, 0);
			oxy[1][1] = Point(as.i_high, as.w_wide - as.w_wide / 2);
			oxy[2][0] = Point(0, -as.w_wide / 4);
			oxy[2][1] = Point(as.i_high, as.w_wide / 2 - as.w_wide / 4);
			break;
		default:
			qCritical("not support dir %d", as.dir);
			break;
		}
		for (int i = 0; i < 3; i++) { //offset0,1 is side rect, offset2 is middle rect
			offset[i][0] = (oxy[i][0].y * (int)ig.step.p[0] + oxy[i][0].x * (int)ig.step.p[1]) / sizeof(int);
			offset[i][1] = (oxy[i][0].y * (int)ig.step.p[0] + oxy[i][1].x * (int)ig.step.p[1]) / sizeof(int);
			offset[i][2] = (oxy[i][1].y * (int)ig.step.p[0] + oxy[i][0].x * (int)ig.step.p[1]) / sizeof(int);
			offset[i][3] = (oxy[i][1].y * (int)ig.step.p[0] + oxy[i][1].x * (int)ig.step.p[1]) / sizeof(int);
		}		
		int th0 = as.i_high * (as.w_wide / 2) * dw.gray_i + as.i_high * (as.w_wide / 2) * (dw.gray_w - dw.gray_i) * (dw.gray_th / 100.0); //th0 is wire threshold
		int th1 = as.i_high * (as.w_wide - as.w_wide / 2) * dw.gray_i + as.i_high * (as.w_wide - as.w_wide / 2) * (dw.gray_w - dw.gray_i) * (dw.gray_th / 100.0); //th0 is wire threshold
		int th2 = th0;
		while(1) {
			if ((as.dir == DIR_LEFT && org.x < as.i_high) || (as.dir == DIR_UP && org.y < as.i_high) ||
				(as.dir == DIR_RIGHT && org.x >= ig.cols - as.i_high) || (as.dir == DIR_DOWN && org.y >= ig.rows - as.i_high))
				return (as.dir == DIR_UP || as.dir == DIR_DOWN) ? BRICK_I_0 : BRICK_I_90; //wire outside
			const unsigned * p_ig = ig.ptr<unsigned>(org.y, org.x);
			int sum0 = p_ig[offset[0][3]] + p_ig[offset[0][0]] - p_ig[offset[0][1]] - p_ig[offset[0][2]];
			int sum1 = p_ig[offset[1][3]] + p_ig[offset[1][0]] - p_ig[offset[1][1]] - p_ig[offset[1][2]];
			int sum2 = p_ig[offset[2][3]] + p_ig[offset[2][0]] - p_ig[offset[2][1]] - p_ig[offset[2][2]];
			if (sum0 < th0 || sum1 < th1 || sum2 < th2) 
				return BRICK_NO_WIRE; //wire break
			trails.push_back(org * (1<<scale) + lt);
			org += Point(dxy[as.dir][1], dxy[as.dir][0]);			
		}
	}

	//return 0 if success
	int get_path(pair<Point, Point> & line) {
		if (trails.empty() || trails.size() < 2)
			return -1;
		line.first = trails.front();
		line.second = trails.back();
		return 0;
	}
};

VWExtractAnt::VWExtractAnt()
{
	layer0 = -1;
}

Mat VWExtractAnt::color2gray(Mat & color_img)
{
	CV_Assert(color_img.channels() == 3);
	Mat img;
	Mat mv[4];
	split(color_img, mv);
    if (dw0.channel < 3) {
        img = mv[dw0.channel];
        return img;
    }
	img.create(color_img.rows, color_img.cols, CV_8UC1);
    int sum = dw0.cb + dw0.cg + dw0.cr;
	for (int y = 0; y < img.rows; y++) {
		unsigned char * praw = img.ptr<unsigned char>(y);
		unsigned char * pb = mv[0].ptr<unsigned char>(y);
		unsigned char * pg = mv[1].ptr<unsigned char>(y);
		unsigned char * pr = mv[2].ptr<unsigned char>(y);
		for (int x = 0; x < img.cols; x++) {
            int temp;
            if (pb[x] + pg[x] + pr[x] <= sum)
                temp = (pb[x] * dw0.cb + pg[x] * dw0.cg + pr[x] * dw0.cr) >> 8;
            else {
                float ratio = (float) sum / ((pb[x] + pg[x] + pr[x]) * 256);
                temp = ratio * (pb[x] * dw0.cb + pg[x] * dw0.cg + pr[x] * dw0.cr);
            }
            praw[x] = min(255, temp);
		}
	}
	return img;
}
/*
input ic_layer
input scale, 0,1,2,3...
input rect, non-scale pixel unit
Return image
*/
Mat VWExtractAnt::prepare_img(ICLayerWrInterface * ic_layer, int scale, QRect rect, bool replace_buf)
{
	if (prev_scale != scale) {
		img_bufs.clear();
		prev_scale = scale;
	}
	if (replace_buf) {
		for (int i = 0; i < img_bufs.size(); i++)
			img_bufs[i].replace = true;
	}
	Mat img((rect.height() >> scale), (rect.width() >> scale), CV_8UC1);
	int xl = (rect.left() / ic_layer->getBlockWidth() >> scale) << scale; //xl is picture idx x
	int yt = (rect.top() / ic_layer->getBlockWidth() >> scale) << scale;
	int xr = (rect.right() / ic_layer->getBlockWidth() >> scale) << scale;
	int yb = (rect.bottom() / ic_layer->getBlockWidth() >> scale) << scale;
	bool right_hit = false, bottom_hit = false;
	for (int y = yt; y <= yb; y += 1<<scale) //both y & x is picture idx
	for (int x = xl; x <= xr; x += 1<<scale) {
		int idx = -1;
		Mat raw_img;
		for (int i = 0; i < img_bufs.size(); i++)
			if (img_bufs[i].lt == Point(x, y)) {
				qDebug("load img from buffer s=%d, x=%d,y=%d", scale, x, y);
				idx = i;
				img_bufs[i].replace = false;
				raw_img = img_bufs[i].raw_img;
				break;
			}
		if (idx < 0) {
			vector<uchar> encode_img;
			qDebug("load img from database scale=%d, x=%d,y=%d", scale, x, y);
			if (ic_layer->getRawImgByIdx(encode_img, x, y, scale, 0, true) != 0) {
				qCritical("load image error at s=%d, (%d,%d)", scale, x, y);
				return Mat();
			}
			Mat dec_img = imdecode(Mat(encode_img), -1);
			if (dec_img.channels() > 1) {
				if (dec_img.channels() != 3) {
					qCritical("load image error, channel =%d, dw0.channel =%d", dec_img.channels(), dw0.channel);
					return Mat();
				}
                dw0.is_color = (dw0.channel >= 3) ? 3 : 0;
                if (dw0.cr < -299 && dw0.cg < -299 && dw0.channel >= 3) { //auto extract color
					if (img.type() != CV_8UC3)
						img.create(img.rows, img.cols, CV_8UC3);
					raw_img = dec_img;
				}
				else
					raw_img = color2gray(dec_img);
			}
			else
				raw_img = dec_img;
            if (replace_buf && !dw0.is_color)
				img_bufs.push_back(ImageBuf(Point(x, y), raw_img));
		}
		//now raw_img is loaded, copy it to img;
		QRect raw_img_rect(x *ic_layer->getBlockWidth(), y *ic_layer->getBlockWidth(), ic_layer->getBlockWidth() << scale, ic_layer->getBlockWidth() << scale);
		QRect overlap_rect = raw_img_rect & rect; //overlap rect is the copy rect, unit is pixel
		QRect src = overlap_rect.translated(- raw_img_rect.topLeft());
		QRect tgt = overlap_rect.translated(- rect.topLeft());
		CV_Assert((tgt.left() >> scale) + (tgt.width() >> scale) <= img.cols &&
			(tgt.top() >> scale) + (tgt.height() >> scale) <= img.rows);
		CV_Assert((src.left() >> scale) + (src.width() >> scale) <= raw_img.cols &&
			(src.top() >> scale) + (src.height() >> scale) <= raw_img.rows);
		if ((tgt.left() >> scale) + (tgt.width() >> scale) == img.cols)
			right_hit = true;
		if ((src.top() >> scale) + (src.height() >> scale) == raw_img.rows)
			bottom_hit = true;
		if ((src.width() >> scale) == 0 || (src.height() >> scale) == 0)
			continue;
		raw_img(Rect(src.left() >> scale, src.top() >> scale, src.width() >> scale, src.height() >> scale)).copyTo(
			img(Rect(tgt.left() >> scale, tgt.top() >> scale, tgt.width() >> scale, tgt.height() >> scale)));
	}
	if (!right_hit && !bottom_hit)
		img = img(Rect(0, 0, img.cols - 1, img.rows - 1));
	if (!right_hit && bottom_hit)
		img = img(Rect(0, 0, img.cols - 1, img.rows));
	if (right_hit && !bottom_hit)
		img = img(Rect(0, 0, img.cols, img.rows - 1));
    if (dw0.cr < -299 && dw0.cg < -299 && dw0.is_color) { //auto extract color
		unsigned char * pcolor = img.ptr<unsigned char>(img.rows / 2, img.cols / 2);
		dw0.cb = pcolor[0] + pcolor[-3] + pcolor[3];
		dw0.cg = pcolor[1] + pcolor[-2] + pcolor[4];
		dw0.cr = pcolor[2] + pcolor[-1] + pcolor[5];
		int sum = dw0.cb + dw0.cg + dw0.cr;
        if (sum==0) {
            dw0.cb = 100;
            dw0.cg = 100;
            dw0.cr = 100;
        } else {
            dw0.cb = dw0.cb * 300 / sum;
            dw0.cg = dw0.cg * 300 / sum;
            dw0.cr = dw0.cr * 300 / sum;
        }

		qInfo("change cr=%d, cg=%d, cb=%d", dw0.cr, dw0.cg, dw0.cb);
		img = color2gray(img);
		imwrite("grayimg.jpg", img);
    }
	CV_Assert(img.type() == CV_8UC1);
	if (replace_buf) {
		for (int i = 0; i < img_bufs.size(); i++) {
			while (img_bufs[i].replace && !img_bufs[i].must_reserve) {
				if (i != img_bufs.size() - 1) { //not the last one
					img_bufs[i] = img_bufs.back(); //replace with last one
					img_bufs.pop_back(); //delete last one
				}
				else { //it is the last one
					img_bufs.pop_back(); //delete last one
					break;
				}					
			}
		}
	}
	return img;
}

/*
	31..24 23..16 15..8 7..0
pi1       w_max     w_min
pi2 channel gray_th search_opt i_high
pi3			cg		cr	scale
*/
int VWExtractAnt::set_extract_param(int layer, int, int pi1, int pi2, int pi3, int , int, int, int, float)
{
	dw0.w_min = pi1 & 0xffff;
	dw0.w_max = pi1 >> 16 & 0xffff;
	dw0.i_high = pi2 & 0xff;
	search_opt = pi2 >> 8 & 0xff;
	dw0.gray_th = pi2 >> 16 & 0xff;
	dw0.dir_mask = (search_opt & SEARCH_DIR_MASK) ? 0xff : 0xf;
	dw0.channel = pi2 >> 24 & 0xff;
	dw0.w_max = max(dw0.w_min, dw0.w_max);
	dw0.gray_th = min(dw0.gray_th, 80);
	dw0.gray_th = max(20, dw0.gray_th);
	dw0.i_high = min(dw0.i_high, 9);
	dw0.i_high = max(2, dw0.i_high);
	dw0.channel = min(3, dw0.channel);
	dw0.scale = pi3 & 0xff;
    dw0.cr = pi3 >> 8 & 0xff;
    dw0.cg = pi3 >> 16 & 0xff;
    if (dw0.cr >= 128)
        dw0.cr = -dw0.cr;
    if (dw0.cg >= 128)
        dw0.cg = -dw0.cg;
    float cr = dw0.cr / 100.0;
    cr = max(-1.0f, min(1.0f, cr));
    float cg = dw0.cg / 100.0;
    cg = max(-1.0f, min(1.0f, cg));
    float cb = 1 - cr - cg;
    cb = max(-1.0f, min(1.0f, cb));
    dw0.cr = cr * 300;
    dw0.cg = cg * 300;
    dw0.cb = cr * 300;

	if (layer0 != layer) {
		img_bufs.clear();
		layer0 = layer;
	}
    qInfo("VWExtractAnt set param: w_min=%d, w_max=%d, channel=%d, dir_mask=0x%x, gray_th=%d, i_high=%d, scale=%d, cr=%d,cg=%d,cb=%d",
          dw0.w_min, dw0.w_max, dw0.channel, dw0.dir_mask, dw0.gray_th, dw0.i_high, dw0.scale, dw0.cr, dw0.cg, dw0.cb);
	return 0;
}

/*
input shape current shape
input as, coming dir
output next_dir
*/
void get_next_dir(int shape, AntStart as, vector <int> & next_dir)
{
	next_dir.clear();
	switch (shape) {
	case BRICK_L_0:
		CV_Assert(as.dir == DIR_DOWN || as.dir == DIR_LEFT);
		if (as.dir == DIR_DOWN) 
			next_dir.push_back(DIR_RIGHT);
		if (as.dir == DIR_LEFT)
			next_dir.push_back(DIR_UP);
		break;
	case BRICK_L_90:
		CV_Assert(as.dir == DIR_LEFT || as.dir == DIR_UP);
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_DOWN);
		}
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_RIGHT);
		}
		break;
	case BRICK_L_180:
		CV_Assert(as.dir == DIR_RIGHT || as.dir == DIR_UP);
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_DOWN);
		}
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_LEFT);
		}
		break;
	case BRICK_L_270:
		CV_Assert(as.dir == DIR_RIGHT || as.dir == DIR_DOWN);
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_UP);
		}
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_LEFT);
		}
		break;
	case BRICK_T_0:
		CV_Assert(as.dir == DIR_LEFT || as.dir == DIR_DOWN || as.dir == DIR_UP);
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_DOWN);
		}
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_RIGHT);
			next_dir.push_back(DIR_UP);
		}
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_RIGHT);
			next_dir.push_back(DIR_DOWN);
		}
		break;
	case BRICK_T_90:
		CV_Assert(as.dir == DIR_LEFT || as.dir == DIR_RIGHT || as.dir == DIR_UP);
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_DOWN);
		}
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_RIGHT);
			next_dir.push_back(DIR_DOWN);
		}
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_RIGHT);
		}
		break;
	case BRICK_T_180:
		CV_Assert(as.dir == DIR_RIGHT || as.dir == DIR_UP || as.dir == DIR_DOWN);
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_DOWN);
		}
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_UP);
		}
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_DOWN);
		}
		break;
	case BRICK_T_270:
		CV_Assert(as.dir == DIR_RIGHT || as.dir == DIR_DOWN || as.dir == DIR_LEFT);
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_RIGHT);
		}
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_UP);
		}
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_RIGHT);
		}
		break;
	case BRICK_X_0:
		CV_Assert(as.dir == DIR_RIGHT || as.dir == DIR_DOWN || as.dir == DIR_LEFT || as.dir == DIR_UP);
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_DOWN);
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_RIGHT);
		}
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_DOWN);
		}
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_RIGHT);
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_DOWN);
		}
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_RIGHT);
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_UP);
		}
		break;
	case BRICK_J_0:
		CV_Assert(as.dir == DIR_DOWN || as.dir == DIR_UPRIGHT);
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_DOWNLEFT);
		}
		if (as.dir == DIR_UPRIGHT) {
			next_dir.push_back(DIR_UP);
		}
		break;
	case BRICK_J_90:
		CV_Assert(as.dir == DIR_LEFT || as.dir == DIR_DOWNRIGHT);
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_UPLEFT);
		}
		if (as.dir == DIR_DOWNRIGHT) {
			next_dir.push_back(DIR_RIGHT);
		}
		break;
	case BRICK_J_180:
		CV_Assert(as.dir == DIR_UP || as.dir == DIR_DOWNLEFT);
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_UPRIGHT);
		}
		if (as.dir == DIR_DOWNLEFT) {
			next_dir.push_back(DIR_DOWN);
		}
		break;
	case BRICK_J_270:
		CV_Assert(as.dir == DIR_RIGHT || as.dir == DIR_UPLEFT);
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_DOWNRIGHT);
		}
		if (as.dir == DIR_UPLEFT) {
			next_dir.push_back(DIR_LEFT);
		}
		break;
	case BRICK_l_0:
		CV_Assert(as.dir == DIR_DOWN || as.dir == DIR_UPLEFT);
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_DOWNRIGHT);
		}			
		if (as.dir == DIR_UPLEFT) {
			next_dir.push_back(DIR_UP);
		}
		break;
	case BRICK_l_90:
		CV_Assert(as.dir == DIR_LEFT || as.dir == DIR_UPRIGHT);
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_DOWNLEFT);
		}
		if (as.dir == DIR_UPRIGHT) {
			next_dir.push_back(DIR_RIGHT);
		}
		break;
	case BRICK_l_180:
		CV_Assert(as.dir == DIR_UP || as.dir == DIR_DOWNRIGHT);
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_UPLEFT);
		}
		if (as.dir == DIR_DOWNRIGHT) {
			next_dir.push_back(DIR_DOWN);
		}
		break;
	case BRICK_l_270:
		CV_Assert(as.dir == DIR_RIGHT || as.dir == DIR_DOWNLEFT);
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_UPRIGHT);
		}
		if (as.dir == DIR_DOWNLEFT) {
			next_dir.push_back(DIR_LEFT);
		}
		break;
	case BRICK_Y_45:
		CV_Assert(as.dir == DIR_LEFT || as.dir == DIR_DOWN || as.dir == DIR_UPRIGHT);
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_DOWNLEFT);
		}
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_RIGHT);
			next_dir.push_back(DIR_DOWNLEFT);
		}
		if (as.dir == DIR_UPRIGHT) {
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_RIGHT);
		}
		break;
	case BRICK_Y_135:
		CV_Assert(as.dir == DIR_LEFT || as.dir == DIR_UP || as.dir == DIR_DOWNRIGHT);
		if (as.dir == DIR_LEFT) {
			next_dir.push_back(DIR_DOWN);
			next_dir.push_back(DIR_UPLEFT);
		}
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_RIGHT);
			next_dir.push_back(DIR_UPLEFT);
		}
		if (as.dir == DIR_DOWNRIGHT) {
			next_dir.push_back(DIR_DOWN);
			next_dir.push_back(DIR_RIGHT);
		}
		break;
	case BRICK_Y_225:
		CV_Assert(as.dir == DIR_RIGHT || as.dir == DIR_UP || as.dir == DIR_DOWNLEFT);
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_DOWN);
			next_dir.push_back(DIR_UPRIGHT);
		}
		if (as.dir == DIR_UP) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_UPRIGHT);
		}
		if (as.dir == DIR_DOWNLEFT) {
			next_dir.push_back(DIR_DOWN);
			next_dir.push_back(DIR_LEFT);
		}
		break;
	case BRICK_Y_315:
		CV_Assert(as.dir == DIR_DOWN || as.dir == DIR_RIGHT || as.dir == DIR_UPLEFT);
		if (as.dir == DIR_DOWN) {
			next_dir.push_back(DIR_LEFT);
			next_dir.push_back(DIR_DOWNRIGHT);
		}
		if (as.dir == DIR_RIGHT) {
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_DOWNRIGHT);
		}
		if (as.dir == DIR_UPLEFT) {
			next_dir.push_back(DIR_UP);
			next_dir.push_back(DIR_LEFT);
		}
		break;
	}
}

int VWExtractAnt::extract(string file_name, QRect rect, vector<MarkObj> & obj_sets)
{
	Mat img = imread(file_name, 0);
	Mat detect_img, ig, iig;
	Point org_best;
	int w_best;
	dw0.abs_org0 = Point(rect.x(), rect.y());
	detect_gray(img, dw0);
	qDebug("detect_gray, gray_i=%d, gray_w=%d", dw0.gray_i, dw0.gray_w);
	clip_img(img, dw0.gray_i, dw0.gray_w, detect_img);
	Mat temp1, temp2;
	integral_square(detect_img, ig, iig, temp1, temp2, false);
	dw0.gray_w -= dw0.gray_i;
	dw0.gray_i = 0;
	unsigned s = detect_wire_para(ig, iig, dw0, w_best, org_best);
	if (S_SHAPE(s) == BRICK_INVALID)
		return -1;
	qDebug("detect_wire_para, w_best=%d, org_best=(x=%d,y=%d), s=%x", w_best, org_best.x, org_best.y, s);
	obj_sets.clear();

	vector <AntStart> asq;
	switch (S_SHAPE(s)) {
	case BRICK_I_0:
		asq.push_back(AntStart(DIR_UP, org_best, w_best, dw0.i_high));
		asq.push_back(AntStart(DIR_DOWN, org_best, w_best, dw0.i_high));
		break;
	case BRICK_I_90:
		asq.push_back(AntStart(DIR_RIGHT, org_best, w_best, dw0.i_high));
		asq.push_back(AntStart(DIR_LEFT, org_best, w_best, dw0.i_high));
		break;
	case BRICK_Z_0:
		asq.push_back(AntStart(DIR_UPRIGHT, org_best, w_best, dw0.i_high));
		asq.push_back(AntStart(DIR_DOWNLEFT, org_best, w_best, dw0.i_high));
		break;
	case BRICK_Z_90:
		asq.push_back(AntStart(DIR_DOWNRIGHT, org_best, w_best, dw0.i_high));
		asq.push_back(AntStart(DIR_UPLEFT, org_best, w_best, dw0.i_high));
		break;
	default:
		qWarning("click shape %d", S_SHAPE(s));
		break;
	}
	Ant ant0(0);
	pair<Point, Point> lines[2];
	for (int i = 0; i < 2; i++) {
		lines[i].first = org_best;
		lines[i].second = org_best;
		ant0.go(ig, iig, Point(0, 0), dw0, asq[i]);
		ant0.get_path(lines[i]);
	}
	MarkObj wire;
	wire.p0 = QPoint(lines[0].second.x, lines[0].second.y);
	wire.p1 = QPoint(lines[1].second.x, lines[1].second.y);
	wire.prob = 1;
	wire.type = OBJ_LINE;
	wire.type2 = LINE_WIRE_AUTO_EXTRACT;
	wire.type3 = layer0;
	wire.state = 0;
	obj_sets.push_back(wire);
	return 0;
}

int VWExtractAnt::extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & _area, vector<MarkObj> & obj_sets)
{
	obj_sets.clear();
	Point org = Point(_area[0].rect.x(), _area[0].rect.y());
	org.x = org.x / (32768 / ic_layer[0]->getBlockWidth());
	org.y = org.y / (32768 / ic_layer[0]->getBlockWidth());
	DetectWirePara dw1 = dw0; //backup dw0 for restore before return
	//change non-scale pixel w_min, w_max to scale pixel unit
	dw0.w_min = dw0.w_min >> dw0.scale;
	dw0.w_max = dw0.w_max >> dw0.scale;
	dw0.w_min = max(3, dw0.w_min);
	dw0.w_max = max(dw0.w_min, dw0.w_max);
	dw0.w_max = min(DETECT_WIRE_RADIUS, dw0.w_max);
	dw0.w_min = min(dw0.w_min, dw0.w_max);
	dw0.abs_org0 = Point(DETECT_WIRE_RADIUS, DETECT_WIRE_RADIUS);
	dw0.is_color = 0;
	for (int i = 0; i < img_bufs.size(); i++)
		img_bufs[i].must_reserve = false;
	Mat img = prepare_img(ic_layer[0], dw0.scale, QRect(org.x - (DETECT_WIRE_RADIUS << dw0.scale), org.y - (DETECT_WIRE_RADIUS << dw0.scale), 
		DETECT_WIRE_RADIUS << (1 + dw0.scale), DETECT_WIRE_RADIUS << (1 + dw0.scale)), true);
	for (int i = 0; i < img_bufs.size(); i++)
		img_bufs[i].must_reserve = true;
	if (img.empty()) {
		dw0 = dw1;
		return -1;
	}
	Mat detect_img, ig, iig;
	Point org_best;
	int w_best;
	if (dw0.is_color)
		detect_gray2(img, dw0);
	else
		detect_gray(img, dw0);
	qDebug("detect_gray, gray_i=%d, gray_w=%d", dw0.gray_i, dw0.gray_w);
	dw1.gray_w = dw0.gray_w;
	dw1.gray_i = dw0.gray_i;
	clip_img(img, dw0.gray_i, dw0.gray_w, detect_img);	
	Mat temp1, temp2;
	integral_square(detect_img, ig, iig, temp1, temp2, false);
	dw0.gray_w -= dw0.gray_i;
	dw0.gray_i = 0;
	unsigned s = detect_wire_para(ig, iig, dw0, w_best, org_best);
	if (S_SHAPE(s) == BRICK_INVALID) {
		dw0 = dw1;
		return -1;
	}
	org_best.x = org.x - (DETECT_WIRE_RADIUS << dw0.scale) + (org_best.x << dw0.scale);
	org_best.y = org.y - (DETECT_WIRE_RADIUS << dw0.scale) + (org_best.y << dw0.scale);
	qDebug("detect_wire_para, scale=%d, w_best=%d, org_best=(x=%d,y=%d), s=%x", dw0.scale, w_best << dw0.scale, org_best.x, org_best.y, s);
	if (w_best <= 6 && dw0.scale > 0) { //zoom in 
		vector<SearchArea> areas(_area);
		dw0 = dw1;
		dw0.w_min = w_best << dw0.scale; //change back to non-scale
		dw0.w_max = w_best << dw0.scale; //use exact w_best to reduce detect_wire_para time
		dw0.scale--;
		int ret = extract(ic_layer, areas, obj_sets);
		dw0 = dw1;
		return ret;
	}
	if (w_best >= 32 && dw0.scale < ic_layer[0]->getMaxScale()) { //zoom out
		vector<SearchArea> areas(_area);
		dw0 = dw1;
		dw0.w_min = w_best << dw0.scale; //change back to non-scale
		dw0.w_max = w_best << dw0.scale; //use exact w_best to reduce detect_wire_para time
		while (w_best >= 32 && dw0.scale < ic_layer[0]->getMaxScale()) {			
			dw0.scale++;
			w_best = w_best >> 1;
		}
		int ret = extract(ic_layer, areas, obj_sets);
		dw0 = dw1;
		return ret;
	}
	vector <AntStart> asq;
	switch (S_SHAPE(s)) {
	case BRICK_I_0:
		asq.push_back(AntStart(DIR_UP, org_best, w_best, dw0.i_high));
		asq.push_back(AntStart(DIR_DOWN, org_best, w_best, dw0.i_high));
		break;
	case BRICK_I_90:
		asq.push_back(AntStart(DIR_RIGHT, org_best, w_best, dw0.i_high));
		asq.push_back(AntStart(DIR_LEFT, org_best, w_best, dw0.i_high));
		break;
	case BRICK_Z_0:
		asq.push_back(AntStart(DIR_UPRIGHT, org_best, w_best, dw0.i_high));
		asq.push_back(AntStart(DIR_DOWNLEFT, org_best, w_best, dw0.i_high));
		break;
	case BRICK_Z_90:
		asq.push_back(AntStart(DIR_DOWNRIGHT, org_best, w_best, dw0.i_high));
		asq.push_back(AntStart(DIR_UPLEFT, org_best, w_best, dw0.i_high));
		break;
	default:
		qWarning("click shape %d", S_SHAPE(s));
		break;
	}
	Ant ant0(0);
	pair<Point, Point> lines[2];
	for (int i = 0; i < 2; i++) {
		unsigned s;
		do {
			QRect srect = ant0.get_search_rect(asq[i], ic_layer[0]->getBlockWidth(), dw0.scale);
			qDebug("SearchRect=(l=%d,t=%d,r=%d,b=%d),start=(x=%d,y=%d)", srect.left(), srect.top(),
				srect.right(), srect.bottom(), asq[i].abs_org.x, asq[i].abs_org.y);

			img = prepare_img(ic_layer[0], dw0.scale, srect, true);
			clip_img(img, dw1.gray_i, dw1.gray_w, detect_img);
			integral_square(detect_img, ig, iig, temp1, temp2, false);

			s = ant0.go(ig, iig, Point(srect.left(), srect.top()), dw0, asq[i]);
			lines[i].first = asq[i].abs_org;
			lines[i].second = asq[i].abs_org;
			ant0.get_path(lines[i]);
			asq[i].abs_org = lines[i].second;
		} while (s != BRICK_NO_WIRE);
	}
	qInfo("extract single wire (%d,%d)->(%d,%d)", asq[0].abs_org.x, asq[0].abs_org.y, asq[1].abs_org.x, asq[1].abs_org.y);
	MarkObj wire;
	asq[0].abs_org = asq[0].abs_org * (32768 / ic_layer[0]->getBlockWidth());
	asq[1].abs_org = asq[1].abs_org * (32768 / ic_layer[0]->getBlockWidth());
	wire.p0 = QPoint(asq[0].abs_org.x, asq[0].abs_org.y);
	wire.p1 = QPoint(asq[1].abs_org.x, asq[1].abs_org.y);
	wire.prob = 1;
	wire.type = OBJ_LINE;
	wire.type2 = LINE_WIRE_AUTO_EXTRACT;
	wire.type3 = layer0;
	wire.state = 0;
	obj_sets.push_back(wire);
	dw0 = dw1;
	return 0;
}
void VWExtractAnt::get_feature(int, int, int, std::vector<float> &, std::vector<int> &)
{
	return;
}
