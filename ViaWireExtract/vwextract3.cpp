#include "vwextract3.h"
#include "vwextract_public.h"

#define S_SCORE(p) ((int)((p) >> 16 & 0xffff))
#define S_TYPE(p) ((unsigned)((p) >> 8 & 0xff))
#define S_SHAPE(p) ((unsigned)((p) & 0xff))
#define MAKE_S(score, type, shape) ((unsigned)(score) << 16 | (unsigned)(type) << 8 | (shape))
#define K_DISTORT1 1
#define K_DISTORT2 6
#define K_DISTORT3 20
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
	{ { -1, 00, 00, 00, -1, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, -1, 00, 00, 00, -1 }, BRICK_NO_WIRE },
	{ { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, .8, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_i_0 },
	{ { -1, -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 00, .8, 01, 01, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_i_90 },
	{ { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, .8, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_i_180 },
	{ { -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 01, 01, .8, 00, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_i_270 },
	{ { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_I_0 },
	{ { -1, -1, -1, -1, -1, 00, 00, 00, 00, 00, 01, 01, 01, 01, 01, 00, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_I_90 },
	{ { -1, 00, 01, 00, -1, -1, 00, 01, 00, 00, -1, 00, 01, 01, 01, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_L_0 },
	{ { -1, -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 00, 01, 01, 01, -1, 00, 01, 00, 00, -1, 00, 01, 00, -1 }, BRICK_L_90 },
	{ { -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 01, 01, 01, 00, -1, 00, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_L_180 },
	{ { -1, 00, 01, 00, -1, 00, 00, 01, 00, -1, 01, 01, 01, 00, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_L_270 },
	{ { -1, 01, 01, 00, -1, -1, 01, 01, 00, -1, 00, 01, 01, 00, -1, -1, 01, 01, 00, -1, -1, 01, 01, 00, -1 }, BRICK_II_0 },
	{ { -1, -1, -1, -1, -1, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 00, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_II_90 },
	{ { -1, 00, 01, 01, -1, -1, 00, 01, 01, -1, -1, 00, 01, 01, -1, -1, 00, 01, 01, -1, -1, 00, 01, 01, -1 }, BRICK_II_180 },
	{ { -1, -1, -1, -1, -1, 00, 00, 00, 00, 00, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, -1, -1, -1, -1, -1 }, BRICK_II_270 },
	{ { -1, 01, 01, 01, -1, 00, 01, 01, 01, 00, 00, 01, 01, 01, 00, 00, 01, 01, 01, 00, -1, 01, 01, 01, -1 }, BRICK_III_0 },
	{ { -1, 00, 00, 00, -1, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, -1, 00, 00, 00, -1 }, BRICK_III_90 },
	{ { -1, -1, -1, -1, -1, -1, -1, 01, -1, -1, -1, .3, .3, .3, -1, -1, -1, 01, -1, -1, -1, -1, -1, -1, -1 }, BRICK_HOLLOW },
	{ { -1, -1, -1, -1, -1, -1, -1, .3, -1, -1, -1, 01, .3, 01, -1, -1, -1, .3, -1, -1, -1, -1, -1, -1, -1 }, BRICK_HOLLOW },
	{ { -1, 00, 01, 00, -1, -1, 00, 01, 00, 00, -1, 00, 01, 01, 01, -1, 00, 01, 00, 00, -1, 00, 01, 00, -1 }, BRICK_T_0 },
	{ { -1, -1, -1, -1, -1, 00, 00, 00, 00, 00, 01, 01, 01, 01, 01, 00, 00, 01, 00, 00, -1, 00, 01, 00, -1 }, BRICK_T_90 },
	{ { -1, 00, 01, 00, -1, 00, 00, 01, 00, -1, 01, 01, 01, 00, -1, 00, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_T_180 },
	{ { -1, 00, 01, 00, -1, 00, 00, 01, 00, 00, 01, 01, 01, 01, 01, 00, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_T_270 },
	{ { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, 01, 01, 01, 01, 01, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_X_0 },
	//   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
	{ { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, .1, .5, .9, 00, -1, 01, 01, .5, 00, -1, 01, 01, .1, -1, -1 }, BRICK_J_0 },
	{ { 01, 01, .1, -1, -1, 01, 01, .5, 00, 00, .1, .5, .9, 01, 01, -1, 00, 00, 00, 00, -1, -1, -1, -1, -1 }, BRICK_J_90 },
	{ { -1, -1, .1, 01, 01, -1, 00, .5, 01, 01, -1, 00, .9, .5, .1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_J_180 },
	{ { -1, -1, -1, -1, -1, 00, 00, 00, 00, -1, 01, 01, .9, .5, .1, 00, 00, .5, 01, 01, -1, -1, .1, 01, 01 }, BRICK_J_270 },

	{ { -1, 00, 01, 00, -1, -1, 00, 01, 00, -1, -1, 00, .9, .5, .1, -1, 00, .5, 01, 01, -1, -1, .1, 01, 01 }, BRICK_l_0 },
	{ { -1, -1, -1, -1, -1, -1, 00, 00, 00, 00, .1, .5, .9, 01, 01, 01, 01, .5, 00, 00, 01, 01, .1, -1, -1 }, BRICK_l_90 },
	{ { 01, 01, .1, -1, -1, 01, 01, .5, 00, -1, .1, .5, .9, 00, -1, -1, 00, 01, 00, -1, -1, 00, 01, 00, -1 }, BRICK_l_180 },
	{ { -1, -1, .1, 01, 01, 00, 00, .5, 01, 01, 01, 01, .9, .5, .1, 00, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_l_270 },

	{ { -1, -1, .1, 01, 01, -1, 00, .5, 01, 01, .1, .5, .9, .5, .1, 01, 01, .5, 00, -1, 01, 01, .1, -1, -1 }, BRICK_Z_0 },
	{ { 01, 01, .1, -1, -1, 01, 01, .5, 00, -1, .1, .5, .9, .5, .1, -1, 00, .5, 01, 01, -1, -1, .1, 01, 01 }, BRICK_Z_90 },
	
	{ { -1, -1, .1, 01, 01, -1, 00, .5, 01, 01, -1, 00, .9, .5, .1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_P_0 },	    
	{ { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, -1, 00, .9, .5, .1, -1, 00, .5, 01, 01, -1, -1, .1, 01, 01 }, BRICK_P_90 },	    
	{ { -1, -1, -1, -1, -1, -1, 00, 00, 00, -1, .1, .5, .9, 00, -1, 01, 01, .5, 00, -1, 01, 01, .1, -1, -1 }, BRICK_P_180 },	    
	{ { 01, 01, .1, -1, -1, 01, 01, .5, 00, -1, .1, .5, .9, 00, -1, -1, 00, 00, 00, -1, -1, -1, -1, -1, -1 }, BRICK_P_270 },
	/*
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
		i_high = round(0.3 * w);
		i_wide = round(0.3 * w);
		w_high1 = round(0.4 * w);
		w_wide1 = round(0.4 * w);
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
		qInfo("Wire_25RectCompute3 Prepare w=%d,h=%d,w1=%d,h1=%d,i_w=%d,i_h=%d", wp.w_wide, wp.w_high,
			wp.w_wide1, wp.w_high1, wp.i_wide, wp.i_high);
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
#if 1
				if (stat.sum < 0) {
					qCritical("shape[%d].ratio[%d]=%f, area=%d, esum=%d", j, k, wire_25_shape[j].ratio[k], area[k], stat.sum);
				}
#endif
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
	int forbiden_dirmask;
	Point joint_pt;
	Point abs_org; //origin in base (pixel unit)
	int w;
	AntStart(int _dir, int _forbiden_dirmask, Point _abs_org, Point _joint_pt, int _w) {
		forbiden_dirmask = _forbiden_dirmask;
		dir = _dir;
		abs_org = _abs_org;
		joint_pt = _joint_pt;
		w = _w;
	}
};
/*
  Input: ig, iig
  Input: dw
  Output: wp, best wire parameter
  Output: org_best, best ant point
  scan all possible wire parameter to find best one
  Return best s
*/
unsigned detect_wire_para(Mat & ig, Mat & iig, const DetectWirePara & dw, const Point &org, int & w_best, Point & org_best)
{
	//following compute wp_best and org_best 
	unsigned best_s = 0xffffffff;
	Wire_25RectCompute3 wc;
	for (int w = dw.w_min; w <= dw.w_max; w++) {		
		WireParameter3 wp(w, dw.gray_w, dw.gray_i, 1);
		wc.prepare(wp, ig, iig);
		for (int dir = 0; dir < 8; dir++) {
			if (dw.dir_mask >> dir & 1)
			for (int d = 0; d <= w / 2; d++) {
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
	return best_s;
}

/*
Input: img
Inout: dw, 
  input dw.w_max, dw.org output dw.gray_i, dw.gray_w
*/
void detect_gray(Mat & img, Point org, DetectWirePara & dw)
{
	int ws = max(dw.w_max, 50);
	QRect dr(org.x - ws, org.y - ws, ws * 2, ws * 2);
	QRect img_rect(0, 0, img.cols, img.rows);
	dr &= img_rect;

	vector<unsigned> bins, th;
	cal_bins(img, dr, bins, 2);
	cal_threshold(bins, th, 0.5, 0.1, 0.1);
	dw.gray_i = th[0];
	dw.gray_w = th[1];
}

#define ANTWIRENUM	50
class Ant {
public:
	int ant_idx;
protected:
	vector<Wire_25RectCompute3> wc;
	vector<int> wc_init;
	vector<Point> trails;

public:
	Ant(int idx) {
		ant_idx = idx;
		wc.resize(ANTWIRENUM);
		wc_init.resize(ANTWIRENUM);
	}
	/*
	Input img
	Input lt, lt is img left-top xy(pixel unit)
	Inout dw,
		Inout dw.abs_org, abs_org is ant original xy(pixel unit)
		input dw.w_max, dw.w_min, dw.dir_mask 
		output dw.gray_i, dw.gray_w
	inout w, wire width
	input dir, 8 search dir
	input scale 
	Return last_S
	*/
	unsigned go(const Mat & ig, const Mat & iig, Point lt, const DetectWirePara & dw, AntStart & as, int scale) {
		CV_Assert(as.dir < 8);
		trails.clear();
		//1 compute gray_i and gray_w, and then do preprocess
		Point org = as.abs_org - lt;
		org.x = org.x / scale;
		org.y = org.y / scale;
		as.w = as.w / scale;
#if 0
		detect_gray(img, org, dw);
		clip_img(img, dw.gray_i, dw.gray_w, new_img);
		integral_square(img, ig, iig, Mat(), Mat(), false);
#endif
		qDebug("Ant%d begin at (x=%d,y=%d), gray_i=%d, gray_w=%d", ant_idx, as.abs_org.x, as.abs_org.y, dw.gray_i, dw.gray_w);
		//2 prepare Wire_25RectCompute
		fill(wc_init.begin(), wc_init.end(), 0);
		CV_Assert(as.w + 2 < wc.size() && as.w >= 3);
		for (int i = as.w - 1; i <= as.w + 1; i++) {
			WireParameter3 wp(i, dw.gray_w, dw.gray_i, i);
			wc[i].prepare(wp, ig, iig);
			wc_init[i] = 1;
		}

		int ep_brick;
		Point dp1, dp2;
		switch (as.dir) {
		case DIR_UP:
		case DIR_DOWN:
			ep_brick = BRICK_I_0;
			dp1 = Point(1, 0);
			dp2 = Point(-1, 0);
			break;
		case DIR_LEFT:
		case DIR_RIGHT:
			ep_brick = BRICK_I_90;
			dp1 = Point(0, 1);
			dp2 = Point(0, -1);
			break;
		case DIR_UPRIGHT:
		case DIR_DOWNLEFT:
			ep_brick = BRICK_Z_0;
			dp1 = Point(-1, -1);
			dp2 = Point(1, 1);
			break;
		default:
			ep_brick = BRICK_Z_90;
			dp1 = Point(-1, 1);
			dp2 = Point(1, -1);
			break;
		}
		int step = as.w / 10 + 1;
		Point pdir(dxy[as.dir][1], dxy[as.dir][0]);		
		
		while (1) {
			unsigned s[15];
			unsigned min_s = 0xffffffff;
			if (as.w >= ANTWIRENUM - 2) {
				trails.clear();//Already wrong
				return min_s;
			}

			CV_Assert(wc_init[as.w] && wc_init[as.w - 1] && wc_init[as.w + 1]);
			if (org.x < 2 * as.w || org.y < 2 * as.w || org.x >= ig.cols - 2 * as.w || org.y >= ig.rows - 2 * as.w) {
				if (!trails.empty())
					as.abs_org = trails.back();
				qDebug("Ant%d outbound at (x=%d,y=%d), s=0x%x", ant_idx, as.abs_org.x, as.abs_org.y, min_s);
				return 0xffffffff;
			}
			s[0] = wc[as.w].compute(org, ig, iig);
			s[1] = wc[as.w].compute(org + dp1, ig, iig);
			s[2] = wc[as.w].compute(org + dp2, ig, iig);
			s[3] = wc[as.w - 1].compute(org, ig, iig);
			s[4] = wc[as.w - 1].compute(org + dp1, ig, iig);
			s[5] = wc[as.w - 1].compute(org + dp2, ig, iig);
			s[6] = wc[as.w + 1].compute(org, ig, iig);
			s[7] = wc[as.w + 1].compute(org + dp1, ig, iig);
			s[8] = wc[as.w + 1].compute(org + dp2, ig, iig);
			for (int i = 0; i < 9; i++) {
				CV_Assert(s[i] != 0xffffffff);
				min_s = min(min_s, s[i]);
			}
			int cur_s = S_SHAPE(min_s);
			if (cur_s == BRICK_NO_WIRE || cur_s == BRICK_HOLLOW) { //Wrong case, why it happen
				if (trails.size() < as.w)
					trails.clear();//Warning
				else
					as.abs_org = trails.back();
				return min_s;
			}
			if (cur_s == ep_brick) { //still on line, continue search
				if (S_TYPE(min_s) == as.w) {
					if (min_s == s[1])
						org += dp1;
					if (min_s == s[2])
						org += dp2;
					step = as.w / 10 + 1;
				} else
				if (S_TYPE(min_s) == as.w - 1) { //adjust w
					if (as.w > 3)
						as.w--;
					if (min_s == s[4])
						org += dp1;
					if (min_s == s[5])
						org += dp2;
					if (!wc_init[as.w - 1]) {						
						WireParameter3 wp(as.w - 1, dw.gray_w, dw.gray_i, as.w - 1);
						wc[as.w - 1].prepare(wp, ig, iig);
						wc_init[as.w - 1] = 1;
					}
					step = 1;
				}
				else { //adjust w
					if (as.w + 1 < ANTWIRENUM)
						as.w++;
					if (min_s == s[7])
						org += dp1;
					if (min_s == s[8])
						org += dp2;
					if (!wc_init[as.w + 1]) {
						WireParameter3 wp(as.w + 1, dw.gray_w, dw.gray_i, as.w + 1);
						wc[as.w + 1].prepare(wp, ig, iig);
						wc_init[as.w + 1] = 1;
					}
					step = 1;
				}
				trails.push_back(lt + org * scale);
				org += pdir * step;
			} else 
			if (cur_s == BRICK_FAKE_VIA || cur_s == BRICK_II_0 || cur_s == BRICK_II_90
				|| cur_s == BRICK_II_180 || cur_s == BRICK_II_270 || cur_s == BRICK_III_0 ||
				cur_s == BRICK_III_90 || cur_s == BRICK_INVALID) {
#if 0
				if (trails.size() < as.w) {
					org += pdir * step;
					continue;
				}
#endif
				switch (cur_s) { //adjust width
				case BRICK_II_0:
					as.w = min(S_TYPE(min_s) + 2, (unsigned)ANTWIRENUM - 2);
					org += Point(-1, 0);
					break;
				case BRICK_II_90:
					as.w = min(S_TYPE(min_s) + 2, (unsigned)ANTWIRENUM - 2);
					org += Point(0, -1);
					break;
				case BRICK_II_180:
					as.w = min(S_TYPE(min_s) + 2, (unsigned)ANTWIRENUM - 2);
					org += Point(1, 0);
					break;
				case BRICK_II_270:
					as.w = min(S_TYPE(min_s) + 2, (unsigned)ANTWIRENUM - 2);
					org += Point(0, 1);
					break;
				case BRICK_III_0:
				case BRICK_III_90:
					as.w = min(S_TYPE(min_s) + 4, (unsigned)ANTWIRENUM - 2);
					break;
				}
				step = 1;
				trails.push_back(lt + org * scale);
				org += pdir * step;
				CV_Assert(as.w >= 3 && as.w + 1 <= ANTWIRENUM - 1);
				for (int i = as.w - 1; i <= as.w + 1; i++)
				if (!wc_init[i]) {
					WireParameter3 wp(i, dw.gray_w, dw.gray_i, i);
					wc[i].prepare(wp, ig, iig);
					wc_init[i] = 1;
				}
			}
			else
			{ //search turn points, TODO detect wire width when met turn point
				CV_Assert(cur_s <= BRICK_IN_USE);
				Point turn_pt;
				bool fit_dir = brick_conn.fit(as.dir, ep_brick, S_SHAPE(min_s));
				if (trails.size() < as.w && (bricks[S_SHAPE(min_s)].shape & as.forbiden_dirmask ||!fit_dir)) { //Still in last turn point
					org += pdir * step;
					continue;
				}
				
				unsigned turn_s = 0xffffffff;
				int guard = as.w * 2;
				for (int j = 1 - step; j < guard; j++) {
					min_s = 0xffffffff;
					Point o = org + pdir * j;
					s[0] = wc[as.w].compute(o, ig, iig);
					s[1] = wc[as.w].compute(o + dp1, ig, iig);
					s[2] = wc[as.w].compute(o + dp2, ig, iig);
					s[3] = wc[as.w - 1].compute(o, ig, iig);
					s[4] = wc[as.w - 1].compute(o + dp1, ig, iig);
					s[5] = wc[as.w - 1].compute(o + dp2, ig, iig);
					s[6] = wc[as.w + 1].compute(o, ig, iig);
					s[7] = wc[as.w + 1].compute(o + dp1, ig, iig);
					s[8] = wc[as.w + 1].compute(o + dp2, ig, iig);
					s[9] = wc[as.w].compute(o + dp1 + dp1, ig, iig);
					s[10] = wc[as.w].compute(o + dp2 + dp2, ig, iig);
					s[11] = wc[as.w - 1].compute(o + dp1 + dp1, ig, iig);
					s[12] = wc[as.w - 1].compute(o + dp2 + dp2, ig, iig);
					s[13] = wc[as.w + 1].compute(o + dp1 + dp1, ig, iig);
					s[14] = wc[as.w + 1].compute(o + dp2 + dp2, ig, iig);
					for (int i = 0; i < 15; i++) {
						min_s = min(min_s, s[i]);
					}					
					if (brick_conn.fit(as.dir, ep_brick, S_SHAPE(min_s)) || S_SHAPE(min_s) == BRICK_HOLLOW) {
						if (S_SHAPE(min_s) == ep_brick && j > 0) //search BRICK_I or BRICK_Z, stop
							turn_s = min_s;
						else
							turn_s = min(turn_s, min_s);
						if (turn_s == s[0] || turn_s == s[3] || turn_s == s[6])
							turn_pt = o;
						if (turn_s == s[1] || turn_s == s[4] || turn_s == s[7])
							turn_pt = o + dp1;
						if (turn_s == s[2] || turn_s == s[5] || turn_s == s[8])
							turn_pt = o + dp2;
						if (turn_s == s[9] || turn_s == s[11] || turn_s == s[13])
							turn_pt = o + dp1 + dp1;
						if (turn_s == s[10] || turn_s == s[12] || turn_s == s[14])
							turn_pt = o + dp2 + dp2;
						if (S_SHAPE(min_s) == ep_brick && j > 0)
							break;
						if (S_SHAPE(min_s) == BRICK_HOLLOW)
							break;
					}
				}
				if (turn_s == 0xffffffff) { //it is impossible to happen
					qWarning("why reach here? Fixme! continue search...");
					org += pdir * step;
					continue;
				}
				org = turn_pt;
				if (S_SHAPE(turn_s) == ep_brick)
					as.w = S_TYPE(turn_s);
				else
				switch (S_SHAPE(turn_s)) {
				case BRICK_II_0:
					as.w = min(S_TYPE(turn_s) + 2, (unsigned) ANTWIRENUM - 2);
					org += Point(-1, 0);
					break;
				case BRICK_II_90:
					as.w = min(S_TYPE(turn_s) + 2, (unsigned)ANTWIRENUM - 2);
					org += Point(0, -1);
					break;
				case BRICK_II_180:
					as.w = min(S_TYPE(turn_s) + 2, (unsigned)ANTWIRENUM - 2);
					org += Point(1, 0);
					break;
				case BRICK_II_270:
					as.w = min(S_TYPE(turn_s) + 2, (unsigned)ANTWIRENUM - 2);
					org += Point(0, 1);
					break;
				case BRICK_III_0:
				case BRICK_III_90:
					as.w = min(S_TYPE(turn_s) + 4, (unsigned)ANTWIRENUM - 2);
					break;
				case BRICK_NO_WIRE:
				case BRICK_HOLLOW:					
					if (trails.size() < as.w)
						trails.clear();//Warning
					else
						as.abs_org = trails.back();
					return turn_s;
				default:
					CV_Assert(S_SHAPE(turn_s) <= BRICK_IN_USE);
					//BRICK_L, BRICK_T, BRICK_J, BRICK_l
					as.abs_org = lt + org * scale;
					trails.push_back(as.abs_org);
					qDebug("Ant%d stop at (x=%d,y=%d), turn_s=0x%x", ant_idx, as.abs_org.x, as.abs_org.y, turn_s);
					return turn_s;
				}
				step = 1;
				trails.push_back(lt + org * scale);
				org += pdir * step;
				CV_Assert(as.w >= 3 && as.w + 1 <= ANTWIRENUM - 1);
				for (int i = as.w - 1; i <= as.w + 1; i++) 
				if (!wc_init[i]) {
					WireParameter3 wp(i, dw.gray_w, dw.gray_i, i);
					wc[i].prepare(wp, ig, iig);
					wc_init[i] = 1;
				}
			}
		}
	}

	//return 0 if path cut
	int get_path(int layer, AntStart &as, vector<MarkObj> & obj_sets, int scale) {
		static const int cnear[8][2] = {
			//y, x
			{ 0, 1 },	//up
			{ 1, 0 },	//right
			{ 0, 1 },	//down
			{ 1, 0 },	//left
			{ 1, 1 },	//right_up
			{ 1, -1 },	//right_down
			{ 1, 1 },	//left_down
			{ 1, -1 }	//left_up
		};
		static const int cfar[8][2] = {
			//y, x
			{ 1, 0 },	//up
			{ 0, 1 },	//right
			{ 1, 0 },	//down
			{ 0, 1 },	//left
			{ 1, -1 },	//right_up
			{ 1, 1 },	//right_down
			{ 1, -1 },	//left_down
			{ 1, 1 }	//left_up
		};
		int ret = 1;
		if (trails.empty())
			return 0;
		
		//Following start checking
		CV_Assert(as.abs_org == trails.back());
		CV_Assert(as.dir < 8);
		vector<Point> turn_pts;
		turn_pts.push_back(as.joint_pt);
		int org_xy = cnear[as.dir][0] * as.joint_pt.y + cnear[as.dir][1] * as.joint_pt.x;
		int new_xy, last_xy = org_xy;
		for (int i = 0; i < trails.size(); i++) {
			new_xy = cnear[as.dir][0] * trails[i].y + cnear[as.dir][1] * trails[i].x;
			if (new_xy == last_xy)
				continue;
			if (abs(new_xy - last_xy) >= 3 * scale) {
				ret = 0;
				break;
			}
			if ((new_xy - last_xy) * (new_xy - org_xy) <= 0) { //always online
				while (turn_pts.size() > 1) { 
					turn_pts.pop_back(); //pop stack
					last_xy = cnear[as.dir][0] * turn_pts.back().y + cnear[as.dir][1] * turn_pts.back().x;
					if ((new_xy - last_xy) * (new_xy - org_xy) > 0)
						break;
				}
			}
			else { //check if it is offline
				if (turn_pts.size() > 1) {
					Point & prev_last = turn_pts[turn_pts.size() - 2];
					int prev_last_xy = cnear[as.dir][0] * prev_last.y + cnear[as.dir][1] * prev_last.x;
					int new_yx = cfar[as.dir][0] * trails[i].y + cfar[as.dir][1] * trails[i].x;
					int prev_last_yx = cfar[as.dir][0] * prev_last.y + cfar[as.dir][1] * prev_last.x;
					if (abs(new_xy - prev_last_xy) * K_DISTORT1 > abs(new_yx - prev_last_yx)) {
						ret = 0;
						break;
					}						
				}
				if (turn_pts.size() > 2) {
					Point & prev_last = turn_pts[turn_pts.size() - 3];
					int prev_last_xy = cnear[as.dir][0] * prev_last.y + cnear[as.dir][1] * prev_last.x;
					int new_yx = cfar[as.dir][0] * trails[i].y + cfar[as.dir][1] * trails[i].x;
					int prev_last_yx = cfar[as.dir][0] * prev_last.y + cfar[as.dir][1] * prev_last.x;
					if (abs(new_xy - prev_last_xy) * K_DISTORT2 > abs(new_yx - prev_last_yx)) {
						ret = 0;
						break;
					}
				}
				if (turn_pts.size() > 3) {
					Point & prev_last = turn_pts[turn_pts.size() - 4];
					int prev_last_xy = cnear[as.dir][0] * prev_last.y + cnear[as.dir][1] * prev_last.x;
					int new_yx = cfar[as.dir][0] * trails[i].y + cfar[as.dir][1] * trails[i].x;
					int prev_last_yx = cfar[as.dir][0] * prev_last.y + cfar[as.dir][1] * prev_last.x;
					if (abs(new_xy - prev_last_xy) * K_DISTORT3 > abs(new_yx - prev_last_yx)) {
						ret = 0;
						break;
					}
				}
			}
			turn_pts.push_back(trails[i]);
			last_xy = new_xy;
		}
		if (ret == 1)
			turn_pts.push_back(trails.back());
		else
			qWarning("trails check failed at (x=%d,y=%d), newxy=%d", turn_pts.back().x, turn_pts.back().y, new_xy);
		MarkObj wire;
		wire.type = OBJ_LINE;
		wire.type2 = LINE_WIRE_AUTO_EXTRACT;
		wire.type3 = layer;
		wire.state = 0;
		wire.prob = 1;
		wire.p0 = QPoint(as.joint_pt.x, as.joint_pt.y);
		wire.p1 = QPoint(turn_pts.back().x, turn_pts.back().y);
		obj_sets.push_back(wire);
		return ret;
	}
};

VWExtractAnt::VWExtractAnt()
{
	
}

int VWExtractAnt::set_extract_param(int layer, int, int pi1, int pi2, int pi3, int pi4, int, int, int, float)
{
	dw0.w_min = pi1 & 0xffff;
	dw0.w_max = pi1 >> 16 & 0xffff;
	dw0.dir_mask = 0xffffffff;
	search_opt = min(pi2, SEARCH_NET);
	abs_org0.x = pi3;
	abs_org0.y = pi4;	
	layer0 = layer;
	qInfo("VWExtractAnt set param: w_min=%d, w_max=%d, dir_mask=%x, x=%d, y=%d", dw0.w_min, dw0.w_max, 
		dw0.dir_mask, abs_org0.x, abs_org0.y);
	return 0;
}

Rect torect(QRect &r)
{
	return Rect(r.left(), r.top(), r.width(), r.height());
}

QRect toqrect(Rect &r)
{
	return QRect(r.x, r.y, r.width, r.height);
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
int VWExtractAnt::extract(string file_name, QRect , vector<MarkObj> & obj_sets)
{
	Mat img = imread(file_name, 0);
	Mat detect_img, ig, iig;
	Point org_best;
	int w_best;
	detect_gray(img, abs_org0, dw0);
	qDebug("detect_gray, gray_i=%d, gray_w=%d", dw0.gray_i, dw0.gray_w);
	dw0.gray_w = (dw0.gray_w - dw0.gray_i) * 0.7 + dw0.gray_i;
	clip_img(img, dw0.gray_i, dw0.gray_w, detect_img);
	integral_square(detect_img, ig, iig, Mat(), Mat(), false);
	dw0.gray_w -= dw0.gray_i;
	dw0.gray_i = 0;
	unsigned s = detect_wire_para(ig, iig, dw0, abs_org0, w_best, org_best);
	qDebug("detect_wire_para, w_best=%d, org_best=(x=%d,y=%d), s=%x", w_best, org_best.x, org_best.y, s);
	obj_sets.clear();

	vector <AntStart> asq;
	switch (S_SHAPE(s)) {
	case BRICK_I_0:
		asq.push_back(AntStart(DIR_UP, DIR_DOWN, org_best, org_best, w_best));
		asq.push_back(AntStart(DIR_DOWN, DIR_UP, org_best, org_best, w_best));
		break;
	case BRICK_I_90:
		asq.push_back(AntStart(DIR_RIGHT, DIR_LEFT, org_best, org_best, w_best));
		asq.push_back(AntStart(DIR_LEFT, DIR_RIGHT, org_best, org_best, w_best));
		break;
	case BRICK_Z_0:
		asq.push_back(AntStart(DIR_UPRIGHT, DIR_DOWNLEFT, org_best, org_best, w_best));
		asq.push_back(AntStart(DIR_DOWNLEFT, DIR_UPRIGHT, org_best, org_best, w_best));
		break;
	case BRICK_Z_90:
		asq.push_back(AntStart(DIR_DOWNRIGHT, DIR_UPLEFT, org_best, org_best, w_best));
		asq.push_back(AntStart(DIR_UPLEFT, DIR_DOWNRIGHT, org_best, org_best, w_best));
		break;
	default:
		qWarning("click shape %d", S_SHAPE(s));
		break;
	}
	Ant ant0(0);
	while (!asq.empty()) {
		AntStart as = asq.back();
		asq.pop_back();
		unsigned s = ant0.go(ig, iig, Point(0, 0), dw0, as, 1);
		int ret = ant0.get_path(layer0, as, obj_sets, 1);
		if (s == 0xffffffff || ret == 0)
			continue;
		if (search_opt == SEARCH_SINGLE_LINE)
			continue;
		vector<int> next_dir;
		get_next_dir(S_SHAPE(s), as, next_dir);
		for (int i = 0; i < (int)next_dir.size(); i++) {
			CV_Assert(S_SHAPE(s) < BRICK_IN_USE);
			if (search_opt == SEARCH_STRAIGHT_LINE && next_dir[i] != as.dir)
				continue;
			int forbiden_dir = bricks[S_SHAPE(s)].shape & ~(1 << next_dir[i]);
			switch (next_dir[i]) {
			case DIR_RIGHT:
				asq.push_back(AntStart(next_dir[i], forbiden_dir, as.abs_org + Point(as.w * 0.5, 0), as.abs_org, as.w));
				break;
			case DIR_LEFT:
				asq.push_back(AntStart(next_dir[i], forbiden_dir, as.abs_org + Point(-as.w * 0.5, 0), as.abs_org, as.w));
				break;
			case DIR_UP:
				asq.push_back(AntStart(next_dir[i], forbiden_dir, as.abs_org + Point(0, -as.w * 0.5), as.abs_org, as.w));
				break;
			case DIR_DOWN:
				asq.push_back(AntStart(next_dir[i], forbiden_dir, as.abs_org + Point(0, as.w * 0.5), as.abs_org, as.w));
				break;
			case DIR_UPRIGHT:
				asq.push_back(AntStart(next_dir[i], forbiden_dir, as.abs_org + Point(as.w * 0.5, -as.w * 0.5), as.abs_org, as.w));
				break;
			case DIR_DOWNRIGHT:
				asq.push_back(AntStart(next_dir[i], forbiden_dir, as.abs_org + Point(as.w * 0.5, as.w * 0.5), as.abs_org, as.w));
				break;
			case DIR_DOWNLEFT:
				asq.push_back(AntStart(next_dir[i], forbiden_dir, as.abs_org + Point(-as.w * 0.5, as.w * 0.5), as.abs_org, as.w));
				break;
			case DIR_UPLEFT:
				asq.push_back(AntStart(next_dir[i], forbiden_dir, as.abs_org + Point(-as.w * 0.5, -as.w * 0.5), as.abs_org, as.w));
				break;
			}
		}	
	}
	return 0;
}

int VWExtractAnt::extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets)
{
	return 0;
}
void VWExtractAnt::get_feature(int, int, int, std::vector<float> &, std::vector<int> &)
{
	return;
}