#include "cellextract.h"

void cal_threshold(vector<unsigned> bins, unsigned init_num, vector<unsigned> & th)
{
	unsigned total1=0, sep, total2, old_sep=0xffffffff;
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
		total1 = 0,	total2 = 0;
		for (unsigned j = 0; j < sep; j++) {
			m1_l += bins[j] * j;
			total1 += bins[j];
		}
		for (unsigned j = sep; j < bins.size(); j++) {
			m1_r += bins[j] * j;
			total2 += bins[j];
		}
		m1_l = m1_l / total1;
		m1_r = m1_r / total2;
		sep = (m1_l + m1_r) / 2;		
	}
	th.push_back(m1_l*0.6 + m1_r*0.4);
	th.push_back(sep);
	th.push_back(m1_l*0.4 + m1_r*0.6);
}

CellFeature::CellFeature()
{
	width = -1;
	height = -1;
	wunit = -1;
	hunit = -1;
}

int CellFeature::cmp_vector(const vector<int> & tn0, const vector<int> & tn1, int & max_score)
{
	int im = 100000, ism = 100000, diff = 0;
	int gh = height / hunit, gw = width / wunit;
	int mmax, submax;

	CV_Assert(tn0.size() == tn1.size());
	mmax = -1; submax = -1;

#define CONJ(a, b) (b % gw!=0 && b==a+1 || b==a+gw)

	for (int i = 0; i < tn0.size(); i++) {
		int d = abs(tn0[i] - tn1[i]);
		diff += d;
		if (d > mmax) {
			if (CONJ(im, i)) {
				mmax = d;
				im = i;
			}
			else {
				submax = mmax;
				ism = im;
				mmax = d;
				im = i;
			}
		}
		else 
			if (d > submax && !CONJ(im, i)) {
				submax = d;
				ism = i;
			}
	}

	int conj_m = 0;
	if (im % gw != 0)
		conj_m = abs(tn0[im - 1] - tn1[im - 1]);
	if ((im + 1) % gw != 0)
		conj_m = max(conj_m, abs(tn0[im + 1] - tn1[im + 1]));
	if (im >= gw)
		conj_m = max(conj_m, abs(tn0[im - gw] - tn1[im - gw]));
	if (im / gw + 1 < gh)
		conj_m = max(conj_m, abs(tn0[im + gw] - tn1[im + gw]));
	mmax += conj_m;

	conj_m = 0;
	if (ism % gw != 0)
		conj_m = abs(tn0[ism - 1] - tn1[ism - 1]);
	if ((ism + 1) % gw != 0)
		conj_m = max(conj_m, abs(tn0[ism + 1] - tn1[ism + 1]));
	if (ism >= gw)
		conj_m = max(conj_m, abs(tn0[ism - gw] - tn1[ism - gw]));
	if (ism / gw + 1 < gh)
		conj_m = max(conj_m, abs(tn0[ism + gw] - tn1[ism + gw]));
	submax += min(conj_m, submax);

	max_score = mmax + submax * 4 / 5;
	return diff;
}

void CellFeature::cal_feature(int * ig, int lsize, int _width, int _height, int _wunit, int _hunit)
{
	int sy, sx;
	
	width = _width;
	height = _height;
	wunit = _wunit;
	hunit = _hunit;
	for (sy=0; sy<20; sy++) {
		if (height % (hunit + sy) <= height / (hunit + sy)) {
			hunit += sy;
			break;
		}
		if (height % (hunit - sy) <= height / (hunit - sy)) {
			hunit -= sy;
			break;
		}
	} 
	sy = height / hunit - height % hunit;

	for (sx=0; sx<20; sx++) {
		if (width % (wunit + sx) <= width / (wunit + sx)) {
			wunit += sx;
			break;
		}
		if (width % (wunit - sx) <= width / (wunit - sx)) {
			wunit -= sx;
			break;
		}
	}
	sx = width / wunit - width % wunit;

	for (int i = 0; i < 4; i++)
		tn[i].clear();
	int y, yy, h = hunit;
	for (y = 0, yy = 0; y < height; yy++, y += h) {
		if (yy >= sy)
			h = hunit + 1;
		int w = wunit, x, xx;
		for (x = 0, xx = 0; x < width; xx++, x += w) {
			if (xx >= sx)
				w = wunit + 1;
			//calculate (x,y) (x+w, y+h)
			tn[0].push_back(ig[y*lsize + x] + ig[(y + h)*lsize + x + w]
				- ig[y*lsize + x + w] - ig[(y + h)*lsize + x]);
		}
		CV_Assert(x == width);

		w = wunit;
		for (x = width, xx = 0; x > 0; xx++, x -= w) {
			if (xx >= sx)
				w = wunit + 1;
			//calculate (x-w, y) (x, y+h)
			tn[1].push_back(ig[y*lsize + x - w] + ig[(y + h)*lsize + x]
				- ig[y*lsize + x] - ig[(y + h)*lsize + x - w]);
		}
		CV_Assert(x == 0);
	}
	CV_Assert(y == height && tn[0].size() == (height / hunit) * (width / wunit)
		&& tn[1].size() == (height / hunit) * (width / wunit));

	h = hunit;
	for (y = height, yy = 0; y > 0; yy++, y -= h) {
		if (yy >= sy)
			h = hunit + 1;
		int w = wunit, x, xx;
		for (x = 0, xx = 0; x < width; xx++, x += w) {
			if (xx >= sx)
				w = wunit + 1;
			//calculate (x,y-h) (x+w, y)
			tn[2].push_back(ig[(y - h)*lsize + x] + ig[y *lsize + x + w]
				- ig[(y - h)*lsize + x + w] - ig[y*lsize + x]);
		}
		CV_Assert(x == width);

		w = wunit;
		for (x = width, xx = 0; x > 0; xx++, x -= w) {
			if (xx >= sx)
				w = wunit + 1;
			//calculate (x-w, y-h) (x, y)
			tn[3].push_back(ig[(y - h)*lsize + x - w] + ig[y *lsize + x]
				- ig[y*lsize + x - w] - ig[(y - h)*lsize + x]);
		}
		CV_Assert(x == 0);
	}
	CV_Assert(y == 0 && tn[2].size() == (height / hunit) * (width / wunit)
		&& tn[3].size() == (height / hunit) * (width / wunit));

	valid_area = ig[height*lsize + width] + ig[0] - ig[width] - ig[height*lsize];
	if (valid_area > height*width / 2)
		valid_area = height*width - valid_area;
	CV_Assert(valid_area > 0);
}

float CellFeature::cmp(int *ig, int lsize, int & dir, float & m_score)
{
	int sy = height / hunit - height % hunit;
	int sx = width / wunit - width % wunit;
	vector<int> tno;

	CV_Assert(sy >= 0 && sx >= 0);
	int y, yy, h = hunit;
	for (y = 0, yy = 0; y < height; yy++, y += h) {
		if (yy >= sy)
			h = hunit + 1;
		int w = wunit, x, xx;
		for (x = 0, xx = 0; x < width; xx++, x += w) {
			if (xx >= sx)
				w = wunit + 1;
			//calculate (x,y) (x+w, y+h)
			tno.push_back(ig[y*lsize + x] + ig[(y + h)*lsize + x + w]
				- ig[y*lsize + x + w] - ig[(y + h)*lsize + x]);
		}
	}

	if (dir == -1) {
		int min_d=100000000, max_score;
		for (int i = 0; i < 4; i++) {
			int m;
			int d = cmp_vector(tno, tn[i], m);
			if (min_d > d) {
				min_d = d;
				max_score = m;
				dir = i;
			}			
		}
		m_score = (float)max_score / valid_area;
		return (float)min_d / valid_area;
	}
	else {
		int m;
		float d = cmp_vector(tno, tn[dir], m);
		m_score = (float)m / valid_area;
		return d / valid_area;
	}
}

CellExtract::CellExtract()
{

}

void CellExtract::train(string file_name, const vector<MarkObj> & obj_sets, int _feature_method, int _learn_method)
{
	img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);
	mark.create(img.rows, img.cols, CV_8UC1);
	mark1.create(img.rows, img.cols, CV_8UC1);
	mark2.create(img.rows, img.cols, CV_8UC1);
	mark3.create(img.rows, img.cols, CV_8UC1);

	vector<unsigned> bins, th;
	for (unsigned i = 0; i < obj_sets.size(); i++) {		
		if (obj_sets[i].type == OBJ_AREA && obj_sets[i].type2 == AREA_CELL) {
			Mat ig;
			bins.resize(256, 0);
			QRect rect(obj_sets[i].p0, obj_sets[i].p1);
			
			for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
				unsigned char * p_img = img.ptr<unsigned char>(y);				
				for (int x = rect.x(); x < rect.x() + rect.width(); x++) 
					bins[p_img[x]]++;
			}			
			cal_threshold(bins, rect.height() * rect.width() / 2, th);	
			qDebug("train, th0=%d, th1=%d, th2=%d", th[0], th[1], th[2]);
			threshold(img, mark, th[1], 1, THRESH_BINARY);
			integral(mark, ig, CV_32S);
			feature[0].cal_feature(ig.ptr<int>(rect.y()) + rect.x(),
				ig.cols, rect.width(), rect.height(), 27, 27);
			feature[1].cal_feature(ig.ptr<int>(rect.y()) + rect.x(),
				ig.cols, rect.width(), rect.height(), 13, 13);
			feature[2].cal_feature(ig.ptr<int>(rect.y()) + rect.x(),
				ig.cols, rect.width(), rect.height(), 6, 6);
#if 1
			int sum = 0;
			for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
				unsigned char * p_img = mark.ptr<unsigned char>(y);
				for (int x = rect.x(); x < rect.x() + rect.width(); x++)
					sum += p_img[x];
			}
			if (sum > rect.height() * rect.width() / 2)
				sum = rect.height() * rect.width() - sum;
			CV_Assert(feature[0].valid_area == sum && feature[1].valid_area == sum && feature[2].valid_area == sum);
#endif
			/*vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			vector<Mat> bgr;
			Mat pic = Mat::zeros(img.rows, img.cols, CV_8UC3);
			findContours(mark2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			drawContours(pic, contours, -1, Scalar(255, 255, 255), 2);
			split(pic, bgr);
			mark1 = bgr[0];			
			threshold(img, mark2, th[1], 255, THRESH_BINARY);
			mark3 = 0;*/
		}			
	}
		
}

void CellExtract::extract(string file_name, QRect rect, vector<MarkObj> & obj_sets)
{
	img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);
	Mat sml = Mat::zeros(rect.height(), rect.width(), CV_32FC3);
	Mat ig;

	vector<unsigned> bins, th;
	bins.resize(256, 0);
	for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
		unsigned char * p_img = img.ptr<unsigned char>(y);
		for (int x = rect.x(); x < rect.x() + rect.width(); x++)
			bins[p_img[x]]++;
	}
	cal_threshold(bins, rect.height() * rect.width() / 2, th);
	qDebug("extract, th0=%d, th1=%d, th2=%d", th[0], th[1], th[2]);
	threshold(img, mark, th[1], 1, THRESH_BINARY);
	integral(mark, ig, CV_32S);

	for (int y = rect.y(); y < rect.y() + rect.height() - feature[0].height; y+=2) {
		Vec3f * p_sml = sml.ptr<Vec3f>(y);
		for (int x = rect.x(); x < rect.x() + rect.width() - feature[0].width; x += 2) {
			float diff, m;
			int dir = -1;
			diff = feature[0].cmp(ig.ptr<int>(y)+x, ig.cols, dir, m);
			p_sml[x][0] = diff;
			p_sml[x][1] = m;
			p_sml[x][2] = dir;
		}
	}

	for (int y = rect.y(); y < rect.y() + rect.height() - feature[0].height; y += 2) {
		Vec3f * p_sml = sml.ptr<Vec3f>(y);
		for (int x = rect.x(); x < rect.x() + rect.width() - feature[0].width; x += 2)
			if (p_sml[x][0] < param1 * 1.3f && p_sml[x][1] < param2 * 1.3f) {
				bool is_mmin = true;
				for (int dy = -6; dy < 7; dy += 2)
					for (int dx = -6; dx < 7; dx += 2)
						if (y + dy >= rect.y() && y + dy < rect.y() + rect.height() - feature[0].height &&
							x + dx >= rect.x() && x + dx < rect.x() + rect.width() - feature[0].width)
							if (sml.at<Vec3f>(y + dy, x + dx)[0] < p_sml[x][0]) {
								is_mmin = false;
								break;
							}
				if (is_mmin) {
					MarkObj new_cell;
					new_cell.type = OBJ_AREA;
					new_cell.type2 = AREA_CELL;
					new_cell.select_state = 0;
					new_cell.p0 = QPoint(x, y);
					new_cell.p1 = QPoint(x + feature[0].width, y + feature[0].height);
					obj_sets.push_back(new_cell);
				}
			}
	}
}