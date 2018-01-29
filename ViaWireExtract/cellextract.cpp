#include "cellextract.h"
#include <QDir>

#define FEATURE0_SIZE 27
#define FEATURE1_SIZE 16
//TODO use auto FEATURE_SIZE instead of fixed FEATURE_SIZE
static string get_time_str()
{
	QDateTime t = QDateTime::currentDateTime();
	return "./DImg/" + t.toString("hh.mm.ss.zzz").toStdString();
}

static void deldir(const string &path)
{
	if (path.empty())
		return;

	QDir dir(QString::fromStdString(path));
	if (!dir.exists())
		return;
	dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);
	QFileInfoList fileList = dir.entryInfoList(); //get all file info
	foreach(QFileInfo file, fileList) {
		if (file.isFile())
			file.dir().remove(file.fileName());
		else
			deldir(file.absoluteFilePath().toStdString());
	}
}

static int cal_bins(Mat &img, QRect rect, vector<unsigned> &bins, int step=1)
{
	CV_Assert(img.type() == CV_8UC1 && step >= 1 && rect.y() >= 0 && rect.bottom() < img.rows && rect.x() >= 0 && rect.right() < img.cols);
    int total = 0;
    bins.assign(256, 0);
    for (int y = rect.y(); y < rect.y() + rect.height(); y+=step) {
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
TODO: change to multi threshold instead of 2
*/
static void cal_threshold(vector<unsigned> bins, vector<unsigned> & th, float init_ratio = 0.5, float cut_via_ratio = 0.01)
{
	CV_Assert(cut_via_ratio < 0.05);
	int total = 0, total1 = 0, sep, total2, old_sep = 0xffffffff;
	for (sep = 0; sep < bins.size(); sep++) 
		total += bins[sep];
	
	total1 = total * cut_via_ratio;
	for (sep = (int) bins.size() - 1; sep >= 0 && total1 >= 0; sep--) {
		total1 -= bins[sep];
		total -= bins[sep];
		bins[sep] = 0;
	}
	total1 = 0;
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
        total1 = 0,	total2 = 0;
        for (int j = 0; j < sep; j++) {
            m1_l += bins[j] * j;
            total1 += bins[j];
        }
        for (int j = sep; j < bins.size(); j++) {
            m1_r += bins[j] * j;
            total2 += bins[j];
        }
        m1_l = m1_l / total1;
        m1_r = m1_r / total2;
        sep = (m1_l + m1_r) / 2;
    }
	th.clear();
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

#define CONJ(a, b) (b % gw!=0 && b==a+1 || b % gw != gw-1 && a==b+1 || b==a+gw || a==b+gw)
	for (int i = 0; i < tn0.size(); i++) {
		int d = abs(tn0[i] - tn1[i]);
		diff += d;
		if (d > mmax) {
			mmax = d;
			im = i;
		}
	}

	for (int i = 0; i < tn0.size(); i++) {
		int d = abs(tn0[i] - tn1[i]);
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

void CellFeature::cal_feature(int * ig, int lsize, int _width, int _height, int _wunit, int _hunit, int dir)
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

    for (int i = 0; i < 8; i++)
        tn[i].resize((height / hunit) * (width / wunit));
    int d0, d1, d2, d3, d4, d5, d6, d7;
	/*
	0: 0 1 2 		4: 0 1
	   3 4 5		   2 3
					   4 5
	1: 2 1 0		5: 4 5
	   5 4 3		   2 3
					   0 1
	2: 3 4 5		6: 1 0
	   0 1 2		   3 2
					   5 4
	3: 5 4 3		7: 5 4
	   2 1 0		   3 2
					   1 0
	*/
    switch (dir) {
    case POWER_UP_L:
        d0 = 0; d1 = 1; d2 = 2; d3 = 3;
        d4 = 4; d5 = 5; d6 = 6; d7 = 7;
        break;
	case POWER_UP_R:
		d0 = 1; d1 = 0; d2 = 3; d3 = 2;
		d4 = 5; d5 = 4; d6 = 7; d7 = 6;
		break;
    case POWER_DOWN_L:
        d0 = 2; d1 = 3; d2 = 0; d3 = 1;
        d4 = 6; d5 = 7; d6 = 4; d7 = 5;
        break;
	case POWER_DOWN_R:
		d0 = 3; d1 = 2; d2 = 1; d3 = 0;
		d4 = 7; d5 = 6; d6 = 5; d7 = 4;
		break;
    case POWER_LEFT_U:
        d0 = 4; d1 = 6; d2 = 5; d3 = 7;
        d4 = 0; d5 = 2; d6 = 1; d7 = 3;
        break;
	case POWER_LEFT_D:
		d0 = 5; d1 = 7; d2 = 4; d3 = 6;
		d4 = 1; d5 = 3; d6 = 0; d7 = 2;
		break;
    case POWER_RIGHT_U:
        d0 = 6; d1 = 4; d2 = 7; d3 = 5;
        d4 = 2; d5 = 0; d6 = 3; d7 = 1;
        break;
	case POWER_RIGHT_D:
		d0 = 7; d1 = 5; d2 = 6; d3 = 4;
		d4 = 3; d5 = 1; d6 = 2; d7 = 0;
		break;
    default:
		qCritical("Cal feature Dir error!"); 
		return;
    }
    tn[d0].clear();
    tn[d1].clear();
    tn[d2].clear();
    tn[d3].clear();
    int y, yy, h = hunit;
    for (y = 0, yy = 0; y < height; yy++, y += h) {
        if (yy >= sy)
            h = hunit + 1;
        int w = wunit, x, xx;
        for (x = 0, xx = 0; x < width; xx++, x += w) {
            if (xx >= sx)
                w = wunit + 1;
            //calculate (x,y) (x+w, y+h)
            int block_sum = ig[y*lsize + x] + ig[(y + h)*lsize + x + w]
                - ig[y*lsize + x + w] - ig[(y + h)*lsize + x];
            tn[d0].push_back(block_sum);
            tn[d4][xx * (height / hunit) + yy] = block_sum;
        }
        CV_Assert(x == width);

        w = wunit;
        for (x = width, xx = 0; x > 0; xx++, x -= w) {
            if (xx >= sx)
                w = wunit + 1;
            //calculate (x-w, y) (x, y+h)
            int block_sum = ig[y*lsize + x - w] + ig[(y + h)*lsize + x]
                - ig[y*lsize + x] - ig[(y + h)*lsize + x - w];
            tn[d1].push_back(block_sum);
            tn[d5][xx * (height / hunit) + yy] = block_sum;
        }
        CV_Assert(x == 0);
    }
    CV_Assert(y == height && tn[d0].size() == (height / hunit) * (width / wunit)
        && tn[d1].size() == (height / hunit) * (width / wunit));

    h = hunit;
    for (y = height, yy = 0; y > 0; yy++, y -= h) {
        if (yy >= sy)
            h = hunit + 1;
        int w = wunit, x, xx;
        for (x = 0, xx = 0; x < width; xx++, x += w) {
            if (xx >= sx)
                w = wunit + 1;
            //calculate (x,y-h) (x+w, y)
            int block_sum = ig[(y - h)*lsize + x] + ig[y *lsize + x + w]
                - ig[(y - h)*lsize + x + w] - ig[y*lsize + x];
            tn[d2].push_back(block_sum);
            tn[d6][xx * (height / hunit) + yy] = block_sum;
        }
        CV_Assert(x == width);

        w = wunit;
        for (x = width, xx = 0; x > 0; xx++, x -= w) {
            if (xx >= sx)
                w = wunit + 1;
            //calculate (x-w, y-h) (x, y)
            int block_sum = ig[(y - h)*lsize + x - w] + ig[y *lsize + x]
                - ig[y*lsize + x - w] - ig[(y - h)*lsize + x];
            tn[d3].push_back(block_sum);
            tn[d7][xx * (height / hunit) + yy] = block_sum;
        }
        CV_Assert(x == 0);
    }
    CV_Assert(y == 0 && tn[d2].size() == (height / hunit) * (width / wunit)
        && tn[d3].size() == (height / hunit) * (width / wunit));

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

    CV_Assert(dir > 0 && dir < 256);
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

    int min_d = 100000000, max_score=0, dbit=0;
    for (int i = 0; i < 8; i++)
        if ((dir >>i) & 1) {
            int m;
            int d = cmp_vector(tno, tn[i], m);
            if (min_d > d) {
                min_d = d;
                max_score = m;
                dbit = i;
            }
        }
    dir = 1 << dbit;
    m_score = (float)max_score / (width * height);
    return (float)min_d / valid_area;
}

int CellExtract::train(string file_name, const vector<MarkObj> & obj_sets)
{
	QDir *qdir = new QDir;
	deldir("./DImg");
	bool exist = qdir->exists("./DImg");
	if (!exist) {
		bool ok = qdir->mkdir("./DImg");
		if (!ok)
			qCritical("mkdir failed");
	}
	cell.clear();
    img = imread(file_name, 0);
    CV_Assert(img.type() == CV_8UC1);
    mark.create(img.rows, img.cols, CV_8UC1);
    mark1.create(img.rows, img.cols, CV_8UC1);
    mark2.create(img.rows, img.cols, CV_8UC1);
    mark3.create(img.rows, img.cols, CV_8UC1);

    vector<unsigned> bins, th;
    for (unsigned i = 0; i < obj_sets.size(); i++) {
        if (obj_sets[i].type == OBJ_AREA && obj_sets[i].type2 == AREA_CELL) {
            if (obj_sets[i].type3 != POWER_UP_L && obj_sets[i].type3 != POWER_DOWN_L &&
				obj_sets[i].type3 != POWER_UP_R && obj_sets[i].type3 != POWER_DOWN_R &&
                obj_sets[i].type3 != POWER_LEFT_U && obj_sets[i].type3 != POWER_RIGHT_U &&
				obj_sets[i].type3 != POWER_LEFT_D && obj_sets[i].type3 != POWER_RIGHT_D) {
                qCritical("cell direction %d is wrong!", obj_sets[i].type3);
                return -1;
            }
            Mat ig;
            CellFeatures cf;
            QRect rect(obj_sets[i].p0, obj_sets[i].p1);
            cal_bins(img, QRect(0,0,img.cols, img.rows), bins, 2);
            cal_threshold(bins, th, 0.5, 0.02);
            qDebug("train, th0=%d, th1=%d, th2=%d", th[0], th[1], th[2]);
            threshold(img, mark, th[1], 1, THRESH_BINARY);
            integral(mark, ig, CV_32S);
			imwrite(get_time_str() + "_train_cell.jpg", mark * 100);
            cf.feature[0].cal_feature(ig.ptr<int>(rect.y()) + rect.x(),
                ig.cols, rect.width(), rect.height(), FEATURE0_SIZE, FEATURE0_SIZE, obj_sets[i].type3);
            cf.feature[1].cal_feature(ig.ptr<int>(rect.y()) + rect.x(),
                ig.cols, rect.width(), rect.height(), FEATURE1_SIZE, FEATURE1_SIZE, obj_sets[i].type3);
            cell.push_back(cf);
        }
    }
    return 0;
}

int CellExtract::extract(string file_name, QRect rect, vector<MarkObj> & obj_sets)
{
    img = imread(file_name, 0);
    CV_Assert(img.type() == CV_8UC1);
	int height = cell[0].feature[0].height;
	int width = cell[0].feature[0].width;
	if (rect.width() < width || rect.height() < height) {
		rect = QRect(0, 0, img.cols, img.rows);
	}
    Mat sml = Mat::zeros(rect.height(), rect.width(), CV_32FC3);
    Mat ig;
    int stepy = 2, stepx = 3; //TODO make stepx 2
	
    qInfo("Allow total similar > %f, block similar < %f", 1 - param1, param2);
    vector<unsigned> bins, th;
    bins.resize(256, 0);
	cal_bins(img, rect, bins, 2);
    cal_threshold(bins, th, 0.5, 0.02);
    qDebug("extract, th0=%d, th1=%d, th2=%d", th[0], th[1], th[2]);
    threshold(img, mark, th[1], 1, THRESH_BINARY);
    integral(mark, ig, CV_32S);
	imwrite(get_time_str() + "_extract_cell.jpg", mark * 100);
    
    for (int y = rect.y(); y < rect.y() + rect.height() - height; y += stepy) {
        Vec3f * p_sml = sml.ptr<Vec3f>(y);
        for (int x = rect.x(); x < rect.x() + rect.width() - width; x += stepx) {
            int sum = ig.at<int>(y, x) + ig.at<int>(y + height, x + width)
                - ig.at<int>(y, x + width) - ig.at<int>(y + height, x);
            if (sum > height * width / 2)
                sum = height * width - sum;
            if ((float)abs(sum - cell[0].feature[0].valid_area) / cell[0].feature[0].valid_area > param1) {
                p_sml[x][0] = 100000;
                p_sml[x][1] = 100000;
                p_sml[x][2] = 0;
                continue;
            }
            float diff, m;
            int dir = 0xf0;
            diff = cell[0].feature[0].cmp(ig.ptr<int>(y)+x, ig.cols, dir, m);
            p_sml[x][0] = diff;
            p_sml[x][1] = m;
            p_sml[x][2] = dir;
        }
    }

    for (int y = rect.y(); y < rect.y() + rect.height() - height; y += stepy) {
        Vec3f * p_sml = sml.ptr<Vec3f>(y);
        for (int x = rect.x(); x < rect.x() + rect.width() - width; x += stepx)
            if (p_sml[x][0] < param1 * 1.1f && p_sml[x][1] < param2 * 1.1f) {
                bool is_mmin = true;
                for (int dy = -stepy * 3; dy <= stepy * 3; dy += stepy)
                    for (int dx = -stepx * 3; dx <= stepx * 3; dx += stepx)
                        if (y + dy >= rect.y() && y + dy < rect.y() + rect.height() - height &&
                            x + dx >= rect.x() && x + dx < rect.x() + rect.width() - width)
                            if (sml.at<Vec3f>(y + dy, x + dx)[0] < p_sml[x][0]) {
                                is_mmin = false;
                                break;
                            }
                if (is_mmin) {
                    int my=0, mx=0;
                    float mmin = 100000;
                    for (int dy = -stepy; dy <= stepy; dy++)
                        for (int dx = -stepx; dx <= stepx; dx++)
                            if (y + dy >= rect.y() && y + dy < rect.y() + rect.height() - height &&
                                x + dx >= rect.x() && x + dx < rect.x() + rect.width() - width) {
                                float diff, m;
                                int dir = p_sml[x][2];
                                diff = cell[0].feature[1].cmp(ig.ptr<int>(y + dy) + x + dx, ig.cols, dir, m);
                                if (mmin > diff) {
                                    mmin = diff;
                                    my = y + dy;
                                    mx = x + dx;
                                }
                            }
                    if (mmin < param1) {
                        float m;
                        int dir = p_sml[x][2];
                        cell[0].feature[0].cmp(ig.ptr<int>(my) +mx, ig.cols, dir, m);
                        if (m < param2) {
                            MarkObj new_cell;
                            new_cell.type = OBJ_AREA;
                            new_cell.type2 = AREA_CELL;
                            new_cell.type3 = (dir < POWER_UP) ? POWER_UP :
                                ((dir < POWER_DOWN) ? POWER_DOWN :
                                ((dir < POWER_LEFT) ? POWER_LEFT : POWER_RIGHT));
                            new_cell.state = 0;
                            new_cell.p0 = QPoint(mx, my);
                            new_cell.p1 = QPoint(mx + width, my + height);
                            obj_sets.push_back(new_cell);
                            qDebug("Find new, x=%d, y=%d, dir=%d, d=%f, m=%f", mx, my, dir, mmin, m);
                        }
                    }
                }
            }
    }
    return 0;
}

typedef unsigned long long MapID;


int CellExtract::train(vector<ICLayerWrInterface *> & ic_layer, const std::vector<MarkObj> & obj_sets)
{
    vector<unsigned> bins, th;
    Mat mark, img;
    cell.clear();
    for (unsigned i = 0; i < obj_sets.size(); i++)
        if (obj_sets[i].type == OBJ_AREA && obj_sets[i].type2 == AREA_CELL) {
		if (obj_sets[i].type3 != POWER_UP_L && obj_sets[i].type3 != POWER_UP_R &&
			obj_sets[i].type3 != POWER_DOWN_L && obj_sets[i].type3 != POWER_DOWN_R &&
			obj_sets[i].type3 != POWER_LEFT_U && obj_sets[i].type3 != POWER_LEFT_D &&
			obj_sets[i].type3 != POWER_RIGHT_U && obj_sets[i].type3 != POWER_RIGHT_D) {
                qCritical("cell direction %d is wrong!", obj_sets[i].type3);
                return -1;
            }
			int scale = 32768 / ic_layer[0]->getBlockWidth();
			QPoint p0 = obj_sets[i].p0 / scale * scale;
			QPoint p1 = obj_sets[i].p1 / scale * scale;
            QRect rect(p0, p1);
            
			if (rect.height() / scale <= FEATURE0_SIZE * 3 && rect.width() / scale <= FEATURE0_SIZE * 3) {
				qCritical("cell size (%d, %d) is small!", rect.height() / scale, rect.width() / scale);
				return -1;
			}
            img.create(rect.height() / scale, rect.width() / scale, CV_8UC1);
            for (int yy = rect.y() >> 15; yy <= (rect.y() + rect.height()) >> 15; yy++)
                for (int xx = rect.x() >> 15; xx <= (rect.x() + rect.width()) >> 15; xx++) {
                    vector<uchar> encode_img;
                    if (ic_layer[0]->getRawImgByIdx(encode_img, xx, yy, 0, 0, false) != 0)
                        return -2;
                    Mat image = imdecode(Mat(encode_img), 0);
                    CV_Assert(ic_layer[0]->getBlockWidth() == image.cols && image.cols == image.rows);
                    cal_bins(image, QRect(0, 0, image.cols, image.rows), bins, 3);
                    cal_threshold(bins, th, 0.5, 0.02);
                    threshold(image, mark, th[1], 1, THRESH_BINARY);

                    QRect cr(xx << 15, yy << 15, 32768, 32768);
                    QRect ir = cr & rect;
                    QPoint lt = (ir.topLeft() - cr.topLeft()) / scale;
                    QPoint mlt = (ir.topLeft() - rect.topLeft()) / scale;
                    QSize size(ir.width() / scale, ir.height() / scale);
                    qInfo("CellTrain copy from img%d_%d (%d,%d) to (%d,%d), w=%d, h=%d", yy, xx,
                        lt.x(), lt.y(), mlt.x(), mlt.y(), size.width(), size.height());
                    if (size.width()==0 || size.height()==0)
                        continue;
                    mark(Rect(lt.x(), lt.y(), size.width(), size.height())).copyTo(
                        img(Rect(mlt.x(), mlt.y(), size.width(), size.height())));
                }
            Mat ig;
            CellFeatures cf;
            integral(img, ig, CV_32S);
            cf.feature[0].cal_feature(ig.ptr<int>(0), ig.cols, img.cols, img.rows, FEATURE0_SIZE, FEATURE0_SIZE, obj_sets[i].type3);
            cf.feature[1].cal_feature(ig.ptr<int>(0), ig.cols, img.cols, img.rows, FEATURE1_SIZE, FEATURE1_SIZE, obj_sets[i].type3);
            cf.cell_type = obj_sets[i].type3;
            cell.push_back(cf);
        }
    return 0;
}

bool comp_y(const SearchArea &a, const SearchArea &b)
{
    if (a.rect.y() < b.rect.y())
        return true;
    else if (a.rect.y() == b.rect.y())
            return a.rect.x() < b.rect.x();
    return false;
}

bool comp_x(const SearchArea &a, const SearchArea &b)
{
    if (a.rect.x() < b.rect.x())
        return true;
    else if (a.rect.x() == b.rect.x())
            return a.rect.y() < b.rect.y();
    return false;
}

struct SearchResult {
    int x, y, dir;
    float diff;
    SearchResult(int x_, int y_, int dir_, float diff_) {
        x = x_;
        y = y_;
        dir = dir_;
        diff = diff_;
    }
    bool operator< (const SearchResult& b) const {
        return diff < b.diff;
    }
};

int CellExtract::extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & area_, vector<MarkObj> & obj_sets)
{
	if (cell.empty()) {
		qCritical("Cell must be traininged before extract");
		return -1;
	}
    list<SearchArea> area;
    int scale = 32768 / ic_layer[0]->getBlockWidth();
    int block_x, block_y;
    ic_layer[0]->getBlockNum(block_x, block_y);
    int chigh = cell[0].feature[0].height;
    int cwide = cell[0].feature[0].width;
    int stepy = 2;
    int stepx = 3;
    obj_sets.clear();

	//1 Do some check and sort area
    qInfo("Allow total similar th=%f, block similar th=%f. overlap > %f", 1 - param1, param2, param3);
    for (int i = 0; i < area_.size(); i++) {
        qInfo("Receive search rect(%d,%d) (%d,%d), mask=%x", area_[i].rect.left(), area_[i].rect.top(),
              area_[i].rect.right(), area_[i].rect.bottom(), area_[i].option);
        if (area_[i].rect.width() < cwide * scale || area_[i].rect.height() < chigh *scale)
            qWarning("Rect(%d,%d) can't contain object (%d,%d)", area_[i].rect.width(), area_[i].rect.height(),
            cwide, chigh);
        else
            if (area_[i].rect.right() >> 15 > block_x || area_[i].rect.height() >> 15 > block_y)
                qWarning("Rect bottom right (%d,%d) exceed (%d,%d)", area_[i].rect.right(), area_[i].rect.height(),
                block_x << 15, block_y << 15);
            else
                area.push_back(area_[i]);
    }
    if (area_[0].option & (POWER_UP | POWER_DOWN))
        area.sort(comp_y);    
    else
        area.sort(comp_x);


    vector<SearchArea> nsech;
    list<SearchResult> result;
    vector<unsigned> bins, th;
    while (!area.empty()) {
        nsech.clear();
        result.clear();
        QRect cr = area.front().rect;
        QRect srect(cr.x() / scale, cr.y() / scale, cr.width() / scale - cwide + 1, cr.height() / scale - chigh + 1);
        nsech.push_back(SearchArea(srect, area.front().option));
        area.pop_front();

        //2 merge search area and select new search area
#define OCCUPY(r) (((r.right() >>15) - (r.x() >>15) + 1) * ((r.bottom() >>15) - (r.y() >> 15) + 1))
        for (int i = 0; i < 2; i++)
        for (list<SearchArea>::iterator pa = area.begin(); pa != area.end();) {
            QRect r = pa->rect;
            QRect ur = r.united(cr);
            if ((ur.bottom() >> 15) - (ur.y() >> 15) > 3)
                break;
            if (OCCUPY(r) + OCCUPY(cr) > OCCUPY(ur)) {
                cr = ur;
                QRect srect(pa->rect.x() / scale, pa->rect.y() / scale,
                    pa->rect.width() / scale - cwide + 1,
                    pa->rect.height() / scale - chigh + 1);
                nsech.push_back(SearchArea(srect, pa->option));
                pa = area.erase(pa);
            }
            else
                pa++;
        }

        //Search from left to right, prepare initial ig
        int fb_y = (cr.bottom() >> 15) - (cr.y() >> 15) + 1;
        int fb_x = min(cell[0].feature[0].width / ic_layer[0]->getBlockWidth() + 2,
            (cr.right() >> 15) - (cr.x() >> 15) +1);
        Mat img(fb_y * ic_layer[0]->getBlockWidth(), fb_x * ic_layer[0]->getBlockWidth(), CV_8UC1);
        Mat mark;
        qDebug("load image, img(%d,%d),(w=%d,h=%d)", cr.y() >> 15, cr.x() >> 15, fb_x, fb_y);
        for (int yy = cr.y() >> 15, y = 0; yy < (cr.y() >> 15) + fb_y; yy++, y += ic_layer[0]->getBlockWidth())
            for (int xx = cr.x() >> 15, x = 0; xx < (cr.x() >> 15) + fb_x; xx++, x += ic_layer[0]->getBlockWidth()) {
                vector<uchar> encode_img;
                if (ic_layer[0]->getRawImgByIdx(encode_img, xx, yy, 0, 0, false) != 0)
                    return -2;
                Mat image = imdecode(Mat(encode_img), 0);
                CV_Assert(ic_layer[0]->getBlockWidth() == image.cols && image.cols == image.rows);
                cal_bins(image, QRect(0, 0, image.cols, image.rows), bins, 3);
                cal_threshold(bins, th, 0.5, 0.02);
                threshold(image, mark, th[1], 1, THRESH_BINARY);
                CV_Assert(mark.cols == image.cols && mark.rows == image.rows);
                mark.copyTo(img(Rect(x, y, image.cols, image.rows)));
            }
        Mat ig;
        integral(img, ig, CV_32S);
        img.create(fb_y * ic_layer[0]->getBlockWidth(), ic_layer[0]->getBlockWidth(), CV_8UC1);

        for (int xx = cr.x() >> 15; xx <= cr.right() >> 15; xx++) {
            qDebug("search image, img(%d,%d),(w=%d,h=%d)", cr.y() >> 15, xx, 1, fb_y);
            QRect rr(xx*ic_layer[0]->getBlockWidth(), (cr.y() >> 15) * ic_layer[0]->getBlockWidth(),
                ic_layer[0]->getBlockWidth(), fb_y * ic_layer[0]->getBlockWidth());

            //search current ig
            for (int i = 0; i < nsech.size(); i++) {
                QRect ir = rr.intersected(nsech[i].rect); //ir is search rect
                for (int y = ir.y(); y < ir.y() + ir.height(); y+=stepy) {
                    int iy = y - rr.top();
                    for (int x = ir.x(); x < ir.x() + ir.width(); x+=stepx) {
                        int ix = x - rr.left();
                        CV_Assert(iy + chigh<ig.rows && ix + cwide < ig.cols);
                        int sum = ig.at<int>(iy, ix) + ig.at<int>(iy + chigh, ix + cwide)
                            - ig.at<int>(iy, ix + cwide) - ig.at<int>(iy + chigh, ix);
                        if (sum > chigh * cwide / 2)
                            sum = chigh * cwide - sum;
                        if ((float)abs(sum - cell[0].feature[0].valid_area) / cell[0].feature[0].valid_area > param1)
                            continue;
                        //coarse search
                        float diff, m;
                        int dir = nsech[i].option;
                        diff = cell[0].feature[0].cmp(ig.ptr<int>(iy)+ix, ig.cols, dir, m);
                        if (diff < param1 * 1.1f && m < param2 * 1.1f) {
                            //fine search
                            int my=0, mx=0;
                            float mmin = 100000;
                            for (int dy = 0; dy < stepy; dy++)
                                for (int dx = 0; dx < stepx; dx++)
                                    if (y + dy < ir.y() + ir.height() && x + dx < ir.x() + ir.width()) {
                                        diff = cell[0].feature[1].cmp(ig.ptr<int>(iy + dy) + ix + dx, ig.cols, dir, m);
                                        if (mmin > diff) {
                                            mmin = diff;
                                            my = dy;
                                            mx = dx;
                                        }
                                    }
                            if (mmin < param1) {
                                float m;
                                cell[0].feature[0].cmp(ig.ptr<int>(iy + my) + ix + mx, ig.cols, dir, m);
                                if (m < param2)
                                    result.push_back(SearchResult(x + mx, y + my, dir, mmin));
                            }
                        }
                    }
                }
            }

            //Shift ig left
            for (int y = 0; y < ig.rows; y++) {
                int * p_ig = ig.ptr<int>(y);
                for (int x = ic_layer[0]->getBlockWidth(); x < ig.cols; x++)
                    p_ig[x - ic_layer[0]->getBlockWidth()] = p_ig[x];
            }

            //fill new ig
            if (xx + fb_x <= cr.right() >> 15) {
                qDebug("load image, img(%d,%d),(w=%d,h=%d)", cr.y() >> 15, xx + fb_x, 1, fb_y);
                for (int yy = cr.y() >> 15, y = 0; yy < (cr.y() >> 15) + fb_y; yy++, y += ic_layer[0]->getBlockWidth()) {
                    vector<uchar> encode_img;
                    if (ic_layer[0]->getRawImgByIdx(encode_img, xx + fb_x, yy, 0, 0, false)!=0)
                        return -2;
                    Mat image = imdecode(Mat(encode_img), 0);
                    CV_Assert(ic_layer[0]->getBlockWidth() == image.cols && image.cols == image.rows);
                    cal_bins(image, QRect(0, 0, image.cols, image.rows), bins, 3);
                    cal_threshold(bins, th, 0.5, 0.02);
                    threshold(image, mark, th[1], 1, THRESH_BINARY);
                    mark.copyTo(img(Rect(0, y, image.cols, image.rows)));
                }
                Mat ign;
                integral(img, ign, CV_32S);
                for (int y = 0; y < ig.rows; y++) {
                    int * p_ig = ig.ptr<int>(y);
                    int * p_ign = ign.ptr<int>(y);
                    int x0 = ig.cols - ic_layer[0]->getBlockWidth();
                    for (int x = x0, xn = 1; x < ig.cols; x++, xn++)
                        p_ig[x] = p_ign[xn] + p_ig[x0 - 1];
                }
            }
        }

        //pick best solution from search result
        result.sort();
        for (list<SearchResult>::iterator pr = result.begin(); pr != result.end(); pr++) {
            list<SearchResult>::iterator pr1 = pr;
            for (pr1++; pr1 != result.end();) {
                if (abs(pr->x - pr1->x) < param3 * cwide && abs(pr->y - pr1->y) < param3 * chigh)
                    pr1 = result.erase(pr1);
                else
                    pr1++;
            }
            MarkObj new_cell;
            new_cell.type = OBJ_AREA;
            new_cell.type2 = AREA_CELL;
            new_cell.type3 = pr->dir;
            new_cell.state = 0;
            new_cell.p0 = QPoint(pr->x * scale, pr->y * scale);
            new_cell.p1 = QPoint((pr->x + cwide) * scale, (pr->y + chigh) * scale);
            new_cell.prob = pr->diff;
            obj_sets.push_back(new_cell);
            qDebug("Find new, img(%d,%d) (x=%d,y=%d) dir=%d, d=%f",
                pr->y / ic_layer[0]->getBlockWidth(), pr->x / ic_layer[0]->getBlockWidth(),
                pr->x % ic_layer[0]->getBlockWidth(), pr->y %ic_layer[0]->getBlockWidth(), pr->dir, pr->diff);
        }
    }
    return 0;
}
