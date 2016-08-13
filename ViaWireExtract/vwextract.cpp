#include "vwextract.h"
#include <algorithm>
using namespace std;


bool intersect(const QLine &line, const QRect &rect, QLine &inter)
{
	QPoint p1(line.p1());
	QPoint p2(line.p2());
	if (max(p1.x(), p2.x()) <= rect.left() || min(p1.x(), p2.x()) >= rect.right() ||
		max(p1.y(), p2.y()) <= rect.top() || min(p1.y(), p2.y()) >= rect.bottom())
		return false;
	if (p1.x() > rect.right())
		p1.setX(rect.right());
	if (p1.x() < rect.left())
		p1.setX(rect.left());
	if (p1.y() > rect.bottom())
		p1.setY(rect.bottom());
	if (p1.y() < rect.top())
		p1.setY(rect.top());
	if (p2.x() > rect.right())
		p2.setX(rect.right());
	if (p2.x() < rect.left())
		p2.setX(rect.left());
	if (p2.y() > rect.bottom())
		p2.setY(rect.bottom());
	if (p2.y() < rect.top())
		p2.setY(rect.top());
	inter.setPoints(p1, p2);
	return true;
}

Rect toRect(QRect &r)
{
    return Rect(r.x(), r.y(), r.width(), r.height());
}

QRect toQRect(Rect &r)
{
    return QRect(r.x, r.y, r.width, r.height);
}

void fill_circle(Mat & mark, int x0, int y0, int r, unsigned char v, QRect & rect)
{
	CV_Assert(mark.type() == CV_8UC1);

	for (int y = max(rect.top(), y0 - r); y <= min(y0 + r, rect.bottom()); y++) {
		int dx = (int) sqrt((float) r*r - (y - y0) *(y - y0));
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), x0 - dx); x <= min(x0 + dx, rect.right()); x++)			
			p_mark[x] = v;
	}
}

void feature_extract_6x6(unsigned char * a, int lsize, float * feature)
{
#define HEAP_NUM 64
#define HEAP_SHIFT 2
	int s[3][3];
	int n[HEAP_NUM] = { 0 };

	for (int y = 0; y < 3; y++) {
		unsigned char * b = a + lsize * (2 * y - 2) - 2;
		unsigned char * c = b + lsize;
		for (int x = 0; x < 3; x++) {
			unsigned char b0 = b[2 * x];
			unsigned char b1 = b[2 * x + 1];
			unsigned char c0 = c[2 * x];
			unsigned char c1 = c[2 * x + 1];
			s[y][x] = b0 + b1 + c0 + c1;
			n[b0 >> HEAP_SHIFT]++;
			n[b1 >> HEAP_SHIFT]++;
			n[c0 >> HEAP_SHIFT]++;
			n[c1 >> HEAP_SHIFT]++;
		}
	}
	feature[0] = s[0][0] + s[0][1] + s[0][2] - s[1][0] - s[1][1] - s[1][2];
	feature[1] = s[1][0] + s[1][1] + s[1][2] - s[2][0] - s[2][1] - s[2][2];
	feature[2] = s[0][0] + s[1][0] + s[2][0] - s[0][1] - s[1][1] - s[2][1];
	feature[3] = s[0][1] + s[1][1] + s[2][1] - s[0][2] - s[1][2] - s[2][2];

	int num = 0, acc = 0;
	int t = 4;
	for (int i = 0; i < HEAP_NUM; i++) {
		int num1 = num + n[i];
		if (num1 >= 6) {
			acc += (6 - num)*i;
			feature[t++] = acc;
			if (t == 10)
				break;
			num = num1 - 6;
			acc = num * i;
			while (num >= 6) {
				feature[t++] = 6 * i;
				num -= 6;
				acc -= 6 * i;
			}
		} else
			if (num1 < 6) {
				acc += i * n[i];
				num = num1;
			}			
	}
	CV_Assert(t == 10);
	for (int i = 0; i < 4; i++)
		feature[i] = feature[i] / (12 * 64); //12point - 12point
	for (int i = 4; i < 10; i++)
		feature[i] = feature[i] / (6 * 128);
}

void feature_extract_7x7(unsigned char * a, int lsize, float * feature)
{
	static int xy[7][7] = {
		{ 0, 0, 1, 1, 1, 2, 2 },
		{ 0, 0, 1, 1, 1, 2, 2 },
		{ 3, 3, 4, 4, 4, 5, 5 },
		{ 3, 3, 4, 4, 4, 5, 5 },
		{ 3, 3, 4, 4, 4, 5, 5 },
		{ 6, 6, 7, 7, 7, 8, 8 },
		{ 6, 6, 7, 7, 7, 8, 8 }
	};
#define HEAP_NUM 64
#define HEAP_SHIFT 2
#define ACC_NUM 8
	int s[9] = { 0 };
	int n[HEAP_NUM] = { 0 };

	unsigned char * b = a - lsize * 3 - 3;
	for (int y = 0; y < 7; y++, b += lsize) {
		for (int x = 0; x < 7; x++) {
			s[xy[y][x]] += b[x];
			n[b[x] >> HEAP_SHIFT]++;
		}
	}
	feature[0] = (s[0] + s[3] + s[6]) * 3 - (s[1] + s[4] + s[7]) * 2;
	feature[1] = (s[1] + s[4] + s[7]) * 2 - (s[2] + s[5] + s[8]) * 3;
	feature[2] = (s[0] + s[1] + s[2]) * 3 - (s[3] + s[4] + s[5]) * 2;
	feature[3] = (s[3] + s[4] + s[5]) * 2 - (s[6] + s[7] + s[8]) * 3;

	int num = -1, acc = 0;
	int t = 4;
	for (int i = 0; i < HEAP_NUM; i++) {
		int num1 = num + n[i];
		if (num1 >= ACC_NUM) {
			acc += (ACC_NUM - num) * i;
			feature[t++] = acc;
			if (t == 10)
				break;
			num = num1 - ACC_NUM;
			acc = num * i;
			while (num >= ACC_NUM) {
				feature[t++] = ACC_NUM * i;
				num -= ACC_NUM;
				acc -= ACC_NUM * i;
			}
		}
		else {
			acc += i * n[i];
			num = num1;
		}			
	}
	CV_Assert(t == 10);
	//normalize
	for (int i = 0; i < 4; i++)
		feature[i] = feature[i] / (42 * 64); //42point - 42point
	feature[4] = feature[4] / (9 * 128);
	for (int i = 5; i < 10; i++)
		feature[i] = feature[i] / (ACC_NUM * 128);
#undef ACC_NUM
#undef HEAP_NUM
#undef HEAP_SHIFT
}


void feature_extract_9x9(unsigned char * a, int lsize, float * feature)
{
	static int xy[9][9] = {
			{ 0, 0, 0, 1, 1, 1, 2, 2, 2 },
			{ 0, 0, 0, 1, 1, 1, 2, 2, 2 },
			{ 0, 0, 0, 1, 1, 1, 2, 2, 2 },
			{ 3, 3, 3, 4, 4, 4, 5, 5, 5 },
			{ 3, 3, 3, 4, 4, 4, 5, 5, 5 },
			{ 3, 3, 3, 4, 4, 4, 5, 5, 5 },
			{ 6, 6, 6, 7, 7, 7, 8, 8, 8 },
			{ 6, 6, 6, 7, 7, 7, 8, 8, 8 },
			{ 6, 6, 6, 7, 7, 7, 8, 8, 8 }
	};
#define HEAP_NUM 64
#define HEAP_SHIFT 2
#define ACC_NUM 13
	int s[9] = { 0 };
	int n[HEAP_NUM] = { 0 };

	unsigned char * b = a - lsize * 4 - 4;
	for (int y = 0; y < 9; y++, b += lsize) {
		for (int x = 0; x < 9; x++) {
			s[xy[y][x]] += b[x];
			n[b[x] >> HEAP_SHIFT]++;
		}
	}
	feature[0] = (s[0] + s[3] + s[6]) - (s[1] + s[4] + s[7]);
	feature[1] = (s[1] + s[4] + s[7]) - (s[2] + s[5] + s[8]);
	feature[2] = (s[0] + s[1] + s[2]) - (s[3] + s[4] + s[5]);
	feature[3] = (s[3] + s[4] + s[5]) - (s[6] + s[7] + s[8]);

	int num = -3, acc = 0;
	int t = 4;
	for (int i = 0; i < HEAP_NUM; i++) {
		int num1 = num + n[i];
		if (num1 >= ACC_NUM) {
			acc += (ACC_NUM - num) * i;
			feature[t++] = acc;
			if (t == 10)
				break;
			num = num1 - ACC_NUM;
			acc = num * i;
			while (num >= ACC_NUM) {
				feature[t++] = ACC_NUM * i;
				num -= ACC_NUM;
				acc -= ACC_NUM * i;
			}
		}
		else {
			acc += i * n[i];
			num = num1;
		}
	}
	CV_Assert(t == 10);
	//normalize
	for (int i = 0; i < 4; i++)
		feature[i] = feature[i] / (27 * 64); //27point - 27point
	feature[4] = feature[4] / (16 * 128);
	for (int i = 5; i < 10; i++)
		feature[i] = feature[i] / (ACC_NUM * 128);
#undef ACC_NUM
#undef HEAP_NUM
#undef HEAP_SHIFT
}

void feature_extract_9x5(unsigned char * a, int lsize, float * feature)
{
	unsigned char * b = a - lsize * 2 - 4;
	int s[4] = { 0 };

	for (int y = 0, i = 0; y < 5; y++, b += lsize) {
		if (y == 2) {
			i++;
			continue;
		}			
		for (int x = 0; x < 9; x++)
			s[i] += b[x];
	}

	feature[0] = s[0] - s[1];

	b = a - lsize * 4 - 2;
	for (int y = 0; y < 9; y++, b += lsize) {
		s[2] += b[0] + b[1];
		s[3] += b[3] + b[4];
	}
	feature[1] = s[2] - s[3];

	feature[0] = feature[0] / (18 * 64);
	feature[1] = feature[1] / (18 * 64);
}

VWExtract::VWExtract()
{

}

struct MARK_NL {
	string xy_str;
	unsigned char nl;
} mark_nl[] = {
		{ "", M_UNKNOW },										//M_UNKNOW
		{ "010203102030", M_WNL },								//M_W
		{ "01021020", M_VNL },									//M_V
		{ "0110", M_W_INL },									//M_W_I
		{ "0110", M_V_INL },									//M_V_I
		{ "010203102030", M_INL },								//M_I
		{ "", M_W_V },											//M_W_V
		{ "", M_V_I_W },										//M_V_I_W
		{ "", M_V_I_V },										//M_V_I_V
		{ "", M_INL },											//M_INL
		{ "", M_WNL },											//M_WNL
		{ "", M_VNL },											//M_VNL
		{ "", M_W_INL },										//M_W_INL
		{ "", M_V_INL }											//M_V_INL
};

int VWExtract::fill_mark(const std::vector<MarkObj> & obj_sets)
{
	typedef vector<int> VI;
	vector<VI> offset_nl;
    unsigned mark_type_num = sizeof(mark_nl) / sizeof(mark_nl[0]);
	int learn_point_num = 0;
	l_areas.clear();
	mark.create(img.rows, img.cols, CV_8UC1);
	mark = M_UNKNOW;
	
	for (unsigned i = 0; i < mark_type_num; i++) {
		CV_Assert(mark_nl[i].xy_str.length() % 2 == 0);
		vector<int> offset;
		for (unsigned j = 0; j < mark_nl[i].xy_str.length(); j += 2) {
			int y = mark_nl[i].xy_str[j] - '0';
			int x = mark_nl[i].xy_str[j + 1] - '0';
			offset.push_back(y*(int)img.step[0] + x);
		}
		offset_nl.push_back(offset);
	}
	for (unsigned i = 0; i < obj_sets.size(); i++) {
		if (obj_sets[i].type == OBJ_AREA && obj_sets[i].type2 == AREA_LEARN)
			l_areas.push_back(LearnContainer(QRect(obj_sets[i].p0, obj_sets[i].p1)));
	}

	for (unsigned i = 0; i < obj_sets.size(); i++) {
		QLine line, line_in;
		switch (obj_sets[i].type) {
		case OBJ_WIRE:
			line.setPoints(obj_sets[i].p0, obj_sets[i].p1);
			for (unsigned j = 0; j < l_areas.size(); j++)
				if (intersect(line, l_areas[j].learn_rect, line_in)) {
				int rw = wire_wd - wire_wd / 2 - 1;
				QPoint lt(min(line_in.p1().x(), line_in.p2().x()) - wire_wd / 2, min(line_in.p1().y(), line_in.p2().y()) - wire_wd / 2);
				QPoint rb(max(line_in.p1().x(), line_in.p2().x()) + rw, max(line_in.p1().y(), line_in.p2().y()) + rw);
				l_areas[j].wires.push_back(QRect(lt, rb) & l_areas[j].learn_rect);
				}
			break;
		case OBJ_VIA:
			for (unsigned j = 0; j < l_areas.size(); j++)
				if (l_areas[j].learn_rect.contains(obj_sets[i].p1))
					l_areas[j].vias.push_back(obj_sets[i].p1);
		}
	}

	for (unsigned la = 0; la < l_areas.size(); la++) {
		QRect rect = l_areas[la].learn_rect;
		mark(toRect(rect)) = M_INL;
		mark(toRect(rect.marginsRemoved(QMargins(wire_wd / 2, wire_wd / 2, wire_wd / 2, wire_wd / 2)))) = M_I;

		for (int w = 0; w < l_areas[la].wires.size(); w++)
			mark(toRect(l_areas[la].wires[w].marginsAdded(QMargins(2, 2, 2, 2)) & rect)) = M_INL;

		for (int v = 0; v < l_areas[la].vias.size(); v++)
			fill_circle(mark, l_areas[la].vias[v].x(), l_areas[la].vias[v].y(), via_rd + 2, M_INL, rect);

		for (int w = 0; w < l_areas[la].wires.size(); w++)
			mark(toRect(l_areas[la].wires[w].marginsAdded(QMargins(1, 1, 1, 1)) & rect)) = M_W_I;

		for (int w = 0; w < l_areas[la].wires.size(); w++)
			mark(toRect(l_areas[la].wires[w].marginsRemoved(QMargins(1, 1, 1, 1)))) = M_WNL;

		for (int w = 0; w < l_areas[la].wires.size(); w++)
			mark(toRect(l_areas[la].wires[w].marginsRemoved(QMargins(2, 2, 2, 2)))) = M_W;

		for (int v = 0; v < l_areas[la].vias.size(); v++) {
            vector <QRect> rws;

			for (int w = 0; w < l_areas[la].wires.size(); w++)
                if (l_areas[la].wires[w].contains(l_areas[la].vias[v])) {
                    QRect rw = l_areas[la].wires[w].marginsAdded(QMargins(1, 1, 1, 1)) & rect;
                    rws.push_back(rw);
                }

			int x0 = l_areas[la].vias[v].x();
			int y0 = l_areas[la].vias[v].y();
			int r = via_rd + 1;

			for (int y = max(rect.top(), y0 - r); y <= min(y0 + r, rect.bottom()); y++) {
				int dx = (int)sqrt((float)r*r - (y - y0) *(y - y0));
				unsigned char * p_mark = mark.ptr<unsigned char>(y);
				for (int x = max(rect.left(), x0 - dx); x <= min(x0 + dx, rect.right()); x++)
					switch (p_mark[x]) {
					case M_I:
					case M_INL:
						p_mark[x] = M_V_I;
						break;
					case M_W:
					case M_W_I:
					case M_WNL:
                        p_mark[x] = M_V_I_W;
                        for (int i=0; i<rws.size(); i++)
                            if (rws[i].contains(x, y))
                                p_mark[x] = M_W_V;
						break;
					case M_W_V:
						p_mark[x] = M_V;
						break;
					case M_V_I:
						p_mark[x] = M_V_I_V;
						break;
				}
			}
		}

		for (int v = 0; v < l_areas[la].vias.size(); v++) {
			fill_circle(mark, l_areas[la].vias[v].x(), l_areas[la].vias[v].y(), via_rd - 1, M_VNL, rect);
			fill_circle(mark, l_areas[la].vias[v].x(), l_areas[la].vias[v].y(), via_rd - 2, M_V, rect);
		}
				
		rect = l_areas[la].learn_rect.marginsRemoved(QMargins(wire_wd / 2, wire_wd / 2, wire_wd / 2, wire_wd / 2));
		for (int y = rect.top(); y < rect.bottom(); y++) {
			unsigned char * p_mark = mark.ptr<unsigned char>(y);
			for (int x = rect.left(); x < rect.right(); x++) {
				CV_Assert(p_mark[x] < mark_type_num);
				unsigned char type = p_mark[x];
				if (type < M_INL) {
					learn_point_num++;
					for (int k = 0; k < offset_nl[type].size(); k++)
						if (p_mark[x + offset_nl[type][k]] == type)
							p_mark[x + offset_nl[type][k]] = mark_nl[type].nl;
				}
					
			}
		}
	}

	return learn_point_num;
}

void VWExtract::train(string file_name, const vector<MarkObj> & obj_sets, int _feature_method, int learn_method)
{
	img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);

	int learn_point_num = fill_mark(obj_sets);
	int feature_size;
	int learn_point = 0;
	feature_method = _feature_method;
	if (feature_method == FEA_GRADXY_HIST_7x7 || feature_method == FEA_GRADXY_HIST_6x6 || feature_method == FEA_GRADXY_HIST_9x9)
		feature_size = 10;
	Mat pt_features(learn_point_num, feature_size, CV_32FC1);
	Mat_<float> pt_types(learn_point_num, 1);
	for (unsigned la = 0; la < l_areas.size(); la++) {
		QRect rect = l_areas[la].learn_rect.marginsRemoved(QMargins(wire_wd / 2, wire_wd / 2, wire_wd / 2, wire_wd / 2));
		for (int y = rect.top(); y < rect.bottom(); y++) {
			unsigned char * p_mark = mark.ptr<unsigned char>(y);
			unsigned char * p_img = img.ptr<unsigned char>(y);			
			for (int x = rect.left(); x < rect.right(); x++) {
				if (p_mark[x] != M_UNKNOW && p_mark[x] < M_INL) {
					pt_types(learn_point, 0) = p_mark[x];
					switch (feature_method) {
					case FEA_GRADXY_HIST_6x6:
                        feature_extract_6x6(p_img + x, (int) img.step[0], pt_features.ptr<float>(learn_point));
						break;
					case FEA_GRADXY_HIST_7x7:
                        feature_extract_7x7(p_img + x, (int) img.step[0], pt_features.ptr<float>(learn_point));
						break;
					case FEA_GRADXY_HIST_9x9:
                        feature_extract_9x9(p_img + x, (int) img.step[0], pt_features.ptr<float>(learn_point));
						break;
					}
					learn_point++;
				}
			}
		}
	}

	if (learn_method == LEARN_SVM) {
		CvSVMParams params;
		params.svm_type = CvSVM::C_SVC;
		params.kernel_type = CvSVM::RBF;
        params.C = param1;
		qDebug("feature=%d, iter_num=%d, c=%f", feature_method, iter_num, (double)param1);
        params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, iter_num, 1e-6);
		svm.train(pt_features, pt_types, Mat(), Mat(), params);
	}
}

void VWExtract::extract(string file_name, QRect rect, vector<MarkObj> & obj_sets)
{
	img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);
	mark.create(img.rows, img.cols, CV_8UC1);
	Mat_<float> feature;
	if (feature_method == FEA_GRADXY_HIST_7x7 || feature_method == FEA_GRADXY_HIST_6x6 || feature_method == FEA_GRADXY_HIST_9x9)
		feature.create(1, 10);
	for (int y = rect.top(); y < rect.bottom(); y += 2) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		unsigned char * p_img = img.ptr<unsigned char>(y);
		for (int x = rect.left(); x < rect.right(); x += 2) {
			switch (feature_method) {
			case FEA_GRADXY_HIST_6x6:
                feature_extract_6x6(p_img + x, (int) img.step[0], feature.ptr<float>(0));
				break;
			case FEA_GRADXY_HIST_7x7:
                feature_extract_7x7(p_img + x, (int) img.step[0], feature.ptr<float>(0));
				break;
			case FEA_GRADXY_HIST_9x9:
                feature_extract_9x9(p_img + x, (int) img.step[0], feature.ptr<float>(0));
				break;
			}
			float response = svm.predict(feature);
			CV_Assert(response < M_INL);
			p_mark[x] = response;
		}
	}		
}
