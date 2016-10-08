#include "vwextract.h"
#include <algorithm>
#include <functional>
#include <list>
#include <queue>  
using namespace std;
#define SGN(x) (((x)>0) ? 1 : (((x)==0) ? 0 : -1))

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

void fill_circle_check(Mat & mark, int x0, int y0, int r, unsigned char v, QRect & rect, unsigned long long forbid_mark)
{
	CV_Assert(mark.type() == CV_8UC1);

	for (int y = max(rect.top(), y0 - r); y <= min(y0 + r, rect.bottom()); y++) {
		int dx = (int)sqrt((float)r*r - (y - y0) *(y - y0));
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), x0 - dx); x <= min(x0 + dx, rect.right()); x++)
			if (!((forbid_mark >> p_mark[x]) & 1))
				p_mark[x] = v;
	}
}

void fill_rect(Mat & mark, QPoint lt, QPoint rb, unsigned char v, int eu, int ed, int el, int er, QRect & rect)
{
	for (int y = max(rect.top(), lt.y() - eu); y <= min(rect.bottom(), rb.y() + ed); y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), lt.x() - el); x <= min(rect.right(), rb.x() + er); x++)
			p_mark[x] = v;
	}
}

void fill_rect_check(Mat & mark, QPoint lt, QPoint rb, unsigned char v, int eu, int ed, int el, int er, 
	QRect & rect, unsigned long long forbid_mark)
{
	for (int y = max(rect.top(), lt.y() - eu); y <= min(rect.bottom(), rb.y() + ed); y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		for (int x = max(rect.left(), lt.x() - el); x <= min(rect.right(), rb.x() + er); x++)
			if (!((forbid_mark >> p_mark[x]) & 1))
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
	unsigned d0=0, d1=0;
	int t;
	for (int y = 0, i = 0; y < 5; y++, b += lsize) {
		if (y == 2) {
			i++;
			continue;
		}			
		for (int x = 0; x < 9; x++) {
			t = b[x];
			s[i] += t;
			d0 += t * t;
		}			
	}
	t = (s[0] + s[1]) / 36; //36 point average value
	d0 -= t * t;

	b = a - lsize * 4 - 2;
	for (int y = 0; y < 9; y++, b += lsize) {
		s[2] += b[0] + b[1];
		s[3] += b[3] + b[4];
		t = b[0];	d1 += t * t;
		t = b[1];	d1 += t * t;
		t = b[3];	d1 += t * t;
		t = b[4];	d1 += t * t;
	}
	t = (s[2] + s[3]) / 36; //36 point average value
	d1 -= t * t;
	feature[0] = (s[0] - s[1]) * (s[0] - s[1]) + (s[2] - s[3]) * (s[2] - s[3]);
	feature[0] = sqrt(feature[0]);

#define HEAP_NUM 64
#define HEAP_SHIFT 2
#define ACC_NUM 15
	int n[HEAP_NUM] = { 0 };
	if (d0 > d1) {
		b = a - lsize * 4 - 2;
		for (int y = 0; y < 9; y++, b += lsize) {
			for (int x = 0; x < 5; x++) 
				n[b[x] >> HEAP_SHIFT]++;			
		}
	}
	else {
		b = a - lsize * 2 - 4;
		for (int y = 0; y < 5; y++, b += lsize) {
			for (int x = 0; x < 9; x++)
				n[b[x] >> HEAP_SHIFT]++;
		}
	}
	int num = 0, acc = 0;
	t = 1;
	for (int i = 0; i < HEAP_NUM; i++) {
		int num1 = num + n[i];
		if (num1 >= ACC_NUM) {
			acc += (ACC_NUM - num) * i;
			feature[t++] = acc;
			if (t == 4)
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
	CV_Assert(t == 4);
	//normalize
	feature[0] = feature[0] / (18 * 64); // 18 points - 18 points
	feature[1] = feature[1] / (15 * 128);
	for (int i = 2; i < 4; i++)
		feature[i] = feature[i] / (ACC_NUM * 128);
#undef ACC_NUM
#undef HEAP_NUM
#undef HEAP_SHIFT
}
#if 1
/*
Return Vec[0], Vec[1] is up down grad
		Vec[2], Vec[3] is left right grad
		Vec[4] is via grad
*/
Vec<float, 5> feature_extract_5(const unsigned char * a, int lsize, int w1, int w2, int r, vector<int> dx)
{
	int s[6] = { 0 };
	int mmin[4] = { 0xfffffff };
	int mmax[4] = { 0 };

	CV_Assert(w1 >= w2);
	const unsigned char * b;
	for (int y = 0, i = 0; y < 4; y++) {
		int w = (y == 0 || y == 3) ? w2 : w1;
		b = a + lsize * (y - 1) - w / 2;
		if (y == 2)
			i++;
		for (int x = 0; x < w; x++) {
			s[i] += b[x];
			mmin[i] = min(mmin[i], (int)b[x]);
			mmax[i] = max(mmax[i], (int)b[x]);
		}
	}

	b = a - lsize * (w1 / 2);
	for (int y = 0; y < w1; y++, b += lsize) {
		s[2] += b[0];
		mmin[2] = min(mmin[2], (int) b[0]);
		mmax[2] = max(mmax[2], (int) b[0]);
		
		s[3] += b[1];
		mmin[3] = min(mmin[3], (int) b[1]);
		mmax[3] = max(mmax[3], (int) b[1]);
	}

	b = a - lsize * (w2 / 2) - 1;
	for (int y = 0; y < w2; y++, b += lsize) {
		s[2] += b[0];
		mmin[2] = min(mmin[2], (int)b[0]);
		mmax[2] = max(mmax[2], (int)b[0]);

		s[3] += b[3];
		mmin[3] = min(mmin[3], (int)b[3]);
		mmax[3] = max(mmax[3], (int)b[3]);
	}
	for (int y = -r, i=0; y <= r; y++, i++) {
		//int dx = (int)sqrt((float)r*r - y *y);
		b = a + lsize * y;
		for (int x = -dx[i]; x <= dx[i]; x++)
			s[4] += b[x];
	}

	for (int y = -r - 1; y <= r + 1; y++) {
		b = a + lsize * y;
		for (int x = -r - 1; x <= r + 1; x++)
			s[5] += b[x];
	}

	for (int i = 0; i < 4; i++)
		s[i] = s[i] - mmin[i] - mmax[i];

	return Vec<float, 5>((float) log((s[1] + 0.001f) / (s[0] + 0.001f)) * 10,
		s[1] - s[0],
		(float) log((s[3] + 0.001f) / (s[2] + 0.001f)) * 10,
		s[3] - s[2],
		s[4] / (s[4] + 2 * (s[5] - s[4]) + 0.001f));
}

#else
/*
Return Vec[0], Vec[1] is up down grad
Vec[2], Vec[3] is left right grad
Vec[4] is via grad
*/
Vec<float, 5> feature_extract_5(unsigned char * a, int lsize, int w1, int w2, int r, vector<int> dx)
{
	int s[10] = { 0 };
	int mmin[8] = { 0xfffffff };
	int mmax[8] = { 0 };

	unsigned char * b, * c;
	for (int y = 0, i = 0; y < 4; y++) {
		int w = (y == 0 || y == 3) ? w2 : w1;
		b = a + lsize * (y - 1) - w;
		if (y == 2)
			i++;
		for (int x = 1; x <= w; x++) {
			s[i] += b[x];
			mmin[i] = min(mmin[i], (int)b[x]);
			mmax[i] = max(mmax[i], (int)b[x]);
		}
		for (int x = w; x < w * 2; x++) {
			s[i + 2] += b[x];
			mmin[i + 2] = min(mmin[i + 2], (int)b[x]);
			mmax[i + 2] = max(mmax[i + 2], (int)b[x]);
		}
	}

	b = a;
	c = a;
	for (int y = 0; y < w1; y++, b -= lsize, c += lsize) {
		s[4] += b[0];
		mmin[4] = min(mmin[4], (int)b[0]);
		mmax[4] = max(mmax[4], (int)b[0]);

		s[5] += b[1];
		mmin[5] = min(mmin[5], (int)b[1]);
		mmax[5] = max(mmax[5], (int)b[1]);

		s[6] += c[0];
		mmin[6] = min(mmin[6], (int)c[0]);
		mmax[6] = max(mmax[6], (int)c[0]);

		s[7] += c[1];
		mmin[7] = min(mmin[7], (int)c[1]);
		mmax[7] = max(mmax[7], (int)c[1]);
	}

	b = a - 1;
	c = a - 1;
	for (int y = 0; y < w2; y++, b -= lsize, c += lsize) {
		s[4] += b[0];
		mmin[4] = min(mmin[4], (int)b[0]);
		mmax[4] = max(mmax[4], (int)b[0]);

		s[5] += b[3];
		mmin[5] = min(mmin[5], (int)b[3]);
		mmax[5] = max(mmax[5], (int)b[3]);

		s[6] += c[0];
		mmin[6] = min(mmin[6], (int)c[0]);
		mmax[6] = max(mmax[6], (int)c[0]);

		s[7] += c[3];
		mmin[7] = min(mmin[7], (int)c[3]);
		mmax[7] = max(mmax[7], (int)c[3]);
	}
	for (int y = -r, i = 0; y <= r; y++, i++) {
		//int dx = (int)sqrt((float)r*r - y *y);
		b = a + lsize * y;
		for (int x = -dx[i]; x <= dx[i]; x++)
			s[8] += b[x];
	}

	for (int y = -r - 1; y <= r + 1; y++) {
		b = a + lsize * y;
		for (int x = -r - 1; x <= r + 1; x++)
			s[9] += b[x];
	}

	for (int i = 0; i < 8; i++)
		s[i] = s[i] - mmin[i] - mmax[i];

	int choose_ud = 0;
	if (abs(s[3] - s[2]) > abs(s[1] - s[0]))
		choose_ud = 2;
	int choose_lr = 4;
	if (abs(s[7] - s[6]) > abs(s[5] - s[4]))
		choose_lr = 6;
	return Vec<float, 5>((float)log((s[choose_ud + 1] + 0.001f) / (s[choose_ud] + 0.001f)) * 10,
		s[choose_ud + 1] - s[choose_ud],
		(float)log((s[choose_lr + 1] + 0.001f) / (s[choose_lr] + 0.001f)) * 10,
		s[choose_lr + 1] - s[choose_lr],
		s[8] / (s[8] + 2 * (s[9] - s[8]) + 0.001f));
}
#endif
/*
Four suppose condition
1 Insulator area's grad is lower than g0
2 Insulator area's width is larger than l0
3 Edge's grad is larger than g1
4 Edge's sum grad is larger than g2
5 Metal area's width is equal to l1
No constraint for metal's grad
In: ew, grad
In: g0, Insulator grad weight threshold
In: g1, Edge grad weight threshold
In: g2, Edge grad weight sum threshold
In: l0, Insulator area's width constraint
In: l1, Metal width constraint
Inout: mark, metal is 1, insulator is -1, via is 2
*/
void find_obvious_edge(const vector<float> & ew, float g0, float g1, float g2, int l0, int l1, vector<float> & mark)
{
	CV_Assert(ew.size() == mark.size() && g1 >= g0 && g2 >= g1 * 2);
	vector<float> weight(ew.size());
	for (int i = 0; i < l0; i++)
		weight[i] = 0;
	for (int i =(int) ew.size() - l1 - l0; i < ew.size(); i++)
		weight[i] = 0;
	for (int i = l0 - 1; i < ew.size() - l1 -l0; i++) {
		bool judge;
		if (ew[i] < g1 || ew[i + l1] > -g1 || mark[i] >= 2 || mark[i + l1] >= 2) {
			weight[i] = 0;
			continue; //Condition 3 and 5
		}
			
		judge = true;
		for (int j = 2 - l0; j <= -2; j++)
			if (fabs(ew[i + j]) > g0)
				judge = false; //condition 1 and 2
		if (judge)
			for (int j = l1 + 2; j <= l1 + l0 - 2; j++)
				if (fabs(ew[i + j]) > g0)
					judge = false; //condition 1 and 2
		weight[i] = judge ? ew[i] - ew[i + l1] : 0;
	}

	for (int i = l0 - 1; i < ew.size() - l1 - l0; i++) {
		if (weight[i] > weight[i - 1] && weight[i] > weight[i + 1] && weight[i] > g2) {//condition 4
			bool judge = true;
			for (int j = 2 - l0; j <= -1; j++)
				if (mark[i + j] == 1 || mark[i + j] >= 2)
					judge = false;
			for (int j = l1 + 2; j <= l1 + l0 - 1; j++)
				if (mark[i + j] == 1 || mark[i + j] >= 2)
					judge = false;
			for (int j = 2; j <= l1 - 1; j++)
				if (mark[i + j] == -1 || mark[i + j] >= 2)
					judge = false;
			if (judge) {
				for (int j = 2 - l0; j <= -1; j++)
					mark[i + j] = -1;
				for (int j = l1 + 2; j <= l1 + l0 - 1; j++)
					mark[i + j] = -1;
				for (int j = 2; j <= l1 - 1; j++)
					mark[i + j] = 1;
				i += l1 + l0 - 2;
			}
		}
	}
}

/*
use 3 lines to transfer threshold to resistence
tp0<1, tp2>1
*/
float adjust(float ew, float g0, float g1, float g2, float tp0, float tp2)
{
	if (fabs(ew) < g0)
		return tp0 * ew / g0;
	if (fabs(ew) < g1)
		return (tp0 + (fabs(ew) - g0) * (1 - tp0) / (g1 - g0)) * SGN(ew);

	return (1 + (fabs(ew) - g1) * (tp2 - 1) / (g2 - g1)) * SGN(ew);
}

/*
in: ew, channel 0 is for right | grad, channel is for bottom _ grad
inout: mark, input idicate metal is 1, insulator is -1, via is 2, other is 0
			 output indicate probability for metal and insulator, more near to 1 means more like metal, more near to -1 means more like insu
in: iter, max iter number
*/
void iter_compute_pblty(const Mat & ew_, Mat & mark, float g0, float g1, float g2, float tp0, float tp2, int iter=10)
{
	CV_Assert(mark.type() == CV_32FC1 && ew_.type() == CV_32FC2);
	CV_Assert(mark.rows == ew_.rows && mark.cols == ew_.cols);

	Mat ew(mark.rows, mark.cols, CV_32FC2);
	for (int y = 0; y < mark.rows; y++) {
		const float * p_ew_ = ew_.ptr<float>(y);
		float * p_ew = ew.ptr<float>(y);
		for (int x = 0; x < mark.cols * 2; x++)
			p_ew[x] = adjust(p_ew_[x], g0, g1, g2, tp0, tp2);
	}

	Mat gw(mark.rows, mark.cols, CV_32FC4);
	vector<unsigned int> tobe_decide;
	tobe_decide.reserve(655360);

	for (int y = 1; y < mark.rows - 1; y++) {
		const float * p_ew = ew.ptr<float>(y);
		const float * p_ew_1 = ew.ptr<float>(y - 1);
		const float * p_mark = mark.ptr<float>(y);
		const float * p_mark_1 = mark.ptr<float>(y - 1);
		const float * p_mark_a1 = mark.ptr<float>(y + 1);
		float * p_gw = gw.ptr<float>(y);
		for (int x = 1; x < mark.cols - 1; x++) 
			if (fabs(p_mark[x]) < 1) {
				float yr = (p_mark[x + 1] >= 2) ? 0 : ((p_ew[2 * x] == 0) ? 1000000 : 1 / p_ew[2 * x]);
				float yd = (p_mark_a1[x] >= 2) ? 0 : ((p_ew[2 * x + 1] == 0) ? 1000000 : 1 / p_ew[2 * x + 1]);
				float yl = (p_mark[x - 1] >= 2) ? 0 : ((p_ew[2 * x - 2] == 0) ? 1000000 : 1 / p_ew[2 * x - 2]);
				float yu = (p_mark_1[x] >= 2) ? 0 : ((p_ew_1[2 * x + 1] == 0) ? 1000000 :1 / p_ew_1[2 * x + 1]);
				float sum = fabs(yr) + fabs(yd) + fabs(yl) + fabs(yu) + 0.001f;
				p_gw[4 * x] = yu / sum;
				p_gw[4 * x + 1] = yr / sum;
				p_gw[4 * x + 2] = yd / sum;
				p_gw[4 * x + 3] = yl / sum;
				if ((p_mark[x + 1] != 0 && p_mark[x + 1] != 2) ||
					(p_mark[x - 1] != 0 && p_mark[x - 1] != 2) ||
					(p_mark_1[x] != 0 && p_mark_1[x] != 2) ||
					(p_mark_a1[x] != 0 && p_mark_a1[x] != 2))
					tobe_decide.push_back((y << 16) | x);
			}
	}
	
	for (; iter > 0; iter--) {		
		int prob_stat[256] = { 0 };
		int y0 = -1;
		float * p_mark;
		float * p_mark_1;
		float * p_mark_a1;
		float * p_gw;

		for (vector<unsigned int>::iterator it = tobe_decide.begin(); it != tobe_decide.end(); it++) {
			int y = (*it >> 16) & 0xffff;
			int x = *it & 0xffff;
			if (y != y0) {
				y0 = y;
				p_mark = mark.ptr<float>(y);
				p_mark_1 = mark.ptr<float>(y - 1);
				p_mark_a1 = mark.ptr<float>(y + 1);
				p_gw = gw.ptr<float>(y);
			}	
			
			p_mark[x] = p_mark[x + 1] * fabs(p_gw[4 * x + 1]) +
				p_mark_a1[x] * fabs(p_gw[4 * x + 2]) +
				p_mark[x - 1] * fabs(p_gw[4 * x + 3]) +
				p_mark_1[x] * fabs(p_gw[4 * x]);			
		}

		for (vector<unsigned int>::reverse_iterator it = tobe_decide.rbegin(); it != tobe_decide.rend(); it++) {
			int y = (*it >> 16) & 0xffff;
			int x = *it & 0xffff;
			if (y != y0) {
				y0 = y;
				p_mark = mark.ptr<float>(y);
				p_mark_1 = mark.ptr<float>(y - 1);
				p_mark_a1 = mark.ptr<float>(y + 1);
				p_gw = gw.ptr<float>(y);
			}

			p_mark[x] = p_mark[x + 1] * fabs(p_gw[4 * x + 1]) +
				p_mark_a1[x] * fabs(p_gw[4 * x + 2]) +
				p_mark[x - 1] * fabs(p_gw[4 * x + 3]) +
				p_mark_1[x] * fabs(p_gw[4 * x]);
			prob_stat[(int)(255.0f * fabs(p_mark[x]))]++;
		}

		int flip_num = 0;
		float th;
		for (int fi= 255; fi > 0; fi--) {
			flip_num += prob_stat[fi];
			if (flip_num >= tobe_decide.size() / 3) {
				th = (float)fi / 255.0f;
				break;
			}				
		}

		tobe_decide.clear();
		for (int y = 1; y < mark.rows - 1; y++) {
			float * p_mark = mark.ptr<float>(y);
			const float * p_mark_1 = mark.ptr<float>(y - 1);
			const float * p_mark_a1 = mark.ptr<float>(y + 1);
			for (int x = 1; x < mark.cols - 1; x++)
				if (fabs(p_mark[x]) < 1) {
					if (fabs(p_mark[x]) > th)
						p_mark[x] = (p_mark[x] > 0) ? 1 : -1;
					else
						if ((fabs(p_mark[x + 1]) > th && p_mark[x + 1] != 2) ||
							(fabs(p_mark[x - 1]) > th && p_mark[x - 1] != 2) ||
							(fabs(p_mark_1[x]) > th && p_mark_1[x] != 2) ||
							(fabs(p_mark_a1[x]) > th && p_mark_a1[x] != 2))
							tobe_decide.push_back((y << 16) | x);
				}					
		}
		
		qDebug("%d:flip=%d, th=%f", iter, flip_num, th);
	}
}

/*
use 3 lines to transfer threshold to resistence
tp0<1, tp2>1
*/
float adjust2(double ew, double a, float g1, float g2, float tp2)
{
	if (fabs(ew) < g1)
		return a * ew * ew;
	else
		return (1 + (fabs(ew) - g1) * (tp2 - 1) / (g2 - g1));
}
/*
in: ew, channel 0 is for right | grad, channel is for bottom _ grad
inout: mark, input idicate metal is 1, insulator is -1, via is 2, other is 0
output indicate probability for metal and insulator, more near to 1 means more like metal, more near to -1 means more like insu
*/
void spath_compute_pblty(const Mat & ew_, Mat & mark, const float g0[], const float g1[], const float g2[], float tp0, float tp2)
{
	CV_Assert(mark.type() == CV_32FC1 && ew_.type() == CV_32FC2);
	CV_Assert(mark.rows == ew_.rows && mark.cols == ew_.cols);

	double a[2];
	a[0] = 1 / ((double)g1[0] * g1[0]);
	a[1] = 1 / ((double)g1[1] * g1[1]);

	Mat ew(mark.rows, mark.cols, CV_32FC2);
	for (int y = 0; y < mark.rows; y++) {
		float * p_ew = ew.ptr<float>(y);
		const float * p_ew_ = ew_.ptr<float>(y);
		for (int x = 0; x < mark.cols * 2; x += 2) {
			p_ew[x] = adjust2((double)p_ew_[x], a[0], g1[0], g2[0], tp2); //adjust left right grad
			p_ew[x + 1] = adjust2((double)p_ew_[x + 1], a[1], g1[1], g2[1], tp2); //adjust up down grad
		}			
	}

	float pntfat = (a[0] * g0[0] * g0[0] + a[1] * g0[1] * g0[1]) * tp0 /2;
	if (pntfat < 0.000005f)
		pntfat = 0.000005f;
	qDebug("penelty factor=%f", pntfat);

	struct PixelPblty {		
		float ewsum;
		unsigned short x, y;
		bool operator < (const PixelPblty &a) const {
			return ewsum > a.ewsum;
		}
		PixelPblty() { ewsum = 0; }
		PixelPblty(float e, unsigned short xx, unsigned short yy) { 
			ewsum = e;
			x = xx;
			y = yy;
		}
	};
	priority_queue<PixelPblty> queue[2];
	Mat sp(mark.rows, mark.cols, CV_32FC2);

	for (int y = 1; y < mark.rows - 1; y++) {
		const float * p_ew = ew.ptr<float>(y);
		const float * p_ew_1 = ew.ptr<float>(y - 1);
		const float * p_mark = mark.ptr<float>(y);
		const float * p_mark_1 = mark.ptr<float>(y - 1);
		const float * p_mark_a1 = mark.ptr<float>(y + 1);
		float * p_sp = sp.ptr<float>(y);
		for (int x = 1; x < mark.cols - 1; x++)
			if (fabs(p_mark[x]) == 1) {
				if (p_mark[x] == 1) {
					p_sp[2 * x + 1] = 0;
					p_sp[2 * x] = 1000000;
				}									
				else {
					p_sp[2 * x + 1] = 1000000;
					p_sp[2 * x] = 0;
				}
			}				
			else {
				p_sp[2 * x + 1] = 1000000;
				p_sp[2 * x] = 1000000;
				if (p_mark[x + 1] == 1)
					p_sp[2 * x + 1] = min(p_sp[2 * x + 1], p_ew[2 * x]);
				if (p_mark[x + 1] == -1)
					p_sp[2 * x] = min(p_sp[2 * x], p_ew[2 * x]);
				if (p_mark_a1[x] == 1)
					p_sp[2 * x + 1] = min(p_sp[2 * x + 1], p_ew[2 * x + 1]);
				if (p_mark_a1[x] == -1)
					p_sp[2 * x] = min(p_sp[2 * x], p_ew[2 * x + 1]);
				if (p_mark[x - 1] == 1)
					p_sp[2 * x + 1] = min(p_sp[2 * x + 1], p_ew[2 * x - 2]);
				if (p_mark[x - 1] == -1)
					p_sp[2 * x] = min(p_sp[2 * x], p_ew[2 * x - 2]);
				if (p_mark_1[x] == 1)
					p_sp[2 * x + 1] = min(p_sp[2 * x + 1], p_ew_1[2 * x + 1]);
				if (p_mark_1[x] == -1)
					p_sp[2 * x] = min(p_sp[2 * x], p_ew_1[2 * x + 1]);
				if (p_sp[2 * x] < 900000)
					queue[0].push(PixelPblty(p_sp[2 * x], x, y));
				if (p_sp[2 * x + 1] < 900000)
					queue[1].push(PixelPblty(p_sp[2 * x + 1], x, y));
			}
	}

	for (int idx = 0; idx < 2; idx++) {
		while (!queue[idx].empty()) {
			PixelPblty pixel = queue[idx].top();
			queue[idx].pop();
			int y = pixel.y;
			int x = pixel.x;
			float path_val = sp.at<float>(y, x * 2 + idx);
			if (pixel.ewsum - path_val < 0.000001f) { // valid top pixel
				float new_val = path_val + ew.at<float>(y, x * 2) + pntfat;
				if (sp.at<float>(y, x * 2 + idx + 2) > new_val) {
					sp.at<float>(y, x * 2 + idx + 2) = new_val;
					queue[idx].push(PixelPblty(new_val, x + 1, y));
				}
				new_val = path_val + ew.at<float>(y, x * 2 + 1) + pntfat;
				if (sp.at<float>(y + 1, x * 2 + idx) > new_val) {
					sp.at<float>(y + 1, x * 2 + idx) = new_val;
					queue[idx].push(PixelPblty(new_val, x, y + 1));
				}
				new_val = path_val + ew.at<float>(y, x * 2 - 2) + pntfat;
				if (sp.at<float>(y, x * 2 + idx - 2) > new_val) {
					sp.at<float>(y, x * 2 + idx - 2) = new_val;
					queue[idx].push(PixelPblty(new_val, x - 1, y));
				}
				new_val = path_val + ew.at<float>(y - 1, x * 2 + 1) + pntfat;
				if (sp.at<float>(y - 1, x * 2 + idx) > new_val) {
					sp.at<float>(y - 1, x * 2 + idx) = new_val;
					queue[idx].push(PixelPblty(new_val, x, y - 1));
				}
			}
		}
	}

	for (int y = 0; y < mark.rows; y++) {
		float * p_mark = mark.ptr<float>(y);
		float * p_sp = sp.ptr<float>(y);
		for (int x = 0; x < mark.cols; x++) {
			float alpha = (p_sp[2 * x] + 0.000001f) / (p_sp[2 * x] + p_sp[2 * x + 1] + 0.000001f); // avoid divide by 0
			p_mark[x] = 2 * alpha - 1;
		}
	}
}

/*
inout: ew, input channel 0 is for right | grad, channel is for bottom _ grad
	       output adjust result
in: g0, g1, g2, l0, l1 is same meaning as find_obvious_edge
in: tp0, tp2 is for adjust, seperate line nihe
in: wire_up_down, 0 scan row first, 1 scan column first
out: mark more near to 1 means more like metal, more near to -1 means more like insu
*/
void estimate_metal_pblty(Mat & ew, const float g0[], const float g1[], const float g2[], 
	float tp0, float tp2, int l0, int l1, int wire_up_down, Mat & mark, int iter_num)
{
	CV_Assert(wire_up_down < 2 && tp0<1 && tp2>1);
	CV_Assert(mark.rows == ew.rows && mark.cols == ew.cols && ew.type() == CV_32FC2 && mark.type() == CV_32FC1);

	for (int scan = 0; scan < 2; scan++) {
		if (scan == wire_up_down) {
			//scan row, from left to right, detect up-down wire (shu xian)
			vector<float> row_weight(ew.cols);
			vector<float> row_mark(ew.cols);
			for (int y = 0; y < mark.rows; y++) {
				const float * p_ew = ew.ptr<float>(y);
				float * p_mark = mark.ptr<float>(y);
				for (int x = 0; x < mark.cols; x++) {
					row_weight[x] = p_ew[2 * x];
					row_mark[x] = p_mark[x];
				}
				find_obvious_edge(row_weight, g0[scan], g1[scan], g2[scan], l0, l1, row_mark);
				for (int x = 0; x < mark.cols; x++)
					p_mark[x] = row_mark[x];
			}
		}
		else {
			//scan column, from up to down, detect left-right wire (heng xian)
			vector<float> col_weight(ew.rows);
			vector<float> col_mark(ew.rows);
			for (int x = 0; x < mark.cols; x++) {
				for (int y = 0; y < mark.rows; y++) {
					col_weight[y] = ew.at<float>(y, 2 * x + 1);
					col_mark[y] = mark.at<float>(y, x);
				}
				find_obvious_edge(col_weight, g0[scan], g1[scan], g2[scan], l0, l1, col_mark);
				for (int y = 0; y < mark.rows; y++)
					mark.at<float>(y, x) = col_mark[y];
			}
		}
	}

	//iter_compute_pblty(ew, mark, g0, g1, g2, tp0, tp2, iter_num);
	if (tp0>0)
		spath_compute_pblty(ew, mark, g0, g1, g2, tp0, tp2);
}

/*
inout: img input: image raw data
		   output: image data which remove via
input: mark, via is 2 others is 0
*/
void remove_via_line(const vector<unsigned char> & mark, vector<unsigned char> & img, int w)
{
	CV_Assert(img.size() == mark.size());

	for (int i = 0; i < img.size(); i++) {		
		if (mark[i] == 2) {	
			CV_Assert(i > w);
			int len = 0;
			bool tail_good;
			do {
				CV_Assert(mark[i + len] == 2);
				for (; i + len < img.size() && mark[i + len] == 2; len++);
				CV_Assert(mark[i + len] != 2);
				tail_good = true;				
				for (int j = len + i + 1; j < len + i + w; j++)
					if (mark[j] == 2) {
						tail_good = false;
						len = j - i;
						break;
					}
			} while (!tail_good);

			CV_Assert(mark[i + len] != 2);
			int a = 0, b = 0;
			for (int j = 1; j <= w; j++) {
				a += img[i - j];
				b += img[i + len + j - 1];
			}				
			a = a / w;
			b = b / w;
			for (int j = 0; j < len / 2; j++)
				img[i + j] = a;
			for (int j = len / 2 + 1; j < len; j++)
				img[i + j] = b;
			img[i + len / 2] = (a + b) / 2;
			i += len;
		}
	}
}

/*
input mark, via is 2 others is 0
inout img, input: image raw data
		   output: image data which remove via
in wire_up_down, 0 is shuxian rule, 1 is hengxian rule.
*/
void remove_via(const Mat & mark, Mat & img, int wire_up_down, int w)
{
	CV_Assert(wire_up_down < 2 && mark.rows == img.rows && mark.cols == img.cols);
	CV_Assert(mark.type() == CV_8UC1 && img.type() == CV_8UC1);

	if (wire_up_down == 0) {
		vector<unsigned char> img_line(img.rows);
		vector<unsigned char> mark_line(img.rows);
		for (int x = 0; x < mark.cols; x++) {
			for (int y = 0; y < mark.rows; y++) {
				img_line[y] = img.at<unsigned char>(y, x);
				mark_line[y] = (unsigned char) mark.at<unsigned char>(y, x);
			}
			remove_via_line(mark_line, img_line, w);
			for (int y = 0; y < mark.rows; y++)
				img.at<unsigned char>(y, x) = img_line[y];
		}
	}
	else {
		vector<unsigned char> img_line(img.cols);
		vector<unsigned char> mark_line(img.cols);
		for (int y = 0; y < mark.rows; y++) {
			unsigned char * p_img = img.ptr<unsigned char>(y);
			const unsigned char * p_mark = mark.ptr<unsigned char>(y);
			for (int x = 0; x < mark.cols; x++) {
				img_line[x] = p_img[x];
				mark_line[x] = p_mark[x];
			}
			remove_via_line(mark_line, img_line, w);
			for (int x = 0; x < mark.cols; x++)
				p_img[x] = img_line[x];
		}
	}
}

void feature_extract_via(const unsigned char * a, int lsize, int r, float * feature)
{
	for (int i = 0; i < 4; i++) {
		int x0, y0, x1, y1, s0 = 0, s1 = 0;
		switch (i) {
		case 0:
			x0 = -r;
			x1 = -r / 3;
			y0 = -r;
			y1 = -r / 3;
			break;
		case 1:
			x0 = r / 3;
			x1 = r;
			y0 = -r; 
			y1 = -r / 3;
			break;
		case 2:
			x0 = r / 3;
			x1 = r;
			y0 = r / 3;
			y1 = r;
			break;
		case 3:
			x0 = -r;
			x1 = -r / 3;
			y0 = r / 3;
			y1 = r;
			break;
		}
		for (int y = y0; y <= y1; y++) {
			const unsigned char * b = a + lsize * y;
			for (int x = x0; x <= x1; x++) {
				s0 += b[x];
				if ((i == 0) && (y + x > y1 + x0) || (i == 2) && (y + x < y1 + x0) ||
					(i == 1) && (x - y < x0 - y0) || (i == 3) && (x - y > x0 - y0))
					s1 += b[x];
			}
		}
		feature[i] = (float)s1 / s0;
	}
	
}

/*
in: mark_f, more near to 1 means more like metal, more near to -1 means more like insu
in: via via coordinate
in: av via's weight for compute grid_line
in: w wire width
in: grid, grid width, grid don't need to be big than wire width
in: row 0 is wire up down, 1 is wire left right
out: grid_line
*/
void find_grid_line(const Mat & mark_f, const vector<QPoint> & via, float av, int w, int grid, int row, vector<int> & grid_line)
{
#define WEIGHT_FB 3
	CV_Assert(mark_f.type() == CV_32FC1 && grid > WEIGHT_FB + 1 && mark_f.rows > grid * 5 && mark_f.cols > grid * 5);
	vector<double> weight;

	if (row == 0) {
		weight.resize(mark_f.cols - w + 1, 0);
		vector<float> lweight(mark_f.cols);
		for (int y = 0; y < mark_f.rows; y++) {
			const float * p_mark_f = mark_f.ptr<float>(y);
			lweight[0] = p_mark_f[0];
			for (int x = 1; x < w; x++)
				lweight[x] = lweight[x - 1] + p_mark_f[x];
			for (int x = w; x < mark_f.cols; x++) {
				lweight[x] = lweight[x - 1] + p_mark_f[x] - p_mark_f[x - w];
				weight[x - w] += lweight[x - 1];
			}
			weight[mark_f.cols - w] += lweight[mark_f.cols - 1];
		}
		for (int i = 0; i < via.size(); i++) {
			if (via[i].x() >= w / 2 && via[i].x() < weight.size() + w / 2)
				weight[via[i].x() - w / 2] += w*av;
			if (via[i].x() >= w / 2 + 1 && via[i].x() < weight.size() + w / 2 + 1)
				weight[via[i].x() - w / 2 - 1] += w*av*0.7;
			if (via[i].x() >= w / 2 + 2 && via[i].x() < weight.size() + w / 2 + 2)
				weight[via[i].x() - w / 2 - 2] += w*av*0.3;
			if (via[i].x() >= w / 2 - 1 && via[i].x() < weight.size() + w / 2 - 1)
				weight[via[i].x() - w / 2 + 1] += w*av*0.7;
			if (via[i].x() >= w / 2 - 2 && via[i].x() < weight.size() + w / 2 - 2)
				weight[via[i].x() - w / 2 + 2] += w*av*0.3;
		}	
	}
	else {
		weight.resize(mark_f.rows - w + 1, 0);
		vector<float> lweight(mark_f.rows);
		for (int x = 0; x < mark_f.cols; x++) {
			lweight[0] = mark_f.at<float>(0, x);
			for (int y = 1; y < w; y++)
				lweight[y] = lweight[y - 1] + mark_f.at<float>(y, x);
			for (int y = w; y < mark_f.rows; y++) {
				lweight[y] = lweight[y - 1] + mark_f.at<float>(y, x) - mark_f.at<float>(y - w, x);
				weight[y - w] += lweight[y - 1];
			}
			weight[mark_f.rows - w] += lweight[mark_f.rows - 1];
		}
		for (int i = 0; i < via.size(); i++) {
			if (via[i].y() >= w / 2 && via[i].y() < weight.size() + w / 2)
				weight[via[i].y() - w / 2] += w*av;
			if (via[i].y() >= w / 2 + 1 && via[i].y() < weight.size() + w / 2 + 1)
				weight[via[i].y() - w / 2 - 1] += w*av*0.7;
			if (via[i].y() >= w / 2 + 2 && via[i].y() < weight.size() + w / 2 + 2)
				weight[via[i].y() - w / 2 - 2] += w*av*0.3;
			if (via[i].y() >= w / 2 - 1 && via[i].y() < weight.size() + w / 2 - 1)
				weight[via[i].y() - w / 2 + 1] += w*av*0.7;
			if (via[i].y() >= w / 2 - 2 && via[i].y() < weight.size() + w / 2 - 2)
				weight[via[i].y() - w / 2 + 2] += w*av*0.3;
		}
	}
	vector<double> weight_f[WEIGHT_FB], weight_b[WEIGHT_FB];

	for (int i = 0; i < WEIGHT_FB; i++) {
		weight_f[i].resize(weight.size());
		weight_b[i].resize(weight.size());
		for (int j = 0; j < weight.size(); j++) {
			int jf0 = min((int)weight.size() - 1, j + grid);
			int jf1 = min((int)weight.size() - 1, j + grid + 1);
			int jb0 = max(0, j - grid);
			int jb1 = max(0, j - grid - 1);
			if (i == 0) {
				weight_f[0][j] = weight[j] + max(weight[jf0], weight[jf1]);
				weight_b[0][j] = weight[j] + max(weight[jb0], weight[jb1]);
			}
			else {
				weight_f[i][j] = weight[j] + max(weight_f[i - 1][jf0], weight_f[i - 1][jf1]);
				weight_b[i][j] = weight[j] + max(weight_b[i - 1][jb0], weight_b[i - 1][jb1]);
			}
		}					
	}
	
	int base_line;
	double max_base = -100000000;
	for (int i = grid * 2; i < weight.size() - grid * 2; i++)
		if (max_base < weight_f[WEIGHT_FB - 1][i] + weight_b[WEIGHT_FB - 1][i] - weight[i]) {
			max_base = weight_f[WEIGHT_FB - 1][i] + weight_b[WEIGHT_FB - 1][i] - weight[i];
			base_line = i;
		}
	grid_line.push_back(base_line + w / 2);
    qDebug("base%d=%d",row, base_line + w / 2);
	for (int i = base_line; i > 0;) {
		int ib0 = max(0, i - grid);
		int ib1 = max(0, i - grid - 1);
		i = (weight_b[WEIGHT_FB - 1][ib0] > weight_b[WEIGHT_FB - 1][ib1]) ? ib0 : ib1;
		if (i != 0)
			grid_line.push_back(i + w / 2);
	}
	reverse(grid_line.begin(), grid_line.end());
	for (int i = base_line; i < (int)weight.size() - 1;) {
		int if0 = min((int)weight.size() - 1, i + grid);
		int if1 = min((int)weight.size() - 1, i + grid + 1);
		i = (weight_f[WEIGHT_FB - 1][if0] > weight_f[WEIGHT_FB - 1][if1]) ? if0 : if1;
		if (i < (int)weight.size() - 1)
			grid_line.push_back(i + w / 2);
	}	
}

/*
in: mark_f, more near to 1 means more like metal, more near to -1 means more like insu
in: w wire width
in: gl_x, grid line x
in: gl_y, grid line y
in: alpha<=0.1, mark_f prize parameter, more big more prize
out: grid_prob more near to 1 means more like metal, more near to -1 means more like insu
*/
void compute_grid_prob(const Mat & mark_f, int w, const vector<int> & gl_x, const vector<int> & gl_y, float alpha, Mat & grid_prob)
{
	CV_Assert(mark_f.type() == CV_32FC1 && alpha < 0.1);
	grid_prob.create(gl_y.size(), gl_x.size(), CV_32FC1);

	for (int i = 0; i < grid_prob.rows; i++)
		for (int j = 0; j < grid_prob.cols; j++) {
			int y0 = gl_y[i], x0 = gl_x[j];
			float weng = 0, ieng = 0;
			for (int y = max(0, y0 - w / 2); y < min(y0 + w - w / 2, mark_f.rows); y++) {
				const float * p_mark_f = mark_f.ptr<float>(y);
				for (int x = max(0, x0 - w / 2); x < min(x0 + w - w / 2, mark_f.cols); x++) {
                    if (p_mark_f[x] >= 0.5f)
						weng += 1;
					else
                        if (p_mark_f[x] <= -0.5f)
							ieng += 1;
						else {
							float t = p_mark_f[x];
							if (t > 0 && t < alpha)
								t = alpha;
							if (t < 0 && t >-alpha)
								t = -alpha;
							weng += 0.5f + t;
							ieng += 0.5f - t;
						}
				}					
			}
			if (weng + ieng != 0)
				grid_prob.at<float>(i, j) = (float)(weng) / (float)(weng + ieng);
			else
				grid_prob.at<float>(i, j) = 0.5f;
		}
}

/*
in: alpha better<=0.5, grid_prob prize parameter, more big, more prize
inout: grid_prob more near to 1 means more like metal, more near to -1 means more like insu
*/
void post_process_grid_prob(float alpha, Mat & grid_prob)
{
	CV_Assert(grid_prob.type() == CV_32FC1 && grid_prob.cols % 2 == 1 && grid_prob.rows % 2 == 1 && alpha <= 0.7f);

#define PUSH_T_MAX(t, m, s) { if (m < t) { s = m; m = t; } else	if (s < t) s = t; }

    for (int y = 0; y < grid_prob.rows; y += 2) {
        float * p_grid_prob = grid_prob.ptr<float>(y);
        for (int x = 0; x < grid_prob.cols; x += 2) {
            float mmax = -10000000;
            float submax = -10000000;
            float t;

            if (x>0)
                mmax = min(p_grid_prob[x - 2], p_grid_prob[x - 1]);
            if (x + 2 < grid_prob.cols) {
                t = min(p_grid_prob[x + 2], p_grid_prob[x + 1]);
                PUSH_T_MAX(t, mmax, submax);
            }
            if (y > 0) {
                t = min(grid_prob.at<float>(y - 2, x), grid_prob.at<float>(y - 1, x));
                PUSH_T_MAX(t, mmax, submax);
            }
            if (y + 2 < grid_prob.rows) {
                t = min(grid_prob.at<float>(y + 2, x), grid_prob.at<float>(y + 1, x));
                PUSH_T_MAX(t, mmax, submax);
            }
            if (mmax + submax > 2 * p_grid_prob[x] && mmax + submax > 1.2f) //feed forward p_grid_prob[x]
                p_grid_prob[x] = p_grid_prob[x] * (1 - alpha) + alpha * (mmax + submax) / 2;

        }
    }

#undef PUSH_T_MAX
}

/*
in: grid_prob, more near to 1 means more like metal, more near to -1 means more like insu
in: rule
out: connet, each grid metal connection, up 1, right 2, down 4, left 8
*/
void assemble_grid(const Mat & grid_prob, unsigned long long rule, Mat & conet)
{
	CV_Assert(grid_prob.type() == CV_32FC1 && grid_prob.cols % 2 == 1 && grid_prob.rows % 2 == 1);
	conet.create(grid_prob.rows / 2 + 1, grid_prob.cols / 2 + 1, CV_32SC1);
	conet = 0;

	vector <unsigned long long> edge;

	for (int y = 0; y < grid_prob.rows; y++) {
		const float * p_grid_prob_1 = grid_prob.ptr<float>(y - 1);
		const float * p_grid_prob = grid_prob.ptr<float>(y);
		const float * p_grid_prob_a1 = grid_prob.ptr<float>(y + 1);
        for (int x = (y + 1) & 1; x < grid_prob.cols; x += 2) {
			float t;
            if (y & 1) {
				t = min(p_grid_prob[x], p_grid_prob_1[x]);
				t = min(p_grid_prob_a1[x], t);
				CV_Assert(t <= 1);
				if (t > 0.5f) {
					unsigned long long ti = t * 1000000;
					ti = (ti << 32) | (y << 16) | x;
					edge.push_back(ti);
				}
			}
			else {
				t = min(p_grid_prob[x], p_grid_prob[x - 1]);
				t = min(p_grid_prob[x + 1], t);
				CV_Assert(t <= 1);
				if (t > 0.5f) {
					unsigned long long ti = t * 1000000;
					ti = (ti << 32) | (y << 16) | x;
					edge.push_back(ti);
				}					
			}
		}
	}
	sort(edge.begin(), edge.end(), greater<unsigned long long>());

	Mat belong(grid_prob.rows / 2 + 1, grid_prob.cols / 2 + 1, CV_32SC1);
	vector<int> path;
	if (rule & RULE_TREE)
		for (int y = 0; y < belong.rows; y++)
			for (int x = 0; x < belong.cols; x++)
				belong.at<int>(y, x) = (y << 16) | x;

	for (int i = 0; i < edge.size(); i++) {
		int y = (edge[i] >> 16) & 0x7fff;
		int x = edge[i] & 0x7fff;		
		int y0 = y / 2;
        int y1 = (y & 1) ? y / 2 + 1 : y / 2;
		int x0 = x / 2;
        int x1 = (x & 1) ? x / 2 + 1 : x / 2;
        int a0 = (y & 1) ? 4 : 2;
        int a1 = (y & 1) ? 1 : 8;
		int e0 = conet.at<int>(y0, x0);
		int e1 = conet.at<int>(y1, x1);

        CV_Assert((y & 1) != (x & 1) && (e0 & a0)==0 && (e1 & a1)==0);

		if ((rule & RULE_TREE) && belong.at<int>(y0, x0) == belong.at<int>(y1, x1))
			continue;
		if ((rule & RULE_NO_4CONN) && (e0 + a0 == 15 || e1 + a1 == 15))
			continue;
		if ((rule & RULE_NO_3CONN_PAIR) && (e0 == 5 && e1 == 5 || e0 == 10 && e1 == 10))
			continue;

        if (rule & RULE_TREE) {
			int t;
			if (belong.at<int>(y0, x0) < belong.at<int>(y1, x1)) {
				t = belong.at<int>(y0, x0);
				path.push_back((y1 << 16) | x1);
			}
			else {
				t = belong.at<int>(y1, x1);
				path.push_back((y0 << 16) | x0);
			}
			while (!path.empty()) {
				int yy = path.back() >> 16;
				int xx = path.back() & 0x7fff;
				int e = conet.at<int>(yy, xx);
				path.pop_back();
				belong.at<int>(yy, xx) = t;
				if ((e & 1) && belong.at<int>(yy - 1, xx) != t)
					path.push_back(((yy - 1) << 16) | xx);
				if ((e & 2) && belong.at<int>(yy, xx + 1) != t)
					path.push_back((yy << 16) | (xx + 1));
				if ((e & 4) && belong.at<int>(yy + 1, xx) != t)
					path.push_back(((yy + 1) << 16) | xx);
				if ((e & 8) && belong.at<int>(yy, xx - 1) != t)
					path.push_back((yy << 16) | (xx - 1));
			}
		}

		conet.at<int>(y0, x0) = e0 + a0;
		conet.at<int>(y1, x1) = e1 + a1;
	}
}

/*
in: connet, each grid metal connection, up 1, right 2, down 4, left 8
in: gl_x, grid line x
in: gl_y, grid line y
out: obj_sets
*/
void grid2_wire_obj(const Mat & conet, const vector<int> & gl_x, const vector<int> & gl_y, vector<MarkObj> & obj_sets)
{
	CV_Assert(conet.rows == gl_y.size() && conet.cols == gl_x.size() && conet.type()==CV_32SC1);
	MarkObj wire;
	wire.type = OBJ_WIRE;
	wire.type2 = 0;
	wire.select_state = 0;

	for (int y = 0; y < conet.rows; y++) {
		int state = 0;
		wire.p0.setY(gl_y[y]);
		wire.p1.setY(gl_y[y]);
		const int * p_conet = conet.ptr<int>(y);
		for (int x = 0; x < conet.cols; x++) 
			if (p_conet[x] & 2) {
				if (state == 0) 
					wire.p0.setX(gl_x[x]);
				state = 1;
			}
			else {
				if (state == 1) {
					wire.p1.setX(gl_x[x]);
					obj_sets.push_back(wire);
				}
				state = 0;
			}		
	}
	for (int x = 0; x < conet.cols; x++) {
		int state = 0;
		wire.p0.setX(gl_x[x]);
		wire.p1.setX(gl_x[x]);
		for (int y = 0; y < conet.rows; y++)
			if (conet.at<int>(y, x) & 4) {
				if (state == 0)
					wire.p0.setY(gl_y[y]);
				state = 1;
			}
			else {
				if (state == 1) {
					wire.p1.setY(gl_y[y]);
					obj_sets.push_back(wire);
				}
				state = 0;
			}
	}
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
		{ "", M_EDGE},
		{ "", M_NOEDGE},
		{ "", M_INL },											//M_INL
		{ "", M_WNL },											//M_WNL
		{ "", M_VNL },											//M_VNL
		{ "", M_W_INL },										//M_W_INL
		{ "", M_V_INL }											//M_V_INL
};

int VWExtractClasic::fill_mark(const std::vector<MarkObj> & obj_sets)
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

void VWExtractClasic::train(string file_name, const vector<MarkObj> & obj_sets, int _feature_method, int _learn_method)
{		
	img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);

	feature_method = _feature_method;
	learn_method = _learn_method;
	
	if (obj_sets.empty()) 
		return;
	
	int learn_point_num = fill_mark(obj_sets);
	int feature_size;
	int learn_point = 0;
	
	if (feature_method == FEA_GRADXY_HIST_7x7 || feature_method == FEA_GRADXY_HIST_6x6 || feature_method == FEA_GRADXY_HIST_9x9)
		feature_size = 10;
	if (feature_method == FEA_GRADXY_HIST_9x5)
		feature_size = 4;
	Mat pt_features(learn_point_num, feature_size, CV_32FC1);
	Mat_<float> pt_types(learn_point_num, 1);
	for (unsigned la = 0; la < l_areas.size(); la++) {
		QRect rect = l_areas[la].learn_rect.marginsRemoved(QMargins(wire_wd / 2, wire_wd / 2, wire_wd / 2, wire_wd / 2));
		for (int y = rect.top(); y < rect.bottom(); y++) {
			unsigned char * p_mark = mark.ptr<unsigned char>(y);
			unsigned char * p_img = img.ptr<unsigned char>(y);			
			for (int x = rect.left(); x < rect.right(); x++) {
				if (p_mark[x] != M_UNKNOW && p_mark[x] < M_INL) {
					if (learn_method == LEARN_SVM)
						pt_types(learn_point, 0) = p_mark[x];
					if (learn_method == LEARN_BAYES) {
						switch (p_mark[x]) {
						case M_W_I:
						case M_V_I:
						case M_W_V:
						case M_V_I_W:
						case M_V_I_V:
							pt_types(learn_point, 0) = M_EDGE;
							break;
						default:
							pt_types(learn_point, 0) = M_NOEDGE;
							break;
						}
					}
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
					case FEA_GRADXY_HIST_9x5:
						feature_extract_9x5(p_img + x, (int)img.step[0], pt_features.ptr<float>(learn_point));
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
	if (learn_method == LEARN_BAYES)
		bayes.train(pt_features, pt_types);	
}

void VWExtractClasic::extract(string file_name, QRect rect, vector<MarkObj> & obj_sets)
{
	img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);
	mark.create(img.rows, img.cols, CV_8UC1);
	
	Mat_<float> feature;
	if (feature_method == FEA_GRADXY_HIST_7x7 || feature_method == FEA_GRADXY_HIST_6x6 || feature_method == FEA_GRADXY_HIST_9x9)
		feature.create(1, 10);
	if (feature_method == FEA_GRADXY_HIST_9x5)
		feature.create(1, 4);
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
			case FEA_GRADXY_HIST_9x5:
				feature_extract_9x5(p_img + x, (int) img.step[0], feature.ptr<float>(0));
				break;
			}
			float response;
			if (learn_method == LEARN_SVM)
				response = svm.predict(feature);
			if (learn_method == LEARN_BAYES)
				response = bayes.predict(feature);
			CV_Assert(response < M_INL);
			p_mark[x] = response;
		}
	}		
}

void VWExtractClasic::get_feature(int x, int y, vector<float> & feature)
{
}

void VWExtractStat::train(string file_name, const std::vector<MarkObj> & obj_sets, int _feature_method, int _learn_method)
{
	img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);
	mark.create(img.rows, img.cols, CV_8UC1);
	mark = M_UNKNOW;

	vector<int> dx;
	for (int y = -via_rd; y <= via_rd; y++)
		dx.push_back((int)sqrt((float)via_rd*via_rd - y *y));

	l_areas.clear();
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
				QPoint lt(min(line_in.p1().x(), line_in.p2().x()), min(line_in.p1().y(), line_in.p2().y()));
				QPoint rb(max(line_in.p1().x(), line_in.p2().x()), max(line_in.p1().y(), line_in.p2().y()));
				l_areas[j].wires.push_back(QRect(lt, rb) & l_areas[j].learn_rect);
				}
			break;
		case OBJ_VIA:
			for (unsigned j = 0; j < l_areas.size(); j++)
				if (l_areas[j].learn_rect.contains(obj_sets[i].p1))
					l_areas[j].vias.push_back(obj_sets[i].p1);
		}
	}

	vector<float> wire_th, via_th;
	vector<float> edge_grad_r[2], edge_grads_r[2], insu_grad_r[2], edge_grad_d[2], edge_grads_d[2], insu_grad_d[2];
	for (unsigned la = 0; la < l_areas.size(); la++) {
		//collect feature sample for via
		QRect rect = l_areas[la].learn_rect;
		float feature[10];		
		
		for (int v = 0; v < l_areas[la].vias.size(); v++) {
			feature_extract_via(img.ptr<unsigned char>(l_areas[la].vias[v].y()) + l_areas[la].vias[v].x(), (int)img.step[0],
				via_rd, feature);
			fill_circle(mark, l_areas[la].vias[v].x(), l_areas[la].vias[v].y(), via_rd + 4, M_V_I, rect);
			fill_circle(mark, l_areas[la].vias[v].x(), l_areas[la].vias[v].y(), via_rd, M_V, rect); // fill via mark
			float th = 0;
			for (int j = 0; j < 4; j++)
				th += feature[j];
			via_th.push_back(th);
		}
		
		for (int w = 0; w < l_areas[la].wires.size(); w++) {
			QPoint lt = l_areas[la].wires[w].topLeft();
			QPoint rb = l_areas[la].wires[w].bottomRight();
			QPoint d = (lt.x() >= rb.x()) ? QPoint(0, 3) : QPoint(3, 0);
			int len = (lt.x() >= rb.x()) ? rb.y() - lt.y() : rb.x() - lt.x();
			fill_rect_check(mark, lt, rb, M_W, wire_wd / 2, wire_wd - wire_wd / 2 - 1, wire_wd / 2, wire_wd - wire_wd / 2 - 1,
				rect, (1 << M_V) | (1 << M_V_I)); // fill wire mark for wire feature sample extract
			for (int i = 0; i < len / 3; i++) {
				lt += d;
				if (mark.at<unsigned char>(lt.y(), lt.x()) == M_V)
					continue;
				feature_extract_via(img.ptr<unsigned char>(lt.y()) + lt.x(), (int)img.step[0],
					via_rd, feature);
				float th = 0;
				for (int j = 0; j < 4; j++)
					th += feature[j];
				wire_th.push_back(th);
			}
		}

		//collect feature sample for wire
		for (int w = 0; w < l_areas[la].wires.size(); w++) {
			QPoint lt = l_areas[la].wires[w].topLeft();
			QPoint rb = l_areas[la].wires[w].bottomRight();
			QPoint d, d1, d2;
			int len;
			int grad_row;
			if (lt.x() >= rb.x()) {
				d = QPoint(0, 2);
				len = rb.y() - lt.y();
				d1 = QPoint(-wire_wd / 2 - insu_wd / 2 - 1, 0);
				d2 = QPoint(1, 0);
				grad_row = 0;
			}
			else {
				d = QPoint(2, 0);
				len = rb.x() - lt.x();
				d1 = QPoint(0, -wire_wd / 2 - insu_wd / 2 - 1);
				d2 = QPoint(0, 1);
				grad_row = 1;
			}

			for (int i = 0; i < len / 2; i++) {
				lt += d;
				if (mark.at<unsigned char>(lt.y(), lt.x()) == M_V || mark.at<unsigned char>(lt.y(), lt.x()) == M_V_I)
					continue;
				QPoint p1 = lt + d1;
				Vec<float, 5> feature_vec;
				vector<float> grad_r, grad_d;
				bool valid_wire = true;

				for (int j = 0; j < wire_wd + insu_wd + 2; j++, p1 += d2) {
					unsigned char mark_at = mark.at<unsigned char>(p1.y(), p1.x());
					if (mark_at == M_V || mark_at == M_V_I)
						valid_wire = false;
					if ((j < insu_wd / 2 || j > wire_wd + insu_wd / 2 + 2) && mark_at == M_W)
						valid_wire = false;
					if (!valid_wire)
						break;

					feature_vec = feature_extract_5(img.ptr<unsigned char>(p1.y()) + p1.x(), (int)img.step[0],
						wire_wd + 2, wire_wd - 2, via_rd, dx);
					if (!grad_row) {
						grad_r.push_back(feature_vec[2]); //left right grad
						grad_d.push_back(feature_vec[3]);
					}
					else {
						grad_r.push_back(feature_vec[0]); //up down grad
						grad_d.push_back(feature_vec[1]);
					}
				}
				if (!valid_wire)
					continue;
				int max_idx;
				float grad_max = -100000;
				for (int j = insu_wd / 2 - 2; j < insu_wd / 2 + 3; j++) 
					if (grad_max < grad_r[j] - grad_r[j + wire_wd]) {
						grad_max = grad_r[j] - grad_r[j + wire_wd];
						max_idx = j;
					}
				if (grad_max < 0) {
					qWarning("Metal edge grad <0 at %s: (%d,%d)", file_name.c_str(), lt.x(), lt.y());
					continue;
				}

				edge_grads_r[grad_row].push_back(grad_max);
				edge_grad_r[grad_row].push_back(fabs(grad_r[max_idx]));
				edge_grad_r[grad_row].push_back(fabs(grad_r[max_idx + wire_wd]));
				for (int j = 0; j < max_idx - 1; j++)
					insu_grad_r[grad_row].push_back(fabs(grad_r[j]));
				for (int j = max_idx + wire_wd + 2; j < grad_r.size(); j++)
					insu_grad_r[grad_row].push_back(fabs(grad_r[j]));

				//do same step for grad_d
				grad_max = -1000000;
				for (int j = insu_wd / 2 - 2; j < insu_wd / 2 + 3; j++)
					if (grad_max < grad_d[j] - grad_d[j + wire_wd]) {
						grad_max = grad_d[j] - grad_d[j + wire_wd];
						max_idx = j;
					}
				edge_grads_d[grad_row].push_back(grad_max);
				edge_grad_d[grad_row].push_back(fabs(grad_d[max_idx]));
				edge_grad_d[grad_row].push_back(fabs(grad_d[max_idx + wire_wd]));
				for (int j = 0; j < max_idx - 1; j++)
					insu_grad_d[grad_row].push_back(fabs(grad_d[j]));
				for (int j = max_idx + wire_wd + 2; j < grad_d.size(); j++)
					insu_grad_d[grad_row].push_back(fabs(grad_d[j]));
			}
		}
	}

	//compute via threshold
	if (via_th.empty() || wire_th.empty()) {
		qInfo("learning rect doesn't contain via or wire");
		return;
	}
	float thw=0, thv=0;
	sort(wire_th.begin(), wire_th.end(), greater<float>());
	sort(via_th.begin(), via_th.end(), less<float>());
	if (wire_th[0] < via_th[0]) {
		thw = wire_th[0];
		thv = via_th[0];
		qDebug("min via feature is greater than max wire feature, perfect, thw=%f, thv=%f", thw, thv);
	}
	else {
		int vnum = (5 < via_th.size()) ? 5 : (int) via_th.size();
		int wnum = (5 < wire_th.size()) ? 5 : (int) wire_th.size();
		for (int i = 0; i < vnum; i++)
			thv += via_th[i];
		for (int i = 0; i < wnum; i++)
			thw += wire_th[i];
		thv = thv / vnum;
		thw = thw / wnum;
		if (thw < thv)
			qDebug("min 5 via feature is greater than max 5 wire feature, imperfect. thw=%f, thv=%f", thw, thv);
		else
			qDebug("min 5 via feature is less than max 5 wire feature, bad. thw=%f, thv=%f", thw, thv);
	}	

	via_feature_th = (thv + thw) / 2;

	//compute wire threshold
	if (param1 < 0.07f)
		param1 = 0.07f;
	if (param1 > 0.39f)
		param1 = 0.39f;
	for (int row_col = 0; row_col < 2; row_col++) {
		float g0, g1, g2, insu_avg_grad, edge_avg_grad, energy_gap;
		if (edge_grads_r[row_col].size() < 50)
			continue;
		sort(edge_grad_r[row_col].begin(), edge_grad_r[row_col].end(), less<float>());
		sort(edge_grads_r[row_col].begin(), edge_grads_r[row_col].end(), less<float>());
		sort(insu_grad_r[row_col].begin(), insu_grad_r[row_col].end(), greater<float>());
		insu_avg_grad = insu_grad_r[row_col][insu_grad_r[row_col].size() / 2];
		edge_avg_grad = edge_grad_r[row_col][edge_grad_r[row_col].size() / 2];
		do {
			g0 = insu_grad_r[row_col][insu_grad_r[row_col].size()*param1];
			g1 = edge_grad_r[row_col][edge_grad_r[row_col].size()*param1];
			if (g1 - g0 < (edge_avg_grad - insu_avg_grad) / 10)
				param1 += 0.01f;
			else
				break;
		} while (1);
		g2 = edge_grads_r[row_col][edge_grads_r[row_col].size()*param1];
		energy_gap = (g1 - g0) / (edge_avg_grad - insu_avg_grad);
		insu_feature_th[row_col] = g0 *0.65f + g1 * 0.35f;
		edge_feature_th1[row_col] = g1 *0.65f + g0 *0.35f;
		edge_feature_th2[row_col] = g2;
		qDebug("ratio=%f, g0=%f, g1=%f, g2=%f, insu_avg=%f, edge_avg=%f, energy_gap=%f",
			param1, g0, g1, g2, insu_avg_grad, edge_avg_grad, energy_gap);

		sort(edge_grad_d[row_col].begin(), edge_grad_d[row_col].end(), less<float>());
		sort(edge_grads_d[row_col].begin(), edge_grads_d[row_col].end(), less<float>());
		sort(insu_grad_d[row_col].begin(), insu_grad_d[row_col].end(), greater<float>());
		insu_avg_grad = insu_grad_d[row_col][insu_grad_d[row_col].size() / 2];
		edge_avg_grad = edge_grad_d[row_col][edge_grad_d[row_col].size() / 2];
		g0 = insu_grad_d[row_col][insu_grad_d[row_col].size()*param1];
		g1 = edge_grad_d[row_col][edge_grad_d[row_col].size()*param1];
		g2 = edge_grads_d[row_col][edge_grads_d[row_col].size()*param1];
		qDebug("%d: ratio=%f, g0=%f, g1=%f, g2=%f, insu_avg=%f, edge_avg=%f, energy_gap=%f", row_col,
			param1, g0, g1, g2, insu_avg_grad, edge_avg_grad, (g1 - g0) / (edge_avg_grad - insu_avg_grad));

		if (energy_gap > (g1 - g0) / (edge_avg_grad - insu_avg_grad))
			use_ratio[row_col] = true;
		else {
			use_ratio[row_col] = false;
			insu_feature_th[row_col] = g0 *0.65f + g1 * 0.35f;
			edge_feature_th1[row_col] = g1 *0.65f + g0 *0.35f;
			edge_feature_th2[row_col] = g2;
		}
	}

	for (int row_col = 0; row_col < 2; row_col++) 
		if (edge_grads_r[row_col].size() < 50) {
			qDebug("%d sample number %d, too low, use %d threshold", row_col, edge_grads_r[row_col].size(), 1 - row_col);
			insu_feature_th[row_col] = insu_feature_th[1 - row_col];
			edge_feature_th1[row_col] = edge_feature_th1[1 - row_col];
			edge_feature_th2[row_col] = edge_feature_th2[1 - row_col];
			use_ratio[row_col] = use_ratio[1 - row_col];
		}
}

void VWExtractStat::extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets)
{
	img = imread(file_name, 0);
	CV_Assert(img.type() == CV_8UC1);

	//feature extract
	mark.create(img.rows, img.cols, CV_8UC1);
	mark1.create(img.rows, img.cols, CV_8UC1);
	mark2.create(img.rows, img.cols, CV_8UC1);
	mark = M_UNKNOW;
	Mat mark_f(rect.height(), rect.width(), CV_32FC1);
	mark_f = 0;
	Mat ew(rect.height(), rect.width(), CV_32FC2);
	Mat link(img.rows, img.cols, CV_32SC1);
	vector <int> link_head(1000, -1);
	vector<int> xlimit;
	for (int y = -via_rd; y <= via_rd; y++)
		xlimit.push_back((int)sqrt((float)via_rd*via_rd - y *y));

	for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
		float * p_ew = ew.ptr<float>(y - rect.y());
		Vec<float, 5> feature_vec;
		for (int x = rect.x(), xx=0; x < rect.x() + rect.width(); x++, xx++) {			
			feature_vec = feature_extract_5(img.ptr<unsigned char>(y) + x, (int)img.step[0],
				wire_wd + 2, wire_wd - 2, via_rd, xlimit);
			p_ew[xx * 2] = (use_ratio[0]) ? feature_vec[2] : feature_vec[3]; //left right grad
			p_ew[xx * 2 + 1] = (use_ratio[1]) ? feature_vec[0] : feature_vec[1]; // up down grad
			int idx = 1000 * feature_vec[4];
			link.at<int>(y, x) = link_head[idx];
			link_head[idx] = (y << 16) | x;
		}
	}

	//find via	
	vector<QPoint> vias;
	MarkObj via;
	via.type = OBJ_VIA;
	via.type2 = 0;
	via.select_state = 0;
	for (int i = (int) link_head.size() - 1; i >= 0; i--) {
		int yx = link_head[i];
		int y, x;
		while (yx >= 0) {
			y = yx >> 16;
			x = yx & 0xffff;
			float feature[10];
			if (mark.at<unsigned char>(y, x) == M_UNKNOW) {
				feature_extract_via(img.ptr<unsigned char>(y) +x, (int)img.step[0], via_rd, feature);
				float th = 0;
				for (int j = 0; j < 4; j++)
					th += feature[j];
				if (th > via_feature_th) {
                    QPoint via_center(x - rect.x(), y - rect.y());
                    vias.push_back(via_center);
					via.p0 = via_center;
					via.p1 = via_center;
					obj_sets.push_back(via);
					fill_circle(mark, x, y, via_rd / 2, M_V, rect);
					fill_circle_check(mark, x, y, via_rd + 2, M_VNL, rect, 1 << M_V); 
					fill_circle_check(mark, x, y, via_rd + 4, M_V_INL, rect, (1 << M_VNL) | (1 << M_V));
				}
			}
			else
				if (mark.at<unsigned char>(y, x) == M_V_INL)
					break;
			yx = link.at<int>(y, x);
		}
		if (yx >= 0)
			if (mark.at<unsigned char>(y, x) == M_V_INL)
				break;
	}
	
	//find metal
	for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		unsigned char * p_mark1 = mark1.ptr<unsigned char>(y);
		for (int x = rect.x(); x < rect.x() + rect.width(); x++)
			p_mark1[x] = (p_mark[x] == M_V || p_mark[x] == M_VNL) ? 2 : 0; //M_V and M_VNL is for remove area, M_V_INL is not remove
	}
	remove_via(mark1, img, (int)param2, insu_wd - 1);
	for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);
		float * p_ew = ew.ptr<float>(y - rect.y());
		Vec<float, 5> feature_vec;
		for (int x = rect.x(), xx = 0; x < rect.x() + rect.width(); x++, xx++) 
			if (p_mark[x] == M_V || p_mark[x] == M_VNL || p_mark[x] == M_V_INL) { //M_V, M_VNL, M_V_INL needs recompute
				feature_vec = feature_extract_5(img.ptr<unsigned char>(y) +x, (int)img.step[0],
					wire_wd + 2, wire_wd - 2, via_rd, xlimit);
				p_ew[xx * 2] = (use_ratio[0]) ? feature_vec[2] : feature_vec[3]; //left right grad
				p_ew[xx * 2 + 1] = (use_ratio[1]) ? feature_vec[0] : feature_vec[1]; // up down grad
			}
	}

	estimate_metal_pblty(ew, insu_feature_th, edge_feature_th1, edge_feature_th2, param3, 2.5f,
		insu_wd - 1, wire_wd, (int)param2, mark_f, iter_num);
	vector<int> gl_x, gl_y, gl_x2, gl_y2;
	Mat grid_prob, conet;
	find_grid_line(mark_f, vias, 10, wire_wd, grid_wd, 0, gl_x);
	find_grid_line(mark_f, vias, 10, wire_wd, grid_wd, 1, gl_y);

	gl_x2.resize(gl_x.size() * 2 - 1);
	gl_y2.resize(gl_y.size() * 2 - 1);
	gl_x2[0] = gl_x[0];
	for (int i = 0; i + 1 < gl_x.size(); i++) {
		gl_x2[i * 2 + 2] = gl_x[i + 1];
		gl_x2[i * 2 + 1] = (gl_x[i] + gl_x[i + 1]) / 2;
	}
	gl_y2[0] = gl_y[0];
	for (int i = 0; i + 1 < gl_y.size(); i++) {
		gl_y2[i * 2 + 2] = gl_y[i + 1];
		gl_y2[i * 2 + 1] = (gl_y[i] + gl_y[i + 1]) / 2;
	}
	compute_grid_prob(mark_f, wire_wd, gl_x2, gl_y2, 0.08f, grid_prob);
	post_process_grid_prob(0.5, grid_prob);

    assemble_grid(grid_prob, RULE_TREE | RULE_NO_4CONN | RULE_NO_3CONN_PAIR, conet);

	grid2_wire_obj(conet, gl_x, gl_y, obj_sets);
    for (int i=0; i < obj_sets.size(); i++) {
        obj_sets[i].p0.setX(obj_sets[i].p0.x() + rect.x());
        obj_sets[i].p0.setY(obj_sets[i].p0.y() + rect.y());
        obj_sets[i].p1.setX(obj_sets[i].p1.x() + rect.x());
        obj_sets[i].p1.setY(obj_sets[i].p1.y() + rect.y());
    }
	for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
		unsigned char * p_mark = mark.ptr<unsigned char>(y);		
		float * p_mark_f = mark_f.ptr<float>(y - rect.y());
		for (int x = rect.x(), xx = 0; x < rect.x() + rect.width(); x++, xx++) {
			if (p_mark[x] == M_V_INL || p_mark[x] == M_VNL)
				p_mark[x] = M_UNKNOW;
			if (p_mark[x] == M_UNKNOW) {
				if (fabs(p_mark_f[xx]) < 0.1f) 
					p_mark[x] = M_V_I_W;
				else
					p_mark[x] = (p_mark_f[xx] < 0) ? M_I : M_W;
			}
		}
	}

	mark1 = 0;
	mark2 = 0;
	for (int y = rect.y(); y < rect.y() + rect.height(); y++) {
		unsigned char * p_mark1 = mark1.ptr<unsigned char>(y);
		unsigned char * p_mark2 = mark2.ptr<unsigned char>(y);
		float * p_ew = ew.ptr<float>(y - rect.y());
		for (int x = rect.x(), xx = 0; x < rect.x() + rect.width(); x++, xx++) {
			int c = (int) (fabs(p_ew[xx * 2]) * 100);
			p_mark1[x] = (c > 255) ? 255 : c;
			c = (int)(fabs(p_ew[xx * 2 + 1]) * 100);
			p_mark2[x] = (c > 255) ? 255 : c;
		}
	}
}

void VWExtractStat::get_feature(int x, int y, vector<float> & feature)
{
	if (img.empty() || y<20 || y>img.rows - 20 || x<20 || x>img.cols - 20) {
		feature.clear();
		return;
	}

	Vec<float, 5> feature_vec;
	vector<int> xlimit;
	for (int yy = -via_rd; yy <= via_rd; yy++)
		xlimit.push_back((int)sqrt((float)via_rd*via_rd - yy *yy));
	feature_vec = feature_extract_5(img.ptr<unsigned char>(y) +x, (int)img.step[0],
		wire_wd + 2, wire_wd - 2, via_rd, xlimit);
	feature.resize(5);
	for (int i = 0; i < 5; i++)
		feature[i] = feature_vec[i];
}
