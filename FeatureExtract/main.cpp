#include "featurewindow.h"
#include <QApplication>
#include "featext.h"
#include "bundleadjust.h"
#include "opencv2/highgui/highgui.hpp"
#include <QDateTime>
#include <QThread>

static FILE * fp = NULL;
#define QMSG_FLUSH 1

void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	if (fp == NULL) {
		fp = fopen("log.txt", "wt");
		if (fp == NULL)
			exit(-1);
	}

	QTime datetime;
	datetime = QTime::currentTime();
	QString str_dt = datetime.toString("hh:mm:ss.zzz");

	if (context.function == NULL) {
		unsigned thread_id = quintptr(QThread::currentThreadId());
		switch (type) {
		case QtDebugMsg:
			fprintf(fp, "<D>[%s] [%d] %s\n", qPrintable(str_dt), thread_id, qPrintable(msg));
#if QMSG_FLUSH
			fflush(fp);
#endif
			break;
		case QtInfoMsg:
			fprintf(fp, "<I>[%s] [%d] %s\n", qPrintable(str_dt), thread_id, qPrintable(msg));
#if QMSG_FLUSH
			fflush(fp);
#endif
			break;
		case QtWarningMsg:
			fprintf(fp, "<W>[%s] [%d] %s\n", qPrintable(str_dt), thread_id, qPrintable(msg));
			fflush(fp);
			break;
		case QtCriticalMsg:
			fprintf(fp, "<E>[%s] [%d] %s\n", qPrintable(str_dt), thread_id, qPrintable(msg));
			fflush(fp);
			break;
		case QtFatalMsg:
			fprintf(fp, "<F>[%s] [%d] %s\n", qPrintable(str_dt), thread_id, qPrintable(msg));
			fclose(fp);
			exit(-1);
		}
		return;
	}
	char func[100];
	char file[100];
	int size, kuo = 0;
	const char * pend, *pmid, *pbegin;
	size = (int)strlen(context.function);
	for (pend = context.function + size; pend != context.function; pend--) {
		if (*pend == ')')
			kuo++;
		if (*pend == '(') {
			kuo--;
			if (kuo <= 0)
				break;
		}
	}
	if (pend == context.function)
		pend = context.function + size;

	for (pmid = pend; pmid != context.function && *pmid != ':'; pmid--);
	if (*pmid == ':')
		pmid++;
	size = pend - pmid;
	memcpy(func, pmid, size);
	func[size] = 0;
	while (*(pmid - 1) == ':' && pmid != context.function)
		pmid--;
	for (pbegin = pmid; *pbegin != ' ' && pbegin != context.function; pbegin--);
	size = pmid - pbegin;
	memcpy(file, pbegin, size);
	file[size] = 0;

	switch (type) {
	case QtDebugMsg:
		fprintf(fp, "<D>[%d,%s] [%s] [%s] %s\n", context.line, file, qPrintable(str_dt), func, qPrintable(msg));
#if QMSG_FLUSH
		fflush(fp);
#endif
		break;
	case QtInfoMsg:
		fprintf(fp, "<I>[%d,%s] [%s] [%s] %s\n", context.line, file, qPrintable(str_dt), func, qPrintable(msg));
#if QMSG_FLUSH
		fflush(fp);
#endif
		break;
	case QtWarningMsg:
		fprintf(fp, "<W>[%d,%s] [%s] [%s] %s\n", context.line, file, qPrintable(str_dt), func, qPrintable(msg));
		fflush(fp);
		break;
	case QtCriticalMsg:
		fprintf(fp, "<E>[%d,%s] [%s] [%s] %s\n", context.line, file, qPrintable(str_dt), func, qPrintable(msg));
		fflush(fp);
		break;
	case QtFatalMsg:
		fprintf(fp, "<F>[%d,%s] [%s] [%s] %s\n", context.line, file, qPrintable(str_dt), func, qPrintable(msg));
		fclose(fp);
		exit(-1);
	}
}

#ifdef Q_OS_WIN
#include <Windows.h>
#include <Dbghelp.h>
void print_stack(void)
{
	unsigned int   i;
	void         * stack[100];
	unsigned short frames;
	SYMBOL_INFO  * symbol;
	HANDLE         process;

	process = GetCurrentProcess();

	SymInitialize(process, NULL, TRUE);

	frames = CaptureStackBackTrace(0, 100, stack, NULL);
	symbol = (SYMBOL_INFO *)calloc(sizeof(SYMBOL_INFO)+256 * sizeof(char), 1);
	symbol->MaxNameLen = 255;
	symbol->SizeOfStruct = sizeof(SYMBOL_INFO);

	for (i = 0; i < frames; i++)
	{
		SymFromAddr(process, (DWORD64)(stack[i]), 0, symbol);

		qInfo("%i: %s ", frames - i - 1, symbol->Name);
	}

	free(symbol);
}
#else
void print_stack(void) {

}
#endif

class MorphoFeatures
{
private:
	int threShold, size;
	Mat cross;
	Mat diamond;
	Mat square;
	Mat  x;
public:
	MorphoFeatures() :
		threShold(15), cross(5, 5, CV_8U, Scalar(0)), diamond(5, 5, CV_8U, Scalar(0)), square(5, 5, CV_8U, Scalar(1)), x(5, 5, CV_8U, Scalar(0))
	{
		/*
		for (int i = 0; i < 5; i++)
		{
			cross.at<uchar>(2, i) = 1;
			cross.at<uchar>(i, 2) = 1;
		}

		diamond.at<uchar>(0, 2) = 1;
		diamond.at<uchar>(1, 1) = 1;
		diamond.at<uchar>(1, 3) = 1;
		diamond.at<uchar>(2, 0) = 1;
		diamond.at<uchar>(2, 4) = 1;
		diamond.at<uchar>(3, 1) = 1;
		diamond.at<uchar>(3, 3) = 1;
		diamond.at<uchar>(4, 2) = 1;

		for (int i = 0; i < 5; i++)
		{
			x.at<uchar>(i, i) = 1;
			x.at<uchar >(4 - i, i) = 1;
		}*/
		set_block_size(2);
	}

	void setThreshold(int t)
	{
		threShold = t;
	}

	void set_block_size(int s)
	{
		cross.create(2 * s + 1, 2 * s + 1, CV_8U);
		cross = 0;
		x = cross.clone();
		square = cross.clone();
		square = 1;
		
		for (int i = 0; i <= 2 * s; i++) {
			cross.at<uchar>(s, i) = 1;
			cross.at<uchar>(i, s) = 1;
		}
		
		for (int i = 0; i <= 2 * s; i++)
		{
			x.at<uchar>(i, i) = 1;
			x.at<uchar >(2 * s - i, i) = 1;
		}
		if (s == 1)
			diamond = square.clone();
		else {
			diamond = cross.clone();
			for (int i = -s; i <= s; i++) 
			for (int j= abs(i) - s; j <= s - abs(i); j++) {
				diamond.at<uchar>(i + s, j + s) = 1;
			}
		}
	}

	void applyThreshold(Mat & result)
	{
		if (threShold > 0)
		{
			threshold(result, result, threShold, 255, THRESH_BINARY);
		}
	}

	Mat getCorners(const Mat &image)
	{
		Mat result;

		dilate(image, result, cross);

		erode(result, result, diamond);

		Mat result2;

		dilate(image, result2, x);

		erode(result2, result2, square);

		absdiff(result2, result, result);

		applyThreshold(result);
		return result;
	}


	Mat getEdges(const Mat &image)
	{
		Mat result, result2;
		morphologyEx(image, result, MORPH_GRADIENT, Mat());
		applyThreshold(result);
		return result;
	}
} morph;

static void dump_mat(Mat m)
{
	char line[1000];
	int type = m.type();
	for (int y = 0; y < m.rows; y++) {
		int idx = 0;
		for (int x = 0; x < m.cols; x++)
		if (type == CV_32FC1)
			idx += sprintf(line + idx, "%3f,", m.at<float>(y, x));
		else
		if (type == CV_8UC1)
			idx += sprintf(line + idx, "%3d,", (int)m.at<uchar>(y, x));
		else
		if (type == CV_16SC1)
			idx += sprintf(line + idx, "%3d,", (int)m.at<short>(y, x));
		sprintf(line + idx, "\n");
		qInfo(line);
	}
}

static bool greaterPoint3i(const Point3i & a, const Point3i & b) { return a.z > b.z; }
static bool lessPoint3i(const Point3i & a, const Point3i & b) { return a.z < b.z; }

#define MORE_EIG_NUMBER  30

/*
inout vs
Input v
input mindistance
if v is at least mindistance away from all points in vs, push v to vs
Return 0 if fail to push
*/
static int push_point(vector<Point3i> & vs, Point3i v, double mindistance, bool checkonly = false)
{
	for (auto & o : vs) {
		int dx = v.x - o.x;
		int dy = v.y - o.y;
		if (dx*dx + dy*dy <= mindistance)
			return 0;
	}
	if (checkonly)
		return 1;
	vs.push_back(v);
	return 1;
}


/*
inout vs
input remain_number
input d1
input d2
reduce vs.z if vs.x and vs.y is within range d1, d2. then resort, this is to prevent
same line occupy too much weight.
*/
static void resort_points(vector<Point3i> & vs, int remain_number, float d1, float d2)
{
	for (int i = 1; i<(int)vs.size(); i++) {
		double reduce = 1;
		for (int j = 0; j < i; j++) {
			int dx = abs(vs[i].x - vs[j].x);
			int dy = abs(vs[i].y - vs[j].y);
			if (dx <= d1) { //reduce vs[i].z
				double w = (dx == d1) ? 0.9 : 0.85;
				float r = dy / d2;
				r = (r > 1) ? 1 : (1 - w) * r + w;
				reduce = reduce * r;
			}
			if (dy <= d1) { //reduce vs[i].z
				double w = (dy == d1) ? 0.9 : 0.85;
				float r = dx / d2;
				r = (r > 1) ? 1 : (1 - w) * r + w;
				reduce = reduce * r;
			}
		}
		vs[i].z *= reduce;
	}
	sort(vs.begin(), vs.end(), greaterPoint3i);
	if (vs.size() > remain_number)
		vs.erase(vs.begin() + remain_number, vs.end());
}

template<typename T> struct greaterThanPtr
{
	bool operator()(const T* a, const T* b) const { return *a > *b; }
};

static void calc_eigenval(const Mat& _cov, Mat& _dst, Mat& _dst1, double th)
{
	int i, j;
	Size size = _cov.size();

	if (_cov.isContinuous() && _dst.isContinuous() && _dst1.isContinuous())
	{
		size.width *= size.height;
		size.height = 1;
	}

	for (i = 0; i < size.height; i++)
	{
		const float* cov = (const float*)(_cov.data + _cov.step*i);
		float* dst = (float*)(_dst.data + _dst.step*i);
		float* dst1 = (float*)(_dst1.data + _dst1.step*i);
		j = 0;
		for (; j < size.width; j++)
		{
			float a = cov[j * 3] * 0.5f;
			float b = cov[j * 3 + 1];
			float c = cov[j * 3 + 2] * 0.5f;
			if (a * 2 < th && c * 2 < th) {
				dst[j] = 0;
				dst1[j] = 0;
			}
			else {
				float d = std::sqrt((a - c)*(a - c) + b*b);
				float x0 = a + c - d; //dst[j] = a + c - d;
				float x1 = a + c + d;
				dst1[j] = x1;
				dst[j] = x0 * x0 / x1;
			}
		}
	}
}

static void corner_eigen_valsvecs(const Mat& src, Mat& eigenv0, Mat & eigenv1, Mat & cov, int block_size, int aperture_size, double th, int borderType = BORDER_DEFAULT)
{
	double scale = (double)(1 << ((aperture_size > 0 ? aperture_size : 3) - 1)) * sqrt(10);// *sqrt((double)(block_size + 1) / 2);
	if (aperture_size < 0)
		scale *= 2.;
	//if (depth == CV_8U)
	//scale *= 255.;
	scale = 1. / scale;

	CV_Assert(src.type() == CV_8UC1 || src.type() == CV_32FC1);

	Mat Dx, Dy;
	if (aperture_size > 0)
	{
		Mat kx, ky;
		Sobel(src, Dx, CV_32F, 1, 0, aperture_size, scale, 0, borderType);
		Sobel(src, Dy, CV_32F, 0, 1, aperture_size, scale, 0, borderType);
	}
	else
	{
		Scharr(src, Dx, CV_32F, 1, 0, scale, 0, borderType);
		Scharr(src, Dy, CV_32F, 0, 1, scale, 0, borderType);
	}

	Size size = src.size();
	cov.create(size, CV_32FC3);
	int i, j;

	for (i = 0; i < size.height; i++)
	{
		float* cov_data = (float*)(cov.data + i*cov.step);
		const float* dxdata = (const float*)(Dx.data + i*Dx.step);
		const float* dydata = (const float*)(Dy.data + i*Dy.step);

		for (j = 0; j < size.width; j++)
		{
			float dx = dxdata[j];
			float dy = dydata[j];

			cov_data[j * 3] = dx*dx;
			cov_data[j * 3 + 1] = dx*dy;
			cov_data[j * 3 + 2] = dy*dy;
		}
	}

	boxFilter(cov, cov, cov.depth(), Size(block_size, block_size),
		Point(-1, -1), false, borderType);

	calc_eigenval(cov, eigenv0, eigenv1, th);
}

static void corner_eigenval(InputArray _src, OutputArray _dst, Mat & dst1, Mat & cov, int blockSize, int ksize, double th, int borderType)
{
	Mat src = _src.getMat();
	_dst.create(src.size(), CV_32F);
	dst1.create(src.size(), CV_32F);
	Mat dst = _dst.getMat();
	corner_eigen_valsvecs(src, dst, dst1, cov, blockSize, ksize, th, borderType);
}

static void find_good_corner(Mat & eig, vector<const float*> & pcorners, double th, int blocksize, bool need_dilate = true)
{
	Mat tmp;
	int bs = blocksize;
	if (need_dilate)
		dilate(eig, tmp, Mat());

	Size imgsize = eig.size();

	// collect list of pointers to features - put them into vector
	for (int y = bs; y < imgsize.height - bs; y++)
	{
		const float* eig_data = (const float*)eig.ptr(y);
		const float* tmp_data = need_dilate ? (const float*)tmp.ptr(y) : eig_data;

		for (int x = bs; x < imgsize.width - bs; x++)
		{
			float val = eig_data[x];
			if (val > th && (val == tmp_data[x] || y == bs || x == bs || y + 1 == imgsize.height - bs || x + 1 == imgsize.width - bs))
				pcorners.push_back(eig_data + x);
		}
	}

	sort(pcorners, greaterThanPtr<float>());
}

/*
Compute integrate and line integral
in img
in compute_line_integral, compute lg, llg
out ig, same as openCV integral
out iig, sum(sum(img(i,j)*img(i,j), j=0..x-1) i=0..y-1
out lg, sum(img(i,j), j=0..x-1)
out llg, sum(img(i,j)*img(i,j), j=0..x-1)
*/
static void integral_square(const Mat & img, Mat & ig, Mat & iig, Mat & lg, Mat & llg, bool compute_line_integral)
{
	CV_Assert(img.type() == CV_8UC1);
	ig.create(img.rows + 1, img.cols + 1, CV_32SC1);
	iig.create(img.rows + 1, img.cols + 1, CV_32SC1);
	if (compute_line_integral) {
		lg.create(img.rows, img.cols + 1, CV_32SC1);
		llg.create(img.rows, img.cols + 1, CV_32SC1);
	}
	for (int y = 0; y < ig.rows; y++) {
		unsigned * p_iig = iig.ptr<unsigned>(y);
		unsigned * p_ig = ig.ptr<unsigned>(y);
		if (y == 0)
		for (int x = 0; x < ig.cols; x++) {
			p_ig[x] = 0;
			p_iig[x] = 0;
		}
		else {
			unsigned * p_iig_1 = iig.ptr<unsigned>(y - 1);
			unsigned * p_ig_1 = ig.ptr<unsigned>(y - 1);
			const unsigned char * p_img = img.ptr<const unsigned char>(y - 1);
			unsigned * p_lg = compute_line_integral ? lg.ptr<unsigned>(y - 1) : NULL;
			unsigned * p_llg = compute_line_integral ? llg.ptr<unsigned>(y - 1) : NULL;
			p_ig[0] = 0;
			p_iig[0] = 0;
			if (compute_line_integral) {
				p_lg[0] = 0;
				p_llg[0] = 0;
			}
			unsigned lsum = 0, lsum2 = 0;
			for (int x = 0; x < img.cols; x++) {
				unsigned img2 = p_img[x];
				lsum += img2;
				lsum2 += img2 * img2;
				p_ig[x + 1] = p_ig_1[x + 1] + lsum;
				p_iig[x + 1] = p_iig_1[x + 1] + lsum2;
				if (compute_line_integral) {
					p_lg[x + 1] = lsum;
					p_llg[x + 1] = lsum2;
				}
			}
		}
	}
}

static void my_corner_eigenval(const Mat & image, Mat & eig0, Mat & eig1, int blockSize, double & quality, int ox, int oy)
{
	CV_Assert(image.type() == CV_8UC1 && blockSize % 2 == 1);
	if (ox < 0)
		ox = image.cols / 2;
	if (oy < 0)
		oy = image.rows / 2;
	Mat eig2, shape;
	eig0.create(image.size(), CV_32FC1);
	eig1.create(image.size(), CV_32FC1);
	eig2.create(image.size(), CV_32FC1);
	shape.create(image.size(), CV_8UC1);
	eig0 = Scalar::all(0);
	eig1 = Scalar::all(0);
	eig2 = Scalar::all(0);
	shape = Scalar::all(100);

	Mat ig, iig;
	integral_square(image, ig, iig, Mat(), Mat(), false);

	vector<int> offset; //offset
	vector<int> b; //eighth belong
	int bs2 = blockSize / 2;
	double area = 1.0 / (blockSize * blockSize);
	float a0 = 0, a1 = 0, a2 = 0, a3 = 0;

	//prepare offset and eighth belong
	for (int y = -bs2; y <= bs2; y++)
	for (int x = -bs2; x <= bs2; x++) {
		offset.push_back((y * (int)image.step.p[0] + x * (int)image.step.p[1]) / sizeof(uchar));
		int _b = 0;
		if (y < x && y < -x)
			a1 += 1;
		else
		if (y <= x && y == -x)
			a1 += 0.25;
		else
		if (y == x && y < -x)
			a1 += 0.25;

		if (y < 0 && y < -x)
			a2 += 1;
		else
		if (y == 0 && x <= 0)
			a2 += 0.25;
		else
		if (y < 0 && y == -x)
			a2 += 0.25;

		if (x > 0)
			a3 += 1;
		else
		if (x == 0)
			a3 += 0.25;

		if (x == 0) {
			if (y < 0)
				_b = 1 << 5 | 8;
			else
			if (y > 0)
				_b = 4 << 5 | 5;
			else
				_b = 0;
		}
		else
		if (y == 0)
			_b = (x < 0) ? 6 << 5 | 7 : 2 << 5 | 3;
		else
		if (y == x)
			_b = (x < 0) ? 7 << 5 | 8 : 3 << 5 | 4;
		else
		if (y == -x)
			_b = (x < 0) ? 5 << 5 | 6 : 1 << 5 | 2;
		else
		if (x > 0) {
			if (y < 0)
				_b = (y < -x) ? 1 : 2;
			else
				_b = (y < x) ? 3 : 4;
		}
		else {
			if (y < 0)
				_b = (y < x) ? 8 : 7;
			else
				_b = (y < -x) ? 6 : 5;
		}
		if (_b == 1)
			a0 += 1;
		if ((_b >> 5) == 1)
			a0 += 0.25;
		b.push_back(_b);
	}

	CV_Assert(offset.size() == blockSize * blockSize);
	int offset_size = (int)offset.size();
	vector<int> stat[4];
	int total[4] = { 0 };
	for (int i = 0; i < 4; i++)
		stat[i].resize(1024, 0);
	int border = bs2 + 1;
	//compute var stat
	for (int y = border; y < image.rows - border; y++) {
		int * pig = ig.ptr<int>(y - bs2);
		int * pig1 = ig.ptr<int>(y - bs2 + blockSize);
		int * piig = iig.ptr<int>(y - bs2);
		int * piig1 = iig.ptr<int>(y - bs2 + blockSize);
		float * peig1 = eig1.ptr<float>(y);
		float * peig2 = eig2.ptr<float>(y);
		for (int x = border; x < image.cols - border; x++) {
			if (y >= oy && y < image.rows - oy && x >= ox && x < image.cols - ox)
				x = image.cols - ox;
			int s = pig[x - bs2] + pig1[x - bs2 + blockSize] - pig1[x - bs2] - pig[x - bs2 + blockSize];
			int ss = piig[x - bs2] + piig1[x - bs2 + blockSize] - piig1[x - bs2] - piig[x - bs2 + blockSize];
			float var = ss * area - s * s * area * area; //compute variance
			peig2[x] = s;
			peig1[x] = var;
			CV_Assert(var < 16384);
			int idx = (int)var / 16;
			if (y < oy) {
				stat[DIR_UP][idx]++;
				total[DIR_UP]++;
			}
			if (y >= image.rows - oy) {
				stat[DIR_DOWN][idx]++;
				total[DIR_DOWN]++;
			}
			if (x < ox) {
				stat[DIR_LEFT][idx]++;
				total[DIR_LEFT]++;
			}
			if (x >= image.cols - ox) {
				stat[DIR_RIGHT][idx]++;
				total[DIR_RIGHT]++;
			}
		}
	}

	float th[4] = { 16, 16, 16, 16 }; //var threshold for corner UP, DOWN, LEFT RIGHT
	for (int j = 0; j < 4; j++) {
		int agg = 0;
		for (int i = 1023; i > 1; i--) {
			agg += stat[j][i];
			if (agg >= total[j] * quality) {
				th[j] = i * 8;
				break;
			}
		}
	}
	quality = min(min(th[0], th[1]), min(th[2], th[3])) * 2;
	qInfo("th0=%4f, th1=%4f, th2=%4f, th3=%4f, quality=%4f", th[0], th[1], th[2], th[3], quality);
	vector<Point> corner_pts;
	for (int y = border; y < image.rows - border; y++) {
		float * peig1 = eig1.ptr<float>(y);
		float * peig2 = eig2.ptr<float>(y);
		const uchar * pi = image.ptr<uchar>(y);
		uchar * pshape = shape.ptr<uchar>(y);
		int cor[31];
		for (int x = border; x < image.cols - border; x++) {
			if (y >= oy && y < image.rows - oy && x >= ox && x < image.cols - ox)
				x = image.cols - ox;
			bool pass = (y < oy && peig1[x] > th[DIR_UP] || y >= image.rows - oy && peig1[x] > th[DIR_DOWN]
				|| x < ox && peig1[x] > th[DIR_LEFT] || x >= image.cols - ox && peig1[x] > th[DIR_RIGHT]);
			if (!pass) { //below corner pass
				peig2[x] = 0;
				continue;
			}

			uchar mi = 255, ma = 0;
			int s = peig2[x];
			int avg = s / offset_size;
			const uchar * p0 = pi + x;
			for (int i = 0; i < offset_size; i++) { //compute min, max avg
				uchar a = p0[offset[i]];
				mi = min(mi, a);
				ma = max(ma, a);
			}
			bool choose_min = (avg - mi < ma - avg); //choose min-adjust or max-adjust			
			int bs[9] = { 0 }; //store eight sum
			int ss = 0;
			//following compute xiang guan
			for (int i = 0; i < offset_size; i++) {
				int a = choose_min ? p0[offset[i]] - mi : ma - p0[offset[i]];
				if (b[i] >= 32) {
					bs[b[i] >> 5] += a / 2;
					bs[b[i] & 31] += a / 2;
				}
				else
					bs[b[i]] += a;
				ss += a * a;
			}
			bs[0] = bs[0] / 2;
			for (int i = 0; i < 7; i++)
				cor[i] = bs[0] + bs[i + 1] + bs[i + 2]; //cor[0..7] for 1/4 corner
			cor[7] = bs[0] + bs[8] + bs[1];
			for (int i = 8; i < 14; i++)
				cor[i] = bs[0] + bs[i - 7] + bs[i - 6] + bs[i - 5]; //cor[8..15] for 3/8 corner
			cor[14] = bs[0] + bs[7] + bs[8] + bs[1];
			cor[15] = bs[0] + bs[8] + bs[1] + bs[2];
			for (int i = 16; i < 21; i++) // cor[16..23] for 1/2 corner
				cor[i] = bs[0] + bs[i - 15] + bs[i - 14] + bs[i - 13] + bs[i - 12];
			cor[21] = bs[0] + bs[6] + bs[7] + bs[8] + bs[1];
			cor[22] = bs[0] + bs[7] + bs[8] + bs[1] + bs[2];
			cor[23] = bs[0] + bs[8] + bs[1] + bs[2] + bs[3];
			float mc0 = -1, mc1 = -1, mc2 = -1, mc3 = -1, max_cor, max_cor0;
			int shape1, shape2, shape3;
			for (int i = 0; i < 8; i++)
				mc0 = max(mc0, (float)bs[i]);
			mc0 = mc0 * mc0 / (a0 * ss);
			for (int i = 0; i < 8; i++)
			if (mc1 < cor[i]) {
				mc1 = cor[i];
				shape1 = i;
			}
			mc1 = mc1 * mc1 / (a1 * ss);
			for (int i = 8; i < 16; i++)
			if (mc2 < cor[i]) {
				mc2 = cor[i];
				shape2 = i;
			}
			mc2 = mc2 * mc2 / (a2 * ss);
			for (int i = 16; i < 24; i++)
			if (mc3 < cor[i]) {
				mc3 = cor[i];
				shape3 = i;
			}
			mc3 = mc3 * mc3 / (a3 * ss);
			max_cor = max(mc1, mc2);
			max_cor0 = max(mc3, mc0);
			CV_Assert(max_cor <= 1);
			if (max_cor > 0.6667 && max_cor > max_cor0) {
				peig2[x] = max_cor;
				corner_pts.push_back(Point(x, y));
				shape2 = (shape2 >= 12) ? shape2 - 4 : shape2;
				pshape[x] = mc1 > mc2 ? shape1 : shape2;
			}
			else {
				peig2[x] = max_cor0;
				pshape[x] = (shape3 >= 20) ? shape3 - 4 : shape3;
			}
		}
	}

	for (auto & c : corner_pts) {
		float * peig2 = eig2.ptr<float>(c.y, c.x);
		uchar * pshape = shape.ptr<uchar>(c.y, c.x);
		uchar * pshape1 = shape.ptr<uchar>(c.y + 1, c.x);
		uchar * pshape_1 = shape.ptr<uchar>(c.y - 1, c.x);
		bool in_void = (pshape[1] == 100 && pshape[-1] == 100) || (pshape1[0] == 100 && pshape_1[0] == 100)
			|| (pshape1[1] == 100 && pshape_1[-1] == 100) || (pshape1[-1] == 100 && pshape_1[1] == 100);
		if (!in_void)
		if (peig2[0] > peig2[1] && peig2[0] > peig2[-1] &&
			peig2[0] > eig2.at<float>(c.y - 1, c.x) && peig2[0] > eig2.at<float>(c.y + 1, c.x))
			eig0.at<float>(c) = 3000 * (peig2[0] - 0.6667);
		else
		if (pshape[0] >= 8 && pshape[0] < 16) {
			bool check_fake = (pshape[0] == 8 && (pshape_1[0] == 16 && pshape1[0] == 16 || pshape_1[-1] == 19 && pshape1[1] == 19) ||
				pshape[0] == 9 && (pshape_1[1] == 17 && pshape1[-1] == 17 || pshape_1[0] == 16 && pshape1[0] == 16) ||
				pshape[0] == 10 && (pshape[1] == 18 && pshape[-1] == 18 || pshape_1[1] == 17 && pshape1[-1] == 17) ||
				pshape[0] == 11 && (pshape_1[-1] == 19 && pshape1[1] == 19 || pshape[1] == 18 && pshape[-1] == 18));

			if (!check_fake)
				eig0.at<float>(c) = 2500 * (peig2[0] - 0.6667);
		}
	}
}

struct CornerInfo {
	vector<Point3i> corners[4];  //used as good_features_to_track output para
	int max_number; //used as good_features_to_track input para
	double th;  //used as good_features_to_track input para
	double minDistance;  //used as good_features_to_track input para
};
static void good_features_to_track(Mat & img, CornerInfo * cinfo, int scale, int ox, int oy, int method, bool debug_en)
{
	CV_Assert(cinfo[1].minDistance >= cinfo[0].minDistance);
	int blockSize, ksize;
	Mat eig[2], cov;
	vector<const float*> tmpCorners[2];
	if (ox < 0)
		ox = img.cols / 4;
	if (oy < 0)
		oy = img.rows / 4;
	qInfo("good feature to track");
	ksize = (scale == 4) ? 3 : 5;
	medianBlur(img, img, ksize);
	Mat image;
	if (method == 0) {
		image = img;
		cinfo[0].th *= cinfo[0].th;
		cinfo[1].th *= cinfo[1].th;
		blockSize = (scale == 4) ? 5 : (scale == 2) ? 9 : 13;
		ksize = (scale == 4) ? 3 : (scale == 2) ? 5 : 7;
		corner_eigenval(image, eig[0], eig[1], cov, blockSize, ksize, cinfo[0].th, BORDER_DEFAULT);
		find_good_corner(eig[0], tmpCorners[0], cinfo[0].th, blockSize); //eig[0] stores corner
		find_good_corner(eig[1], tmpCorners[1], cinfo[1].th, blockSize); //eig[1] stores edge
	}
	else {
		blockSize = (scale == 4) ? 5 : 9;
		cinfo[1].th /= 100;
		if (scale == 1) {
			resize(img, image, Size(img.cols / 2, img.rows / 2));
			cinfo[0].minDistance /= 2;
			cinfo[1].minDistance /= 2;
			ox = ox / 2;
			oy = oy / 2;
		}
		else
			image = img;
		my_corner_eigenval(image, eig[0], eig[1], blockSize, cinfo[1].th, ox, oy);
		find_good_corner(eig[0], tmpCorners[0], cinfo[0].th, blockSize, false); //eig[0] stores corner
		find_good_corner(eig[1], tmpCorners[1], cinfo[1].th, blockSize, false); //eig[1] stores edge
	}

	for (int eig_idx = 0; eig_idx < 2; eig_idx++) {
		CornerInfo & c = cinfo[eig_idx];
		c.minDistance *= c.minDistance;
		for (int i = 0; i < (int)tmpCorners[eig_idx].size(); i++)
		{
			int ofs = (int)((const uchar*)tmpCorners[eig_idx][i] - eig[eig_idx].data);
			int y = (int)(ofs / eig[eig_idx].step);
			int x = (int)((ofs - y * eig[eig_idx].step) / sizeof(float));
			//push to different corner set
			Point3i v(x, y, eig[eig_idx].at<float>(y, x) + 0.5f);
			if (y < oy && c.corners[DIR_UP].size() < c.max_number * MORE_EIG_NUMBER)
				push_point(c.corners[DIR_UP], v, c.minDistance);
			if (y > image.rows - oy && c.corners[DIR_DOWN].size() < c.max_number * MORE_EIG_NUMBER)
				push_point(c.corners[DIR_DOWN], v, c.minDistance);
			if (x < ox && c.corners[DIR_LEFT].size() < c.max_number * MORE_EIG_NUMBER)
				push_point(c.corners[DIR_LEFT], v, c.minDistance);
			if (x > image.cols - ox && c.corners[DIR_RIGHT].size() < c.max_number * MORE_EIG_NUMBER)
				push_point(c.corners[DIR_RIGHT], v, c.minDistance);
		}
		if (eig_idx == 0) {
			resort_points(cinfo[0].corners[DIR_UP], c.max_number, 4 / scale, image.cols / 2);
			resort_points(cinfo[0].corners[DIR_DOWN], c.max_number, 4 / scale, image.cols / 2);
			resort_points(cinfo[0].corners[DIR_LEFT], c.max_number, 4 / scale, image.rows / 2);
			resort_points(cinfo[0].corners[DIR_RIGHT], c.max_number, 4 / scale, image.rows / 2);
			for (int i = 0; i < 4; i++)
				cinfo[1].corners[i] = cinfo[0].corners[i];
			cinfo[1].max_number += cinfo[0].max_number;
		}
		else {
			for (int i = 0; i < 4; i++)
				cinfo[1].corners[i].erase(cinfo[1].corners[i].begin(), cinfo[1].corners[i].begin() + cinfo[0].corners[i].size());
			cinfo[1].max_number -= cinfo[0].max_number;
			resort_points(cinfo[1].corners[DIR_UP], c.max_number, 4 / scale, image.cols / 2);
			resort_points(cinfo[1].corners[DIR_DOWN], c.max_number, 4 / scale, image.cols / 2);
			resort_points(cinfo[1].corners[DIR_LEFT], c.max_number, 4 / scale, image.rows / 2);
			resort_points(cinfo[1].corners[DIR_RIGHT], c.max_number, 4 / scale, image.rows / 2);
		}
	}

	if (debug_en)
	for (int eig_idx = 0; eig_idx < 2; eig_idx++)
	for (int i = 0; i < 4; i++) {
		char line[1000];
		int idx = 0;
		for (int j = 0; j < min((int)cinfo[eig_idx].corners[i].size(), 5); j++) {
			int x = cinfo[eig_idx].corners[i][j].x;
			int y = cinfo[eig_idx].corners[i][j].y;
			idx += sprintf(line + idx, "%4d,%4d,%5.0f,%5.0f  ", y, x,
				eig[0].at<float>(y, x), eig[1].at<float>(y, x));
		}
		qInfo(line);
	}

	if (scale == 1)
	for (int eig_idx = 0; eig_idx < 2; eig_idx++) {
		for (int i = 0; i < 4; i++)
		for (auto & v : cinfo[eig_idx].corners[i]) {
			v.x = v.x * 2;
			v.y = v.y * 2;
		}
	}
}


static void feature_extract(Mat m, int scale, string filename, Mat & c, Mat & e)
{
	Mat img;
	resize(m, img, Size(m.cols / scale, m.rows / scale));
	/*qInfo("porcess %s", filename.c_str());
	c = morph.getCorners(img);
	imwrite(filename + "_c.jpg", c);*/
		
	CornerInfo cinfo[2];
	cinfo[0].max_number = 50;
	cinfo[0].minDistance = 8 / scale;
	cinfo[0].th = 100;
	cinfo[1].max_number = 30;
	cinfo[1].minDistance = 32 / scale;
	cinfo[1].th = 17;
	good_features_to_track(img, cinfo, scale, img.cols / 2, img.rows / 2, 1, true);
	Mat c2 = img;
	for (int eig_idx = 0; eig_idx < 2; eig_idx++) {
		uchar color = 255;
		for (int i = 0; i < 4; i++) {
			Mat tmp(cinfo[eig_idx].corners[i], true); //test vector to mat convert
			vector<Point3i> vs;
			vs = Mat_<Point3i>(tmp);
			for (auto & v : vs) {
				Point o(v.x, v.y);
				c2.at<uchar>(o + Point(-1, 0)) = color;
				c2.at<uchar>(o + Point(1, 0)) = color;
				c2.at<uchar>(o + Point(0, 1)) = color;
				c2.at<uchar>(o + Point(0, -1)) = color;
				if (eig_idx == 0)
					c2.at<uchar>(o + Point(0, 0)) = color;
			}
		}
	}
	imwrite(filename + "_c2.jpg", c2);

	/*e = morph.getEdges(img);
	imwrite(filename + "_e.jpg", e);*/
}

void test_read_img()
{
	Mat m, c, e;
	int s = 4;
	
	m = imread("C:/chenyu/data/A01/M_13_29.jpg", 0);
	feature_extract(m, s, "C:/chenyu/data/A01/M1_13_29", c, e);

	m = imread("C:/chenyu/data/A01/M_13_30.jpg", 0);
	feature_extract(m, s, "C:/chenyu/data/A01/M1_13_30", c, e);

	m = imread("C:/chenyu/data/A01/M_13_31.jpg", 0);
	feature_extract(m, s, "C:/chenyu/data/A01/M1_13_31", c, e);

	m = imread("C:/chenyu/data/A01/M_14_29.jpg", 0);
	feature_extract(m, s, "C:/chenyu/data/A01/M1_14_29", c, e);

	m = imread("C:/chenyu/data/A01/M_14_30.jpg", 0);
	feature_extract(m, s, "C:/chenyu/data/A01/M1_14_30", c, e);

	m = imread("C:/chenyu/data/A01/M_14_31.jpg", 0);
	feature_extract(m, s, "C:/chenyu/data/A01/M1_14_31", c, e);

	m = imread("C:/chenyu/data/A01/M_21_34.jpg", 0);
	feature_extract(m, s, "C:/chenyu/data/A01/M_21_34", c, e);

	m = imread("C:/chenyu/data/A01/M_22_34.jpg", 0);
	feature_extract(m, s, "C:/chenyu/data/A01/M_22_34", c, e);
}

int main(int argc, char** argv)
{
	qInstallMessageHandler(myMessageOutput);

    FeatExt feature;
    ConfigPara cpara;

    cpara.clip_l = 0;
    cpara.clip_r = 0;
    cpara.clip_u = 0;
    cpara.clip_d = 0;
	cpara.rescale = 4;
    /*cpara.max_lr_xshift = 200;
    cpara.max_lr_yshift = 100;
    cpara.max_ud_xshift = 100;
    cpara.max_ud_yshift = 120;
    cpara.img_path = "c:/chenyu/data/A01/M_";*/
	cpara.max_lr_xshift = 120;
	cpara.max_lr_yshift = 60;
	cpara.max_ud_xshift = 60;
	cpara.max_ud_yshift = 80;
	cpara.img_path = "c:/chenyu/data/A01/ST_";
    cpara.img_num_w = 2;
	cpara.img_num_h = 2;
    cpara.offset.create(cpara.img_num_h, cpara.img_num_w);
    for (int y = 0; y < cpara.img_num_h; y++) {
        for (int x = 0; x < cpara.img_num_w; x++) {
	//		cpara.offset(y, x)[1] = 1500 * x;
	//		cpara.offset(y, x)[0] = 1150 * y;
			cpara.offset(y, x)[1] = 1248 * x;
			cpara.offset(y, x)[0] = 948 * y;
        }
    }
	ExtractParam ep;
	ep.read_file("./tune.xml");
	TuningPara _tpara(ep, "Default");
	int start_y = 7, start_x = 2;
	//int start_y = 21, start_x = 34;
	//int start_y = 4, start_x = 22;
#if 1
	feature.set_cfg_para(cpara);
	feature.set_tune_para(_tpara);
	feature.generate_feature_diff(start_x - 1, start_y - 1, 1);
    feature.write_diff_file("diff.xml");
#else
	feature.read_diff_file("diff.xml");
#endif
    double minval, maxval;
    Point minloc, maxloc;
	int y0 = 0, x0 = 0, y1 = 0, x1 = 1;
	//int y0 = 0, x0 = 1, y1 = 0, x1 = 2;
	char img1_name[50], img2_name[50];
	sprintf(img1_name, "%d_%d.jpg", start_y + y0, start_x + x0);
	sprintf(img2_name, "%d_%d.jpg", start_y + y1, start_x + x1);
	minMaxLoc(feature.get_edge((y0 == y1) ? 1 : 0, y0, x0)->dif, &minval, &maxval, &minloc, &maxloc);
	minloc *= cpara.rescale;
	Mat img1 = imread(cpara.img_path + img1_name, 0);
	Mat img2 = imread(cpara.img_path + img2_name, 0);
    img1 = img1(Rect(cpara.clip_l, cpara.clip_u, img1.cols - cpara.clip_l - cpara.clip_r, img1.rows - cpara.clip_u - cpara.clip_d));
    img2 = img2(Rect(cpara.clip_l, cpara.clip_u, img2.cols - cpara.clip_l - cpara.clip_r, img2.rows - cpara.clip_u - cpara.clip_d));
	Point offset = feature.get_edge((y0 == y1) ? 1 : 0, y0, x0)->offset + minloc;
	qDebug("minloc(y=%d,x=%d), offset(y=%d,x=%d),minval=%d", minloc.y, minloc.x, offset.y, offset.x, minval);
	Mat left, right;
	if (y0 == y1) {
		left = img1(Rect(offset.x, max(offset.y, 0), img1.cols - offset.x, img1.rows - abs(offset.y)));
		right = img2(Rect(0, max(-offset.y, 0), img1.cols - offset.x, img1.rows - abs(offset.y)));
	}
	else {
		left = img1(Rect(max(offset.x, 0), offset.y, img1.cols - abs(offset.x), img1.rows - offset.y));
		right = img2(Rect(max(-offset.x, 0), 0, img1.cols - abs(offset.x), img1.rows - offset.y));
	}
    resize(left, left, Size(left.cols / cpara.rescale, left.rows / cpara.rescale));
    resize(right, right, Size(right.cols / cpara.rescale, right.rows / cpara.rescale));
	imshow(img1_name, left);
	imshow(img2_name, right);

	//y0 = 1, x0 = 1, y1 = 1, x1 = 2;
	y0 = 1, x0 = 0, y1 = 1, x1 = 1;	
	sprintf(img1_name, "%d_%d.jpg", start_y + y0, start_x + x0);
	sprintf(img2_name, "%d_%d.jpg", start_y + y1, start_x + x1);
	minMaxLoc(feature.get_edge((y0 == y1) ? 1 : 0, y0, x0)->dif, &minval, &maxval, &minloc, &maxloc);
	minloc *= cpara.rescale;
	img1 = imread(cpara.img_path + img1_name, 0);
    img2 = imread(cpara.img_path + img2_name, 0);
    img1 = img1(Rect(cpara.clip_l, cpara.clip_u, img1.cols - cpara.clip_l - cpara.clip_r, img1.rows - cpara.clip_u - cpara.clip_d));
    img2 = img2(Rect(cpara.clip_l, cpara.clip_u, img2.cols - cpara.clip_l - cpara.clip_r, img2.rows - cpara.clip_u - cpara.clip_d));
	offset = feature.get_edge((y0 == y1) ? 1 : 0, y0, x0)->offset + minloc;
	qDebug("minloc(y=%d,x=%d), offset(y=%d,x=%d),minval=%d", minloc.y, minloc.x, offset.y, offset.x, minval);
	Mat up, down;
	if (y0 == y1) {
		up = img1(Rect(offset.x, max(offset.y, 0), img1.cols - offset.x, img1.rows - abs(offset.y)));
		down = img2(Rect(0, max(-offset.y, 0), img1.cols - offset.x, img1.rows - abs(offset.y)));
	}
	else {
		up = img1(Rect(max(offset.x, 0), offset.y, img1.cols - abs(offset.x), img1.rows - offset.y));
		down = img2(Rect(max(-offset.x, 0), 0, img1.cols - abs(offset.x), img1.rows - offset.y));
	}
    resize(up, up, Size(up.cols / cpara.rescale, up.rows / cpara.rescale));
    resize(down, down, Size(down.cols / cpara.rescale, down.rows / cpara.rescale));

	imshow(img1_name, up);
	imshow(img2_name, down);
	qWarning("finished");
    waitKey();

    return 0;
}

/*
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    FeatureWindow w;
    w.show();

    return a.exec();
}
*/
