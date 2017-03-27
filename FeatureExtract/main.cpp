#include "featurewindow.h"
#include <QApplication>
#include "featext.h"
#include "bundleadjust.h"
#include "opencv2/highgui/highgui.hpp"
#include <QDateTime>

static FILE * fp = NULL;

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
		switch (type) {
		case QtDebugMsg:
			fprintf(fp, "<D>[%s] %s\n", qPrintable(str_dt), qPrintable(msg));
			break;
		case QtInfoMsg:
			fprintf(fp, "<I>[%s] %s\n", qPrintable(str_dt), qPrintable(msg));
			break;
		case QtWarningMsg:
			fprintf(fp, "<W>[%s] %s\n", qPrintable(str_dt), qPrintable(msg));
			break;
		case QtCriticalMsg:
			fprintf(fp, "<E>[%s] %s\n", qPrintable(str_dt), qPrintable(msg));
			fflush(fp);
			break;
		case QtFatalMsg:
			fprintf(fp, "<F>[%s] %s\n", qPrintable(str_dt), qPrintable(msg));
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
		break;
	case QtInfoMsg:
		fprintf(fp, "<I>[%d,%s] [%s] [%s] %s\n", context.line, file, qPrintable(str_dt), func, qPrintable(msg));
		break;
	case QtWarningMsg:
		fprintf(fp, "<W>[%d,%s] [%s] [%s] %s\n", context.line, file, qPrintable(str_dt), func, qPrintable(msg));
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

void test_bundleadjust()
{
    FeatExt feature;
    BundleAdjust ba;

    feature.read_diff_file("C:/chenyu/work/ChipPintu/app/diff.xml");
    ba.arrange(feature, 5, 5);
}

int main(int argc, char** argv)
{
	qInstallMessageHandler(myMessageOutput);
    test_bundleadjust();
    return 0;
    FeatExt feature;
    ConfigPara cpara;

    cpara.clip_l = 6;
    cpara.clip_r = 0;
    cpara.clip_u = 0;
    cpara.clip_d = 12;
    cpara.rescale = 8;
    cpara.max_lr_xshift = 80;
    cpara.max_lr_yshift = 16;
    cpara.max_ud_xshift = 16;
    cpara.max_ud_yshift = 80;
    cpara.img_path = "F:/chenyu/work/ChipStitch/data/m3/";
    cpara.img_num_w = 71;
	cpara.img_num_h = 101;
    cpara.offset.create(cpara.img_num_h, cpara.img_num_w);
    for (int y = 0; y < cpara.img_num_h; y++) {
        for (int x = 0; x < cpara.img_num_w; x++) {
            cpara.offset(y, x)[1] = 1780 * x;
            cpara.offset(y, x)[0] = 1572 * y;
        }
    }
#if 1
	feature.set_cfg_para(cpara);
    feature.generate_feature_diff();
    feature.write_diff_file("diff.xml");
#else
	feature.read_diff_file("diff.xml");
#endif
    double minval, maxval;
    Point minloc, maxloc;
	minMaxLoc(feature.get_edge(1, 1, 0)->diff, &minval, &maxval, &minloc, &maxloc);
	minloc *= cpara.rescale;
    Mat img1 = imread(cpara.img_path + "2_1.jpg", 0);
    Mat img2 = imread(cpara.img_path + "2_2.jpg", 0);
    img1 = img1(Rect(cpara.clip_l, cpara.clip_u, img1.cols - cpara.clip_l - cpara.clip_r, img1.rows - cpara.clip_u - cpara.clip_d));
    img2 = img2(Rect(cpara.clip_l, cpara.clip_u, img2.cols - cpara.clip_l - cpara.clip_r, img2.rows - cpara.clip_u - cpara.clip_d));
	Point offset = feature.get_edge(1, 1, 0)->offset + minloc;
	Mat left = img1(Rect(offset.x, max(offset.y, 0), img1.cols - offset.x, img1.rows - abs(offset.y)));
	Mat right = img2(Rect(0, max(-offset.y, 0), img1.cols - offset.x, img1.rows - abs(offset.y)));
    resize(left, left, Size(left.cols / cpara.rescale, left.rows / cpara.rescale));
    resize(right, right, Size(right.cols / cpara.rescale, right.rows / cpara.rescale));
    imshow("x0", left);
    imshow("x1", right);

	minMaxLoc(feature.get_edge(0, 2, 1)->diff, &minval, &maxval, &minloc, &maxloc);
	minloc *= cpara.rescale;
    img1 = imread(cpara.img_path + "3_2.jpg", 0);
    img2 = imread(cpara.img_path + "4_2.jpg", 0);
    img1 = img1(Rect(cpara.clip_l, cpara.clip_u, img1.cols - cpara.clip_l - cpara.clip_r, img1.rows - cpara.clip_u - cpara.clip_d));
    img2 = img2(Rect(cpara.clip_l, cpara.clip_u, img2.cols - cpara.clip_l - cpara.clip_r, img2.rows - cpara.clip_u - cpara.clip_d));
	offset = feature.get_edge(0, 2, 1)->offset + minloc;
    Mat up = img1(Rect(max(offset.x, 0), offset.y, img1.cols - abs(offset.x), img1.rows - offset.y));
    Mat down = img2(Rect(max(-offset.x, 0), 0, img1.cols - abs(offset.x), img1.rows - offset.y));
    resize(up, up, Size(up.cols / cpara.rescale, up.rows / cpara.rescale));
    resize(down, down, Size(down.cols / cpara.rescale, down.rows / cpara.rescale));
    imshow("y0", up);
    imshow("y1", down);
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
