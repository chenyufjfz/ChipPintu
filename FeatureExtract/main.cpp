#include "featurewindow.h"
#include <QApplication>
#include "featext.h"
#include "bundleadjust.h"
#include "opencv2/highgui/highgui.hpp"
#include <QDateTime>
#include <QThread>

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

    feature.read_diff_file("C:/chenyu/work/ChipPintu/app/diff.xml");
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
    cpara.max_lr_xshift = 40;
    cpara.max_lr_yshift = 16;
    cpara.max_ud_xshift = 16;
    cpara.max_ud_yshift = 40;
    cpara.img_path = "C:/chenyu/data/A01/M1/M1_";
    cpara.img_num_w = 3;
	cpara.img_num_h = 3;
    cpara.offset.create(cpara.img_num_h, cpara.img_num_w);
    for (int y = 0; y < cpara.img_num_h; y++) {
        for (int x = 0; x < cpara.img_num_w; x++) {
			cpara.offset(y, x)[1] = 1817 * x;
			cpara.offset(y, x)[0] = 1558 * y;
        }
    }
	ExtractParam ep;
	ep.read_file("./tune.xml");
	TuningPara _tpara(ep, "RawGray");
#if 1
	feature.set_cfg_para(cpara);
	feature.set_tune_para(_tpara);
	feature.generate_feature_diff(0, 0, 1);
    feature.write_diff_file("diff.xml");
#else
	feature.read_diff_file("diff.xml");
#endif
    double minval, maxval;
    Point minloc, maxloc;
	int y0 = 1, x0 = 1, y1 = 1, x1 = 2;
	char img1_name[50], img2_name[50];
	sprintf(img1_name, "%d_%d.jpg", y0, x0);
	sprintf(img2_name, "%d_%d.jpg", y1, x1);
	minMaxLoc(feature.get_edge((y0 == y1) ? 1 : 0, y0 - 1, x0 - 1)->dif, &minval, &maxval, &minloc, &maxloc);
	minloc *= cpara.rescale;
	Mat img1 = imread(cpara.img_path + img1_name, 0);
	Mat img2 = imread(cpara.img_path + img2_name, 0);
    img1 = img1(Rect(cpara.clip_l, cpara.clip_u, img1.cols - cpara.clip_l - cpara.clip_r, img1.rows - cpara.clip_u - cpara.clip_d));
    img2 = img2(Rect(cpara.clip_l, cpara.clip_u, img2.cols - cpara.clip_l - cpara.clip_r, img2.rows - cpara.clip_u - cpara.clip_d));
	Point offset = feature.get_edge((y0 == y1) ? 1 : 0, y0 - 1, x0 - 1)->offset + minloc;
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
	
	y0 = 2, x0 = 2, y1 = 3, x1 = 2;
	sprintf(img1_name, "%d_%d.jpg", y0, x0);
	sprintf(img2_name, "%d_%d.jpg", y1, x1);
	minMaxLoc(feature.get_edge((y0 == y1) ? 1 : 0, y0 - 1, x0 - 1)->dif, &minval, &maxval, &minloc, &maxloc);
	minloc *= cpara.rescale;
	img1 = imread(cpara.img_path + img1_name, 0);
    img2 = imread(cpara.img_path + img2_name, 0);
    img1 = img1(Rect(cpara.clip_l, cpara.clip_u, img1.cols - cpara.clip_l - cpara.clip_r, img1.rows - cpara.clip_u - cpara.clip_d));
    img2 = img2(Rect(cpara.clip_l, cpara.clip_u, img2.cols - cpara.clip_l - cpara.clip_r, img2.rows - cpara.clip_u - cpara.clip_d));
	offset = feature.get_edge((y0 == y1) ? 1 : 0, y0 - 1, x0 - 1)->offset + minloc;
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
