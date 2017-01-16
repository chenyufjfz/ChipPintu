#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "mainwindow.h"
#include <QApplication>
#include <QDateTime>
#include "viawireeditview.h"
using namespace cv;
using namespace std;


void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static FILE * fp = NULL;
    if (fp==NULL) {
        fp = fopen("log.txt", "wt");
        if (fp==NULL)
            exit(-1);
    }

    QTime datetime;
    datetime = QTime::currentTime();
    QString str_dt = datetime.toString("hh:mm:ss.zzz");

    if (context.function==NULL) {
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
    int size, kuo=0;
    const char * pend, * pmid, * pbegin;
    size = (int) strlen(context.function);
    for (pend = context.function +size; pend!=context.function; pend--) {
        if (*pend==')')
            kuo++;
        if (*pend=='(') {
            kuo--;
            if (kuo<=0)
                break;
        }
    }
    if (pend==context.function)
        pend = context.function +size;

    for (pmid = pend; pmid!=context.function && *pmid!=':'; pmid--);
    if (*pmid==':')
        pmid++;
    size= pend- pmid;
    memcpy(func, pmid, size);
    func[size]=0;
    while (*(pmid-1)==':' && pmid!=context.function)
        pmid--;
    for (pbegin=pmid; *pbegin!=' ' && pbegin !=context.function; pbegin--);
    size = pmid -pbegin;
    memcpy(file, pbegin, size);
    file[size]=0;

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

void edge_mixer(const Mat & img_in, const Mat & mask, Mat & img_out)
{

    CV_Assert(img_in.type() == CV_16SC1 && mask.type() == CV_8UC1);
    Mat out(img_in.rows, img_in.cols, CV_8SC1);
    for (int y = 0; y < img_in.rows; y++) {
        const unsigned short * p_img_in = img_in.ptr<unsigned short>(y);
        const unsigned char * p_mask = mask.ptr<unsigned char>(y);
        unsigned char * p_img_out = out.ptr<unsigned char>(y);
        for (int x = 0; x < img_in.cols; x++)
        if (p_mask[x])
            p_img_out[x] = p_img_in[x] >> 4;
        else
            p_img_out[x] = 0;

    }
    img_out = out;
}


void prepare_grad(const Mat & img_in, Mat & grad_x, Mat & grad_y, Mat & edge_mask, int rescale = 4, int bfilt_w = 3, 
	int bfilt_csigma = 20, int canny_high_th=150, int canny_low_th=50, int sobel_w=3)
{
    Mat filt_mat;

    bilateralFilter(img_in, filt_mat, bfilt_w, bfilt_csigma, 0);
	if (rescale!=1)
		resize(filt_mat, filt_mat, Size(filt_mat.cols / rescale, filt_mat.rows / rescale));
    Canny(filt_mat, edge_mask, canny_low_th, canny_high_th, 3);
    dilate(edge_mask, edge_mask, Mat());

    Sobel(filt_mat, grad_x, CV_16S, 1, 0, sobel_w);
    edge_mixer(grad_x, edge_mask, grad_x);

    Sobel(filt_mat, grad_y, CV_16S, 0, 1, sobel_w);
    edge_mixer(grad_y, edge_mask, grad_y);
}

int cell_extract_test()
{	
	vector <MarkObj> obj_set; 

#if 0
	ViaWireEditView::load_objects("C:/chenyu/work/ChipPintu/images/Project_21_45.xml", obj_set, wire_width, via_radius);
	VWExtractStat vwe;
	vwe.set_train_param(wire_width, via_radius, 10, 0.1, 0, 0.2, 6, 16);
	vwe.train("C:/chenyu/work/ChipPintu/images/Project_21_45.jpg", obj_set);
	double t0 = getTickCount();
	vwe.extract("C:/chenyu/work/ChipPintu/images/Project_21_45.jpg", QRect(50, 50, 550, 250), obj_set);
	double t1 = getTickCount() - t0;
	qDebug("t1=%f, t2=%f", t1, 0);
#else
	CellExtract ce;
	ICLayerWr ic("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M1.dat", true);
    vector<ICLayerWr *> pic;
    pic.push_back(&ic);
	ce.set_train_param(0, 0, 0, 0, 0, 0,  0.1, 3, 0.5, 0);
	MarkObj obj;
	vector<MarkObj> objs;
	obj.type = OBJ_AREA;
	obj.type2 = AREA_CELL;
	obj.type3 = POWER_UP_L;
	obj.p0 = QPoint(295936, 236864);
	obj.p1 = QPoint(304768, 241088);
	obj.state = 0;
	objs.push_back(obj);
    ce.train(pic, objs);
	QRect r(0, 0, 327680, 327680);
	vector<SearchArea> search;
	search.push_back(SearchArea(r, POWER_UP | POWER_DOWN));
    ce.extract(pic, search, objs);
#endif
    return 0;
}

int wire_extract_test()
{
	vector <MarkObj> objs;
	ICLayerWr ic[4]; 
	ic[0].create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M1.dat", true);
	ic[1].create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M2.dat", true);
	ic[2].create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M3.dat", true);
	ic[3].create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M4.dat", true);
    vector<ICLayerWr *> pic;
    pic.push_back(&ic[0]);
    pic.push_back(&ic[1]);
    pic.push_back(&ic[2]);
	pic.push_back(&ic[3]);

	VWExtract * vwe = VWExtract::create_extract(0);
	vwe->set_extract_param(0, 4, 9, RULE_END_WITH_VIA, 0, 16, 0.5, 0.5, 2, 0);
	vwe->set_extract_param(1, 10, 9, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP, 
		RULE_NO_hCONN, 16, 0.5, 0.5, 2, 0);
	vwe->set_extract_param(2, 12, 10, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP, 
		RULE_NO_hCONN, 16, 0.5, 0.5, 2, 0);
	vwe->set_extract_param(3, 12, 10, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP, 
		RULE_NO_hCONN, 16, 0.5, 0.5, 1, 0);
	vector<SearchArea> search;  
	search.push_back(SearchArea(QRect(QPoint(1427951, 623749), QPoint(1653743, 777349)), 0));
    vwe->extract(pic, search, objs);
	return 0;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
	/*
	Mat src1 = imread("F:/chenyu/work/ChipStitch/data/chip2/M2/Project_9_48.jpg", 0);
	Mat grad_x0, grad_y0, grad_x1, grad_y1, diff, edge;
	prepare_grad(src1, grad_x1, grad_y1, edge, 1);

	imshow("edge", edge);
	imshow("grad_x", grad_x1);
	imshow("grad_y", grad_y1);
	waitKey();*/

	//cell_extract_test();
	qInstallMessageHandler(myMessageOutput);
#if 1
	wire_extract_test();
	return 0;
#endif
    w.show();

    return a.exec();
}

