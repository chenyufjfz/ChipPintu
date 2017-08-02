#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "mainwindow.h"
#include <QApplication>
#include <QDateTime>
#include "viawireeditview.h"
#include "extractparam.h"
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
			fflush(fp);
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

	CellExtract ce;
	ICLayerWrInterface * ic = ICLayerWrInterface::create("C:/chenyu/data/hanzhou/M1/M1.db", true);
	
    vector<ICLayerWrInterface *> pic;
    pic.push_back(ic);
	ce.set_train_param(0, 0, 0, 0, 0, 0,  30, 300, 50, 0);
	MarkObj obj;
	vector<MarkObj> objs;
	obj.type = OBJ_AREA;
	obj.type2 = AREA_CELL;
	obj.type3 = POWER_UP_L;
	obj.p0 = QPoint(1520896,234899);
	obj.p1 = QPoint(1534975,239059);
	obj.state = 0;
	objs.push_back(obj);
    ce.train(pic, objs);
	vector<SearchArea> search;
	search.push_back(SearchArea(QRect(QPoint(1463296, 229374), QPoint(1654911, 234621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 234494), QPoint(1654911, 239741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 239614), QPoint(1654911, 244861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 244734), QPoint(1654911, 249981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 249854), QPoint(1654911, 255101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 254974), QPoint(1654911, 260221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 260094), QPoint(1654911, 265341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 265214), QPoint(1654911, 270461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 270334), QPoint(1654911, 275581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 275454), QPoint(1654911, 280701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 280574), QPoint(1654911, 285821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 285694), QPoint(1654911, 290941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 290814), QPoint(1654911, 296061)), POWER_UP | POWER_DOWN));
    ce.extract(pic, search, objs);

    return 0;
}

int wire_extract_test()
{
	vector <MarkObj> objs;
	ICLayerWrInterface * ic[4]; 
	ic[0] = ICLayerWrInterface::create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M1.db", true);
	ic[1] = ICLayerWrInterface::create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M2.db", true);
	ic[2] = ICLayerWrInterface::create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M3.db", true);
	ic[3] = ICLayerWrInterface::create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M4.db", true);
    vector<ICLayerWrInterface *> pic;
    pic.push_back(ic[0]);
    pic.push_back(ic[1]);
    pic.push_back(ic[2]);
	pic.push_back(ic[3]);

	VWExtract * vwe = VWExtract::create_extract(0);
	vwe->set_extract_param(0, 0, 0x1000409, RULE_END_WITH_VIA, 0, 16, 50, 50, 200, 0);
	vwe->set_extract_param(1, 10, 9, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		RULE_NO_hCONN | RULE_VIA_NO_LCONN, 16, 50, 50, 200, 0);
	vwe->set_extract_param(2, 12, 10, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		RULE_NO_hCONN | RULE_VIA_NO_LCONN, 16, 50, 50, 200, 0);
	vwe->set_extract_param(3, 12, 10, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		RULE_NO_hCONN | RULE_VIA_NO_LCONN, 16, 50, 50, 100, 0);
	vector<SearchArea> search; 
	search.push_back(SearchArea(QRect(QPoint(1427671, 674029), QPoint(1657047, 828653)), 0));
    vwe->extract(pic, search, objs);
	return 0;
}

int wire_extract_test_pipeprocess()
{
	vector <MarkObj> objs;
	VWExtract * vwe = VWExtract::create_extract(0);
	//layer, type, opt0, opt1, opt2
	vwe->set_extract_param(0, 0x80020000, 64, 10, 3, 5*64, 0, 0, 0, 0); //filter min
	vwe->set_extract_param(0, 0x80030000, 0x0a0c4000, 0x08083810, 0x0120b030, 0x00646432, 0x00646432, 0x00646432, 0, 0); //adjust_gray_lvl
	vwe->set_extract_param(1, 0x80020000, 64, 10, 3, 5 * 64, 0, 0, 0, 0); //filter min
	vwe->set_extract_param(1, 0x80030000, 0x0a0c4000, 0x080a3810, 0x0120b030, 0x00646432, 0x00646432, 0x00646416, 0, 0); //adjust_gray_lvl
	vwe->set_extract_param(-1, 0x80000000, 0x00402006, 0, 0, 0, 0, 0, 0, 0); //set compute border and gs
	vwe->set_extract_param(1, 0x80000000, 0x00000105, 0x06, 0x0a060a06, 0x0606, 0, 0, 0, 0);	//set M1 wire
	vwe->set_extract_param(1, 0x80000000, 0x00000106, 0x06, 0x0a060a06, 0x0606, 0, 0, 0, 0);	//set M1 wire
	vwe->set_extract_param(1, 0x80000000, 0x000101fe, 0x0c09, 0x0a0a0905, 0x010708, 0, 0, 0, 0); //set M1 via
	vwe->set_extract_param(1, 0x80050001, 0x1, 0x0a0c01, 0, 0, 0, 0, 0, 0); //coarse_via_search
	vwe->set_extract_param(1, 0x80040030, 0x10102020, 0x00101002, 0x0001, 0x0101, 0, 0, 0, 0); //coarse line search
	vwe->set_extract_param(1, 0x80063210, 0x1, 0x0101, 0, 0, 0, 0, 0, 0); //fine_via_search
	vwe->set_extract_param(1, 0x80070002, 0x00001001, 0x0101, 0, 0, 0, 0, 0, 0); //remove_via
	vwe->set_extract_param(1, 0x80040030, 0x10102020, 0x01101002, 0x0001, 0x0101, 0, 0, 0, 0); //coarse line search
	vwe->set_extract_param(1, 0x80080004, 0x0102, 0x02120105, 0x1000, 0x02120106, 0x1000, 0, 0, 0); //coarse line mask
	vwe->set_extract_param(1, 0x80090040, 0x1002, 0x000105, 0x000106, 0, 0, 0, 0, 0); //fine line search
	vwe->set_extract_param(1, 0x800a0000, 0x01, 0x20200301, 0, 0, 0, 0, 0, 0); //assemble line
	vwe->extract("C:/chenyu/work/ChipPintu/images/Project_21_51_M0.jpg", QRect(0, 0, 2000, 2000), objs);
	return 0;
}

int test_extractparam()
{
	ExtractParam ep;
	vector<string> layer0, layer1, global;
	vector<string> action;

	ep.read_file("action.xml");

	vector<ParamItem> params;
	ep.get_param("action", params);
	vector <MarkObj> objs;
	VWExtract * vwe = VWExtract::create_extract(0);

	for (int i = 0; i < params.size(); i++) {
		qInfo("params %x, %x, %x, %x, %x, %x, %x, %x, %x, %f", params[i].pi[0], params[i].pi[1], params[i].pi[2],
			params[i].pi[3], params[i].pi[4], params[i].pi[5], params[i].pi[6], params[i].pi[7], params[i].pi[8], params[i].pf);
		vwe->set_extract_param(params[i].pi[0], params[i].pi[1], params[i].pi[2], params[i].pi[3], params[i].pi[4],
			params[i].pi[5], params[i].pi[6], params[i].pi[7], params[i].pi[8], params[i].pf);
	}
	vwe->extract("C:/chenyu/work/ChipPintu/images/Project_21_51_M0.jpg", QRect(0, 0, 2000, 2000), objs);
	
	return 0;
}
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
	
	qInstallMessageHandler(myMessageOutput);
#if 0
	
	//wire_extract_test_pipeprocess();
	//wire_extract_test();
	cell_extract_test();
	return 0;
#endif
    w.show();

    int ret = a.exec();
	qWarning("code exit with ret=%d", ret);
	return ret;
}

