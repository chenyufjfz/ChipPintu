#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "mainwindow.h"
#include <QApplication>
#include <QDateTime>
#include <set>
#include <QThread>
#include "viawireeditview.h"
#include "extractparam.h"
using namespace cv;
using namespace std;

#define QMSG_FLUSH 1

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

	if (msg == "*#*#DumpMessage#*#*") {
		fflush(fp);
		return;
	}
	
	if (context.function == NULL) {
		unsigned thread_id = quintptr(QThread::currentThreadId());
		switch (type) {
		case QtDebugMsg:
			fprintf(fp, "<D>[%s] [%d] %s\n", qPrintable(str_dt), thread_id & 0x7ffffff, qPrintable(msg));
#if QMSG_FLUSH
			fflush(fp);
#endif
			break;
		case QtInfoMsg:
			fprintf(fp, "<I>[%s] [%d] %s\n", qPrintable(str_dt), thread_id & 0x7ffffff, qPrintable(msg));
#if QMSG_FLUSH
			fflush(fp);
#endif
			break;
		case QtWarningMsg:
			fprintf(fp, "<W>[%s] [%d] %s\n", qPrintable(str_dt), thread_id & 0x7ffffff, qPrintable(msg));
			fflush(fp);
			break;
		case QtCriticalMsg:
			fprintf(fp, "<E>[%s] [%d] %s\n", qPrintable(str_dt), thread_id & 0x7ffffff, qPrintable(msg));
			fflush(fp);
			break;
		case QtFatalMsg:
			fprintf(fp, "<F>[%s] [%d] %s\n", qPrintable(str_dt), thread_id & 0x7ffffff, qPrintable(msg));
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
	ICLayerWrInterface * ic = ICLayerWrInterface::create("c:/chenyu/data/PL_1.db", "", true, 1.0, 1.0, 0, 0, 2, 0, 0);
	
    vector<ICLayerWrInterface *> pic;
    pic.push_back(ic);
	ce.set_train_param(0, 0, 0, 0, 0, 0, 40, 300, 50, 0);
	MarkObj obj;
	vector<MarkObj> objs;
	obj.type = OBJ_AREA;
	obj.type2 = AREA_CELL;
	obj.type3 = POWER_DOWN_L;
	obj.p0 = QPoint(392447, 1082756);
	obj.p1 = QPoint(396765, 1099065);
	obj.state = 0;
	objs.push_back(obj);
    ce.train(pic, objs);
	
	vector<SearchArea> search;
	search.push_back(SearchArea(QRect(QPoint(636528, 1251618), QPoint(641486, 1269207)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(648201, 1251618), QPoint(653159, 1269207)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(641384, 1251618), QPoint(646342, 1269207)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(726555, 1251618), QPoint(731513, 1269207)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(565084, 1217718), QPoint(570042, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(695036, 1217718), QPoint(699994, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401466, 1251618), QPoint(406424, 1269207)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(406205, 1251618), QPoint(411163, 1269207)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(411015, 1251618), QPoint(415973, 1269207)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(620138, 1217718), QPoint(625096, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(447530, 1217718), QPoint(452488, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(405948, 1217718), QPoint(410906, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(410781, 1217718), QPoint(415739, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(489042, 1217718), QPoint(494000, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(971003, 1251618), QPoint(975961, 1269207)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(521612, 1217718), QPoint(526570, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(461935, 1217718), QPoint(466893, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(452363, 1217718), QPoint(457321, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(415567, 1217718), QPoint(420525, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(420400, 1217718), QPoint(425358, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(503494, 1217718), QPoint(508452, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(493875, 1217718), QPoint(498833, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(498684, 1217718), QPoint(503642, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(457149, 1217718), QPoint(462107, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(526398, 1217718), QPoint(531356, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(971073, 1217718), QPoint(976031, 1235307)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(521798, 1183817), QPoint(526756, 1201406)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(550959, 1183817), QPoint(555917, 1201406)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(671362, 1183817), QPoint(676320, 1201406)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(396889, 1132967), QPoint(401847, 1150556)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(666576, 1183817), QPoint(671534, 1201406)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(396889, 1183817), QPoint(401847, 1201406)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(923304, 1149917), QPoint(928262, 1167506)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401746, 1183817), QPoint(406704, 1201406)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(794917, 1149917), QPoint(799875, 1167506)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(970980, 1183817), QPoint(975938, 1201406)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(487244, 1099066), QPoint(492202, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(428759, 1099066), QPoint(433717, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(971096, 1149917), QPoint(976054, 1167506)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(971026, 1116016), QPoint(975984, 1133605)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(406112, 1099066), QPoint(411070, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(411061, 1099066), QPoint(416019, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401372, 1099066), QPoint(406330, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401676, 1132967), QPoint(406634, 1150556)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(475500, 1149917), QPoint(480458, 1167506)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(545706, 1099066), QPoint(550664, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(435273, 1200767), QPoint(440231, 1218356)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(411061, 1234668), QPoint(416019, 1252257)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401722, 1200767), QPoint(406680, 1218356)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(970980, 1200767), QPoint(975938, 1218356)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(406205, 1234668), QPoint(411163, 1252257)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(524016, 1200767), QPoint(528974, 1218356)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401466, 1234668), QPoint(406424, 1252257)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(396936, 1200767), QPoint(401894, 1218356)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(970956, 1234668), QPoint(975914, 1252257)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(672202, 1234668), QPoint(677160, 1252257)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(453297, 1200767), QPoint(458255, 1218356)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(734610, 1234668), QPoint(739568, 1252257)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(552734, 1234668), QPoint(557692, 1252257)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(636458, 1234668), QPoint(641416, 1252257)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(446269, 1082116), QPoint(451227, 1099705)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401746, 1149917), QPoint(406704, 1167506)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(396843, 1149917), QPoint(401801, 1167506)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(504731, 1082116), QPoint(509689, 1099705)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(604028, 1099066), QPoint(608986, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(891458, 1166867), QPoint(896416, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(822653, 1166867), QPoint(827611, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(799236, 1132967), QPoint(804194, 1150556)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(944270, 1166867), QPoint(949228, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(473306, 1166867), QPoint(478264, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(552920, 1166867), QPoint(557878, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(478162, 1166867), QPoint(483120, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(920596, 1099066), QPoint(925554, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(862087, 1099066), QPoint(867045, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(971120, 1166867), QPoint(976078, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(907451, 1166867), QPoint(912409, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(810163, 1132967), QPoint(815121, 1150556)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(970886, 1132967), QPoint(975844, 1150556)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(396936, 1166867), QPoint(401894, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(912284, 1166867), QPoint(917242, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401722, 1166867), QPoint(406680, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(938667, 1099066), QPoint(943625, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(401652, 1116016), QPoint(406610, 1133605)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(970980, 1099066), QPoint(975938, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(845651, 1166867), QPoint(850609, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(935001, 1166867), QPoint(939959, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(961407, 1099066), QPoint(966365, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(966217, 1099066), QPoint(971175, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(761180, 1166867), QPoint(766138, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(896245, 1166867), QPoint(901203, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(923701, 1166867), QPoint(928659, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(827276, 1166867), QPoint(832234, 1184456)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(662560, 1099066), QPoint(667518, 1116655)), 0xf));
	search.push_back(SearchArea(QRect(QPoint(396843, 1116016), QPoint(401801, 1133605)), 0xf));

    ce.extract(pic, search, objs);
	int scale = 32768 / ic->getBlockWidth();
	FILE * fp = fopen("result.txt", "w");
	for (int i = 0; i < objs.size(); i++) {
		unsigned t = objs[i].type;
		fprintf(fp, "cell, l=%d, (x=%d,y=%d)->(x=%d,y=%d)\n", 0, objs[i].p0.x() / scale, objs[i].p0.y() / scale,
			objs[i].p1.x() / scale, objs[i].p1.y() / scale);
	}
	fclose(fp);

    return 0;
}
/*
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
}*/

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

int test_extractparam2()
{
	ExtractParam ep;
	vector<string> layer0, layer1, global;
	vector<string> action;
	BkImgRoMgr bkimg_faty;
#ifdef WIN32
	QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("C:/chenyu/data/A22180604/chip.prj", 0);
	//QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("C:/chenyu/data/A1002/chip_enc.prj", 0);
#else
	QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("/home/chenyu/work/share/imgdb/chip_enc.prj", 0);
#endif
	vector<SearchArea> search;

	ep.read_file("action.xml");
	vector<ParamItem> params;
	ep.get_param("action_all_layer", params);
	VWExtract * vwe = VWExtract::create_extract(0);

	vector<ICLayerWrInterface *> pic;
	
	set<int> layer_sets;
	vector<int> map_layer; //map from 0, 1, 2 to actual layer
	map<int, int> anti_map_layer; //map from ParamItem to layer 0, 1, 2

	for (int i = 0; i < params.size(); i++)
		if (params[i].pi[0] >= 0)
			layer_sets.insert(params[i].pi[0]);
	for (set<int>::iterator it = layer_sets.begin(); it != layer_sets.end(); it++) {
		anti_map_layer[*it] = map_layer.size();
		int l = -1;
		for (int i = 0; i < bk_img->getLayerNum(); i++) {
			string layer_name = bk_img->getLayerName(i);
			int t = layer_name.find_last_of('.');
			int h = layer_name.find_last_of('M', t);
			string sub = layer_name.substr(h + 1, t - h - 1);
			if (atoi(sub.c_str()) == *it) {
				l = i;
				break;
			}
		}
		if (l >= 0) {
			map_layer.push_back(l);
			pic.push_back(bk_img->get_layer(l));
			if (pic.back() == NULL)
				qFatal("vw_extract_req receive layer[%d]=%d, invalid", map_layer.size() - 1, l);
			else
				qInfo("vw_extract_req receive layer[%d]=%d", map_layer.size() - 1, l);
		}
		else
			qFatal("vw_extract_req layer M%d not exist", *it);
	}

	for (int l = 0; l < params.size(); l++) {
		if (params[l].pi[0] >= 0) {
			vwe->set_extract_param(anti_map_layer[params[l].pi[0]], params[l].pi[1], params[l].pi[2], params[l].pi[3], params[l].pi[4],
				params[l].pi[5], params[l].pi[6], params[l].pi[7], params[l].pi[8], params[l].pf);
		}
		else
			vwe->set_extract_param(params[l].pi[0], params[l].pi[1], params[l].pi[2], params[l].pi[3], params[l].pi[4],
			params[l].pi[5], params[l].pi[6], params[l].pi[7], params[l].pi[8], params[l].pf);

		qInfo("extract l=%d, i0=%x,i1=%x,i2=%x,i3=%x,i4=%x,i5=%x,i6=%x,i7=%x,i8=%x,f=%f", params[l].pi[0],
			params[l].pi[1], params[l].pi[2], params[l].pi[3], params[l].pi[4],
			params[l].pi[5], params[l].pi[6], params[l].pi[7], params[l].pi[8], params[l].pf);
	}
	//search.push_back(SearchArea(QRect(QPoint(100000, 100000), QPoint(224000, 672000)), 0));
	search.push_back(SearchArea(QRect(QPoint(-78072, 14914), QPoint(100000, 100000)), 0));
	//search.push_back(SearchArea(QRect(QPoint(8000*32, 13000*32), QPoint(11000 *32, 15000*32)), 0));
	//search.push_back(SearchArea(QRect(QPoint(106496, 786528), QPoint(200302, 905000)), 0));
	//search.push_back(SearchArea(QRect(QPoint(106496, 720992), QPoint(674304, 1181792)), 0));

	vector <MarkObj> objs;
	vwe->extract(pic, search, objs);
	delete vwe;

	FILE * fp;
	int scale = 32768 / bk_img->getBlockWidth();
	fp = fopen("result.txt", "w");
	for (int i = 0; i < objs.size(); i++) {
		unsigned t = objs[i].type;
		if (t == OBJ_POINT) {
			fprintf(fp, "via, l=%d, x=%d, y=%d, prob=%f\n", map_layer[objs[i].type3], objs[i].p0.x() / scale, objs[i].p0.y() / scale, objs[i].prob);
		}
		else {
			fprintf(fp, "wire, l=%d, (x=%d,y=%d)->(x=%d,y=%d), prob=%f\n", map_layer[objs[i].type3], objs[i].p0.x() / scale, objs[i].p0.y() / scale,
				objs[i].p1.x() / scale, objs[i].p1.y() / scale, objs[i].prob);
		}
		continue;
	}
	fclose(fp);
	return 0;
}

void test_ml_train()
{
	BkImgRoMgr bkimg_faty;
#ifdef WIN32
	QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("C:/chenyu/data/A22190918/chip.prj", "", 0);
#else
	QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("/home/chenyu/work/share/imgdb/chip_enc.prj", 0);
#endif
	vector<ICLayerWrInterface *> pic;
	for (int l = 0; l < bk_img->getLayerNum(); l++) {
		pic.push_back(bk_img->get_layer(l));
		CV_Assert(pic.back() != NULL);
	}
	QScopedPointer<VWExtract> vwe(VWExtract::create_extract(2));
	vector<MarkObj> objs;
	vwe->set_train_param(0, 0x0e0d, 0, 0, 0, 0, 0, 0, 0, 0);
	MarkObj obj;
	obj.type = OBJ_POINT;
	obj.type2 = POINT_NO_VIA;
	obj.type3 = 1;
	obj.p0 = QPoint(169280, 43424);
	obj.p1 = QPoint(0, 0);
	objs.push_back(obj);	
	vwe->train(pic, objs);
}

void test_ml_extract()
{
	BkImgRoMgr bkimg_faty;
#ifdef WIN32
	QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("C:/chenyu/data/A22190918/chip.prj", "", 0);
#else
	QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("/home/chenyu/work/share/imgdb/chip_enc.prj", 0);
#endif
	vector<ICLayerWrInterface *> pic;
	for (int l = 0; l < bk_img->getLayerNum(); l++) {
		pic.push_back(bk_img->get_layer(l));
		CV_Assert(pic.back() != NULL);
	}
	QScopedPointer<VWExtract> vwe(VWExtract::create_extract(2));
	vector<MarkObj> objs;
	vwe->set_extract_param(0x0101, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	vector<SearchArea> areas;
	//QPoint tl(135680, 19968), rb(597505, 517121);
	//QPoint tl(135680, 369968), rb(397505, 517121);
	areas.push_back(SearchArea(QRect(128000, 128000, 256000, 128000), OPT_POLYGON_SEARCH));
	areas.push_back(SearchArea(QRect(256000, 128000, 256000, 32000), OPT_POLYGON_SEARCH));
	areas.push_back(SearchArea(QRect(256000, 32000, 384000, 32000), OPT_POLYGON_SEARCH));
	areas.push_back(SearchArea(QRect(384000, 32000, 384000, 128000), OPT_POLYGON_SEARCH));
	areas.push_back(SearchArea(QRect(384000, 128000, 512000, 128000), OPT_POLYGON_SEARCH));
	areas.push_back(SearchArea(QRect(512000, 128000, 512000, 256000), OPT_POLYGON_SEARCH));
	areas.push_back(SearchArea(QRect(512000, 256000, 128000, 256000), OPT_POLYGON_SEARCH | 1));
	vwe->extract(pic, areas, objs);
}

int test_single_wire_extract()
{
	BkImgRoMgr bkimg_faty;
	ParamItem pa;
#ifdef WIN32
	QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("C:/chenyu/data/A13/chip.prj", 0);
	//QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("C:/chenyu/data/A1002/chip_enc.prj", 0);
#else
	QSharedPointer<BkImgInterface> bk_img = bkimg_faty.open("/home/chenyu/work/share/imgdb/chip_enc.prj", 0);
#endif
	int wmax = 128, wmin = 5, channel = 0, gray_th = 50, opt = 0, ihigh = 5;
	int scale = 2, layer = 3;
	pa.pi[0] = layer;
	pa.pi[1] = 0xffffffff;
	pa.pi[2] = wmax << 16 | wmin;
	pa.pi[3] = channel << 24 | gray_th << 16 | opt << 8 | ihigh;
	pa.pi[4] = scale;
	VWExtract * vwe = VWExtract::create_extract(1);
	vwe->set_extract_param(pa.pi[0], pa.pi[1], pa.pi[2], pa.pi[3], pa.pi[4], pa.pi[5], pa.pi[6], pa.pi[7], pa.pi[8], pa.pf);
	qInfo("extract single wire l=%d, i0=%x,i1=%x,i2=%x,i3=%x,i4=%x,i5=%x,i6=%x,i7=%x,i8=%x,f=%f",
		pa.pi[0], pa.pi[1], pa.pi[2], pa.pi[3], pa.pi[4], pa.pi[5], pa.pi[6], pa.pi[7], pa.pi[8], pa.pf);
	QRect rect(QPoint(500544, 120640), QPoint(500544, 120640));
	vector<SearchArea> search;
	search.push_back(SearchArea(rect, 0));
	vector<ICLayerWrInterface *> pic;
	pic.push_back(bk_img->get_layer(pa.pi[0]));
	if (pic.back() == NULL)
		qFatal("test_single_wire_extract receive layer[%d]=%d, invalid", pa.pi[0]);
	else
		qInfo("test_single_wire_extract receive layer[%d]=%d", pa.pi[0]);
	vector <MarkObj> objs;
	vwe->extract(pic, search, objs);
	delete vwe;
	FILE * fp;
	scale = 32768 / bk_img->getBlockWidth();
	fp = fopen("result.txt", "w");
	for (int i = 0; i < objs.size(); i++) {
		unsigned t = objs[i].type;
		if (t == OBJ_POINT) {
			fprintf(fp, "via, l=%d, x=%d, y=%d\n", pa.pi[0], objs[i].p0.x() / scale, objs[i].p0.y() / scale);
		}
		else {
			fprintf(fp, "wire, l=%d, (x=%d,y=%d)->(x=%d,y=%d)\n", pa.pi[0], objs[i].p0.x() / scale, objs[i].p0.y() / scale,
				objs[i].p1.x() / scale, objs[i].p1.y() / scale);
		}
		continue;
	}
	fclose(fp);
	return 0;
}
#ifdef Q_OS_WIN
#ifdef QT_DEBUG
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif

#include <Windows.h>
#include <DbgHelp.h>
void CreateMiniDump(PEXCEPTION_POINTERS pep, LPCTSTR strFileName)
{ 
	HANDLE hFile = CreateFile(strFileName, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if ((hFile != NULL) && (hFile != INVALID_HANDLE_VALUE)) {
		MINIDUMP_EXCEPTION_INFORMATION mdei;
		mdei.ThreadId = GetCurrentThreadId();
		mdei.ExceptionPointers = pep;
		mdei.ClientPointers = FALSE;
		//MINIDUMP_TYPE mdt = (MINIDUMP_TYPE)(MiniDumpWithFullMemory | MiniDumpWithFullMemoryInfo | MiniDumpWithHandleData | MiniDumpWithThreadInfo | MiniDumpWithUnloadedModules);
		MINIDUMP_TYPE mdt = (MINIDUMP_TYPE)(MiniDumpWithHandleData | MiniDumpWithThreadInfo);
		BOOL rv = MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, mdt, (pep != 0) ? &mdei : 0, 0, 0);
		CloseHandle(hFile);
	}
} 

LONG __stdcall MyUnhandledExceptionFilter(PEXCEPTION_POINTERS pExceptionInfo)
{
	CreateMiniDump(pExceptionInfo, L"core.dmp");
	qWarning("crash happen");
	return EXCEPTION_EXECUTE_HANDLER;
}
#endif
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
#ifdef Q_OS_WIN
#ifdef QT_DEBUG
	//_CrtSetBreakAlloc(36207);
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);  
	_CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_DEBUG);
#endif
	SetUnhandledExceptionFilter(MyUnhandledExceptionFilter);
#endif
	qInstallMessageHandler(myMessageOutput);
#if 0
	
	//wire_extract_test_pipeprocess();
	cell_extract_test();
	//test_extractparam2();
	//test_single_wire_extract();
	//test_ml_extract();
	return 0;
#endif
    w.show();

    int ret = a.exec();
	qWarning("code exit with ret=%d", ret);
	return ret;
}