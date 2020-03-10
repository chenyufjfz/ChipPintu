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
	ICLayerWrInterface * ic = ICLayerWrInterface::create("C:/chenyu/data/A13/PL.db", "", true, 1.0, 1.0, 0, 0, 2, 0, 0);
	
    vector<ICLayerWrInterface *> pic;
    pic.push_back(ic);
	ce.set_train_param(0, 0, 0, 0, 0, 0,  20, 300, 50, 0);
	MarkObj obj;
	vector<MarkObj> objs;
	obj.type = OBJ_AREA;
	obj.type2 = AREA_CELL;
	obj.type3 = POWER_UP_L;
	obj.p0 = QPoint(1114802,377712);
	obj.p1 = QPoint(1116083,382201);
	obj.state = 0;
	objs.push_back(obj);
    ce.train(pic, objs);
	
	vector<SearchArea> search;
	search.push_back(SearchArea(QRect(QPoint(103442, 1372746), QPoint(1774121, 1377875)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1367615), QPoint(1774121, 1372744)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1362485), QPoint(1774121, 1367614)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(681927, 1357354), QPoint(1774121, 1362483)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1352223), QPoint(1774121, 1357352)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1347093), QPoint(1774121, 1352222)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1341962), QPoint(1774121, 1347091)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1336831), QPoint(1774121, 1341960)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683210, 1331701), QPoint(1773480, 1336830)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1326570), QPoint(1774121, 1331699)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1321439), QPoint(1774121, 1326568)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1316308), QPoint(1774121, 1321437)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1311178), QPoint(1774121, 1316307)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(681927, 1306047), QPoint(1774121, 1311176)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1300916), QPoint(1774121, 1306045)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1295786), QPoint(1774121, 1300915)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1290655), QPoint(1774121, 1295784)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1285524), QPoint(1774121, 1290653)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(682568, 1280394), QPoint(1774121, 1285523)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1275263), QPoint(1774121, 1280392)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1270132), QPoint(1774121, 1275261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1265002), QPoint(1774121, 1270131)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1259871), QPoint(1774121, 1265000)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1254740), QPoint(1774121, 1259869)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(678079, 1249610), QPoint(1773480, 1254739)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1244479), QPoint(1774121, 1249608)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1239348), QPoint(1774121, 1244477)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1234217), QPoint(1774121, 1239346)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1229087), QPoint(1774121, 1234216)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(682568, 1223956), QPoint(1774121, 1229085)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1218825), QPoint(1774121, 1223954)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1213695), QPoint(1774121, 1218824)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1208564), QPoint(1774121, 1213693)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1203433), QPoint(1774121, 1208562)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683210, 1198303), QPoint(1773480, 1203432)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1193172), QPoint(1774121, 1198301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1188041), QPoint(1774121, 1193170)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1182911), QPoint(1774121, 1188040)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1177780), QPoint(1774121, 1182909)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(682568, 1172649), QPoint(1774121, 1177778)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1167518), QPoint(1122524, 1172647)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1162388), QPoint(1122524, 1167517)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1157257), QPoint(1122524, 1162386)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1152126), QPoint(1122524, 1157255)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(682568, 1146996), QPoint(1122523, 1152125)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1141865), QPoint(1122524, 1146994)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1136734), QPoint(1122524, 1141863)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1121342), QPoint(1122524, 1126471)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1116212), QPoint(1122524, 1121341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1111081), QPoint(1122524, 1116210)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1105950), QPoint(1122524, 1111079)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1100820), QPoint(1122524, 1105949)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1095689), QPoint(1122524, 1100818)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(682568, 1090558), QPoint(1121882, 1095687)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1085427), QPoint(1122524, 1090556)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1080297), QPoint(1122524, 1085426)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1075166), QPoint(1122524, 1080295)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1070035), QPoint(1122524, 1075164)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683210, 1064905), QPoint(1121882, 1070034)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1059774), QPoint(1122524, 1064903)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1054643), QPoint(1122524, 1059772)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1049513), QPoint(1122524, 1054642)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1044382), QPoint(1122524, 1049511)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(681927, 1039251), QPoint(1121882, 1044380)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1034121), QPoint(1122524, 1039250)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1028990), QPoint(1122524, 1034119)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1023859), QPoint(1122524, 1028988)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1018729), QPoint(1122524, 1023858)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(685454, 1013598), QPoint(1124768, 1018727)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1008467), QPoint(1122524, 1013596)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 1003336), QPoint(1122524, 1008465)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 993075), QPoint(1122524, 998204)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683851, 987944), QPoint(1123165, 993073)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 982814), QPoint(1122524, 987943)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 977683), QPoint(1122524, 982812)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 972552), QPoint(1122524, 977681)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 967422), QPoint(1122524, 972551)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683210, 962291), QPoint(1121882, 967420)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 957160), QPoint(1122524, 962289)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 952030), QPoint(1122524, 957159)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 946899), QPoint(1122524, 952028)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 941768), QPoint(1122524, 946897)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683210, 936637), QPoint(1121882, 941766)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 931507), QPoint(1122524, 936636)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 926376), QPoint(1122524, 931505)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 921245), QPoint(1122524, 926374)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 916115), QPoint(1122524, 921244)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683691, 910984), QPoint(1123005, 916113)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 905853), QPoint(1122524, 910982)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 900723), QPoint(1122524, 905852)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 895592), QPoint(1122524, 900721)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 890461), QPoint(1122524, 895590)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683210, 885331), QPoint(1122524, 890460)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 880200), QPoint(1122524, 885329)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 875069), QPoint(1122524, 880198)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 869939), QPoint(1122524, 875068)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 864808), QPoint(1122524, 869937)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 859677), QPoint(1122524, 864806)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 844285), QPoint(1122524, 849414)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 839154), QPoint(1122524, 844283)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(681286, 834024), QPoint(1122524, 839153)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 828893), QPoint(1122524, 834022)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 823762), QPoint(1122524, 828891)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 818632), QPoint(1122524, 823761)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 813501), QPoint(1122524, 818630)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(682568, 808370), QPoint(1121882, 813499)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 803240), QPoint(1122524, 808369)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 798109), QPoint(1122524, 803238)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 792978), QPoint(1122524, 798107)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(681286, 787848), QPoint(1121882, 792977)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 782717), QPoint(1122524, 787846)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 777586), QPoint(1122524, 782715)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 772455), QPoint(1122524, 777584)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 767325), QPoint(1122524, 772454)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(681286, 762194), QPoint(1121882, 767323)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 757063), QPoint(1122524, 762192)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 751933), QPoint(1122524, 757062)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 746802), QPoint(1122524, 751931)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 741671), QPoint(1122524, 746800)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(683210, 736541), QPoint(1121882, 741670)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 731410), QPoint(1122524, 736539)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 726279), QPoint(1122524, 731408)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 721149), QPoint(1122524, 726278)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 716018), QPoint(1122524, 721147)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(684492, 710887), QPoint(1121882, 716016)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 705756), QPoint(1122524, 710885)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 700626), QPoint(1122524, 705755)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 695495), QPoint(1122524, 700624)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 690364), QPoint(1122524, 695493)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(681286, 685234), QPoint(1121241, 690363)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(681286, 680103), QPoint(1121241, 685232)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 674972), QPoint(1121882, 680101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 669842), QPoint(1121882, 674971)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 664711), QPoint(1121882, 669840)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 659580), QPoint(1121882, 664709)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 654450), QPoint(1121882, 659579)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 649319), QPoint(1121882, 654448)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 644188), QPoint(1121882, 649317)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 639058), QPoint(1121882, 644187)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 633927), QPoint(1121882, 639056)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 628796), QPoint(1121882, 633925)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 623665), QPoint(1121882, 628794)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 618535), QPoint(1121882, 623664)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 613404), QPoint(1121882, 618533)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 608273), QPoint(1121882, 613402)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 603143), QPoint(1121882, 608272)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 598012), QPoint(1121882, 603141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 592881), QPoint(1121882, 598010)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 587751), QPoint(1121882, 592880)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229144, 582620), QPoint(1124448, 587749)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 562097), QPoint(1122523, 567226)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 556967), QPoint(1122523, 562096)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 551836), QPoint(1122523, 556965)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 546705), QPoint(1122523, 551834)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 541574), QPoint(1122523, 546703)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 536444), QPoint(1122523, 541573)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 531313), QPoint(1122523, 536442)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 526182), QPoint(1122523, 531311)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 521052), QPoint(1122523, 526181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 515921), QPoint(1122523, 521050)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 510790), QPoint(1122523, 515919)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 505660), QPoint(1122523, 510789)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 500529), QPoint(1122523, 505658)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 495398), QPoint(1122523, 500527)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 490268), QPoint(1122523, 495397)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 485137), QPoint(1122523, 490266)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 480006), QPoint(1122523, 485135)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 474875), QPoint(1122523, 480004)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 469745), QPoint(1122523, 474874)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 464614), QPoint(1122523, 469743)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 459483), QPoint(1122523, 464612)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 454353), QPoint(1122523, 459482)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 449222), QPoint(1122523, 454351)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 444091), QPoint(1122523, 449220)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(226578, 438961), QPoint(1122523, 444090)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 433830), QPoint(1123165, 438959)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 428699), QPoint(1123165, 433828)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 423569), QPoint(1123165, 428698)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 418438), QPoint(1123165, 423567)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 413307), QPoint(1123165, 418436)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 408177), QPoint(1123165, 413306)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 403046), QPoint(1123165, 408175)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 397915), QPoint(1123165, 403044)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 392784), QPoint(1123165, 397913)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 387654), QPoint(1123165, 392783)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 382523), QPoint(1123165, 387652)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 377392), QPoint(1123165, 382521)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 372262), QPoint(1123165, 377391)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 367131), QPoint(1123165, 372260)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 362000), QPoint(1123165, 367129)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 356870), QPoint(1123165, 361999)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 351739), QPoint(1123165, 356868)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 346608), QPoint(1123165, 351737)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(104083, 341478), QPoint(1125089, 346607)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 336347), QPoint(1123165, 341476)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 331216), QPoint(1123165, 336345)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(102800, 695495), QPoint(1121882, 700624)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 320955), QPoint(1123165, 326084)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 315824), QPoint(1123165, 320953)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(103442, 310693), QPoint(1123165, 315822)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 295301), QPoint(1124447, 300430)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 290171), QPoint(1124447, 295300)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 285040), QPoint(1125089, 290169)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 279909), QPoint(1157156, 285038)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 274779), QPoint(1157156, 279908)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 269648), QPoint(1157156, 274777)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 264517), QPoint(1157156, 269646)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 259387), QPoint(1157156, 264516)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 254256), QPoint(1157156, 259385)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 249125), QPoint(1157156, 254254)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 243994), QPoint(1157156, 249123)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 238864), QPoint(1157156, 243993)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 233733), QPoint(1157156, 238862)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 228602), QPoint(1157156, 233731)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 223472), QPoint(1157156, 228601)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 218341), QPoint(1157156, 223470)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 213210), QPoint(1157156, 218339)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 208080), QPoint(1157156, 213209)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 202949), QPoint(1157156, 208078)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 197818), QPoint(1157156, 202947)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 192688), QPoint(1157156, 197817)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 187557), QPoint(1157156, 192686)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 182426), QPoint(1157156, 187555)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 177296), QPoint(1157156, 182425)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 172165), QPoint(1157156, 177294)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 167034), QPoint(1157156, 172163)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 161903), QPoint(1157156, 167032)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 156773), QPoint(1157156, 161902)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 151642), QPoint(1157156, 156771)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(229785, 146511), QPoint(1157156, 151640)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(230426, 141381), QPoint(1157155, 146510)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(230426, 136250), QPoint(1157155, 141379)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(230426, 131119), QPoint(1157155, 136248)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(230426, 125989), QPoint(1157155, 131118)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(230426, 120858), QPoint(1157155, 125987)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(230426, 115727), QPoint(1157155, 120856)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(230426, 110597), QPoint(1157155, 115726)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(230587, 105466), QPoint(1157316, 110595)), POWER_UP | POWER_DOWN));

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
	//cell_extract_test();
	//test_extractparam2();
	//test_single_wire_extract();
	test_ml_extract();
	return 0;
#endif
    w.show();

    int ret = a.exec();
	qWarning("code exit with ret=%d", ret);
	return ret;
}