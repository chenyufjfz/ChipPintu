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
	ICLayerWr ic("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M1.db", true);
    vector<ICLayerWr *> pic;
    pic.push_back(&ic);
	ce.set_train_param(0, 0, 0, 0, 0, 0,  0.30, 3, 0.5, 0);
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
	search.push_back(SearchArea(QRect(QPoint(1463296, 295934), QPoint(1654911, 301181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 301054), QPoint(1654911, 306301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 306174), QPoint(1654911, 311421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 311294), QPoint(1654911, 316541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 316414), QPoint(1654911, 321661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 321534), QPoint(1654911, 326781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 326654), QPoint(1654911, 331901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 331774), QPoint(1654911, 337021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 336894), QPoint(1654911, 342141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 342014), QPoint(1654911, 347261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 347134), QPoint(1654911, 352381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 352254), QPoint(1654911, 357501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 357374), QPoint(1654911, 362621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 362494), QPoint(1654911, 367741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 367614), QPoint(1654911, 372861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 372734), QPoint(1654911, 377981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 377854), QPoint(1654911, 383101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 382974), QPoint(1654911, 388221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 388094), QPoint(1654911, 393341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 393214), QPoint(1654911, 398461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 398334), QPoint(1654911, 403581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 403454), QPoint(1654911, 408701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 408574), QPoint(1654911, 413821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 413694), QPoint(1654911, 418941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 418814), QPoint(1654911, 424061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 423934), QPoint(1654911, 429181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 429054), QPoint(1654911, 434301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 434174), QPoint(1654911, 439421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 439294), QPoint(1654911, 444541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 444414), QPoint(1654911, 449661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 449534), QPoint(1654911, 454781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 454654), QPoint(1654911, 459901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1463296, 459774), QPoint(1654911, 465021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 464894), QPoint(1657471, 470141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 470014), QPoint(1657471, 475261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 475134), QPoint(1657471, 480381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 480254), QPoint(1657471, 485501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 485374), QPoint(1657471, 490621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 490494), QPoint(1657471, 495741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 495614), QPoint(1657471, 500861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 500734), QPoint(1657471, 505981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 505854), QPoint(1657471, 511101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 510974), QPoint(1657471, 516221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 516094), QPoint(1657471, 521341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 521214), QPoint(1657471, 526461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 526334), QPoint(1657471, 531581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 531454), QPoint(1657471, 536701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 536574), QPoint(1657471, 541821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 541694), QPoint(1657471, 546941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 546814), QPoint(1657471, 552061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 551934), QPoint(1657471, 557181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 557054), QPoint(1657471, 562301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 562174), QPoint(1657471, 567421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 567294), QPoint(1657471, 572541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 572414), QPoint(1657471, 577661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 577534), QPoint(1657471, 582781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 582654), QPoint(1657471, 587901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1433088, 587774), QPoint(1657471, 593021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 746494), QPoint(1656703, 751741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 741374), QPoint(1656703, 746621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 736254), QPoint(1656703, 741501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 731134), QPoint(1656703, 736381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 726014), QPoint(1656703, 731261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 720894), QPoint(1656703, 726141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 715774), QPoint(1656703, 721021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 710654), QPoint(1656703, 715901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 705534), QPoint(1656703, 710781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 700414), QPoint(1656703, 705661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 695294), QPoint(1656703, 700541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 690174), QPoint(1656703, 695421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 685054), QPoint(1656703, 690301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 679934), QPoint(1656703, 685181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 674814), QPoint(1656703, 680061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 669694), QPoint(1656703, 674941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 664574), QPoint(1656703, 669821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 659454), QPoint(1656703, 664701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 654334), QPoint(1656703, 659581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 649214), QPoint(1656703, 654461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 644094), QPoint(1656703, 649341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 638974), QPoint(1656703, 644221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 633854), QPoint(1656703, 639101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 628734), QPoint(1656703, 633981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 623614), QPoint(1656703, 628861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 618494), QPoint(1656703, 623741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 613374), QPoint(1656703, 618621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 608254), QPoint(1656703, 613501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 603134), QPoint(1656703, 608381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 598014), QPoint(1656703, 603261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 592894), QPoint(1656703, 598141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 1012734), QPoint(1656703, 1017981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 1007614), QPoint(1656703, 1012861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 1002494), QPoint(1656703, 1007741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 997374), QPoint(1656703, 1002621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 992254), QPoint(1656703, 997501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 987134), QPoint(1656703, 992381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 982014), QPoint(1656703, 987261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 976894), QPoint(1656703, 982141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 971774), QPoint(1656703, 977021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 966654), QPoint(1656703, 971901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 961534), QPoint(1656703, 966781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 956414), QPoint(1656703, 961661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 951294), QPoint(1656703, 956541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 946174), QPoint(1656703, 951421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 941054), QPoint(1656703, 946301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 935934), QPoint(1656703, 941181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 930814), QPoint(1656703, 936061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 925694), QPoint(1656703, 930941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 920574), QPoint(1656703, 925821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 915454), QPoint(1656703, 920701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 910334), QPoint(1656703, 915581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 905214), QPoint(1656703, 910461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 900094), QPoint(1656703, 905341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 894974), QPoint(1656703, 900221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 889854), QPoint(1656703, 895101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 884734), QPoint(1656703, 889981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 879614), QPoint(1656703, 884861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 874494), QPoint(1656703, 879741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 869374), QPoint(1656703, 874621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 864254), QPoint(1656703, 869501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 859134), QPoint(1656703, 864381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 854014), QPoint(1656703, 859261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 848894), QPoint(1656703, 854141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 843774), QPoint(1656703, 849021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 838654), QPoint(1656703, 843901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 833534), QPoint(1656703, 838781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 828414), QPoint(1656703, 833661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 823294), QPoint(1656703, 828541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 818174), QPoint(1656703, 823421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 813054), QPoint(1656703, 818301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 807934), QPoint(1656703, 813181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 802814), QPoint(1656703, 808061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 797694), QPoint(1656703, 802941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 792574), QPoint(1656703, 797821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 787454), QPoint(1656703, 792701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 782334), QPoint(1656703, 787581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 777214), QPoint(1656703, 782461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 772094), QPoint(1656703, 777341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 766974), QPoint(1656703, 772221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 761854), QPoint(1656703, 767101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 756734), QPoint(1656703, 761981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1017854), QPoint(1656959, 1023101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1022974), QPoint(1656959, 1028221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1028094), QPoint(1656959, 1033341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1033214), QPoint(1656959, 1038461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1038334), QPoint(1656959, 1043581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1043454), QPoint(1656959, 1048701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1048574), QPoint(1656959, 1053821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1053694), QPoint(1656959, 1058941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1058814), QPoint(1656959, 1064061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1063934), QPoint(1656959, 1069181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1069054), QPoint(1656959, 1074301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1074174), QPoint(1656959, 1079421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1079294), QPoint(1656959, 1084541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1084414), QPoint(1656959, 1089661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1089534), QPoint(1656959, 1094781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1094654), QPoint(1656959, 1099901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1099774), QPoint(1656959, 1105021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1104894), QPoint(1656959, 1110141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1110014), QPoint(1656959, 1115261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1115134), QPoint(1656959, 1120381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1426688, 751614), QPoint(1656703, 756861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(983040, 1120254), QPoint(1656959, 1125501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1268734), QPoint(1657727, 1273981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1263614), QPoint(1657727, 1268861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1258494), QPoint(1657727, 1263741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1253374), QPoint(1657727, 1258621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1248254), QPoint(1657727, 1253501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1243134), QPoint(1657727, 1248381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1238014), QPoint(1657727, 1243261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1232894), QPoint(1657727, 1238141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1227774), QPoint(1657727, 1233021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1222654), QPoint(1657727, 1227901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1217534), QPoint(1657727, 1222781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1212414), QPoint(1657727, 1217661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1207294), QPoint(1657727, 1212541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1202174), QPoint(1657727, 1207421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1197054), QPoint(1657727, 1202301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1191934), QPoint(1657727, 1197181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1186814), QPoint(1657727, 1192061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1181694), QPoint(1657727, 1186941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1176574), QPoint(1657727, 1181821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1171454), QPoint(1657727, 1176701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1166334), QPoint(1657727, 1171581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1161214), QPoint(1657727, 1166461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1156094), QPoint(1657727, 1161341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1150974), QPoint(1657727, 1156221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1145854), QPoint(1657727, 1151101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1140734), QPoint(1657727, 1145981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1135614), QPoint(1657727, 1140861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1130494), QPoint(1657727, 1135741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1125374), QPoint(1657727, 1130621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1678334), QPoint(1657727, 1683581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1673214), QPoint(1657727, 1678461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1668094), QPoint(1657727, 1673341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1662974), QPoint(1657727, 1668221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1657854), QPoint(1657727, 1663101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1652734), QPoint(1657727, 1657981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1647614), QPoint(1657727, 1652861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1642494), QPoint(1657727, 1647741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1637374), QPoint(1657727, 1642621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1632254), QPoint(1657727, 1637501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1627134), QPoint(1657727, 1632381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1622014), QPoint(1657727, 1627261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1616894), QPoint(1657727, 1622141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1611774), QPoint(1657727, 1617021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1606654), QPoint(1657727, 1611901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1601534), QPoint(1657727, 1606781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1596414), QPoint(1657727, 1601661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1591294), QPoint(1657727, 1596541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1586174), QPoint(1657727, 1591421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1581054), QPoint(1657727, 1586301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1575934), QPoint(1657727, 1581181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1570814), QPoint(1657727, 1576061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1565694), QPoint(1657727, 1570941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1560574), QPoint(1657727, 1565821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1555454), QPoint(1657727, 1560701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1550334), QPoint(1657727, 1555581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1545214), QPoint(1657727, 1550461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1540094), QPoint(1657727, 1545341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1534974), QPoint(1657727, 1540221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1529854), QPoint(1657727, 1535101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1524734), QPoint(1657727, 1529981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1519614), QPoint(1657727, 1524861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1514494), QPoint(1657727, 1519741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1509374), QPoint(1657727, 1514621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1504254), QPoint(1657727, 1509501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1499134), QPoint(1657727, 1504381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1494014), QPoint(1657727, 1499261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1488894), QPoint(1657727, 1494141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1483774), QPoint(1657727, 1489021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1478654), QPoint(1657727, 1483901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1473534), QPoint(1657727, 1478781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1468414), QPoint(1657727, 1473661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1463294), QPoint(1657727, 1468541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1458174), QPoint(1657727, 1463421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1453054), QPoint(1657727, 1458301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1447934), QPoint(1657727, 1453181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1442814), QPoint(1657727, 1448061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1437694), QPoint(1657727, 1442941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1432574), QPoint(1657727, 1437821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1427454), QPoint(1657727, 1432701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1422334), QPoint(1657727, 1427581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1417214), QPoint(1657727, 1422461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1412094), QPoint(1657727, 1417341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1406974), QPoint(1657727, 1412221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1401854), QPoint(1657727, 1407101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1396734), QPoint(1657727, 1401981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1391614), QPoint(1657727, 1396861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1386494), QPoint(1657727, 1391741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1381374), QPoint(1657727, 1386621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1376254), QPoint(1657727, 1381501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1371134), QPoint(1657727, 1376381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1366014), QPoint(1657727, 1371261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1360894), QPoint(1657727, 1366141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1355774), QPoint(1657727, 1361021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1350654), QPoint(1657727, 1355901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1345534), QPoint(1657727, 1350781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1340414), QPoint(1657727, 1345661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1335294), QPoint(1657727, 1340541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1330174), QPoint(1657727, 1335421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1325054), QPoint(1657727, 1330301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1319934), QPoint(1657727, 1325181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1314814), QPoint(1657727, 1320061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1309694), QPoint(1657727, 1314941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1304574), QPoint(1657727, 1309821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1299454), QPoint(1657727, 1304701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1294334), QPoint(1657727, 1299581)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1289214), QPoint(1657727, 1294461)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1284094), QPoint(1657727, 1289341)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1278974), QPoint(1657727, 1284221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1403136, 1273854), QPoint(1657727, 1279101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1683454), QPoint(1657471, 1688701)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1688574), QPoint(1657471, 1693821)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1693694), QPoint(1657471, 1698941)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1698814), QPoint(1657471, 1704061)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1703934), QPoint(1657471, 1709181)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1709054), QPoint(1657471, 1714301)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1714174), QPoint(1657471, 1719421)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1719294), QPoint(1657471, 1724541)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1724414), QPoint(1657471, 1729661)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1729534), QPoint(1657471, 1734781)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1734654), QPoint(1657471, 1739901)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1739774), QPoint(1657471, 1745021)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1744894), QPoint(1657471, 1750141)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1750014), QPoint(1657471, 1755261)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1755134), QPoint(1657471, 1760381)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1760254), QPoint(1657471, 1765501)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1765374), QPoint(1657471, 1770621)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1770494), QPoint(1657471, 1775741)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1775614), QPoint(1657471, 1780861)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1780734), QPoint(1657471, 1785981)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1785854), QPoint(1657471, 1791101)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1790974), QPoint(1657471, 1796221)), POWER_UP | POWER_DOWN));
	search.push_back(SearchArea(QRect(QPoint(1387008, 1796094), QPoint(1657471, 1801341)), POWER_UP | POWER_DOWN));
    ce.extract(pic, search, objs);
#endif
    return 0;
}

int wire_extract_test()
{
	vector <MarkObj> objs;
	ICLayerWr ic[4]; 
	ic[0].create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M1.db", true);
	ic[1].create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M2.db", true);
	ic[2].create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M3.db", true);
	ic[3].create("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M4.db", true);
    vector<ICLayerWr *> pic;
    pic.push_back(&ic[0]);
    pic.push_back(&ic[1]);
    pic.push_back(&ic[2]);
	pic.push_back(&ic[3]);

	VWExtract * vwe = VWExtract::create_extract(0);
	vwe->set_extract_param(0, 4, 9, RULE_END_WITH_VIA, 0, 16, 0.5, 0.5, 2, 0);
	vwe->set_extract_param(1, 10, 9, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		RULE_NO_hCONN | RULE_VIA_NO_LCONN, 16, 0.5, 0.5, 2, 0);
	vwe->set_extract_param(2, 12, 10, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		RULE_NO_hCONN | RULE_VIA_NO_LCONN, 16, 0.5, 0.5, 2, 0);
	vwe->set_extract_param(3, 12, 10, RULE_NO_LOOP | RULE_NO_HCONN | RULE_NO_TT_CONN | RULE_END_WITH_VIA | RULE_EXTEND_VIA_OVERLAP | RULE_NO_ADJ_VIA_CONN,
		RULE_NO_hCONN | RULE_VIA_NO_LCONN, 16, 0.5, 0.5, 1, 0);
	vector<SearchArea> search; 
	search.push_back(SearchArea(QRect(QPoint(1427671, 674029), QPoint(1657047, 828653)), 0));
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
	cell_extract_test();
	return 0;
#endif
    w.show();

    return a.exec();
}

