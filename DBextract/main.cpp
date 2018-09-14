#include <QCoreApplication>
#include "iclayer.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <QDateTime>
#include <fstream>
#include <QDir>
using namespace std;
using namespace cv;

void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static FILE * fp = NULL;
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

class ExtractParam {
public:
    string prj_name;
    string output_dir;
    void read_file(string filename) {
        FileStorage fs(filename, FileStorage::READ);
        fs["prj_name"] >> prj_name;
        fs["output_dir"] >> output_dir;
    }
} gen;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    qInstallMessageHandler(myMessageOutput);
    BkImgInterface * bk_img_db = BkImgInterface::create_BkImgDB();
    gen.read_file("extract.xml");

    if (bk_img_db->open(gen.prj_name, true) !=0)
        qFatal("Prj file not exist");

    for (int l = 0; l < bk_img_db->getLayerNum(); l++) {
        vector<uchar> buff;
        int bx, by;
        string layer_name = bk_img_db->getLayerName(l);
        layer_name = layer_name.substr(layer_name.find_last_of("\\/"));
        layer_name = layer_name.substr(0, layer_name.find(".db"));
        qDebug("layer_name %s", layer_name.c_str());
        string target_path = gen.output_dir + layer_name;
        qDebug("target_path %s", target_path.c_str());
        bk_img_db->getBlockNum(bx, by);
        QDir *qdir = new QDir;
        bool exist = qdir->exists(QString::fromStdString(target_path));
        if (!exist) {
            bool ok = qdir->mkdir(QString::fromStdString(target_path));
            if (!ok)
                qFatal("mkdir failed");
        }
        delete qdir;
        for (int y=0; y<by; y++)
            for (int x=0; x<bx; x++) {
                bk_img_db->getRawImgByIdx(buff, l, x, y, 0, 0);
                if (buff.empty())
                    break;
                char expand[50];
                sprintf(expand, "_%d_%d.jpg", y+1, x+1);
                string file_name = target_path + "/" + layer_name + expand;
                qDebug("extract %s", file_name.c_str());
                ofstream fout(file_name, ofstream::binary);
                fout.write((char*)&buff[0], buff.size());
                fout.close();
            }
    }
    qDebug("finished");
    return 0;
}

