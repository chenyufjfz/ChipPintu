#include <QCoreApplication>
#include "iclayer.h"
#include <QDateTime>
#include <fstream>
using namespace std;


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

void test_one_layer()
{
	ICLayerWr new_db("F:/chenyu/work/ChipStitch/data/hanzhou/M1/PL.db", false, false);
	//new_db.generateDatabase("F:/chenyu/work/ChipStitch/data/hanzhou/PL/Project_", 1, 60, 1, 75);
	new_db.generateDatabase("F:/chenyu/work/ChipStitch/data/hanzhou/PL/Project_", 1, 6, 1, 7);
	ICLayerWr read_db("F:/chenyu/work/ChipStitch/data/hanzhou/M1/PL.db", true, false);
	int bx, by, width;
	read_db.getBlockNum(bx, by);
	width = read_db.getBlockWidth();
	qDebug("read new database, bx=%d, by=%d, w=%d", bx, by, width);
	vector<uchar> buff;

	for (int s = 1; s < 16; s++) {
		char file_name[30];
		sprintf(file_name, "%d.jpg", s);
		read_db.getRawImgByIdx(buff, 0, 0, s, 0);
		if (buff.empty())
			break;
		ofstream fout(file_name, ofstream::binary);
		fout.write((char*)&buff[0], buff.size());
		fout.close();
	}
}

class DeleteMe {
public:
	int a;
	DeleteMe(int _a) { a = _a; }
	~DeleteMe() { qDebug("%d is killed", a); }
};
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
	qInstallMessageHandler(myMessageOutput);

    BkImgInterface * bk_img_db = BkImgInterface::create_BkImgDB();
    
    bk_img_db->open("F:/chenyu/work/ChipStitch/data/ADCLK/chip_a.prj", false);
	//bk_img_db->addNewLayer("1.db", "F:/chenyu/work/ChipStitch/data/ADCLK/1/ST_", 1, 41, 1, 40); 
	bk_img_db->addNewLayer("2.db", "F:/chenyu/work/ChipStitch/data/ADCLK/2/ST_", 1, 41, 1, 40);
	bk_img_db->addNewLayer("3.db", "F:/chenyu/work/ChipStitch/data/ADCLK/3/ST_", 1, 41, 1, 40);
	bk_img_db->addNewLayer("4.db", "F:/chenyu/work/ChipStitch/data/ADCLK/4/ST_", 1, 41, 1, 40);
	bk_img_db->addNewLayer("5.db", "F:/chenyu/work/ChipStitch/data/ADCLK/5/ST_", 1, 41, 1, 40);
	bk_img_db->addNewLayer("6.db", "F:/chenyu/work/ChipStitch/data/ADCLK/6/ST_", 1, 41, 1, 40);
	bk_img_db->addNewLayer("7.db", "F:/chenyu/work/ChipStitch/data/ADCLK/7/ST_", 1, 41, 1, 40);
#if 1
	bk_img_db->open("F:/chenyu/work/ChipStitch/data/ADCLK/chip_a.prj", true);
#else
	bk_img_db->open("\\\\10.233.140.185/Linuxshare/imgdb/chip.prj", true);
#endif
	for (int l = 0; l < 5; l++) {
		vector<uchar> buff;
		for (int s = 0; s < 16; s++) {
			char file_name[30];
			sprintf(file_name, "%d_%d.jpg", l, s);
			bk_img_db->getRawImgByIdx(buff, l, 0, 0, s, 0);
			if (buff.empty())
				break;
			ofstream fout(file_name, ofstream::binary);
			fout.write((char*)&buff[0], buff.size());
			fout.close();
		}		
	}
	
	qDebug("finished");
}
