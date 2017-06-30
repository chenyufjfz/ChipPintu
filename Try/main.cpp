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
void encrypt(char * a, int len, int magic);
void decrypt(char * a, int len, int magic);
int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);
	qInstallMessageHandler(myMessageOutput);

    BkImgInterface * bk_img_db = BkImgInterface::create_BkImgDB();
    
    bk_img_db->open("C:/chenyu/data/A09/chip.prj", false);
	bk_img_db->addNewLayer("DF.db", "C:/chenyu/data/A09/DF/DF_", 1, 20, 1, 20); 
	
	/*bk_img_db->addNewLayer("ST.db", "C:/chenyu/data/A09/ST/ST_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("PL.db", "C:/chenyu/data/A09/PL/PL_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("PL_R1.db", "C:/chenyu/data/A09/PL_R1/PL_R1_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("M1.db", "C:/chenyu/data/A09/M1/M1_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("M2.db", "C:/chenyu/data/A09/M2/M2_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("M3.db", "C:/chenyu/data/A09/M3/M3_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("M4.db", "C:/chenyu/data/A09/M4/M4_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("M5.db", "C:/chenyu/data/A09/M5/M5_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("M6.db", "C:/chenyu/data/A09/M6/M6_", 1, 100, 1, 98);
	bk_img_db->addNewLayer("M7.db", "C:/chenyu/data/A09/M7/M7_", 1, 100, 1, 98);*/
#if 1
	bk_img_db->open("C:/chenyu/data/A09/chip.prj", true);
#else
	bk_img_db->open("\\\\10.233.140.185/Linuxshare/imgdb/chip.prj", true);
#endif
	for (int l = 0; l < bk_img_db->getLayerNum(); l++) {
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
