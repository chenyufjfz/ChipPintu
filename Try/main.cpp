#include <QCoreApplication>
#include "iclayer.h"
#include <QDateTime>
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

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
	qInstallMessageHandler(myMessageOutput);
	qDebug("generate new database");

    ICLayerWr new_db("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M4.dat", false);
    new_db.generateDatabase("F:/chenyu/work/ChipStitch/data/hanzhou/M4/Project_", 1, 60, 1, 75);
	qDebug("read new database");
    ICLayerWr read_db("F:/chenyu/work/ChipStitch/data/hanzhou/M1/M4.dat", true);
	printf("finished");
}
