#include <stdio.h>
#include <string>
#include <QThread>
#include <QTime>
#include "circuit.h"
#include "circuitmatch.h"
using namespace std;


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
	Qt::HANDLE id = QThread::currentThreadId();
	if (context.function == NULL) {
		switch (type) {
		case QtDebugMsg:
			fprintf(fp, "<D>[%s] [%x] %s\n", qPrintable(str_dt), (int)id, qPrintable(msg));
			fflush(fp);
			break;
		case QtInfoMsg:
			fprintf(fp, "<I>[%s] [%x] %s\n", qPrintable(str_dt), (int)id, qPrintable(msg));
			fflush(fp);
			break;
		case QtWarningMsg:
			fprintf(fp, "<W>[%s] [%x] %s\n", qPrintable(str_dt), (int)id, qPrintable(msg));
			fflush(fp);
			break;
		case QtCriticalMsg:
			fprintf(fp, "<E>[%s] [%x] %s\n", qPrintable(str_dt), (int)id, qPrintable(msg));
			fflush(fp);
			break;
		case QtFatalMsg:
			fprintf(fp, "<F>[%s] [%x] %s\n", qPrintable(str_dt), (int)id, qPrintable(msg));
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
		fprintf(fp, "<D>[%d,%p] [%s] [%s:%s] %s\n", context.line, id, qPrintable(str_dt), file, func, qPrintable(msg));
		fflush(fp);
		break;
	case QtInfoMsg:
		fprintf(fp, "<I>[%d,%p] [%s] [%s:%s] %s\n", context.line, id, qPrintable(str_dt), file, func, qPrintable(msg));
		fflush(fp);
		break;
	case QtWarningMsg:
		fprintf(fp, "<W>[%d,%p] [%s] [%s:%s] %s\n", context.line, id, qPrintable(str_dt), file, func, qPrintable(msg));
		fflush(fp);
		break;
	case QtCriticalMsg:
		fprintf(fp, "<E>[%d,%p] [%s] [%s:%s] %s\n", context.line, id, qPrintable(str_dt), file, func, qPrintable(msg));
		fflush(fp);
		break;
	case QtFatalMsg:
		fprintf(fp, "<F>[%d,%p] [%s] [%s:%s] %s\n", context.line, id, qPrintable(str_dt), file, func, qPrintable(msg));
		fclose(fp);
		exit(-1);
	}
}

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
		MINIDUMP_TYPE mdt = (MINIDUMP_TYPE)(MiniDumpWithFullMemory | MiniDumpWithFullMemoryInfo | MiniDumpWithHandleData | MiniDumpWithThreadInfo | MiniDumpWithUnloadedModules);
		MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, mdt, (pep != 0) ? &mdei : 0, 0, 0);
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
#ifdef Q_OS_WIN
#ifdef QT_DEBUG
	//_CrtSetBreakAlloc(36207);
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_DEBUG);
	//_CrtSetBreakAlloc(182);
#endif
	SetUnhandledExceptionFilter(MyUnhandledExceptionFilter);
#endif
	qInstallMessageHandler(myMessageOutput);

	CircuitMatch c0, c1;
	c0.read_circuit("top.cdl");
	c1.read_circuit("osc.cdl");
	vector<string> nodes, subckt;
	/*nodes.push_back("GND");
	c0.predef_match_nodes(nodes);
	nodes.clear();
	nodes.push_back("GND");
	c1.predef_match_nodes(nodes);	
	subckt.push_back("V50_NAND2_24_14");
	subckt.push_back("V50_INV_24_14");
	c0.predef_match_subckt(subckt);
	c1.predef_match_subckt(subckt);	*/
	vector<MatchResult> result;
	match(c0, c1, "TOP", "OSC", MATCH_PART | SUBCKT_SAME_NAME, result);

	FILE * fp = fopen("result.txt", "w");
	for (int i = 0; i < (int)result.size(); i++) {
		for (int j = 0; (int)j < result[i].mnodes.size(); j++)
			fprintf(fp, "%20s --> %20s\n", result[i].mnodes[j].first.c_str(), result[i].mnodes[j].second.c_str());
		fprintf(fp, "Devices\n");
		for (int j = 0; (int)j < result[i].mdev.size(); j++)
			fprintf(fp, "%20s --> %20s\n", result[i].mdev[j].first.c_str(), result[i].mdev[j].second.c_str());
	}
	fclose(fp);
    return 0;
}

