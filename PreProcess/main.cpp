#include <QCoreApplication>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <QDateTime>
#include <fstream>
#include <QDir>
#include <string>
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
			fflush(fp);
            break;
        case QtInfoMsg:
            fprintf(fp, "<I>[%s] %s\n", qPrintable(str_dt), qPrintable(msg));
			fflush(fp);
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

class PreProcessParam {
public:
    string input_dir;
    string output_dir;
    int rotate;
    int clip_l, clip_r, clip_d, clip_u;
    int img_num_w, img_num_h;
    void read_file(string filename) {
        FileStorage fs(filename, FileStorage::READ);
        fs["input_dir"] >> input_dir;
        fs["output_dir"] >> output_dir;
        fs["rotate"] >> rotate;
        fs["clip_l"] >> clip_l;
        fs["clip_r"] >> clip_r;
        fs["clip_d"] >> clip_d;
        fs["clip_u"] >> clip_u;
        fs["img_num_w"] >> img_num_w;
        fs["img_num_h"] >> img_num_h;
        rotate = (rotate + 360) % 360;
    }

    string get_input_name(int x, int y) const {
        string::size_type ypos, xpos;
        ypos = input_dir.find("%y", 0);
        if (ypos == string::npos) {
            char filename[50];
            sprintf(filename, "%d_%d.jpg", y, x); //image name postfix
            return input_dir + filename;
        }
        else {
            char filename[10];
            sprintf(filename, "%d", y);
            string s = input_dir;
            s.replace(ypos, 2, filename, 0, strlen(filename));
            xpos = s.find("%x", 0);
            sprintf(filename, "%d", x);
            s.replace(xpos, 2, filename, 0, strlen(filename));
            return s;
        }
    }

    string get_output_name(int _x, int _y) const {
		int x, y;
        switch (rotate) {
        case 90:
            y = _x;
            x = img_num_h + 1 - _y;
            break;
        case 180:
            x = img_num_w + 1 - _x;
            y = img_num_h + 1 - _y;
            break;
        case 270:
            x = _y;
            y = img_num_w + 1 - _x;
            break;
        }

        string::size_type ypos, xpos;
        ypos = output_dir.find("%y", 0);
        if (ypos == string::npos) {
            char filename[50];
            sprintf(filename, "%d_%d.jpg", y, x); //image name postfix
            return output_dir + filename;
        }
        else {
            char filename[10];
            sprintf(filename, "%d", y);
            string s = output_dir;
            s.replace(ypos, 2, filename, 0, strlen(filename));
            xpos = s.find("%x", 0);
            sprintf(filename, "%d", x);
            s.replace(xpos, 2, filename, 0, strlen(filename));
            return s;
        }
    }
} gen;

Mat rotate(Mat m, int degree)
{
	Mat rm;
	switch (degree) {
	case 0:
		rm = m;
		return rm;
	case 180:
		rm.create(m.rows, m.cols, m.type());
		break;
	case 90:
	case 270:
		rm.create(m.cols, m.rows, m.type());
		break;
	default:
		qFatal("Rotate wrong");
	}
	switch (m.type()) {
	case CV_8UC1:
		for (int y = 0; y < m.rows; y++) {
			uchar * p = m.ptr<uchar>(y);
			for (int x = 0; x < m.cols; x++) {
				switch (degree) {
				case 90:
					rm.at<uchar>(x, m.rows - 1 - y) = p[x];
					break;
				case 180:
					rm.at<uchar>(m.rows - 1 - y, m.cols - 1 - x) = p[x];
					break;
				case 270:
					rm.at<uchar>(m.cols - 1 - x, y) = p[x];
					break;
				}
			}
		}
	case CV_8UC3:
		for (int y = 0; y < m.rows; y++) {
			Vec3b * p = m.ptr<Vec3b>(y);
			for (int x = 0; x < m.cols; x++) {
				switch (degree) {
				case 90:
					rm.at<Vec3b>(x, m.rows - 1 - y) = p[x];
					break;
				case 180:
					rm.at<Vec3b>(m.rows - 1 - y, m.cols - 1 - x) = p[x];
					break;
				case 270:
					rm.at<Vec3b>(m.cols - 1 - x, y) = p[x];
					break;
				}
			}
		}
	}
	return rm;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    qInstallMessageHandler(myMessageOutput);
    gen.read_file("rotate.xml");

    for (int y = 1; y <= gen.img_num_h; y++)
	for (int x = 1; x <= gen.img_num_w; x++) {
		string file_name = gen.get_input_name(x, y);
		qInfo("process %s", file_name.c_str());
		Mat m = imread(file_name, CV_LOAD_IMAGE_UNCHANGED);
		Mat rm = rotate(m, gen.rotate);
		file_name = gen.get_output_name(x, y);
		qInfo("output %s", file_name.c_str());
		try {
			if (!imwrite(file_name, rm))
				qFatal("Exception when imwrite\n");
		}
		catch (runtime_error& ex) {
			qFatal("Exception when imwrite %s\n", ex.what());
			return(1);
		}
	}
    return 1;
}

