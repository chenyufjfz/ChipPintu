#include <QCoreApplication>
#include <vector>
#include <QRect>
#include <string>
#include <fstream>
#include <QDateTime>
using namespace std;

class QRectSet {
protected:
    vector <QPoint> rects;
public:
    bool include(QPoint r, int d) const {
        for (int i=0; i<rects.size(); i++) {
            QPoint p;
			p = rects[i] - r;
			if (abs(p.x()) <= d && abs(p.y()) <= d)
				return true;
        }
		return false;
    }

	void read_file_0(string file, string cell_name) {
		ifstream fin(file);
		rects.clear();
		cell_name = "cell=\"" + cell_name + "\"";
		while (fin.good()) {
			string s;
			getline(fin, s);
			string::size_type loc;
			if ((loc = s.find(cell_name)) == string::npos)
				continue;			
			if ((loc = s.find("placement=")) == string::npos)
				continue;
			string sub_str = s.substr(loc + strlen("placement="));
			int p, l, r, t, b;
			if (sscanf(sub_str.c_str(), "%d left=%d right=%d top=%d bottom=%d", &p, &l, &r, &t, &b) != 5)
				continue;
			rects.push_back(QPoint(l, t));
		}
		qInfo("Read %s num=%d", file.c_str(), rects.size());
	}

	void read_file_1(string file, int w) {
		ifstream fin(file);
		rects.clear();
		while (fin.good()) {
			string s;
			getline(fin, s);
			string::size_type loc;
			if ((loc = s.find("Find new, img")) == string::npos)
				continue;
			string sub_str = s.substr(loc + strlen("Find new, img"));
			int iy, ix, x, y;
			char ss[100];
			if (sscanf(sub_str.c_str(), "(%d,%d) (x=%d,y=%d) %s", &iy, &ix, &x, &y, ss) != 5)
				continue;
			rects.push_back(QPoint(ix * w + x, iy * w + y));
		}
		qInfo("Read %s num=%d", file.c_str(), rects.size());
	}

	void write_file(string file) {
		FILE * fp = fopen(file.c_str(), "wt");
		for (int i = 0; i < rects.size(); i++)
			fprintf(fp, "x=%d, y=%d, (x32=%d, y32=%d)\n", rects[i].x(), rects[i].y(), rects[i].x() * 32, rects[i].y() * 32);
		fclose(fp);
	}
	friend QRectSet operator- (const QRectSet & lhs, const QRectSet & rhs);
};

QRectSet operator- (const QRectSet & lhs, const QRectSet & rhs)
{
	QRectSet r;

	for (int i = 0; i < lhs.rects.size(); i++) {
		if (!rhs.include(lhs.rects[i], 80))
			r.rects.push_back(lhs.rects[i]);
	}
	return r;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

	QRectSet r0, r1;
	QRectSet d0, d1;
	r0.read_file_0("C:/chenyu/work/ChipPintu/app/CELLS_ALL.csf", "SC_REGCELL_13_9");
	r1.read_file_1("C:/chenyu/work/ChipPintu/app/log.txt", 1024);
	d0 = r0 - r1;
	d1 = r1 - r0;
	r0.write_file("C:/chenyu/work/ChipPintu/app/r0.txt");
	r1.write_file("C:/chenyu/work/ChipPintu/app/r1.txt");
	d0.write_file("C:/chenyu/work/ChipPintu/app/r0-r1.txt");
	d1.write_file("C:/chenyu/work/ChipPintu/app/r1-r0.txt");
    //return a.exec();
}

