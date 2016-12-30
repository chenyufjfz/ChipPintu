#include "iclayer.h"
#include <fstream>
#include <QPixmap>
#include <QBuffer>

typedef long long int64;
using namespace std;

class ICLayer : public ICLayerInterface
{
protected:
    vector<vector<int64> > bias;
    vector<vector<int> > len;
    vector<QPoint> corners;
    string file_;
    ifstream fin;
	ofstream fout;
    int num_block_x, num_block_y; //image num in row and col
    int block_w; //image width and height
public:
    ICLayer(string& file, bool read);
	~ICLayer();
    int readLayerFile(const string& file);
	int addRawImg(vector<uchar> & buff, int , int , int);
    int getCorners(vector<QPoint> & corners);
    int getBlockWidth();
	void getBlockNum(int & bx, int &by);
	void putBlockNumWidth(int bx, int by, int width);
    int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved);
};


int ICLayer::readLayerFile(const string& file)
{
    //DEBUG_AUTO_GUARD;
    bias.clear();
    corners.clear();
    //DEBUG_LOG(QString::fromStdString(file));
    fin.open(file.c_str(), ios::binary);
    if (!fin.is_open())
		return -1;

    int ovr_num = 5;
    
	int64 block_bias = 3 * sizeof(int); //skip num_block_x, num_block_y, and width
	int  head_sz = (ovr_num + 4)*sizeof(int);

    fin.seekg(0, ios::end);
    int64 file_len = fin.tellg();
    fin.seekg(0, ios::beg);
	fin.read((char*)&num_block_x, sizeof(int));
	fin.read((char*)&num_block_y, sizeof(int));
	fin.read((char*)&block_w, sizeof(int));
    while (block_bias < file_len)
    {
        fin.seekg(block_bias, ios::beg);
        unsigned int block_len;
		fin.read((char*)&block_len, sizeof(int));

        vector<int64> bias_sub;//x=bias y=length;
        vector<int> len_sub;
        int bias_in = 0;
        for (int i = 0; i < ovr_num; i++)
        {
            int64 bias_tmp;
            int len_tmp;
			fin.read((char*)&len_tmp, sizeof(int));
            bias_tmp = block_bias + head_sz + bias_in;
            //cout << "bias:" << pt.x << "   len:" << pt.y << endl;
            bias_sub.push_back(bias_tmp);
            len_sub.push_back(len_tmp);
            bias_in += len_tmp;
        }

        int x, y;
		fin.read((char*)&x, sizeof(int));
		fin.read((char*)&y, sizeof(int));
        corners.push_back(QPoint(x, y));
		
		qDebug("Add %d, %d, %d, %d, %d", len_sub[0], len_sub[1], len_sub[2], len_sub[3], len_sub[4]);
        bias.push_back(bias_sub);
        len.push_back(len_sub);
        block_bias += block_len;
    }
    return 0;
}

int ICLayer::addRawImg(vector<uchar> & buff, int , int , int)
{	
	int len[8] = { 0 };
	int block_len;

	len[0] = (int) buff.size();
	QImage bkimg, bkimg_1, bkimg_2, bkimg_3, bkimg_4;
	bkimg.loadFromData((uchar*)&buff[0], len[0]);

	QByteArray ba1, ba2, ba3, ba4;
	QBuffer buffer1(&ba1), buffer2(&ba2), buffer3(&ba3), buffer4(&ba4);

	bkimg_1 = bkimg.scaled(bkimg.width() / 2, bkimg.height() / 2);
	buffer1.open(QIODevice::WriteOnly);
	bkimg_1.save(&buffer1, "JPG"); // scale image into ba in JPG format
	len[1] = ba1.size();	

	bkimg_2 = bkimg_1.scaled(bkimg_1.width() / 2, bkimg_1.height() / 2);
	buffer2.open(QIODevice::WriteOnly);
	bkimg_2.save(&buffer2, "JPG"); // scale image into ba in JPG format
	len[2] = ba2.size();	

	bkimg_3 = bkimg_2.scaled(bkimg_2.width() / 2, bkimg_2.height() / 2);
	buffer3.open(QIODevice::WriteOnly);
	bkimg_3.save(&buffer3, "JPG"); // scale image into ba in JPG format
	len[3] = ba3.size();

	bkimg_4 = bkimg_3.scaled(bkimg_3.width() / 2, bkimg_3.height() / 2);
	buffer4.open(QIODevice::WriteOnly);
	bkimg_4.save(&buffer4, "JPG"); // scale image into ba in JPG format
	len[4] = ba4.size();
	
    block_len = len[0] + len[1] + len[2] + len[3] + len[4] + 9*sizeof(int);
	fout.write((char*)&block_len, sizeof(int));
	fout.write((char*)len, sizeof(int) * 8);
	fout.write((char*)&buff[0],len[0]);
	fout.write(ba1.data(), len[1]);
	fout.write(ba2.data(), len[2]);
	fout.write(ba3.data(), len[3]);
	fout.write(ba4.data(), len[4]);

	qDebug("Add %d, %d, %d, %d, %d, bl=%d", len[0], len[1], len[2], len[3], len[4], block_len);
	return 0;
}

ICLayer::ICLayer(string& file, bool read) :file_(file)
{
	if (read) {
		if (readLayerFile(file) != 0)
			qFatal("layer file not exist");
	}		
	else
		fout.open(file_.c_str(), ofstream::binary);
    
}

ICLayer::~ICLayer()
{
	if (fin.is_open())
		fin.close();
	if (fout.is_open())
		fout.close();
}
int ICLayer::getCorners(vector< QPoint >& corners)
{
    //DEBUG_AUTO_GUARD;
    corners = this->corners;
    return 0;
}

int ICLayer::getBlockWidth()
{
    return block_w;
}

void ICLayer::getBlockNum(int & bx, int &by)
{
	bx = num_block_x;
	by = num_block_y;
}

void ICLayer::putBlockNumWidth(int bx, int by, int width)
{
	num_block_x = bx;
	num_block_y = by;
	block_w = width;
	fout.write((char*)&bx, sizeof(int));
	fout.write((char*)&by, sizeof(int));
	fout.write((char*)&width, sizeof(int));
}

int ICLayer::getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved)
{
	if (x < 0 || y < 0 || x >= num_block_x || y >= num_block_y) {
		qCritical("load image x=%d, y=%d, out of range", x, y);
		return -1;
	}
	int idx = y*num_block_x + x;
    if (idx > corners.size()) return -3;
    if (ovr > 5)return -2;
    buff.resize(len[idx][ovr] + reserved);
    fin.seekg(bias[idx][ovr], ios::beg);
    fin.read((char*)&buff[reserved], len[idx][ovr]);
    return 0;
}

ICLayerWr::ICLayerWr()
{
	layer_ = NULL;
}

ICLayerWr::ICLayerWr(string file, bool _read)
{
	create(file, _read);
}

ICLayerWr::~ICLayerWr()
{
    if (layer_)
        delete(layer_);
}

void ICLayerWr::create(string file, bool _read)
{
	layer_ = new ICLayer(file, _read);
}

int ICLayerWr::getBlockWidth()
{
    return layer_->getBlockWidth();
}

void ICLayerWr::getBlockNum(int & bx, int &by)
{
	return layer_->getBlockNum(bx, by);
}

int ICLayerWr::getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved)
{
    return layer_->getRawImgByIdx(buff, x, y, ovr, reserved);
}

void ICLayerWr::generateDatabase(string path, int from_row, int to_row, int from_col, int to_col)
{
	char file_name[500];
	vector<uchar> buff;
	
	for (int y = from_row; y <= to_row; y++)
		for (int x = from_col; x <= to_col; x++) {
			sprintf(file_name, "%s%d_%d.jpg", path.c_str(), y, x);
			ifstream infile(file_name, ifstream::binary);
			infile.seekg(0, infile.end);
			long size = infile.tellg();
			infile.seekg(0);

			buff.resize(size);
			infile.read((char*)&buff[0], size);

			if (y == from_row && x == from_col) {
				QImage bkimg;
				int len = (int)buff.size();
				bkimg.loadFromData((uchar*)&buff[0], len);
				layer_->putBlockNumWidth(to_col - from_col + 1, to_row - from_row + 1, bkimg.width());
			}
			layer_->addRawImg(buff, x - from_col, y - from_row, -1);
		}
	
}
