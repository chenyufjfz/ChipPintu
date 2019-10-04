#include "iclayer.h"
#include <fstream>
#include <QPixmap>
#include <QBuffer>
#include <QPainter>
#include <stdio.h>
#include <QMutexLocker>
#include <QtConcurrent>
#ifdef USE_MDB
#include "lmdb.h"
#endif
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;

#define ENCRYPT_CHECK
#define GENERATE_DB_CONCURRENT 1

void encrypt(char * a, int len, int magic)
{
	magic = magic ^ ((magic + len) << 16) ^ len ^ (len * 11 >> 1) ^ (len * 101 >> 3) ^ (len * 3103 >> 6);
	for (int i = 0; i < len; i++) {
		unsigned b = a[i];
		a[i] = a[i] ^ (magic >> 6);
		magic = (magic ^ 0x55555555) * b + b;
	}
}

void decrypt(char * a, int len, int magic)
{
	magic = magic ^ ((magic + len) << 16) ^ len ^ (len * 11 >> 1) ^ (len * 101 >> 3) ^ (len * 3103 >> 6);
	for (int i = 0; i < len; i++) {		
		a[i] = a[i] ^ (magic >> 6);
		unsigned b = a[i];
		magic = (magic ^ 0x55555555) * b + b;
	}
}

struct ProcessScaleData {
	int x, y, s;
	vector<uchar> buf;
	ICLayerInterface * ic;
	ProcessScaleData(int _x, int _y, int _s, ICLayerInterface * _ic) {
		x = _x;
		y = _y;
		s = _s;
		ic = _ic;
	}
	ProcessScaleData() {
		ic = NULL;
	}
};
static ProcessScaleData scale_image_2x2(const ProcessScaleData & _d)
{
	ProcessScaleData t = _d;
	vector<uchar> buff;
	int x0 = t.x, y0 = t.y, s = t.s, v = 0;
	int x1 = x0 + (1 << (s - 1));
	int y1 = y0 + (1 << (s - 1));
	QImage image0, image1, image2, image3;
	if (t.ic->getRawImgByIdx(buff, x0, y0, s - 1, 0) == 0) {
		if (!image0.loadFromData((uchar *)&buff[0], (int)buff.size()))
			qFatal("image format error, (x=%d,y=%d,s=%d)", x0, y0, s - 1);
		v++;
	}
	else
		qFatal("image miss (x=%d,y=%d,s=%d)", x0, y0, s - 1);
	if (t.ic->getRawImgByIdx(buff, x1, y0, s - 1, 0) == 0) {
		if (!image1.loadFromData((uchar *)&buff[0], (int)buff.size()))
			qFatal("image format error, (x=%d,y=%d,s=%d)", x1, y0, s - 1);
		if (image0.width() != image1.width() || image0.height() != image1.height())
			qFatal("image format or size not same, (x0=%d,y0=%d,f0=%d,w0=%d,h0=%d),(x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", 
			x0, y0, image0.format(), image0.width(), image0.height(),
			x1, y0, s - 1, image1.format(), image1.width(), image1.height());
		v++;
	}
	if (t.ic->getRawImgByIdx(buff, x0, y1, s - 1, 0) == 0) {
		if (!image2.loadFromData((uchar *)&buff[0], (int)buff.size()))
			qFatal("image format error, (x=%d,y=%d,s=%d)", x0, y1, s - 1);
		if (image0.width() != image2.width() || image0.height() != image2.height())
			qFatal("image format or size not same, (x0=%d,y0=%d,f0=%d,w0=%d,h0=%d),(x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", 
			x0, y0, image0.format(), image0.width(), image0.height(),
			x0, y1, s - 1, image2.format(), image2.width(), image2.height());
		v++;
	}
	if (t.ic->getRawImgByIdx(buff, x1, y1, s - 1, 0) == 0) {
		if (!image3.loadFromData((uchar *)&buff[0], (int)buff.size()))
			qFatal("image format error, (x=%d,y=%d,s=%d)", x1, y1, s - 1);
		if (image0.width() != image3.width() || image0.height() != image3.height())
			qFatal("image format or size not same, (x0=%d,y0=%d,f0=%d,w0=%d,h0=%d),(x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", 
			x0, y0, image0.format(), image0.width(), image0.height(),
			x1, y1, s - 1, image3.format(), image3.width(), image3.height());
		v++;
	}
	if (v == 3)
		qFatal("image miss (x=%d, y=%d, s=%d)", x1, y1, s - 1);
	QImage image(image0.width() * 2, image0.height() * 2, image0.format());
	image.fill(QColor(0, 0, 0));
	QPainter painter(&image);
	painter.drawImage(0, 0, image0);
	if (!image1.isNull())
		painter.drawImage(image0.width(), 0, image1);
	if (!image2.isNull())
		painter.drawImage(0, image0.height(), image2);
	if (!image3.isNull())
		painter.drawImage(image0.width(), image0.height(), image3);
	QImage image_s;
	QByteArray ba;
	QBuffer buffer(&ba);
	image_s = image.scaled(image.width() / 2, image.height() / 2, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	buffer.open(QIODevice::WriteOnly);
	image_s.save(&buffer, "JPG", 85);
	t.buf.resize(ba.size());
	memcpy(t.buf.data(), ba.data(), ba.size());	
	return t;
}

static void scale_image_add(vector<int> & ret, const ProcessScaleData & d)
{
	d.ic->addRawImg(d.buf, d.x, d.y, d.s);
	ret.push_back(1);
}

class ICLayer : public ICLayerInterface
{
protected:
    vector<vector<long long> > bias;
    vector<vector<int> > len;
    vector<QPoint> corners;
    ifstream fin;
	ofstream fout;
	int num_block_x, num_block_y; //image num in row and col
	int block_w; //image width and height

public:
    ICLayer(const string& file, bool read);
	~ICLayer();	
    int readLayerFile(const string& file);
	int addRawImg(const vector<uchar> & buff, int , int , int);
    int getCorners(vector<QPoint> & corners);
    int getBlockWidth();
	void getBlockNum(int & bx, int &by);
	void putBlockNumWidth(int bx, int by, int width);
    int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved);
    int getMaxScale();
	void close();
	bool is_active() { return fin.is_open() | fout.is_open(); }
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
    
	long long block_bias = 3 * sizeof(int); //skip num_block_x, num_block_y, and width
	int  head_sz = (ovr_num + 4)*sizeof(int);

    fin.seekg(0, ios::end);
    long long file_len = fin.tellg();
    fin.seekg(0, ios::beg);
	fin.read((char*)&num_block_x, sizeof(int));
	fin.read((char*)&num_block_y, sizeof(int));
	fin.read((char*)&block_w, sizeof(int));
    while (block_bias < file_len)
    {
        fin.seekg(block_bias, ios::beg);
        unsigned int block_len;
		fin.read((char*)&block_len, sizeof(int));

        vector<long long> bias_sub;//x=bias y=length;
        vector<int> len_sub;
        int bias_in = 0;
        for (int i = 0; i < ovr_num; i++)
        {
            long long bias_tmp;
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

int ICLayer::addRawImg(const vector<uchar> & buff, int , int , int)
{	
	int len[8] = { 0 };
	int block_len;

	if (fout.is_open())
		return -1;
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

ICLayer::ICLayer(const string& file, bool read)
{
	if (read) {
		if (readLayerFile(file) != 0)
			qFatal("layer file not exist");
	}		
	else
		fout.open(file.c_str(), ofstream::binary);
    
}

ICLayer::~ICLayer()
{
	close();
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
	buff.clear();
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

int ICLayer::getMaxScale()
{
	return 4;
}

void ICLayer::close()
{
	if (fin.is_open())
		fin.close();
	if (fout.is_open())
		fout.close();
}
#ifdef USB_MDB
//ICLayerMdb can't be used for network file system
class ICLayerMdb : public ICLayerInterface
{
protected:
	MDB_env *env;
	MDB_dbi dbi;
	int num_block_x, num_block_y, block_w;
	
	struct ParaStruct {
		int bx, by, width;
		ParaStruct() {}
		ParaStruct(int _bx, int _by, int _width) {
			bx = _bx;
			by = _by;
			width = _width;
		}
	};
public:
	bool read_write, active;
	
public:
	ICLayerMdb(const string& file, bool _read);
	~ICLayerMdb();
	int getBlockWidth();
	void getBlockNum(int & bx, int &by);	
	void putBlockNumWidth(int bx, int by, int width);
	int addRawImg(vector<uchar> & buff, int x, int y, int);
	int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved);
	bool is_active() { return active; }
	int getMaxScale();
	void close();	
};

#define E(expr) do { \
	int rc = expr; \
	if (rc!=MDB_SUCCESS) {\
		qCritical("%s:%d: %s: %d, %s\n", __FILE__, __LINE__, #expr, rc, mdb_strerror(rc)); \
		active=false; return; \
	} \
} while (0) 

#define ER(expr, ret) do { \
	int rc = expr; \
	if (rc!=MDB_SUCCESS) {\
		qCritical("%s:%d: %s: %d, %s\n", __FILE__, __LINE__, #expr, rc, mdb_strerror(rc)); \
		active=false; return ret; \
		} \
} while (0) 

ICLayerMdb::ICLayerMdb(const string& file, bool _read)
{
	MDB_txn *txn;
	
	read_write = _read;
	active = false;
	E(mdb_env_create(&env));
	E(mdb_env_set_maxreaders(env, 100));
	E(mdb_env_set_mapsize(env, 0x800000000));
	E(mdb_env_set_maxdbs(env, 1));
	if (_read)
		if (mdb_env_open(env, file.c_str(), MDB_RDONLY | MDB_NOSUBDIR, 0664) != MDB_SUCCESS)
			return;
	else
		if (mdb_env_open(env, file.c_str(), MDB_WRITEMAP | MDB_NOMETASYNC | MDB_NOSUBDIR, 0664) != MDB_SUCCESS)
			return;

	E(mdb_txn_begin(env, NULL, 0, &txn));
	E(mdb_dbi_open(txn, NULL, 0, &dbi));
	E(mdb_txn_commit(txn));

	if (_read) {
		MDB_val key, data;
		unsigned long long id = 0xffffffffffffffff;
		ParaStruct * para;
		key.mv_size = sizeof(id);
		key.mv_data = &id;
		E(mdb_txn_begin(env, NULL, MDB_RDONLY, &txn));
		if (mdb_get(txn, dbi, &key, &data) != MDB_SUCCESS) {
			qCritical("database %s no block_num and block_w", file.c_str());
			num_block_x = 0;
			num_block_y = 0;
			block_w = 0;
			return;
		}
		para = (ParaStruct *)data.mv_data;
		num_block_x = para->bx;
		num_block_y = para->by;
		block_w = para->width;
		mdb_txn_abort(txn);
	}
	else {
		num_block_x = 0;
		num_block_y = 0;
		block_w = 0;
	}
	active = true;
}

ICLayerMdb::~ICLayerMdb()
{
	close();
}

void ICLayerMdb::getBlockNum(int & bx, int &by)
{
	bx = num_block_x;
	by = num_block_y;
}

int ICLayerMdb::getBlockWidth()
{
	return block_w;
}

void ICLayerMdb::putBlockNumWidth(int bx, int by, int width)
{
	if (read_write || !active) //if it opens for read, return
		return;

	MDB_val key, data;
	unsigned long long id = 0xffffffffffffffff;
	ParaStruct para(bx, by, width);
	MDB_txn *txn;
		
	key.mv_size = sizeof(id);
	key.mv_data = &id;
	data.mv_size = sizeof(para);
	data.mv_data = &para;
	E(mdb_txn_begin(env, NULL, 0, &txn)); //write to database
	E(mdb_put(txn, dbi, &key, &data, 0));
	E(mdb_txn_commit(txn));
	num_block_x = bx;
	num_block_y = by;
	block_w = width;
}

int ICLayerMdb::getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved)
{
	buff.clear();
	if (!active || ovr > 15 || x < 0 || x >= num_block_x || y < 0 || y >= num_block_y)
		return -1;
	MDB_txn *txn;
	MDB_val key, data;
	unsigned long long id = (unsigned long long) x << 32 | y << 16 | ovr;

	key.mv_size = sizeof(id);
	key.mv_data = &id;
	ER(mdb_txn_begin(env, NULL, MDB_RDONLY, &txn), -2);
	if (mdb_get(txn, dbi, &key, &data) != MDB_SUCCESS) {		
		qCritical("read database error (x=%d,y=%d,s=%d)", x, y, ovr);
		return -3;
	}
	buff.resize(reserved + data.mv_size);
	memcpy(&buff[reserved], data.mv_data, data.mv_size);
	mdb_txn_abort(txn);
	return 0;
}

int ICLayerMdb::addRawImg(vector<uchar> & buff, int x, int y, int)
{
	if (x < 0 || x >= num_block_x || y < 0 || y >= num_block_y)
		return -1;
	if (read_write || !active) //if it opens for read, return
		return -2;
	MDB_txn *txn;
	MDB_val key, data;
	unsigned long long id = (unsigned long long) x << 32 | y << 16;

	key.mv_size = sizeof(id);
	key.mv_data = &id;
	data.mv_size = buff.size();
	data.mv_data = &buff[0];
	ER(mdb_txn_begin(env, NULL, 0, &txn), -1);
	qDebug("write raw (x=%d, y=%d)", x, y);
	ER(mdb_put(txn, dbi, &key, &data, 0), -2);		
	ER(mdb_txn_commit(txn), -3);	
	return 0;
}

int ICLayerMdb::getMaxScale()
{
	int s;
	for (s = 1; num_block_x > 1 << (s - 1) || num_block_y > 1 << (s - 1); s++);
	return s - 1;
}

void ICLayerMdb::close()
{
	if (!active)
		return;
	active = false;
	if (read_write) {//if it opens for read, return
		mdb_env_close(env);
		return;
	}
	MDB_txn *txn;
	MDB_val key, data0, data1, data2, data3;
	unsigned long long id;

	key.mv_size = sizeof(id);
	key.mv_data = &id;
	E(mdb_txn_begin(env, NULL, 0, &txn));
	for (int s = 1; num_block_x > 1 << (s-1) || num_block_y > 1 << (s-1); s++)
		for (int y = 0; y < num_block_y; y += 1 << s) 
			for (int x = 0; x < num_block_x; x += 1 << s) {
				int x0 = x, y0 = y, v = 0;
				int x1 = x0 + (1 << (s - 1));
				int y1 = y0 + (1 << (s - 1));
				id = (unsigned long long) x0 << 32 | y0 << 16 | (s - 1);
				if (mdb_get(txn, dbi, &key, &data0) != MDB_SUCCESS)
					data0.mv_data = NULL;
				id = (unsigned long long) x1 << 32 | y0 << 16 | (s - 1);
				if (mdb_get(txn, dbi, &key, &data1) != MDB_SUCCESS)
					data1.mv_data = NULL;
				id = (unsigned long long) x0 << 32 | y1 << 16 | (s - 1);
				if (mdb_get(txn, dbi, &key, &data2) != MDB_SUCCESS)
					data2.mv_data = NULL;
				id = (unsigned long long) x1 << 32 | y1 << 16 | (s - 1);
				if (mdb_get(txn, dbi, &key, &data3) != MDB_SUCCESS)
					data3.mv_data = NULL;
				QImage image0, image1, image2, image3;
				if (data0.mv_data) {
					if (!image0.loadFromData((uchar *)data0.mv_data, (int) data0.mv_size))
						qFatal("image format error, (x=%d,y=%d,s=%d)", x0, y0, s - 1);
					v++;
				}
				else
					qFatal("image miss (x=%d,y=%d,s=%d)", x0, y0, s - 1);
				if (data1.mv_data) {
					if (!image1.loadFromData((uchar *)data1.mv_data, (int) data1.mv_size))
						qFatal("image format error, (x=%d,y=%d,s=%d)", x1, y0, s - 1);
					if (image0.format() != image1.format() || image0.width() != image1.width() || image0.height() != image1.height())
						qFatal("image format or size not same, (x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", x1, y0, s - 1, 
						image1.format(), image1.width(), image1.height());
					v++;
				}
				if (data2.mv_data) {
					if (!image2.loadFromData((uchar *)data2.mv_data, (int) data2.mv_size))
						qFatal("image format error, (x=%d,y=%d,s=%d)", x0, y1, s - 1);
					if (image0.format() != image2.format() || image0.width() != image2.width() || image0.height() != image2.height())
						qFatal("image format or size not same, (x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", x0, y1, s - 1,
						image2.format(), image2.width(), image2.height());
					v++;
				}
				if (data3.mv_data) {
					if (!image3.loadFromData((uchar *)data3.mv_data, (int) data3.mv_size))
						qFatal("image format error, (x=%d,y=%d,s=%d)", x1, y1, s - 1);
					if (image0.format() != image3.format() || image0.width() != image3.width() || image0.height() != image3.height())
						qFatal("image format or size not same, (x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", x1, y1, s - 1,
						image3.format(), image3.width(), image3.height());
					v++;
				}
				if (v == 3)
					qCritical("image miss (x=%d, y=%d, s=%d)", x1, y1, s - 1);
				QImage image(image0.width() * 2, image0.height() * 2, image0.format());
				image.fill(QColor(0, 0, 0));
				QPainter painter(&image);
				painter.drawImage(0, 0, image0);
				if (data1.mv_data)
					painter.drawImage(image0.width(), 0, image1);
				if (data2.mv_data)
					painter.drawImage(0, image0.height(), image2);
				if (data3.mv_data)
					painter.drawImage(image0.width(), image0.height(), image3);
				qDebug("write (x=%d, y=%d, s=%d)", x0, y0, s);
				id = (unsigned long long) x0 << 32 | y0 << 16 | s;
				QImage image_s;
				QByteArray ba;
				QBuffer buffer(&ba);
				image_s = image.scaled(image.width() / 2, image.height() / 2);
				buffer.open(QIODevice::WriteOnly);
				image_s.save(&buffer, "JPG");
				data0.mv_size = ba.size();
				data0.mv_data = ba.data();
				E(mdb_put(txn, dbi, &key, &data0, 0));
			}
	E(mdb_txn_commit(txn));
	MDB_stat mst;
	E(mdb_env_stat(env, &mst));
	mdb_env_close(env);
	qDebug("bracpage=%d, depth=%d, entry=%d, leafpage=%d, ovflpage=%d, psize=%d\n", mst.ms_branch_pages, mst.ms_depth,
		mst.ms_entries, mst.ms_leaf_pages, mst.ms_overflow_pages, mst.ms_psize);
}
#endif

//ICLayerM contain meta info in file header, it is mainly for network file system
/*
File Format
Header: num_block_x(4), num_block_y(4), block_w(4), total image num(4)
Image length Array(total_num): Image0 length(4), Image1 length(4).. 
Image(total_num): Image0 (Image0 length), Image1,...
*/
#define ICLayerM_GENERATE_VERSION 1
class ICLayerM : public ICLayerInterface
{
protected:
	vector<unsigned long long> bias_storage; //pointer image offset from file.begin(), total_num+1
	vector<unsigned long long *> bias;	//point to bias_storage, bias[0] point to scale 0
	ifstream fin;
	fstream fout;
	int x0, y0, s0, idx0;
	struct HeaderStruct {
		char flag[4];
		int num_block_x, num_block_y; //image num in row and col
		int block_w; //image width and height
		int total_num; //total image number
	} head;
	int version;
	QMutex mutex;

protected:
	int compute_bias_num(vector<int> & bias_num);

public:
	ICLayerM(const string & file_, bool _read);
	~ICLayerM();
	int getBlockWidth();
	void getBlockNum(int & bx, int & by);
	void putBlockNumWidth(int bx, int by, int width);
	int addRawImg(const vector<uchar> & buff, int x, int y, int);
	int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved);
	int getMaxScale();
	void close();
	bool is_active() { return fin.is_open() | fout.is_open(); }
};

ICLayerM::ICLayerM(const string & _file, bool _read)
{	
	memset(&head, 0, sizeof(head));
	if (_read) {		
		vector<unsigned> img_len;
		fin.open(_file.c_str(), ios::binary);
		if (!fin.is_open())
			return;
		fin.read((char*)&head, sizeof(head));
		if (strcmp(head.flag, "CID")) {
			qCritical("File is not chip layer image");
			fin.close();
			return;
		}
		version = head.num_block_x >> 24;
		head.num_block_x = head.num_block_x & 0xffffff;
		head.num_block_y = head.num_block_y & 0xffffff;
		vector<int> bias_num;
		int total_num = compute_bias_num(bias_num);
		if (total_num != head.total_num) {
			qCritical("Read file corrupted");
			fin.close();
			throw std::exception("Read file corrupted");
			return;
		}
		img_len.resize(total_num);
		fin.read((char*)& img_len[0], sizeof(unsigned) * total_num);
		if (version >= 1)
			decrypt((char*)&img_len[0], sizeof(unsigned)* head.total_num, 0x87654321);
		bias_storage.resize(total_num + 1);
		bias_storage[0] = sizeof(head) + total_num * sizeof(unsigned);
		for (int i = 0; i < total_num; i++)
			bias_storage[i + 1] = bias_storage[i] + img_len[i];
		bias.resize(bias_num.size());
		for (int i = 0, j = 0; i < bias.size(); j += bias_num[i++]) 
			bias[i] = &bias_storage[j];
	}
	else {
		fout.open(_file.c_str(), fstream::in | fstream::out | fstream::binary | fstream::trunc);
		if (!fout.is_open()) {
			qCritical("open file write error");
			throw std::exception("open file write error");
			return;
		}
		strncpy(head.flag, "CID", 4);
		x0 = -12345;
		y0 = -12345;
		s0 = -54321;
		idx0 = 0;
		version = ICLayerM_GENERATE_VERSION;
	}		
}

int ICLayerM::compute_bias_num(vector<int> & bias_num)
{
	int bx = head.num_block_x;
	int by = head.num_block_y;
	int total_num = 0;
	for (int s = 0; s < 16; s++) {
		bias_num.push_back(bx * by);
		total_num += bx * by;
		bx = (bx == 0) ? 1 : ((bx - 1) >> 1) + 1;
		by = (by == 0) ? 1 : ((by - 1) >> 1) + 1;
		if (bx == 1 && by == 1) {
			bias_num.push_back(1);
			total_num += 1;
			break;
		}		
	}
	return total_num;
}

ICLayerM::~ICLayerM()
{
	close();
}

int ICLayerM::getBlockWidth()
{
	return head.block_w;
}

void ICLayerM::getBlockNum(int & bx, int & by)
{
	bx = head.num_block_x;
	by = head.num_block_y;
}

void ICLayerM::putBlockNumWidth(int bx, int by, int width)
{
	if (head.num_block_x != 0 || head.num_block_y != 0 || head.block_w != 0) {
		qCritical("Set block number twice!");
		return;
	}
	vector<int> bias_num;
	vector<int> img_len;
	
	head.num_block_x = bx;
	head.num_block_y = by;
	head.block_w = width;
	head.total_num = compute_bias_num(bias_num);
	qDebug("ICLayerM putBlockNumWidth bx=%d, by=%d, Total_num=%d", bx, by, head.total_num);
	bias_storage.assign(head.total_num + 1, 0);
	bias_storage[0] = sizeof(head) + head.total_num * sizeof(unsigned);
	bias.resize(bias_num.size());
	for (int i = 0, j = 0; i < bias.size(); j += bias_num[i++]) {
		bias[i] = &bias_storage[j];
		qDebug("bias[%d]=%d", i, j);
	}
	img_len.assign(head.total_num, 0);
	head.num_block_x = version << 24 | bx;
	fout.write((char*)&head, sizeof(head)); //write header
	head.num_block_x = bx;
	fout.write((char*)&img_len[0], sizeof(unsigned) * head.total_num); //write all image len as 0
	
	x0 = 0;
	y0 = 0;
	s0 = 0;
	idx0 = 0;
}

int ICLayerM::addRawImg(const vector<uchar> & buff, int x, int y, int)
{
	if (!fout.is_open()) {
		qCritical("File is not open for write");
		return -2;
	}
		
	qDebug("add image (%d, %d, %d)", x0, y0, s0);
	if (version >= 1) {
		if (buff.size() < 128)
			qFatal("error, file length=%d, too small", buff.size());
		int magic = s0 * head.total_num + y * head.num_block_y + x;
#ifdef ENCRYPT_CHECK
		unsigned char org_buf[128];
		memcpy(org_buf, (char*)&buff[0], 112);
		memcpy(&org_buf[112], (char*)&buff[0] + buff.size() - 16, 16);
#endif
		encrypt((char*)&buff[0], 112, magic);
		encrypt((char*)&buff[0] + buff.size() - 16, 16, magic);
#ifdef ENCRYPT_CHECK	
		char dec_buf[128];
		memcpy(dec_buf, (char*)&buff[0], 112);
		decrypt(dec_buf, 112, magic);
		memcpy(&dec_buf[112], (char*)&buff[0] + buff.size() - 16, 16);
		decrypt((char*)& dec_buf[112], 16, magic);
		if (memcmp(org_buf, dec_buf, 128) != 0)
			qFatal("encrypt error x=%d, y=%d", x, y);
		
#endif
	}
	QMutexLocker locker(&mutex);
	if (x != x0 || y != y0) {
		qCritical("Image must be add in order, expect (%d,%d), receive(%d,%d)", y0, x0, y, x);
		return -1;
	}
	fout.write((char*)&buff[0], buff.size());
	bias_storage[idx0 + 1] = bias_storage[idx0] + buff.size();

	Q_ASSERT(idx0 + 1 < bias_storage.size());
	qDebug("addRawImg x=%d, y=%d, bias[%d]=%lld, len=%d", x, y, idx0, bias_storage[idx0], buff.size());

	idx0++;
	x0 += 1 << s0;
	if (x0 >= head.num_block_x) {
		x0 = 0;
		y0 += 1 << s0;
		if (y0 >= head.num_block_y) {
			y0 = 0;
			s0++;
		}
	}
	return 0;
}

int ICLayerM::getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved)
{
	buff.clear();
	if (!fin.is_open() && !fout.is_open()) {
		qCritical("File is not open for read");
		return -2;
	}
	if (ovr >= bias.size() || x < 0 || x >= head.num_block_x || y < 0 || y >= head.num_block_y)
		return -1;
	
	x = x >> ovr;
	y = y >> ovr;
	int bx = ((head.num_block_x - 1) >> ovr) + 1;
	int idx = y * bx + x;
	int len = bias[ovr][idx + 1] - bias[ovr][idx];

	qDebug("getRawImgByIdx x=%d, y=%d, bias=%lld, len=%d", x, y, bias[ovr][idx], len);

	buff.resize(len + reserved);
	QMutexLocker locker(&mutex);
	if (fin.is_open()) {
		fin.seekg(bias[ovr][idx], ios::beg);
		fin.read((char*)&buff[reserved], len);
		if (version >= 1) {
			if (len < 128)
				qFatal("error, getRawImgByIdx len=%d", len);
			int magic = ovr * head.total_num + (y << ovr) * head.num_block_y + (x << ovr);
			decrypt((char*)&buff[reserved], 112, magic);
			decrypt((char*)&buff[0] + buff.size() - 16, 16, magic);
		}
	}
	else {
		unsigned long long pos = fout.tellp();
		fout.seekg(bias[ovr][idx], ios::beg);
		fout.read((char*)&buff[reserved], len);
		if (version >= 1) {
			if (len < 128)
				qFatal("error, getRawImgByIdx len=%d", len);
			int magic = ovr * head.total_num + (y << ovr) * head.num_block_y + (x << ovr);
			decrypt((char*)&buff[reserved], 112, magic);
			decrypt((char*)&buff[0] + buff.size() - 16, 16, magic);
		}
		fout.seekp(pos);
	}
	return 0;
}

int ICLayerM::getMaxScale()
{
	return (int)bias.size() - 1;
}

void ICLayerM::close()
{
	if (fin.is_open()) {
		fin.close();
		return;
	}
	if (fout.is_open()) {
		if (idx0 != head.num_block_x * head.num_block_y) {
			qCritical("When close, Image is added %d, expected %d", idx0, head.num_block_x * head.num_block_y);
			return;
		}
		if (idx0 == 0)
			return;
		for (int s = 1; head.num_block_x > 1 << (s - 1) || head.num_block_y > 1 << (s - 1); s++)
#if !GENERATE_DB_CONCURRENT
			for (int y = 0; y < head.num_block_y; y += 1 << s)
				for (int x = 0; x < head.num_block_x; x += 1 << s) {
					vector<uchar> buff;
					int x0 = x, y0 = y, v = 0;
					int x1 = x0 + (1 << (s - 1));
					int y1 = y0 + (1 << (s - 1));
					QImage image0, image1, image2, image3;
					if (getRawImgByIdx(buff, x0, y0, s - 1, 0) == 0) {
						if (!image0.loadFromData((uchar *)&buff[0], (int) buff.size()))
							qFatal("image format error, (x=%d,y=%d,s=%d)", x0, y0, s - 1);
						v++;
					}
					else
						qFatal("image miss (x=%d,y=%d,s=%d)", x0, y0, s - 1);
					if (getRawImgByIdx(buff, x1, y0, s - 1, 0) == 0) {
						if (!image1.loadFromData((uchar *)&buff[0], (int) buff.size()))
							qFatal("image format error, (x=%d,y=%d,s=%d)", x1, y0, s - 1);
						if (image0.format() != image1.format() || image0.width() != image1.width() || image0.height() != image1.height())
							qFatal("image format or size not same, (x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", x1, y0, s - 1,
							image1.format(), image1.width(), image1.height());
						v++;
					}
					if (getRawImgByIdx(buff, x0, y1, s - 1, 0) == 0) {
						if (!image2.loadFromData((uchar *)&buff[0], (int) buff.size()))
							qFatal("image format error, (x=%d,y=%d,s=%d)", x0, y1, s - 1);
						if (image0.format() != image2.format() || image0.width() != image2.width() || image0.height() != image2.height())
							qFatal("image format or size not same, (x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", x0, y1, s - 1,
							image2.format(), image2.width(), image2.height());
						v++;
					}
					if (getRawImgByIdx(buff, x1, y1, s - 1, 0) == 0) {
						if (!image3.loadFromData((uchar *)&buff[0], (int) buff.size()))
							qFatal("image format error, (x=%d,y=%d,s=%d)", x1, y1, s - 1);
						if (image0.format() != image3.format() || image0.width() != image3.width() || image0.height() != image3.height())
							qFatal("image format or size not same, (x=%d,y=%d,s=%d),f=%x, w=%d, h=%d", x1, y1, s - 1,
							image3.format(), image3.width(), image3.height());
						v++;
					}
					if (v == 3)
						qFatal("image miss (x=%d, y=%d, s=%d)", x1, y1, s - 1);
					QImage image(image0.width() * 2, image0.height() * 2, image0.format());
					image.fill(QColor(0, 0, 0));
					QPainter painter(&image);
					painter.drawImage(0, 0, image0);
					if (!image1.isNull())
						painter.drawImage(image0.width(), 0, image1);
					if (!image2.isNull())
						painter.drawImage(0, image0.height(), image2);
					if (!image3.isNull())
						painter.drawImage(image0.width(), image0.height(), image3);
					QImage image_s;
					QByteArray ba;
					QBuffer buffer(&ba);
					image_s = image.scaled(image.width() / 2, image.height() / 2, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
					buffer.open(QIODevice::WriteOnly);
					image_s.save(&buffer, "JPG", 85);
					buff.resize(ba.size());
					memcpy(buff.data(), ba.data(), ba.size());
					addRawImg(buff, x0, y0, s);
				}
#else
		{
			vector<ProcessScaleData> scale_sets;
			for (int y = 0; y < head.num_block_y; y += 1 << s)
			for (int x = 0; x < head.num_block_x; x += 1 << s)
				scale_sets.push_back(ProcessScaleData(x, y, s, this));
			
			vector<int> temp_vec = QtConcurrent::blockingMappedReduced<vector<int>, vector<ProcessScaleData> >(scale_sets, scale_image_2x2, scale_image_add,
				QtConcurrent::OrderedReduce | QtConcurrent::SequentialReduce);
			if (temp_vec.size() != scale_sets.size())
				qFatal("scale internal error");
		}
#endif
		if (idx0 != head.total_num)
			qFatal("internal error, fix me");
		//wirte image len
		vector<unsigned> img_len(head.total_num);
		for (unsigned i = 0; i < img_len.size(); i++)
			img_len[i] = bias_storage[i + 1] - bias_storage[i];
		fout.seekp(sizeof(head), ios::beg);
		if (version >= 1) {
			encrypt((char*)&img_len[0], sizeof(unsigned)* head.total_num, 0x87654321);
			fout.write((char*)&img_len[0], sizeof(unsigned)* head.total_num);
			decrypt((char*)&img_len[0], sizeof(unsigned)* head.total_num, 0x87654321);
		} else
			fout.write((char*)&img_len[0], sizeof(unsigned)* head.total_num);
		fout.close();
	}
}

#define CACHE_ENABLE_TH (256 * 1024)
/*
It support multi-thread access when open as read, it use global lock for one layer
IClayerWr is ICLayerInterface + cache, and cache size is max_cache_size
*/
class ICLayerWr : public ICLayerWrInterface
{
public:
	ICLayerWr();
	/*
	Input: file, layer database name
	Input: _read, read or write database
	Input: _cache_enable, only valid when read, if need memory cache,
	0: don't need cache
	1: default cache size
	2: cache size is decided automatically
	other: actual cache size
	Input type: 0 default type
	*/
	ICLayerWr(const string file, bool _read, int _cache_size = 2, int type = 0);
	~ICLayerWr();
	/*
	When call create, ICLayerWr close associated ICLayerInterface and create new ICLayerInterface
	Input: file, layer database name
	Input: _read, read or write database
	Input: _cache_size, only valid when read, if need memory cache,
	0: don't need cache
	1: default cache size
	2: cache size is decided automatically
	other: actual cache size
	Input type: 0 default type
	*/
	void create(const string file, bool _read, int _cache_size = 2, int type = 0);
	int getBlockWidth();
	void getBlockNum(int & bx, int &by);
	int getMaxScale();
	int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved, bool need_cache = true);
	void generateDatabase(GenerateDatabaseParam & gdp);
	void adjust_cache_size(int _delta_cache_size);
	string get_file_name();
	void get_cache_stat(int & _cache_size, QTime & earlest_tm);
	void release_cache(int cache_limit, QTime tm);
	void set_cache_size(int _max_cache_size);
	void close();
	ICLayerInterface * get_iclayer_inf();
	friend class ICLayerZoomWr;
protected:
	struct BkImgMeta{
		unsigned long long id;
		QTime tm;
		BkImgMeta(unsigned long long _id) {
			id = _id;
			tm = QTime::currentTime();
		}
	};
	struct BkImg {
		list <BkImgMeta>::iterator plist;
		unsigned char * data;
		unsigned int len;
	};

	ICLayerInterface * layer;
	map <unsigned long long, BkImg> cache_map;
	list <BkImgMeta> cache_list;
	QMutex mutex;
	int cache_size, max_cache_size; //shall I add set_cache_size to set max_cache_size
	bool read_write;
	string file_name;
};

ICLayerWr::ICLayerWr()
{
	layer = NULL;
	cache_size = 0;
}

ICLayerWr::ICLayerWr(const string file, bool _read, int _cache_size, int type)
{
	layer = NULL;
	cache_size = 0;
	create(file, _read, _cache_size, type);
}

ICLayerWr::~ICLayerWr()
{
	QMutexLocker locker(&mutex);
	close();
}

void ICLayerWr::create(const string file, bool _read, int _cache_size, int type)
{
	QMutexLocker locker(&mutex);
	close();	
	file_name = file;
	qInfo("layer file %s is opened for %s, cache_size=%d", file_name.c_str(), _read ? "read" : "write", _cache_size);	
	read_write = _read;	
	if (!_read) {
		remove(file.c_str());
		max_cache_size = 0;
		if (_cache_size > 2)
			qWarning("Cache enable only work for read");
	} else
		switch (_cache_size) {
		case 0:
			max_cache_size = 0;
			break;
		case 1:
			max_cache_size = 1 << 27; //default 128M
			break;
		case 2:
			if (file[0] == '/' && file[1] == '/' || file[0] == '\\' && file[1] == '\\')
				max_cache_size = 1 << 27;
			else
				max_cache_size = 1 << 23;
			break;
		default:
			if (_cache_size <= CACHE_ENABLE_TH)
				max_cache_size = 0;
			else
				max_cache_size = _cache_size;
		}

	switch (type) {
	case 0:
		layer = new ICLayerM(file, _read);
		break;
#ifdef USB_MDB
	case 1:
		layer = new ICLayerMdb(file, _read);
		break;
#endif
	case 2:
		layer = new ICLayer(file, _read);
		break;
	}	
    int bx, by;
    layer->getBlockNum(bx, by);
    qInfo("img_num_x=%d, img_num_y=%d", bx, by);
}


int ICLayerWr::getBlockWidth()
{
	QMutexLocker locker(&mutex);
	if (layer!=NULL && read_write)
		return layer->getBlockWidth();
	else {
		if (layer==NULL)
			qCritical("layer is NULL when getBlockWidth");
		if (!read_write)
			qCritical("getBlockWidth can't be called when open write");
		return 0;
	}
}

void ICLayerWr::getBlockNum(int & bx, int &by)
{
	QMutexLocker locker(&mutex);
	if (layer != NULL && read_write)
		return layer->getBlockNum(bx, by);
	else {
		if (layer == NULL)
			qCritical("layer is NULL when getBlockNum");
		if (!read_write)
			qCritical("getBlockNum can't be called when open write");
		return;
	}		 
}

int ICLayerWr::getMaxScale()
{
	QMutexLocker locker(&mutex);
	if (layer != NULL && read_write)
		return layer->getMaxScale();
	else {
		if (layer == NULL)
			qCritical("layer is NULL when getMaxScale");
		if (!read_write)
			qCritical("getMaxScale can't be called when open write");
		return -1;
	}
}

int ICLayerWr::getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved, bool need_cache)
{
	QMutexLocker locker(&mutex);
	int ret;
    unsigned long long id = (unsigned long long) x << 32 | y << 16 | ovr;
	if (!read_write) {
		qCritical("getRawImgByIdx can't be called when open write");
		return -5;
	}
	if (max_cache_size > CACHE_ENABLE_TH) {
		map<unsigned long long, BkImg>::iterator pmap;
		pmap = cache_map.find(id);
		if (pmap != cache_map.end()) {
			buff.resize(reserved + pmap->second.len);
			qDebug("get cache image, (%d,%d,%d)", x, y, ovr);
			memcpy(&buff[reserved], pmap->second.data, pmap->second.len);
			cache_list.erase(pmap->second.plist);
			cache_list.push_back(BkImgMeta(id));
			pmap->second.plist = cache_list.end();
			pmap->second.plist--;
			Q_ASSERT(pmap->second.plist->id == id);
			return 0;
		}
	}
    ret = layer->getRawImgByIdx(buff, x, y, ovr, reserved);
	if (ret == 0 && max_cache_size > CACHE_ENABLE_TH && need_cache) { //add to cache
		BkImg img;
		img.data = (unsigned char *)malloc(buff.size() - reserved);
		img.len = (unsigned) buff.size() - reserved;
		if (img.data) {
			memcpy(img.data, &buff[reserved], img.len);
			cache_list.push_back(BkImgMeta(id));
			img.plist = cache_list.end();
			img.plist--;
			cache_size += img.len;
			Q_ASSERT(img.plist->id == id);
			cache_map[id] = img;
		}
		if (cache_size > max_cache_size) {
			locker.unlock();
			release_cache(max_cache_size, cache_list.begin()->tm);
		}
	}
	return ret;
}

void ICLayerWr::get_cache_stat(int & _cache_size, QTime & earlest_tm)
{
	QMutexLocker locker(&mutex);
	if (max_cache_size > CACHE_ENABLE_TH) {
		_cache_size = cache_size;
		if (cache_size != 0)
			earlest_tm = cache_list.begin()->tm;
		else
			earlest_tm = QTime::currentTime();
	}
	else {
		_cache_size = 0;
		earlest_tm = QTime::currentTime();
	}
}

void ICLayerWr::release_cache(int cache_limit, QTime tm)
{
	qDebug("release cache cache_size=%d, limit=%d", cache_size, cache_limit);
	QMutexLocker locker(&mutex);
	for (list<BkImgMeta>::iterator it = cache_list.begin(); it != cache_list.end(); it = cache_list.begin()) {
		if (cache_size <= cache_limit && it->tm >= tm)
			break;
		unsigned long long id = it->id;
		map<unsigned long long, BkImg>::iterator pmap;
		pmap = cache_map.find(id);
		Q_ASSERT(pmap != cache_map.end() && pmap->second.plist == it);
		free(pmap->second.data);
		cache_size -= pmap->second.len;
		cache_map.erase(id);
		cache_list.erase(it);
		Q_ASSERT(cache_size >= 0);
	}
}

void ICLayerWr::generateDatabase(GenerateDatabaseParam & gdp)
{
	string path = gdp.path;
	int from_row = gdp.from_row;
	int to_row = gdp.to_row;
	int from_col = gdp.from_col;
	int to_col = gdp.to_col;
	int _block_num_x = gdp.block_num_x;
	int _block_num_y = gdp.block_num_y;

	QMutexLocker locker(&mutex);
	char file_name[500];
	vector<uchar> buff;
	if (read_write) {
		qCritical("generateDatabase can't be called when open read");
		return;
	}
	if (_block_num_x == -1)
		_block_num_x = to_col - from_col + 1;
	if (_block_num_x != to_col - from_col + 1) {
		qCritical("_block_num_x(%d) != to_col(%d) - from_col(%d) + 1", _block_num_x, to_col, from_col);
		return;
	}
	if (_block_num_y == -1)
		_block_num_y = to_row - from_row + 1;
	if (_block_num_y != to_row - from_row + 1) {
		qCritical("_block_num_y(%d) != to_row(%d) - from_row(%d) + 1", _block_num_y, to_row, from_row);
		return;
	}
	qInfo("ICLayerWr::generateDatabase enter, frow=%d, trow=%d, fcol=%d, tcol=%d, bx=%d, by=%d",
		from_row, to_row, from_col, to_col, _block_num_x, _block_num_y);
	for (int y = from_row; y <= to_row; y++)
		for (int x = from_col; x <= to_col; x++) {
			sprintf(file_name, "%s%d_%d.jpg", path.c_str(), y, x);
			qInfo("read file %s", file_name);
			ifstream infile(file_name, ifstream::binary);
			if (infile.fail())
				qFatal("file %s not exist", file_name);
			infile.seekg(0, infile.end);
			long size = infile.tellg();
			infile.seekg(0);

			buff.resize(size);
			infile.read((char*)&buff[0], size);

			if (y == from_row && x == from_col) {
				QImage bkimg;
				int len = (int)buff.size();
				bkimg.loadFromData((uchar*)&buff[0], len);
				layer->putBlockNumWidth(to_col - from_col + 1, to_row - from_row + 1, bkimg.width());
			}
            layer->addRawImg(buff, x - from_col, y - from_row, 0);
		}
	close();
}

//Release all resource, mutex should be acquired before call this function
void ICLayerWr::close()
{
	if (layer != NULL) {
		delete layer;
		qInfo("layer file %s is closed", file_name.c_str());
	}
		
	layer = NULL;
	Q_ASSERT(cache_list.size() == cache_map.size());
	for (map<unsigned long long, BkImg>::iterator it = cache_map.begin(); it != cache_map.end(); it++) {
		cache_size -= it->second.len;
		free(it->second.data);
	}
	Q_ASSERT(cache_size == 0);
	cache_map.clear();
	cache_list.clear();
	cache_size = 0;	
}

ICLayerInterface * ICLayerWr::get_iclayer_inf()
{
	return layer;
}

void ICLayerWr::set_cache_size(int _max_cache_size)
{
	QMutexLocker locker(&mutex);
	if (_max_cache_size <= CACHE_ENABLE_TH)
		max_cache_size = 0;
	else
		max_cache_size = _max_cache_size;
	if (cache_size > max_cache_size) {
		locker.unlock();
		release_cache(max_cache_size, cache_list.begin()->tm);
	}
}

string ICLayerWr::get_file_name()
{
	QMutexLocker locker(&mutex);
	return file_name;
}

void ICLayerWr::adjust_cache_size(int _delta_cache_size)
{
	QMutexLocker locker(&mutex);
	int _max_cache_size = max_cache_size + _delta_cache_size;
	if (_max_cache_size <= CACHE_ENABLE_TH)
		max_cache_size = 0;
	else
		max_cache_size = _max_cache_size;
	if (cache_size > max_cache_size) {
		locker.unlock();
		release_cache(max_cache_size, cache_list.begin()->tm);
	}
}

class ICLayerZoomWr : public ICLayerWrInterface
{
protected:
	ICLayerWr * layer;
	double zoom_x, zoom_y;
	double offset_x, offset_y;
	int block_num_x, block_num_y;

public:
	ICLayerZoomWr()
	{
		layer = new ICLayerWr();
	}
	/*
	Input: file, layer database name
	Input: _read, read or write database
	Input: _cache_enable, only valid when read, if need memory cache,
	0: don't need cache
	1: default cache size
	2: cache size is decided automatically
	other: actual cache size
	Input type: 0 default type
	*/
	ICLayerZoomWr(const string file, bool _read, int _cache_size = 2, int type = 0, double _zoom_x = 1, double _zoom_y = 1, double _offset_x = 0, double _offset_y = 0)
	{
		zoom_x = _zoom_x;
		zoom_y = _zoom_y;
		layer = new ICLayerWr(file, _read, _cache_size, type);
		offset_x = _offset_x;
		offset_y = _offset_y;
	}
	~ICLayerZoomWr()
	{
		delete layer;
	}
	/*
	When call create, ICLayerZoomWr close associated ICLayerInterface and create new ICLayerInterface
	Input: file, layer database name
	Input: _read, read or write database
	Input: _cache_size, only valid when read, if need memory cache,
	0: don't need cache
	1: default cache size
	2: cache size is decided automatically
	other: actual cache size
	Input type: 0 default type
	zoom: normally it is <=1
	offset_x, offset_y:: it is global image shirt

	*/
	void create(const string file, bool _read, int _cache_size, int type, double _zoom_x, double _zoom_y, double _offset_x, double _offset_y)
	{
		zoom_x = _zoom_x;
		zoom_y = _zoom_y;
		offset_x = _offset_x;
		offset_y = _offset_y;
		layer->create(file, _read, _cache_size, type);
	}

	int getBlockWidth()
	{
		return layer->getBlockWidth();
	}

	void getBlockNum(int & bx, int &by)
	{
		layer->getBlockNum(bx, by);
	}

	int getMaxScale()
	{
		return layer->getMaxScale();
	}

	int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved, bool need_cache = true) 
	{
		return layer->getRawImgByIdx(buff, x, y, ovr, reserved, need_cache);
	}

	void generateDatabase(GenerateDatabaseParam & gdp);
	void adjust_cache_size(int _delta_cache_size) 
	{
		layer->adjust_cache_size(_delta_cache_size);
	}

	string get_file_name()
	{
		return layer->get_file_name();
	}

	void get_cache_stat(int & _cache_size, QTime & earlest_tm) 
	{
		layer->get_cache_stat(_cache_size, earlest_tm);
	}
	void release_cache(int cache_limit, QTime tm) 
	{
		layer->release_cache(cache_limit, tm);
	}
	void set_cache_size(int _max_cache_size) 
	{
		layer->set_cache_size(_max_cache_size);
	}
	void close()
	{
		layer->close();
	}

	ICLayerInterface * get_iclayer_inf()
	{
		return layer->layer;
	}
};

void ICLayerZoomWr::generateDatabase(GenerateDatabaseParam & gdp)
{
	string path = gdp.path;
	int from_row = gdp.from_row;
	int to_row = gdp.to_row;
	int from_col = gdp.from_col;
	int to_col = gdp.to_col;
	int _block_num_x = gdp.block_num_x;
	int _block_num_y = gdp.block_num_y;
	int clip_left = gdp.clip_left;
	int clip_right = gdp.clip_right;
	int clip_up = gdp.clip_up;
	int clip_down = gdp.clip_down;
	float quality = gdp.quality;
	int bundle_x = gdp.bundle_x;
	int bundle_y = gdp.bundle_y;
	int gen_image_width = gdp.gen_image_width;	
	
	char file_name[500];
	vector<uchar> buff;
	qInfo("ICLayerZoomWr::generateDatabase enter, frow=%d, trow=%d, fcol=%d, tcol=%d, bnum_x=%d, bnum_y=%d, zoomx=%f, zoomy=%f, quality=%f",
		from_row, to_row, from_col, to_col, _block_num_x, _block_num_y, zoom_x, zoom_y, quality);
	qInfo("clip_l=%d, clip_r=%d, clip_u=%d, clip_d=%d, bundle_x=%d, bundle_y=%d, gen_width=%d", 
		clip_left, clip_right, clip_up, clip_down, gen_image_width, bundle_x, bundle_y);
	if (zoom_x > 1 || zoom_x <= 0.02 || zoom_y > 1 || zoom_y <= 0.02)
		qFatal("invalid zoom");
	if (quality > 1 || quality < 0)
		qFatal("invalid quality");
	vector<int> quality_param(2);
	quality_param[0] = CV_IMWRITE_JPEG_QUALITY;
	quality_param[1] = quality * 100;
	vector<Mat> row_buf0(to_col - from_col + 1), row_buf1(to_col - from_col + 1);
	double rowz, colz; //Dest image size in origrinal picture
	double y0, y1 = 0; //y1 is row_buf1 bottom line zuobiao in orignal picture
	int width; //it is bundle image's width
	int raw_cols, raw_rows; //it is raw image row - clip_up - clip_down
	Mat matx, maty;
	Mat black_img;
	Mat img2x2;

	for (int y = from_row, yy = 0; yy < _block_num_y || _block_num_y <=0; y += bundle_y) {
		if (y <= to_row) {//read one row bundle image from file			
			for (int x = from_col; x <= to_col; x += bundle_x) {
				Mat raw_img, bundle_img;
				//following compute one bundle imge
				for (int by = 0; by < bundle_y; by++)
				for (int bx = 0; bx < bundle_x; bx++) {					
					if (y + by <= to_row && x + bx <= to_col) { //copy one raw image
						sprintf(file_name, "%s%d_%d.jpg", path.c_str(), y + by, x + bx);
						qInfo("read file %s", file_name);
						ifstream infile(file_name, ifstream::binary);
						if (infile.fail())
							qFatal("file %s not exist", file_name);
						infile.seekg(0, infile.end);
						long size = infile.tellg();
						infile.seekg(0);

						buff.resize(size);
						infile.read((char*)&buff[0], size);

						raw_img = imdecode(Mat(buff), CV_LOAD_IMAGE_UNCHANGED);
						raw_img = raw_img(Rect(clip_left, clip_up, raw_img.cols - clip_left - clip_right,
							raw_img.rows - clip_up - clip_down));
						if (bx == 0 && by == 0) {
							raw_cols = raw_img.cols;
							raw_rows = raw_img.rows;
							bundle_img.create(raw_rows * bundle_y, raw_cols * bundle_x, raw_img.type());
						}
						Rect dst_rect(bx * raw_cols, by * raw_rows, raw_cols, raw_rows);
						raw_img.copyTo(bundle_img(dst_rect));
					}
					else { //copy black image
						Rect dst_rect(bx * raw_cols, by * raw_rows, raw_cols, raw_rows);
						bundle_img(dst_rect) = Scalar::all(0);
					}
				}
#if 0
				char test_name[100];
				sprintf(test_name, "bundle_%d_%d.jpg", x, y);
				imwrite(test_name, bundle_img);
#endif
				row_buf1[(x - from_col) / bundle_x] = bundle_img; //now one bundle_img is ready
				if (y == from_row && x == from_col) {
					rowz = gen_image_width * zoom_y;
					colz = gen_image_width * zoom_x;
					width = bundle_img.cols;
					qInfo("raw_rows=%d, raw_cols=%d, bundle_image's cols=%d, rows=%d, rowz=%d", 
						raw_rows, raw_cols, bundle_img.cols, bundle_img.rows, rowz);
					if (bundle_img.cols < colz || bundle_img.rows < rowz)
						qFatal("bundle image cols(%d) or rows(%d) < rowz(%d)", bundle_img.cols, bundle_img.rows, rowz);
					y0 = -offset_y;
					if (_block_num_x <= 0)
						_block_num_x = (to_col - from_col + 1) * raw_cols / colz + 1;
					if (_block_num_y <= 0)
						_block_num_y = (to_row - from_row + 1) * raw_rows / rowz + 1;
					layer->layer->putBlockNumWidth(_block_num_x, _block_num_y, gen_image_width);
					matx.create(gen_image_width, gen_image_width, CV_32FC1);
					maty.create(gen_image_width, gen_image_width, CV_32FC1);
					black_img.create(bundle_img.size(), bundle_img.type());
					black_img = Scalar::all(0);
					for (int x = 0; x < row_buf0.size(); x++)
						row_buf0[x] = black_img;
					img2x2.create(bundle_img.rows * 2, bundle_img.cols * 2, bundle_img.type());
				}
			}
		}
		else { //file row image with black
			for (int x = 0; x < row_buf1.size(); x++)
				row_buf1[x] = black_img;
		}
		y1 += row_buf1[0].rows;
				
		Mat image, left_img, up_img, leftup_img;
		for (; y0 + rowz < y1 && yy < _block_num_y; y0 += rowz, yy++) { // y0 begin at row_buf0
			//Generate one row at dest database
			int x1 = 0; //x1 is right line zuobiao in original picture
			//if (y == from_row && y0 + rowz < 0 || -offset_x + colz < 0) {
			image = black_img;
			up_img = black_img;
			left_img = black_img;
			leftup_img = black_img;			
			
			for (double x0 = -offset_x, xx = 0; xx < _block_num_x; x0 += colz, xx++) {
				while (x0 + colz >= x1) {
					x1 += width;
					left_img = image;
					leftup_img = up_img;
					if (x1 / width - 1 < row_buf1.size()) {
						if (y == from_row && y0 + rowz < 0) {
							image = black_img;
							up_img = black_img;
						}
						else {
							image = row_buf1[x1 / width - 1];
							up_img = row_buf0[x1 / width - 1];
						}
					}
					else {
						image = black_img;
						up_img = black_img;
					}
				}
				Q_ASSERT(image.size() == left_img.size() && image.size() == leftup_img.size() && image.size() == up_img.size() &&
					image.rows * 2 == img2x2.rows && image.cols * 2 ==img2x2.cols);
				leftup_img.copyTo(img2x2(Rect(0, 0, image.cols, image.rows)));
				up_img.copyTo(img2x2(Rect(image.cols, 0, image.cols, image.rows)));
				left_img.copyTo(img2x2(Rect(0, image.rows, image.cols, image.rows)));
				image.copyTo(img2x2(Rect(image.cols, image.rows, image.cols, image.rows)));
				
				for (int i = 0; i < gen_image_width; i++)
					matx.col(i) = (float)(x0 + zoom_x * i - (x1 - image.cols * 2));
				for (int i = 0; i < gen_image_width; i++)
					maty.row(i) = (float)(y0 + zoom_y * i - (y1 - image.rows * 2));
				qInfo("copy from (lx=%.2f, ly=%.2f), (rx=%.2f, ry=%.2f)", x0, y0,
					x0 + matx.at<float>(matx.rows - 1, matx.cols - 1) - matx.at<float>(0, 0), 
					y0 + maty.at<float>(matx.rows - 1, matx.cols - 1) - maty.at<float>(0, 0));
				Mat dst(gen_image_width, gen_image_width, image.type());
				remap(img2x2, dst, matx, maty, CV_INTER_LINEAR, BORDER_CONSTANT);
#if 0
				if (yy == 0) {
					char test_name[100];
					sprintf(test_name, "img_%d.jpg", (int)xx);
					imwrite(test_name, image);
				}
#endif
				imencode(".jpg", dst, buff, quality_param);
				layer->layer->addRawImg(buff, xx, yy, 0);
			}
		}
		row_buf0.swap(row_buf1);
	}			
	
	layer->close();
}

ICLayerWrInterface * ICLayerWrInterface::create(const string file, bool _read, double _zoom_x, double _zoom_y,
	double _offset_x, double _offset_y, int _cache_size, int dbtype, int wrtype)
{
	if (wrtype == 0) {
		if (_zoom_x == 1 && _zoom_y == 1 && _offset_x == 0 && _offset_y == 0)
			return new ICLayerWr(file, _read, _cache_size, dbtype);
		else
			return new ICLayerZoomWr(file, _read, _cache_size, dbtype, _zoom_x, _zoom_y, _offset_x, _offset_y);
	}
	if (wrtype == 1)
		return new ICLayerZoomWr(file, _read, _cache_size, dbtype, _zoom_x, _zoom_y, _offset_x, _offset_y);
	return NULL;
}

class BkImg : public BkImgInterface
{
protected:
	vector<ICLayerWrInterface *> bk_img_layers;
	vector<string> layer_file;
	string prj_file;
	bool read_write;
	int max_cache_size, poll_count, acc_img_size;
	QMutex mutex, open_mutex; //protect upper global member

protected:
	string full_path_layer_file(string filename);
	void close();

public:
	BkImg(); 
	~BkImg();
	int getBlockWidth();
	void getBlockNum(int & bx, int &by);
	int getMaxScale();
	int getRawImgByIdx(vector<uchar> & buff, int layer, int x, int y, int ovr, unsigned reserved, bool need_cache = true);
	int getLayerNum();
	string getLayerName(int l);
	ICLayerWrInterface * get_layer(int layer);
	void addNewLayer(GenerateDatabaseParam & gdp);
	int open(const string prj, bool _read, int _max_cache_size);
	void set_cache_size(int _max_cache_size);
	void adjust_cache_size(int _delta_cache_size);
	string get_prj_name();
};

BkImg::BkImg()
{
	read_write = true; 
	poll_count = 0;
	acc_img_size = 0;
}

BkImg::~BkImg()
{
	QMutexLocker locker(&mutex);
	if (!prj_file.empty())
		close();
}

//Mutex should be obtained before call this function
string BkImg::full_path_layer_file(string filename)
{
	while (filename[0] == ' ' || filename[0] == '\t')
		filename.erase(filename.begin());
	int len = (int) filename.length() - 1;
	for (; filename[len] == ' ' || filename[len] == '\t' || filename[len] == '\n'; len--);
	filename.erase(len+1);
    string path = prj_file.substr(0, prj_file.find_last_of("\\/")+1);
	return path + filename;
}

int BkImg::getBlockWidth()
{
	QMutexLocker locker(&mutex);
	if (!read_write) {
		qCritical("getBlockWidth not work for write mode");
		return -2;
	}
	if (layer_file.empty())
		return 0;
	else {
		if (bk_img_layers[0] == NULL)
			bk_img_layers[0] = new ICLayerWr(layer_file[0], true, (max_cache_size == 0) ? 0 : 2); 
		locker.unlock();
		return bk_img_layers[0]->getBlockWidth();
	}
		
}

void BkImg::getBlockNum(int & bx, int &by)
{
	QMutexLocker locker(&mutex);
	if (!read_write) {
		qCritical("getBlockNum not work for write mode");
		return;
	}
	if (layer_file.empty()) {
		bx = 0;
		by = 0;
	}
	else {
		if (bk_img_layers[0] == NULL)
			bk_img_layers[0] = new ICLayerWr(layer_file[0], true, (max_cache_size==0) ? 0 : 2);
		locker.unlock();
		bk_img_layers[0]->getBlockNum(bx, by);
	}
}

int BkImg::getMaxScale()
{
	QMutexLocker locker(&mutex);
	if (!read_write) {
		qCritical("getMaxScale not work for write mode");
		return -1;
	}
	if (layer_file.empty())
		return 0;
	else {
		if (bk_img_layers[0] == NULL) 
			bk_img_layers[0] = new ICLayerWr(layer_file[0], true, (max_cache_size == 0) ? 0 : 2);
		locker.unlock();
		return bk_img_layers[0]->getMaxScale();
	}
}

int BkImg::getRawImgByIdx(vector<uchar> & buff, int layer, int x, int y, int ovr, unsigned reserved, bool need_cache)
{	
	QMutexLocker locker(&mutex);
	if (!read_write) {
		qCritical("getRawImgByIdx not work for write mode");
		return -2;
	}
	if (layer >= layer_file.size()) {
		buff.clear();
		qCritical("Layer_request %d is bigger than actual layer %d", layer, layer_file.size() - 1);
		return -1;
	}
	else {
		if (bk_img_layers[layer] == NULL)
			bk_img_layers[layer] = new ICLayerWr(layer_file[layer], true, (max_cache_size == 0) ? 0 : 2);
		locker.unlock();
		int ret = bk_img_layers[layer]->getRawImgByIdx(buff, x, y, ovr, reserved, need_cache);
		locker.relock();
		if (need_cache) {
			poll_count++;
			acc_img_size += (int) buff.size();
		}
		if (poll_count >= 5) {
			poll_count = 0;
			int cache_size = 0;
			int reserve_size;
			int remove_layer;
			QTime earlest_tm = QTime::currentTime();
			for (int i = 0; i < bk_img_layers.size(); i++)				
				if (bk_img_layers[i]) {
					int layer_cache_size;
					QTime tm;
					bk_img_layers[i]->get_cache_stat(layer_cache_size, tm);
					if (earlest_tm > tm) {
						earlest_tm = tm;
						reserve_size = max(layer_cache_size - acc_img_size, acc_img_size); //remove 5 image
						remove_layer = i;
					}
					cache_size += layer_cache_size;
				}
			if (cache_size > max_cache_size) {
				earlest_tm.addSecs(5);
				bk_img_layers[remove_layer]->release_cache(reserve_size, earlest_tm);
			}
			acc_img_size = 0;
		}
		return ret;
	}		
}

int BkImg::getLayerNum()
{
	QMutexLocker locker(&mutex);
	return (int) layer_file.size();
}

string BkImg::getLayerName(int l)
{
	QMutexLocker locker(&mutex);
	if (l < (int)layer_file.size())
		return layer_file[l];
	else
		return string();
}

ICLayerWrInterface * BkImg::get_layer(int layer) {
	QMutexLocker locker(&mutex);
	if (!read_write) {
		qCritical("get_layer not work for write mode");
		return NULL;
	}
		
	if (layer >= layer_file.size()) {
		qCritical("get_layer %d is bigger than actual layer %d", layer, layer_file.size() - 1);
		return NULL;
	}
	else {
		if (bk_img_layers[layer] == NULL)
			bk_img_layers[layer] = new ICLayerWr(layer_file[layer], true, (max_cache_size == 0) ? 0 : 2); 
		return bk_img_layers[layer];
	}
}

void BkImg::addNewLayer(GenerateDatabaseParam & gdp)
{
	string filename = gdp.db_name;
	double offset_x = gdp.offset_x;
	double offset_y = gdp.offset_y;

	QMutexLocker locker(&mutex);
	if (read_write) {
		qCritical("addNewLayer not work for read mode");
		return;
	}
	layer_file.push_back(full_path_layer_file(filename));
	ICLayerWrInterface * new_db = ICLayerWrInterface::create(layer_file.back(), false, gdp.zoom_x, gdp.zoom_y, offset_x, offset_y, 2, gdp.db_type, gdp.wr_type);
	qInfo("Add layer %d=%s", layer_file.size() - 1, layer_file.back().c_str());
	bk_img_layers.push_back(NULL);
	locker.unlock();
	new_db->generateDatabase(gdp);	
}

int BkImg::open(const string prj, bool _read, int _max_cache_size)
{
	QMutexLocker locker(&mutex);
	if (!prj_file.empty())
		close();
	prj_file = prj;
	read_write = _read;
	qInfo("Open BkImg Prj %s for %s, cache_size=%d", prj_file.c_str(), _read ? "read" : "write", _max_cache_size);
	if (read_write) {
		//Read prj file to load each layer file name
		FILE *fp = fopen(prj.c_str(), "rt");
		if (fp == NULL) {
			qCritical("open prj file wrong");
			return -1;
		}
		while (!feof(fp)) {
			char name[200];
			char line[500];
			fgets(line, sizeof(line), fp);
			if (sscanf(line, "layer=%s", name) == 1) {
				string filename(name);
				string full_path_name = full_path_layer_file(filename);
				if (layer_file.empty() || layer_file.back() != full_path_name) {
					layer_file.push_back(full_path_name);
                    qInfo("Prj layer %d=%s", layer_file.size()-1, layer_file.back().c_str());
                    bk_img_layers.push_back(NULL);
                }
			}
		}
		fclose(fp);
		if (_max_cache_size <= CACHE_ENABLE_TH)
			max_cache_size = 0;
		else
			max_cache_size = _max_cache_size;
	}
	else
		max_cache_size = 0;
	return 0;
}

//mutex should be acquired before call close()
void BkImg::close()
{
	qInfo("close BkImg %s", prj_file.c_str());
	for (unsigned i = 0; i<bk_img_layers.size(); i++)
		if (bk_img_layers[i])
			delete bk_img_layers[i];
	if (!read_write) {
		//write prj file
		FILE *fp = fopen(prj_file.c_str(), "w");
		for (int i = 0; i < layer_file.size(); i++)
			fprintf(fp, "layer=%s\n", layer_file[i].substr(layer_file[i].find_last_of("\\/") + 1).c_str());
		fclose(fp);
	}
	bk_img_layers.clear();
	layer_file.clear();
	prj_file.clear();
	read_write = true;
}

void BkImg::set_cache_size(int _max_cache_size)
{
	QMutexLocker locker(&mutex);
	if (!read_write) {
		qCritical("set_cache_size not work for write mode");
		return;
	}
	if (_max_cache_size <= CACHE_ENABLE_TH) {
		max_cache_size = 0;
		for (int i = 0; i < bk_img_layers.size(); i++)
			if (bk_img_layers[i])
				bk_img_layers[i]->set_cache_size(0);
	}
	else
		max_cache_size = _max_cache_size;
}

void BkImg::adjust_cache_size(int _delta_cache_size)
{
	QMutexLocker locker(&mutex);
	int _max_cache_size = max_cache_size + _delta_cache_size;
	if (!read_write) {
		qCritical("set_cache_size not work for write mode");
		return;
	}
	if (_max_cache_size <= CACHE_ENABLE_TH) {
		max_cache_size = 0;
		for (int i = 0; i < bk_img_layers.size(); i++)
			if (bk_img_layers[i])
				bk_img_layers[i]->set_cache_size(0);
	}
	else
		max_cache_size = _max_cache_size;
}

string BkImg::get_prj_name()
{
	QMutexLocker locker(&mutex);
	return prj_file;
}

BkImgInterface * BkImgInterface::create_BkImgDB()
{
	return new BkImg();
}

QSharedPointer<BkImgInterface> BkImgRoMgr::open(const string prj, int _cache_size)
{
    QSharedPointer<BkImgInterface> ret;
	if (_cache_size < 0)
		_cache_size = 0;
    QMutexLocker locker(&mutex);
    map<string, QWeakPointer<BkImgInterface> >::iterator bk_img_iter = bk_imgs_opened.find(prj);
    if (bk_img_iter != bk_imgs_opened.end()) { //already opened
        ret = bk_img_iter->second.toStrongRef();
		if (!ret.isNull()) {
            qInfo("Prj %s is already opened", prj.c_str());
			ret->adjust_cache_size(_cache_size);
			return ret;
		}
		bk_imgs_opened.erase(prj); //it is already released, delete it in map
    }
    ret = QSharedPointer<BkImgInterface>(BkImgInterface::create_BkImgDB());
    qInfo("Now open %s", prj.c_str());
	int rst = ret->open(prj, true, _cache_size);
	if (rst == 0) {
		bk_imgs_opened[prj] = ret.toWeakRef();
		return ret;
	}
	else {
		ret.reset();
		return ret;
	}
		
}
