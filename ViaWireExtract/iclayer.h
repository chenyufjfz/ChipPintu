#ifndef ICLAYER_H
#define ICLAYER_H

#include <vector>
#include <QImage>
#include <string>
#include <map>
#include <list>
#include <QTime>
using namespace std;

class ICLayerInterface
{
public:    
    virtual int getBlockWidth() =0;
	virtual void getBlockNum(int & bx, int &by) = 0;
    virtual int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved) =0;
	virtual void putBlockNumWidth(int bx, int by, int width) = 0;
	virtual int addRawImg(vector<uchar> & buff, int x, int y, int reserved) = 0;
	virtual void close() = 0;
	virtual bool is_active() = 0;
    virtual ~ICLayerInterface() {}
};

class ICLayerWr
{
public:
	ICLayerWr();
	/*
	Input: file, layer database name
	Input: _read, read or write database
	Input: _cache_enable, only valid when read, if need memory cache, 
			0: don't need cache
			1: need cache
			2: cache is decided automatically
	Input type: 0 default type
	*/
    ICLayerWr(string file, bool _read, int _cache_enable = 2, int type = 0);
    ~ICLayerWr();
	/*
	When call create, ICLayerWr close associated ICLayerInterface and create new ICLayerInterface
	Input: file, layer database name
	Input: _read, read or write database
	Input: _cache_enable, only valid when read, if need memory cache, 
			0: don't need cache
			1: need cache
			2: cache is decided automatically
	Input type: 0 default type
	*/
	void create(string file, bool _read, int _cache_enable = 2, int type = 0);
	//getBlockWidth can be called when read database
    int getBlockWidth();
	//getBlockNum can be called when read database
	void getBlockNum(int & bx, int &by);
	/*
	getRawImgByIdx can be called when read database
	Output buff, Image encoded raw data
	Input x, y, ovr, for raw image, ovr=0, for up_scale image, ovr=scale.
		ovr =1, x=0, 2, 4..., y=0, 2, 4
		ovr =2, x=0, 4, 8..., y=0, 4, 8
	Input: reserved, it is reserved space at buff.begin
	*/
    int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved);
	/*
	get_cache_stat can only be called when read database
	Output _cache_size, count in bytes
	OUtput earlest_tm, earlest cache image time
	*/
	void get_cache_stat(int & _cache_size, QTime & earlest_tm);
	/*
	release cache until cache_size is lower than cache_limit and earlest_tm is later than tm
	release_cache can only be called when read database
	Input cache_limit, release cache image, so cache_size is lower than cache_limit
	Input tm, release cache image before tm
	*/
	void release_cache(int cache_limit, QTime tm);
	/*
	generateDatabase can only be called when write database, it call close() after finish generation 
	Input, path, filename prefix for source image file
	Input, from_row, to_row, from_col, to_col, used to generate file name postfix 
	*/
	void generateDatabase(string path, int from_row, int to_row, int from_col, int to_col);	
	/*Only be called when open for read
	Input: max_cache_size
	*/
	void set_cache_size(int _max_cache_size);
	
protected:
	struct BkImgMeta{
		unsigned long long id;
		QTime tm;
		BkImgMeta(int _id) {
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
	int cache_size, max_cache_size; //shall I add set_cache_size to set max_cache_size
	bool cache_enable;
	bool read_write;
protected:
	void close();
};

/*
Typical call use
1 Write use case
img_db = BkImgDBInterface::create_BkImgDB();
img_db->open("chip.prj", false);
img_db->addNewLayer("f:/layer1/Name_", 1, 60, 1, 75);
img_db->addNewLayer("f:/layer2/Name_", 1, 60, 1, 75);
img_db->close();
2 Read use case
img_db = BkImgDBInterface::create_BkImgDB();
img_db->open("chip.prj", true);
img_db->getBlockWidth();
img_db->getBlockNum(bx, by);
img_db->getRawImgByIdx(buff, l, x, y, s, 0);
img_db->close();
3 Read by layer
img_db = BkImgDBInterface::create_BkImgDB();
img_db->open("chip.prj", true);
ICLayerWr * ic_layer = get_layer(1);
ic_layer->getRawImgByIdx(buff, x, y, s, 0);
img_db->close();
*/
class BkImgDBInterface
{
public:
	static BkImgDBInterface * create_BkImgDB();
	//Only be called when open for read
	virtual int getBlockWidth() = 0;
	//Only be called when open for read
	virtual void getBlockNum(int & bx, int &by) = 0;
	/*Only be called when open for read
		Output buff, Image encoded raw data
		Input x, y, ovr, for raw image, ovr = 0, for up_scale image, ovr = scale.
		ovr = 1, x = 0, 2, 4..., y = 0, 2, 4
		ovr = 2, x = 0, 4, 8..., y = 0, 4, 8
		Input: reserved, it is reserved space at buff.begin*/
	virtual int getRawImgByIdx(vector<uchar> & buff, int layer, int x, int y, int ovr, unsigned reserved) = 0;
	virtual int getLayerNum() = 0;
	//Only be called when open for write
	virtual ICLayerWr * get_layer(int layer) = 0;
	/*Only be called when open for write
	Input, filename, layer database filename 
	Input, path, filename prefix for source image file
	Input, from_row, to_row, from_col, to_col, used to generate file name postfix
	*/
	virtual void addNewLayer(string filename, string path, int from_row, int to_row, int from_col, int to_col) = 0;
	virtual int open(string prj, bool _read) = 0;
	virtual void close() = 0;	
	/*Only be called when open for read
	Input: max_cache_size
	*/
	virtual void set_cache_size(int _max_cache_size) = 0;
	virtual ~BkImgDBInterface() {}
};
#endif // ICLAYER_H
