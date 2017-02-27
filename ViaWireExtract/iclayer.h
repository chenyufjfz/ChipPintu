#ifndef ICLAYER_H
#define ICLAYER_H

#include <vector>
#include <QImage>
#include <string>
#include <map>
#include <list>
#include <QTime>
#include <QMutex>
#include <QWeakPointer>
#include <QSharedPointer>
using namespace std;

class ICLayerInterface
{
public:    
    virtual int getBlockWidth() =0;
	virtual void getBlockNum(int & bx, int &by) = 0;
	virtual int getMaxScale() = 0;
    virtual int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved) =0;
	virtual void putBlockNumWidth(int bx, int by, int width) = 0;	
	virtual int addRawImg(vector<uchar> & buff, int x, int y, int reserved) = 0;
	virtual void close() = 0;
	virtual bool is_active() = 0;
    virtual ~ICLayerInterface() {}
};

/*
It support multi-thread access when open as read, it use global lock for one layer
IClayerWr has cache, and cache size is max_cache_size
*/
class ICLayerWr
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
	Input: _cache_enable, only valid when read, if need memory cache, 
			0: don't need cache
			1: default cache size
			2: cache size is decided automatically
			other: actual cache size
	Input type: 0 default type
	*/
	void create(const string file, bool _read, int _cache_size = 2, int type = 0);
	//getBlockWidth can be called when read database
    int getBlockWidth();
	//getBlockNum can be called when read database
	void getBlockNum(int & bx, int &by);
	//getMaxScale can be called when read database
	int getMaxScale();
	/*
	getRawImgByIdx can be called when read database
	Output buff, Image encoded raw data
	Input x, y, ovr, for raw image, ovr=0, for up_scale image, ovr=scale.
		ovr =1, x=0, 2, 4..., y=0, 2, 4
		ovr =2, x=0, 4, 8..., y=0, 4, 8
	Input: reserved, it is reserved space at buff.begin
	Input: need_cache, if true, put raw image into cache for next read fast
	*/
	int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved, bool need_cache=true);
	/*
	generateDatabase can only be called when write database, it call close() after finish generation 
	Input, path, filename prefix for source image file
	Input, from_row, to_row, from_col, to_col, used to generate file name postfix 
	*/
	void generateDatabase(const string path, int from_row, int to_row, int from_col, int to_col);		
	/*Only be called when open for read
	Input: _delta_cache_size
	*/
	void adjust_cache_size(int _delta_cache_size);
	string get_file_name();
	friend class BkImg;
protected:
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
	/*Only be called when open for read
	Input: max_cache_size
	*/
	void set_cache_size(int _max_cache_size);
	void close();
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

/* TODO need close()?
It support multi-thread access when open as read, each layer has its own lock, 
Typical call use
1 Write use case
img_db = BkImgDBInterface::create_BkImgDB();
img_db->open("chip.prj", false);
img_db->addNewLayer("f:/layer1/Name_", 1, 60, 1, 75);
img_db->addNewLayer("f:/layer2/Name_", 1, 60, 1, 75);
delete img_db
2 Read use case
img_db = BkImgDBInterface::create_BkImgDB();
img_db->open("chip.prj", true, cache_size);
img_db->getBlockWidth();
img_db->getBlockNum(bx, by);
img_db->getRawImgByIdx(buff, l, x, y, s, 0);
img_db->adjust_cache_size(delta_size);
img_db->getRawImgByIdx(buff, l, x, y, s, 0);
delete img_db
3 Read by layer
img_db = BkImgDBInterface::create_BkImgDB();
img_db->open("chip.prj", true, cache_size);
ICLayerWr * ic_layer = get_layer(1);
ic_layer->getRawImgByIdx(buff, x, y, s, 0);
ic_layer->adjust_cache_size(delta_size);
ic_layer->getRawImgByIdx(buff, x, y, s, 0);
delete img_db

BkImgInterface has a variable named max_cache_size, sum of all ICLayerWr's cache_size is lower than max_cache_size.
If user call getRawImgByIdx, both BkImgInterface's max_cache_size and ICLayerWr's cache_size take effect, 
 since BkImg create ICLayerWr with cache_size big enough, so normally BkImgInterface's max_cache_size take main effect.
If user call get_layer() and then call ICLayerWr->getRawImgByIdx, only ICLayerWr's cache_size take effect.
*/
class BkImgInterface
{
public:
    static BkImgInterface * create_BkImgDB();
	//Only be called when open for read
	virtual int getBlockWidth() = 0;
	//Only be called when open for read
	virtual void getBlockNum(int & bx, int &by) = 0;
	//getMaxScale can be called when read database
	virtual int getMaxScale() = 0;
	/*Only be called when open for read
	Output buff, Image encoded raw data
	Input x, y, ovr, for raw image, ovr = 0, for up_scale image, ovr = scale.
	ovr = 1, x = 0, 2, 4..., y = 0, 2, 4
	ovr = 2, x = 0, 4, 8..., y = 0, 4, 8
	Input: reserved, it is reserved space at buff.begin
	Input: need_cache, if true, put raw image into cache for next read fast*/
	virtual int getRawImgByIdx(vector<uchar> & buff, int layer, int x, int y, int ovr, unsigned reserved, bool need_cache=true) = 0;
	virtual int getLayerNum() = 0;
	/*Only be called when open for write
	If fail, return NULL
	*/
	virtual ICLayerWr * get_layer(int layer) = 0;
	/*Only be called when open for write
	Input, filename, layer database filename 
	Input, path, filename prefix for source image file
	Input, from_row, to_row, from_col, to_col, used to generate file name postfix
	*/
	virtual void addNewLayer(const string filename, const string path, int from_row, int to_row, int from_col, int to_col) = 0;
	virtual int open(const string prj, bool _read, int _max_cache_size=0) = 0;
	/*Only be called when open for read
	Input: _delta_cache_size
	*/
	virtual void adjust_cache_size(int _delta_cache_size) = 0;
	virtual string get_prj_name() = 0;
    virtual ~BkImgInterface() {}
};

/*
With BkImgRoMgr user don't call BkImgInterface.open directly and same prj's BkImgInterface is shared by different user.
Use case 1
1 QSharedPointer<BkImgInterface> sp = BkImgRoMgr.open(prj, cache_size);
2 sp->getBlockWidth();
3 sp->getBlockNum(bx, by);
4 sp->getRawImgByIdx(buff, l, x, y, s, 0);
5 sp->adjust_cache_size(-cache_size); //release cache before killed
6 sp is killed and so BkImgInterface is killed
*/
class BkImgRoMgr
{
protected:
    map<string, QWeakPointer<BkImgInterface> > bk_imgs_opened;
    QMutex mutex;
public:
	/* User call open to get BkImgInterface, if user doesn't need, user don't need to close, 
	   Delete BkImgInterface is managed by QSharedPointer. 
	   But user should call BkImgInterface->adjust_cache_size to release cache size. 
	*/
    QSharedPointer<BkImgInterface> open(const string prj, int _cache_size=0);
};

#endif // ICLAYER_H
