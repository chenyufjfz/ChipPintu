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
Each raw image process
1 clip, clip_left. clip_right, clip_up, clip_down
  clip_image.cols = raw_image.cols - clip_left - clip_right,
  clip_image.rows = new_image.rows - clip_up - clip_down
2 bundle, bundle_x, bundle_y
  bundle_image.cols = (raw_image.cols - clip_left - clip_right) * bundle_x
  bundle_image.rows = (raw_image.cols - clip_up - clip_down) * bundle_y
3 zoom
4 cut to gen_image_width
*/
class GenerateDatabaseParam {
public:
	string db_name; //destination db file name
	string path; //source image path
	int from_row, to_row, from_col, to_col; //the filename range
	int block_num_x; //final block num x
	int block_num_y; //final block num y
	int clip_left; //clip raw image left before zoom, alwasy > 0
	int clip_right;//clip raw image right before zoom, alwasy > 0
	int clip_up;//clip raw image up before zoom, alwasy > 0
	int clip_down; //clip raw image down before zoom, alwasy > 0
	int bundle_x, bundle_y; //it treat image as bundle
	int gen_image_width;
	int db_type; //normally it is 0
	int wr_type; //if it is 0, choose ICLayerWr, if it is 1 choose ICLayerZoomWr
	float quality;
	double zoom; // zoom is destination image / source image, always <=1, >0
	double zoom_x, zoom_y;
	double offset_x, offset_y;
	GenerateDatabaseParam() {
		block_num_x = 0;
		block_num_y = 0; 
		quality = 0.7f;
		clip_left = 0;
		clip_right = 0;
		clip_up = 0;
		clip_down = 0;
		gen_image_width = 1024;
		zoom = 1;
		zoom_x = 1;
		zoom_y = 1;
		offset_x = 0;
		offset_y = 0;
		bundle_x = 1;
		bundle_y = 1;
		db_type = 0;
		wr_type = 0;
	}
};
class ICLayerWrInterface
{
public:
	/*Input: file, it is layername
	  Input: _read, 
	  Input: _cache_size,
		  0: don't need cache
		  1: default cache size
		  2: cache size is decided automatically
		  other: actual cache size
	  Input: dbtype, normally it is 0
	  Input: wrtype, normally it is 0
	  Input: _zoom, same as GenerateDatabaseParam zoom
	  Input: _offset_x, _offset_y, same as GenerateDatabaseParam _offset_x, _offset_y
	*/
	static ICLayerWrInterface * create(const string file, bool _read, double _zoom_x, double _zoom_y, double _offset_x, double _offset_y,
		int _cache_size, int dbtype, int wrtype);
	//getBlockWidth can be called when read database
	virtual int getBlockWidth() = 0;
	//getBlockNum can be called when read database
	virtual void getBlockNum(int & bx, int &by) = 0;
	//getMaxScale can be called when read database
	virtual int getMaxScale() = 0;
	/*
	getRawImgByIdx can be called when read database
	Output buff, Image encoded raw data
	Input x, y, ovr, for raw image, ovr=0, for up_scale image, ovr=scale.
	ovr =1, x=0, 2, 4..., y=0, 2, 4
	ovr =2, x=0, 4, 8..., y=0, 4, 8
	Input: reserved, it is reserved space at buff.begin
	Input: need_cache, if true, put raw image into cache for next read fast
	*/
	virtual int getRawImgByIdx(vector<uchar> & buff, int x, int y, int ovr, unsigned reserved, bool need_cache = true) = 0;
	/*
	generateDatabase can only be called when write database, it call close() after finish generation
	Input, path, filename prefix for source image file
	Input, from_row, to_row, from_col, to_col, used to generate file name postfix
	*/
	virtual void generateDatabase(GenerateDatabaseParam & gdp) = 0;
	/*Only be called when open for read
	Input: _delta_cache_size
	*/
	virtual void adjust_cache_size(int _delta_cache_size) = 0;
	/*
	Return file name of this layer
	*/
	virtual string get_file_name() = 0;
	/*
	get_cache_stat can only be called when read database, better not called from user directly
	Output _cache_size, count in bytes
	OUtput earlest_tm, earlest cache image time
	*/
	virtual void get_cache_stat(int & _cache_size, QTime & earlest_tm) = 0;
	/*
	release cache until cache_size is lower than cache_limit and earlest_tm is later than tm
	release_cache can only be called when read database, better not called from user directly
	Input cache_limit, release cache image, so cache_size is lower than cache_limit
	Input tm, release cache image before tm
	*/
	virtual void release_cache(int cache_limit, QTime tm) = 0;
	/*Only be called when open for read, better not called from user directly
	Input: max_cache_size
	*/
	virtual void set_cache_size(int _max_cache_size) = 0;
	/*
	close is manged by BkImgInterface, better not called from user directly
	*/
	virtual void close() = 0;
	virtual ~ICLayerWrInterface() {}
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
img_db->adjust_cache_size(-delta_size);
delete img_db
3 Read by layer
img_db = BkImgDBInterface::create_BkImgDB();
img_db->open("chip.prj", true, cache_size);
ICLayerWrInterface * ic_layer = get_layer(1);
ic_layer->getRawImgByIdx(buff, x, y, s, 0);
ic_layer->adjust_cache_size(delta_size);
ic_layer->getRawImgByIdx(buff, x, y, s, 0);
ic_layer->adjust_cache_size(-delta_size);
delete img_db

BkImgInterface has a variable named max_cache_size, sum of all ICLayerWrInterface's cache_size is lower than max_cache_size.
If user call getRawImgByIdx, both BkImgInterface's max_cache_size and ICLayerWrInterface's cache_size take effect, 
 since BkImg create ICLayerWrInterface with cache_size big enough, so normally BkImgInterface's max_cache_size take main effect.
If user call get_layer() and then call ICLayerWrInterface->getRawImgByIdx, only ICLayerWrInterface's cache_size take effect.
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
	/*input layer l, return layername*/
	virtual string getLayerName(int l) = 0;
	/*Only be called when open for write
	If fail, return NULL
	*/
	virtual ICLayerWrInterface * get_layer(int layer) = 0;
	/*Only be called when open for write
	Input, filename, layer database filename 
	Input, path, filename prefix for source image file
	Input, from_row, to_row, from_col, to_col, used to generate file name postfix
	Input, by, by, used to set block_x_num and block_y_num
	Input, quality, used to set image quality
	*/
	virtual void addNewLayer(GenerateDatabaseParam & gdp) = 0;
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
