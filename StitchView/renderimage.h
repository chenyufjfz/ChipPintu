#ifndef RENDERIMAGE_H
#define RENDERIMAGE_H
#include "featext.h"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <QImage>
#include <map>
#include <list>
#include <QMutexLocker>

#define ABS_LAYER 255

enum {
	MERGE_BY_DRAW_ORDER,
	MERGE_MULTI_SRC
};

struct TurnPoint {
	double sp, sz; //src origin point and zoom
	double dp, dz; //dst origin point and zoom
	TurnPoint() {

	}
	TurnPoint(double _sp, double _dp) {
		sp = _sp;
		dp = _dp;
		sz = 1;
		dz = 1;
	}
};

class MapXY {
protected:
	double beta; //rotate angle
	int merge_method; //for picture merge, it should not be here, but there is no good place to put it
	int merge_pt_distance, max_pt_error;
	double z0x, z0y;
	double max_slope;

public:
	virtual ~MapXY() {}
	virtual void copy_base(MapXY & b) = 0;

	static MapXY * create_mapxy(int method = -1);
	void read_file(const FileNode& node) {
		beta = (double)node["beta"];
		z0x = (double)node["z0x"];
		z0y = (double)node["z0y"];
		merge_method = (int)node["merge_method"];
		merge_pt_distance = (int)node["merge_pt_distance"];
		max_pt_error = (int)node["max_pt_error"];
		max_slope = (double)node["max_slope"];
		copy_base(*this);
	}

	void write_file(FileStorage& fs) const {
		fs << "{" << "beta" << beta;
		fs << "z0x" << z0x;
		fs << "z0y" << z0y;
		fs << "merge_method" << merge_method;
		fs << "merge_pt_distance" << merge_pt_distance;
		fs << "max_pt_error" << max_pt_error;
		fs << "max_slope" << max_slope << "}";
	}

	int get_max_pt_error() const {
		return max_pt_error;
	}

	void set_max_pt_error(int _max_pt_error) {
		max_pt_error = _max_pt_error;
	}

	int get_merge_pt_distance() const {
		return merge_pt_distance;
	}

	void set_merge_pt_distance(int _merge_pt_distance) {
		merge_pt_distance = _merge_pt_distance;
	}

	int get_merge_method() const {
		return merge_method;
	}

	void set_merge_method(int _merge_method) {
		merge_method = _merge_method;
	}

	double get_max_slope() const {
		return max_slope;
	}

	void set_max_slope(double _max_slope) {
		if (_max_slope > 0)
			max_slope = _max_slope;
	}

	virtual MapXY * clone() const = 0;
	virtual void set_original() = 0;
	virtual bool is_original() const = 0;
	virtual void set_default_zoomx(double _sz) = 0;
	virtual double get_default_zoomx() const = 0;
	virtual void set_default_zoomy(double _sz) = 0;
	virtual double get_default_zoomy() const = 0;
	virtual void set_beta(double _beta) = 0;
	virtual double get_beta() const = 0;
	virtual Point2d src2dst(Point2d src) const = 0;
	virtual Point2d dst2src(Point2d dst) const = 0;
	//nail.first is src, nail.second is dst
	virtual double recompute(const vector<pair<Point, Point> > & nail) = 0;
};

class MapXY0 : public MapXY {
protected:	
	double cos_beta, sin_beta;
	vector<TurnPoint> tx, ty; //fold line
	double recompute_turn_point(vector<TurnPoint> & tp, vector<pair<int, int> > & nxy, double & z);
	void recompute_tp(vector<TurnPoint> & tp, double default_zoom);

public:
	MapXY0() {
		set_original();
	}
	
	MapXY * clone() const {
		MapXY0 * a = new MapXY0();
		*a = *this;
		return a;
	}

	void copy_base(MapXY & b)
	{
		beta = b.get_beta();
		merge_method = b.get_merge_method();
		merge_pt_distance = b.get_merge_pt_distance();
		max_pt_error = b.get_max_pt_error();
		z0x = b.get_default_zoomx();
		z0y = b.get_default_zoomy();
		max_slope = b.get_max_slope();
		cos_beta = cos(beta);
		sin_beta = sin(beta);
	}

	void set_original();

	bool is_original() const;

	double get_default_zoomx() const {
		return z0x;
	}

	void set_default_zoomx(double _sz) {
		CV_Assert(_sz != 0);
		if (tx.size() <= 1)
			z0x = _sz;
	}

	double get_default_zoomy() const {
		return z0y;
	}

	void set_default_zoomy(double _sz) {
		CV_Assert(_sz != 0);
		if (ty.size() <= 1)
			z0y = _sz;
	}

	void set_beta(double _beta) {
		beta = _beta;
		cos_beta = cos(beta);
		sin_beta = sin(beta);
	}

	double get_beta() const {
		return beta;
	}

	Point2d mid2dst(Point2d mid) const;

	Point2d dst2mid(Point2d dst) const;

    Point2d src2mid(Point2d src) const {
        return Point2d(cos_beta * src.x + sin_beta * src.y, - sin_beta * src.x + cos_beta * src.y);
    }

    Point2d mid2src(Point2d mid) const {
        return Point2d(cos_beta * mid.x - sin_beta * mid.y, sin_beta * mid.x + cos_beta * mid.y);
    }

	Point2d src2dst(Point2d src) const {
		return mid2dst(src2mid(src));
	}

	Point2d dst2src(Point2d dst) const {
		return mid2src(dst2mid(dst));
	}

	/*return 0 means success*/
	double recompute(const vector<pair<Point, Point> > & nail);
};

class MapXY1 : public MapXY {
protected:
	double cos_beta, sin_beta;
	vector<pair<Point, Point> > ns;
	Mat_<double> trans;
	Mat_<Vec2d> s, d; //vec0 is y, vec1 is x
	void cluster(vector<pair<int, int> > & nxy);

public:
	MapXY1() {
		set_original();
	}

	MapXY * clone() const {
		MapXY1 * a = new MapXY1();
		*a = *this;
		return a;
	}

	void copy_base(MapXY & b)
	{
		beta = b.get_beta();
		merge_method = b.get_merge_method();
		merge_pt_distance = b.get_merge_pt_distance();
		max_pt_error = b.get_max_pt_error();
		z0x = b.get_default_zoomx();
		z0y = b.get_default_zoomy();
		max_slope = b.get_max_slope();
		cos_beta = cos(beta);
		sin_beta = sin(beta);
	}

	void set_original();

	bool is_original() const;

	double get_default_zoomx() const {
		return z0x;
	}

	void set_default_zoomx(double _sz) {
		z0x = _sz;
	}

	double get_default_zoomy() const {
		return z0y;
	}

	void set_default_zoomy(double _sz) {
		z0y = _sz;
	}

	void set_beta(double _beta) {
		beta = _beta;
		cos_beta = cos(beta);
		sin_beta = sin(beta);
	}

	double get_beta() const {
		return beta;
	}

	Point2d src2dst(Point2d src) const;

	Point2d dst2src(Point2d dst) const;

	double recompute(const vector<pair<Point, Point> > & nail);
};

void write(FileStorage& fs, const std::string&, const MapXY & x);
void read(const FileNode& node, MapXY& x, const MapXY& default_value);

typedef unsigned long long MapID;
#define INVALID_MAP_ID 0xffffffffffffffff
#define MAPID_L(m) ((unsigned)((m) >> 32 & 0xff))
#define MAPID_X(m) ((unsigned)((m) & 0xffff))
#define MAPID_Y(m) ((unsigned)((m) >> 16 & 0xffff))
#define MAPID(l, x, y) ((unsigned long long)(l) << 32 | (y) << 16 | (x))
Point TOPOINT(QPoint q);
QPoint TOQPOINT(Point p);

Point find_src_map(const ConfigPara & cpara, const Point & p, const Size & wh, int method);

class PremapImg {
public:
	list <MapID>::iterator plist;
	Mat m;
	PremapImg() {}
	PremapImg(const Mat _m, list <MapID>::iterator _plist) {
		m = _m;
		plist = _plist;
	}
};

class PostmapImg {
public:
	list <MapID>::iterator plist;
	QImage data;
	PostmapImg() {}
	PostmapImg(list <MapID>::iterator _plist, const QImage & _data) {
		plist = _plist;
		data = _data;
	}
};

enum {
	ALREADY_EXIST,
	ALREADY_FETCH,
	NOT_FETCH
};

class PreMapCache {
protected:
	map <MapID, PremapImg> m; //store Premap image, its number is big
	list<MapID> l; //premap id is src file name
	QMutex mutex; //protect pre_map and premap_list
	int size;

public:
	void insert(MapID id, const Mat & _m) {
		map <MapID, PremapImg>::iterator map_it;
		list <MapID>::iterator plist;
		QMutexLocker locker(&mutex);
		if ((map_it = m.find(id)) ==  m.end()) { // not found
			l.push_back(id);	//push it to cache_list tail
			plist = l.end();
			plist--;
			CV_Assert(*(plist) == id);
			m[id] = PremapImg(_m, plist);
		}
		else {
			if (!map_it->second.m.empty())
				l.erase(map_it->second.plist);
			l.push_back(id);	//repush it to cache_list tail
			plist = l.end();
			plist--;
			CV_Assert(*(plist) == id);
			m[id] = PremapImg(_m, plist);
		}
	}

	void repush(MapID id) {
		map <MapID, PremapImg>::iterator map_it;
		list <MapID>::iterator plist;
		QMutexLocker locker(&mutex);
		if ((map_it = m.find(id)) == m.end())  // not found
			qFatal("repush Id=%d not exist in map", id);
		else {
			l.erase(map_it->second.plist);
			l.push_back(id);	//repush it to cache_list tail
			plist = l.end();
			plist--;
			map_it->second.plist = plist;
		}
	}

	//find or reserve id
	int find_reserve(MapID id, PremapImg **premap_img) {
		map <MapID, PremapImg>::iterator map_it;
		QMutexLocker locker(&mutex);
		map_it = m.find(id);
		if (map_it == m.end()) { //not exist
			m[id] = PremapImg();
			*premap_img = NULL;
			return NOT_FETCH;
		}
		if (map_it->second.m.empty()) {
			*premap_img = NULL;
			return ALREADY_FETCH;
		}			
		else {
			*premap_img = &(map_it->second);
			return ALREADY_EXIST;
		}
	}

	void set_size(int _size) {
		QMutexLocker locker(&mutex);
		size = _size;
	}

	int get_size() const {
		return size;
	}

	void remove_exceed() {
		QMutexLocker locker(&mutex);
		CV_Assert(l.size() == m.size());
		int erase_size = (int)l.size() - size;
		for (int i = 0; i < erase_size; i++) {
			MapID id = *l.begin();
			l.erase(l.begin());
			m.erase(id);
		}
	}

	void clear(int layer) {
		QMutexLocker locker(&mutex);
		if (layer == -1) {
			m.clear();
			l.clear();
		}
		for (list<MapID>::iterator list_it = l.begin(); list_it != l.end();) {
			MapID id = *list_it;
			if (MAPID_L(id) == layer) {
				l.erase(list_it++); //list_it point to next 
				m.erase(id);
			}
			else
				list_it++;
		}
	}
};

class PostMapCache {
protected:
	map <MapID, PostmapImg> m; //store Postmap image, its number is small
	list<MapID> l; //postmap id is dst location dst_w * MAPID_X(id), dst_w * MAPID_Y(id))
	int size;
public:
	void set_size(int _size) {
		size = _size;
	}
	int get_size() const {
		return size;
	}

	void clear(int layer) {
		if (layer == -1) {
			m.clear();
			l.clear();
		}
		for (list<MapID>::iterator list_it = l.begin(); list_it != l.end();) {
			MapID id = *list_it;
			if (MAPID_L(id) == layer) {
				l.erase(list_it++); //list_it point to next 
				m.erase(id);
			}
			else
				list_it++;
		}
	}

	void remove_exceed() {
		CV_Assert(l.size() == m.size());
		int erase_size = (int)l.size() - size;
		for (int i = 0; i < erase_size; i++) {
			MapID id = *l.begin();
			l.erase(l.begin());
			m.erase(id);
		}
	}

	void find_repush(MapID id, PostmapImg ** premap_img) {
		map <MapID, PostmapImg>::iterator map_it;
		map_it = m.find(id);
		if (map_it != m.end()) {
			l.erase(map_it->second.plist);
			l.push_back(id);	//repush it to cache_list tail
			list <MapID>::iterator plist = l.end();
			plist--;
			map_it->second.plist = plist;
			*premap_img = &(map_it->second);
		}
		else
			*premap_img = NULL;
	}

	void insert(MapID id, const QImage & _m) {
		map <MapID, PostmapImg>::iterator map_it;
		list <MapID>::iterator plist;
		if ((map_it = m.find(id)) == m.end()) { // not found
			l.push_back(id);	//push it to cache_list tail
			plist = l.end();
			plist--;
			CV_Assert(*(plist) == id);
			m[id] = PostmapImg(plist, _m);
		}
		else {
			l.push_back(id);	//repush it to cache_list tail
			plist = l.end();
			plist--;
			CV_Assert(*(plist) == id);
			m[id] = PostmapImg(plist, _m);
		}
	}
};

class RenderImage
{
protected:
	//Following is for map dst to src
	vector<const ConfigPara *> cpara; //each layer's cpara, index is layer
	vector<MapXY *> mapxy;
	vector<Size> src_img_size; //each layer's src raw image size
	int dst_w;
	//upper is for map dst to src

	//Following is for Premap and Postmap image cache
	PreMapCache premap_cache;
	PostMapCache postmap_cache;
	//Upper is for Premap and Postmap image cache

	int prev_load_flag;
	vector<MapID> prev_draw_order;
public:
	void set_cfg_para(int layer, const ConfigPara * _cpara);
	const ConfigPara * get_cfg_para(int layer);
	Size get_src_img_size(int layer);
	void set_mapxy(int layer, const MapXY * _mapxy);
	MapXY * get_mapxy(int layer);
	int mapxy_merge_method(int layer);
	bool is_mapxy_origin(int layer);
	void set_dst_wide(int wide);
	int get_dst_wide();
	void invalidate_cache(int layer) {
		postmap_cache.clear(layer);
	}
	Point2d src2dst(int layer, Point src) const {
		if (layer == ABS_LAYER)
			return src;
		if (layer >= cpara.size() || layer < 0)
			return Point2d(0, 0);
		return mapxy[layer]->src2dst(src);
	}

	Point2d dst2src(int layer, Point dst) const {
		if (layer == ABS_LAYER)
			return dst;
		if (layer >= cpara.size() || layer < 0)
			return Point2d(0, 0);
		return mapxy[layer]->dst2src(dst);
	}
	//Input: mapid is for dst mapid
	//Output: imgs is dst QImage
	void render_img(const vector<MapID> & map_id, vector<QImage> & imgs, const vector<MapID> & draw_order);
    RenderImage();
	~RenderImage();
};

#endif // RENDERIMAGE_H
