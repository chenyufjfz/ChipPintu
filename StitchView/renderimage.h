#ifndef RENDERIMAGE_H
#define RENDERIMAGE_H
#include "featext.h"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <QImage>
#include <map>
#include <list>
#include <QMutexLocker>

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
	double beta;
	double cos_beta, sin_beta;
	int merge_method;
	vector<TurnPoint> tx, ty;
    double z0x, z0y;
public:
	MapXY() {
		beta = 0; //set default rotation angle
		cos_beta = 1;
		sin_beta = 0;
        z0x = 1;
        z0y = 1;
		tx.push_back(TurnPoint(0, 0)); //push default zoom x 1
		ty.push_back(TurnPoint(0, 0)); //push default zoom y 1
		merge_method = MERGE_BY_DRAW_ORDER;
	}
	
	void read_file(const FileNode& node) {
		beta = (double)node["beta"];
		z0x = (double)node["z0x"];
		z0y = (double)node["z0y"];
		merge_method = (int)node["merge_method"];
		cos_beta = cos(beta);
		sin_beta = sin(beta);
	}

	void write_file(FileStorage& fs) const {
		fs << "{" << "beta" << beta;
		fs << "z0x" << z0x;
		fs << "z0y" << z0y;
		fs << "merge_method" << merge_method << "}";		
	}

	bool is_original() const {
		return (beta == 0 && z0x == 1 && z0y == 1 && tx.size()==1 && ty.size()==1
			&& tx[0].sp==tx[0].dp && ty[0].sp==ty[0].dp);
	}

	int get_merge_method() const {
		return merge_method;
	}

	int set_merge_method(int _merge_method) {
		merge_method = _merge_method;
	}

	void set_default_zoomx(double _sz) {
		CV_Assert(_sz != 0);
        z0x = _sz;
	}

	double get_default_zoomx() const {
		return z0x;
	}

	void set_default_zoomy(double _sz) {
		CV_Assert(_sz != 0);
        z0y = _sz;
	}

	double get_default_zoomy() const {
		return z0y;
	}

	void set_beta(double _beta) {
		beta = _beta;
		cos_beta = cos(beta);
		sin_beta = sin(beta);
	}

	double get_beta() const {
		return beta;
	}

	void recompute_tp(vector<TurnPoint> & tp, double default_zoom) {
		for (int i = 0; i < (int)tp.size() - 1; i++) {
			CV_Assert(tp[i + 1].sp != tp[i].sp && tp[i + 1].dp != tp[i].dp);
			tp[i].sz = (tp[i + 1].dp - tp[i].dp) / (tp[i + 1].sp - tp[i].sp);
			tp[i].dz = 1 / tp[i].sz;
		}
		tp.back().sz = default_zoom;
		tp.back().dz = 1 / default_zoom;
	}

	void add_turn_pointx(double sx, double dx) {
		for (int i = 0; i < (int)tx.size(); i++) {
			if (tx[i].sp >= sx) {
				if (tx[i].dp <= dx || tx[i].sp == sx)
					qFatal("add_turn_pointx error, dp=%f, dx=%f, sp=%f, sx=%f", tx[i].dp, dx, tx[i].sp, sx);

				if (i != 0) {
					if (tx[i - 1].dp >= dx)
						qFatal("add_turn_pointx err, dp=%f, dx=%f, sp=%f, sx=%f", tx[i - 1].dp, dx, tx[i].sp, sx);
				}
				tx.insert(tx.begin() + i, TurnPoint(sx, dx));
				recompute_tp(tx, z0x);
				return;
			}
		}
		tx.insert(tx.end(), TurnPoint(sx, dx));
		recompute_tp(tx, z0x);
	}

	void add_turn_pointy(double sy, double dy) {
		for (int i = 0; i < (int)ty.size(); i++) {
			if (ty[i].sp >= sy) {
				if (ty[i].dp <= dy || ty[i].sp == sy)
					qFatal("add_turn_pointx error, dp=%f, dx=%f, sp=%f, sx=%f", ty[i].dp, dy, ty[i].sp, sy);

				if (i != 0) {
					if (ty[i - 1].dp >= dy)
						qFatal("add_turn_pointx err, dp=%f, dx=%f, sp=%f, sx=%f", ty[i - 1].dp, dy, ty[i].sp, sy);
				}
				ty.insert(ty.begin() + i, TurnPoint(sy, dy));
				recompute_tp(ty, z0y);
				return;
			}
		}
		ty.insert(ty.end(), TurnPoint(sy, dy));
		recompute_tp(ty, z0y);
	}

    Point2d mid2dst(Point2d mid) const {
		Point2d dst;
		int i;
		for (i = 0; i < (int)tx.size(); i++) {
            if (mid.x < tx[i].sp) {
				if (i == 0)
                    dst.x = tx[0].dp + z0x * (mid.x - tx[0].sp);
				else
                    dst.x = tx[i - 1].dp + tx[i - 1].sz * (mid.x - tx[i - 1].sp);
				break;
			}
		}
		if (i == (int)tx.size())
            dst.x = tx[i - 1].dp + z0x * (mid.x - tx[i - 1].sp);
		
		for (i = 0; i < (int)ty.size(); i++) {
            if (mid.y < ty[i].sp) {
				if (i == 0)
                    dst.y = ty[0].dp + z0y * (mid.y - ty[0].sp);
				else
                    dst.y = ty[i - 1].dp + ty[i - 1].sz * (mid.y - ty[i - 1].sp);
				break;
			}
		}
		if (i == (int)ty.size())
            dst.y = ty[i - 1].dp + z0y * (mid.y - ty[i - 1].sp);
		return dst;
	}

    Point2d dst2mid(Point2d dst) const {
        Point2d mid;
		int i;
		for (i = 0; i < (int)tx.size(); i++) {
            if (dst.x < tx[i].dp) {
				if (i == 0)
                    mid.x = tx[0].sp + (dst.x - tx[0].dp) / z0x;
				else
                    mid.x = tx[i - 1].sp + tx[i - 1].dz * (dst.x - tx[i - 1].dp);
				break;
			}
		}
		if (i == (int)tx.size())
            mid.x = tx[i - 1].sp + (dst.x - tx[i - 1].dp) / z0x;

		for (i = 0; i < (int)ty.size(); i++) {
            if (dst.y < ty[i].dp) {
				if (i == 0)
                    mid.y = ty[0].sp + (dst.y - ty[0].dp) / z0y;
				else
                    mid.y = ty[i - 1].sp + ty[i - 1].dz * (dst.y - ty[i - 1].dp);
				break;
			}
		}
		if (i == (int)ty.size())
			mid.y = ty[i - 1].sp + (dst.y - ty[i - 1].dp) / z0y;
        return mid;
	}

    Point2d src2mid(Point src) const {
        return Point2d(cos_beta * src.x + sin_beta * src.y, - sin_beta * src.x + cos_beta * src.y);
    }

    Point2d mid2src(Point mid) const {
        return Point2d(cos_beta * mid.x - sin_beta * mid.y, sin_beta * mid.x + cos_beta * mid.y);
    }

	Point2d src2dst(Point src) const {
		return mid2dst(src2mid(src));
	}

	Point2d dst2src(Point dst) const {
		return mid2src(dst2mid(dst));
	}
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
	vector<ConfigPara> cpara; //each layer's cpara, index is layer
	vector<MapXY> mapxy;
	vector<Size> src_img_size; //each layer's image size
	int dst_w;
	//upper is for map dst to src

	//Following is for Premap and Postmap image cache
	PreMapCache premap_cache;
	PostMapCache postmap_cache;
	//Upper is for Premap and Postmap image cache

	int prev_load_flag;
	vector<MapID> prev_draw_order;
public:
	void set_cfg_para(int layer, const ConfigPara & _cpara);
	ConfigPara get_cfg_para(int layer);
	Size get_src_img_size(int layer);
	void set_mapxy(int layer, const MapXY & _mapxy);
	MapXY get_mapxy(int layer);
	bool is_mapxy_origin(int layer);
	void set_dst_wide(int wide);
	int get_dst_wide();

	//Input: mapid is for dst mapid
	//Output: imgs is dst QImage
	void render_img(const vector<MapID> & map_id, vector<QImage> & imgs, const vector<MapID> & draw_order, int load_flag);
    RenderImage();
};

#endif // RENDERIMAGE_H
