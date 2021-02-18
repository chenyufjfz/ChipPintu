#ifndef OBJEXTRACT_H
#define OBJEXTRACT_H

#include <vector>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iclayer.h"
#include <QRect>
#include <QLine>
#include "markobj.h"

struct SearchArea {
	QRect rect;	
	int option;
    SearchArea(QRect r, int o) {
		rect = r;
		option = o;
	}
};

class ObjExtract {
protected:
	void * notify_para0;
	void(*notify)(void *, MarkObj * o);
public:
	void register_callback(void * p0, void(*_notify)(void *, MarkObj * o)) {
		notify_para0 = p0;
		notify = _notify;
	}
	virtual int set_train_param(int, int, int, int, int, int, int, int, int, float) = 0;
    virtual int set_extract_param(int, int, int, int, int, int, int, int, int, float) = 0;
    virtual int train(std::string file_name, std::vector<MarkObj> & obj_sets) = 0;
    virtual int extract(std::string file_name, QRect rect, std::vector<MarkObj> & obj_sets) = 0;
    virtual int train(std::vector<ICLayerWrInterface *> & ic_layer, std::vector<MarkObj> & obj_sets) = 0;
    virtual int extract(std::vector<ICLayerWrInterface *> & ic_layer, const std::vector<SearchArea> & area_, std::vector<MarkObj> & obj_sets) = 0;
    virtual void get_feature(int, int , int , std::vector<float> &, std::vector<int> & ) = 0;
    virtual cv::Mat get_mark(int) = 0;
	virtual cv::Mat get_mark1(int) = 0;
	virtual cv::Mat get_mark2(int) = 0;
	virtual cv::Mat get_mark3(int) = 0;
	virtual cv::Mat get_mark4(int) = 0;
	virtual cv::Mat get_mark5(int) = 0;
	virtual cv::Mat get_mark6(int) = 0;
	virtual cv::Mat get_mark7(int) = 0;
	ObjExtract() {
		notify = NULL;
	}
    virtual ~ObjExtract() {

    }
};

#endif // OBJEXTRACT_H

