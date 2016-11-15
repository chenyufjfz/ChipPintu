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
public:
	virtual int set_train_param(int, int, int, int, int, float, float, float) = 0;
    virtual int set_extract_param(int, int, int, int, int, float, float, float, float) = 0;
    virtual int train(std::string file_name, const std::vector<MarkObj> & obj_sets) = 0;
    virtual int extract(std::string file_name, QRect rect, std::vector<MarkObj> & obj_sets) = 0;
	virtual int train(ICLayerWr *ic_layer, const std::vector<MarkObj> & obj_sets) = 0;
	virtual int extract(ICLayerWr * ic_layer, const std::vector<SearchArea> & area_, std::vector<MarkObj> & obj_sets) = 0;
    virtual void get_feature(int , int , std::vector<float> & ) = 0;
    virtual cv::Mat get_mark() = 0;
    virtual cv::Mat get_mark1() = 0;
    virtual cv::Mat get_mark2() = 0;
    virtual cv::Mat get_mark3() = 0;
    virtual ~ObjExtract() {

    }
};

#endif // OBJEXTRACT_H

