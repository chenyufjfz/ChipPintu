#ifndef OBJEXTRACT_H
#define OBJEXTRACT_H

#include <vector>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <QRect>
#include <QLine>

struct MarkObj {
    int type;
    int type2;
    int select_state;
    QPoint p0, p1;
};


enum {
    OBJ_NONE=0,
    OBJ_AREA,
    OBJ_WIRE,
    OBJ_VIA,
    SELECT_OBJ
};

enum {
    AREA_LEARN=0,
    AREA_METAL,
    AREA_CELL
};

class ObjExtract {
public:
    virtual void set_param(int , int , int , float , float , float , int , int ) = 0;
    virtual void train(std::string file_name, const std::vector<MarkObj> & obj_sets,
        int , int ) = 0;
    virtual void extract(std::string file_name, QRect rect, std::vector<MarkObj> & obj_sets) = 0;
    virtual void get_feature(int , int , std::vector<float> & ) = 0;
    virtual cv::Mat get_mark() = 0;
    virtual cv::Mat get_mark1() = 0;
    virtual cv::Mat get_mark2() = 0;
    virtual cv::Mat get_mark3() = 0;
    virtual ~ObjExtract() {

    }
};

#endif // OBJEXTRACT_H

