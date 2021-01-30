#ifndef VWEXTRACT3_H
#define VWEXTRACT3_H
#include "vwextract_public.h"

struct DetectWirePara {
	int w_min, w_max; //in non-scale pixel unit
	int gray_i, gray_w;
	int dir_mask; //means move abs_org0 up, down, left, right
	int shape_mask; //1 means allow BRICK_I, 2 means allow BRICK_I_90
	int i_high; //scale image pixel
	int gray_th; //0~100
	int channel; //0~3
	int scale;
    int cr, cg, cb;
	int is_color;
	Point abs_org0; //scale image pixel for origin point
};

struct ImageBuf {
	Point lt;
	Mat raw_img;
	bool must_reserve;
	bool replace;
	ImageBuf() {

	}
    ImageBuf(const Point & _lt, Mat & _raw_img) {
		lt = _lt;
		raw_img = _raw_img;
		replace = false;
		must_reserve = false;
	}
};

class VWExtractAnt : public VWExtract
{
protected:
	DetectWirePara dw0;
	int layer0;
	int search_opt;
	int prev_scale;
	vector<ImageBuf> img_bufs;
	Mat color2gray(Mat & color_img);
	Mat prepare_img(ICLayerWrInterface * ic_layer, int scale, QRect rect, bool replace_buf);

public:
	VWExtractAnt();
	~VWExtractAnt() {}
	int set_train_param(int , int , int , int , int , int , int , int , int , float ) { return 0; }
	int set_extract_param(int layer, int type, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, float pf1);
	Mat get_mark(int ) { return Mat(); }
	Mat get_mark1(int ) { return Mat(); }
	Mat get_mark2(int ) { return Mat(); }
	Mat get_mark3(int ) { return Mat(); }
	Mat get_mark4(int) { return Mat(); }
	Mat get_mark5(int) { return Mat(); }
	Mat get_mark6(int) { return Mat(); }
	Mat get_mark7(int) { return Mat(); }
	int train(string, std::vector<MarkObj> &) { return 0; }
	int extract(string file_name, QRect rect, std::vector<MarkObj> & obj_sets);
	int train(vector<ICLayerWrInterface *> &, std::vector<MarkObj> &) { return 0; }
	int extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & _area, vector<MarkObj> & obj_sets);
	void get_feature(int, int, int, std::vector<float> &, std::vector<int> &) {}
};
#endif // VWEXTRACT3_H

