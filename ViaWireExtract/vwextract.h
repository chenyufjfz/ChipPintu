#ifndef VWEXTRACT_H
#define VWEXTRACT_H
#include "vwextract_public.h"

using namespace std;
using namespace cv;

class VWExtractML : public VWExtract
{
protected:
	vector<VWfeature> vwf;
	int layer_min, layer_max, train_cmd;
	int via_diameter_min, via_diameter_max, insu_min, wire_min;
	vector<Mat> via_mark;
	vector<Mat> edge_mark, edge_mark1, edge_mark2, edge_mark3;
public:
	VWExtractML();
	~VWExtractML() {}
	int set_train_param(int , int , int, int, int, int, int, int, int, float);
	int set_extract_param(int layer, int , int , int , int , int , int , int , int , float );
	Mat get_mark(int layer);
	Mat get_mark1(int layer);
	Mat get_mark2(int layer);
	Mat get_mark3(int layer);
	int train(string img_name, vector<MarkObj> & obj_sets);
	int extract(string file_name, QRect rect, vector<MarkObj> & obj_sets);
	int train(vector<ICLayerWrInterface *> &, vector<MarkObj> &);
	int extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & _area, vector<MarkObj> & obj_sets);
	void get_feature(int, int, int, vector<float> &, vector<int> &) {}
};

#endif // VWEXTRACT_H
