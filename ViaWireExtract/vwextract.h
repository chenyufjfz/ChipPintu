#ifndef VWEXTRACT_H
#define VWEXTRACT_H
#include "vwextract_public.h"

using namespace std;
using namespace cv;


class VWExtract : public ObjExtract
{
public:
	VWExtract() {}
	static VWExtract * create_extract(int method);

	virtual ~VWExtract() {

	}
};

class VWExtractML : public VWExtract
{
protected:
	vector<VWfeature> vwf;
	int current_layer;
	int via_diameter;
	vector<Mat> via_mark;

public:
	VWExtractML();
	~VWExtractML() {}
	int set_train_param(int layer, int d, int , int, int, int, int, int, int, float);
	int set_extract_param(int layer, int , int , int , int , int , int , int , int , float );
	Mat get_mark(int layer);
	Mat get_mark1(int layer) { return Mat(); }
	Mat get_mark2(int layer) { return Mat();}
	Mat get_mark3(int) { return Mat(); }
	int train(string img_name, vector<MarkObj> & obj_sets);
	int extract(string file_name, QRect rect, vector<MarkObj> & obj_sets);
	int train(vector<ICLayerWrInterface *> &, const vector<MarkObj> &) { return 0; }
	int extract(vector<ICLayerWrInterface *> & ic_layer, const vector<SearchArea> & _area, vector<MarkObj> & obj_sets) { return 0; }
	void get_feature(int, int, int, vector<float> &, vector<int> &);
};

#endif // VWEXTRACT_H
