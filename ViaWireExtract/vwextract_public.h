#ifndef VWEXTRACT_PUBLIC_H
#define VWEXTRACT_PUBLIC_H
#include "opencv2/imgproc/imgproc.hpp"
#include "objextract.h"
#include <opencv2/ml/ml.hpp>
#include <map>
#include <set>
#include <opencv2/contrib/contrib.hpp>
#include <QMutexLocker>
#include "qsharedpointer.h"

using namespace std;
using namespace cv;

#ifndef QT_DEBUG
void print_stack(void);
#undef CV_Assert
#define CV_Assert(x) do {if (!(x)) {print_stack(); qFatal("Wrong at %s, %d", __FILE__, __LINE__);}} while(0)
#endif

#ifdef QT_DEBUG
#ifdef Q_OS_WIN
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#define new DEBUG_NEW
#endif
#endif

#define BRICK_NO_WIRE		0
#define BRICK_i_0			1
#define BRICK_i_90			2
#define BRICK_i_180			3
#define BRICK_i_270			4
#define BRICK_I_0			5
#define BRICK_I_90			6
#define BRICK_L_0			7
#define BRICK_L_90			8
#define BRICK_L_180			9
#define BRICK_L_270			10
#define BRICK_T_0			11
#define BRICK_T_90			12
#define BRICK_T_180			13
#define BRICK_T_270			14
#define BRICK_X_0			15
#define BRICK_J_0			16
#define BRICK_J_90			17
#define BRICK_J_180			18
#define BRICK_J_270			19
#define BRICK_l_0			20
#define BRICK_l_90			21
#define BRICK_l_180			22
#define BRICK_l_270			23
#define BRICK_Z_0			24
#define BRICK_Z_90			25
#define BRICK_P_0			26
#define BRICK_P_90			27
#define BRICK_P_180			28
#define BRICK_P_270			29
#define BRICK_Y_45			30
#define BRICK_Y_135			31
#define BRICK_Y_225			32
#define BRICK_Y_315			33
#define BRICK_IN_USE		33
#define BRICK_ONE_POINT		64
#define BRICK_II_0			65
#define BRICK_II_90			66
#define BRICK_II_180		67
#define BRICK_II_270		68
#define BRICK_III_0			69
#define BRICK_III_90		70
#define BRICK_HOLLOW		252
#define BRICK_FAKE_VIA		253
#define BRICK_VIA			254
#define BRICK_INVALID		255

#define MIN_SCORE			10

#define DIR_UP				0
#define DIR_RIGHT			1
#define DIR_DOWN			2
#define DIR_LEFT			3
#define DIR_UPRIGHT			4
#define DIR_DOWNRIGHT		5
#define DIR_DOWNLEFT		6
#define DIR_UPLEFT			7

#define DIR_UP1_MASK		(1 << DIR_UP)
#define DIR_RIGHT1_MASK		(1 << DIR_RIGHT)
#define DIR_DOWN1_MASK		(1 << DIR_DOWN)
#define DIR_LEFT1_MASK		(1 << DIR_LEFT)
#define DIR_UPLEFT_MASK		(1 << DIR_UPLEFT)
#define DIR_UPRIGHT_MASK	(1 << DIR_UPRIGHT)
#define DIR_DOWNLEFT_MASK	(1 << DIR_DOWNLEFT)
#define DIR_DOWNRIGHT_MASK	(1 << DIR_DOWNRIGHT)

#define MAX_LAYER_NUM		50
//search opt
#define OPT_PARALLEL_SEARCH		1
#define OPT_POLYGON_SEARCH		2

#define FEATURE_ROW				3
#define POINT_TOT_PROB			100

#define EdgeFeatureVecLen		3
//POINT_IS_NOT_EDGE = POINT_IS_INSU + POINT_IS_WIRE
//POINT_IS_NOT_CARE = POINT_IS_VIA + POINT_IS_EDGE_VIA_WIRE + POINT_IS_EDGE_VIA_INSU
#define POINT_IS_INSU			0
#define POINT_IS_WIRE			1
#define POINT_IS_EDGE_WIRE_INSU 2
#define POINT_DIR				3
#define POINT_VIA_CENTER		16
#define POINT_VIA_REGION		8
#define MINIMUM_EDGE_GRAD		5
#define EDGE_JUDGE_BORDER		3
struct Brick {
	int a[3][3];
	int shape;
};

class BrickConnect {
protected:
	unsigned long long bfm[8][64];
	int sa[BRICK_IN_USE + 1][BRICK_IN_USE + 1];
protected:

	/*
	input: brick0, brick1
	output: brick0 + brick1
	*/
	int shape_add(int brick0, int brick1);

public:
	//fit check if brick0's dir can be brick1
	bool fit(int dir, int brick0, int brick1);
	BrickConnect();
	int quick_shape_add(int brick0, int brick1);
};


struct ElementObj : public MarkObj
{
	union {
		unsigned long long attach;
		void * ptr;
	} un;

	ElementObj() {

	}

	ElementObj(const MarkObj &o) : MarkObj(o) {
		un.attach = 0;
	}

	ElementObj(const ElementObj &o) {
		*this = o;
	}

	bool intersect_rect(const QRect & r);
};

#ifndef VWEXTRACT_PUBLIC_C
extern struct Brick bricks[];
extern int dxy[8][2];
extern int dir_1[8];
extern int dir_2[8];
extern int dir_3[8];
extern BrickConnect brick_conn;
#endif
int get_pts_dir(Point pt1, Point pt2);
void get_line_pts(Point pt1, Point pt2, vector <Point> & pts);
void get_line_pts2(Point pt1, Point pt2, vector <Point> & pts);
bool intersect_line(Point p0, int dir0, Point p1, int dir1, Point & pis);
float pt2line_distance(Point p1, Point p0, int dir);

/*
Compute integrate and line integral
in img
in compute_line
out ig, same as openCV integral
out iig, sum(sum(img(i,j)*img(i,j), j=0..x-1) i=0..y-1
out lg, sum(img(i,j), j=0..x-1)
out llg, sum(img(i,j)*img(i,j), j=0..x-1)
*/
void integral_square(const Mat & img, Mat & ig, Mat & iig, Mat & lg, Mat & llg, bool compute_line_integral);
void clip_img(Mat & img, int gray_low, int gray_high, Mat & new_img);
bool contain_dir(int d1, int d2);
void deldir(const string &path);
void save_rst_to_file(const vector<MarkObj> & obj_sets, int scale, FILE * fp);
void save_obj_to_file(const vector<ElementObj *> obj_sets, FILE * fp);

//CircleCheck use octagon to approximate circle, use grad accumulation to judge every edge
class CircleCheck {
protected:
	int d0, th, margin;
	vector<vector<Point> > eo; //eo[i] is one edge, eo[i][0] is edge direction, eo[i][j] is edge point's offset to origin
	vector<vector<int> > eshift; //eshift is eo's shift, use shift to optimize speed
	int dir_shift[8]; //for dir up, down, left, right shift
	vector<Vec8i> ei; //ei[i] is one octagon, ei[i][j] point to eo or eshift index
	float coef[8][8]; //index is dir, encourage if dir is same, punish if dir is opposite or orthogonality, 0 if dir is compatible

	void init(int _d0, int _th, const Mat * d, const Mat * s);

	Point check(const Mat * d, const Mat * s, const Mat * img, int _d0, int _th, int _thr, vector<Point> * vs);

public:
	CircleCheck() {
		d0 = 0;
		th = -1;
	}

	Point via_double_check(const Mat * img, int xo, int yo, int d0, int r1, int th, int th2, int thr, vector<Point> * vs);
};

int compute_circle_dx(int d, vector<Vec3i> & d0);

#define VIA_FEATURE			0x40000000
#define EDGE_FEATURE		0x20000000
#define VIA_IS_VIA			1
#define EDGE_IS_WIRE_INSU	1
#define EDGE_IS_WIRE		2
#define EDGE_IS_INSU		4
#define EDGE_NEAR_VIA		8
#define TRAIN_CMD_INSERT	0
#define TRAIN_CMD_DELETE	1
#define TRAIN_CMD_GET		2

#define MAX_DARK_LEVEL		160
#define MAX_GRAD_LEVEL		192
class EdgeFeatureML {
protected:
	uchar wi_prob[MAX_DARK_LEVEL][MAX_GRAD_LEVEL][2];
	Range wi_dark0, wi_dark1, edge_g0, edge_g1;
	/*
	Input img: raw image
	Input grad: image's grad
	Input o: get o's edge vector
	output e: edge vector
	inout dir: feature vector in dir, if input dir <0, it will search max grad as dir,
	if dir>=0, it will search specified dir.
	*/
	void get_feature_vector(const Mat & img, const Mat &grad, Point o, int e[], int & dir);
public:
	EdgeFeatureML();
	bool is_valid;
	/*
	input img, raw image
	input grad, image grad
	inout org, for intput it is edge nearby point, for output it is edge point
	input range, search range, better within [2,2]
	input label
	output features
	return true, if find feature, false, not find feature
	Single sample feature extract, after collect several samples, call train
	*/
	bool feature_extract(const Mat & img, Point & org, Point & range, int label, vector<vector<int> > & features);
	/*
	input feature, it is output from feature_extrac
	return true, if train success, false if wrong
	Before train, need collect several sample feature
	*/
	bool train(const vector<vector<int> > & features);
	/*
	Input img
	output prob
	Input via_mark, via mark which is result of ViaML::judge
	Output debug_mark
	Use case
	feature_extract
	feature_extract
	feature_extract
	....
	train
	judge
	*/
	void judge(const Mat & img, Mat & grad, Mat & prob, const Mat & via_mark, Mat & debug_mark);
};

class ViaML {
protected:
	QSharedPointer<CvSVM> via_svm; //it is via or not, trained by feature
	int dir_remap[8];
	set<int> via_dia;  //it is trained by feature
	map<int, vector<Vec3i> > d0s; //d0s.second store one circle xy, d0s.second[0] is (y, x1, x2), (x2, y) to (x1, y) forms circle in-points
	map<int, vector<vector<Vec3i> > > circles; //circles.second is octagon, circles.second[i] is one edge, circles.second[i][j] is edge's point,
	//circles.second[i][j][0] is y-grad, circles.second[i][j][1] is x-grad, circles.second[i][j][2] is edge's shift
	map<int, vector<vector<Point3i> > > circle_edges; //circle_edges is circle extern edge circle_edges.second[0] are point sets in circle r+1.
	//circle_edges.second[1] are point set in circle r+2, circle_edges.second[0][0] is circle r+1 1st point.
	//circle_edges.second[0][0].x & y is offset, circle_edges.second[0][0].z is degree
	/*
	input lg, line sum mat
	input d, via diameter
	input sep0, search range [sep0, lg.rows-d-sep0] *[sep0, lg.cols-d-sep0]
	output loc, top left location for brightest circle
	find local brightest location in loc, it is a pre-filter for compute_feature
	*/
	void find_best_bright(const Mat & lg, int d, int sep0, vector<Point> & loc);
	/*
	input ig, raw image
	input grad, raw image grad
	input d, diam
	input loc, it is output from find_best_bright
	output feature, feature[0][0] is label which is filled by caller, feature[0][2], feature[0][3] is diameter-x, diameter-y
	feature[0][4] and feature[0][5] is average gray in via
	feature[1][0..7] is grad for each octagon edge
	feature[2][0..7] is middle filter for outside octagon edge (1 point away)
	feature[3][0..7] is middle filter for outside octagon edge (2 point away)
	Return feature likelihood, bigger is better
	*/
	int compute_feature(const Mat & img, const Mat & lg, const Mat & grad, int d0, const vector<Point> & loc, Mat features[], int score_min);
	/*
	input img,
	inout org, for input, it is a point inside via, for output it is via's center
	inout range, for intput, search range = max{d} + range, normally it is 0, for output it is via's diameter
	input d, via diameter
	output feature, feature[0][0] is label which is filled by caller, feature[0][2], feature[0][3] is diameter-x, diameter-y
	output vs, via circle xy
	return feature score
	*/
	int feature_extract(const Mat & img, Point & org, Point & range, const vector<int> & d0, Mat & feature, vector<Point> * vs, int score_min);
public:
	ViaML();
	~ViaML();
	bool is_valid;
	/*
	input img,
	inout org, for input, it is a point inside via, for output it is via's center
	inout range, for intput, search range = max{d} + range, normally it is 0, for output it is via's diameter
	input min_d0, max_d0, via diameter range
	input label, via or not via
	output features, 
	output vs, via circle xy
	input multi_thread, run in multi_thread or not
	return true, if find feature, false, not find feature
	Single sample feature extract, after collect several samples, call train
	*/
	bool feature_extract(const Mat & img, Point & org, Point & range, int min_d0, int max_d0, int label, vector<Mat> & features, 
		vector<Point> * vs);
	/*
	input feature, it is output from feature_extract
	input weight, weight for miss via vs false via
	return true, if train success, false if wrong
	Before train, need collect several sample feature
	*/
	bool train(const vector<Mat> & feature, float weight = 0.5f);
	/*
	Input img
	output via_mark, output for edge_search
	Inout org, via location
	Input range, normally it is 0
	output label
	Return probability, bigger is via, smaller is no-via
	Use case
	feature_extract
	feature_extract
	feature_extract
	....
	train
	judge
	*/
	float judge(const Mat & img, Mat & via_mark, Point & org, Point & range, int &label);
};

/*
Input es
Output ms
Note: it will release es and appends to ms
*/
void convert_element_obj(const vector<ElementObj *> & es, vector<MarkObj> & ms, int scale);

class VWfeature {
protected:
	ViaML vml;
	EdgeFeatureML eml, veml;
	bool retrain_via, retrain_edge;
	float via_weight;
	vector<Mat> via_features;
	vector<Point> via_locs, edge_locs;
	vector<vector<int> > edge_features;
	/*
	split edge_features to edge_via and edge_no_via
	edge_via contain edges which have vias nearby,
	edge_no_via contain edges which have no via nearby.
	*/
	void split_via_edges(vector<vector<int> > & edge_via, vector<vector<int> > & edge_no_via);
	/*
	Input img
	output via_mark
	Inout org, for input, it is a point inside via, for output it is via center
	Inout range, for input, it is search range= max{d} + range, for output it is diameter
	Output label, via & wire label
	input multi_thread, if call with multi_thread, true
	Return probability, bigger is via, smaller is no via, it call vml judge
	*/
	float via_judge(const Mat & img, Mat & via_mark, Point & org, Point & range, int & label);
public:
	VWfeature();
	void set_via_para(float via_weight_ = 0.5f);

	/*
	Input img
	Input local, via/wire xy related to img
	Input global, via/wire xy global
	Output range
	Input d0, via diameter
	Input label, via or wire label
	Output vs, via edge
	It will delete existing fature in same location and then add
	*/
	bool add_feature(const Mat & img, Point local, Point & global, Point & range, int min_d0, int max_d0, int label, vector<Point> * vs);
	/*
	Input global, via/wire xy global, same as add_feature global
	Input d0, via diameter
	*/
	bool del_feature(Point global, int d0);
	/*
	Inout global, via/wire xy global
	Input d0, via diameter
	return label with the via
	*/
	int get_via_label(Point & global, int d0);
	//delete all feature
	void clear_feature();
	//get via and edge features
	void get_features(vector<MarkObj> & obj_sets);
	//return max diameter
	int get_max_d();
	void write_file(string project_path, int layer, int insu_min, int wire_min);
	bool read_file(string project_path, int layer);
	//return if via is valid or not
	bool via_valid();
	/*
	input img
	output via_mark, via mark to be used by edge_search
	output vs, vs.p0 is via center, vs.p1 is via diameter.
	input multi_thread, true means multithread search
	It calls via_judge to check every via
	*/
	void via_search(const Mat & img, Mat & via_mark, Mat & mark_dbg, vector<ElementObj *> & vs);
	/*
	input img
	output prob
	input via_mark, it is via_search output
	output debug_mark
	call EdgeFeatureML::judge
	*/
	void edge_search(const Mat & img, Mat & prob, const Mat & via_mark, Mat & debug_mark);
};


class VWExtract : public ObjExtract
{
public:
	VWExtract() {}
	static VWExtract * create_extract(int method);

	virtual ~VWExtract() {

	}
};


#endif // VWEXTRACT_PUBLIC_H

