#ifndef VWEXTRACT_PUBLIC_H
#define VWEXTRACT_PUBLIC_H
#include "opencv2/imgproc/imgproc.hpp"
#include "objextract.h"
#include <opencv2/ml/ml.hpp>
#include <map>
#include <set>
#include <opencv2/contrib/contrib.hpp>
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

//search opt
#define OPT_PARALLEL_SEARCH		1
#define OPT_POLYGON_SEARCH		2

#define FEATURE_ROW			8
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
void save_rst_to_file(const vector<MarkObj> & obj_sets, int scale);

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
#define VIA_IS_VIA			1
#define VIA_UP_WIRE			2
#define VIA_RIGHT_WIRE		4
#define VIA_DOWN_WIRE		8
#define VIA_LEFT_WIRE		16
#define VIA_UPRIGHT_WIRE	32
#define VIA_DOWNRIGHT_WIRE	64
#define VIA_DOWNLEFT_WIRE	128
#define VIA_UPLEFT_WIRE		0x100
#define VIA_UP_VALID		0x200
#define VIA_RIGHT_VALID		0x400
#define VIA_DOWN_VALID		0x800
#define VIA_LEFT_VALID		0x1000
#define VIA_UPRIGHT_VALID	0x2000
#define VIA_DOWNRIGHT_VALID 0x4000
#define VIA_DOWNLEFT_VALID	0x8000
#define VIA_LR_SYMMETRY		1
#define VIA_UD_SYMMETRY		2

class ViaML {
protected:
	QSharedPointer<CvSVM> via_svm; //it is via or not, trained by feature
	QSharedPointer<CvSVM> vwi_svm[8]; //via connect to wire or insu, trained by feature
	set<int> via_dia;  //it is trained by feature
	map<int, vector<Vec3i> > d0s; //d0s.second store one circle xy, d0s.second[0] is (y, x1, x2), (x2, y) to (x1, y) forms circle in-points
	map<int, vector<vector<Vec3i> > > circles; //circles.second is octagon, circles.second[i] is one edge, circles.second[i][j] is edge's point,
	//circles.second[i][j][0] is y-grad, circles.second[i][j][1] is x-grad, circles.second[i][j][2] is edge's shift
	map<int, vector<Vec6i> > circle_edges; //circle_edges is octagon,
	//circle_edges.second[i][0] and circle_edges.second[i][1] is outside edge start point
	//circle_edges.second[i][3] is direction, circles.second[i][4] is outside edge's len
	//circles.second[i][5] is scan line direction for middle sort
	/*
	input lg, line sum mat
	input d, via diameter
	input sep0, search range [sep0, lg.rows-d-sep0] *[sep0, lg.cols-d-sep0]
	output loc, top left location for brightest circle
	find local brightest location in loc, it is a pre-filter for compute_feature
	*/
	void find_best_bright(const Mat & lg, int d, int sep0, vector<Point> & loc, bool multi_thread);
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
	int compute_feature(const Mat & img, const Mat & lg, const Mat & grad, int d0, const vector<Point> & loc, Mat features[], bool multi_thread, int score_min);
	/*
	input img,
	inout org, for input, it is a point inside via, for output it is via's center
	inout range, for intput, search range = max{d} + range, normally it is 0, for output it is via's diameter
	input d, via diameter
	output feature, feature[0][0] is label which is filled by caller, feature[0][2], feature[0][3] is diameter-x, diameter-y
	output vs, via circle xy
	return true, if find feature, false, not find feature
	*/
	int feature_extract(const Mat & img, Point & org, Point & range, const vector<int> & d0, Mat & feature, vector<Point> * vs, bool multi_thread, int score_min);
public:
	ViaML();
	~ViaML();
	
	bool feature_extract(const Mat & img, Point & org, Point & range, int min_d0, int max_d0, int label, vector<Mat> & features, 
		vector<Point> * vs, bool multi_thread);
	/*
	input feature, it is output from feature_extract
	input flag, only for via-wire & via-insu training
	input weight, weight for miss via vs false via
	return true, if train success, false if wrong
	*/
	bool train(const vector<Mat> & feature, int flag = 0, float weight = 0.5f);
	/*
	Input img
	Inout org, via location
	Input range, normally it is 0
	output label
	Return probability, bigger is via, smaller is no-via
	*/
	float judge(const Mat & img, Point & org, Point & range, int &label, bool multi_thread);
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
	bool retrain_via, is_via_valid;
	int via_flag;
	float via_weight;
	vector<Mat> via_features;
	vector<Point> via_locs;

	/*
	Input img
	Inout org, for input, it is a point inside via, for output it is via center
	Inout range, for input, it is search range= max{d} + range, for output it is diameter
	Output label, via & wire label
	input multi_thread, if call with multi_thread, true
	Return probability, bigger is via, smaller is no via
	*/
	float via_judge(const Mat & img, Point & org, Point & range, int & label, bool multi_thread);
public:
	VWfeature();
	void set_via_para(int via_flag_, float via_weight_ = 0.5f);

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
	//return max diameter
	int get_max_d();
	void write_file(string project_path, int layer);
	bool read_file(string project_path, int layer);
	//return if via is valid or not
	bool via_valid(bool multi_thread);
	/*
	input img
	output vs, vs.p0 is via center, vs.p1 is via diameter.
	input multi_thread, true means multithread search
	*/
	void via_search(const Mat & img, Mat & mark_dbg, vector<ElementObj *> & vs, bool multi_thread);
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

