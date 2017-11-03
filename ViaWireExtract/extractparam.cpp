#include "extractparam.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <QtGlobal>
using namespace cv;

#define PP_SET_PARAM			0
#define PP_RGB2GRAY				1
#define PP_COMPUTE_MIN_STAT		2
#define PP_ADJUST_GRAY_LVL		3
#define PP_COARSE_LINE_SEARCH	4
#define PP_COARSE_VIA_MASK		5
#define PP_FINE_VIA_SEARCH		6
#define PP_REMOVE_VIA			7
#define PP_FINE_SEARCH_MASK		8
#define PP_FINE_LINE_SEARCH		9
#define PP_ASSEMBLE				10
#define PP_CHECK_VIA_WIRE_CONNECT	11
#define PP_OBJ_PROCESS			254
enum {
	LayerInfo=0,
	ViaInfo,
	WireInfo,
	Rgb2Gray,
	MinStat,
	GrayLevel,
	CoarseLineSearch,
	CoarseViaSearch,
	FineViasSearch,
	RemoveVia,
	HotlineMask,
	FineLineSearch,
	AssembleLine,
	CheckViaWireConnect,
	FilterObj
};

string method_name[] = {
	"LayerInfo",
	"ViaInfo",
	"WireInfo",
	"Rgb2Gray",
	"MinStat",
	"GrayLevel",
	"CoarseLineSearch",
	"CoarseViaSearch",
	"FineViasSearch",
	"RemoveVia",
	"HotlineMask",
	"FineLineSearch",
	"AssembleLine",
	"CheckViaWireConnect",
	"FilterObj"
};

ExtractParam::ExtractParam()
{
	memset(method_count, 0, sizeof(method_count));
	depth = 0;
}

/*
For ParamItem name, it will return one single param
For ParamSet,it will expand param set to multiple params
*/
void ExtractParam::get_param(string name, vector<ParamItem> & _params)
{
	if (depth == 0) {
		_params.clear();
		qInfo("get_param for action %s", name.c_str());
	}
	map<string, ParamSet>::iterator it = param_sets.find(name);
	if (it != param_sets.end()) { // it is paramset name
		vector<string> names;
		names = it->second.names;
		for (int i = 0; i < names.size(); i++) {
			depth++;
			get_param(names[i], _params); //call get_param recursively
			depth--;
		}
	}
	else {
		map<string, ParamItem>::iterator it = params.find(name);
		if (it != params.end()) //it is param name
			_params.push_back(it->second);
		else
			qCritical("get_param name=%s not exist", name.c_str());
	}
}

/*
For ParamSet,it won't expand param set to multiple params
*/
void ExtractParam::get_param_sets(string name, vector<string> & names)
{
	names.clear();
	map<string, ParamSet>::iterator it = param_sets.find(name);
	if (it != param_sets.end())
		names = it->second.names;
}

void ExtractParam::get_param_set_list(vector<string> & names)
{
	names.clear();
	for (map<string, ParamSet>::iterator it = param_sets.begin(); it != param_sets.end(); it++)
		names.push_back(it->first);
}

string ExtractParam::set_param(int pi0, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, int pi8, float pf0)
{
	int method = pi1 >> 16 & 0xff;
	switch (method) {
	case PP_SET_PARAM:
		if (pi0 == -1)
			method = LayerInfo;
		else {
			if ((pi2 & 0x80) == 0x80)
				method = ViaInfo;
			else
				method = WireInfo;
		}
		break;
	case PP_RGB2GRAY:
		method = Rgb2Gray;
		break;
	case PP_COMPUTE_MIN_STAT:
		method = MinStat;
		break;
	case PP_ADJUST_GRAY_LVL:
		method = GrayLevel;
		break;
	case PP_COARSE_LINE_SEARCH:
		method = CoarseLineSearch;
		break;
	case PP_COARSE_VIA_MASK:
		method = CoarseViaSearch;
		break;
	case PP_FINE_VIA_SEARCH:
		method = FineViasSearch;
		break;
	case PP_REMOVE_VIA:
		method = RemoveVia;
		break;
	case PP_FINE_SEARCH_MASK:
		method = HotlineMask;
		break;
	case PP_FINE_LINE_SEARCH:
		method = FineLineSearch;
		break;
	case PP_ASSEMBLE:
		method = AssembleLine;
		break;
	case PP_CHECK_VIA_WIRE_CONNECT:
		method = CheckViaWireConnect;
	case PP_OBJ_PROCESS:
		if ((pi2 & 0xff) ==0)
			method = FilterObj;
		break;
	default:
		qCritical("set_param unknow method");
		return "";
	}
	stringstream stream;
	stream << method_count[method];
	string name = method_name[method] + '_' + stream.str();
	ParamItem param(pi0, pi1, pi2, pi3, pi4, pi5, pi6, pi7, pi8, pf0);
	params[name] = param;
	method_count[method]++;
	return name;
}

string ExtractParam::set_param_sets(string name, vector<string> & _param_set)
{
	ParamSet param_set(_param_set);
	for (int i = 0; i < sizeof(method_name) / sizeof(method_name[0]); i++) 
		if (name.find(method_name[i]) == 0) {
			qCritical("set_param_sets name=%s should not begin with %s", name.c_str(), method_name[i].c_str());
			return "";
		}
	map<string, ParamSet>::iterator it = param_sets.find(name);
	if (it == param_sets.end())
		param_sets[name] = param_set;
	else {
		qCritical("set_param_sets name=%s already exist", name.c_str());
		return "";
	}
	return name;
}

bool ExtractParam::read_file(string filename)
{
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	params.clear();
	param_sets.clear();
	memset(method_count, 0, sizeof(method_count));
	depth = 0;
	FileNode param_items = fs["ParamItems"];
	for (FileNodeIterator it = param_items.begin(); it != param_items.end(); it++) {
		ParamItem param;
		string name = (string)(*it)["name"];
		int method = -1;
		bool check_pass = true;
		for (int i = 0; i < sizeof(method_name) / sizeof(method_name[0]); i++)
		if (name.find(method_name[i]) == 0) {
			method = i;
			string num = name.substr(method_name[i].length() + 1);
			method_count[method] = max(method_count[method], atoi(num.c_str()));
			break;
		}
		if (method == -1) {
			qCritical("ParamItems file error, name =%s", name.c_str());
			continue;
		}
		switch (method) {
		case LayerInfo: {
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int gs = (int) (*it)["gs"];
				int border_size = (int)(*it)["border_size"];
				int computer_border = (int)(*it)["computer_border"];
				if (gs > 256 || computer_border > 256 || border_size > 256) {
					qCritical("ParamItems file error, name=%s, gs=%d, computer_border=%d, border_size=%d", 
						name.c_str(), gs, computer_border, border_size);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_SET_PARAM << 16;
				param.pi[2] = 1 << 24 | border_size << 16 | computer_border << 8 | gs;
			}
			break;

		case ViaInfo: {				
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int shape = (int)(*it)["shape"];
				int type = (int)(*it)["type"];
				int subtype = (int)(*it)["subtype"];
				int guard = (int)(*it)["guard"];
				int remove_rd = (int)(*it)["remove_rd"];
				int arfactor = (int)(*it)["arfactor"];
				int pair_distance = (int)(*it)["pair_d"];
				int connect_d = (int)(*it)["connect_d"];
				int cgray_d = (int)(*it)["cgray_d"];
				int cgray_ratio = (int)(*it)["cgray_ratio"];
				int connect_rd = (int)(*it)["connect_rd"];
				int rd0 = (int)(*it)["rd0"];
				int rd1 = (int)(*it)["rd1"];
				int rd2 = (int)(*it)["rd2"];
				int rd3 = (int)(*it)["rd3"];
				int gray0 = (int)(*it)["gray0"];
				int gray1 = (int)(*it)["gray1"];
				int gray2 = (int)(*it)["gray2"];
				int gray3 = (int)(*it)["gray3"];				
				if (shape > 255 || type > 255 || subtype > 255) {
					qCritical("ParamItems file error, name=%s shape=%d, type=%d, subtype=%d", 
						name.c_str(), shape, type, subtype);
					check_pass = false;
				}
				if (arfactor > 255 || remove_rd > 255 || guard > 255) {
					qCritical("ParamItems file error, name=%s remove_rd=%d, arfactor=%d, guard=%d", 
						name.c_str(), remove_rd, arfactor, guard);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_SET_PARAM << 16;
				param.pi[2] = subtype << 16 | type << 8 | shape;
				param.pi[3] = pair_distance << 24 | arfactor << 16 | remove_rd << 8 | guard;
				if (rd3 > 255 || rd2 > 255 || rd1 > 255 || rd0 > 255) {
					qCritical("ParamItems file error, name=%s rd0=%d, rd1=%d, rd2=%d, rd3=%d", 
						name.c_str(), rd0, rd1, rd2, rd3);
					check_pass = false;
				}
				param.pi[4] = rd3 << 24 | rd2 << 16 | rd1 << 8 | rd0;
				if (gray3 > 255 || gray2 > 255 || gray1 > 255 || gray0 > 255) {
					qCritical("ParamItems file error, name=%s gray0=%d, gray1=%d, gray2=%d, gray3=%d", 
						name.c_str(), gray0, gray1, gray2, gray3);
					check_pass = false;
				}
				if (connect_d > 255 || cgray_d > 255 || cgray_ratio > 255 || connect_rd > 255) {
					qCritical("ParamItems file error, name=%s connect_d=%d, cgray_d=%d, cgray_ratio=%d, connect_rd=%d",
						name.c_str(), connect_d, cgray_d, cgray_ratio, connect_rd);
					check_pass = false;
				}
				param.pi[5] = gray3 << 24 | gray2 << 16 | gray1 << 8 | gray0;
				param.pi[6] = connect_d <<24 | cgray_d << 16 | cgray_ratio << 8 | connect_rd;
			}
			break;

		case WireInfo:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int pattern = (int)(*it)["pattern"];
				int type = (int)(*it)["type"];
				int subtype = (int)(*it)["subtype"];
				int guard = (int)(*it)["guard"];
				int arfactor = (int)(*it)["arfactor"];
				int w_wide = (int)(*it)["w_wide"];
				int w_wide1 = (int)(*it)["w_wide1"];
				int w_high = (int)(*it)["w_high"];
				int w_high1 = (int)(*it)["w_high1"];
				int i_wide = (int)(*it)["i_wide"];
				int i_high = (int)(*it)["i_high"];
				int dis_vw0 = (int)(*it)["dis_vw0"];
				int dis_vw1 = (int)(*it)["dis_vw1"];
				int wgid0 = (int)(*it)["wgid0"];
				int wgid1 = (int)(*it)["wgid1"];
				int wgid2 = (int)(*it)["wgid2"];
				int wgid3 = (int)(*it)["wgid3"];
				int wgid4 = (int)(*it)["wgid4"];
				int wgid5 = (int)(*it)["wgid5"];
				int wgid6 = (int)(*it)["wgid6"];
				int wgid7 = (int)(*it)["wgid7"];
				int uni_id = (int)(*it)["uni_id"];
				if (pattern > 255 || type > 255 || subtype > 255) {
					qCritical("ParamItems file error, name=%s, pattern=%d, type=%d, subtype=%d", 
						name.c_str(), pattern, type, subtype);
					check_pass = false;
				}
				if (arfactor > 255 || guard > 255) {
					qCritical("ParamItems file error, name=%s, arfactor=%d, guard=%d", name.c_str(), arfactor, guard);
					check_pass = false;
				}
				if (w_wide > 255 || w_wide1 > 255 || w_high > 255 || w_high1 > 255) {
					qCritical("ParamItems file error, name=%s, w_wide=%d, w_wide1=%d, w_high=%d, w_high1=%d",
						name.c_str(), w_wide, w_wide1, w_high, w_high1);
					check_pass = false;
				}
				if (wgid0 > 255 || wgid1 > 255 || wgid2 > 255 || wgid3 > 255) {
					qCritical("ParamItems file error, name=%s, wgid0=%d, wgid1=%d, wgid2=%d, wgid3=%d",
						name.c_str(), wgid0, wgid1, wgid2, wgid3);
					check_pass = false;
				}
				if (wgid4 > 255 || wgid5 > 255 || wgid6 > 255 || wgid7 > 255) {
					qCritical("ParamItems file error, name=%s, wgid4=%d, wgid5=%d, wgid6=%d, wgid7=%d",
						name.c_str(), wgid4, wgid5, wgid6, wgid7);
					check_pass = false;
				}
				if (uni_id > 65535) {
					qCritical("ParamItems file error, name=%s, uni_id=%d",
						name.c_str(), uni_id);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_SET_PARAM << 16;
				param.pi[2] = subtype << 16 | type << 8 | pattern;
				param.pi[3] = arfactor << 8 | guard;				
				param.pi[4] = w_wide << 24 | w_wide1 << 16 | w_high << 8 | w_high1;
				param.pi[5] = dis_vw0 << 24 | dis_vw1 << 16 | i_wide << 8 | i_high;
				param.pi[6] = wgid3 << 24 | wgid2 << 16 | wgid1 << 8 | wgid0;
				param.pi[7] = wgid7 << 24 | wgid6 << 16 | wgid5 << 8 | wgid4;
				param.pi[8] = uni_id;
			}
			break;

		case Rgb2Gray:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int red = (int)(*it)["red"];
				int green = (int)(*it)["green"];
				int blue = (int)(*it)["blue"];
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_RGB2GRAY << 16;
				param.pi[2] = red;
				param.pi[3] = green;
				param.pi[4] = blue;
			}
			break;

		case MinStat:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int stat_grid = (int)(*it)["stat_grid"];
				int min_percent = (int)(*it)["min_percent"];
				int grid_win = (int)(*it)["grid_win"];
				int ksize = (int)(*it)["ksize"];
				int filter_method = (int)(*it)["filter_method"];
				if (min_percent >= 100) {
					qCritical("ParamItems file error, name=%s, min_percent=%d", name.c_str(), min_percent);
					check_pass = false;
				}
				if (stat_grid >= 1000 || grid_win > 20) {
					qCritical("ParamItems file error, name=%s, stat_grid=%d, grid_win=%d", name.c_str(), stat_grid, grid_win);
						check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_COMPUTE_MIN_STAT << 16;
				param.pi[2] = stat_grid;
				param.pi[3] = min_percent;
				param.pi[4] = grid_win;
				param.pi[5] = ksize;
				param.pi[6] = filter_method;
			}
			break;

		case GrayLevel:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_tp = (int)(*it)["opidx_tp"];
				int low_sep_min = (int)(*it)["low_sep_min"];
				int low_sep_max = (int)(*it)["low_sep_max"];
				int low_win = (int)(*it)["low_win"];
				int low_dis_min = (int)(*it)["low_dis_min"];
				int low_k0 = (int)(*it)["low_k0"];
				int low_k1 = (int)(*it)["low_k1"];
				int low_k2 = (int)(*it)["low_k2"];

				int mid_sep_min = (int)(*it)["mid_sep_min"];
				int mid_sep_max = (int)(*it)["mid_sep_max"];
				int mid_win = (int)(*it)["mid_win"];
				int mid_dis_min = (int)(*it)["mid_dis_min"];
				int mid_k0 = (int)(*it)["mid_k0"];
				int mid_k1 = (int)(*it)["mid_k1"];
				int mid_k2 = (int)(*it)["mid_k2"];

				int high_sep_min = (int)(*it)["high_sep_min"];
				int high_sep_max = (int)(*it)["high_sep_max"];
				int high_win = (int)(*it)["high_win"];
				int high_dis_min = (int)(*it)["high_dis_min"];
				int high_k0 = (int)(*it)["high_k0"];
				int high_k1 = (int)(*it)["high_k1"];
				int high_k2 = (int)(*it)["high_k2"];

				if (opidx_tp >= 16) {
					qCritical("ParamItems file error, name=%s, opidx_tp=%d", name.c_str(), opidx_tp);
					check_pass = false;
				}

				if (low_dis_min > 255 || low_win > 255 || low_sep_max > 255 || low_sep_min > 255) {
					qCritical("ParamItems file error, name=%s, low_dis_min=%d, low_win=%d, low_sep_max=%d, low_sep_min=%d", 
						name.c_str(), low_dis_min, low_win, low_sep_max, low_sep_min);
					check_pass = false;
				}

				if (mid_dis_min > 255 || mid_win > 255 || mid_sep_max > 255 || mid_sep_min > 255) {
					qCritical("ParamItems file error, name=%s, mid_dis_min=%d, mid_win=%d, mid_sep_max=%d, mid_sep_min=%d",
						name.c_str(), mid_dis_min, mid_win, mid_sep_max, mid_sep_min);
					check_pass = false;
				}
				
				if (high_dis_min > 255 || high_win > 255 || high_sep_max > 255 || high_sep_min > 255) {
					qCritical("ParamItems file error, name=%s, high_dis_min=%d, high_win=%d, high_sep_max=%d, high_sep_min=%d",
						name.c_str(), high_dis_min, high_win, high_sep_max, high_sep_min);
					check_pass = false;
				}

				if (low_k2 > 255 || low_k1 > 255 || low_k0 > 255) {
					qCritical("ParamItems file error, name=%s, low_k2=%d, low_k1=%d, low_k0=%d",
						name.c_str(), low_k2, low_k1, low_k0);
					check_pass = false;
				}

				if (mid_k2 > 255 || mid_k1 > 255 || mid_k0 > 255) {
					qCritical("ParamItems file error, name=%s, mid_k2=%d, mid_k1=%d, mid_k0=%d",
						name.c_str(), mid_k2, mid_k1, mid_k0);
					check_pass = false;
				}

				if (high_k2 > 255 || high_k1 > 255 || high_k0 > 255) {
					qCritical("ParamItems file error, name=%s, high_k2=%d, high_k1=%d, high_k0=%d",
						name.c_str(), high_k2, high_k1, high_k0);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_ADJUST_GRAY_LVL << 16 | opidx_tp;
				param.pi[2] = low_dis_min << 24 | low_win << 16 | low_sep_max << 8 | low_sep_min;
				param.pi[3] = mid_dis_min << 24 | mid_win << 16 | mid_sep_max << 8 | mid_sep_min;
				param.pi[4] = high_dis_min << 24 | high_win << 16 | high_sep_max << 8 | high_sep_min;
				param.pi[5] = low_k2 << 16 | low_k1 << 8 | low_k0;
				param.pi[6] = mid_k2 << 16 | mid_k1 << 8 | mid_k0;
				param.pi[7] = high_k2 << 16 | high_k1 << 8 | high_k0;
			}
			break;

		case CoarseLineSearch:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_tp = (int)(*it)["opidx_tp"];
				int opidx_shadow_prob = (int)(*it)["opidx_shadow_prob"];
				int opidx_rv_mask = (int)(*it)["opidx_rv_mask"];
				int wlong0 = (int)(*it)["wlong0"];
				int wlong1 = (int)(*it)["wlong1"];
				int inc0 = (int)(*it)["inc0"];
				int inc1 = (int)(*it)["inc1"];
				int wnum = (int)(*it)["wnum"];
				int th = (int)(*it)["th"];
				int search_opt = (int)(*it)["search_opt"];
				int update_prob = (int)(*it)["update_prob"];
				int subtype0 = (int)(*it)["subtype0"];
				int type0 = (int)(*it)["type0"];				
				int dir0 = (int)(*it)["dir0"];
				int pattern0 = (int)(*it)["pattern0"];
				int subtype1 = (int)(*it)["subtype1"];
				int type1 = (int)(*it)["type1"];
				int dir1 = (int)(*it)["dir1"];
				int pattern1 = (int)(*it)["pattern1"];
				int subtype2 = (int)(*it)["subtype2"];
				int type2 = (int)(*it)["type2"];
				int dir2 = (int)(*it)["dir2"];
				int pattern2 = (int)(*it)["pattern2"];
				int subtype3 = (int)(*it)["subtype3"];
				int type3 = (int)(*it)["type3"];
				int dir3 = (int)(*it)["dir3"];
				int pattern3 = (int)(*it)["pattern3"];

				param.pi[0] = layer;
				if (opidx_tp >= 16 || opidx_shadow_prob >= 16 || opidx_rv_mask >=16) {
					qCritical("ParamItems file error, name=%s, opidx_tp=%d, opidx_shadow_prob=%d, opidx_rv_mask=%d", 
						name.c_str(), opidx_tp, opidx_shadow_prob, opidx_rv_mask);
					check_pass = false;
				}

				if (inc1 > 255 || inc0 > 255 || wlong1 > 255 || wlong0 > 255) {
					qCritical("ParamItems file error, name=%s, inc1=%d, inc0=%d, wlong1=%d, wlong0=%d",
						name.c_str(), inc1, inc0, wlong1, wlong0);
					check_pass = false;
				}

				if (update_prob > 255 || search_opt > 255 || th > 255 || wnum > 255) {
					qCritical("ParamItems file error, name=%s, update_prob=%d, search_opt=%d, th=%d, wnum=%d",
						name.c_str(), update_prob, search_opt, th, wnum);
					check_pass = false;
				}

				if (dir0 > 255 || type0 > 255 || subtype0 > 255 || pattern0 > 255) {
					qCritical("ParamItems file error, name=%s, dir0=%d, type0=%d, subtype0=%d, pattern0=%d",
						name.c_str(), dir0, type0, subtype0, pattern0);
					check_pass = false;
				}

				if (dir1 > 255 || type1 > 255 || subtype1 > 255 || pattern1 > 255) {
					qCritical("ParamItems file error, name=%s, dir1=%d, type1=%d, subtype1=%d, pattern1=%d",
						name.c_str(), dir1, type1, subtype1, pattern1);
					check_pass = false;
				}

				if (dir2 > 255 || type2 > 255 || subtype2 > 255 || pattern2 > 255) {
					qCritical("ParamItems file error, name=%s, dir2=%d, type2=%d, subtype2=%d, pattern2=%d",
						name.c_str(), dir2, type2, subtype2, pattern2);
					check_pass = false;
				}

				if (dir3 > 255 || type3 > 255 || subtype3 > 255 || pattern3 > 255) {
					qCritical("ParamItems file error, name=%s, dir3=%d, type3=%d, subtype3=%d, pattern3=%d",
						name.c_str(), dir3, type3, subtype3, pattern3);
					check_pass = false;
				}
				param.pi[1] = debug_opt << 24 | PP_COARSE_LINE_SEARCH << 16 | opidx_rv_mask << 8 | opidx_shadow_prob << 4 | opidx_tp;
				param.pi[2] = inc1 << 24 | inc0 << 16 | wlong1 << 8 | wlong0;
				param.pi[3] = update_prob << 24 | search_opt << 16 | th << 8 | wnum;
				param.pi[4] = subtype0 << 24 | pattern0 << 16 | dir0 << 8 | type0;
				param.pi[5] = subtype1 << 24 | pattern1 << 16 | dir1 << 8 | type1;
				param.pi[6] = subtype2 << 24 | pattern2 << 16 | dir2 << 8 | type2;
				param.pi[7] = subtype3 << 24 | pattern3 << 16 | dir3 << 8 | type3;
			}
			break;

		case CoarseViaSearch:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_via_mask = (int)(*it)["opidx_via_mask"];
				int vnum = (int)(*it)["vnum"];
				int subtype0 = (int)(*it)["subtype0"];
				int wide0 = (int)(*it)["wide0"];
				int percent0 = (int)(*it)["percent0"];
				int subtype1 = (int)(*it)["subtype1"];
				int wide1 = (int)(*it)["wide1"];
				int percent1 = (int)(*it)["percent1"];
				int subtype2 = (int)(*it)["subtype2"];
				int wide2 = (int)(*it)["wide2"];
				int percent2 = (int)(*it)["percent2"];
				int subtype3 = (int)(*it)["subtype3"];
				int wide3 = (int)(*it)["wide3"];
				int percent3 = (int)(*it)["percent3"];
				int subtype4 = (int)(*it)["subtype4"];
				int wide4 = (int)(*it)["wide4"];
				int percent4 = (int)(*it)["percent4"];

				if (opidx_via_mask >= 16) {
					qCritical("ParamItems file error, name=%s, opidx_via_mask=%d", name.c_str(), opidx_via_mask);
					check_pass = false;
				}
				if (percent0 > 255 || wide0 > 255 || subtype0 > 255) {
					qCritical("ParamItems file error, name=%s, percent0=%d, wide0=%d, subtype0=%d",
						name.c_str(), percent0, wide0, subtype0);
					check_pass = false;
				}

				if (percent1 > 255 || wide1 > 255 || subtype1 > 255) {
					qCritical("ParamItems file error, name=%s, percent1=%d, wide1=%d, subtype1=%d",
						name.c_str(), percent1, wide1, subtype1);
					check_pass = false;
				}

				if (percent2 > 255 || wide2 > 255 || subtype2 > 255) {
					qCritical("ParamItems file error, name=%s, percent2=%d, wide2=%d, subtype2=%d",
						name.c_str(), percent2, wide2, subtype2);
					check_pass = false;
				}

				if (percent3 > 255 || wide3 > 255 || subtype3 > 255) {
					qCritical("ParamItems file error, name=%s, percent3=%d, wide3=%d, subtype3=%d",
						name.c_str(), percent3, wide3, subtype3);
					check_pass = false;
				}

				if (percent4 > 255 || wide4 > 255 || subtype4 > 255) {
					qCritical("ParamItems file error, name=%s, percent4=%d, wide4=%d, subtype4=%d",
						name.c_str(), percent4, wide4, subtype4);
					check_pass = false;
				}

				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_COARSE_VIA_MASK << 16 | opidx_via_mask;
				param.pi[2] = vnum;
				param.pi[3] = percent0 << 16 | wide0 << 8 | subtype0;
				param.pi[4] = percent1 << 16 | wide1 << 8 | subtype1;
				param.pi[5] = percent2 << 16 | wide2 << 8 | subtype2;
				param.pi[6] = percent3 << 16 | wide3 << 8 | subtype3;
				param.pi[7] = percent4 << 16 | wide4 << 8 | subtype4;
			}
			break;

		case FineViasSearch:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_tp = (int)(*it)["opidx_tp"];
				int opidx_via_mask = (int)(*it)["opidx_via_mask"];
				int opidx_via_info = (int)(*it)["opidx_via_info"];
				int opidx_shadow_prob = (int)(*it)["opidx_shadow_prob"];
				int vnum = (int)(*it)["vnum"];
				int update_fv = (int)(*it)["update_fv"];
				int subtype0 = (int)(*it)["subtype0"];
				int type0 = (int)(*it)["type0"];
				int th0 = (int)(*it)["th0"];
				int pattern0 = (int)(*it)["pattern0"];
				int subtype1 = (int)(*it)["subtype1"];				
				int type1 = (int)(*it)["type1"];
				int th1 = (int)(*it)["th1"];
				int pattern1 = (int)(*it)["pattern1"];
				int subtype2 = (int)(*it)["subtype2"];
				int type2 = (int)(*it)["type2"];
				int th2 = (int)(*it)["th2"];
				int pattern2 = (int)(*it)["pattern2"];
				int subtype3 = (int)(*it)["subtype3"];
				int type3 = (int)(*it)["type3"];
				int th3 = (int)(*it)["th3"];
				int pattern3 = (int)(*it)["pattern3"];
				int subtype4 = (int)(*it)["subtype4"];
				int type4 = (int)(*it)["type4"];
				int th4 = (int)(*it)["th4"];
				int pattern4 = (int)(*it)["pattern4"];

				if (opidx_tp >= 16 || opidx_via_mask >= 16 || opidx_via_info >= 16 || opidx_shadow_prob >= 16) {
					qCritical("ParamItems file error, name=%s, opidx_tp=%d, opidx_via_mask=%d, opidx_via_info=%d, opidx_shadow_prob=%d", 
						name.c_str(), opidx_tp, opidx_via_mask, opidx_via_info, opidx_shadow_prob);
					check_pass = false;
				}

				if (pattern0 > 255 || th0 > 255 || subtype0 > 255 || type0 > 255) {
					qCritical("ParamItems file error, name=%s, pattern0=%d, th0=%d, subtype0=%d, type0=%d",
						name.c_str(), pattern0, th0, subtype0, type0);
					check_pass = false;
				}

				if (pattern1 > 255 || th1 > 255 || subtype1 > 255 || type1 > 255) {
					qCritical("ParamItems file error, name=%s, pattern1=%d, th1=%d, subtype1=%d, type1=%d",
						name.c_str(), pattern1, th1, subtype1, type1);
					check_pass = false;
				}

				if (pattern2 > 255 || th2 > 255 || subtype2 > 255 || type2 > 255) {
					qCritical("ParamItems file error, name=%s, pattern2=%d, th2=%d, subtype2=%d, type2=%d",
						name.c_str(), pattern2, th2, subtype2, type2);
					check_pass = false;
				}

				if (pattern3 > 255 || th3 > 255 || subtype3 > 255 || type3 > 255) {
					qCritical("ParamItems file error, name=%s, pattern3=%d, th3=%d, subtype3=%d, type3=%d",
						name.c_str(), pattern3, th3, subtype3, type3);
					check_pass = false;
				}

				if (pattern4 > 255 || th4 > 255 || subtype4 > 255 || type4 > 255) {
					qCritical("ParamItems file error, name=%s, pattern4=%d, th4=%d, subtype4=%d, type4=%d",
						name.c_str(), pattern4, th4, subtype4, type4);
					check_pass = false;
				}

				if (subtype4 > 255 || type4 > 255 || vnum > 255 ) {
					qCritical("ParamItems file error, name=%s, subtype4=%d, type4=%d, vnum=%d",
						name.c_str(), subtype4, type4, vnum);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_FINE_VIA_SEARCH << 16 | opidx_shadow_prob << 12 | opidx_via_info << 8 | opidx_via_mask << 4 | opidx_tp;
				param.pi[2] = update_fv << 8 | vnum;
				param.pi[3] = pattern0 << 24 | th0 << 16 | subtype0 << 8 | type0;
				param.pi[4] = pattern1 << 24 | th1 << 16 | subtype1 << 8 | type1;
				param.pi[5] = pattern2 << 24 | th2 << 16 | subtype2 << 8 | type2;
				param.pi[6] = pattern3 << 24 | th3 << 16 | subtype3 << 8 | type3;
				param.pi[7] = pattern4 << 24 | th4 << 16 | subtype4 << 8 | type4;
			}
			break;

		case RemoveVia:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_via_info = (int)(*it)["opidx_via_info"];
				int opidx_rv_mask = (int)(*it)["opidx_rv_mask"];
				int opidx_tp = (int)(*it)["opidx_tp"];
				int vnum = (int)(*it)["vnum"];
				int check_len = (int)(*it)["check_len"];
				int default_dir = (int)(*it)["default_dir"];
				int remove_opt = (int)(*it)["remove_opt"];
				int subtype0 = (int)(*it)["subtype0"];
				int type0 = (int)(*it)["type0"];
				int pattern0 = (int)(*it)["pattern0"];
				int subtype1 = (int)(*it)["subtype1"];
				int type1 = (int)(*it)["type1"];
				int pattern1 = (int)(*it)["pattern1"];
				int subtype2 = (int)(*it)["subtype2"];
				int type2 = (int)(*it)["type2"];
				int pattern2 = (int)(*it)["pattern2"];
				int subtype3 = (int)(*it)["subtype3"];
				int type3 = (int)(*it)["type3"];
				int pattern3 = (int)(*it)["pattern3"];
				int subtype4 = (int)(*it)["subtype4"];
				int type4 = (int)(*it)["type4"];
				int pattern4 = (int)(*it)["pattern4"];

				if (opidx_rv_mask >= 16 || opidx_via_info >= 16 || opidx_tp >= 16) {
					qCritical("ParamItems file error, name=%s, opidx_rv_mask=%d, opidx_via_info=%d, opidx_tp=%d",
						name.c_str(), opidx_rv_mask, opidx_via_info, opidx_tp);
					check_pass = false;
				}

				if (remove_opt > 255 || default_dir > 255 || check_len > 255 || vnum > 255) {
					qCritical("ParamItems file error, name=%s, clear_mask=%d, default_dir=%d, check_len=%d, vnum=%d",
						name.c_str(), remove_opt, default_dir, check_len, vnum);
					check_pass = false;
				}
				if (pattern0 > 255 || subtype0 > 255 || type0 > 255) {
					qCritical("ParamItems file error, name=%s, pattern0=%d, subtype0=%d, type0=%d",
						name.c_str(), pattern0, subtype0, type0);
					check_pass = false;
				}

				if (pattern1 > 255 || subtype1 > 255 || type1 > 255) {
					qCritical("ParamItems file error, name=%s, pattern1=%d, subtype1=%d, type1=%d",
						name.c_str(), pattern1, subtype1, type1);
					check_pass = false;
				}

				if (pattern2 > 255 || subtype2 > 255 || type2 > 255) {
					qCritical("ParamItems file error, name=%s, pattern2=%d, subtype2=%d, type2=%d",
						name.c_str(), pattern2, subtype2, type2);
					check_pass = false;
				}

				if (pattern3 > 255 || subtype3 > 255 || type3 > 255) {
					qCritical("ParamItems file error, name=%s, pattern3=%d, subtype3=%d, type3=%d",
						name.c_str(), pattern3, subtype3, type3);
					check_pass = false;
				}

				if (pattern4 > 255 || subtype4 > 255 || type4 > 255) {
					qCritical("ParamItems file error, name=%s, pattern4=%d, subtype4=%d, type4=%d",
						name.c_str(), pattern4, subtype4, type4);
					check_pass = false;
				}

				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_REMOVE_VIA << 16 | opidx_rv_mask << 4 | opidx_via_info;
				param.pi[2] = remove_opt << 24 | default_dir << 16 | check_len << 8 | vnum;
				param.pi[3] = pattern0 << 16 | subtype0 << 8 | type0;
				param.pi[4] = pattern1 << 16 | subtype1 << 8 | type1;
				param.pi[5] = pattern2 << 16 | subtype2 << 8 | type2;
				param.pi[6] = pattern3 << 16 | subtype3 << 8 | type3;
				param.pi[7] = pattern4 << 16 | subtype4 << 8 | type4;
			}
			break;

		case HotlineMask:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_hl_mask = (int)(*it)["opidx_hl_mask"];
				int wnum = (int)(*it)["wnum"];
				int hotline_opt = (int)(*it)["hotline_opt"];
				int shape0 = (int)(*it)["shape0"];
				int type0 = (int)(*it)["type0"];
				int clong0 = (int)(*it)["clong0"];
				int cwide0 = (int)(*it)["cwide0"];
				int subtype0 = (int)(*it)["subtype0"];
				int extend0 = (int)(*it)["extend0"];
				int shape1 = (int)(*it)["shape1"];
				int type1 = (int)(*it)["type1"];
				int clong1 = (int)(*it)["clong1"];
				int cwide1 = (int)(*it)["cwide1"];
				int subtype1 = (int)(*it)["subtype1"];
				int extend1 = (int)(*it)["extend1"];
				int shape2 = (int)(*it)["shape2"];
				int type2 = (int)(*it)["type2"];
				int clong2 = (int)(*it)["clong2"];
				int cwide2 = (int)(*it)["cwide2"];
				int subtype2 = (int)(*it)["subtype2"];
				int extend2 = (int)(*it)["extend2"];

				if (opidx_hl_mask >= 16 || hotline_opt >= 256 || wnum >= 256) {
					qCritical("ParamItems file error, name=%s, opidx_hl_mask=%d, hotline_opt=%d, wnum=%d",
						name.c_str(), opidx_hl_mask, hotline_opt, wnum);
					check_pass = false;
				}

				if (cwide0 > 255 || clong0 > 255 || type0 > 255 || shape0 > 255) {
					qCritical("ParamItems file error, name=%s, cwide0=%d, clong0=%d, type0=%d, shape0=%d",
						name.c_str(), cwide0, clong0, type0, shape0);
					check_pass = false;
				}

				if (cwide1 > 255 || clong1 > 255 || type1 > 255 || shape1 > 255) {
					qCritical("ParamItems file error, name=%s, cwide1=%d, clong1=%d, type1=%d, shape1=%d",
						name.c_str(), cwide1, clong1, type1, shape1);
					check_pass = false;
				}

				if (cwide2 > 255 || clong2 > 255 || type2 > 255 || shape2 > 255) {
					qCritical("ParamItems file error, name=%s, cwide2=%d, clong2=%d, type2=%d, shape2=%d",
						name.c_str(), cwide2, clong2, type2, shape2);
					check_pass = false;
				}

				if (extend0 > 255 || subtype0 > 255 || extend1 > 255 || subtype1 > 255) {
					qCritical("ParamItems file error, name=%s, extend0=%d, subtype0=%d, extend1=%d, subtype1=%d",
						name.c_str(), extend0, subtype0, extend1, subtype1);
					check_pass = false;
				}

				if (extend2 > 255 || subtype2 > 255) {
					qCritical("ParamItems file error, name=%s, extend2=%d, subtype2=%d",
						name.c_str(), extend2, subtype2);
					check_pass = false;
				}

				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_FINE_SEARCH_MASK << 16  | opidx_hl_mask;
				param.pi[2] = hotline_opt << 8 | wnum;
				param.pi[3] = cwide0 << 24 | clong0 << 16 | type0 << 8 | shape0;
				param.pi[4] = extend0 << 8 | subtype0;
				param.pi[5] = cwide1 << 24 | clong1 << 16 | type1 << 8 | shape1;
				param.pi[6] = extend1 << 8 | subtype1;
				param.pi[7] = cwide2 << 24 | clong2 << 16 | type2 << 8 | shape2;
				param.pi[8] = extend2 << 8 | subtype2;
			}
			break;

		case FineLineSearch:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_tp = (int)(*it)["opidx_tp"];
				int opidx_hl_mask = (int)(*it)["opidx_hl_mask"];
				int opidx_rv_mask = (int)(*it)["opidx_rv_mask"];
				int opidx_via_info = (int)(*it)["opidx_via_info"];
				int wnum = (int)(*it)["wnum"];
				int search_opt = (int)(*it)["search_opt"];
				int subtype0 = (int)(*it)["subtype0"];
				int type0 = (int)(*it)["type0"];
				int pattern0 = (int)(*it)["pattern0"];
				int subtype1 = (int)(*it)["subtype1"];
				int type1 = (int)(*it)["type1"];
				int pattern1 = (int)(*it)["pattern1"];
				int subtype2 = (int)(*it)["subtype2"];
				int type2 = (int)(*it)["type2"];
				int pattern2 = (int)(*it)["pattern2"];
				int subtype3 = (int)(*it)["subtype3"];
				int type3 = (int)(*it)["type3"];
				int pattern3 = (int)(*it)["pattern3"];
				
				if (opidx_tp >= 16 || opidx_hl_mask >= 16 || opidx_rv_mask >= 16 || opidx_via_info >= 16 || wnum >= 256 || search_opt >= 256) {
					qCritical("ParamItems file error, name=%s, opidx_tp=%d, opidx_hl_mask=%d, opidx_rv_mask=%d, opidx_via_info=%d, wnum=%d, search_opt=%d",
						name.c_str(), opidx_tp, opidx_hl_mask, opidx_rv_mask, opidx_via_info, wnum, search_opt);
					check_pass = false;
				}

				if (pattern0 > 255 || subtype0 > 255 || type0 > 255) {
					qCritical("ParamItems file error, name=%s, pattern0=%d, subtype0=%d, type0=%d",
						name.c_str(), pattern0, subtype0, type0);
					check_pass = false;
				}

				if (pattern1 > 255 || subtype1 > 255 || type1 > 255) {
					qCritical("ParamItems file error, name=%s, pattern1=%d, subtype1=%d, type1=%d",
						name.c_str(), pattern1, subtype1, type1);
					check_pass = false;
				}

				if (pattern2 > 255 || subtype2 > 255 || type2 > 255) {
					qCritical("ParamItems file error, name=%s, pattern2=%d, subtype2=%d, type2=%d",
						name.c_str(), pattern2, subtype2, type2);
					check_pass = false;
				}

				if (pattern3 > 255 || subtype3 > 255 || type3 > 255) {
					qCritical("ParamItems file error, name=%s, pattern3=%d, subtype3=%d, type3=%d",
						name.c_str(), pattern3, subtype3, type3);
					check_pass = false;
				}

				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_FINE_LINE_SEARCH << 16 | opidx_via_info << 12 | opidx_rv_mask << 8 | opidx_hl_mask << 4 | opidx_tp;
				param.pi[2] = search_opt << 16 | wnum;
				param.pi[3] = subtype0 << 16 | type0 << 8 | pattern0;
				param.pi[4] = subtype1 << 16 | type1 << 8 | pattern1;
				param.pi[5] = subtype2 << 16 | type2 << 8 | pattern2;
				param.pi[6] = subtype3 << 16 | type3 << 8 | pattern3;
			}
			break;

		case AssembleLine:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_via_info = (int)(*it)["opidx_via_info"];				
				int wnum = (int)(*it)["wnum"];
				int assemble_opt = (int)(*it)["assemble_opt"];
				int type0 = (int)(*it)["type0"];
				int cwide0 = (int)(*it)["cwide0"];
				int clong_shu0 = (int)(*it)["clong_shu0"];
				int clong_heng0 = (int)(*it)["clong_heng0"];
				int type1 = (int)(*it)["type1"];
				int cwide1 = (int)(*it)["cwide1"];
				int clong_shu1 = (int)(*it)["clong_shu1"];
				int clong_heng1 = (int)(*it)["clong_heng1"];
				int type2 = (int)(*it)["type2"];
				int cwide2 = (int)(*it)["cwide2"];
				int clong_shu2 = (int)(*it)["clong_shu2"];
				int clong_heng2 = (int)(*it)["clong_heng2"];

				if (wnum > 255 || assemble_opt > 255) {
					qCritical("ParamItems file error, name=%s, wnum=%d, assemble_opt=%d", name.c_str(), wnum, assemble_opt);
					check_pass = false;
				}

				if (opidx_via_info >= 16) {
					qCritical("ParamItems file error, name=%s, opidx_via_info=%d", name.c_str(), opidx_via_info);
					check_pass = false;
				}

				if (cwide0 > 255 || type0 > 255 || clong_shu0 > 255 || clong_heng0 > 255) {
					qCritical("ParamItems file error, name=%s, cwide0=%d, type0=%d, clong_shu0=%d, clong_heng0=%d",
						name.c_str(), cwide0, type0, clong_shu0, clong_heng0);
					check_pass = false;
				}

				if (cwide1 > 255 || type1 > 255 || clong_shu1 > 255 || clong_heng1 > 255) {
					qCritical("ParamItems file error, name=%s, cwide1=%d, type1=%d, clong_shu1=%d, clong_heng1=%d",
						name.c_str(), cwide1, type1, clong_shu1, clong_heng1);
					check_pass = false;
				}

				if (cwide2 > 255 || type2 > 255 || clong_shu2 > 255 || clong_heng2 > 255) {
					qCritical("ParamItems file error, name=%s, cwide2=%d, type2=%d, clong_shu2=%d, clong_heng2=%d",
						name.c_str(), cwide2, type2, clong_shu2, clong_heng2);
					check_pass = false;
				}

				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_ASSEMBLE << 16 | opidx_via_info;
				param.pi[2] = assemble_opt << 8 | wnum;
				param.pi[3] = clong_heng0 << 24 | clong_shu0 << 16 | cwide0 << 8 | type0;
				param.pi[4] = clong_heng1 << 24 | clong_shu1 << 16 | cwide1 << 8 | type1;
				param.pi[5] = clong_heng2 << 24 | clong_shu2 << 16 | cwide2 << 8 | type2;
			}
			break;

		case CheckViaWireConnect:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_tp = (int)(*it)["opidx_tp"];
				int opidx_hl_mask = (int)(*it)["opidx_hl_mask"];
				int v_pattern = (int)(*it)["v_pattern"];
				int v_type = (int)(*it)["v_type"];
				int v_subtype = (int)(*it)["v_subtype"];
				int check_opt = (int)(*it)["check_opt"];
				int w_type0 = (int)(*it)["w_type0"];
				int i_wide0 = (int)(*it)["i_wide0"];
				int w_wide0 = (int)(*it)["w_wide0"];
				int w_len_near0 = (int)(*it)["w_len_near0"];
				int w_len_far0 = (int)(*it)["w_len_far0"];
				int extend0 = (int)(*it)["extend0"];
				int w_type1 = (int)(*it)["w_type1"];
				int i_wide1 = (int)(*it)["i_wide1"];
				int w_wide1 = (int)(*it)["w_wide1"];
				int w_len_near1 = (int)(*it)["w_len_near1"];
				int w_len_far1 = (int)(*it)["w_len_far1"];
				int extend1 = (int)(*it)["extend1"];
				int w_type2 = (int)(*it)["w_type2"];
				int i_wide2 = (int)(*it)["i_wide2"];
				int w_wide2 = (int)(*it)["w_wide2"];
				int w_len_near2 = (int)(*it)["w_len_near2"];
				int w_len_far2 = (int)(*it)["w_len_far2"];
				int extend2 = (int)(*it)["extend2"];
				if (opidx_tp > 16 || opidx_hl_mask > 16) {
					qCritical("ParamItems file error, name=%s, opidx_tp=%d, opidx_hl_mask=%d", name.c_str(), opidx_tp, opidx_hl_mask);
					check_pass = false;
				}
				if (v_pattern > 255 || v_type > 255 || v_subtype > 255 || check_opt > 255) {
					qCritical("ParamItems file error, name=%s, v_pattern=%d, v_type=%d, v_subtype=%d, check_opt=%d", 
						name.c_str(), v_pattern, v_type, v_subtype, check_opt);
					check_pass = false;
				}
				if (w_type0 > 255 || i_wide0 > 255 || w_wide0 > 255 || w_len_near0 > 255 || w_len_far0 > 255 || extend0 > 255) {
					qCritical("ParamItems file error, name=%s, w_type0=%d, i_wide0=%d, w_wide0=%d, w_len_near0=%d, w_len_far0=%d, extend0=%d",
						name.c_str(), w_type0, i_wide0, w_wide0, w_len_near0, w_len_far0, extend0);
					check_pass = false;
				}
				if (w_type1 > 255 || i_wide1 > 255 || w_wide1 > 255 || w_len_near1 > 255 || w_len_far1 > 255 || extend1 > 255) {
					qCritical("ParamItems file error, name=%s, w_type1=%d, i_wide1=%d, w_wide1=%d, w_len_near1=%d, w_len_far1=%d, extend1=%d",
						name.c_str(), w_type1, i_wide1, w_wide1, w_len_near1, w_len_far1, extend1);
					check_pass = false;
				}
				if (w_type2 > 255 || i_wide2 > 255 || w_wide2 > 255 || w_len_near2 > 255 || w_len_far2 > 255 || extend2 > 255) {
					qCritical("ParamItems file error, name=%s, w_type2=%d, i_wide2=%d, w_wide2=%d, w_len_near2=%d, w_len_far2=%d, extend2=%d",
						name.c_str(), w_type2, i_wide2, w_wide2, w_len_near2, w_len_far2, extend2);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_CHECK_VIA_WIRE_CONNECT << 16 | opidx_hl_mask << 4 | opidx_tp;
				param.pi[2] = check_opt << 24 | v_subtype << 16 | v_type << 8 | v_pattern;
				param.pi[3] = w_type0 << 24 | i_wide0 << 16 | w_wide0 << 8 | w_len_near0;
				param.pi[4] = extend0 << 8 | w_len_far0;
				param.pi[5] = w_type1 << 24 | i_wide1 << 16 | w_wide1 << 8 | w_len_near1;
				param.pi[6] = extend1 << 8 | w_len_far1;
				param.pi[7] = w_type2 << 24 | i_wide2 << 16 | w_wide2 << 8 | w_len_near2;
				param.pi[8] = extend2 << 8 | w_len_far2;
			}
			break;
		case FilterObj:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int filter_method = (int)(*it)["filter_method"];
				int opt1 = (int)(*it)["opt1"];
				int opt2 = (int)(*it)["opt2"];
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_OBJ_PROCESS << 16;
				param.pi[2] = filter_method << 8;
				param.pi[3] = opt1;
				param.pi[4] = opt2;
				if (filter_method > 255) {
					qCritical("ParamItems file error, name=%s, filter=%d", name.c_str(), filter_method);
					check_pass = false;
				}
			}
			break;
		}
		if (check_pass)
			params[name] = param;
	}

	FileNode param_sets0 = fs["ParamSets"];
	for (FileNodeIterator it = param_sets0.begin(); it != param_sets0.end(); it++) {
		string name = (string)(*it)["name"];
		vector<string> items;
		(*it)["items"] >> items;
		set_param_sets(name, items);
	}
	fs.release();
    return true;
}

void ExtractParam::write_file(string filename)
{
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "ParamItems" << "[";
	for (map<string, ParamItem>::iterator it = params.begin(); it != params.end(); it++) {
		fs << "{" << "name" << it->first;
		int method = -1;
		for (int i = 0; i < sizeof(method_name) / sizeof(method_name[0]); i++)
		if (it->first.find(method_name[i]) == 0) {
			method = i;
			break;
		}
		if (method == -1) {
			qCritical("ParamItems internal error, name =%s", it->first.c_str());
			continue;
		}
		switch (method) {
		case LayerInfo:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "gs" << (it->second.pi[2] & 0xff);
			fs << "border_size" << (it->second.pi[2] >> 16 & 0xff);
			fs << "computer_border" << (it->second.pi[2] >> 8 & 0xff);
			break;

		case ViaInfo:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "shape" << (it->second.pi[2] & 0xff);
			fs << "type" << (it->second.pi[2] >> 8 & 0xff);
			fs << "subtype" << (it->second.pi[2] >> 16 & 0xff);
			fs << "guard" << (it->second.pi[3] & 0xff);
			fs << "remove_rd" << (it->second.pi[3] >> 8 & 0xff);
			fs << "arfactor" << (it->second.pi[3] >> 16 & 0xff);
			fs << "pair_d" << (it->second.pi[3] >> 24 & 0xff);
			fs << "connect_d" << (it->second.pi[6] >> 24 & 0xff);
			fs << "cgray_d" << (it->second.pi[6] >> 16 & 0xff);
			fs << "cgray_ratio" << (it->second.pi[6] >> 8 & 0xff);
			fs << "connect_rd" << (it->second.pi[6] & 0xff);
			fs << "rd0" << (it->second.pi[4] & 0xff);
			fs << "rd1" << (it->second.pi[4] >> 8 & 0xff);
			fs << "rd2" << (it->second.pi[4] >> 16 & 0xff);
			fs << "rd3" << (it->second.pi[4] >> 24 & 0xff);
			fs << "gray0" << (it->second.pi[5] & 0xff);
			fs << "gray1" << (it->second.pi[5] >> 8 & 0xff);
			fs << "gray2" << (it->second.pi[5] >> 16 & 0xff);
			fs << "gray3" << (it->second.pi[5] >> 24 & 0xff);
			break;

		case WireInfo:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "pattern" << (it->second.pi[2] & 0xff);
			fs << "type" << (it->second.pi[2] >> 8 & 0xff);
			fs << "subtype" << (it->second.pi[2] >> 16 & 0xff);
			fs << "guard" << (it->second.pi[3] & 0xff);
			fs << "arfactor" << (it->second.pi[3] >> 8 & 0xff);
			fs << "w_wide" << (it->second.pi[4] >> 24 & 0xff);
			fs << "w_wide1" << (it->second.pi[4] >> 16 & 0xff);
			fs << "w_high" << (it->second.pi[4] >> 8 & 0xff);
			fs << "w_high1" << (it->second.pi[4] & 0xff);
			fs << "i_wide" << (it->second.pi[5] >> 8 & 0xff);
			fs << "i_high" << (it->second.pi[5] & 0xff);
			fs << "dis_vw0" << (it->second.pi[5] >> 24 & 0xff);
			fs << "dis_vw1" << (it->second.pi[5] >> 16 & 0xff);
			fs << "wgid0" << (it->second.pi[6] & 0xff);
			fs << "wgid1" << ((it->second.pi[6] >> 8) & 0xff);
			fs << "wgid2" << ((it->second.pi[6] >> 16) & 0xff);
			fs << "wgid3" << ((it->second.pi[6] >> 24) & 0xff);
			fs << "wgid4" << (it->second.pi[7] & 0xff);
			fs << "wgid5" << ((it->second.pi[7] >> 8) & 0xff);
			fs << "wgid6" << ((it->second.pi[7] >> 16) & 0xff);
			fs << "wgid7" << ((it->second.pi[7] >> 24) & 0xff);
			fs << "uni_id" << (it->second.pi[8] & 0xffff);
			break;

		case Rgb2Gray:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "red" << it->second.pi[2];
			fs << "green" << it->second.pi[3];
			fs << "blue" << it->second.pi[4];
			break;

		case MinStat:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "stat_grid" << it->second.pi[2];
			fs << "min_percent" << it->second.pi[3];
			fs << "grid_win" << it->second.pi[4];
			fs << "ksize" << it->second.pi[5];
			fs << "filter_method" << it->second.pi[6];
			break;

		case GrayLevel:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "opidx_tp" << (it->second.pi[1] & 0xf);
			fs << "low_sep_min" << (it->second.pi[2] & 0xff);
			fs << "low_sep_max" << (it->second.pi[2] >> 8 & 0xff);
			fs << "low_win" << (it->second.pi[2] >> 16 & 0xff);
			fs << "low_dis_min" << (it->second.pi[2] >> 24 & 0xff);
			fs << "low_k0" << (it->second.pi[5] & 0xff);
			fs << "low_k1" << (it->second.pi[5] >> 8 & 0xff);
			fs << "low_k2" << (it->second.pi[5] >> 16 & 0xff);

			fs << "mid_sep_min" << (it->second.pi[3] & 0xff);
			fs << "mid_sep_max" << (it->second.pi[3] >> 8 & 0xff);
			fs << "mid_win" << (it->second.pi[3] >> 16 & 0xff);
			fs << "mid_dis_min" << (it->second.pi[3] >> 24 & 0xff);
			fs << "mid_k0" << (it->second.pi[6] & 0xff);
			fs << "mid_k1" << (it->second.pi[6] >> 8 & 0xff);
			fs << "mid_k2" << (it->second.pi[6] >> 16 & 0xff);

			fs << "high_sep_min" << (it->second.pi[4] & 0xff);
			fs << "high_sep_max" << (it->second.pi[4] >> 8 & 0xff);
			fs << "high_win" << (it->second.pi[4] >> 16 & 0xff);
			fs << "high_dis_min" << (it->second.pi[4] >> 24 & 0xff);
			fs << "high_k0" << (it->second.pi[7] & 0xff);
			fs << "high_k1" << (it->second.pi[7] >> 8 & 0xff);
			fs << "high_k2" << (it->second.pi[7] >> 16 & 0xff);
			break;

		case CoarseLineSearch:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "opidx_tp" << (it->second.pi[1] & 0xf);
			fs << "opidx_shadow_prob" << (it->second.pi[1] >> 4 & 0xf);
			fs << "opidx_rv_mask" << (it->second.pi[1] >> 8 & 0xf);
			fs << "wlong0" << (it->second.pi[2] & 0xff);
			fs << "wlong1" << (it->second.pi[2] >> 8 & 0xff);
			fs << "inc0" << (it->second.pi[2] >> 16 & 0xff);
			fs << "inc1" << (it->second.pi[2] >> 24 & 0xff);
			fs << "wnum" << (it->second.pi[3] & 0xff);
			fs << "th" << (it->second.pi[3] >> 8 & 0xff);
			fs << "search_opt" << (it->second.pi[3] >> 16 & 0xff);
			fs << "update_prob" << (it->second.pi[3] >> 24 & 0xff);
			fs << "subtype0" << (it->second.pi[4] >> 24 & 0xff);
			fs << "type0" << (it->second.pi[4] & 0xff);
			fs << "dir0" << (it->second.pi[4] >> 8 & 0xff);
			fs << "pattern0" << (it->second.pi[4] >> 16 & 0xff);
			fs << "subtype1" << (it->second.pi[5] >> 24 & 0xff);
			fs << "type1" << (it->second.pi[5] & 0xff);
			fs << "dir1" << (it->second.pi[5] >> 8 & 0xff);
			fs << "pattern1" << (it->second.pi[5] >> 16 & 0xff);
			fs << "subtype2" << (it->second.pi[6] >> 24 & 0xff);
			fs << "type2" << (it->second.pi[6] & 0xff);
			fs << "dir2" << (it->second.pi[6] >> 8 & 0xff);
			fs << "pattern2" << (it->second.pi[6] >> 16 & 0xff);
			fs << "subtype3" << (it->second.pi[7] >> 24 & 0xff);
			fs << "type3" << (it->second.pi[7] & 0xff);
			fs << "dir3" << (it->second.pi[7] >> 8 & 0xff);
			fs << "pattern3" << (it->second.pi[7] >> 16 & 0xff);
			break;

		case CoarseViaSearch:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "opidx_via_mask" << (it->second.pi[1] & 0xf);
			fs << "vnum" << (it->second.pi[2] & 0xff);
			fs << "subtype0" << (it->second.pi[3] & 0xff);
			fs << "wide0" << (it->second.pi[3] >> 8 & 0xff);
			fs << "percent0" << (it->second.pi[3] >> 16 & 0xff);
			fs << "subtype1" << (it->second.pi[4] & 0xff);
			fs << "wide1" << (it->second.pi[4] >> 8 & 0xff);
			fs << "percent1" << (it->second.pi[4] >> 16 & 0xff);
			fs << "subtype2" << (it->second.pi[5] & 0xff);
			fs << "wide2" << (it->second.pi[5] >> 8 & 0xff);
			fs << "percent2" << (it->second.pi[5] >> 16 & 0xff);
			fs << "subtype3" << (it->second.pi[6] & 0xff);
			fs << "wide3" << (it->second.pi[6] >> 8 & 0xff);
			fs << "percent3" << (it->second.pi[6] >> 16 & 0xff);
			fs << "subtype4" << (it->second.pi[7] & 0xff);
			fs << "wide4" << (it->second.pi[7] >> 8 & 0xff);
			fs << "percent4" << (it->second.pi[7] >> 16 & 0xff);
			break;

		case FineViasSearch:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "opidx_tp" << (it->second.pi[1] & 0xf);
			fs << "opidx_via_mask" << (it->second.pi[1] >> 4 & 0xf);
			fs << "opidx_via_info" << (it->second.pi[1] >> 8 & 0xf);
			fs << "opidx_shadow_prob" << (it->second.pi[1] >> 12 & 0xf);
			fs << "vnum" << (it->second.pi[2] & 0xff);
			fs << "update_fv" << (it->second.pi[2] >> 8 & 0xff);
			fs << "subtype0" << (it->second.pi[3] >> 8 & 0xff);
			fs << "type0" << (it->second.pi[3] & 0xff);
			fs << "th0" << (it->second.pi[3] >> 16 & 0xff);
			fs << "pattern0" << (it->second.pi[3] >> 24 & 0xff);
			fs << "subtype1" << (it->second.pi[4] >> 8 & 0xff);
			fs << "type1" << (it->second.pi[4] & 0xff);
			fs << "th1" << (it->second.pi[4] >> 16 & 0xff);
			fs << "pattern1" << (it->second.pi[4] >> 24 & 0xff);
			fs << "subtype2" << (it->second.pi[5] >> 8 & 0xff);
			fs << "type2" << (it->second.pi[5] & 0xff);
			fs << "th2" << (it->second.pi[5] >> 16 & 0xff);
			fs << "pattern2" << (it->second.pi[5] >> 24 & 0xff);
			fs << "subtype3" << (it->second.pi[6] >> 8 & 0xff);
			fs << "type3" << (it->second.pi[6] & 0xff);
			fs << "th3" << (it->second.pi[6] >> 16 & 0xff);
			fs << "pattern3" << (it->second.pi[6] >> 24 & 0xff);
			fs << "subtype4" << (it->second.pi[7] >> 8 & 0xff);
			fs << "type4" << (it->second.pi[7] & 0xff);
			fs << "th4" << (it->second.pi[7] >> 16 & 0xff);
			fs << "pattern4" << (it->second.pi[7] >> 24 & 0xff);
			break;

		case RemoveVia:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "opidx_via_info" << (it->second.pi[1] & 0xf);
			fs << "opidx_rv_mask" << (it->second.pi[1] >> 4 & 0xf);
			fs << "vnum" << (it->second.pi[2] & 0xff);
			fs << "check_len" << (it->second.pi[2] >> 8 & 0xff);
			fs << "default_dir" << (it->second.pi[2] >> 16 & 0xff);
			fs << "remove_opt" << (it->second.pi[2] >> 24 & 0xff);
			fs << "subtype0" << (it->second.pi[3] >> 8 & 0xff);
			fs << "type0" << (it->second.pi[3] & 0xff);
			fs << "pattern0" << (it->second.pi[3] >> 16 & 0xff);
			fs << "subtype1" << (it->second.pi[4] >> 8 & 0xff);
			fs << "type1" << (it->second.pi[4] & 0xff);
			fs << "pattern1" << (it->second.pi[4] >> 16 & 0xff);
			fs << "subtype2" << (it->second.pi[5] >> 8 & 0xff);
			fs << "type2" << (it->second.pi[5] & 0xff);
			fs << "pattern2" << (it->second.pi[5] >> 16 & 0xff);
			fs << "subtype3" << (it->second.pi[6] >> 8 & 0xff);
			fs << "type3" << (it->second.pi[6] & 0xff);
			fs << "pattern3" << (it->second.pi[6] >> 16 & 0xff);
			fs << "subtype4" << (it->second.pi[7] >> 8 & 0xff);
			fs << "type4" << (it->second.pi[7] & 0xff);
			fs << "pattern4" << (it->second.pi[7] >> 16 & 0xff);
			break;

		case HotlineMask:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "opidx_hl_mask" << (it->second.pi[1] & 0xf);
			fs << "wnum" << (it->second.pi[2] & 0xff);
			fs << "hotline_opt" << (it->second.pi[2] >> 8 & 0xff);
			fs << "shape0" << (it->second.pi[3] & 0xff);
			fs << "type0" << (it->second.pi[3] >> 8 & 0xff);
			fs << "clong0" << (it->second.pi[3] >> 16 & 0xff);
			fs << "cwide0" << (it->second.pi[3] >> 24 & 0xff);
			fs << "subtype0" << (it->second.pi[4] & 0xff);
			fs << "extend0" << (it->second.pi[4] >> 8 & 0xff);
			fs << "shape1" << (it->second.pi[5] & 0xff);
			fs << "type1" << (it->second.pi[5] >> 8 & 0xff);
			fs << "clong1" << (it->second.pi[5] >> 16 & 0xff);
			fs << "cwide1" << (it->second.pi[5] >> 24 & 0xff);
			fs << "subtype1" << (it->second.pi[6] & 0xff);
			fs << "extend1" << (it->second.pi[6] >> 8 & 0xff);
			fs << "shape2" << (it->second.pi[7] & 0xff);
			fs << "type2" << (it->second.pi[7] >> 8 & 0xff);
			fs << "clong2" << (it->second.pi[7] >> 16 & 0xff);
			fs << "cwide2" << (it->second.pi[7] >> 24 & 0xff);
			fs << "subtype2" << (it->second.pi[8] & 0xff);
			fs << "extend2" << (it->second.pi[8] >> 8 & 0xff);
			break;

		case FineLineSearch:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "opidx_via_info" << (it->second.pi[1] & 0xf);
			fs << "opidx_tp" << (it->second.pi[1] & 0xf);
			fs << "opidx_hl_mask" << (it->second.pi[1] >> 4 & 0xf);
			fs << "opidx_rv_mask" << (it->second.pi[1] >> 8 & 0xff);
			fs << "opidx_via_info" << (it->second.pi[1] >> 12 & 0xff);
			fs << "wnum" << (it->second.pi[2] & 0xff);
			fs << "search_opt" << (it->second.pi[2] >> 16 & 0xff);
			fs << "subtype0" << (it->second.pi[3] >> 16 & 0xff);
			fs << "type0" << (it->second.pi[3] >> 8 & 0xff);
			fs << "pattern0" << (it->second.pi[3] & 0xff);
			fs << "subtype1" << (it->second.pi[4] >> 16 & 0xff);
			fs << "type1" << (it->second.pi[4] >> 8 & 0xff);
			fs << "pattern1" << (it->second.pi[4] & 0xff);
			fs << "subtype2" << (it->second.pi[5] >> 16 & 0xff);
			fs << "type2" << (it->second.pi[5] >> 8 & 0xff);
			fs << "pattern2" << (it->second.pi[5] & 0xff);
			fs << "subtype3" << (it->second.pi[6] >> 16 & 0xff);
			fs << "type3" << (it->second.pi[6] >> 8 & 0xff);
			fs << "pattern3" << (it->second.pi[6] & 0xff);
			break;

		case AssembleLine:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "wnum" << (it->second.pi[2] & 0xff);
			fs << "assemble_opt" << (it->second.pi[2] >> 8 & 0xff);
			fs << "type0" << (it->second.pi[3] & 0xff);
			fs << "cwide0" << (it->second.pi[3] >> 8 & 0xff);
			fs << "clong_shu0" << (it->second.pi[3] >> 16 & 0xff);
			fs << "clong_heng0" << (it->second.pi[3] >> 24 & 0xff);
			fs << "type1" << (it->second.pi[4] & 0xff);
			fs << "cwide1" << (it->second.pi[4] >> 8 & 0xff);
			fs << "clong_shu1" << (it->second.pi[4] >> 16 & 0xff);
			fs << "clong_heng1" << (it->second.pi[4] >> 24 & 0xff);
			fs << "type2" << (it->second.pi[5] & 0xff);
			fs << "cwide2" << (it->second.pi[5] >> 8 & 0xff);
			fs << "clong_shu2" << (it->second.pi[5] >> 16 & 0xff);
			fs << "clong_heng2" << (it->second.pi[5] >> 24 & 0xff);
			break;
			
		case CheckViaWireConnect:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "opidx_tp" << (it->second.pi[1] & 0xff);
			fs << "opidx_hl_mask" << (it->second.pi[1] >> 4 & 0xff);
			fs << "v_pattern" << (it->second.pi[2] & 0xff);
			fs << "v_type" << (it->second.pi[2] >> 8 & 0xff);
			fs << "v_subtype" << (it->second.pi[2] >> 16 & 0xff);
			fs << "check_opt" << (it->second.pi[2] >> 24 & 0xff);
			fs << "w_type0" << (it->second.pi[3] >> 24 & 0xff);
			fs << "i_wide0" << (it->second.pi[3] >> 16 & 0xff);
			fs << "w_wide0" << (it->second.pi[3] >> 8 & 0xff);
			fs << "w_len_near0" << (it->second.pi[3] & 0xff);
			fs << "extend0" << (it->second.pi[4] >> 8 & 0xff);
			fs << "w_len_far0" << (it->second.pi[4] & 0xff);
			fs << "w_type1" << (it->second.pi[5] >> 24 & 0xff);
			fs << "i_wide1" << (it->second.pi[5] >> 16 & 0xff);
			fs << "w_wide1" << (it->second.pi[5] >> 8 & 0xff);
			fs << "w_len_near1" << (it->second.pi[5] & 0xff);
			fs << "extend1" << (it->second.pi[6] >> 8 & 0xff);
			fs << "w_len_far1" << (it->second.pi[6] & 0xff);
			fs << "w_type2" << (it->second.pi[7] >> 24 & 0xff);
			fs << "i_wide2" << (it->second.pi[7] >> 16 & 0xff);
			fs << "w_wide2" << (it->second.pi[7] >> 8 & 0xff);
			fs << "w_len_near2" << (it->second.pi[7] & 0xff);
			fs << "extend2" << (it->second.pi[8] >> 8 & 0xff);
			fs << "w_len_far2" << (it->second.pi[8] & 0xff);
			break;		

		case FilterObj:
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "layer" << it->second.pi[0];
			fs << "filter_method" << (it->second.pi[2] >> 8 & 0xff);
			fs << "opt1" << it->second.pi[3];
			fs << "dir" << it->second.pi[4];
			break;
		}
		fs << "}";
	}
	fs << "]";
	fs << "ParamSets" << "[";
	for (map<string, ParamSet>::iterator it = param_sets.begin(); it != param_sets.end(); it++) {
		fs << "{" << "name" << it->first;
		fs << "items" << "[";
		for (int i = 0; i < it->second.names.size(); i++)
			fs << it->second.names[i];
		fs << "]";
		fs << "}";
	}
	fs << "]";
	fs.release();
}

void ExtractParam::clear()
{
	params.clear();
	param_sets.clear();
	memset(method_count, 0, sizeof(method_count));
	depth = 0;
}

bool ExtractParam::operator==(ExtractParam & ep)
{
	if (params.size() != ep.params.size() || param_sets.size() != ep.param_sets.size())
		return false;

	for (map<string, ParamItem>::iterator it = params.begin(); it != params.end(); it++) {
		map<string, ParamItem>::iterator it1 = ep.params.find(it->first);
		if (it1 == ep.params.end())
			return false;
		if (it->second != it1->second)
			return false;
	}

	for (map<string, ParamSet>::iterator it = param_sets.begin(); it != param_sets.end(); it++) {
		map<string, ParamSet>::iterator it1 = ep.param_sets.find(it->first);
		if (it1 == ep.param_sets.end())
			return false;
		if (it->second != it1->second)
			return false;
	}
	return true;
}