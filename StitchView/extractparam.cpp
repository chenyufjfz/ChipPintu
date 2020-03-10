#include <QtGlobal>
#include "opencv2/imgproc/imgproc.hpp"
#include "extractparam.h"


using namespace cv;

enum {
	Rgb2Gray,
	ComputeGrad,
	DetectBlackImg,
	ComputeCorner,
	DiffNormal,
	DiffWeight,
};

string method_name[] = {
	"Rgb2Gray",
	"ComputeGrad",
	"DetectBlackImg",
	"ComputeCorner",
	"DiffNormal",
	"DiffWeight"
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
		break;
	case PP_RGB2GRAY:
		method = Rgb2Gray;
		break;
	case PP_COMPUTE_GRAD:
		method = ComputeGrad;
		break;
	case PP_DETECT_BLACK_IMG:
		method = DetectBlackImg;
		break;
	case PP_COMPUTE_CORNER:
		method = ComputeCorner;
		break;
	case DIFF_NORMAL:
		method = DiffNormal;
		break;
	case DIFF_WEIGHT:
		method = DiffWeight;
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
		case ComputeGrad:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_src = (int)(*it)["opidx_src"];
				int opidx_gx = (int)(*it)["opidx_gx"];
				int opidx_gy = (int)(*it)["opidx_gy"];
				int bfilt_w = (int)(*it)["bfilt_w"];
				int bfilt_csigma = (int)(*it)["bfilt_csigma"];
				int canny_low_th = (int)(*it)["canny_low_th"];
				int canny_high_th = (int)(*it)["canny_high_th"];
				int sobel_w = (int)(*it)["sobel_w"];
				int sobel_th = (int)(*it)["sobel_th"];
				int edge_dialte = (int)(*it)["edge_dialte"];
				int alpha = (int)(*it)["alpha"];

				if (opidx_src >= 16 || opidx_gx >= 16 || opidx_gy >= 16 || debug_opt > 255) {
					qCritical("ParamItems file error, name=%s, opidx_src=%d, opidx_gx=%d, opidx_gy=%d, debug_opt=%d", 
						name.c_str(), opidx_src, opidx_gx, opidx_gy, debug_opt);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_COMPUTE_GRAD << 16 | opidx_src << 8 | opidx_gy << 4 | opidx_gx;
				param.pi[2] = canny_high_th << 24 | canny_low_th << 16 | bfilt_csigma << 8 | bfilt_w;
				param.pi[3] = alpha << 24 | edge_dialte << 16 | sobel_th << 8 | sobel_w;				
			}
			break;

		case DetectBlackImg:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int th = (int)(*it)["th"];
				int ratio = (int)(*it)["ratio"];
				if (th > 255 || ratio > 255) {
					qCritical("ParamItems file error, name=%s, th=%d, ratio=%d",
						name.c_str(), th, ratio);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | PP_DETECT_BLACK_IMG << 16 | th << 8 | ratio;
			}
			break;

		case ComputeCorner:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int method = (int)(*it)["method"];
				int corner_num = (int)(*it)["corner_num"]; 
				int edge_num = (int)(*it)["edge_num"];
				int corner_distance = (int)(*it)["corner_distance"];
				int edge_distance = (int)(*it)["edge_distance"]; 
				int corner_th = (int)(*it)["corner_th"]; 
				int edge_th = (int)(*it)["edge_th"];
				int weight_corner = (int)(*it)["max_weight_corner"];
				int weight_edge = (int)(*it)["max_weight_edge"];
				int overlap_x = (int)(*it)["overlap_x"];
				int overlap_y = (int)(*it)["overlap_y"];
				int edge_reduce = (int)(*it)["edge_reduce"];
				if (edge_num > 255 || corner_num > 255 || corner_distance > 255 || edge_distance > 255) {
					qCritical("ParamItems file error, name=%s, edge_num=%d, corner_num=%d, corner_distance=%d, edge_distance=%d",
						name.c_str(), edge_num, corner_num, corner_distance, edge_distance);
					check_pass = false;
				}
				if (corner_th > 255 || edge_th > 255 || edge_reduce > 255 || overlap_x > 255 || overlap_y > 255) {
					qCritical("ParamItems file error, name=%s, corner_th=%d, edge_th=%d, edge_reduce=%d, overlap_x=%d, overlap_y=%d",
						name.c_str(), corner_th, edge_th, edge_reduce, overlap_x, overlap_y);
					check_pass = false;
				}
				if (weight_corner > 65535 || weight_edge > 65535) {
					qCritical("ParamItems file error, name=%s, weight_corner=%d, weight_edge=%d",
						name.c_str(), weight_corner, weight_edge);
					check_pass = false;
				}
				param.pi[0] = method << 8 | layer;
				param.pi[1] = debug_opt << 24 | PP_COMPUTE_CORNER << 16 | corner_num << 8 | edge_num;
				param.pi[2] = corner_distance << 24 | edge_distance << 16 | corner_th << 8 | edge_th;
				param.pi[3] = weight_corner << 16 | weight_edge;
				param.pi[4] = edge_reduce << 16 | overlap_x << 8 | overlap_y;
			}
			break;

		case DiffNormal:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int opidx_diff0 = (int)(*it)["opidx_diff0"];
				int opidx_diff1 = (int)(*it)["opidx_diff1"];
				int opidx_diff2 = (int)(*it)["opidx_diff2"];
				int opidx_diff3 = (int)(*it)["opidx_diff3"];
				int mix_diff0 = (int)(*it)["mix_diff0"];
				int mix_diff1 = (int)(*it)["mix_diff1"];
				int mix_diff2 = (int)(*it)["mix_diff2"];
				int mix_diff3 = (int)(*it)["mix_diff3"];
				if (opidx_diff0 >= 16 || opidx_diff1 >= 16 || opidx_diff2 >= 16 || opidx_diff3 >= 16) {
					qCritical("ParamItems file error, name=%s, opidx_diff0=%d, opidx_diff1=%d, opidx_diff2=%d, opidx_diff3=%d", 
						name.c_str(), opidx_diff0, opidx_diff1, opidx_diff2, opidx_diff3);
					check_pass = false;
				}
				if (mix_diff0 >= 256 || mix_diff1 >= 256 || mix_diff2 >= 256 || mix_diff3 >= 256) {
					qCritical("ParamItems file error, name=%s, mix_diff0=%d, mix_diff1=%d, mix_diff2=%d, mix_diff3=%d",
						name.c_str(), opidx_diff0, opidx_diff1, opidx_diff2, opidx_diff3);
					check_pass = false;
				}
				param.pi[0] = layer;
				param.pi[1] = debug_opt << 24 | DIFF_NORMAL << 16 | opidx_diff3 << 12 | opidx_diff2 << 8| opidx_diff1 << 4 | opidx_diff0;
				param.pi[2] = mix_diff3 << 24 | mix_diff2 << 16 | mix_diff1 << 8 | mix_diff0;
			}
			break;

		case DiffWeight:
			{
				int layer = (int)(*it)["layer"];
				int debug_opt = (int)(*it)["debug_opt"];
				int var_th = (int)(*it)["var_th"];
				int method = (int)(*it)["method"];
				int fenzi0 = (int)(*it)["fenzi0"];
				int fenmu0 = (int)(*it)["fenmu0"];
				if (fenzi0 > 65535 || fenmu0 > 65535 || var_th > 65535) {
					qCritical("ParamItems file error, name=%s, var_th=%d, fenzi0=%d, fenmu0=%d",
						name.c_str(), var_th, fenzi0, fenmu0);
					check_pass = false;
				}
				param.pi[0] = method << 8 | layer;
				param.pi[1] = debug_opt << 24 | DIFF_WEIGHT << 16 | var_th;
				param.pi[2] = fenzi0 << 16 | fenmu0;
			}
			break;

		default:
			check_pass = false;
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
		case ComputeGrad:
			fs << "layer" << it->second.pi[0];
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);	
			fs << "opidx_src" << (it->second.pi[1] >> 8 & 0xf);
			fs << "opidx_gx" << (it->second.pi[1] & 0xf);
			fs << "opidx_gy" << (it->second.pi[1] >> 4 & 0xf);
			fs << "bfilt_w" << (it->second.pi[2] & 0xff);
			fs << "bfilt_csigma" << (it->second.pi[2] >> 8 & 0xff);
			fs << "canny_low_th" << (it->second.pi[2] >> 16 & 0xff);
			fs << "canny_high_th" << (it->second.pi[2] >> 24 & 0xff);
			fs << "sobel_w" << (it->second.pi[3] & 0xff);
			fs << "sobel_th" << (it->second.pi[3] >> 8 & 0xff);
			fs << "edge_dialte" << (it->second.pi[3] >> 16 & 0xff);
			fs << "alpha" << (it->second.pi[3] >> 24 & 0xff);
			break;
		case DetectBlackImg:			
			fs << "layer" << it->second.pi[0];
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "th" << (it->second.pi[1] >> 8 & 0xff);
			fs << "ratio" << (it->second.pi[1] & 0xff);
			break;
		case DiffNormal:
			fs << "layer" << it->second.pi[0];
			fs << "debug_opt" << (it->second.pi[1] >> 24 & 0xff);
			fs << "opidx_diff0" << (it->second.pi[1] & 0xf);
			fs << "opidx_diff1" << (it->second.pi[1] >> 4 & 0xf);
			fs << "opidx_diff2" << (it->second.pi[1] >> 8 & 0xf);
			fs << "opidx_diff3" << (it->second.pi[1] >> 12 & 0xf);
			fs << "mix_diff0" << (it->second.pi[2] & 0xff);
			fs << "mix_diff1" << (it->second.pi[2] >> 8 & 0xff);
			fs << "mix_diff2" << (it->second.pi[2] >> 16 & 0xff);
			fs << "mix_diff3" << (it->second.pi[2] >> 24 & 0xff);
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