#ifndef EXTRACTPARAM_H
#define EXTRACTPARAM_H
#include <string>
#include <vector>
#include <map>
#include <math.h>
#include <string.h>
using namespace std;

class ParamItem {
public:
	int pi[9];
	float pf;
	ParamItem() {
		memset(pi, 0, sizeof(pi));
		pf = 0;
	}
	ParamItem(int pi0, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, int pi8, float pf0) {
		pi[0] = pi0;
		pi[1] = pi1;
		pi[2] = pi2;
		pi[3] = pi3;
		pi[4] = pi4;
		pi[5] = pi5;
		pi[6] = pi6;
		pi[7] = pi7;
		pi[8] = pi8;
		pf = pf0;
	}
	bool operator==(const ParamItem & item) const {
		for (int i = 0; i < sizeof(pi) / sizeof(pi[0]); i++)
			if (pi[i] != item.pi[i])
				return false;
        return (fabs(pf - item.pf) < 0.0001);
	}
	bool operator!=(const ParamItem & item) const {
		return !(*this == item);
	}
};

class ParamSet {
public:
	vector<string> names;
	ParamSet() {}
	ParamSet(vector<string> & _names) {
		names = _names;
	}
	bool operator==(const ParamSet & s) const {
		for (int i = 0; i < names.size(); i++)
		if (names[i] != s.names[i])
			return false;
		return true;
	}
	bool operator!=(const ParamSet & s) const {
		return !(*this == s);
	}
};

class ExtractParam
{
protected:
	map<string, ParamItem> params;
	map<string, ParamSet> param_sets;
	int method_count[255];
	int depth;
public:
    ExtractParam();
	void get_param(string name, vector<ParamItem> & _params);
	void get_param_sets(string name, vector<string> & names);
	void get_param_set_list(vector<string> & names);
	string set_param(int pi0, int pi1, int pi2, int pi3, int pi4, int pi5, int pi6, int pi7, int pi8, float pf0);
	string set_param_sets(string name, vector<string> & _param_set);
	bool read_file(string filename);
	void write_file(string filename);
	void clear();
	bool operator==(ExtractParam & ep);
	bool operator!=(ExtractParam & ep) {
		return !(*this == ep);
	}
};

#endif // EXTRACTPARAM_H
