#ifndef CIRCUIT_H
#define CIRCUIT_H
#include <string>
#include <vector>
#include <string.h>
#include <ctype.h>
#include <map>
#include <QtGlobal>
using namespace std;

enum DeviceType{
	DEVICE_UNKNOWN,
    DEVICE_R,
    DEVICE_L,
    DEVICE_C,
    DEVICE_DIODE,
    DEVICE_BIPOLAR,
    DEVICE_JFET,
    DEVICE_MOSFET,
    DEVICE_MESFET,
    DEVICE_X, //DEVICE_X must be end
};

extern int yylineno;

struct StrCmp
{
    bool operator()( const char * s1, const char * s2 ) const
    {
        return strcmp( s1, s2 ) < 0;
    }
};

struct Node {
    string name;
    vector<int> cd; //idx in Circuit->devs
    Node();
    Node(string _name) {
		name = _name;
    }
	void connect_device(int device_idx) {
		cd.push_back(device_idx);
	}
};

struct Device {
    string name;
    int type;
	vector<int> cn; //idx in Circuit->nodes
	Device() {

	}
    Device(string _name) {
        name = _name;
		switch (toupper(_name[0])) {
		case 'D':
			type = DEVICE_DIODE;
			break;
		case 'Q':
			type = DEVICE_BIPOLAR;
			break;
		case 'J':
			type = DEVICE_JFET;
			break;
		case 'M':
			type = DEVICE_MOSFET;
			break;
		case 'Z':
			type = DEVICE_MESFET;
			break;
		case 'R':
			type = DEVICE_R;
			break;
		case 'L':
			type = DEVICE_L;
			break;
		case 'C':
			type = DEVICE_C;
			break;
		case 'X':
			type = DEVICE_X;
			break;
		default:
			type = DEVICE_UNKNOWN;
			qCritical("unknow device, line%d:%s", yylineno, _name.c_str());
			return;
		}
    }
	void connect_node(int node_idx) {
		cn.push_back(node_idx);
	}
};

/*
Circuit description
*/
class Circuit
{
protected:
	string name;
	vector<int> inf; //it is index of nodes, nodes[inf[0]] is 1st port,  nodes[inf[1]] is 2nd port
	vector<Device> devs; //may contain X device
	vector<Node> nodes;
	vector<Circuit *> subckts; //point to sub-circuit description, tree struct
	Circuit * parent; //point to parent
	map<string, int> nname2idx; //map node name to nodes idx
	int search_or_add_node(string node_name);

public:
	Circuit();
	~Circuit();
	static Circuit * parse(string filename);
	Circuit * new_subckt(string name, vector<char *> _nodes);
	pair<Circuit *, int> search_subckt(string name);
	int search_dev(string name);
	string get_subckt_name(int idx);
	int get_subckt_num();
	Circuit * get_parent_ckt();	
	int search_node(string node_name);
	int new_device(string name, char * nname1, char * nname2);
	int new_device(string name, char * nname1, char * nname2, char * nname3);
	int new_device(string name, char * nname1, char * nname2, char * nname3, char * nname4);
	int new_device(string name, char * nname1, char * nname2, char * nname3, char * nname4, char * nname5);
	int new_device(string name, vector<char *> nodes_name);
	bool is_port(int node);
	string get_dev_name(int dev_idx) const {
		return devs[dev_idx].name;
	}
	int get_dev_cn(int dev_idx, int port) const {
		return devs[dev_idx].cn[port];
	}
	string get_node_name(int node_idx) const {
		return nodes[node_idx].name;
	}
	int get_node_cd_size(int node_idx) const {
		return (int) nodes[node_idx].cd.size();
	}
	void get_sorted_nodes(vector<int> & node_idx);
	friend class CircuitMatch;
};

#endif // CIRCUIT_H
