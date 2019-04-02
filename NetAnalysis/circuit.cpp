#include "circuit.h"
#include <algorithm>
#include <functional>
extern int yyparse();
extern int prepare(string filename, Circuit * c);
extern string yy_filename;

Circuit::Circuit()
{
    parent = NULL;
	name = "";
	subckts.push_back(this);
}

Circuit::~Circuit()
{
	if (parent == NULL)
    for (int i=1; i<(int) subckts.size(); i++)
        delete subckts[i];
}

Circuit * Circuit::parse(string filename)
{
	Circuit * ret = new Circuit();
	prepare(filename, ret);
	yyparse();
	return ret;
}

/*
Input: name, subckt name
Input: nodes, subckt port
Notes: check if subckt is already added, if not add new subckt, if yes, redefine subckt
*/
Circuit * Circuit::new_subckt(string name, vector<char *> _nodes)
{
	if (parent)
		return parent->new_subckt(name, _nodes);
	Circuit * subckt = search_subckt(name).first;
	if (subckt == NULL) {
		subckt = new Circuit();
		subckt->parent = this;
		subckt->name = name;
		subckts.push_back(subckt);
	}
	else {
		if (subckt->inf.size() != 0)
			qWarning("Redefine subckt %s in file %s, line %d", name.c_str(), yy_filename.c_str(), yylineno - 1);
	}
	
    for (int i=0; i<(int) _nodes.size(); i++)
        subckt->inf.push_back(subckt->search_or_add_node(_nodes[i]));
    return subckt;
}

/*
Convert name to subckt index
*/
pair<Circuit *, int>  Circuit::search_subckt(string name)
{
	if (parent)
		return parent->search_subckt(name);
	for (int i = 0; i<(int)subckts.size(); i++)
	if (subckts[i]->name == name)
		return make_pair(subckts[i], DEVICE_X + 1 + i);
	return make_pair((Circuit *) NULL, -1);
}

/*
Convert subckt index to name
*/
string Circuit::get_subckt_name(int idx)
{
	if (parent)
		return parent->get_subckt_name(idx);
	if (idx > DEVICE_X && idx - DEVICE_X - 1 < subckts.size())
		return subckts[idx - DEVICE_X - 1]->name;
	else
		return "";
}

int Circuit::get_subckt_num()
{
	if (parent)
		return parent->get_subckt_num();
	else
		return subckts.size();
}

Circuit * Circuit::get_parent_ckt()
{
    return parent;
}

/*
Convert node_name to nodes idx
Input: node_name
Return: nodes idx
Notes: if not exist, add new node
*/
int Circuit::search_or_add_node(string node_name)
{
    //search if node already exist
	map<string, int>::iterator it = nname2idx.find(node_name);
	if (it != nname2idx.end())
		return it->second;
    nodes.push_back(Node(node_name));
	int ret = (int) nodes.size() - 1;
	nname2idx[node_name] = ret;
    return ret;
}

/*
Convert device name to device index
*/
int Circuit::search_dev(string name)
{
	for (int i = 0; i < (int)devs.size(); i++)
	if (devs[i].name == name)
		return i;
	return -1;
}

/*
Convert node_name to nodes idx
Input: node_name
Return: nodes idx or -1 if not found
*/
int Circuit::search_node(string node_name)
{
	//search if node already exist
	map<string, int>::iterator it = nname2idx.find(node_name);
	if (it != nname2idx.end())
		return it->second;
	else
		return -1;
}

int Circuit::new_device(string name, char * nname1, char * nname2)
{
	vector<char *> nodes_name;
	nodes_name.push_back(nname1);
	nodes_name.push_back(nname2);
	return new_device(name, nodes_name);
}

int Circuit::new_device(string name, char * nname1, char * nname2, char * nname3)
{
	vector<char *> nodes_name;
	nodes_name.push_back(nname1);
	nodes_name.push_back(nname2);
	nodes_name.push_back(nname3);
	return new_device(name, nodes_name);
}

int Circuit::new_device(string name, char * nname1, char * nname2, char * nname3, char * nname4)
{
	vector<char *> nodes_name;
	nodes_name.push_back(nname1);
	nodes_name.push_back(nname2);
	nodes_name.push_back(nname3);
	nodes_name.push_back(nname4);
	return new_device(name, nodes_name);
}

int Circuit::new_device(string name, char * nname1, char * nname2, char * nname3, char * nname4, char * nname5)
{
	vector<char *> nodes_name;
	nodes_name.push_back(nname1);
	nodes_name.push_back(nname2);
	nodes_name.push_back(nname3);
	nodes_name.push_back(nname4);
	nodes_name.push_back(nname5);
	return new_device(name, nodes_name);
}
/*
Add new device and its connected nodes to circuit
Input: name, device name
Input: nodes_name, node name
return device_idx
*/
int Circuit::new_device(string name, vector<char *> nodes_name)
{
	if (search_dev(name) >= 0) {
		qWarning("Dev %s is already added in file %s, line %d", name.c_str(), yy_filename.c_str(), yylineno - 1);
		return -1;
	}
	devs.push_back(Device(name));
	int nodes_size = (int)nodes_name.size();
	if (devs.back().type == DEVICE_X) {
		for (int i = 1; i < nodes_size; i++) { //loop to find subckt name
			pair<Circuit *, int> ret = search_subckt(nodes_name[i]);
			if (ret.first) {
				nodes_size = i; //found subckt name, reduce nodes_size
				devs.back().type = ret.second;
				if (find(subckts.begin(), subckts.end(), ret.first) == subckts.end())
					subckts.push_back(ret.first);
				break;
			}
		}
		if (devs.back().type == DEVICE_X) {
			qWarning("unknow subckt, in file %s, line %d, reserve subck name %s", yy_filename.c_str(), 
				yylineno - 1, nodes_name.back());
			new_subckt(nodes_name.back(), vector<char*>());
			devs.back().type = search_subckt(nodes_name.back()).second;
			Q_ASSERT(devs.back().type != DEVICE_X);
		}
	}
	int ret = (int) devs.size() - 1;
	for (int i = 0; i < nodes_size; i++) {
		int node_idx = search_or_add_node(nodes_name[i]);
		devs.back().connect_node(node_idx);
		nodes[node_idx].connect_device(ret);
	}
	return ret;
}

bool Circuit::is_port(int node)
{
	return (find(inf.begin(), inf.end(), node) != inf.end());
}

void Circuit::get_sorted_nodes(vector<int> & node_idx)
{
	//sort nodes from big load to small load
	vector<unsigned long long> sn;
	for (int i = 0; i < (int)nodes.size(); i++)
		sn.push_back((unsigned long long) nodes[i].cd.size() << 32 | i);
	sort(sn.begin(), sn.end(), greater<unsigned long long>());
	node_idx.clear();
	for (int i = 0; i < (int)sn.size(); i++)
		node_idx.push_back(sn[i] & 0xffffffff);
}