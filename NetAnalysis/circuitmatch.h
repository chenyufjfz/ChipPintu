#ifndef CIRCUITMATCH_H
#define CIRCUITMATCH_H
#include "circuit.h"
#include <set> 
#include <QSharedData>

typedef unsigned HASH_TYPE;
class DeviceInstData : public QSharedData {
public:
	HASH_TYPE hash;
	vector<int> port_net; //port_net[0] is for type, port_net[1] is for port0
	int dev_idx; //index in Circuit.devs
	DeviceInstData() {}
	DeviceInstData(const DeviceInstData & o) : QSharedData(o){
		hash = o.hash;
		port_net = o.port_net;
		dev_idx = o.dev_idx;
	}
	~DeviceInstData() {}
};

class DeviceInst {
protected:
	QSharedDataPointer<DeviceInstData> d;
public:
	DeviceInst() {
	}
	DeviceInst(const DeviceInst &other)
		: d(other.d)
	{
	}
	DeviceInst & operator=(const DeviceInst & other) {
		d = other.d;
		return *this;
	}
	DeviceInst(int dev_idx, int net_num) {
		d = new DeviceInstData;
		d->dev_idx = dev_idx;
		d->port_net.resize(net_num, -1);
	}
	void compute_hash() {
		HASH_TYPE hashid = 0;
		int type = d->port_net[0];
		if (type == DEVICE_R || type == DEVICE_L || type == DEVICE_C)
			hashid = type ^ (d->port_net[1] ^ d->port_net[2]) << 8;
		else
		if (type == DEVICE_MOSFET)
			hashid = type ^ (d->port_net[1] ^ d->port_net[3]) << 8 ^ d->port_net[2] << 16 ^ d->port_net[4] << 24;
		else
		for (int i = 0; i < (int)d->port_net.size(); i++)
			hashid ^= (HASH_TYPE)d->port_net[i] << (i % sizeof(HASH_TYPE)* 8);
		d->hash = hashid;
	}
	inline void set_port_net(int port, int net) {
		d->port_net[port] = net;
	}
	inline int get_port_net(int port) const {
		return d->port_net[port];
	}
	inline int get_port_net_num() const {
		return (int) d->port_net.size();
	}
	bool equal(const DeviceInst & o) const {
		if (d->hash != o.d->hash || d->port_net.size() != o.d->port_net.size())
			return false;
		int type = d->port_net[0];
		if (type == DEVICE_R || type == DEVICE_L || type == DEVICE_C)
			return (type == o.d->port_net[0] &&
			(d->port_net[1] == o.d->port_net[1] && d->port_net[2] == o.d->port_net[2] ||
			d->port_net[1] == o.d->port_net[2] && d->port_net[2] == o.d->port_net[1]));
		else 
		if (type == DEVICE_MOSFET)
			return (type == o.d->port_net[0] && d->port_net[2] == o.d->port_net[2] && d->port_net[4] == o.d->port_net[4] &&
			(d->port_net[1] == o.d->port_net[1] && d->port_net[3] == o.d->port_net[3] ||
			d->port_net[3] == o.d->port_net[1] && d->port_net[1] == o.d->port_net[3]));
		else {
			for (int i = 0; i < (int) d->port_net.size(); i++)
			if (d->port_net[i] != o.d->port_net[i])
				return false;
			return true;
		}
	}

	inline HASH_TYPE hash() const {
		return d->hash;
	}

	inline int dev_idx() const {
		return d->dev_idx;
	}
};

struct DeviceInstCmp
{
	bool operator()(const DeviceInst & s1, const DeviceInst & s2) const
	{
		return s1.hash() < s2.hash();
	}
};

enum MatchMethod {
	MATCH_ALL,
	MATCH_SUBCKT,
	MATCH_PART,
};

struct MatchResult {
	vector<pair<string, string> > mdev, mnodes;
};

struct TryMatchNode {
	CircuitMatch * cm0;
	CircuitMatch * cm1;
	MatchMethod method;
	int n0, n1;
	TryMatchNode();
	TryMatchNode(CircuitMatch * _cm0, CircuitMatch * _cm1, MatchMethod _method, int _n0, int _n1) {
		cm0 = _cm0;
		cm1 = _cm1;
		method = _method;
		n0 = _n0;
		n1 = _n1;
	}
};
class CircuitMatch
{
protected:
	Circuit * cir;
	Circuit * root_cir;
	multiset<DeviceInst, DeviceInstCmp> wait_pick;
	vector<DeviceInst> dm; //device match
	vector<int> nm; //nodes match
	vector<int> sm; //subckt match
	vector<string> predef_nm;
	vector<string> predef_sm;
	int search_node(int node);
	int search_device(int device);
	int search_subckt(int subckt);
	bool match_wait_pick(const CircuitMatch * o);
	bool contain_wait_pick(const CircuitMatch * o);
	void pick_dev(const CircuitMatch * o, vector<pair<DeviceInst, DeviceInst > > & dp);
	multiset<DeviceInst, DeviceInstCmp>::iterator search_wait_pick(int d);
	int add_match_node(int n, vector<int> & affect_dm);
	void add_match_device(int d, vector<int> & nodes);
    void init();
	static bool try_match(CircuitMatch * cm0, CircuitMatch * cm1, MatchMethod method);
	friend MatchResult try_match_node(const TryMatchNode & tm);

public:
    CircuitMatch();
	void read_circuit(string filename);
	void predef_match_nodes(vector<string> node_name);
	void predef_match_subckt(vector<string> subckt_name);
	friend void match(CircuitMatch & one, CircuitMatch & other, string subckt_name, string other_subckt_name, MatchMethod method, vector<MatchResult> result);
};

#endif // CIRCUITMATCH_H
