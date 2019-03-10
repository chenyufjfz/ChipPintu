#include "circuitmatch.h" 
#include <stdio.h>

CircuitMatch::CircuitMatch()
{
	cir = NULL;
}

void CircuitMatch::read_circuit(string filename)
{
	root_cir = Circuit::parse(filename);
    init();
}

/*
Input node_name
Return 0 if success, -1 if fail
Note: predefine match clock, vdd, gnd
*/
void CircuitMatch::predef_match_nodes(vector<string> node_name)
{
	predef_nm = node_name;
}

/*
Input subckt_name
Note: predefine match device, line AND gate, OR gate
*/
void CircuitMatch::predef_match_subckt(vector<string> subckt_name)
{
	predef_sm = subckt_name;
}

/*
Input: node is idx in cir->nodes
Return: node index if found, else -1
Notes: node index is same with other CircuitMatch
*/
int CircuitMatch::search_node(int node)
{
	for (int i = 0; i < (int)nm.size(); i++)
	if (nm[i] == node)
		return i;
	return -1;
}

/*
Input: device is idx in cir->devs
Return: device index
Notes: device index is same with other CircuitMatch
*/
int CircuitMatch::search_device(int device)
{
	for (int i = 0; i < (int)dm.size(); i++)
	if (dm[i].dev_idx() == device)
		return i;
	return -1;
}

/*
Input: subckt
Return: subckt type, not found return DEVICE_X
Notes: subckt type is same with other CircuitMatch
*/
int CircuitMatch::search_subckt(int subckt)
{
	if (subckt < DEVICE_X)
		return subckt;
	for (int i = 0; i < (int)sm.size(); i++)
	if (sm[i] == subckt)
		return DEVICE_X + i + 1;
	return -1;
}

/*
Input o, other CircuitMatch
Return true if match
Notes: compare wait_pick hash set
*/
bool CircuitMatch::match_wait_pick(const CircuitMatch * o)
{
	multiset<DeviceInst, DeviceInstCmp>::iterator oit = o->wait_pick.begin();
	multiset<DeviceInst, DeviceInstCmp>::iterator it = wait_pick.begin();
	if (o->wait_pick.size() != wait_pick.size())
		return false;
	for (; oit != o->wait_pick.end(); oit++) {
		if (oit->hash() != it->hash())
			return false;
		it++;
	}
	return true;
}

/*
Input o, other CircuitMatch
Return true if contain other CircuitMatch
Notes: compare wait_pick hash set
*/
bool CircuitMatch::contain_wait_pick(const CircuitMatch * o)
{
	multiset<DeviceInst, DeviceInstCmp>::iterator oit = o->wait_pick.begin();
	multiset<DeviceInst, DeviceInstCmp>::iterator it = wait_pick.begin();
	if (o->wait_pick.size() > wait_pick.size())
		return false;
	for (; oit != o->wait_pick.end(); oit++) {
		while (it != wait_pick.end() && oit->hash() > it->hash())
			it++;
		if (it == wait_pick.end() || oit->hash() < it->hash())
			return false;
		it++;
	}
	return true;
}


/*
Input o, other CircuitMatch
Return unique device pointer, pair.first is DeviceInst in this CircuitMatch, pair.second is DeviceInst in o
Notes: caller should check return value pair.first == pair.second

pair<QSharedPointer<DeviceInst>, QSharedPointer<DeviceInst> > CircuitMatch::find_unique_dev(const CircuitMatch * o)
{
    multiset<QSharedPointer<DeviceInst>, DeviceInstCmp>::iterator oit = o->wait_pick.begin();
    multiset<QSharedPointer<DeviceInst>, DeviceInstCmp>::iterator it = wait_pick.begin();
    for (; oit != o->wait_pick.end(); oit++) {
        //start to find unique oit
        multiset<QSharedPointer<DeviceInst>, DeviceInstCmp>::iterator next_oit = oit;
        if (next_oit != o->wait_pick.end()) {
            HASH_TYPE unique_hash = (*next_oit)->hash;
            if ((*oit)->hash == unique_hash) { //not unique
                //step cross this hash
                while (oit != o->wait_pick.end() && (*oit)->hash == unique_hash)
                    oit++;
                if (oit == o->wait_pick.end()) //return not found
                    break;
                else
                    continue;
            }
        }
        //now oit is unique hash
        while (it != wait_pick.end() && (*oit)->hash > (*it)->hash)
            it++;
        if (it == wait_pick.end()) //return not found
            break;
        if ((*oit)->hash == (*it)->hash) {
            //check if it is unique hash
             multiset<QSharedPointer<DeviceInst>, DeviceInstCmp>::iterator next_it = it;
             if (next_it == wait_pick.end() || (*next_it)->hash != (*it)->hash)
                 return make_pair(*it, *oit);
        }
    }
    return make_pair(QSharedPointer<DeviceInst>(), QSharedPointer<DeviceInst>());
}
*/

/*
Notes: pick minimum equal dev, minimum can reduce time
*/
void CircuitMatch::pick_dev(const CircuitMatch * o, vector<pair<DeviceInst, DeviceInst > > & dp)
{
    dp.clear();
	multiset<DeviceInst, DeviceInstCmp>::iterator oit = o->wait_pick.begin(), best_oit;
	multiset<DeviceInst, DeviceInstCmp>::iterator it = wait_pick.begin(), best_it;
	int best_num = 0x8000, best_onum = 0x8000; //best_num * best_onum is minimum
	//1 because equal dev has same hash, find minimum match number with same hash
	for (; oit != o->wait_pick.end();) {
		multiset<DeviceInst, DeviceInstCmp>::iterator next_oit = oit;
		int onum = 0; //onum is hash repeat number in o
		while (next_oit != o->wait_pick.end() && oit->hash() == next_oit->hash()) {
			next_oit++;
			onum++;
		}
		//find same hash it with oit
		while (it != wait_pick.end() && oit->hash() > it->hash())
			it++;
		if (it == wait_pick.end() || oit->hash() < it->hash()) //this not contain oit, pick no dev
			return;
		
		//here we found it, compute repeat number
		multiset<DeviceInst, DeviceInstCmp>::iterator next_it = it;
		int num = 0; //num is hash repeat number in this
		while (next_it != wait_pick.end() && oit->hash() == next_it->hash()) {
			next_it++;
			num++;
		}
		if (onum * num < best_num * best_onum) {
			best_num = num;
			best_onum = onum;
			best_it = it;
			best_oit = oit;
		}
		oit = next_oit;
		it = next_it;
	}
	//following find equal pair
	oit = best_oit;
	for (int j = 0; j < best_onum; j++, oit++)  {
		it = best_it;
		bool find_equal = false;
		for (int i = 0; i < best_num; i++, it++) 		
		if (oit->equal(*it)) {
            dp.push_back(make_pair(*it, *oit));
			find_equal = true;
		}
		if (!find_equal) {
            dp.clear();
			return;
		}
	}
}
/*
Input circuit device idx
Return: pointer to DeviceInst if found device idx, else NULL
*/
multiset<DeviceInst, DeviceInstCmp>::iterator CircuitMatch::search_wait_pick(int d)
{
	multiset<DeviceInst, DeviceInstCmp>::iterator it;
	for (it = wait_pick.begin(); it != wait_pick.end(); it++)
	if (it->dev_idx() == d)
		return it;
	return wait_pick.end();
}

/*
 Input: n is idx in cir->nodes
 Return: if success, return 0, fail, return <0
 Notes: update wait_pick with all adj device
*/
int CircuitMatch::add_match_node(int n, vector<int> & affect_dm)
{
	affect_dm.clear();
	nm.push_back(n);
	int node_idx = (int) nm.size() - 1;
	vector<int> & cd = cir->nodes[n].cd; //cd is network n connected device
	for (int i = 0; i < (int)cd.size(); i++) {
		int dev_idx = cd[i];
		int dm_idx = search_device(dev_idx);
		if (dm_idx >= 0) {//in match device set,
			for (int j = 0; j < (int)cir->devs[dev_idx].cn.size(); j++)
			if (cir->devs[dev_idx].cn[j] == n)
				dm[dm_idx].set_port_net(j + 1, node_idx);
			dm[dm_idx].compute_hash();
			affect_dm.push_back(dm_idx);
		}
		else {			
			multiset<DeviceInst, DeviceInstCmp>::iterator it = search_wait_pick(dev_idx);
			if (it == wait_pick.end()) {  //found new adj device	
				DeviceInst pd(dev_idx, (int) cir->devs[dev_idx].cn.size() + 1);
				int type = search_subckt(cir->devs[dev_idx].type);
				if (type < 0) { //unknow subckt, return error directkly
					qWarning("found unknow subckt %s", cir->get_subckt_name(cir->devs[dev_idx].type).c_str());
					continue;
				}
				pd.set_port_net(0, type); //port 0 is for type, other is port				
				for (int j = 0; j < (int)cir->devs[dev_idx].cn.size(); j++)
					pd.set_port_net(j + 1, search_node(cir->devs[dev_idx].cn[j]));
				pd.compute_hash();
				wait_pick.insert(pd);
			}
			else { //in wait_pick set, delete it and reinsert
				DeviceInst pd(*it);
				wait_pick.erase(it);
				for (int j = 0; j < (int)cir->devs[dev_idx].cn.size(); j++)
				if (cir->devs[dev_idx].cn[j] == n)
					pd.set_port_net(j + 1, node_idx);
				pd.compute_hash();
				wait_pick.insert(pd);
			}
		}
	}
	return 0;
}

/*
Input: d, match device is idx in cir->devs
Output: nodes, match network adj to d,nodes[i] is idx in cir->nodes
*/
void CircuitMatch::add_match_device(int d, vector<int> & nodes)
{
	multiset<DeviceInst, DeviceInstCmp>::iterator it = search_wait_pick(d);
	Q_ASSERT(it != wait_pick.end());
	dm.push_back(*it);
	wait_pick.erase(it);
	nodes.clear();
	vector<int> & cn = cir->devs[d].cn;
	int type = cir->devs[d].type;
	if (type == DEVICE_R || type == DEVICE_L || type == DEVICE_C) {
		for (int i = 0; i < (int)cn.size(); i++)
		if (dm.back().get_port_net(i) == -1)
			nodes.push_back(cn[i]);
	}
	else
		for (int i = 0; i < (int)cn.size(); i++)
			nodes.push_back(cn[i]);
}

void CircuitMatch::init()
{
	wait_pick.clear();
	dm.clear();
	nm.clear();
	sm.clear();
	for (int i = 0; i < (int)predef_nm.size(); i++)
		nm.push_back(cir->search_node(predef_nm[i]));
	for (int i = 0; i < (int)predef_sm.size(); i++)
		sm.push_back(cir->search_subckt(predef_sm[i]).second);
}

bool CircuitMatch::try_match(CircuitMatch * cm0, CircuitMatch * cm1, MatchMethod method)
{
	vector<pair<DeviceInst, DeviceInst > > dp;
	for (;;) {
		if (cm1->wait_pick.empty())
			return true;        
        cm0->pick_dev(cm1, dp);
        if (dp.size() > 1)
			break;
        if (dp.size() == 0)
			return false;
		vector<int> node0, node1;
        cm0->add_match_device(dp[0].first.dev_idx(), node0);
        cm1->add_match_device(dp[0].second.dev_idx(), node1);
		Q_ASSERT(node0.size() == node1.size());
		for (int i = 0; i < (int)node0.size(); i++) //check node match
		switch (method) {
		case MATCH_ALL:
			if (cm0->cir->nodes[node0[i]].cd.size() != cm1->cir->nodes[node1[i]].cd.size()) //connect device num must ==
				return false;
			break;
		case MATCH_SUBCKT:
			if (cm0->cir->nodes[node0[i]].cd.size() < cm1->cir->nodes[node1[i]].cd.size())
				return false;
			if (!cm1->cir->is_port(node1[i])) //internal nodes device num must ==
			if (cm0->cir->nodes[node0[i]].cd.size() != cm1->cir->nodes[node1[i]].cd.size())
				return false;
			break;
		case MATCH_PART:
			if (cm0->cir->nodes[node0[i]].cd.size() < cm1->cir->nodes[node1[i]].cd.size()) //connect device num must >=
				return false;
			break;
		}
		//add match node
		for (int i = 0; i < node0.size(); i++) {
			int is_match = cm0->search_node(node0[i]);
			if (cm0->search_node(node0[i]) != cm1->search_node(node1[i]))
				return false;
			if (is_match >= 0) //node already match
				continue;
			vector<int> adm0, adm1;
			cm0->add_match_node(node0[i], adm0);
			cm1->add_match_node(node1[i], adm1);
			//check affect device still match
			if (adm0.size() != adm1.size())
				return false;
			for (int i = 0; i < (int)adm0.size(); i++)
			if (!(cm0->dm[adm0[i]].equal(cm1->dm[adm1[i]])))
				return false;
		}
	}
	CircuitMatch cm0_back = *cm0, cm1_back = *cm1;
	for (int i = 0; i < (int)dp.size(); i++) {
		bool check = true;
		vector<int> node0, node1;
		cm0->add_match_device(dp[i].first.dev_idx(), node0);
		cm1->add_match_device(dp[i].second.dev_idx(), node1);
		Q_ASSERT(node0.size() == node1.size());
		for (int i = 0; check && i < (int)node0.size(); i++) //check node match
		switch (method) {
		case MATCH_ALL:
			if (cm0->cir->nodes[node0[i]].cd.size() != cm1->cir->nodes[node1[i]].cd.size()) //connect device num must ==
				check = false;
			break;
		case MATCH_SUBCKT:
			if (cm0->cir->nodes[node0[i]].cd.size() < cm1->cir->nodes[node1[i]].cd.size())
				check = false;
			if (!cm1->cir->is_port(node1[i])) //internal nodes device num must ==
			if (cm0->cir->nodes[node0[i]].cd.size() != cm1->cir->nodes[node1[i]].cd.size())
				check = false;
			break;
		case MATCH_PART:
			if (cm0->cir->nodes[node0[i]].cd.size() < cm1->cir->nodes[node1[i]].cd.size()) //connect device num must >=
				check = false;
			break;
		}
		//add match node
		for (int i = 0; i < node0.size(); i++) {
			int is_match = cm0->search_node(node0[i]);
			if (cm0->search_node(node0[i]) != cm1->search_node(node1[i])) {
				check = false;
				break;
			}
			if (is_match >= 0) //node already match
				continue;
			vector<int> adm0, adm1;
			cm0->add_match_node(node0[i], adm0); //add new match node
			cm1->add_match_node(node1[i], adm1);
			//check affect device still match
			if (adm0.size() != adm1.size()) {
				check = false;
				break;
			}
			for (int i = 0; i < (int)adm0.size(); i++)
			if (!(cm0->dm[adm0[i]].equal(cm1->dm[adm1[i]]))) {
				check = false;
				break;
			}
		}
		if (check)
			check = try_match(cm0, cm1, method);
		if (check)
			return true;
		else {
			*cm0 = cm0_back;
			*cm1 = cm1_back;
		}
	}
	return false;
}

MatchResult try_match_node(const TryMatchNode & tm)
{
	CircuitMatch cm0 = *tm.cm0, cm1 = *tm.cm1;
	vector<int> adm0, adm1;
	MatchResult result;
	cm0.add_match_node(tm.n0, adm0);
	cm1.add_match_node(tm.n1, adm1);
	Q_ASSERT(adm0.empty() && adm1.empty());
	bool check = CircuitMatch::try_match(&cm0, &cm1, tm.method);

	if (check) {
		Q_ASSERT(cm0.nm.size() == cm1.nm.size());
		for (int i = 0; i < (int)cm0.nm.size(); i++)
			result.mnodes.push_back(make_pair(cm0.cir->get_node_name(cm0.nm[i]), cm1.cir->get_node_name(cm1.nm[i])));
		Q_ASSERT(cm0.dm.size() == cm1.dm.size());
		for (int i = 0; i < (int)cm0.dm.size(); i++)
			result.mdev.push_back(make_pair(cm0.cir->get_dev_name(cm0.dm[i].dev_idx()), cm1.cir->get_dev_name(cm1.dm[i].dev_idx())));
	}
	
	if (check) {
		//self check circuit match
		for (int i = 0; i < (int)cm0.dm.size(); i++) {
			Q_ASSERT(cm0.dm[i].get_port_net_num() == cm1.dm[i].get_port_net_num());			
			for (int j = 0; j < cm0.dm[i].get_port_net_num(); j++)
				Q_ASSERT(cm0.dm[i].get_port_net(j) == cm1.dm[i].get_port_net(j));
			for (int j = 0; j < cm0.dm[i].get_port_net_num() - 1; j++) {
				Q_ASSERT(cm0.cir->get_dev_cn(cm0.dm[i].dev_idx(), j) == cm0.nm[cm0.dm[i].get_port_net(j + 1)]);
				Q_ASSERT(cm1.cir->get_dev_cn(cm1.dm[i].dev_idx(), j) == cm1.nm[cm1.dm[i].get_port_net(j + 1)]);
			}			
		}
	}
	return result;
}

static void merge_result(vector<MatchResult> & result, const MatchResult & t)
{
	if (!t.mdev.empty())
		result.push_back(t);
}

void match(CircuitMatch & one, CircuitMatch & other, string subckt_name, string other_subckt_name, MatchMethod method, vector<MatchResult> result)
{
	result.clear();
	if (one.root_cir == NULL || other.root_cir == NULL)
		return;
	one.cir = one.root_cir->search_subckt(subckt_name).first;
	other.cir = other.root_cir->search_subckt(other_subckt_name).first;
	if (one.cir == NULL || other.cir == NULL)
		return;
	one.init();
	other.init();
	vector<int> sn0, sn1;
	one.cir->get_sorted_nodes(sn0);
	other.cir->get_sorted_nodes(sn1);
	int connect_th, good_node;
	for (int i = 0; i < sn1.size(); i++)
	if (find(other.nm.begin(), other.nm.end(), sn1[i]) == other.nm.end()) {
		connect_th = (int)other.cir->get_node_cd_size(sn1[i]);
		good_node = i;
		break;
	}
	vector<TryMatchNode> tms;
	for (int i = 0; i < (int)sn0.size(); i++)
	if (one.cir->get_node_cd_size(sn0[i]) >= connect_th) {
		if (find(one.nm.begin(), one.nm.end(), sn0[i]) == one.nm.end())
			tms.push_back(TryMatchNode(&one, &other, method, sn0[i], sn1[good_node]));
	}
	else
		break;
	
	for (int i = 0; i < (int)tms.size(); i++) {
		MatchResult t = try_match_node(tms[i]);
		merge_result(result, t);
	}

	FILE * fp = fopen("match_result.txt", "w");
	for (int i = 0; i < (int)result.size(); i++) {
		fprintf(fp, "Match %d for %s and %s, Nodes\n", i + 1, subckt_name.c_str(), other_subckt_name.c_str());
		for (int j = 0; (int)j < result[i].mnodes.size(); j++)
			fprintf(fp, "%20s --> %20s\n", result[i].mnodes[j].first.c_str(), result[i].mnodes[j].second.c_str());
		fprintf(fp, "Devices\n");
		for (int j = 0; (int)j < result[i].mdev.size(); j++)
			fprintf(fp, "%20s --> %20s\n", result[i].mdev[j].first.c_str(), result[i].mdev[j].second.c_str());
	}
	fclose(fp);
}
