// Header
#include "header.h"
#include "gen.h"

// Initializer
void GenInput::initialize(int k, int numOfFlow){

	// Variable
	int totalSwitch;
	int src, dst;
	Switch stmp;
	Link ltmp;
	map<int, int>mtmp;

	// Copy parameters
	this->pod = k;
	this->numOfFlow = numOfFlow;

	// Number of switches
	this->numOfCore = k*k/4;
	this->numOfAggr = k*k/2;
	this->numOfEdge = k*k/2;
	totalSwitch = numOfCore + numOfAggr + numOfEdge;

	// Initialize link map
	for(int i = 0; i < totalSwitch; i++)
		linkMap.push_back(mtmp);

	// Create switch
	for(int i = 0; i < totalSwitch; i++){
		stmp.ID = i;
		switches.push_back(stmp);
	}

	// Link: Core - Aggregate
	for(int i = 0; i < numOfAggr; i++){
		src = numOfCore + i;
		for(int j = 0; j < k/2; j++){
			dst = (i % (k/2)) * (k/2) + j;

			// Aggr -> Core
			switches[src].port.push_back(dst);
			switches[src].linkID.push_back(links.size());
			linkMap[src][dst] = links.size();
			ltmp.srcID = src;
			ltmp.dstID = dst;
			links.push_back(ltmp);

			// Core -> Aggr
			switches[dst].port.push_back(src);
			switches[dst].linkID.push_back(links.size());
			linkMap[dst][src] = links.size();
			ltmp.srcID = dst;
			ltmp.dstID = src;
			links.push_back(ltmp);
		}
	}

	// Link: Aggregate - Edge
	for(int i = 0; i < numOfAggr; i++){
		src = numOfCore + i;
		for(int j = 0; j < k/2; j++){
			dst = numOfCore + numOfAggr + (i / (k/2)) * (k/2) + j;

			// Aggr -> Edge
			switches[src].port.push_back(dst);
			switches[src].linkID.push_back(links.size());
			linkMap[src][dst] = links.size();
			ltmp.srcID = src;
			ltmp.dstID = dst;
			links.push_back(ltmp);

			// Edge -> Aggr
			switches[dst].port.push_back(src);
			switches[dst].linkID.push_back(links.size());
			linkMap[dst][src] = links.size();
			ltmp.srcID = dst;
			ltmp.dstID = src;
			links.push_back(ltmp);
		}
	}

	// Clear flows
	flows.clear();

	// Time seed
	srand((unsigned)clock());
}

// Generate initial state
void GenInput::genInitial(void){

	// Variable
	int tryCnt;
	int srcID, dstID;
	bool found;
	double traffic;
	Flow ftmp;
	PathFlow ptmp;
	vector<int>randList;

	// Clear resource
	clearResource();

	// Generate flows
	for(int flowID = 0; flowID < numOfFlow; flowID++){

		// Source/Traffic
		srcID = numOfCore + numOfAggr + rand()%numOfEdge;
		traffic = genTraffic();

		// Pick random destination until found
		found = false;
		genRandList(randList, numOfEdge);
		for(int i = 0; i < numOfEdge; i++){
			dstID = numOfCore + numOfAggr + randList[i];
			if(srcID != dstID){
				tryCnt = 0;
				while(tryCnt < 10 && !findPath(ptmp.hop[0], traffic, srcID, dstID)) tryCnt++;
				if(tryCnt < 10){
					found = true;
					break;
				}
			}
		}
		if(!found){
//			fprintf(stderr, "[Error] Cannot found path for all destination from %d\n", srcID);
fprintf(stderr, "Fail\n");
			exit(1);
		}

		// Occupy resource
		occupyRes(ptmp.hop[0], traffic);

		// Record to flow list
		ftmp.src = srcID;
		ptmp.traffic = traffic;
		ftmp.pathFlow.clear();
		ftmp.pathFlow.push_back(ptmp);
		flows.push_back(ftmp);
	}
}

// Generate final state
void GenInput::genFinal(void){

	// Variables
	int tryCnt;
	int srcID, dstID, newID;
	bool found;
	double traffic;
	vector<int>randList;
	map<int, bool>picked;

	// Clear resource
	clearResource();

	// Pick up some flow, set final as initial
	picked.clear();
	for(int flowID = 0; flowID < (int)flows.size(); flowID++){
		if(rand()%2) continue;
		srcID = flows[flowID].src;
		dstID = flows[flowID].pathFlow[0].hop[0][ flows[flowID].pathFlow[0].hop[0].size()-1 ].dstID;
		traffic = flows[flowID].pathFlow[0].traffic;
		flows[flowID].pathFlow[0].hop[1] = flows[flowID].pathFlow[0].hop[0];
		picked[flowID] = true;

		// Occupy resource
		occupyRes(flows[flowID].pathFlow[0].hop[1], traffic);
	}

	// For each initial flow
	for(int flowID = 0; flowID < (int)flows.size(); flowID++){
		if(picked[flowID]) continue;
		srcID = flows[flowID].src;
		dstID = flows[flowID].pathFlow[0].hop[0][ flows[flowID].pathFlow[0].hop[0].size()-1 ].dstID;
		traffic = flows[flowID].pathFlow[0].traffic;

		// Pick up a random destination other than original one
		found = false;
		genRandList(randList, numOfEdge);
		for(int i = 0; i < numOfEdge; i++){
			newID = numOfCore + numOfAggr + randList[i];
			if(newID != srcID && newID != dstID){
				tryCnt = 0;
				while(tryCnt < 10 && !findPath(flows[flowID].pathFlow[0].hop[1], traffic, srcID, newID)) tryCnt++;
				if(tryCnt < 10){
					found = true;
					break;
				}
			}
		}
		if(!found){
//			fprintf(stderr, "[Error] Cannot found path for all destination from %d\n", srcID);
fprintf(stderr, "Fail\n");
			exit(1);
		}

		// Occupy resource
		occupyRes(flows[flowID].pathFlow[0].hop[1], traffic);
	}
}

// Clear resource
void GenInput::clearResource(void){

	// Switch
	for(int i = 0; i < (int)switches.size(); i++)
		switches[i].tcamUsage = TCAM_CAPACITY;
	
	// Link
	for(int i = 0; i < (int)links.size(); i++)
		links[i].linkCapacity = LINK_CAPACITY;
}

// Occurpy resource
void GenInput::occupyRes(const vector<Hop>& hopList, double traffic){

	// Variable
	int srcID, dstID, linkID;

	// Update remaining capacity
	for(int i = 0; i < (int)hopList.size(); i++){
		srcID = hopList[i].srcID;
		dstID = hopList[i].dstID;
		linkID = linkMap[srcID][dstID];
		if(links[linkID].linkCapacity < traffic){
//			fprintf(stderr, "[Error] No enough resource [%d] = %.2lf < %.2lf", linkID, links[linkID].linkCapacity, traffic);
//			if(links[linkID].isWireless) fprintf(stderr, "(wireless link).\n");
//			else fprintf(stderr, "(wired link).\n");
fprintf(stderr, "Fail\n");
			exit(1);
		}
		links[linkID].linkCapacity -= traffic;
	}
}

// Generate traffic according to some distribution
double GenInput::genTraffic(void){

	// Traffic distribution
	double T[11] = {
		0.0001,
		0.000805842,
		0.002053525,
		0.004097321,
		0.008116616,
		0.019109530,
		0.062643354,
		0.273841963,
		1.0,
		6.493816315,
		133.352143216
	};

	// Randomly pick one interval
	int intval = rand()%10;

	// Randomly pick with uniform distribution in the picked interval
	return T[intval] + ( (T[intval+1] - T[intval]) * (rand()%100) ) / 100;
}

// Find path from source to destination using traffic with wired/wireless path
bool GenInput::findPath(vector<Hop>& hopList, double traffic, int srcID, int dstID){

	// Variables
	int pod1, pod2;
	int aggrID, coreID, linkID;
	bool found;
	Hop htmp;
	vector<int>randList;
	vector<Hop>ansHop;

	// Initialize
	ansHop.clear();
	pod1 = (srcID - numOfCore - numOfAggr) / (pod/2);
	pod2 = (dstID - numOfCore - numOfAggr) / (pod/2);

	// Wired path only

	// Edge(source) -> Aggr
	found = false;
	genRandList(randList, pod/2);
	for(int i = 0; i < pod/2; i++){
		aggrID = numOfCore + pod1 * (pod/2) + randList[i];
		linkID = linkMap[srcID][aggrID];
		if(links[linkID].linkCapacity >= traffic){
			found = true;
			break;
		}
	}
	if(!found) return false;
	htmp.srcID = srcID;
	htmp.dstID = aggrID;
	ansHop.push_back(htmp);

	// Different pod
	if(pod1 != pod2){

		// Aggr -> Core
		found = false;
		genRandList(randList, pod/2);
		for(int i = 0; i < pod/2; i++){
			coreID = ((aggrID - numOfCore) % (pod/2)) * (pod/2) + randList[i];
			linkID = linkMap[aggrID][coreID];
			if(links[linkID].linkCapacity >= traffic){
				found = true;
				break;
			}
		}
		if(!found) return false;
		htmp.srcID = aggrID;
		htmp.dstID = coreID;
		ansHop.push_back(htmp);

		// Core -> Aggr
		aggrID = numOfCore + pod2 * (pod/2) + coreID / (pod/2);
		linkID = linkMap[coreID][aggrID];
		if(links[linkID].linkCapacity < traffic) return false;
		htmp.srcID = coreID;
		htmp.dstID = aggrID;
		ansHop.push_back(htmp);
	}

	// Aggr -> Edge(destination)
	linkID = linkMap[aggrID][dstID];
	if(links[linkID].linkCapacity < traffic) return false;
	htmp.srcID = aggrID;
	htmp.dstID = dstID;
	ansHop.push_back(htmp);

	// Copy back
	hopList = ansHop;
	return true;
}

// Generate random list (not repeated)
void GenInput::genRandList(vector<int>& randList, int size){

	// Variable
	int tmp, pos;

	// First put 0 ~ size-1 into list
	randList.clear();
	for(int i = 0; i < size; i++)
		randList.push_back(i);

	// Randomly swap each element
	for(int i = 0; i < size-1; i++){
		pos = i + rand()%(size-i);
		tmp = randList[pos];
		randList[pos] = randList[i];
		randList[i] = tmp;
	}
}

// Output flow
void GenInput::output(void){

	// Pod
	printf("%d\n", pod);

	// Number of flows
	printf("%d\n", (int)flows.size());

	// For each flows
	for(int flowID = 0; flowID < (int)flows.size(); flowID++){
		// Ingress switch ID
		printf("%d\n", flows[flowID].src);

		// For each path flows
		printf("%d\n", (int)flows[flowID].pathFlow.size());
		for(int pathID = 0; pathID < (int)flows[flowID].pathFlow.size(); pathID++){

			// Traffic volume
			printf("%.10lf\n", flows[flowID].pathFlow[pathID].traffic);

			// Initial and final state
			for(int state = 0; state < 2; state++){

				// For each hop
				printf("%d\n", (int)flows[flowID].pathFlow[pathID].hop[state].size());
				for(int hop = 0; hop < (int)flows[flowID].pathFlow[pathID].hop[state].size(); hop++){
					printf("%d %d\n", flows[flowID].pathFlow[pathID].hop[state][hop].srcID, flows[flowID].pathFlow[pathID].hop[state][hop].dstID);
				}
			}
		}
	}
}

bool GenInput::testPort(int src, int dst){

	int i;
	for(i = 0; i < (int)switches[src].port.size(); i++){
		if(switches[src].port[i] == dst) return true;
	}
	return false;
}
