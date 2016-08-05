// Header
#include "header.h"
#include "gen.h"

// Initializer
void GenInput::initialize(int k){

	// Variable
	int totalSwitch;
	int src, dst;
	Switch stmp;
	Link ltmp;
	map<int, int>mtmp;

	// Pod
	this->pod = k;

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
void GenInput::genInitial(int cycleLength){

	// Variable
	int podID, curID, picked;
	Flow ftmp;
	PathFlow ptmp;
	ChainRes ctmp;
	vector<int>randList;

	// Clear resource
	clearResource();

	// Randomly pick one pod
	podID = rand()%pod;

	// For each aggregate switch
	picked = 0;
	chainRes.clear();
	for(int i = 0; i < pod/2; i++){
		ctmp.aggrID = numOfCore + podID * (pod/2) + i;

		// At most pod/3 core is picked
		genRandList(randList, pod/2);
		for(int j = 0; j < pod/3 && picked < cycleLength; j++){
			ctmp.coreID = ((ctmp.aggrID - numOfCore) % (pod/2)) * (pod/2) + randList[j];
			ctmp.rID = linkMap[ctmp.aggrID][ctmp.coreID];
			ctmp.maxRate = 0.0;
			chainRes.push_back(ctmp);
			picked++;
		}
	}

//fprintf(stderr, "[Info] Cycle len = %d\n", (int)chainRes.size());

	// Fill the last one to 95% capacity
	curID = chainRes.size()-1;
	while(true){

		// Traffic data rate
		ptmp.traffic = genTraffic();

		// Feasible: 
		if(findWiredPath(ptmp.hop[0], ptmp.traffic, podID, chainRes[curID].coreID, chainRes[curID].aggrID)){

			// Occupy the resource along the path
			occupyRes(ptmp.hop[0], ptmp.traffic);

			// Record and update flow information
			if(chainRes[curID].maxRate < ptmp.traffic){
				chainRes[curID].maxFlowID = flows.size();
				chainRes[curID].maxRate = ptmp.traffic;
			}

			// Record to flow set
			ftmp.src = ptmp.hop[0][0].srcID;
			ftmp.pathFlow.clear();
			ftmp.pathFlow.push_back(ptmp);
			flows.push_back(ftmp);
		}

		// Not feasible: find out another path
		else{

			// Randomly pick another wired paths,
			// which does not pass through current pod
			int retry = 0;
			while(!findAnotherPath(ptmp.hop[0], ptmp.traffic, podID)){
				retry ++;
				if(retry > 10){
//					fprintf(stderr, "[Error] Solution not found, GG.\n");
fprintf(stderr, "Fail\n");
					exit(1);
				}
			}

			// Occupy the resource along the path
			occupyRes(ptmp.hop[0], ptmp.traffic);

			// Record to flow set
			ftmp.src = ptmp.hop[0][0].srcID;
			ftmp.pathFlow.clear();
			ftmp.pathFlow.push_back(ptmp);
			flows.push_back(ftmp);
		}

		// Reach 95% or more (remain 5% or less)
		if(links[ chainRes[curID].rID ].linkCapacity <= LINK_CAPACITY * 0.05) break;
	}

	// Start from last-1 one to the first one
	for(curID = chainRes.size()-2; curID >= 0; curID--){

		// Until chain exist
		while(true){

			// Traffic data rate
			ptmp.traffic = genTraffic();

			// Feasible (1.Can migrate 2.Path found): record and update
			if(myMax(ptmp.traffic, chainRes[curID].maxRate) <= links[ chainRes[curID+1].rID ].linkCapacity + chainRes[curID+1].maxRate
				&& findWiredPath(ptmp.hop[0], ptmp.traffic, podID, chainRes[curID].coreID, chainRes[curID].aggrID)){

				// Occupy the resource along the path
				occupyRes(ptmp.hop[0], ptmp.traffic);

				// Record and update flow information
				if(chainRes[curID].maxRate < ptmp.traffic){
					chainRes[curID].maxFlowID = flows.size();
					chainRes[curID].maxRate = ptmp.traffic;
				}

				// Record to flow set
				ftmp.src = ptmp.hop[0][0].srcID;
				ftmp.pathFlow.clear();
				ftmp.pathFlow.push_back(ptmp);
				flows.push_back(ftmp);
			}

			// Not feasible: find out another path
			else{

				// Randomly pick another wired paths,
				// which does not pass through current pod
				int retry = 0;
				while(!findAnotherPath(ptmp.hop[0], ptmp.traffic, podID)){
					retry ++;
					if(retry > 10){
//						fprintf(stderr, "[Error] Solution not found, GG.\n");
fprintf(stderr, "Fail\n");
						exit(1);
					}
				}

				// Occupy the resource along the path
				occupyRes(ptmp.hop[0], ptmp.traffic);
				
				// Record to flow set
				ftmp.src = ptmp.hop[0][0].srcID;
				ftmp.pathFlow.clear();
				ftmp.pathFlow.push_back(ptmp);
				flows.push_back(ftmp);
			}

			// End:
			// 1. Reach 95% or more (remain 5% or less)
			// 2. Chain created
			if(links[ chainRes[curID].rID ].linkCapacity <= LINK_CAPACITY * 0.05
			&& chainRes[curID].maxRate > links[ chainRes[curID+1].rID ].linkCapacity){

				// Break condition:
				// 1. Not the first one
				// 2. First one and last migration cause cycle
				if(curID != 0 || links[ chainRes[curID].rID ].linkCapacity < chainRes[ chainRes.size()-1 ].maxRate) break;
			}
		}
	}
}

// Generate final state
void GenInput::genFinal(void){

	// Variables
	int aggrID, coreID, edgeID, podID;
	int len;
	double traffic;
	map<int, bool>isCompFlow;

	// Clear resource
	clearResource();

	// Record the competition flow
	isCompFlow.clear();
	for(int i = 0; i < (int)chainRes.size(); i++)
		isCompFlow[chainRes[i].maxFlowID] = true;

	// Assume flows without competition remain the same
	for(int i = 0; i < (int)flows.size(); i++){

		// Skip competition flow
		if(isCompFlow[i]) continue;

		// Currently, all flows only have one path flow
		flows[i].pathFlow[0].hop[1] = flows[i].pathFlow[0].hop[0];

		// Occupy it!
		occupyRes(flows[i].pathFlow[0].hop[1], flows[i].pathFlow[0].traffic);
	}

	// DEADLOCK, need to fix
	len = chainRes.size();
	for(int i = 0; i < len; i++){
		if(chainRes[i].maxRate > links[chainRes[(i+1)%len].rID].linkCapacity + chainRes[(i+1)%len].maxRate){
//			fprintf(stderr, "[Error] Sorry, such plan exists deadlock.\n");
fprintf(stderr, "Fail\n");
			exit(1);
		}
	}

	// Pod ID: pick the first chain resource to check
	podID = (links[ chainRes[0].rID ].srcID - numOfCore) / (pod/2);

	// For each flow in the chain: choose compete resource of initial path of next flow
	for(int i = 0; i < len; i++){
		aggrID = links[ chainRes[(i+1)%len].rID ].srcID;
		coreID = links[ chainRes[(i+1)%len].rID ].dstID;
		edgeID = flows[ chainRes[i].maxFlowID ].src;
		traffic = chainRes[i].maxRate;
		if(!findWiredPath(flows[ chainRes[i].maxFlowID ].pathFlow[0].hop[1], traffic, podID, coreID, aggrID, edgeID)){
//			fprintf(stderr, "[Error] GG, cannot find such a path to gen cycle.\n");
fprintf(stderr, "Fail\n");
			exit(1);
		}

		// Occupy it!
		occupyRes(flows[ chainRes[i].maxFlowID ].pathFlow[0].hop[1], traffic);
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
//			fprintf(stderr, "[Error] No enough resource ");
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

// Find wired path passing through specific link (without edgeID)
bool GenInput::findWiredPath(vector<Hop>& hopList, double traffic, int podID, int coreID, int aggrID){

	// Variables
	int i, edgeID, linkID;
	vector<int>randList;

	// Edge -> Aggr
	genRandList(randList, pod/2);
	for(i = 0; i < pod/2; i++){
		edgeID = numOfCore + numOfAggr + podID*(pod/2) + randList[i];
		linkID = linkMap[edgeID][aggrID];
		if(links[linkID].linkCapacity >= traffic) break;
	}
	if(i == pod/2){
//		fprintf(stderr, "[Info] Edge -> Aggr failed. (all full QQ)\n");
		for(i = 0; i < pod/2; i++){
			edgeID = numOfCore + numOfAggr + podID*(pod/2) + randList[i];
			linkID = linkMap[edgeID][aggrID];
//			fprintf(stderr, "[Info] Edge %d -> Aggr %d = %.2lf (need %.2lf)\n", edgeID, aggrID, links[linkID].linkCapacity, traffic);
		}
		return false;
	}
	return findWiredPath(hopList, traffic, podID, coreID, aggrID, edgeID);
}

// Find wired path passing through specific link (with edgeID)
bool GenInput::findWiredPath(vector<Hop>& hopList, double traffic, int podID, int coreID, int aggrID, int edgeID){

	// Variables
	int i, linkID, podID2;
	Hop htmp;
	vector<Hop>ansTemp;
	vector<int>randList;

	// Clear hop list
	ansTemp.clear();

	// Edge -> Aggr
	linkID = linkMap[edgeID][aggrID];
	if(links[linkID].linkCapacity < traffic) return false;
	htmp.srcID = edgeID;
	htmp.dstID = aggrID;
	ansTemp.push_back(htmp);

	// Aggr -> Core
	linkID = linkMap[aggrID][coreID];
	if(links[linkID].linkCapacity < traffic){
//		fprintf(stderr, "[Info] Aggr -> Core failed.\n");
		return false;
	}
	htmp.srcID = aggrID;
	htmp.dstID = coreID;
	ansTemp.push_back(htmp);

	// Pick random destination pod (other than current one)
	genRandList(randList, pod);
	for(i = 0; i < pod; i++){
		if(randList[i] != podID){
			podID2 = randList[i];
			break;
		}
	}
	if(i == pod){
//		fprintf(stderr, "[Info] Pod picking failed.\n");
		return false;
	}

	// Core -> Aggr
	genRandList(randList, pod/2);
	for(i = 0; i < pod/2; i++){
		aggrID = numOfCore + podID2*(pod/2) + coreID/(pod/2);
		linkID = linkMap[coreID][aggrID];
		if(links[linkID].linkCapacity >= traffic) break;
	}
	if(i == pod/2){
//		fprintf(stderr, "[Info] Core -> Aggr failed.\n");
		return false;
	}
	htmp.srcID = coreID;
	htmp.dstID = aggrID;
	ansTemp.push_back(htmp);

	// Aggr -> Edge
	genRandList(randList, pod/2);
	for(i = 0; i < pod/2; i++){
		edgeID = numOfCore + numOfAggr + podID2*(pod/2) + randList[i];
		linkID = linkMap[aggrID][edgeID];
		if(links[linkID].linkCapacity >= traffic) break;
	}
	if(i == pod/2){
//		fprintf(stderr, "[Info] Aggr -> Edge failed.\n");
		return false;
	}
	htmp.srcID = aggrID;
	htmp.dstID = edgeID;
	ansTemp.push_back(htmp);

	// All links are OK, copy back
	hopList = ansTemp;
	return true;
}

// Find aother path which does not pass through specific pod
bool GenInput::findAnotherPath(vector<Hop>& hopList, double traffic, int podID){

	// Variable
	int i, j, podID2, aggrID, coreID;
	vector<int>randList;

	// Pick a pod other than podID
	genRandList(randList, pod);
	for(i = 0; i < pod; i++){
		if(randList[i] != podID){
			podID2 = randList[i];
			break;
		}
	}
	if(i == pod){
		fprintf(stderr, "[Info] I think this is impossible...\n");
		return false;
	}

	// Decide aggr -> core
	genRandList(randList, pod/2);
	for(i = 0; i < pod/2; i++){
		aggrID = numOfCore + podID2*(pod/2) + randList[i];
		for(j = 0; j < pod/2; j++){
			coreID = ((aggrID-numOfCore)%(pod/2))*(pod/2) + j;
			if(findWiredPath(hopList, traffic, podID2, coreID, aggrID)) return true;
		}
	}
//	fprintf(stderr, "[Info] All path for pod = %d is full\n", podID2);
	return false;
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
