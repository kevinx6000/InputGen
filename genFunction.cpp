// Header
#include "header.h"
#include "gen.h"

// Initializer
void GenInput::initialize(int k, int numOfFlow){

	// Variable
	int totalSwitch;
	int src, dst, mid;
	double x1, x2, y1, y2;
	Switch stmp;
	Link ltmp;
	NodeCap ntmp;
	map<int, int>mtmp;

	// Constants
	const double feet = 0.3048;
	const double inch = 0.0254;
	const double widSw = 24*inch;
	const double lenSw = 48*inch;

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

	// Default: wired links
	ltmp.isWireless = false;

	// Create switch
	for(int i = 0; i < totalSwitch; i++){
		stmp.ID = i;
		switches.push_back(stmp);
	}

	// X-Y positions of ToR switch
	for(int i = 0; i < numOfEdge; i++){
		switches[numOfCore + numOfAggr + i].posXY[0] = (i % (k/2))*widSw + 0.5*widSw + ((i / (k/2)) % 4) * (10 * feet + (k/2) * widSw);
		switches[numOfCore + numOfAggr + i].posXY[1] = 0.5*lenSw + (i / (k*4/2)) * (lenSw + 8*feet);
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

	// Link: Edge - Edge
	// Wireless link
	ltmp.isWireless = true;
	for(int i = 0; i < numOfEdge; i++){
		src = numOfCore + numOfAggr + i;
		for(int j = 0; j < numOfEdge; j++){
			dst = numOfCore + numOfAggr + j;
			if(src == dst) continue;

			// Distance
			x1 = switches[src].posXY[0];
			y1 = switches[src].posXY[1];
			x2 = switches[dst].posXY[0];
			y2 = switches[dst].posXY[1];
			if(dis(x1, y1, x2, y2) <= WIRELESS_RANGE){

				// Src -> Dst
				switches[src].port.push_back(dst);
				switches[src].linkID.push_back(links.size());
				linkMap[src][dst] = links.size();
				ltmp.srcID = src;
				ltmp.dstID = dst;

				// Interference list
				for(int z = 0; z < numOfEdge; z++){
					mid = numOfCore + numOfAggr + z;
					if(src == mid) continue;

					// Position and vector operation
					if(vecdot(switches[src].posXY, switches[dst].posXY, switches[src].posXY, switches[mid].posXY) > 0 &&
						vecdot(switches[src].posXY, switches[dst].posXY, switches[mid].posXY, switches[dst].posXY) >= 0 &&
						vecdis(switches[src].posXY, switches[dst].posXY, switches[src].posXY, switches[mid].posXY) <= 11*inch){
						ltmp.iList.push_back(mid);
					}
				}
				links.push_back(ltmp);
				ltmp.iList.clear();
			}
		}

		// Transceiver and interference node
		switches[src].trancID = trancNode.size();
		ntmp.ID = src;
		trancNode.push_back(ntmp);
		switches[src].interID = interNode.size();
		interNode.push_back(ntmp);
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
	bool found, isWireless;
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
				isWireless = (rand()%2 == 1);
				tryCnt = 0;
				while(tryCnt < 10 && !findPath(ptmp.hop[0], traffic, isWireless, srcID, dstID))
					tryCnt++;
				if(tryCnt < 10){
					found = true;
					break;
				}
			}
		}
		if(!found){
			fprintf(stderr, "[Error] Cannot found path for all destination from %d\n", srcID);
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
	bool found, isWireless;
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
				isWireless = (rand()%2 == 1);
				tryCnt = 0;
				while(tryCnt < 10 && !findPath(flows[flowID].pathFlow[0].hop[1], traffic, isWireless, srcID, newID))
					tryCnt++;
				if(tryCnt < 10){
					found = true;
					break;
				}
			}
		}
		if(!found){
			fprintf(stderr, "[Error] Cannot found path for all destination from %d\n", srcID);
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
	
	// Transceiver node
	for(int i = 0; i < (int)trancNode.size(); i++)
		trancNode[i].nodeCapacity = LINK_CAPACITY;
	
	// Interference node
	for(int i = 0; i < (int)interNode.size(); i++)
		interNode[i].nodeCapacity = LINK_CAPACITY;
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
			fprintf(stderr, "[Error] No enough resource [%d] = %.2lf < %.2lf", linkID, links[linkID].linkCapacity, traffic);
			if(links[linkID].isWireless) fprintf(stderr, "(wireless link).\n");
			else fprintf(stderr, "(wired link).\n");
			exit(1);
		}
		links[linkID].linkCapacity -= traffic;

		// Wireless link
		if(links[linkID].isWireless){

			// Transceiver
			if(trancNode[ switches[srcID].trancID ].nodeCapacity < traffic || trancNode[ switches[dstID].trancID ].nodeCapacity < traffic){
				fprintf(stderr, "[Error] No enough resource (transceiver node).\n");
				exit(1);
			}
			trancNode[ switches[srcID].trancID ].nodeCapacity -= traffic;
			trancNode[ switches[dstID].trancID ].nodeCapacity -= traffic;

			// Interference
			for(int j = 0; j < (int)links[linkID].iList.size(); j++){
				srcID = links[linkID].iList[j];
				if(interNode[ switches[srcID].trancID ].nodeCapacity < traffic){
					fprintf(stderr, "[Error] No enough resource (interference node).\n");
					exit(1);
				}
				interNode[ switches[srcID].interID ].nodeCapacity -= traffic;
			}
		}
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
bool GenInput::findPath(vector<Hop>& hopList, double traffic, bool isWireless, int srcID, int dstID){

	// Variables
	int pod1, pod2;
	int aggrID, coreID, linkID, interID;
	int nowID, nxtID;
	bool found, ok;
	Hop htmp;
	vector<int>randList;
	vector<Hop>ansHop;
	vector<Link>copyLink;
	vector<NodeCap>copyTranc;
	map<int, int>vis;
	BFSNode Bnow, Bnxt, Bfinal;
	queue<BFSNode>que;

	// Initialize
	ansHop.clear();
	pod1 = (srcID - numOfCore - numOfAggr) / (pod/2);
	pod2 = (dstID - numOfCore - numOfAggr) / (pod/2);

	// Wireless path
	if(isWireless){

		// Copy resource
		copyLink = links;
		copyTranc = trancNode;

		// BFS
		found = false;
		Bnow.ID = srcID;
		Bnow.hopList.clear();
		Bnow.inter = interNode;
		vis.clear();
		vis[srcID] = true;
		que.push(Bnow);
		while(!found && !que.empty()){
			Bnow = que.front();
			que.pop();
			nowID = Bnow.ID;

			// Transceiver node
			if(copyTranc[ switches[nowID].trancID ].nodeCapacity < traffic) continue;
			copyTranc[ switches[nowID].trancID ].nodeCapacity -= traffic;

			// Neighbor
			for(int i = 0; i < (int)switches[nowID].port.size(); i++){
				nxtID = switches[nowID].port[i];
				linkID = linkMap[nowID][nxtID];
				if(!links[linkID].isWireless) continue;

				// Check feasibility
				if(!vis[nxtID]){

					// Transceiver node
					if(copyTranc[ switches[nxtID].trancID ].nodeCapacity < traffic) continue;

					// Interference
					ok = true;
					for(int j = 0; j < (int)links[linkID].iList.size(); j++){
						interID = links[linkID].iList[j];
						if(Bnow.inter[ switches[interID].interID ].nodeCapacity < traffic){
							ok = false;
							break;
						}
					}

					// OK
					if(ok){

						// Update resource
						Bnxt = Bnow;
						Bnxt.ID = nxtID;
						copyTranc[ switches[nxtID].trancID ].nodeCapacity -= traffic;
						for(int j = 0; j < (int)links[linkID].iList.size(); j++){
							interID = links[linkID].iList[j];
							Bnxt.inter[ switches[interID].interID ].nodeCapacity -= traffic;
						}

						// Record hop
						htmp.srcID = nowID;
						htmp.dstID = nxtID;
						Bnxt.hopList.push_back(htmp);

						// End
						if(nxtID == dstID){
							found = true;
							Bfinal = Bnxt;
							break;
						}

						// Inqueue
						vis[nxtID] = true;
						que.push(Bnxt);
					}
				}
			}
		}		
		if(!found) return false;
		ansHop = Bfinal.hopList;
	}

	// Wired path
	else{

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
	}

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
