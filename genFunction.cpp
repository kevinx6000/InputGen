// Header
#include "header.h"
#include "gen.h"

// Initializer
void GenInput::initialize(int k){

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

	// Time seed
	srand((unsigned)clock());
}

// Generate input
void GenInput::genInput(void){

	// Variable
	bool hasCycle;
	double traffic;
	Flow ftmp;
	CycleRes cycleRes[2];

	// Clear flows
	flows.clear();

	// Clear resource and copy
	clearResource();
	for(int i = 0; i < 2; i++){
		cycleRes[i].switches = switches;
		cycleRes[i].links = links;
		cycleRes[i].trancNode = trancNode;
		cycleRes[i].interNode = interNode;
	}

	// Until cycle found
	hasCycle = false;
	while(!hasCycle){

		// DEBUG
		fprintf(stderr, "[Info] Current flows = %d\n", (int)flows.size());

		// Generate traffic
		traffic = genTraffic();

		// Generate initial state
		if(!genInitial(cycleRes[0], traffic, ftmp)){
			fprintf(stderr, "[Warning] All s-t pair tried, initial path not found due to congested links.\n");
			break;
		}

		// Generate final state
		if(!genFinal(cycleRes[1], traffic, ftmp)){
			fprintf(stderr, "[Warning] All s-t pair tried, final path not found due to congested links.\n");
			break;
		}

		// Occupy Resource
		occupyRes(ftmp.pathFlow[0].hop[0], traffic, cycleRes[0]);
		occupyRes(ftmp.pathFlow[0].hop[1], traffic, cycleRes[1]);

		// Record flow
		flows.push_back(ftmp);

		// Record require/release list

		// Check cycle
	}

	// Result
	if(hasCycle) fprintf(stderr, "[Info] Cycle found.\n");
	else fprintf(stderr, "[Info] Cycle not found.\n");
}

// Generate initial state
bool GenInput::genInitial(const CycleRes& curRes, double traffic, Flow& curFlow){

	// Variable
	int srcID, dstID;
	bool found, isWireless;
	PathFlow ptmp;
	vector<int>randSrc;
	vector<int>randDst;

	// Random source
	found = false;
	genRandList(randSrc, numOfEdge);
	for(int i = 0; i < numOfEdge; i++){
		srcID = numOfCore + numOfAggr + randSrc[i];

		// Random destination
		genRandList(randDst, numOfEdge);
		for(int j = 0; j < numOfEdge; j++){
			dstID = numOfCore + numOfAggr + randDst[j];
			if(srcID != dstID){

				// Try to find out path
				isWireless = (rand()%2 == 1);
				if(findPath(ptmp.hop[0], traffic, isWireless, srcID, dstID, curRes)){

					// Mark as found
					found = true;
					break;
				}
			}
		}
		if(found) break;
	}

	// Path found
	if(found){

		// Record to flow list
		ptmp.traffic = traffic;
		curFlow.src = srcID;
		curFlow.pathFlow.clear();
		curFlow.pathFlow.push_back(ptmp);
	}
	return found;
}

// Generate final state
bool GenInput::genFinal(const CycleRes& curRes, double traffic, Flow& curFlow){

	// Variable
	int srcID, dstID, newID;
	bool found, isWireless;
	vector<int>randDst;

	// Initialize
	found = false;

	// Some flows remain the same as initial (must have enough final resource)
	if(rand()%2 && enoughRes(curFlow.pathFlow[0].hop[0], traffic, curRes)){
		found = true;
		curFlow.pathFlow[0].hop[1] = curFlow.pathFlow[0].hop[0];
	}

	// Some flows are required to pick new final paths
	else{
		srcID = curFlow.src;
		dstID = curFlow.pathFlow[0].hop[0][ curFlow.pathFlow[0].hop[0].size()-1 ].dstID;

		// Pick up a random destination other than original one
		genRandList(randDst, numOfEdge);
		for(int i = 0; i < numOfEdge; i++){
			newID = numOfCore + numOfAggr + randDst[i];
			if(newID != srcID && newID != dstID){

				// Try to find out path
				isWireless = (rand()%2 == 1);
				if(findPath(curFlow.pathFlow[0].hop[1], traffic, isWireless, srcID, newID, curRes)){
					found = true;
					break;
				}
			}
		}
	}
	return found;
}

// Find path from source to destination using traffic with wired/wireless path
bool GenInput::findPath(vector<Hop>& hopList, double traffic, bool isWireless, int srcID, int dstID, const CycleRes& curRes){

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
		copyLink = curRes.links;
		copyTranc = curRes.trancNode;

		// BFS
		found = false;
		Bnow.ID = srcID;
		Bnow.hopList.clear();
		Bnow.inter = curRes.interNode;
		vis.clear();
		vis[srcID] = true;
		que.push(Bnow);
		while(!found && !que.empty()){
			Bnow = que.front();
			que.pop();
			nowID = Bnow.ID;

			// Transceiver node
			if(copyTranc[ curRes.switches[nowID].trancID ].nodeCapacity < traffic) continue;
			copyTranc[ curRes.switches[nowID].trancID ].nodeCapacity -= traffic;

			// Neighbor
			for(int i = 0; i < (int)curRes.switches[nowID].port.size(); i++){
				nxtID = curRes.switches[nowID].port[i];
				linkID = linkMap[nowID][nxtID];
				if(!curRes.links[linkID].isWireless) continue;

				// Check feasibility
				if(!vis[nxtID]){

					// Transceiver node
					if(copyTranc[ curRes.switches[nxtID].trancID ].nodeCapacity < traffic) continue;

					// Interference node
					ok = true;
					Bnxt = Bnow;
					Bnxt.ID = nxtID;
					for(int j = 0; j < (int)curRes.links[linkID].iList.size(); j++){
						interID = curRes.links[linkID].iList[j];
						if(Bnxt.inter[ curRes.switches[interID].interID ].nodeCapacity < traffic){
							ok = false;
							break;
						}
						Bnxt.inter[ curRes.switches[interID].interID ].nodeCapacity -= traffic;
					}

					// OK
					if(ok){

						// Update transceiver node
						copyTranc[ curRes.switches[nxtID].trancID ].nodeCapacity -= traffic;
						
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
			if(curRes.links[linkID].linkCapacity >= traffic){
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
				if(curRes.links[linkID].linkCapacity >= traffic){
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
			if(curRes.links[linkID].linkCapacity < traffic) return false;
			htmp.srcID = coreID;
			htmp.dstID = aggrID;
			ansHop.push_back(htmp);
		}

		// Aggr -> Edge(destination)
		linkID = linkMap[aggrID][dstID];
		if(curRes.links[linkID].linkCapacity < traffic) return false;
		htmp.srcID = aggrID;
		htmp.dstID = dstID;
		ansHop.push_back(htmp);
	}

	// Copy back
	hopList = ansHop;
	return true;
}

// Check if resource is enough
bool GenInput::enoughRes(const vector<Hop>& hopList, double traffic, const CycleRes& curRes){

	// Variable
	int srcID, dstID, linkID;
	CycleRes tmpRes;

	// For each hop
	tmpRes = curRes;
	for(int i = 0; i < (int)hopList.size(); i++){

		// Wired link
		srcID = hopList[i].srcID;
		dstID = hopList[i].dstID;
		linkID = linkMap[srcID][dstID];
		if(tmpRes.links[linkID].linkCapacity < traffic) return false;
		tmpRes.links[linkID].linkCapacity -= traffic;

		// Wireless link
		if(tmpRes.links[linkID].isWireless){

			// Transceiver node
			if(tmpRes.trancNode[ tmpRes.switches[srcID].trancID ].nodeCapacity < traffic 
			|| tmpRes.trancNode[ tmpRes.switches[dstID].trancID ].nodeCapacity < traffic) return false;
			tmpRes.trancNode[ tmpRes.switches[srcID].trancID ].nodeCapacity -= traffic;
			tmpRes.trancNode[ tmpRes.switches[dstID].trancID ].nodeCapacity -= traffic;

			// Interference node
			for(int j = 0; j < (int)tmpRes.links[linkID].iList.size(); j++){
				srcID = tmpRes.links[linkID].iList[j];
				if(tmpRes.interNode[ tmpRes.switches[srcID].interID ].nodeCapacity < traffic) return false;
				tmpRes.interNode[ tmpRes.switches[srcID].interID ].nodeCapacity -= traffic;
			}
		}
	}
	return true;
}

// Occurpy resource
void GenInput::occupyRes(const vector<Hop>& hopList, double traffic, CycleRes& curRes){

	// Variable
	int srcID, dstID, linkID;

	// Update remaining capacity
	for(int i = 0; i < (int)hopList.size(); i++){
		srcID = hopList[i].srcID;
		dstID = hopList[i].dstID;
		linkID = linkMap[srcID][dstID];
		if(curRes.links[linkID].linkCapacity < traffic){
			fprintf(stderr, "[Error] No enough resource [%d] = %.2lf < %.2lf", linkID, curRes.links[linkID].linkCapacity, traffic);
			if(curRes.links[linkID].isWireless) fprintf(stderr, "(wireless link).\n");
			else fprintf(stderr, "(wired link).\n");
			exit(1);
		}
		curRes.links[linkID].linkCapacity -= traffic;

		// Wireless link
		if(curRes.links[linkID].isWireless){

			// Transceiver
			if(curRes.trancNode[ curRes.switches[srcID].trancID ].nodeCapacity < traffic 
			|| curRes.trancNode[ curRes.switches[dstID].trancID ].nodeCapacity < traffic){
				fprintf(stderr, "[Error] No enough resource (transceiver node).\n");
				exit(1);
			}
			curRes.trancNode[ curRes.switches[srcID].trancID ].nodeCapacity -= traffic;
			curRes.trancNode[ curRes.switches[dstID].trancID ].nodeCapacity -= traffic;

			// Interference
			for(int j = 0; j < (int)curRes.links[linkID].iList.size(); j++){
				srcID = curRes.links[linkID].iList[j];
				if(curRes.interNode[ curRes.switches[srcID].interID ].nodeCapacity < traffic){
					fprintf(stderr, "[Error] No enough resource (interference node).\n");
					exit(1);
				}
				curRes.interNode[ curRes.switches[srcID].interID ].nodeCapacity -= traffic;
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
