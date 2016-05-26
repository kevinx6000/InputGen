// Header
#include "header.h"
#include "gen.h"

// Initializer
void GenInput::initialize(int n, int k){

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

	// Flow
	this->numOfFlow = n;

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
	srand((unsigned)time(NULL));
}

// Generate initial state
void GenInput::genInitial(void){

	// Variable
	bool ok, isWireless;
	Flow ftmp;
	PathFlow ptmp;

	// Clear resource
	clearResource();

	// For each flow
	for(int flowID = 0; flowID < numOfFlow; flowID++){

		// Randomly pick source
		ok = false;
		ftmp.src = rand()%numOfEdge + numOfCore + numOfAggr;
		ftmp.pathFlow.clear();
		while(!ok){
			ok = true;

			// Randomly pick destination
			while((ptmp.dst[0] = rand()%numOfEdge + numOfCore + numOfAggr) == ftmp.src);

			// Wired/wireless path
			if(rand()%2){
				isWireless = true;
				ptmp.traffic = rand()%(LINK_CAPACITY/2)+1;
			}
			else{
				isWireless = false;
				ptmp.traffic = rand()%LINK_CAPACITY+1;
			}

			// Shortest path
			if(findPath(ftmp.src, ptmp.dst[0], ptmp.hop[0], isWireless, ptmp.traffic)){
				
				// Update remaining capacity
				occupyRes(ptmp.hop[0], ptmp.traffic);

				// Record
				ftmp.pathFlow.push_back(ptmp);
			}

			// Not found
			else{
				ok = false;
				fprintf(stderr, "[Warning] Initial state not found, automatically retry.\n");
			}
		}

		// Record
		flows.push_back(ftmp);
	}
}

// Generate final state
void GenInput::genFinal(void){

	// Variable
	int srcID, dstID1, dstID2;
	bool isWireless;
	double traffic;
	map<int, bool>mtmp;
	vector< map<int, bool> >chosen;

	// Clear resource
	clearResource();

	// Initialize map
	mtmp.clear();
	for(int flowID = 0; flowID < (int)flows.size(); flowID++)
		chosen.push_back(mtmp);

	// First: pick some flow as fixed (no change)
	for(int flowID = 0; flowID < (int)flows.size(); flowID++){
		for(int pathID = 0; pathID < (int)flows[flowID].pathFlow.size(); pathID++){
			if(rand()%2) continue;
			chosen[flowID][pathID] = true;
			traffic = flows[flowID].pathFlow[pathID].traffic;
			flows[flowID].pathFlow[pathID].hop[1] = flows[flowID].pathFlow[pathID].hop[0];
			occupyRes(flows[flowID].pathFlow[pathID].hop[1], traffic);
		}
	}

	// Then: the ramaining ones are required to find new paths
	for(int flowID = 0; flowID < (int)flows.size(); flowID++){
		for(int pathID = 0; pathID < (int)flows[flowID].pathFlow.size(); pathID++){
			if(chosen[flowID][pathID]) continue;
			srcID = flows[flowID].src;
			dstID1 = flows[flowID].pathFlow[pathID].dst[0];
			traffic = flows[flowID].pathFlow[pathID].traffic;

			// Randomly pick a destination other than src and dst[0]
			while((dstID2 = rand()%numOfEdge + numOfCore + numOfAggr) == srcID || dstID2 == dstID1);
			flows[flowID].pathFlow[pathID].dst[1] = dstID2;
			
			// Wired/wirless path
			if(rand()%2) isWireless = true;
			else isWireless = false;

			// Shortest path
			if(findPath(srcID, dstID2, flows[flowID].pathFlow[pathID].hop[1], isWireless, traffic)){
				
				// Update remaining capacity
				occupyRes(flows[flowID].pathFlow[pathID].hop[1], traffic);
			}

			// Not found
			else{
				pathID--;
				fprintf(stderr, "[Warning] Final state not found, automatically retry.\n");
			}
		}
	}
}

// Find feasible path
bool GenInput::findPath(int src, int dst, vector<Hop>& hopList, bool isWireless, double traffic){

	// Variable
	int nowID, nxtID;
	int linkID;
	bool done, fail;
	Hop htmp;
	SearchNode bfsNow, bfsNxt;
	queue<SearchNode>que;
	map<int, int>pre;
	map<int, bool>vis;
	vector<NodeCap>copyTranc;

	// Initialize interferene node usage
	bfsNow.interCap.clear();
	for(int i = 0; i < (int)interNode.size(); i++)
		bfsNow.interCap[ interNode[i].ID ] = interNode[i].nodeCapacity;

	// Copy transceiver node usage
	copyTranc.clear();
	for(int i = 0; i < (int)trancNode.size(); i++)
		copyTranc.push_back(trancNode[i]);

	// BFS
	done = false;
	bfsNow.ID = src;
	que.push(bfsNow);
	pre[src] = src;
	vis[src] = true;
	while(!que.empty() && !done){
		bfsNow = que.front();
		nowID = bfsNow.ID;
		que.pop();

		// Transceiver node
		if(isWireless){
			if(copyTranc[ switches[nowID].trancID ].nodeCapacity < traffic) continue;
			copyTranc[ switches[nowID].trancID ].nodeCapacity -= traffic;
		}

		// Search neighbor
		for(int i = 0; i < (int)switches[nowID].port.size(); i++){
			nxtID = switches[nowID].port[i];
			bfsNxt = bfsNow;
			bfsNxt.ID = nxtID;
			linkID = switches[nowID].linkID[i];

			// Not the same type of link with the plan
			if((links[linkID].isWireless && !isWireless) || (!links[linkID].isWireless && isWireless)) continue;

			// Link capacity
			if(links[linkID].linkCapacity < traffic) continue;

			// Not visited
			if(!vis[nxtID]){

				// Wireless link
				if(isWireless){

					// Transceiver node
					if(copyTranc[ switches[nxtID].trancID ].nodeCapacity < traffic) continue;
					copyTranc[ switches[nxtID].trancID ].nodeCapacity -= traffic;

					// Interference node
					fail = false;
					for(int j = 0; j < (int)links[linkID].iList.size(); j++){
						if(bfsNxt.interCap[ links[linkID].iList[j] ] < traffic){
							fail = true;
							break;
						}
						else{
							// Update
							bfsNxt.interCap[ links[linkID].iList[j] ] -= traffic;
						}
					}
					if(fail) continue;
				}

				// Record and enqueue
				pre[nxtID] = nowID;
				vis[nxtID] = true;
				que.push(bfsNxt);
				if(nxtID == dst){
					done = true;
					break;
				}
			}
		}
	}

	// Found
	if(done){

		// Retrieve path
		nowID = dst;
		hopList.clear();
		while(pre[nowID] != nowID){
			htmp.srcID = pre[nowID];
			htmp.dstID = nowID;
			hopList.push_back(htmp);
			nowID = pre[nowID];
		}
	}

	// Return result
	return done;
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
	fprintf(stderr, "[Error] No enough resource ");
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

// Output flow
void GenInput::output(void){

	// Pod
	printf("%d\n", pod);

	// Number of flows
	printf("%d\n", numOfFlow);

	// For each flows
	for(int flowID = 0; flowID < (int)flows.size(); flowID++){
		// Ingress switch ID
		printf("%d\n", flows[flowID].src);

		// For each path flows
		printf("%d\n", (int)flows[flowID].pathFlow.size());
		for(int pathID = 0; pathID < (int)flows[flowID].pathFlow.size(); pathID++){

			// Traffic volume
			printf("%.1lf\n", flows[flowID].pathFlow[pathID].traffic);

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
