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
	srand((unsigned)clock());
}

// Generate initial state
void GenInput::genInitial(void){

	// Variable
	int podID;
	int aggrID[2], coreID[2];
	Flow ftmp;
	PathFlow ptmp;

	// Clear resource
	clearResource();

	// Randomly pick one pod
	podID = rand()%pod;

	// Randomly pick one links (aggr->core) as cycling resource
	aggrID[0] = numOfCore + podID*(pod/2) + rand()%(pod/2);
	coreID[0] = (pod/2)*( (aggrID[0]-numOfCore)%(pod/2) ) + rand()%(pod/2);
	cycleRes[0].rID = linkMap[aggrID[0]][coreID[0]];
	cycleRes[0].maxRate = 0.0;
	
	// Randomly picl another link (aggr->core) as cycleing resource
	while((aggrID[1] = numOfCore + podID*(pod/2) + rand()%(pod/2)) == aggrID[0]);
	coreID[1] = (pod/2)*( (aggrID[1]-numOfCore)%(pod/2) ) + rand()%(pod/2);
	cycleRes[1].rID = linkMap[aggrID[1]][coreID[1]];
	cycleRes[1].maxRate = 0.0;

	// Try to find cycle effect through wired links
	while(true){

		// Check if done
		if(links[linkMap[ aggrID[0] ][ coreID[0] ]].linkCapacity < cycleRes[1].maxRate
		&& links[linkMap[ aggrID[1] ][ coreID[1] ]].linkCapacity < cycleRes[0].maxRate){
			fprintf(stderr, "[Info] Enter extreme phase check...\n");
			if(links[linkMap[ aggrID[0] ][ coreID[0] ]].linkCapacity < cycleRes[1].maxRate * 0.05
			&& links[linkMap[ aggrID[1] ][ coreID[1] ]].linkCapacity < cycleRes[0].maxRate * 0.05)
				break;
		}

		// Add one flow to each cycle resource (if possible)
		for(int i = 0; i < 2; i++){

			// Skip the one already fit
//			if(links[linkMap[ aggrID[i] ][ coreID[i] ]].linkCapacity < cycleRes[(i+1)%2].maxRate) continue;

			// Traffic data rate
			ptmp.traffic = genTraffic();

			// Pass through aggr->core
			if(links[linkMap[ aggrID[i] ][ coreID[i] ]].linkCapacity - ptmp.traffic + myMax(ptmp.traffic, cycleRes[i].maxRate) >= cycleRes[(i+1)%2].maxRate
			&& links[linkMap[ aggrID[(i+1)%2] ][ coreID[(i+1)%2] ]].linkCapacity + cycleRes[(i+1)%2].maxRate >= ptmp.traffic
			&& findWiredPath(ptmp.hop[0], ptmp.traffic, podID, coreID[i], aggrID[i])){
				fprintf(stderr, "[Info] Compete flow path OK.\n");
				
				// Occupy the resource along the path
				occupyRes(ptmp.hop[0], ptmp.traffic);

				// Record and update flow information
				cycleRes[i].flowID.push_back(flows.size());
				cycleRes[i].pathID.push_back(0);
				if(cycleRes[i].maxRate < ptmp.traffic){
					cycleRes[i].maxFlowID = flows.size();
					cycleRes[i].maxPathID = 0;
					cycleRes[i].maxRate = ptmp.traffic;
				}

				// Record to flow set
				ftmp.pathFlow.clear();
				ftmp.src = ptmp.hop[0][0].srcID;
				ftmp.pathFlow.push_back(ptmp);
				flows.push_back(ftmp);
			}

			// Path not found
			else{
				fprintf(stderr, "[Info] Compete flow path not fit, put to another path.\n");

				// Randomly pick another wired(?) paths,
				// which does not pass through current pod
				int retry = 0;
				while(!findAnotherPath(ptmp.hop[0], ptmp.traffic, podID)){
					fprintf(stderr, "[Info] Another path not found, retry... (flow = %d)\n", (int)flows.size());
					retry ++;
					if(retry > 10){
						fprintf(stderr, "[Error] Solution not found, GG.\n");
						exit(1);
					}
				}

				// Occupy the resource along the path
				occupyRes(ptmp.hop[0], ptmp.traffic);
				
				// Record to flow set
				ftmp.pathFlow.clear();
				ftmp.src = ptmp.hop[0][0].srcID;
				ftmp.pathFlow.push_back(ptmp);
				flows.push_back(ftmp);
			}
		}
	}

	// Debug: summary
	for(int i = 0; i < 2; i++)
		fprintf(stderr, "[Info] Cycle resource %d: %d/%d = %.2lf, rem = %.2lf\n", i, cycleRes[i].maxFlowID, cycleRes[i].maxPathID, cycleRes[i].maxRate, links[cycleRes[i].rID].linkCapacity);
}

// Generate final state
void GenInput::genFinal(void){

	// Variables
	int aggrID, coreID, edgeID, podID;
	double traffic;

	// Clear resource
	clearResource();

	// Assume flows without competition remain the same
	for(int i = 0; i < (int)flows.size(); i++){

		// Skip competition flow
		if(i == cycleRes[0].maxFlowID || i == cycleRes[1].maxFlowID) continue;

		// Currently, all flows only have one path flow
		flows[i].pathFlow[0].hop[1] = flows[i].pathFlow[0].hop[0];

		// Occupy it!
		occupyRes(flows[i].pathFlow[0].hop[1], flows[i].pathFlow[0].traffic);
	}

	// DEADLOCK, need to fix
	for(int i = 0; i < 2; i++){
		if(links[cycleRes[i].rID].linkCapacity + cycleRes[i].maxRate < cycleRes[(i+1)%2].maxRate){
			fprintf(stderr, "[Error] Sorry, such plan exists deadlock.\n");
			exit(1);
		}
	}

	// Two picked flow: choose compete resource of each other's initial path
	for(int i = 0; i < 2; i++){
		traffic = flows[ cycleRes[i].maxFlowID ].pathFlow[0].traffic;
		edgeID = flows[ cycleRes[i].maxFlowID ].src;
		aggrID = flows[ cycleRes[(i+1)%2].maxFlowID ].pathFlow[0].hop[0][1].srcID;
		coreID = flows[ cycleRes[(i+1)%2].maxFlowID ].pathFlow[0].hop[0][1].dstID;
		podID = (aggrID - numOfCore)/(pod/2);
		if(!findWiredPath(flows[ cycleRes[i].maxFlowID ].pathFlow[0].hop[1], traffic, podID, coreID, aggrID, edgeID)){
			fprintf(stderr, "[Error] GG, cannot find such a path to gen cycle.\n");
			exit(1);
		}

		// Occupy it!
		occupyRes(flows[ cycleRes[i].maxFlowID ].pathFlow[0].hop[1], traffic);
	}

	// Debug: output two paths of two competition flows
	for(int i = 0; i < 2; i++){
		int flowID = cycleRes[i].maxFlowID;
		fprintf(stderr, "[Info] Flow %d:\n", flowID);
		for(int s = 0; s < 2; s++){
			fprintf(stderr, "[Info]\tPath %d:", s);
			for(int h = 0; h < (int)flows[flowID].pathFlow[0].hop[s].size(); h++)
				fprintf(stderr, " %d-%d",
						flows[flowID].pathFlow[0].hop[s][h].srcID,
						flows[flowID].pathFlow[0].hop[s][h].dstID);
			fprintf(stderr, "\n");
		}
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
		fprintf(stderr, "[Info] Edge -> Aggr failed. (all full QQ)\n");
		for(i = 0; i < pod/2; i++){
			edgeID = numOfCore + numOfAggr + podID*(pod/2) + randList[i];
			linkID = linkMap[edgeID][aggrID];
			fprintf(stderr, "[Info] Edge %d -> Aggr %d = %.2lf (need %.2lf)\n", edgeID, aggrID, links[linkID].linkCapacity, traffic);
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
		fprintf(stderr, "[Info] Aggr -> Core failed.\n");
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
		fprintf(stderr, "[Info] Pod picking failed.\n");
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
		fprintf(stderr, "[Info] Core -> Aggr failed.\n");
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
		fprintf(stderr, "[Info] Aggr -> Edge failed.\n");
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
	fprintf(stderr, "[Info] All path for pod = %d is full\n", podID2);
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
