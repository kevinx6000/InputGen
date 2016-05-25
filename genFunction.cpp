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

	// Initial value for TCAM and Link Cap.
	stmp.tcamUsage = TCAM_CAPACITY;
	ltmp.linkCapacity = LINK_CAPACITY;
	ntmp.nodeCapacity = LINK_CAPACITY;

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
			ltmp.srcID = src;
			ltmp.dstID = dst;
			links.push_back(ltmp);

			// Core -> Aggr
			switches[dst].port.push_back(src);
			switches[dst].linkID.push_back(links.size());
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
			ltmp.srcID = src;
			ltmp.dstID = dst;
			links.push_back(ltmp);

			// Edge -> Aggr
			switches[dst].port.push_back(src);
			switches[dst].linkID.push_back(links.size());
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
	srand((unsigned)time(NULL));
}

// Generate initial state
void GenInput::genInitial(void){
	
	// Randomly pick source and 
}

// Generate final state
void GenInput::genFinal(void){
	
}
