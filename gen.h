#ifndef GEN_INPUT_H
#define GEN_INPUT_H

// Header
#include "header.h"

// Generator Class
class GenInput{

	/* Public Functions */
	public:
		void initialize(int, int);	// Initializer
		void genInitial(void);		// Generate initial state
		void genFinal(void);		// Generate final state

	/* Private Class */
	private:
		class PathFlow{
			public:
				int dst[2];
				double traffic;
				vector<int>hop[2];
		};
		class Flow{
			public:
				int src;
				vector<PathFlow>flowPath;
		};
		class Switch{
			public:
				int ID;
				bool tcamUsage;
				double posXY[2];
				vector<int>port;
				vector<int>linkID;
				int trancID;
				int interID;
		};
		class Link{
			public:
				int srcID;
				int dstID;
				bool isWireless;
				double linkCapacity;
				vector<int>iList;
		};
		class NodeCap{
			public:
				int ID;
				double nodeCapacity;
		};

	/* Private Data */
	private:
		int pod;					// Number of pod in Fattree
		int numOfFlow;				// Number of flows
		int numOfCore;				// Number of core switches
		int numOfAggr;				// Number of aggregate switches
		int numOfEdge;				// Number of edge switches
		vector<Switch>switches;		// Switch info
		vector<Link>links;			// Link info
		vector<NodeCap>trancNode;	// Transceiver node info
		vector<NodeCap>interNode;	// Interference node info

	/* Private Function */
	private:
		double vecdot(double[2], double[2], double[2], double[2]);
		double vecdis(double[2], double[2], double[2], double[2]);
};

#endif
