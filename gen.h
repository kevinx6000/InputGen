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
		void output(void);			// Output flow

	/* Private Class */
	private:
		class Hop{
			public:
				int srcID;
				int dstID;
		};
		class PathFlow{
			public:
				int dst[2];
				double traffic;
				vector<Hop>hop[2];
		};
		class Flow{
			public:
				int src;
				vector<PathFlow>pathFlow;
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
		class SearchNode{
			public:
				int ID; 
				map<int, double>interCap;
		};

	/* Private Data */
	private:
		int pod;						// Number of pod in Fattree
		int numOfFlow;					// Number of flows
		int numOfCore;					// Number of core switches
		int numOfAggr;					// Number of aggregate switches
		int numOfEdge;					// Number of edge switches
		vector<Flow>flows;				// Flow plan to output
		vector<Switch>switches;			// Switch info
		vector<Link>links;				// Link info
		vector<NodeCap>trancNode;		// Transceiver node info
		vector<NodeCap>interNode;		// Interference node info
		vector< map<int, int> >linkMap;	// Map the index from (src,dst) to link resource ID

	/* Private Function */
	private:
		void clearResource(void);
		void occupyRes(const vector<Hop>&, double);
		bool findPath(int, int, vector<Hop>&, bool, double);
		double vecdot(double[2], double[2], double[2], double[2]);
		double vecdis(double[2], double[2], double[2], double[2]);
};

#endif
