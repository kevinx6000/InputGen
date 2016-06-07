#ifndef GEN_INPUT_H
#define GEN_INPUT_H

// Header
#include "header.h"

// Generator Class
class GenInput{

	/* Public Functions */
	public:
		void initialize(int);		// Initializer
		void genInput(void);		// Generate input
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
		class BFSNode{
			public:
				int ID;
				vector<Hop>hopList;
				vector<NodeCap>inter;
		};
		class CycleRes{
			public:
				vector<Switch>switches;
				vector<Link>links;
				vector<NodeCap>trancNode;
				vector<NodeCap>interNode;
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
		bool genInitial(const CycleRes&, double, Flow&);		// Generate initial state
		bool genFinal(const CycleRes&, double, Flow&);		// Generate final state
		bool findPath(vector<Hop>&, double, bool, int, int, const CycleRes&);	// Not this destination
		bool enoughRes(const vector<Hop>&, double, const CycleRes&);
		void occupyRes(const vector<Hop>&, double, CycleRes&);
		void clearResource(void);
		void genRandList(vector<int>&, int);
		double genTraffic(void);
		double vecdot(double[2], double[2], double[2], double[2]);
		double vecdis(double[2], double[2], double[2], double[2]);
		bool testPort(int, int);
};

#endif
