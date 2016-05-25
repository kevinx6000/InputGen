#ifndef GEN_INPUT_H
#define GEN_INPUT_H

// Header
#include "header.h"

// Generator Class
class GenInput{

	public:
		void initialize(int, int);	// Initializer
		void genInitial(void);		// Generate initial state
		void genFinal(void);		// Generate final state

	private:
		int pod;					// Number of pod in Fattree
		int numOfFlow;				// Number of flows
		int numOfCore;				// Number of core switches
		int numOfAggr;				// Number of aggregate switches
		int numOfEdge;				// Number of edge switches
};

#endif
