// Header
#include "header.h"
#include "gen.h"

// Initializer
void GenInput::initialize(int n, int k){

	// Flow
	this->numOfFlow = n;

	// Pod
	this->pod = k;

	// Number of switches
	this->numOfCore = k*k/4;
	this->numOfAggr = k*k/2;
	this->numOfEdge = k*k/2;

	// Time seed
	srand((unsigned)time(NULL));
}

// Generate initial state
void GenInput::genInitial(void){
	
}

// Generate final state
void GenInput::genFinal(void){
	
}