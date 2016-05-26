#include "header.h"
#include "gen.h"

// Main
int main(int argc, char *argv[])
{
	// Exception
	if(argc != 3){
		fprintf(stderr, "Usage: ./main [numOfFlow] [pod]\n");
		exit(1);
	}

	// Variables
	int numOfFlow;
	int numOfPod;
	GenInput genInput;

	// Initialize
	fprintf(stderr, "[Info] Initializing environment and variables...\n");
	numOfFlow = atoi(argv[1]);
	numOfPod = atoi(argv[2]);
	genInput.initialize(numOfFlow, numOfPod);
	fprintf(stderr, "[Info] done.\n");

	// Generate initial traffic distribution
	fprintf(stderr, "[Info] Generating intial traffic distribution...\n");
	genInput.genInitial();
	fprintf(stderr, "[Info] done.\n");

	// Generate initial traffic distribution
	fprintf(stderr, "[Info] Generating final traffic distribution...\n");
	genInput.genFinal();
	fprintf(stderr, "[Info] done.\n");

	// Output
	fprintf(stderr, "[Info] Output resulting flow plans...\n");
	genInput.output();
	fprintf(stderr, "[Info] done.\n");

	return 0;
}
