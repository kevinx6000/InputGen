#include "header.h"
#include "gen.h"

// Main
int main(int argc, char *argv[])
{
	// Exception
	if(argc != 3){
//		fprintf(stderr, "Usage: ./main [pod] [# of flows]\n");
fprintf(stderr, "Fail\n");
		exit(1);
	}

	// Variables
	int numOfPod, numOfFlow;
	GenInput genInput;

	// Initialize
//	fprintf(stderr, "[Info] Initializing environment and variables...\n");
	numOfPod = atoi(argv[1]);
	numOfFlow = atoi(argv[2]);
	genInput.initialize(numOfPod, numOfFlow);
//	fprintf(stderr, "[Info] done.\n");

	// Generate initial traffic distribution
//	fprintf(stderr, "[Info] Generating intial traffic distribution...\n");
	genInput.genInitial();
//	fprintf(stderr, "[Info] done.\n");
	
	// Generate final traffic distribution
//	fprintf(stderr, "[Info] Generating final traffic distribution...\n");
	genInput.genFinal();
//	fprintf(stderr, "[Info] done.\n");

	// Output
//	fprintf(stderr, "[Info] Output resulting flow plans...\n");
	genInput.output();
//	fprintf(stderr, "[Info] done.\n");
fprintf(stderr, "Success\n");

	return 0;
}
