#include "header.h"
#include "gen.h"

// Main
int main(int argc, char *argv[])
{
	// Exception
	if(argc != 3){
//		fprintf(stderr, "Usage: ./main [pod] [Length of chain]\n");
fprintf(stderr, "Fail\n");
		exit(1);
	}

	// Variables
	int numOfPod, chainLength;
	GenInput genInput;

	// Initialize
//	fprintf(stderr, "[Info] Initializing environment and variables...\n");
	numOfPod = atoi(argv[1]);
	chainLength = atoi(argv[2]);
	genInput.initialize(numOfPod);
//	fprintf(stderr, "[Info] done.\n");

	// Generate initial traffic distribution
//	fprintf(stderr, "[Info] Generating intial traffic distribution...\n");
	// Chain Length = X -> Node = X+1
	genInput.genInitial(chainLength+1);
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
