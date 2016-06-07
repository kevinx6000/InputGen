#include "header.h"
#include "gen.h"

// Main
int main(int argc, char *argv[])
{
	// Exception
	if(argc != 2){
		fprintf(stderr, "Usage: ./main [pod]\n");
		exit(1);
	}

	// Variables
	int numOfPod;
	GenInput genInput;

	// Initialize
	fprintf(stderr, "[Info] Initializing environment and variables...\n");
	numOfPod = atoi(argv[1]);
	genInput.initialize(numOfPod);
	fprintf(stderr, "[Info] done.\n");
	
	// Generate input
	genInput.genInput();

	// Output
	fprintf(stderr, "[Info] Output resulting flow plans...\n");
	genInput.output();
	fprintf(stderr, "[Info] done.\n");

	return 0;
}
