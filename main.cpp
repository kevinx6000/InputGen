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
	numOfFlow = atoi(argv[1]);
	numOfPod = atoi(argv[2]);
	genInput.initialize(numOfFlow, numOfPod);
	

	fprintf(stderr, "DEBUG flag\n");
	return 0;
}
