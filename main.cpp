#include "header.h"
#include "gen.h"

// Main
int main(int argc, char *argv[])
{
	// Variables
	GenInput genInput;

	// Read existing flows
	fprintf(stderr, "[Info] Read existing flows...\n");
	genInput.readFlow();
	fprintf(stderr, "[Info] done.\n");

	// Initialize
	fprintf(stderr, "[Info] Initializing environment and variables...\n");
	genInput.initialize();
	fprintf(stderr, "[Info] done.\n");

	// Generate initial traffic distribution
	fprintf(stderr, "[Info] Generating intial traffic distribution...\n");
	genInput.genInitial();
	fprintf(stderr, "[Info] done.\n");
	
	// Generate final traffic distribution
	fprintf(stderr, "[Info] Generating final traffic distribution...\n");
	genInput.genFinal();
	fprintf(stderr, "[Info] done.\n");

	// Output
	fprintf(stderr, "[Info] Output resulting flow plans...\n");
	genInput.output();
	fprintf(stderr, "[Info] done.\n");

	return 0;
}
