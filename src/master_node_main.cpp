#include <stdlib.h>

#include "Communication/Communicator.h"

int
main(int _argc, char** _argv) {

	int port;
	if(_argc == 2) 
		port = atoi(_argv[1]);
	else
		port = 8888;

	Communicator comm(port);
	comm.SetMaster(true);
	comm.Listen();
	return 0;
}
