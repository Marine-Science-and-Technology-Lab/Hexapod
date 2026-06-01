/*! \file x_simple.c
*
* A very simple example for using gclib. See x_examples.cpp for more in-depth examples.
*
*/
#include <stdio.h> //printf()
#include <stdlib.h> //exit()

#include "gclibo.h" //by including the open-source header, all other headers are pulled in.

GCon g = 0; //var used to refer to a unique connection

//check return code from most gclib calls
void check(GReturn rc)
{
	if (rc != G_NO_ERROR)
	{
		printf("ERROR: %d", rc);
		if (g)
			GClose(g);
		exit (rc);
	}
}

int main()
{
	char buf[1024]; //traffic buffer

	check(GVersion(buf, sizeof(buf)));
	printf("version: %s\n", buf); //Print the library version

	check(GOpen("192.168.0.43", &g)); //Open a connection to Galil, store the identifier in g.
	
	check(GInfo(g, buf, sizeof(buf)));
	printf("info: %s\n", buf); //Print the connection info
	
	check(GCommand(g, "MG TIME", buf, sizeof(buf), 0)); //Send MG TIME. Because response is ASCII, don't care about bytes read.
	printf("response: %s\n", buf); //Print the response
	
	if (g) //don't call close on a nullptr
		GClose(g); //Don't forget to close!
	
	return G_NO_ERROR;
}
