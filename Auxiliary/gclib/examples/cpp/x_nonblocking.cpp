/*! \file x_nonblocking.cpp
*
* Example usage of GMessage, GRecord, and GInterrupt for non-blocking operation.
*
*/

#include "x_examples.h"
#include <iomanip> //hex printing

int cur = 0;
char chars[] = { '|', '\\', '-','/'};
//simple progress indicator.
void progress()
{
	cout << chars[cur] << '\r';
	cout.flush();
	++cur %= 4;
}


//demonstrates non-blocking calls of GMessage(), GInterrupt(), and GRecord().
int x_nonblocking(GCon g)
{

	// *** we assume -s ALL was passed to Open. ***
	GReturn rc;
	char buf[1024]; //read buffer
	
	cout << "\n************************************************************************\n";
	cout << "Example GMessage non-blocking usage\n";
	cout << "************************************************************************\n";
	
	x_e(GProgramDownload(g, "WT2000;MG TIME;EN", 0)); //wait 2 seconds, then print message
	x_e(GCmd(g, "XQ"));

	x_e(GTimeout(g, 0)); //set timeout to zero for non-blocking read
	
	for (int i = 0;
		(rc = GMessage(g, buf, sizeof(buf))) == G_GCLIB_NON_BLOCKING_READ_EMPTY; 
		i++)
	{
		x_e(GCmd(g, "MGTIME")); //do something useful here...
		progress(); //and here
	}
	
	x_e(rc); //verify the return code
	x_e(GTimeout(g, -1)); //put the timeout back to the GOpen() setting
	cout << buf << '\n'; //print the message

	cout << "\n************************************************************************\n";
	cout << "Example GInterrupt non-blocking usage\n";
	cout << "************************************************************************\n";
	x_e(GProgramDownload(g, "WT2000;UI1;EN", 0)); //wait 2 seconds, then fire interrupt
	x_e(GCmd(g, "XQ")); //start code

	GStatus byte = 0; //variable for the interrupt byte
	x_e(GTimeout(g, 0)); //set timeout to zero for non-blocking read

	while ((rc = GInterrupt(g, &byte)) == G_GCLIB_NON_BLOCKING_READ_EMPTY)
	{
		x_e(GCmd(g, "MGTIME")); //do something useful here...
		progress(); //and here
	}
	x_e(rc); //verify the return code
	x_e(GTimeout(g, -1)); //put the timeout back to the GOpen() setting
	cout << " " << hex << uppercase << (int) byte << dec << '\n'; //print the byte in hex


	cout << "\n************************************************************************\n";
	cout << "Example GRecord non-blocking usage\n";
	cout << "************************************************************************\n";
	
	GDataRecord dr;
	x_e(GRecordRate(g, 0)); //turn off data record
	x_e(GTimeout(g, 0)); //set timeout to zero for non-blocking read
	GRecord(g, &dr, G_DR); //throw away a waiting record
	x_e(GTimeout(g, -1)); //put the timeout back to the GOpen() setting
	x_e(GRecordRate(g, 2000)); //turn on data record
	x_e(GTimeout(g, 0)); //set timeout to zero for non-blocking read
	while ((rc = GRecord(g, &dr, G_DR)) == G_GCLIB_NON_BLOCKING_READ_EMPTY)
	{
		x_e(GCmd(g, "MGTIME")); //do something useful here...
		progress(); //and here
	}
	x_e(rc); //verify the return code
	x_e(GTimeout(g, -1)); //put the timeout back to the GOpen() setting
	x_e(GRecordRate(g, 0)); //turn off data record
	cout << " " << dr.dmc4000.sample_number << '\n';

	return GALIL_EXAMPLE_OK;
}