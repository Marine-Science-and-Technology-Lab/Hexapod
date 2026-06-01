/*! \file x_gread_gwrite.cpp
*
* Example GRead() and GWrite() usage.
*
*/

#include "x_examples.h"

int x_gread_gwrite(GCon g)
{
	cout << "\n************************************************************************\n";
	cout << "Example GRead() and GWrite() usage\n";
	cout << "************************************************************************\n";

	char buf[1024]; //traffic buffer
	GSize read_bytes = 0; //bytes read
	GSize total_bytes = 0; //total bytes read
	
	//-------------------------------------------------------------------------
	/* 
	*  Ad hoc program downlader
	*
	*  GProgramDownload() should be used for all DMC program downloads. DL is used here to demonstrate
	*  GRead()/GWrite() usage.
	*  
	*  GRead()/GWrite() are useful, for example, for a firmware NRE that provides a binary response, or for a command
	*  that does not follow the colon termination of typical Galil commands. DL is an example of such a command.
	*
	*/

	string program = "i=0\r#loop\ri=i+1\rWT10\rJP#loop,i<10,\rEN\r"; //simple program for download
	
	x_e(GCmd(g, "AB")); //Abort any running programs
	
	x_e(GWrite(g, "DL\r", 3)); //Write the DL command to the controller to access the program buffer. The controller does not respond to this command.
	//don't forget the carriage return when using GWrite() for commands.

	//GSleep(100); //some products need a delay to erase flash. 100ms is overkill, but sufficient.
		
	x_e(GWrite(g, program.c_str(), program.size())); //send the program to the program buffer
	x_e(GWrite(g, "\\", 1)); //send a backslash to exit the program buffer.

	
	/*
	*  Assuming the return format of DL is unknown, this demo reads until a read times out.
	*  A faster approach would be to read for a known terminating sequence that doesn't appear in the data.
	*  For example, standard Galil commands terminate data with a colon.
	*/
	x_e(GTimeout(g, 100)); //adjust timeout
	GReturn rc = G_NO_ERROR; //return code
	while (rc == G_NO_ERROR) //read until timeout
	{
		total_bytes += read_bytes;
		rc = GRead(g, buf, sizeof(buf), &read_bytes);
	}

	x_e(GTimeout(g, G_USE_INITIAL_TIMEOUT)); //restore timeout
	cout << "\nRead " << total_bytes << " byte(s)\n";
	cout.write(buf, total_bytes); //print the received data
	cout << "\n";

	//now test the downloaded program
	x_e(GCmd(g, "i=0")); //force i to zero
	x_e(GCmd(g, "XQ")); //execute the program
	GSleep(200); //loop should take about 100ms

	int i;
	x_e(GCmdI(g, "i=?", &i));// pull the controller's value of i into an int
	if (i == 10)
	{
		cout << "Program test OK.\n";
		return GALIL_EXAMPLE_OK;
	}
	else
	{
		cout << "Program test failed. " << i << "\n";
		return GALIL_EXAMPLE_ERROR;
	}



	
}