/*! \file x_gcommand.cpp
*
* Example GCommand() usage.
*
*/

#include "x_examples.h"

int x_gcommand(GCon g)
{
	cout << "\n************************************************************************\n";
	cout << "Example GCommand() usage\n";
	cout << "************************************************************************\n";

	char buf[1024]; //traffic buffer
	GSize read_bytes = 0; //bytes read in GCommand

	//-------------------------------------------------------------------------
	cout << "Revision report, ^R^V\n";
	x_e(GCommand(g, "\x12\x16", buf, sizeof(buf), &read_bytes));
	cout << buf << "\n\n";

	//-------------------------------------------------------------------------
	//Simple Command, no response accessible.
	x_e(GCmd(g, "ST"));
	
	//-------------------------------------------------------------------------
	cout << "Command Values\n";
	x_e(GCmd(g, "val=10")); //set initial value to an integer
	int vali = 0; //integer to be used
	x_e(GCmdI(g, "val=?", &vali)); //stuff value into integer
	cout << "val is " << vali << '\n';
	vali++;
	sprintf(buf, "val=%d", vali);
	x_e(GCmd(g, buf));
	x_e(GCmdI(g, "val=?", &vali));
	cout << "val is " << vali << '\n';

	x_e(GCmd(g, "val=3.1415")); //set initial value to a decimal
	double vald = 0; //double to be used
	x_e(GCmdD(g, "val=?", &vald));
	cout << "val is " << vald << '\n';
	vald *= vald; //square
	sprintf(buf, "val=%f", vald);
	x_e(GCmd(g, buf));
	x_e(GCmdD(g, "val=?", &vald));
	cout << "val is " << vald << '\n';
	
	//-------------------------------------------------------------------------
	cout << "\nCommand Trimming\n";
	x_e(GCommand(g, "MGTIME", buf, sizeof(buf), &read_bytes)); //standard command call.
	cout << ">" << buf << "<" << '\n';

	x_e(GCmdT(g, "MG TIME", buf, sizeof(buf), 0)); //Trim back.
	cout << ">" << buf << "<" << '\n';

	char* front; //this must not be a pointer on the heap, it will be modified.
	x_e(GCmdT(g, "MG TIME", buf, sizeof(buf), &front)); //Trim back and front.
	cout << ">" << front << "<" << '\n';

	//-------------------------------------------------------------------------
	cout << "\nReceiving Binary Data\n";
	x_e(GCommand(g, "QR", buf, sizeof(buf), &read_bytes)); //QR Response is binary
	cout << "QR read " << read_bytes << " bytes\n"; //Normally use GRecord() for QR.

	//-------------------------------------------------------------------------
	cout << "\nError handling\n";
	GReturn rc = G_NO_ERROR;
	if ((rc = GCommand(g, "QDA[]", buf, sizeof(buf), &read_bytes)) == G_COMMAND_CALLED_WITH_ILLEGAL_COMMAND)
		cout << "QD correctly trapped, not allowed, try GArrayDownload()\n";
	else
	{
		cout << "Unexpected QD behaviour\n";
		return GALIL_EXAMPLE_ERROR;
	}

	if ((rc = GCommand(g, "DL\rvar=3.14\rEN\r\\", buf, sizeof(buf), &read_bytes)) == G_COMMAND_CALLED_WITH_ILLEGAL_COMMAND)
		cout << "DL correctly trapped, not allowed, try GProgramDownload()\n";
	else
	{
		cout << "Unexpected DL behaviour\n";
		return GALIL_EXAMPLE_ERROR;
	}

	//-------------------------------------------------------------------------
	cout << "\nModifying timeout\n";
	x_e(GTimeout(g, 10000)); //increase timeout for BP. 10 seconds is excessive, but sufficient for all products.
	cout << "Burning program...";
	x_e(GCommand(g, "BP", buf, sizeof(buf), &read_bytes));
	x_e(GTimeout(g, G_USE_INITIAL_TIMEOUT)); //restore timeout
	cout << "OK\n";

	return GALIL_EXAMPLE_OK;
}