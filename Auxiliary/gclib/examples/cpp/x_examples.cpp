/*! \file x_examples.cpp
*
* Examples main(). Calls example code.
*
*/
#include "x_examples.h"

#include <iomanip>

int main(int argc, char * argv[])
{
	int rc = GALIL_EXAMPLE_OK; //return code
	int buf_size = G_SMALL_BUFFER;
	char buf[G_SMALL_BUFFER]; //traffic buffer
	
	GCon g = 0; //var used to refer to a unique connection. A valid connection is nonzero.
	try
	{
	
		x_e(GVersion(buf, sizeof(buf))); //library version
		cout << "Library versions: " << buf << "\n";

#if 0
		/*
		*  Change above line to "#if 1" to run examples below.
		*  Listening to requests for IP addresses.
		*/
		x_e(GIpRequests(buf, sizeof(buf))); //listen for ~5 seconds for controllers requesting IP addresses
		cout << "Controllers without IP Address:\n";
		if (strlen(buf) != 0)
			cout << buf << "\n\n";
		else
			cout << "none\n\n";

#endif

#if 0
		/*
		*  Change above line to "#if 1" to run examples below.
		*  Assign IP addresses.
		*/
		x_e(GAssign("192.168.42.100", "00:50:4C:20:52:90")); //assign an ip address to a known MAC
		//NOTE: GAssign does not burn the IP address with BN. This can be done after assignment through GOpen() and GCmd().
#endif

#if 0
		/*
		*  Change above line to "#if 1" to run examples below.
		*  Listing available hardware addresses.
		*/
		x_e(GAddresses(buf, sizeof(buf))); //list available addresses
		cout << "Available addresses:\n";
		if (strlen(buf) != 0)
			cout << buf << "\n\n";
		else
			cout << "none\n\n";
#endif

		cout << "Connecting to hardware\n";

		//Basic connections
		x_e(GOpen("192.168.42.100 --subscribe ALL", &g)); //connect and assign a value to g. 
		//x_e(GOpen("/dev/galilpci0 --subscribe ALL", &g)); 
		//x_e(GOpen("COM1 --baud 115200 --subscribe ALL", &g));
		
		x_e(GInfo(g, buf, sizeof(buf))); //grab connection string
		cout << buf << '\n';

		//x_e(GCmd(g, "BN")); //example to burn the IP address if it was set above

#if 0
		/*
		*  Change above line to "#if 1" to run examples below.
		*  Comment out the function calls below to be avoided.
		*  Note some calls attempt to move motors and not all
		*  functions are compatible with all Galil products.
		*/

		
		x_e(x_gread_gwrite(g)); //call examples for GRead() and GWrite().		
		x_e(x_gcommand(g)); //call examples for GCommand().
		x_e(x_programs(g)); //call examples for GProgramDownload() and GProgramUpload().
		x_e(x_arrays(g)); //call examples for GArrayDownload() and GArrayUpload().
		x_e(x_grecord(g)); //call examples for GRecord(). WARNING, this call will attempt to move motors.
		x_e(x_gmessage(g)); //call examples for GMessage().
		x_e(x_ginterrupt(g)); //call examples for GInterrupt().  WARNING, this call will attempt to move motors.
		x_e(x_gmotioncomplete(g)); //call examples for GMotionComplete. WARNING, this call will attempt to move motors.
		x_e(x_nonblocking(g)); //call examples for using GRecord(), GMessage(), and GInterrupt() in a non-blocking mode.
		
		
#endif	

#if 0
		/*
		*  Change above line to "#if 1" to run examples below.
		*  Loading Firmware
		*/
		x_e(GFirmwareDownload(g, "c:/temp/d212r10r2.hex"));
		x_e(GInfo(g, buf, sizeof(buf)));
		cout << buf << '\n';
#endif

#if 0
		/*
		*  Change above line to "#if 1" to run examples below.
		*  Calling GSetupDownloadFile
		*/
		GOption opt = 0;
		const char* file_path = "C:/dev/test/gcb/test.gcb";
		opt = GSetupDownloadFile(g, file_path, opt, buf, buf_size);
		cout << "Setup file " << file_path << endl;
		cout << "Parameters " << (opt & 0x02 ? "present" : "absent") << endl;
		cout << "Variables " << (opt & 0x08 ? "present" : "absent") << endl;
		cout << "Arrays " << (opt & 0x10 ? "present" : "absent") << endl;
		cout << "Program " << (opt & 0x20 ? "present" : "absent") << endl;
		//cout << buf; //print the setup info

		x_e(GSetupDownloadFile(g, "C:/dev/test/gcb/test.gcb", 0xff, 0, 0));
#endif

		if (g) x_e(GClose(g)); g = 0; //close g
		

	}//try
	catch (GReturn gr) //for x_e() function
	{
		if (gr == GALIL_EXAMPLE_ERROR)
			cout << "ERROR: Example code failed\n";
		else
		{
			cout << "Function returned " << gr << '\n';
			GError(gr, buf, sizeof(buf));
			cout << buf << '\n';
			GSize size = sizeof(buf);
			
			if (g)
			{
				GUtility(g, G_UTIL_ERROR_CONTEXT, buf, &size);
				cout << buf << '\n'; //further context
			}
			
		}
		rc = GALIL_EXAMPLE_ERROR;
		if (g) GClose(g); g = 0; //close g
	}
	catch (std::exception& e)
	{
		std::cerr << "Unexpected std::exception... Kaboom. " << e.what() << std::endl;
		rc = GALIL_EXAMPLE_ERROR;
		if (g) GClose(g); g = 0; //close g
	}
	catch (...)
	{
		cout << "Unexpected error... Kaboom." << endl;
		rc = GALIL_EXAMPLE_ERROR;
		if (g) GClose(g); g = 0; //close g
	}

	if (argc == 1) //if no args on command line, report and pause
	{
		cout << endl << endl;
		if (rc == GALIL_EXAMPLE_OK)
			cout << "examples.cpp executed OK\n";
		else
			cout << "examples.cpp returning error " << rc << '\n';

		cout << "main() is finished. Press Enter to exit:";
		getchar(); //keep window open
	}
	return rc;
}
