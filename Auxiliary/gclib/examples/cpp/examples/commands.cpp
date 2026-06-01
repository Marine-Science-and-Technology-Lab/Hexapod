/** @addtogroup cpp_examples
  * @{
  */

/*! \file commands.cpp
*
* Function calls for the Command Example Project.
*
*/
#include "examples.h"

#include <iostream> //std::cout

using namespace std;

GReturn commands(GCon g)
{
	char buf[G_SMALL_BUFFER]; //traffic buffer
	GSize read_bytes = 0; //bytes read in GCommand
	int value;
	double d_value;

	cout << "*****************************************************************************\n";
	cout << "**************************    GCmdT() example   *****************************\n";
	cout << "*****************************************************************************\n";
	cout << "GCmdT() will return a trimmed response of GCommand()\n";
	cout << "The command 'PR ?,?' will return the relative "
			"position of the A and B axes\n";
	e(GCommand(g, "PR ?,?", buf, G_SMALL_BUFFER, &read_bytes));
	cout << "<<PR ?,? with GCommand(): " << buf << ">>\n";
	e(GCmdT(g, "PR ?,?", buf, G_SMALL_BUFFER, NULL));
	cout << "<<PR ?,? with GCmdT(): " << buf << ">>\n";
	char* front; //this must not be a pointer on the heap, it will be modified.
	e(GCmdT(g, "MG TIME", buf, sizeof(buf), &front)); //Trim back and front.
	cout << "<<MG TIME with GCmdT() and front trimmed: " << front << ">>" << "\n\n";

	cout << "*****************************************************************************\n";
	cout << "**************************    GCmdI() example   *****************************\n";
	cout << "*****************************************************************************\n";
	cout << "GCmdI() will return the value of GCommand() parsed as an int\n";
	cout << "The command 'MG _LMS' will return the available "
			"space in the vector buffer of the S plane.\n";
	e(GCmdT(g, "MG _LMS", buf, G_SMALL_BUFFER, NULL));
	cout << "MG _LMS with GCmdT(): " << buf << "\n";
	e(GCmdI(g, "MG _LMS", &value));
	cout << "MG _LMS with GCmdI(): " << value << "\n\n";

	cout << "*****************************************************************************\n";
	cout << "**************************    GCmd() example   ******************************\n";
	cout << "*****************************************************************************\n";
	cout << "GCmd() will execute the given command but does not return a value.\n";
	cout << "GCmd is useful for basic operations such as beginning"
			" motion or setting speed\n";
	e(GCmd(g, "BG A"));
	e(GCmd(g, "SP 5000"));
	cout << "GCmd(g, \"BG A\");\n";
	cout << "GCmd(g, \"SP 5000\");\n\n";
	
	
	cout << "*****************************************************************************\n";
	cout << "**************************    GCmdD() example   ******************************\n";
	cout << "*****************************************************************************\n";
	cout << "GCmdD() will return the value of GCommand parsed as a double\n";
	cout << "The command 'MG @AN[1]' will return the value of Analog Input 1\n";
	e(GCmdD(g, "MG @AN[1]", &d_value));
	cout << "MG @AN[1] with GCmdD(): " << d_value << "\n\n";
	
	cout << "*****************************************************************************\n";
	cout << "************************    Galil Double Format   ***************************\n";
	cout << "*****************************************************************************\n";
	double d_val = 0.00235;
	sprintf(buf, "%.4f", d_val);
	cout << "Galil Controllers expect double values to be formatted to 4 "
			"decimal places\n";
	cout << "Unformatted double value: " << d_val << "\n";
	cout << "Formatted double value rounded to 4 decimal places: " << buf << "\n\n";

	cout << "*****************************************************************************\n";
	cout << "*******************    G_UTIL_ERROR_CONTEXT example   ***********************\n";
	cout << "*****************************************************************************\n";
	//To check any OS errors - call GUtility with G_UTIL_ERROR_CONTEXT
	GSize size = sizeof(buf);
	GUtility(g, G_UTIL_ERROR_CONTEXT, buf, &size);
	cout << "GUtility() with G_UTIL_ERROR_CONTEXT: " << buf << "\n";

	return	GALIL_EXAMPLE_OK;
}
/** @}*/