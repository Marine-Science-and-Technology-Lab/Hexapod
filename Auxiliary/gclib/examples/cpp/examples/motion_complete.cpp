/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file motion_complete.cpp
*
* Function calls for the Motion Complete Example Project.
*
*/
#include "examples.h"

#include <iostream> //std::cout
#include <string.h>
using namespace std;

int check_interrupts(GCon g, GCStringIn axes);

GReturn motion_complete(GCon g)
{
	char* trimmed;

	cout << "\n*************************************************************\n";
	cout << "Example GInterrupt() usage\n";
	cout << "***************************************************************\n";

	char buf[G_SMALL_BUFFER]; //traffic buffer
	GSize bytes_read;

	//-------------------------------------------------------------------------
	//simple check for appropriate communication bus
	//EI will fail below if interrupts are not supported at all
	bool ei_support = true;
	if (GCommand(g, "WH", buf, G_SMALL_BUFFER, &bytes_read) == G_NO_ERROR)
	{
		//for this example, assume Ethernet connections supports interrupts.
		ei_support = (strstr(buf, "IH") != 0); 
	}//else assume PCI supports

	if (!ei_support)
	{
		cout << "No support on this bus\n";
		return G_UNSUPPORTED_FUNCTION;
	}

	//-------------------------------------------------------------------------
	//Flush interrupts
	e(GCmd(g, "EI0,0")); //turn off interrupts
	GStatus status;
	e(GTimeout(g, 0)); //zero timeout

	//flush interrupts, status will be zero when queue is empty
	while ((GInterrupt(g, &status) == G_NO_ERROR) && status);
	e(GTimeout(g, -1)); //restore timeout


	//-------------------------------------------------------------------------
	// Independent motion
	e(GCmd(g, "DP 0,0")); //define position zero on A and B
	e(GCmdT(g, "RP", buf, G_SMALL_BUFFER, &trimmed));
	cout << "\nPosition: " << trimmed << '\n';
	e(GCmd(g, "SP 4000,4000")); //set up speed
	e(GCmd(g, "AC 1280000,1280000")); //acceleration
	e(GCmd(g, "DC 1280000,1280000")); //deceleration
	e(GCmd(g, "PR 8000,10000")); //Postion Relative. B will take longer to make its move.
	e(GCmd(g, "SH AB")); //Servo Here
	cout << "Beginning independent motion...\n";
	e(GCmd(g, "BG AB")); //Begin motion
	check_interrupts(g, "AB"); //Block until motion is complete on axes A and B
	cout << "Motion Complete on A and B\n";
	e(GCmdT(g, "RP", buf, G_SMALL_BUFFER, &trimmed));
	cout << "Position: " << trimmed << '\n';

	return	GALIL_EXAMPLE_OK;
}

//! Monitors interrupt status on the given axes and returns when interrupts are fired.
int check_interrupts(GCon g, GCStringIn axes)
{
	char buf[G_SMALL_BUFFER]; //traffic buffer
	GReturn rc;
	GStatus status;
	//bit mask of running axes, axes arg is trusted to provide running axes.
	//Low bit indicates running.
	unsigned char axis_mask = 0xFF; 

	int len = strlen(axes);
	//iterate through all chars in axes to make the axis mask
	for (int i = 0; i < len; i++) 
	{
		//support just A-H
		switch (axes[i])
		{
		case 'A':
			axis_mask &= 0xFE;
			break;
		case 'B':
			axis_mask &= 0xFD;
			break;
		case 'C':
			axis_mask &= 0xFB;
			break;
		case 'D':
			axis_mask &= 0xF7;
			break;
		case 'E':
			axis_mask &= 0xEF;
			break;
		case 'F':
			axis_mask &= 0xDF;
			break;
		case 'G':
			axis_mask &= 0xBF;
			break;
		case 'H':
			axis_mask &= 0x7F;
			break;
		}
	}
	sprintf(buf, "EI %u", (unsigned char)~axis_mask);
	e(GCmd(g, buf)); //send EI axis mask to set up interrupt events.

	while (axis_mask != 0xFF) //wait for all interrupts to come in
	{
		if ((rc = GInterrupt(g, &status)) == G_NO_ERROR)
		{
			switch (status)
			{
			case 0xD0: //Axis A complete
				axis_mask |= 0x01;
				break;
			case 0xD1: //Axis B complete
				axis_mask |= 0x02;
				break;
			case 0xD2: //Axis C complete
				axis_mask |= 0x04;
				break;
			case 0xD3: //Axis D complete
				axis_mask |= 0x08;
				break;
			case 0xD4: //Axis E complete
				axis_mask |= 0x10;
				break;
			case 0xD5: //Axis F complete
				axis_mask |= 0x20;
				break;
			case 0xD6: //Axis G complete
				axis_mask |= 0x40;
				break;
			case 0xD7: //Axis H complete
				axis_mask |= 0x80;
				break;
			}
		}
	}

	return rc;
}
/** @}*/