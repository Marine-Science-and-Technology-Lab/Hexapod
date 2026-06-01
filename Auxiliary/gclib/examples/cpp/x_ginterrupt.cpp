/*! \file x_ginterrupt.cpp
*
* Example GInterrupt() usage.
*
*/

#include "x_examples.h"

int x_ginterrupt(GCon g)
{
	cout << "\n************************************************************************\n";
	cout << "Example GInterrupt() usage\n";
	cout << "************************************************************************\n";

	char buf[1024]; //traffic buffer
	GSize bytes_read;

	//-------------------------------------------------------------------------
	//simple check for a interrupt support
	bool ei_support = true;
	if (GCommand(g, "WH", buf, 1024, &bytes_read) == G_NO_ERROR)
	{
		ei_support = (strstr(buf, "IH") != 0); //for this example, assume Ethernet connections supports interrupts.
	}//else assume PCI supports

	if (!ei_support)
	{
		cout << "No support on this bus\n";
		return G_NO_ERROR;
	}
		


	//-------------------------------------------------------------------------
	//Flush interrupts
	x_e(GCmd(g, "EI0,0")); //turn off interrupts
	GStatus status;
	x_e(GTimeout(g, 0)); //zero timeout
	while ((GInterrupt(g, &status) == G_NO_ERROR) && status); //flush interrupts, status will be zero when queue is empty
	x_e(GTimeout(g, -1)); //restore timeout
	
	//-------------------------------------------------------------------------
	// User Interrupts, UI
	GTimeout(g, 1000); //set timeout to 1 sec
	x_e(GProgramDownload(g, "WT500\rUI8\rEN", 0));
	
	x_e(GCmd(g, "XQ")); //execute program
	
	x_e(GInterrupt(g, &status));
	GTimeout(g, -1); //restore timeout
		
	if ((status & 0xF0) == 0xF0) //UI are F0 - FF
	{
		cout << "\"UI " << (int)(status & 0x0F) << "\" executed.\n";
	}
	else
	{
		cout << "Unexpected interrupt, " << hex << (int)status << dec << '\n';
		return GALIL_EXAMPLE_ERROR;
	}


#if 0
	//-------------------------------------------------------------------------
	// Independent motion
	x_e(GCmd(g, "DP 0,0")); //define position zero on A and B
	x_e(GCmdT(g, "RP", buf, sizeof(buf), &trimmed));
	cout << "\nPosition: " << trimmed << '\n';
	x_e(GCmd(g, "SP 4000,4000")); //set up speed
	x_e(GCmd(g, "AC 1280000,1280000")); //acceleration
	x_e(GCmd(g, "DC 1280000,1280000")); //deceleration
	x_e(GCmd(g, "PR 8000,10000")); //Postion Relative. B will take longer to make its move.
	x_e(GCmd(g, "SH AB")); //Servo Here
	cout << "Beginning independent motion... ";
	x_e(GCmd(g, "BG AB")); //Begin motion
	x_e(x_ei_motioncomplete(g, "AB")); //Block until motion is complete on axes A and B
	cout << "Motion Complete on A and B\n";
	x_e(GCmdT(g, "RP", buf, sizeof(buf), &trimmed));
	cout << "Position: " << trimmed << '\n';;
#endif 

	return GALIL_EXAMPLE_OK;
}

int x_ei_motioncomplete(GCon g, GCStringIn axes) //Motion Complete with interrupts.
{
	char buf[1024]; //traffic buffer
	GReturn rc;
	GStatus status;
	unsigned char axis_mask = 0xFF; //bit mask of running axes, axes arg is trusted to provide running axes. Low bit indicates running.

	int len = strlen(axes);
	for (int i = 0; i < len; i++) //iterate through all chars in axes to make the axis mask
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
	sprintf(buf, "EI %u", (unsigned char) ~axis_mask); 
	x_e(GCmd(g, buf)); //send EI axis mask to set up interrupt events.

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