/*! \file x_gmotioncomplete.cpp
*
* Example GMotionComplete() usage.
*
*/

#include "x_examples.h"

int x_gmotioncomplete(GCon g)
{
	cout << "\n************************************************************************\n";
	cout << "Example GMotionComplete() usage\n";
	cout << "************************************************************************\n";
	
	char buf[1024]; //traffic buffer
	char* trimmed; //trimmed string pointer

	x_e(GCmd(g, "ST")); //stop all motion and programs

	//-------------------------------------------------------------------------
	// Independent motion
	x_e(GCmd(g, "DP 0")); //define A position zero
	x_e(GCmdT(g, "RP", buf, sizeof(buf), &trimmed));
	cout << "\nPosition: " << trimmed << '\n';
	x_e(GCmd(g, "SP 4000")); //set up speed
	x_e(GCmd(g, "AC 1280000")); //acceleration
	x_e(GCmd(g, "DC 1280000")); //deceleration
	x_e(GCmd(g, "PR 8000")); //Postion Relative.
	x_e(GCmd(g, "SH A")); //Servo Here
	cout << "Beginning independent motion... ";
	x_e(GCmd(g, "BG A")); //Begin motion
	x_e(GMotionComplete(g, "A")); //Block until motion is complete on axis A
	cout << "Motion Complete on A\n";
	x_e(GCmdT(g, "RP", buf, sizeof(buf), &trimmed));
	cout << "Position: " << trimmed << '\n';;

	//-------------------------------------------------------------------------
	// Vector motion
	x_e(GCmd(g, "DP 0")); //define position zero on A
	x_e(GCmdT(g, "RP", buf, sizeof(buf), &trimmed));
	cout << "\nPosition: " << trimmed << '\n';;
	x_e(GCmd(g, "VS 2000"));  //set up vector speed, S plane
	x_e(GCmd(g, "VA 100000")); //vector Acceleration
	x_e(GCmd(g, "VD 100000")); //vector deceleration
	x_e(GCmd(g, "VM AN")); //invoke vector mode, use virtual axis for 1-axis controllers
	x_e(GCmd(g, "VP 3000, 3000")); //buffer Vector Position
	x_e(GCmd(g, "VP 6000,0")); //buffer Vector Position
	x_e(GCmd(g, "VE")); //indicate Vector End
	cout << "Beginning vector motion... ";
	x_e(GCmd(g, "BG S")); //begin S plane
	x_e(GMotionComplete(g, "S")); //Block until motion is complete on vector plane S
	cout << "Motion Complete on vector plane S\n";
	x_e(GCmdT(g, "RP", buf, sizeof(buf), &trimmed));
	cout << "Position: " << trimmed << '\n';;

	return GALIL_EXAMPLE_OK;
}