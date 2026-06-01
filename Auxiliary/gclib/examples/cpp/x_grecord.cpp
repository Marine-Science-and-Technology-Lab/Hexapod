
/*! \file x_grecord.cpp
*
* Example GRecord() usage.
*
*/

#include "x_examples.h"

int x_grecord(GCon g)
{
	cout << "\n************************************************************************\n";
	cout << "Example GRecord() usage\n";
	cout << "************************************************************************\n";

	char buf[1024]; //traffic buffer
	GSize bytes_read;

	//-------------------------------------------------------------------------
	//Grab a data record via QR and pull out some values.
	cout << "\nQR-based data record\n";
	GDataRecord r; //user's data record union.
	x_e(GRecord(g, &r, G_QR)); //poll for data record.
	cout << r.dmc4000.sample_number << '\n'; //sample number
	cout << r.dmc4000.axis_a_reference_position << "\n"; //Axis A's reference, "RPA"

	//-------------------------------------------------------------------------
	//simple check for a serial connection.
	bool dr_support = true;
	if (GCommand(g, "WH", buf, 1024, &bytes_read) == G_NO_ERROR)
	{
		dr_support = (strstr(buf, "IH") != 0); //for this example, assume Ethernet connections supports DR.
	}//else assume PCI supports, note 18x2 doesn't


	if (dr_support)
	{
		//-------------------------------------------------------------------------
		//Read data records asynchronously for a given interval.
		//note -s DR must have been specified in GOpen()
		cout << "\nDR-based data record\n";
		unsigned short time = 0; //controller's sample time, 1 tick a ms with TM 1000.
		unsigned short deadline = 1000; //listen time, in ms, at TM 1000
		int original_dr;
		x_e(GCmdI(g, "MG_DR", &original_dr)); //grab the current DR value
		x_e(GRecordRate(g, 100)); //set up DR to 10Hz
		for (size_t i = 0; deadline > time; i++)
		{
			x_e(GRecord(g, &r, G_DR)); //get record
			time = r.dmc4000.sample_number; //pull out the desired value
			//time = r.dmc1806.sample_number; //note, these are product-specific
			cout << time << '\n'; //print time
			if (!i) deadline = time + deadline; //sample on first iteration to set deadline
		}

		sprintf(buf, "DR %d", original_dr);
		x_e(GCmd(g, buf)); //restore DR
	}

	//-------------------------------------------------------------------------
	//Use pointer arithmetic to pull out arbitrary types and offsets.
	cout << "\nQR-based data record with offsets\n";
	x_e(GRecord(g, &r, G_QR)); //poll for data record.
	cout << r.dmc4000.sample_number << '\n';
	cout << *((unsigned short*)(r.byte_array + 4)) << '\n'; //equivalent to sample_number for DMC-4000
	

#if 0
		char* trimmed; //trimmed string pointer
		//-------------------------------------------------------------------------
		// Independent motion
		x_e(GCmd(g, "DP 0")); //define position zero on A
		x_e(GCmdT(g, "RP", buf, sizeof(buf), &trimmed));
		cout << "\nPosition: " << trimmed << '\n';
		x_e(GCmd(g, "SP 4000")); //set up speed
		x_e(GCmd(g, "AC 1280000")); //acceleration
		x_e(GCmd(g, "DC 1280000")); //deceleration
		x_e(GCmd(g, "PR 8000")); //Postion Relative. B will take longer to make its move.
		x_e(GCmd(g, "SH A")); //Servo Here
		cout << "Beginning independent motion... ";
		x_e(GCmd(g, "BG A")); //Begin motion
		x_e(x_dr_motioncomplete(g, "A")); //Block until motion is complete on axes A and B
		cout << "Motion Complete on A\n";
		x_e(GCmdT(g, "RP", buf, sizeof(buf), &trimmed));
		cout << "Position: " << trimmed << '\n';;
# endif

	return GALIL_EXAMPLE_OK;
}


int x_dr_motioncomplete(GCon g, GCStringIn axes)
{
		
	GReturn rc;
	GDataRecord r;
	
	
	int original_dr;
	x_e(GCmdI(g, "MG_DR", &original_dr)); //grab the current DR value
	x_e(GRecordRate(g, 100)); //set up DR to 10Hz
	
	int len = strlen(axes);
	UW* axis_status;
	for (int i = 0; i < len; /*blank*/) //iterate through all chars in axes to make the axis mask
	{
		rc = GRecord(g, &r, G_DR);
		//support just A-H
		switch (axes[i])
		{
		case 'A':
			axis_status = &r.dmc4000.axis_a_status;
			break;
		case 'B':
			axis_status = &r.dmc4000.axis_b_status;
			break;
		case 'C':
			axis_status = &r.dmc4000.axis_c_status;
			break;
		case 'D':
			axis_status = &r.dmc4000.axis_d_status;
			break;
		case 'E':
			axis_status = &r.dmc4000.axis_e_status;
			break;
		case 'F':
			axis_status = &r.dmc4000.axis_f_status;
			break;
		case 'G':
			axis_status = &r.dmc4000.axis_g_status;
			break;
		case 'H':
			axis_status = &r.dmc4000.axis_h_status;
			break;
		default:
			axis_status = 0;
		}

		if (axis_status)
			if (!(*axis_status & 0x8000)) //bit 15 is "Move in progress"
				i++;
	}
	
	char buf[16];
	sprintf(buf, "DR %d", original_dr);
	x_e(GCmd(g, buf)); //restore DR

	return GALIL_EXAMPLE_OK;

}