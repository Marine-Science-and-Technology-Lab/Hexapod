#include "gcl_galil.h"
using namespace std;

vector<std::string> Galil::sources()
{
	vector<std::string> s;
	for (auto it = d->map.begin(); it != d->map.end(); ++it)
		s.push_back(it->first);

	std::sort(s.begin(), s.end());

	return s;
}

void Galil::recordsStart(double period_ms)
{
	ec(GRecordRate(d->g, period_ms));
}

vector<char> Galil::record(const std::string& method)
{
	GDataRecord record;
	ec(GRecord(d->g, &record, method == "QR" ? G_QR : G_DR));
	vector <char> record_vector;
	for (int i = 0; i < sizeof(GDataRecord); i++)
		record_vector.push_back(record.byte_array[i]);

	return record_vector;
}

double Galil::sourceValue(const std::vector<char>& record, const std::string& source)
{

	try
	{
		const Source& s = d->map.at(source); //use at() function so silent insert does not occur if bad source string is used.
		int return_value = 0;
		if (s.type[0] == 'U')  //unsigned
			switch (s.type[1])
		{
			case 'B':  return_value = *(unsigned char*)(&record[0] + s.byte);  break;
			case 'W':  return_value = *(unsigned short*)(&record[0] + s.byte);  break;
			case 'L':  return_value = *(unsigned int*)(&record[0] + s.byte);  break;
		}
		else //s.type[0] == 'S'  //signed
			switch (s.type[1])
		{
			case 'B':  return_value = *(char*)(&record[0] + s.byte);  break;
			case 'W':  return_value = *(short*)(&record[0] + s.byte);  break;
			case 'L':  return_value = *(int*)(&record[0] + s.byte);  break;
		}

		if (s.bit >= 0) //this is a bit field
		{
			bool bTRUE = s.scale > 0; //invert logic if scale is <= 0  
			return return_value & (1 << s.bit) ? bTRUE : !bTRUE; //check the bit
		}
		else
			return (return_value / s.scale) + s.offset;

	}
	catch (const std::out_of_range& e) //bad source
	{
		(void)e; //unused
		return 0.0;
	}

}

string Galil::source(const std::string& field, const std::string& source)
{
	try
	{
		const Source& s = d->map.at(source); //use at() function so silent insert does not occur if bad source string is used.
		if (field == "Description")
			return s.description;

		if (field == "Units")
			return s.units;

		if (field == "Scale")
			return to_string(s.scale);

		if (field == "Offset")
			return to_string(s.offset);

		return ""; //no matches
	}
	catch (const std::out_of_range& e) //bad source
	{
		(void)e;//unused
		return ""; //no matches
	}

}

void Galil::setSource(const std::string& field, const std::string& source, const std::string& to)
{
	try
	{
		Source& s = d->map.at(source); //use at() function so silent insert does not occur if bad source string is used.
		if (field == "Description")
		{
			s.description = to;
			return;
		}

		if (field == "Units")
		{
			s.units = to;
			return;
		}

		if (field == "Scale")
		{
			s.scale = stod(to);
			return;
		}

		if (field == "Offset")
		{
			s.offset = stod(to);
			return;
		}
	}
	catch (const std::out_of_range& e) //bad source
	{
		(void)e;//unused
		return;
	}
}



void GalilPrivate::InitializeDataRecord()
{
	map.clear(); //clear the map if thre is anything in it

	//infer data record structure from QZ
	string qz = q->command("QZ");
	vector<int> qz_split;
	size_t start = 0;
	for (size_t i = 0; i < qz.length(); i++) //ad hoc split
	{
		if (qz[i] == ',')
		{
			qz_split.push_back(stoi(qz.substr(start, i - start)));
			start = i + 1; //jump over delim
			continue;
		}

		if (i == qz.length() - 1) //last token
		{
			qz_split.push_back(stoi(qz.substr(start)));
			break;
		}
	}

	if (4 != qz_split.size()) return; //parsing error or bad response

	int axes = qz_split[0];


	//weed out PCI cards
	string rv = q->command("\x12\x16"); //^R^V
	if (rv.find("DMC18") != string::npos) //PCI-based card
	{
		if (rv.length() >= 7)
		{
			if (rv[6] == '6') return Init1806(axes);
			if (rv[6] == '0') return Init1800(axes, false);
			if (rv[6] == '2') return Init1800(axes, true);
		}
	}

	int gen_status = qz_split[1];
	if (gen_status == 18) return Init30010(rv.find("DMC31") != string::npos); // DMC300x0 returns 1 18 16 36, search for "DMC31" in ^R^V to determine 16bit ADC

	int  axis_block = qz_split[3]; //size of the axis block data in data record
	if (axis_block == 36) return Init4000(axes);  //DMC40x0/DMC41x3/DMC50000         8 52 26 36
	if (axis_block == 28) return Init2103(axes);     //DMC14x5/2xxx/                 8 24 16 28 //also Optima

	//if here, should be an RIO
	if (axis_block != 0) return; //RIO has a 0 in the axis block data

	//Determine the RIO type
	int io_block = qz_split[2];

	//RIO-47300 has 4 extra bytes in the I/O block
	bool rio3 = ((io_block == 52) || (io_block == 60) || (io_block == 68)); // RIO-47300 Standard, with Exteneded I/O, with Quad/Biss/SSi

	//SER tacks 4 longs on the end of the data record (4 encoders)
	bool rioser = ((io_block == 64) || (io_block == 68)); // 471x2,472x2 OR 47300 with SER

	//Extended I/O tacks 8 bytes on the end of the data rrecord, three bytes of each of I/O, one padding for each
	bool rio3ex = (io_block == 60); // RIO-47300 with extended I/O. Mutually exclusive with SER


	InitRio(rio3);

	if (rio3ex) InitRio3_24Ex();
	if (rioser) InitRioSer(rio3);

}

string GalilPrivate::ax(string prefix, int axis, string suffix)
{
	return prefix + (char)('A' + axis) + suffix;
}

void GalilPrivate::input_bits(int byte, int num)
{
	stringstream ss;

	for (int i = 0; i < 8; i++)
	{
		ss << "@IN[";
		ss << setw(2) << setfill('0') << right << num;
		ss << "]";
		map[ss.str()] = Source(byte, "UB", i, "Boolean", "Digital input " + to_string(num));
		ss.str("");
		num++;
	}
}

void GalilPrivate::output_bits(int byte, int num)
{
	stringstream ss;

	for (int i = 0; i < 8; i++)
	{
		ss << "@OUT[";
		ss << setw(2) << setfill('0') << right << num;
		ss << "]";
		map[ss.str()] = Source(byte, "UB", i, "Boolean", "Digital output " + to_string(num));
		ss.str("");
		num++;
	}
}

void GalilPrivate::aq_analog(int byte, int input_num)
{
	//When analog voltage decoding depends upon AQ setting.
	string type; //for interpreting analog as signed/unsigned
	double divisor; //for dividing ADC counts to calc volts
	int val;
	string command = "MG{Z10.0}_AQ" + to_string(input_num);
	if (GCmdI(g, command.c_str(), &val) == G_NO_ERROR) //don't add analog if error on AQ
	{
		switch (val)
		{
		case 1: case -1:  divisor = 32768.0 / 5;   type = "SW";  break;   //  -5 to 5  V   -32768 to 32767
		case 3: case -3:  divisor = 65536.0 / 5;   type = "UW";  break;   //   0 to 5  V        0 to 65535
		case 4: case -4:  divisor = 65536.0 / 10;  type = "UW";  break;   //   0 to 10 V        0 to 65535
		case 2: case -2:  default: //AQ 2 is the default value
			divisor = 32768.0 / 10;  type = "SW";  break;   // -10 to 10 V   -32768 to 32767
		}
		map["@AN[" + to_string(input_num + 1) + "]"] = Source(byte, type, -1, "V", "Analog input " + to_string(input_num), divisor);
	}
}

void GalilPrivate::dq_analog(int byte, int input_num)
{
	//When analog voltage decoding depends upon DQ setting.
	string type; //for interpreting analog as signed/unsigned
	double divisor; //for dividing ADC counts to calc volts
	int val;
	string command = "MG{Z10.0}_DQ" + to_string(input_num);
	if (GCmdI(g, command.c_str(), &val) == G_NO_ERROR) //don't add analog if error on AQ
	{
		switch (val)
		{
		case 3: divisor = 32768.0 / 5;   type = "SW";  break;   //  -5 to 5  V   -32768 to 32767
		case 1: divisor = 65536.0 / 5;   type = "UW";  break;   //   0 to 5  V        0 to 65535
		case 2: divisor = 65536.0 / 10;  type = "UW";  break;   //   0 to 10 V        0 to 65535
		case 4: default: //DQ 4 is the default value
			divisor = 32768.0 / 10;  type = "SW";  break;   // -10 to 10 V   -32768 to 32767
		}
		map["@AO[" + to_string(input_num + 1) + "]"] = Source(byte, type, -1, "V", "Analog output " + to_string(input_num), divisor);
	}
}

void GalilPrivate::Init4000(int axes)
{
	//0-3 Header is ignored in GCL

	map["TIME"] = Source(4, "UW", -1, "samples", "Sample counter");

	//Digital Inputs
	map["_TI0"] = Source(6, "UB", -1, "", "Digital inputs 1 to 8");
	input_bits(6, 1);

	map["_TI1"] = Source(7, "UB", -1, "", "Digital inputs 9 to 16"); //TI always included
	if (axes > 4) //9-16 depend on axes 5-8
		input_bits(7, 9);

	//Digital outputs
	map["_OP0"] = Source(16, "UW", -1, "", "Digital outputs 1 to 16");
	output_bits(16, 1);

	if (axes > 4) //9-16 depend on axes 5-8
		output_bits(17, 9);

	//Extended I/O
	int co = -1;
	if (GCmdI(g, "MG_CO", &co) == G_NO_ERROR) //41x3 will ? here
	{
		map["_TI2"] = Source(8, "UB", -1, "", "Digital inputs 17 to 24"); //TI always included in gcl
		map["_TI3"] = Source(9, "UB", -1, "", "Digital inputs 25 to 32");
		map["_TI4"] = Source(10, "UB", -1, "", "Digital inputs 33 to 40");
		map["_TI5"] = Source(11, "UB", -1, "", "Digital inputs 41 to 48");

		map["_OP1"] = Source(18, "UW", -1, "", "Digital outputs 17 to 32"); //OP always included in gcl
		map["_OP2"] = Source(20, "UW", -1, "", "Digital outputs 33 to 48");

		if (co & 0x00000001) //bank 2 is output
			output_bits(18, 17);
		else //bank 2 in input
			input_bits(8, 17);

		if (co & 0x00000002) //bank 3 is output
			output_bits(19, 25);
		else //bank 3 is input
			input_bits(9, 25);

		if (co & 0x00000004) //bank 4 is output
			output_bits(20, 33);
		else //bank 4 is input
			input_bits(10, 33);

		if (co & 0x00000008) //bank 5 is output
			output_bits(21, 41);
		else
			input_bits(11, 41);
	}

	//Ethernet Handle Status
	map["_IHA2"] = Source(42, "UB", -1, "", "Handle A Ethernet status");
	map["_IHB2"] = Source(43, "UB", -1, "", "Handle B Ethernet status");
	map["_IHC2"] = Source(44, "UB", -1, "", "Handle C Ethernet status");
	map["_IHD2"] = Source(45, "UB", -1, "", "Handle D Ethernet status");
	map["_IHE2"] = Source(46, "UB", -1, "", "Handle E Ethernet status");
	map["_IHF2"] = Source(47, "UB", -1, "", "Handle F Ethernet status");
	map["_IHG2"] = Source(48, "UB", -1, "", "Handle G Ethernet status");
	map["_IHH2"] = Source(49, "UB", -1, "", "Handle H Ethernet status");

	map["_TC"] = Source(50, "UB", -1, "", "Error code");

	//Thread status
	map["NO0"] = Source(51, "UB", 0, "Boolean", "Thread 0 running");
	map["NO1"] = Source(51, "UB", 1, "Boolean", "Thread 1 running");
	map["NO2"] = Source(51, "UB", 2, "Boolean", "Thread 2 running");
	map["NO3"] = Source(51, "UB", 3, "Boolean", "Thread 3 running");
	map["NO4"] = Source(51, "UB", 4, "Boolean", "Thread 4 running");
	map["NO5"] = Source(51, "UB", 5, "Boolean", "Thread 5 running");
	map["NO6"] = Source(51, "UB", 6, "Boolean", "Thread 6 running");
	map["NO7"] = Source(51, "UB", 7, "Boolean", "Thread 7 running");

	//Amplifier error status
	map["TA00"] = Source(52, "UB", 0, "Boolean", "Axis A-D over current");
	map["TA01"] = Source(52, "UB", 1, "Boolean", "Axis A-D over voltage");
	map["TA02"] = Source(52, "UB", 2, "Boolean", "Axis A-D over temperature");
	map["TA03"] = Source(52, "UB", 3, "Boolean", "Axis A-D under voltage");
	map["TA04"] = Source(52, "UB", 4, "Boolean", "Axis E-H over current");
	map["TA05"] = Source(52, "UB", 5, "Boolean", "Axis E-H over voltage");
	map["TA06"] = Source(52, "UB", 6, "Boolean", "Axis E-H over temperature");
	map["TA07"] = Source(52, "UB", 7, "Boolean", "Axis E-H under voltage");

	map["TA1A"] = Source(53, "UB", 0, "Boolean", "Axis A hall error");
	map["TA1B"] = Source(53, "UB", 1, "Boolean", "Axis B hall error");
	map["TA1C"] = Source(53, "UB", 2, "Boolean", "Axis C hall error");
	map["TA1D"] = Source(53, "UB", 3, "Boolean", "Axis D hall error");
	map["TA1E"] = Source(53, "UB", 4, "Boolean", "Axis E hall error");
	map["TA1F"] = Source(53, "UB", 5, "Boolean", "Axis F hall error");
	map["TA1G"] = Source(53, "UB", 6, "Boolean", "Axis G hall error");
	map["TA1H"] = Source(53, "UB", 7, "Boolean", "Axis H hall error");

	map["TA2A"] = Source(54, "UB", 0, "Boolean", "Axis A at _TKA peak current");
	map["TA2B"] = Source(54, "UB", 1, "Boolean", "Axis B at _TKB peak current");
	map["TA2C"] = Source(54, "UB", 2, "Boolean", "Axis C at _TVC peak current");
	map["TA2D"] = Source(54, "UB", 3, "Boolean", "Axis D at _TKD peak current");
	map["TA2E"] = Source(54, "UB", 4, "Boolean", "Axis E at _TKE peak current");
	map["TA2F"] = Source(54, "UB", 5, "Boolean", "Axis F at _TKF peak current");
	map["TA2G"] = Source(54, "UB", 6, "Boolean", "Axis G at _TKG peak current");
	map["TA2H"] = Source(54, "UB", 7, "Boolean", "Axis H at _TKH peak current");

	map["TA3AD"] = Source(55, "UB", 0, "Boolean", "Axis A-D ELO active");
	map["TA3EH"] = Source(55, "UB", 1, "Boolean", "Axis E-H ELO active");

	//contour mode
	map["CD"] = Source(56, "UL", -1, "segments", "Contour segment count");
	map["_CM"] = Source(60, "UW", -1, "elements", "Contour buffer space");

	//S plane
	map["_CSS"] = Source(62, "UW", -1, "segments", "Axis S segment count");
	map["VDS"] = Source(64, "UB", 3, "Boolean", "Axis S final deceleration");
	map["STS"] = Source(64, "UB", 4, "Boolean", "Axis S stopping");
	map["VSS"] = Source(64, "UB", 5, "Boolean", "Axis S slewing");
	map["_BGS"] = Source(65, "UB", 7, "Boolean", "Axis S moving");
	map["_AVS"] = Source(66, "SL", -1, "counts", "Axis S length");
	map["_LMS"] = Source(70, "UW", -1, "elements", "Axis S buffer speace");

	//T plane
	map["_CST"] = Source(72, "UW", -1, "segments", "Axis T segment count");
	map["VDT"] = Source(74, "UB", 3, "Boolean", "Axis T final deceleration");
	map["STT"] = Source(74, "UB", 4, "Boolean", "Axis T stopping");
	map["VST"] = Source(74, "UB", 5, "Boolean", "Axis T slewing");
	map["_BGT"] = Source(75, "UB", 7, "Boolean", "Axis T moving");
	map["_AVT"] = Source(76, "SL", -1, "counts", "Axis T length");
	map["_LMT"] = Source(80, "UW", -1, "elements", "Axis T buffer speace");

	//per-axis data
	int base = 82; //start of A axis data
	for (int i = 0; i < axes; i++)
	{
		map[ax("_MO", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " motor off"));
		map[ax("HM", i, "3")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " found index"));
		map[ax("_AL", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " latch armed"));
		map[ax("DC", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " began deceleration"));
		map[ax("ST", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " began stop"));
		map[ax("SP", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " began slew"));
		map[ax("CM", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in contour mode"));
		map[ax("JG", i, "-")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " negative move"));
		++base; //83
		map[ax("VM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in VM or LI mode"));
		map[ax("HM", i, "2")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " finding index"));
		map[ax("HM", i, "1")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " coming off home switch"));
		map[ax("HM", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " home command issued"));
		map[ax("FE", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " find edge issued"));
		map[ax("PA", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " in PA mode"));
		map[ax("PR", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in PA or PR mode"));
		map[ax("_BG", i, "")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " move in progress"));
		++base; //84
		map[ax("MT", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in stepper mode"));
		map[ax("_HM", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " home switch"));
		map[ax("_LR", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " reverse limit switch"));
		map[ax("_LF", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " forward limit switch"));
		//4 and 5 reserved
		map[ax("AL", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch digital input"));
		map[ax("_AL", i, "=0")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch occurred"));
		++base; //85
		map[ax("_SC", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " stop code"));
		++base; //86
		map[ax("_RP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " reference position"));
		base += 4; //90
		map[ax("_TP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " encoder position"));
		base += 4; //94
		map[ax("_TE", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " position error"));
		base += 4; //98
		map[ax("_TD", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " dual encoder position"));
		base += 4; //102
		map[ax("_TV", i, "")] = Source(base, "SL", -1, "counts/s", ax("Axis ", i, " filtered velocity"), 64);
		base += 4; //106
		map[ax("_TT", i, "")] = Source(base, "SL", -1, "V", ax("Axis ", i, " torque (DAC)"), 3255);
		base += 4; //110

		//Analog voltage decoding depends upon AQ setting.
		aq_analog(base, i + 1);
		base += 2; //112

		map[ax("_QH", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " hall sensors"));
		base += 1; //113 reserved
		base += 1; //114
		map[ax("_ZA", i, "")] = Source(base, "SL", -1, "", ax("Axis ", i, " user variable"));
		base += 4; //118

	}// for, axis data
}

void GalilPrivate::Init1806(int axes)
{
	map["TIME"] = Source(0, "UW", -1, "samples", "Sample counter");

	//Digital inputs
	map["_TI0"] = Source(2, "UB", -1, "", "Digital inputs 1 to 8");
	input_bits(2, 1);

	map["_TI1"] = Source(3, "UB", -1, "", "Digital inputs 9 to 16"); //TI always included
	if (axes > 4) //9-16 depend on axes 5-8
		input_bits(3, 9);


	//Digital outputs
	map["_OP0"] = Source(12, "UW", -1, "", "Digital outputs 1 to 16");
	output_bits(12, 1);

	if (axes > 4) //9-16 depend on axes 5-8
		output_bits(13, 9);


	//Extended I/O
	int co = -1;
	if (GCmdI(g, "MG_CO", &co) == G_NO_ERROR)
	{
		map["_TI2"] = Source(4, "UB", -1, "", "Digital inputs 17 to 24"); //TI always included in gcl
		map["_TI3"] = Source(5, "UB", -1, "", "Digital inputs 25 to 32");
		map["_TI4"] = Source(6, "UB", -1, "", "Digital inputs 33 to 40");
		map["_TI5"] = Source(7, "UB", -1, "", "Digital inputs 41 to 48");
		map["_TI6"] = Source(8, "UB", -1, "", "Digital inputs 49 to 56");
		map["_TI7"] = Source(9, "UB", -1, "", "Digital inputs 57 to 64");
		map["_TI8"] = Source(10, "UB", -1, "", "Digital inputs 65 to 72");
		map["_TI9"] = Source(11, "UB", -1, "", "Digital inputs 73 to 80");

		map["_OP1"] = Source(14, "UW", -1, "", "Digital outputs 17 to 32"); //OP always included in gcl
		map["_OP2"] = Source(16, "UW", -1, "", "Digital outputs 33 to 48");
		map["_OP3"] = Source(18, "UW", -1, "", "Digital outputs 49 to 64");
		map["_OP4"] = Source(20, "UW", -1, "", "Digital outputs 65 to 80");

		if (co & 0x00000001) //bank 2 is output
			output_bits(14, 17);
		else //input
			input_bits(4, 17);

		if (co & 0x00000002) //bank 3 is output
			output_bits(15, 25);
		else //input
			input_bits(5, 25);

		if (co & 0x00000004) //bank 4 is output
			output_bits(16, 33);
		else //input
			input_bits(6, 33);

		if (co & 0x00000008) //bank 5 is output
			output_bits(17, 41);
		else //input
			input_bits(7, 41);

		if (co & 0x00000010) //bank 6 is output
			output_bits(18, 49);
		else //input
			input_bits(8, 49);

		if (co & 0x00000020) //bank 7 is output
			output_bits(19, 57);
		else //input
			input_bits(9, 57);

		if (co & 0x00000040) //bank 8 is output
			output_bits(20, 65);
		else //input
			input_bits(10, 65);

		if (co & 0x00000080) //bank 9 is output
			output_bits(21, 73);
		else //input
			input_bits(11, 73);
	} //extended io

	map["_TC"] = Source(46, "UB", -1, "", "Error code");

	//Thread status
	map["NO0"] = Source(47, "UB", 0, "Boolean", "Thread 0 running");
	map["NO1"] = Source(47, "UB", 1, "Boolean", "Thread 1 running");
	map["NO2"] = Source(47, "UB", 2, "Boolean", "Thread 2 running");
	map["NO3"] = Source(47, "UB", 3, "Boolean", "Thread 3 running");
	map["NO4"] = Source(47, "UB", 4, "Boolean", "Thread 4 running");
	map["NO5"] = Source(47, "UB", 5, "Boolean", "Thread 5 running");
	map["NO6"] = Source(47, "UB", 6, "Boolean", "Thread 6 running");
	map["NO7"] = Source(47, "UB", 7, "Boolean", "Thread 7 running");

	//contour mode
	map["CD"] = Source(52, "UL", -1, "segments", "Contour segment count");
	map["_CM"] = Source(56, "UW", -1, "elements", "Contour buffer space");

	//S plane
	map["_CSS"] = Source(58, "UW", -1, "segments", "Axis S segment count");
	map["VDS"] = Source(60, "UB", 3, "Boolean", "Axis S final deceleration");
	map["STS"] = Source(60, "UB", 4, "Boolean", "Axis S stopping");
	map["VSS"] = Source(60, "UB", 5, "Boolean", "Axis S slewing");
	map["_BGS"] = Source(61, "UB", 7, "Boolean", "Axis S moving");
	map["_AVS"] = Source(62, "SL", -1, "counts", "Axis S length");
	map["_LMS"] = Source(66, "UW", -1, "elements", "Axis S buffer speace");

	//T plane
	map["_CST"] = Source(68, "UW", -1, "segments", "Axis T segment count");
	map["VDT"] = Source(70, "UB", 3, "Boolean", "Axis T final deceleration");
	map["STT"] = Source(70, "UB", 4, "Boolean", "Axis T stopping");
	map["VST"] = Source(70, "UB", 5, "Boolean", "Axis T slewing");
	map["_BGT"] = Source(71, "UB", 7, "Boolean", "Axis T moving");
	map["_AVT"] = Source(72, "SL", -1, "counts", "Axis T length");
	map["_LMT"] = Source(76, "UW", -1, "elements", "Axis T buffer speace");

	//per-axis data
	int base = 78; //start of A axis data
	for (int i = 0; i < axes; i++)
	{
		map[ax("_MO", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " motor off"));
		map[ax("HM", i, "3")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " found index"));
		map[ax("_AL", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " latch armed"));
		map[ax("DC", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " began deceleration"));
		map[ax("ST", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " began stop"));
		map[ax("SP", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " began slew"));
		map[ax("CM", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in contour mode"));
		map[ax("JG", i, "-")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " negative move"));
		++base; //79
		map[ax("VM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in VM or LI mode"));
		map[ax("HM", i, "2")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " finding index"));
		map[ax("HM", i, "1")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " coming off home switch"));
		map[ax("HM", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " home command issued"));
		map[ax("FE", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " find edge issued"));
		map[ax("PA", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " in PA mode"));
		map[ax("PR", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in PA or PR mode"));
		map[ax("_BG", i, "")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " move in progress"));
		++base; //80
		map[ax("MT", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in stepper mode"));
		map[ax("_HM", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " home switch"));
		map[ax("_LR", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " reverse limit switch"));
		map[ax("_LF", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " forward limit switch"));
		//4 and 5 reserved
		map[ax("AL", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch digital input"));
		map[ax("_AL", i, "=0")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch occurred"));
		++base; //81
		map[ax("_SC", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " stop code"));
		++base; //82
		map[ax("_RP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " reference position"));
		base += 4; //86
		map[ax("_TP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " encoder position"));
		base += 4; //90
		map[ax("_TE", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " position error"));
		base += 4; //94
		map[ax("_TD", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " dual encoder position"));
		base += 4; //98
		map[ax("_TV", i, "")] = Source(base, "SL", -1, "counts/s", ax("Axis ", i, " filtered velocity"), 64);
		base += 4; //102
		map[ax("_TT", i, "")] = Source(base, "SL", -1, "V", ax("Axis ", i, " torque (DAC)"), 3255);
		base += 4; //106

		map["@AN[" + to_string(i + 1) + "]"] = Source(base, "SW", -1, "V", "Analog input " + to_string(i + 1), 3276.8);

		base += 2; //108 reserved
		base += 1; //109 reserved
		base += 1; //110
		map[ax("_ZA", i, "")] = Source(base, "SL", -1, "", ax("Axis ", i, " user variable"));
		base += 4; //114
	}// for, axis data
}

void GalilPrivate::Init1800(int axes, bool dmc1802)
{

	map["TIME"] = Source(0, "UW", -1, "samples", "Sample counter");

	//Digital Inputs
	map["_TI0"] = Source(2, "UB", -1, "", "Digital inputs 1 to 8");
	input_bits(2, 1);

	map["_TI1"] = Source(3, "UB", -1, "", "Digital inputs 9 to 16"); //TI always included
	if (axes > 4) //9-16 depend on axes 5-8
		input_bits(3, 9);

	//Digital outputs
	map["_OP0"] = Source(12, "UW", -1, "", "Digital outputs 1 to 16");
	output_bits(12, 1);
	if (axes > 4) //9-16 depend on axes 5-8
		output_bits(13, 9);

	//Extended I/O
	int co = -1;
	if (GCmdI(g, "MG_CO", &co) == G_NO_ERROR)
	{
		map["_TI2"] = Source(4, "UB", -1, "", "Digital inputs 17 to 24"); //TI always included in gcl
		map["_TI3"] = Source(5, "UB", -1, "", "Digital inputs 25 to 32");
		map["_TI4"] = Source(6, "UB", -1, "", "Digital inputs 33 to 40");
		map["_TI5"] = Source(7, "UB", -1, "", "Digital inputs 41 to 48");
		map["_TI6"] = Source(8, "UB", -1, "", "Digital inputs 49 to 56");
		map["_TI7"] = Source(9, "UB", -1, "", "Digital inputs 57 to 64");
		map["_TI8"] = Source(10, "UB", -1, "", "Digital inputs 65 to 72");
		map["_TI9"] = Source(11, "UB", -1, "", "Digital inputs 73 to 80");

		map["_OP1"] = Source(14, "UW", -1, "", "Digital outputs 17 to 32"); //OP always included in gcl
		map["_OP2"] = Source(16, "UW", -1, "", "Digital outputs 33 to 48");
		map["_OP3"] = Source(18, "UW", -1, "", "Digital outputs 49 to 64");
		map["_OP4"] = Source(20, "UW", -1, "", "Digital outputs 65 to 80");

		if (co & 0x00000001) //bank 2 is output
			output_bits(14, 17);
		else //input
			input_bits(4, 17);

		if (co & 0x00000002) //bank 3 is output
			output_bits(15, 25);
		else //input
			input_bits(5, 25);

		if (co & 0x00000004) //bank 4 is output
			output_bits(16, 33);
		else //input
			input_bits(6, 33);

		if (co & 0x00000008) //bank 5 is output
			output_bits(17, 41);
		else //input
			input_bits(7, 41);

		if (co & 0x00000010) //bank 6 is output
			output_bits(18, 49);
		else //input
			input_bits(8, 49);

		if (co & 0x00000020) //bank 7 is output
			output_bits(19, 57);
		else //input
			input_bits(9, 57);

		if (co & 0x00000040) //bank 8 is output
			output_bits(20, 65);
		else //input
			input_bits(10, 65);

		if (co & 0x00000080) //bank 9 is output
			output_bits(21, 73);
		else //input
			input_bits(11, 73);
	} //extended io

	map["_TC"] = Source(22, "UB", -1, "", "Error code");

	//general status
	map["_EO"] = Source(23, "UB", 0, "Boolean", "Echo on");
	map["TR"] = Source(23, "UB", 1, "Boolean", "Trace on");
	map["IN"] = Source(23, "UB", 2, "Boolean", "IN waiting for user input");
	map["XQ"] = Source(23, "UB", 7, "Boolean", "Program running");

	//S plane
	map["_CSS"] = Source(24, "UW", -1, "segments", "Axis S segment count");
	map["VDS"] = Source(26, "UB", 3, "Boolean", "Axis S final deceleration");
	map["STS"] = Source(26, "UB", 4, "Boolean", "Axis S stopping");
	map["VSS"] = Source(26, "UB", 5, "Boolean", "Axis S slewing");
	map["_BGS"] = Source(27, "UB", 7, "Boolean", "Axis S moving");
	map["_AVS"] = Source(28, "SL", -1, "counts", "Axis S length");

	//T plane
	map["_CST"] = Source(32, "UW", -1, "segments", "Axis T segment count");
	map["VDT"] = Source(34, "UB", 3, "Boolean", "Axis T final deceleration");
	map["STT"] = Source(34, "UB", 4, "Boolean", "Axis T stopping");
	map["VST"] = Source(34, "UB", 5, "Boolean", "Axis T slewing");
	map["_BGT"] = Source(35, "UB", 7, "Boolean", "Axis T moving");
	map["_AVT"] = Source(36, "SL", -1, "counts", "Axis T length");

	//per-axis data
	int base = 40; //start of A axis data
	for (int i = 0; i < axes; i++)
	{
		map[ax("_MO", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " motor off"));
		map[ax("_OE", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " off-on-error set"));
		map[ax("_AL", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " latch armed"));
		map[ax("DC", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " began deceleration"));
		map[ax("ST", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " began stop"));
		map[ax("SP", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " began slew"));
		map[ax("CM", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in contour mode"));
		map[ax("JG", i, "-")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " negative move"));
		++base; //41
		map[ax("VM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in VM or LI mode"));
		map[ax("HM", i, "2")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " finding index"));
		map[ax("HM", i, "1")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " coming off home switch"));
		map[ax("HM", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " home command issued"));
		map[ax("FE", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " find edge issued"));
		map[ax("PA", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " in PA mode"));
		map[ax("PR", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in PA or PR mode"));
		map[ax("_BG", i, "")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " move in progress"));
		++base; //42
		map[ax("SM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " stepper jumper installed"));
		map[ax("_HM", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " home switch"));
		map[ax("_LR", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " reverse limit switch"));
		map[ax("_LF", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " forward limit switch"));
		//4 and 5 reserved
		map[ax("AL", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch digital input"));
		map[ax("_AL", i, "=0")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch occurred"));
		++base; //43
		map[ax("_SC", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " stop code"));
		++base; //44
		map[ax("_RP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " reference position"));
		base += 4; //48
		map[ax("_TP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " encoder position"));
		base += 4; //52
		map[ax("_TE", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " position error"));
		base += 4; //56
		map[ax("_TD", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " dual encoder position"));
		base += 4; //60
		map[ax("_TV", i, "")] = Source(base, "SL", -1, "counts/s", ax("Axis ", i, " filtered velocity"), 64);
		base += 4; //64
		map[ax("_TT", i, "")] = Source(base, "SW", -1, "V", ax("Axis ", i, " torque (DAC)"), 3255);
		base += 2; //66

		if (!dmc1802) //1800 has onboard Analog inputs
			map["@AN[" + to_string(i + 1) + "]"] = Source(base, "SW", -1, "V", "Analog input " + to_string(i + 1), 3276.8);
		
		base += 2; //68, pointing to next axis
	} //for
}


void GalilPrivate::Init30010(bool dmc31010)
{
	//0-3 Header is ignored in GCL

	map["TIME"] = Source(4, "UW", -1, "samples", "Sample counter");

	map["@IN[1]"] = Source(6, "UB", 0, "Boolean", "Digital input 1");
	map["@IN[2]"] = Source(6, "UB", 1, "Boolean", "Digital input 2");
	map["@IN[3]"] = Source(6, "UB", 2, "Boolean", "Digital input 3");
	map["@IN[4]"] = Source(6, "UB", 3, "Boolean", "Digital input 4");
	map["@IN[5]"] = Source(6, "UB", 4, "Boolean", "Digital input 5");
	map["@IN[6]"] = Source(6, "UB", 5, "Boolean", "Digital input 6");
	map["@IN[7]"] = Source(6, "UB", 6, "Boolean", "Digital input 7");
	map["@IN[8]"] = Source(6, "UB", 7, "Boolean", "Digital input 8");

	map["@OUT[1]"] = Source(8, "UB", 0, "Boolean", "Digital output 1");
	map["@OUT[2]"] = Source(8, "UB", 1, "Boolean", "Digital output 2");
	map["@OUT[3]"] = Source(8, "UB", 2, "Boolean", "Digital output 3");
	map["@OUT[4]"] = Source(8, "UB", 3, "Boolean", "Digital output 4");

	map["_TC"] = Source(10, "UB", -1, "", "Error code");

	//Thread status
	map["NO0"] = Source(11, "UB", 0, "Boolean", "Thread 0 running");
	map["NO1"] = Source(11, "UB", 1, "Boolean", "Thread 1 running");
	map["NO2"] = Source(11, "UB", 2, "Boolean", "Thread 2 running");
	map["NO3"] = Source(11, "UB", 3, "Boolean", "Thread 3 running");
	map["NO4"] = Source(11, "UB", 4, "Boolean", "Thread 4 running"); //Firmware prior to 1.2a has only 4 threads
	map["NO5"] = Source(11, "UB", 5, "Boolean", "Thread 5 running");

	//Analog IO
	//version 1.1b provides 16 bit AQ-compatible data in data record
	if (dmc31010)
		aq_analog(12, 2);
	else
		map["@AN[2]"] = Source(12, "UW", -1, "V", "Analog input 2", 13107.2); //0-5 16 bit upsampling

	map["@AO[1]"] = Source(14, "SW", -1, "V", "Analog output 1", 3276.8); //+/- 10v
	map["@AO[2]"] = Source(16, "SW", -1, "V", "Analog output 2", 3276.8);

	//Amp status
	map["TA00"] = Source(18, "UB", 0, "Boolean", "Axis A over current");
	map["TA01"] = Source(18, "UB", 1, "Boolean", "Axis A over voltage");
	map["TA02"] = Source(18, "UB", 2, "Boolean", "Axis A over temperature");
	map["TA03"] = Source(18, "UB", 3, "Boolean", "Axis A under voltage");
	map["TA1A"] = Source(19, "UB", 0, "Boolean", "Axis A hall error");
	map["TA2A"] = Source(20, "UB", 0, "Boolean", "Axis A at _TKA peak current");
	map["TA3AD"] = Source(21, "UB", 0, "Boolean", "Axis A ELO active");

	//contour mode
	map["CD"] = Source(22, "UL", -1, "segments", "Contour segment count");
	map["_CM"] = Source(26, "UW", -1, "elements", "Contour buffer space");

	//S plane
	map["_CSS"] = Source(28, "UW", -1, "segments", "Axis S segment count");
	map["VDS"] = Source(30, "UB", 3, "Boolean", "Axis S final deceleration");
	map["STS"] = Source(30, "UB", 4, "Boolean", "Axis S stopping");
	map["VSS"] = Source(30, "UB", 5, "Boolean", "Axis S slewing");
	map["_BGS"] = Source(31, "UB", 7, "Boolean", "Axis S moving");
	map["_AVS"] = Source(32, "SL", -1, "counts", "Axis S length");
	map["_LMS"] = Source(36, "UW", -1, "elements", "Axis S buffer speace");

	//per-axis data
	int base = 38; //starting offset
	int i = 0; //only one axis on 30010, no need to iterate axes

	map[ax("_MO", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " motor off"));
	map[ax("HM", i, "3")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " found index"));
	map[ax("_AL", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " latch armed"));
	map[ax("DC", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " began deceleration"));
	map[ax("ST", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " began stop"));
	map[ax("SP", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " began slew"));
	map[ax("CM", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in contour mode"));
	map[ax("JG", i, "-")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " negative move"));
	++base; //39
	map[ax("VM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in VM or LI mode"));
	map[ax("HM", i, "2")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " finding index"));
	map[ax("HM", i, "1")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " coming off home switch"));
	map[ax("HM", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " home command issued"));
	map[ax("FE", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " find edge issued"));
	map[ax("PA", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " in PA mode"));
	map[ax("PR", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in PA or PR mode"));
	map[ax("_BG", i, "")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " move in progress"));
	++base; //40
	map[ax("MT", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in stepper mode"));
	map[ax("_HM", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " home switch"));
	map[ax("_LR", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " reverse limit switch"));
	map[ax("_LF", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " forward limit switch"));
	//4 and 5 reserved
	map[ax("AL", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch digital input"));
	map[ax("_AL", i, "=0")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch occurred"));
	++base; //41
	map[ax("_SC", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " stop code"));
	++base; //42
	map[ax("_RP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " reference position"));
	base += 4; //46
	map[ax("_TP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " encoder position"));
	base += 4; //50
	map[ax("_TE", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " position error"));
	base += 4; //54
	map[ax("_TD", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " dual encoder position"));
	base += 4; //58
	map[ax("_TV", i, "")] = Source(base, "SL", -1, "counts/s", ax("Axis ", i, " filtered velocity"), 64);
	base += 4; //62
	map[ax("_TT", i, "")] = Source(base, "SL", -1, "V", ax("Axis ", i, " torque (DAC)"), 3255);
	base += 4; //66

	//version 1.1b provides 16 bit AQ-compatible data in data record
	if (dmc31010)
		aq_analog(base, i + 1);
	else
		map["@AN[" + to_string(i + 1) + "]"] = Source(base, "UW", -1, "V", "Analog input " + to_string(i + 1), 13107.2);

	base += 2; //68

	map[ax("_QH", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " hall sensors"));
	base++; //69 reserved
	base++; //70
	map[ax("_ZA", i, "")] = Source(base, "SL", -1, "", ax("Axis ", i, " user variable"));
	base += 4; //74

}

void GalilPrivate::Init2103(int axes)
{

	bool db28040 = (GCmd(g, "MG @AN[1]") == G_NO_ERROR); //probe @AN for existance of DB-28040

	//0-3 Header is ignored in GCL

	map["TIME"] = Source(4, "UW", -1, "samples", "Sample counter");

	//Digital Inputs
	map["_TI0"] = Source(6, "UB", -1, "", "Digital inputs 1 to 8");
	input_bits(6, 1);

	map["_TI1"] = Source(7, "UB", -1, "", "Digital inputs 9 to 16"); //TI always included
	if (axes > 4) //9-16 depend on axes 5-8
		input_bits(7, 9);

	//Digital outputs
	map["_OP0"] = Source(16, "UW", -1, "", "Digital outputs 1 to 16");
	output_bits(16, 1);

	if (axes > 4) //9-16 depend on axes 5-8
		output_bits(17, 9);

	//Extended I/O
	int co = -1;
	if (db28040 && (GCmdI(g, "MG_CO", &co) == G_NO_ERROR))
	{
		map["_TI2"] = Source(8, "UB", -1, "", "Digital inputs 17 to 24"); //TI always included in gcl
		map["_TI3"] = Source(9, "UB", -1, "", "Digital inputs 25 to 32");
		map["_TI4"] = Source(10, "UB", -1, "", "Digital inputs 33 to 40");
		map["_TI5"] = Source(11, "UB", -1, "", "Digital inputs 41 to 48");
		map["_TI6"] = Source(12, "UB", -1, "", "Digital inputs 49 to 56");

		map["_OP1"] = Source(18, "UW", -1, "", "Digital outputs 17 to 32"); //OP always included in gcl
		map["_OP2"] = Source(20, "UW", -1, "", "Digital outputs 33 to 48");
		map["_OP3"] = Source(22, "UW", -1, "", "Digital outputs 49 to 64");

		if (co & 0x00000001) //bank 2 is output
			output_bits(18, 17);
		else //bank 2 in input
			input_bits(8, 17);

		if (co & 0x00000002) //bank 3 is output
			output_bits(19, 25);
		else //bank 3 is input
			input_bits(9, 25);

		if (co & 0x00000004) //bank 4 is output
			output_bits(20, 33);
		else //bank 4 is input
			input_bits(10, 33);

		if (co & 0x00000008) //bank 5 is output
			output_bits(21, 41);
		else //bank 5 is input
			input_bits(11, 41);

		if (co & 0x00000010) //bank 6 is output
			output_bits(22, 49);
		else //bank 6 is input
			input_bits(12, 49);
	}

	map["_TC"] = Source(26, "UB", -1, "", "Error code");

	//general status
	map["_EO"] = Source(27, "UB", 0, "Boolean", "Echo on");
	map["TR"] = Source(27, "UB", 1, "Boolean", "Trace on");
	map["IN"] = Source(27, "UB", 2, "Boolean", "IN waiting for user input");
	map["XQ"] = Source(27, "UB", 7, "Boolean", "Program running");

	//S plane
	map["_CSS"] = Source(28, "UW", -1, "segments", "Axis S segment count");
	map["VDS"] = Source(30, "UB", 3, "Boolean", "Axis S final deceleration");
	map["STS"] = Source(30, "UB", 4, "Boolean", "Axis S stopping");
	map["VSS"] = Source(30, "UB", 5, "Boolean", "Axis S slewing");
	map["_BGS"] = Source(31, "UB", 7, "Boolean", "Axis S moving");
	map["_AVS"] = Source(32, "SL", -1, "counts", "Axis S length");

	//T plane
	map["_CST"] = Source(36, "UW", -1, "segments", "Axis T segment count");
	map["VDT"] = Source(38, "UB", 3, "Boolean", "Axis T final deceleration");
	map["STT"] = Source(38, "UB", 4, "Boolean", "Axis T stopping");
	map["VST"] = Source(38, "UB", 5, "Boolean", "Axis T slewing");
	map["_BGT"] = Source(39, "UB", 7, "Boolean", "Axis T moving");
	map["_AVT"] = Source(40, "SL", -1, "counts", "Axis T length");

	//per-axis data
	int base = 44; //start of A axis data
	for (int i = 0; i < axes; i++)
	{
		map[ax("_MO", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " motor off"));
		map[ax("_OE", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " off-on-error set"));
		map[ax("_AL", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " latch armed"));
		map[ax("DC", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " began deceleration"));
		map[ax("ST", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " began stop"));
		map[ax("SP", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " began slew"));
		map[ax("CM", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in contour mode"));
		map[ax("JG", i, "-")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " negative move"));
		++base; //45
		map[ax("VM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in VM or LI mode"));
		map[ax("HM", i, "2")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " finding index"));
		map[ax("HM", i, "1")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " coming off home switch"));
		map[ax("HM", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " home command issued"));
		map[ax("FE", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " find edge issued"));
		map[ax("PA", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " in PA mode"));
		map[ax("PR", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in PA or PR mode"));
		map[ax("_BG", i, "")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " move in progress"));
		++base; //46
		map[ax("SM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " stepper jumper installed"));
		map[ax("_HM", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " home switch"));
		map[ax("_LR", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " reverse limit switch"));
		map[ax("_LF", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " forward limit switch"));
		//4 and 5 reserved
		map[ax("AL", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch digital input"));
		map[ax("_AL", i, "=0")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch occurred"));
		++base; //47
		map[ax("_SC", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " stop code"));
		++base; //48
		map[ax("_RP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " reference position"));
		base += 4; //52
		map[ax("_TP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " encoder position"));
		base += 4; //56
		map[ax("_TE", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " position error"));
		base += 4; //60
		map[ax("_TD", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " dual encoder position"));
		base += 4; //64
		map[ax("_TV", i, "")] = Source(base, "SL", -1, "counts/s", ax("Axis ", i, " filtered velocity"), 64);
		base += 4; //68
		map[ax("_TT", i, "")] = Source(base, "SW", -1, "V", ax("Axis ", i, " torque (DAC)"), 3255);
		base += 2; //70

		if (db28040) //card has onboard Analog inputs
		{
			aq_analog(base, i + 1); //map in the analog
		}
		base += 2; //72
	} //for
}

void GalilPrivate::InitRio(bool rio3)
{


	//0-3 Header is ignored in GCL

	map["TIME"] = Source(4, "UW", -1, "samples", "Sample counter");
	map["_TC"] = Source(6, "UB", -1, "", "Error code");

	//general status
	map["_EO"] = Source(7, "UB", 0, "Boolean", "Echo on");
	map["TR"] = Source(7, "UB", 1, "Boolean", "Trace on");
	map["IN"] = Source(7, "UB", 2, "Boolean", "IN waiting for user input");
	map["XQ"] = Source(7, "UB", 7, "Boolean", "Program running");

	bool aqdq = (q->command("ID").find("(AQ)") != string::npos); //progammable analog I/O

	if (aqdq)
	{
		dq_analog(8, 0);
		dq_analog(10, 1);
		dq_analog(12, 2);
		dq_analog(14, 3);
		dq_analog(16, 4);
		dq_analog(18, 5);
		dq_analog(20, 6);
		dq_analog(22, 7);
	}
	else //fixed 0-5V
	{
		map["@AO[0]"] = Source(8, "UW", -1, "V", "Analog output 0", 13107.2, 0);
		map["@AO[1]"] = Source(10, "UW", -1, "V", "Analog output 1", 13107.2, 0);
		map["@AO[2]"] = Source(12, "UW", -1, "V", "Analog output 2", 13107.2, 0);
		map["@AO[3]"] = Source(14, "UW", -1, "V", "Analog output 3", 13107.2, 0);
		map["@AO[4]"] = Source(16, "UW", -1, "V", "Analog output 4", 13107.2, 0);
		map["@AO[5]"] = Source(18, "UW", -1, "V", "Analog output 5", 13107.2, 0);
		map["@AO[6]"] = Source(20, "UW", -1, "V", "Analog output 6", 13107.2, 0);
		map["@AO[7]"] = Source(22, "UW", -1, "V", "Analog output 7", 13107.2, 0);
	}


	if (aqdq)
	{
		aq_analog(24, 0);
		aq_analog(26, 1);
		aq_analog(28, 2);
		aq_analog(30, 3);
		aq_analog(32, 4);
		aq_analog(34, 5);
		aq_analog(36, 6);
		aq_analog(38, 7);
	}
	else  //fixed 0-5V
	{
		map["@AN[0]"] = Source(24, "UW", -1, "V", "Analog input 0", 13107.2, 0);
		map["@AN[1]"] = Source(26, "UW", -1, "V", "Analog input 1", 13107.2, 0);
		map["@AN[2]"] = Source(28, "UW", -1, "V", "Analog input 2", 13107.2, 0);
		map["@AN[3]"] = Source(30, "UW", -1, "V", "Analog input 3", 13107.2, 0);
		map["@AN[4]"] = Source(32, "UW", -1, "V", "Analog input 4", 13107.2, 0);
		map["@AN[5]"] = Source(34, "UW", -1, "V", "Analog input 5", 13107.2, 0);
		map["@AN[6]"] = Source(36, "UW", -1, "V", "Analog input 6", 13107.2, 0);
		map["@AN[7]"] = Source(38, "UW", -1, "V", "Analog input 7", 13107.2, 0);
	}

	//Data record diverges here for RIO471/472 and RIO473
	int base = 40;

	//outputs
	map["_OP0"] = Source(base, "UB", -1, "", "Digital ouputs 0-7");
	output_bits(base, 0);
	base++;

	map["_OP1"] = Source(base, "UB", -1, "", "Digital outputs 8-15");
	output_bits(base, 8);
	base++;

	if (rio3)
	{
		map["_OP2"] = Source(base, "UB", -1, "", "Digital outputs 16-23");
		output_bits(base, 16);
		base++;
		base++; //one more byte in IO space
	}

	//inputs
	map["_TI0"] = Source(base, "UB", -1, "", "Digital inputs 0-7");
	input_bits(base, 0);
	base++;

	map["_TI1"] = Source(base, "UB", -1, "", "Digital inputs 8-15");
	input_bits(base, 8);
	base++;

	if (rio3)
	{
		map["_TI2"] = Source(base, "UB", -1, "", "Digital inputs 16-23");
		input_bits(base, 16);
		base++;
		base++; //one more byte in IO space
	}

	//pulse counter
	map["_PC"] = Source(base, "UL", -1, "edges", "Pulse counter");
	base += 4;

	//user vars
	map["_ZC"] = Source(base, "SL", -1, "", "1st user variable");
	base += 4;
	map["_ZD"] = Source(base, "SL", -1, "", "2nd user variable");
	base += 4;
}

void GalilPrivate::InitRio3_24Ex()
{
	//Extended I/O tacks 8 bytes on the end of the data record, three bytes of each of I/O, one reserved for each
	map["_OP3"] = Source(60, "UB", -1, "", "Digital outputs 24-31");
	output_bits(60, 24);
	map["_OP4"] = Source(61, "UB", -1, "", "Digital outputs 32-39");
	output_bits(61, 32);
	map["_OP5"] = Source(62, "UB", -1, "", "Digital outputs 40-47");
	output_bits(62, 40);
	//byte 63 is reserved

	map["_TI3"] = Source(64, "UB", -1, "", "Digital inputs 24-31");
	input_bits(64, 24);
	map["_TI4"] = Source(65, "UB", -1, "", "Digital inputs 32-39");
	input_bits(65, 32);
	map["_TI5"] = Source(66, "UB", -1, "", "Digital inputs 40-47");
	input_bits(66, 40);
	//byte 67 is reserved
}

void GalilPrivate::InitRioSer(bool rio3)
{
	//SER tacks 4 longs on the end of the data record (4 encoders)
	int base = rio3 ? 60 : 56; //RIO 47300 base data record is longer than the other RIO products due to 24 i/o standard
	map["_QE0"] = Source(base, "SL", -1, "counts", "encoder position"); base += 4;
	map["_QE1"] = Source(base, "SL", -1, "counts", "encoder position"); base += 4;
	map["_QE2"] = Source(base, "SL", -1, "counts", "encoder position"); base += 4;
	map["_QE3"] = Source(base, "SL", -1, "counts", "encoder position"); base += 4;
}
