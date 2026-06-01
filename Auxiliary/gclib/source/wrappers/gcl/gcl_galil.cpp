#include "gcl_galil.h"
using namespace std;


void ec(GReturn rc)
{
	
	switch (rc)
	{
	case G_NO_ERROR:
		return;

	case G_TIMEOUT:
		throw string("1010 TIMEOUT ERROR.  Galil::command() took longer than timeout to return\n");
		break;

	case G_OPEN_ERROR:
		throw string("5002 OPEN ERROR.  Galil::Galil() failed to open device\n");
		break;

	case G_COMMAND_CALLED_WITH_ILLEGAL_COMMAND:
		throw string("7020 INVALID COMMAND ERROR.  DL, ED, and QD are not allowed from Galil::command()\n");
		break;

	case G_DATA_RECORD_ERROR:
		throw string("6150 WRONG BUS ERROR.  Galil::record(\"DR\") not supported on RS-232.  Use Ethernet or Galil::record(\"QR\")\n");
		break;

	case G_FIRMWARE_LOAD_NOT_SUPPORTED:
		throw string("6130 WRONG BUS ERROR.  Galil::firmwareDownloadFile() isn't allowed via Ethernet.  Use RS-232\n"); //21x3 etc.
		break;

	case G_ILLEGAL_DATA_IN_PROGRAM:
		throw string("7060 INVALID CHARACTER ERROR.  Galil::programDownload() can't download program with backslash \\ character.  Use {^92} in MG commands\n");
		break;

	case G_UNABLE_TO_COMPRESS_PROGRAM_TO_FIT:
		throw string("7061 INVALID LENGTH ERROR.  Galil::programDownload() can't compress\n");
		break;

	case G_BAD_RESPONSE_QUESTION_MARK:
		throw string("2010 COMMAND ERROR.  Galil::command() got ? instead of : response.\n"); 
		break;

	case G_BAD_FILE:
		throw string("4000 FILE ERROR.  Galil::Galil() failed to open file\n");
		break;

	case G_UNSUPPORTED_FUNCTION:
		throw string("6000 WRONG BUG ERROR. Function isn't allowed on this bus\n");
		break;


	/*
	case G_DATA_RECORD_ERROR:
	case G_BAD_ADDRESS:
	case G_BAD_LOST_DATA:
	case G_BAD_VALUE_RANGE:
	case G_BAD_FULL_MEMORY:
	case G_ARRAY_NOT_DIMENSIONED:
	case G_GCLIB_ERROR:
	case G_GCLIB_UTILITY_ERROR:
	case G_INVALID_PREPROCESSOR_OPTIONS:
	*/

	default: //couldn't find error map, make a gcl-like error from gclib error code
	{
		char buf[G_SMALL_BUFFER];
		GError(rc, buf, sizeof(buf)); //get the error message
		throw (to_string(rc) + " GCLIB ERROR. " + string(buf) + '\n');
	}
	
	}//switch
}

//! Takes a GCL address string and returns the equivalent gclib address string
string AddressConvert(const string& gcl_address)
{
	if (gcl_address.size() == 0) //this implementation does not present the user with a connections dialog if a nullstring is passed.
		throw string("5005 OPEN ERROR.  Null string specified in Galil::Galil()\n");

	if (gcl_address.find("OFFLINE") != string::npos)
		throw string("5001 OPEN ERROR.  OFFLINE specified to Galil::Galil()\n");
		
	vector<string> args;
	size_t start = 0; 
	size_t i;
	for (i = 0; i < gcl_address.size(); i++) //split into tokens
	{
		if (gcl_address[i] == ' ')
		{
			if (start < i) //if not zero length
				args.push_back(gcl_address.substr(start, i - start));

			start = i + 1; //jump over space
		}
	}
	if (start < i) //one token still remaining
		args.push_back(gcl_address.substr(start, i - start));
	
	bool ei = true; //bools to remember if data streams should be subscribed to, gcl subscribes by default
	bool mg = true;
	bool dr = true;
	string address;
	args.push_back(""); //allow safe indexing one past the end of the tokens list
	for (i = 0; i < args.size() - 1; i++)
	{
		if (args[i] == "-p1") address.append("--p1 " + args[++i] + " ");
		else if (args[i] == "-p2") address.append("--p2 " + args[++i] + " ");
		else if (args[i] == "-udp") address.append("--command UDP "); //TCP is default in gclib
		else if ((args[i] == "-ei") && (args[++i] == "0")) ei = false;
		else if ((args[i] == "-mg") && (args[++i] == "0")) mg = false;
		else if ((args[i] == "-dr") && (args[++i] == "0")) dr = false;
		else if (args[i] == "-t") address.append("-t " + args[++i] + " ");
		else if (args[i] == "-s")
		{
			address.append("-s NONE ");
			ei = false;
			mg = false;
			dr = false;
		}
		else if (args[i] == "-d") ++i; //Debug not yet supported
		else if (args[i] == "-l") ++i; //Long-timeout not used
		else if (args[i].find("COM") != string::npos)
		{
			address.append(args[i] + " --baud " + args[i + 1] + " "); //COM1 --baud 115200
			i++;// jump over baud rate on next pass
		}
		else address.append(args[i] + " "); //keep unrecognized tokens, e.g. 192.168.0.123
	}

	if (ei) address.append("-s EI ");
	if (mg) address.append("-s MG ");
	if (dr) address.append("-s DR ");
	
	address.append("-d "); //add direct-connect switch

	return address; 
}

string Galil::libraryVersion()
{
	char buf[G_SMALL_BUFFER]; //function is static, so can't access GalilPrivate members
	ec(GVersion(buf, sizeof(buf)));
	return "Galil2.dll wrapper, gclib " + string(buf);
}

vector<string> Galil::addresses()
{
	vector<string> addresses; //this is the return collection
	char buf[1024];
	ec(GAddresses(buf, sizeof(buf)));
	//buf now holds the list, but the gcl only holds the first cell
	
	short commas = 0; //counter for comma delimiters
	string address; //temp buffer for holding chars
	size_t len = strlen(buf); //don't call strlen every iteration
	for (size_t i = 0; i < len; i++)
	{
		if (buf[i] == '\n') //end of line
		{
			addresses.push_back(address);
			address.clear();
			commas = 0; //start counting commas from zero
			continue;
		}
		
		if (commas) //already saw first comma, keep looking for end of line
			continue;

		if (buf[i] == ',')
		{
			commas++; //count it
			continue;
		}
		
		address.push_back(buf[i]); //keep the char
	}//for 

	return addresses;
}

Galil::Galil(std::string address)
{
	GCon g;
	timeout_ms = 500; //default value for gcl
	ec(GOpen(AddressConvert(address).c_str(), &g));
	d = new GalilPrivate(this, g);
	d->InitializeDataRecord();
}

Galil::~Galil()
{
	GClose(d->g); //close the connection in gclib
	delete d; //free memory
}

string Galil::connection()
{
	ec(GInfo(d->g, d->tbuf, sizeof(d->tbuf)));
	return string(d->tbuf);
}

string Galil::command(const std::string& command, const std::string& terminator, const std::string& ack, bool trim)
{
	/*
	*  Note: This wrapper ignores terminator and ack. GCommand does not require them to operate.
	*  If terminator and/or ack are desired, this function can be implemented with GRead and GWrite.
	*  Please contact softwaresupport@galil.com with questions/concerns.
	*/
	ec(GTimeout(d->g, (short)timeout_ms)); //obey timeout_ms setting
	GSize bytes_read;
	char* response;
	if (trim)
		ec(GCmdT(d->g, command.c_str(), d->tbuf, sizeof(d->tbuf), &response));
	else
	{
		ec(GCommand(d->g, command.c_str(), d->tbuf, sizeof(d->tbuf), &bytes_read));
		response = d->tbuf;
	}
	ec(GTimeout(d->g, -1)); //replace timeout

	return string(response);
}

double Galil::commandValue(const std::string& command)
{
	double value;
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GCmdD(d->g, command.c_str(), &value));
	ec(GTimeout(d->g, -1));
	return value;
}

string Galil::message(int timeout_ms)
{
	/*
	*  From GCL documentation, http://www.galil.com/sw/pub/all/doc/galiltools/html/library.html#message
	* "If a zero timeout is specified, no errors will be thrown; message() will simply return the waiting queue (even if it is empty, ""). 
	* A -1 timeout will cause message() to block until a message is received."
	*/
	GReturn rc = G_NO_ERROR;
	short t = 5000; //nominal timeout, to be used if -1 is specified
	if (timeout_ms >= 0)
		t = (short) timeout_ms;
	
	ec(GTimeout(d->g, t));
		
	do
	{
			rc = GMessage(d->g, d->tbuf, sizeof(d->tbuf));
	} while(timeout_ms == -1 && rc == G_TIMEOUT);
	
	ec(GTimeout(d->g, -1));
	
	if (rc == G_GCLIB_NON_BLOCKING_READ_EMPTY)
		return "";

	ec(rc); //check the other possible return codes
	return string(d->tbuf);
	
}

int Galil::interrupt(int timeout_ms)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	GStatus status;
	ec(GInterrupt(d->g, &status));
	ec(GTimeout(d->g, -1));
	return (int)status;
}


string Galil::programUpload()
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GProgramUpload(d->g, d->tbuf, sizeof(d->tbuf)));
	ec(GTimeout(d->g, -1));
	return string(d->tbuf);
}

void Galil::programDownload(const std::string& program)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GProgramDownload(d->g, program.c_str(), 0)); //no special preprocessor directives
	ec(GTimeout(d->g, -1));
}

void Galil::programUploadFile(const std::string& file)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GProgramUploadFile(d->g, file.c_str()));
	ec(GTimeout(d->g, -1));
}

void Galil::programDownloadFile(const std::string& file)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GProgramDownloadFile(d->g, file.c_str(), 0));  //no special preprocessor directives
	ec(GTimeout(d->g, -1));
}

vector<double> Galil::arrayUpload(const std::string& name)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GArrayUpload(d->g, name.c_str(), G_BOUNDS, G_BOUNDS, G_CR, d->tbuf, sizeof(d->tbuf)));
	ec(GTimeout(d->g, -1));

	vector<double> vals;
	int len = strlen(d->tbuf); //gclib null terminates array buf
	char* start = d->tbuf; //start hold pointer to begining of string to atof
	for (int i = 0; i < len; i++)
	{
		if (d->tbuf[i] == '\r')
		{
			vals.push_back(atof(start));
			start = d->tbuf + i;
		}
	}
	vals.push_back(atof(start)); //last number still left in tbuf
	return vals;
}

void Galil::arrayDownload(const std::vector<double>& array, const std::string& name)
{
	//GArrayDownload requires a cstring containing the data
	string array_str;
	for (size_t i = 0; i < array.size(); i++)
	{
		sprintf(d->tbuf, "%0.4f\r", array[i]);
		array_str.append(d->tbuf);
	}
	ec(GTimeout(d->g, (short)timeout_ms));

	//Galil::arrayDownload auto-dimensions the array table
	string command = "DA " + name + "[]";
	ec(GCmd(d->g, command.c_str())); //Deallocate array
	command = "DM " + name + "[" + to_string(array.size()) + "]";
	ec(GCmd(d->g, command.c_str())); //Allocate array with correct dimension

	ec(GArrayDownload(d->g, name.c_str(), G_BOUNDS, G_BOUNDS, array_str.c_str()));
	ec(GTimeout(d->g, -1));
}

void Galil::arrayUploadFile(const std::string& file, const std::string& names)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GArrayUploadFile(d->g, file.c_str(), names.c_str()));
	ec(GTimeout(d->g, -1));
}

void Galil::arrayDownloadFile(const std::string& file)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GArrayDownloadFile(d->g, file.c_str()));
	ec(GTimeout(d->g, -1));
}

void Galil::firmwareDownloadFile(const std::string& file)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GFirmwareDownload(d->g, file.c_str()));
	ec(GTimeout(d->g, -1));
}

int Galil::write(const std::string& bytes)
{
	ec(GTimeout(d->g, (short)timeout_ms));
	ec(GWrite(d->g, bytes.data(), bytes.length()));
	ec(GTimeout(d->g, -1));
	return bytes.length();
}

string Galil::read()
{
	ec(GTimeout(d->g, (short)timeout_ms));
	GSize bytes_read;
	ec(GRead(d->g, d->tbuf, sizeof(d->tbuf), &bytes_read));
	ec(GTimeout(d->g, -1));
	return string(d->tbuf, bytes_read); //data is not null-terminatated, so construct string "from buffer"
}

