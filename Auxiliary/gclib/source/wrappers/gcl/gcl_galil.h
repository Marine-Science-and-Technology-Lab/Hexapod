/*! \gcl_galil.h
*
*/
#ifndef I_34E90AC4_EF44_4CB5_AF47_B9D334EE5FB7
#define I_34E90AC4_EF44_4CB5_AF47_B9D334EE5FB7

#include "gclibo.h" //gclib and gclibo
#include <unordered_map> //used for data record features
#include <sstream> //format source keys, string stream
#include <iomanip> //format source keys
#include <algorithm> //sort sources
#define MAKEDLL //Galil.h requires this for proper export declarations
#include "Galil.h" //gcl header
#define TRAFFICBUF 4096 //Memory used for buffering controller traffic. If program/array upload is getting truncated, make this larger.


struct Source //each data record source key (e.g. "_RPA") maps to one of these, each of which describes the position and width of the variable within the binary data record
{
	int byte; //byte offset within binary data record
	std::string type; //"SB", "UB", "SW", "UW", "SL", "UL".  Specifies width within binary data record and signed/unsigned.
	int bit; //-1 if not bit field (e.g. RPA).  >= 0 if bit field (e.g. _MOA)
	std::string units; //e.g. "counts"
	std::string description; //e.g. "analog input 1"
	double scale; //e.g. 32768, scale factor:  most sources are 1 except TV, TT, @AN, @AO etc.
	double offset; //needed for analog inputs and outputs

	Source(int byte = 0, std::string type = "Ux", int bit = -1, std::string units = "", std::string description = "", double scale = 1, double offset = 0) :
		byte(byte), type(type), bit(bit), units(units), description(description), scale(scale), offset(offset)
	{ /*ctor just initializes values*/ }
};

// Use the gcl private class name to hold useful data for the wrapper
class GalilPrivate
{
public:
	GalilPrivate(Galil* galil_ptr, GCon gclib_handle) 
		: q(galil_ptr), g(gclib_handle)
	{ /*ctor just initializes values*/ }

	GCon g; //gclib handle for this class
	char tbuf[TRAFFICBUF]; //traffic buffer shared by member functions of Galil
	std::unordered_map<std::string, Source> map; //data structure for data record calls
	void InitializeDataRecord(); //called in Galil() ctor to create the data record map
	
private:
	Galil* q; //pointer to the Galil class so we can call Command, etc.

	//functions for initializing the data record for various products
	void Init1806(int axes);
	void Init1800(int axes, bool dmc1802);
	void Init4000(int axes); //41x3 and 50000 too
	void Init30010(bool dmc31010);
	void Init2103(int axes);
	void InitRio(bool rio3); //rio3 indicates if this connection is to an RIO-47300 which has a different length than other RIOs
	
	void InitRio3_24Ex(); //extended I/O option for RIO-47300
	void InitRioSer(bool rio3); //Serial encoder (quad/SSI/BISS) option for RIO

	//Helper/convenience functions
	std::string ax(std::string prefix, int axis, std::string suffix); // Creates an axis string from a prefix, axis number (0-7), and suffix.
	void input_bits(int byte, int num); //Creates 8 digital input sources from an offset and a starting input number.
	void output_bits(int byte, int num); //Creates 8 digital ouput sources from an offset and a starting output number.
	void aq_analog(int byte, int input_num); //polls AQ for input_num and creates an analog input source with the proper AQ scalar
	void dq_analog(int byte, int input_num); //polls DQ for input_num and creates an analog output source with the proper DQ scalar
};

void ec(GReturn rc); // Error Check. Takes a gclib error code and throws a GCL-like error string if there's a gclib error



#endif //I_34E90AC4_EF44_4CB5_AF47_B9D334EE5FB7
