/*! \file x_programs.cpp
*
* Example GProgramDownload() and GProgramUpload() usage.
*
*/

#include "x_examples.h"

int x_programs(GCon g)
{
	cout << "\n************************************************************************\n";
	cout << "Example GProgramDownload() and GProgramUpload() usage\n";
	cout << "************************************************************************\n";
	GReturn rc = G_NO_ERROR; //return code
	char buf[1024]; //traffic buffer

	//-------------------------------------------------------------------------
	//capping compression
	string program = "#A;i=0;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;i=i+1;EN";
	//Program line above is too large to fit on any Galil controller, however it can easily fit if broken up with level 4 compression.
	//the value of i at the end of code execution is 20.

	if ((rc = GProgramDownload(g, program.c_str(), "--max 3"))
		== G_UNABLE_TO_COMPRESS_PROGRAM_TO_FIT)
		cout << "GProgramDownload() correctly errored. Can't fit with level 3 compression\n";
	else
	{
		cout << "Unexpected GProgramDownload() behaviour\n";
		return GALIL_EXAMPLE_ERROR;
	}

	//-------------------------------------------------------------------------
	//full compression
	if ((rc = GProgramDownload(g, program.c_str(), "--max 4"))
		== G_NO_ERROR)
		cout << "Program Downloaded with compression level 4\n";
	else
	{
		cout << "Unexpected GProgramDownload() behaviour\n";
		return GALIL_EXAMPLE_ERROR;
	}

	x_e(GProgramDownload(g, program.c_str(), 0)); //null for preprocessor equivalent to GProgramDownload(g, program.c_str(), "--compress 4")

	//-------------------------------------------------------------------------
	//program upload
	cout << "Uploading program:\n";
	x_e(GProgramUpload(g, buf, sizeof(buf)));
	cout << buf;

	//-------------------------------------------------------------------------
	//program upload file
	x_e(GProgramUploadFile(g, "temp.dmc"));
	
	//-------------------------------------------------------------------------
	//program download file
	x_e(GProgramDownload(g, "", 0)); //erase program
	x_e(GProgramDownloadFile(g, "temp.dmc", 0));

	//-------------------------------------------------------------------------
	//executing/verifying code
	GCmd(g, "XQ"); //execute the code
	GSleep(10); //wait a brief interval for the code to complete, TB could be checked with a mask too.
	int val;
	GCmdI(g, "i=?", &val);
	if (val == 20)
		cout << "\n\nProgram executed as expected";
	else
	{
		cout << "\n\nUnexpected i value " << val << '\n';
		return GALIL_EXAMPLE_ERROR;
	}

	return GALIL_EXAMPLE_OK;
}