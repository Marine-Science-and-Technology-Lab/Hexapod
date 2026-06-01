/*! \file x_arrays.cpp
*
* Example GArrayDownload() and GArrayUpload() usage.
*
*/

#include "x_examples.h"

int x_arrays(GCon g)
{
	cout << "\n************************************************************************\n";
	cout << "Example GArrayDownload(), GArrayUploadFile()\n";
	cout << "GArrayDownloadFile(), and GArrayUpload usage\n";
	cout << "************************************************************************\n";

	char buf[1024]; //traffic buffer

	//-------------------------------------------------------------------------
	//Preparing the array table
	x_e(GCmd(g, "DA *[]")); //deallocate all arrays
	x_e(GCmd(g, "DM A[10]")); //allocate A[] array
	
	//-------------------------------------------------------------------------
	//download full array
	x_e(GArrayDownload(g, "A", G_BOUNDS, G_BOUNDS, "2,4,6,8,10,12,14,16,18,20"));

	//-------------------------------------------------------------------------
	//upload full array
	x_e(GArrayUpload(g, "A", G_BOUNDS, G_BOUNDS, G_COMMA, buf, sizeof(buf)));
	cout << buf << "\n\n";

	//-------------------------------------------------------------------------
	//download subset
	x_e(GArrayDownload(g, "A", 1, 3, "1,3,5"));
	
	//-------------------------------------------------------------------------
	//upload full array
	x_e(GArrayUpload(g, "A", G_BOUNDS, G_BOUNDS, G_COMMA, buf, sizeof(buf)));
	cout << buf << "\n\n";

	//-------------------------------------------------------------------------
	//upload subset
	x_e(GArrayUpload(g, "A", 2, 4, G_COMMA, buf, sizeof(buf))); 
	cout << buf << '\n';
	
	//-------------------------------------------------------------------------
	//upload to file
	x_e(GArrayUploadFile(g, "array.csv", 0));

	//-------------------------------------------------------------------------
	//Download from file
	x_e(GArrayDownloadFile(g, "array.csv"));

	//-------------------------------------------------------------------------
	//upload full array
	x_e(GArrayUpload(g, "A", G_BOUNDS, G_BOUNDS, G_COMMA, buf, sizeof(buf)));
	cout << buf << "\n\n";

	return GALIL_EXAMPLE_OK;
}