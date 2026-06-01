/*! \file x_gmessage.cpp
*
* Example GMessage() usage.
*
*/

#include "x_examples.h"

int x_gmessage(GCon g)
{
	cout << "\n************************************************************************\n";
	cout << "Example GMessage() usage\n";
	cout << "************************************************************************\n";
	
	//-------------------------------------------------------------------------
	char buf[1024]; //traffic buffer
	x_e(GProgramDownload(g, "i=0\r#A\rMGi\ri=i+1\rWT100\rJP#A,i<10\rEN", 0));
	x_e(GCmd(g, "XQ"));
	
	int rc;
	GTimeout(g, 200);//adjust timeout
	int i = 0;
	while ((rc = GMessage(g, buf, sizeof(buf))) == G_NO_ERROR)
	{
		cout << buf;
		if (strstr(buf, ".") != 0) //each MG has a decimal point
			i++; //count it
	}
	GTimeout(g, G_USE_INITIAL_TIMEOUT); //restore timeout
	if (i == 10)
		return GALIL_EXAMPLE_OK;
	else
	{
		cout << "Expected 10 messages\n";
		return GALIL_EXAMPLE_ERROR;
	}
}