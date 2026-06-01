/*! \file x_examples.h
*
* Header file for Galil gclib examples. All example functions start with the letter *e*. 
* Example function calls demonstrating the use of library functions start with *x_*.
*
*/
#ifndef I_7B02A40E_869B_4650_A5A8_859F0A6E3325
#define I_7B02A40E_869B_4650_A5A8_859F0A6E3325

#define _CRT_SECURE_NO_WARNINGS //use traditional C calls like sprintf()

#include <iostream> //std::cout
#include <string> //to_string, string, etc.
#include <cstdio> //sprintf, etc.
#include <cstring> //strlen, etc.

#include "gclib.h"
#include "gclibo.h"

#define GALIL_EXAMPLE_OK G_NO_ERROR //return code for correct code execution
#define GALIL_EXAMPLE_ERROR -100 //return code for error in example code

using namespace std;

//! A trivial, C++ style return code check used in Galil's examples and demos.
/*!
*  Throws GReturn if return value is not `G_NO_ERROR`.
*  See x_examples.cpp for example usage and catch() handler.
*/
inline void x_e(GReturn rc)
{
	if (rc != G_NO_ERROR) 
		throw rc;
}

//! Example GCommand() usage.
/*!
*  Examples of GCommand() and open-source wrappers like GCmd().
*/
int x_gcommand(GCon g);

//! Example GMotionComplete() usage.
/*!
*  \warning This function will attempt to move motors.
*/
int x_gmotioncomplete(GCon g);

//! Example GRead() and GWrite() usage.
/*!
*  Demonstrates usage of Read/Write operations.
*/
int x_gread_gwrite(GCon g);

//! Example GProgramDownload() and GProgramUpload() usage.
/*!
*  Demonstrates program download and upload including compression.
*/
int x_programs(GCon g);

//! Example GArrayDownload() and GArrayUpload() usage.
/*!
*  Demonstrates array download and upload.
*/
int x_arrays(GCon g);

//! Example GRecord() usage.
/*!
*  Demonstrates QR and DR data records with struct names and pointer arithmetic.
*  Also demonstrates a MotionComplete function with data records.
*
*  \warning This function will attempt to move motors.
*/
int x_grecord(GCon g);

//! Example of MotionComplete with data records.
/*!
*  Uses axis status in data record to determine when motion has completed.
*  \param g Connection's handle.
*  \param axes Mult-axis mask for determining motion complete. ABCDEFGH valid.
*
*/
int x_dr_motioncomplete(GCon g, GCStringIn axes);

//! Example GMessage() usage.
/*!
*  Demonstrates retrieving messages.
*/
int x_gmessage(GCon g);

//! Example GInterrupt() usage.
/*!
*  Demonstrates retrieving status bytes via UI, and a MotionComplete function with interrupts.
*
*  \warning This function will attempt to move motors.
*/
int x_ginterrupt(GCon g);

//! Example of MotionComplete with interrupts.
/*!
*  Uses motion complete status bytes to determine when motion has completed.
*  \param g Connection's handle.
*  \param axes Mult-axis mask for determining motion complete. ABCDEFGH valid. Axes must be in motion when function is called.
*
*/
int x_ei_motioncomplete(GCon g, GCStringIn axes); 

//! Examples of using non-blocking operation of GRecord(), GInterrupt(), and GMessage().
/*!
*  Typical usage of the asynchronous data streams is to call the function with a given timeout.
*  The function will then block until the desired data is received of the timeout occurs.
*  However, to check for available messages, ascynchronous records, or interrupts, the user
*  can set a timeout of zero and the functions will return waiting data.
*  See the source of this example for more detail.
*/
int x_nonblocking(GCon g);


#endif //I_7B02A40E_869B_4650_A5A8_859F0A6E3325


