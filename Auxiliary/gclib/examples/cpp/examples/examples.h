/** @defgroup cpp_examples C++ examples */

/** @addtogroup cpp_examples
  * @brief Files included in the C++ examples
  *
  * @{
  */
  
/*! \file examples.h
*
* Header file for Galil gclib example projects.
*/
#ifndef examples_h
#define examples_h

#define _CRT_SECURE_NO_WARNINGS //use traditional C calls like sprintf()

#include "gclib.h"
#include "gclibo.h"

#include <iostream> //std::cout
#include <cstdio> //std::getchar

#define GALIL_EXAMPLE_OK 0//return code for correct code execution
#define GALIL_EXAMPLE_ERROR -100 //return code for error in example code


//! A trivial, C++ style return code check used in Galil's examples and demos.
/*!
*  Throws GReturn if return value is not `G_NO_ERROR`.
*  See Commands_Example.cpp for example usage and catch() handler.
*/
inline void e(GReturn rc)
{
	if (rc != G_NO_ERROR)
		throw rc;
}

//! An example of error handling and debugging information.
inline void error(GCon g, GReturn rc)
{
    char buf[G_SMALL_BUFFER];
    GError(rc, buf, G_SMALL_BUFFER); //Get Error Information
    std::cout << buf << '\n'; 
	if (g)
	{
	    GSize size = sizeof(buf);
	    GUtility(g, G_UTIL_ERROR_CONTEXT, buf, &size);
	
	    if (buf[0])
    		std::cout << buf << '\n'; //further context
    		
		if ((rc == G_BAD_RESPONSE_QUESTION_MARK) 
		    && (GCommand(g, "TC1", buf, G_SMALL_BUFFER, 0) == G_NO_ERROR))
		{
		    std::cout << buf << '\n'; //Error code from controller
		}
	}
}

//! Pauses console apps for a user key stroke.
inline int pause()
{

    std::cout << "Enter any key to exit..." << std::endl;
    return std::getchar();

}

//! Puts controller into Position Tracking Mode and accepts user-entered positions.
/*!
*  \param g Connection's handle.
*  \param speed Optional speed of the controller in Position Tracking Mode.  Default value of 5000.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See position_tracking_example.cpp for an example.
*/
GReturn position_tracking(GCon g, int speed = 5000);

//! Puts controller into Jog Mode and accepts user input to adjust the speed.
/*!
*  \param g Connection's handle.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See jog_example.cpp for an example.
*/
GReturn jog(GCon g);

//! Puts controller into Vector Mode and accepts a file defining vector points.
/*!
*  \param g Connection's handle.
*  \param file A Path to a file that defines vector commands.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See vector_example.cpp for an example.
*/
GReturn vector(GCon g, char* file);

//! Assigns controller an IP Adress given a serial number and a 1 byte address.
/*!
*  \param serial_num Serial Number of the controller.
*  \param address A 1 byte address that defines the last byte of the IP Address.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See ip_assigner_example.cpp for an example.
*/
GReturn ip_assigner(char* serial_num, int address);

//! Demonstrates various uses of GCommand() and GUtility().
/*!
*  \param g Connection's handle.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See commands_example.cpp for an example.
*/
GReturn commands(GCon g);

//! Uses interrupts to track when the motion of controller is completed.
/*!
*  \param g Connection's handle.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See motion_complete_example.cpp for an example.
*/
GReturn motion_complete(GCon g);

//! Demonstrates how to receive messages from the controller and detect differences in Trace and crashed code.
/*!
*  \param g Connection's handle.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See message_example.cpp for an example.
*/
GReturn message(GCon g);

//! Record user's training and saves to a text file.
/*!
*  \param g Connection's handle.
*  \param fileA A Path to a text file where training for Axis A will be recorded.
*  \param fileB A Path to a text file where training for Axis B will be recorded.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See record_position_example.cpp for an example.
*/
GReturn record_position(GCon g, char* fileA, char* fileB);

//! Record user's training and plays back training through contour mode.
/*!
*  \param g Connection's handle.
*  \param fileA A Path to a text file where training for Axis A will be recorded.
*  \param fileB A Path to a text file where training for Axis B will be recorded.
*
*  \return The success status or error code of the function. See gclib_errors.h for possible values.
*
*  See contour_example.cpp for an example.
*/
GReturn contour(GCon g, char* fileA, char* fileB);

//! Publishes local gcaps server to the network 
/*!
*	\param Name to publish server under.
*
*	\return The success status or error code of the function.  See gclib_errors.h for possible values.
*
*	See remote_server_example for an example.
*/
GReturn remote_server(const char* server_name);

//! Lists available remote servers and allows connection to remote server.
/*!
*	\return The success status or error code of the function.See gclib_errors.h for possible values.
*
*	See remote_client_example for an example.
*/
GReturn remote_client();

#endif //examples_h


/** @}*/
