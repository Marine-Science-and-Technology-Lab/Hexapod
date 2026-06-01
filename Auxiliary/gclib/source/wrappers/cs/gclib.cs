/** @defgroup cs_api C#/VB API */


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices; //dll import
using System.IO; //file.exists

using UB = System.Byte;  //unsigned byte
using UW = System.UInt16; //unsigned word
using SW = System.Int16;  //signed word
using SL = System.Int32;  //signed long
using UL = System.UInt32; //unsigned long

#if x86 //defined in "Conditional compilation symbols" of Project Properties
using GReturn = System.Int32;
using GCon = System.IntPtr;
using GSize = System.UInt32;
using GOption = System.Int32;
using GCStringOut = System.Text.StringBuilder;
using GCStringIn = System.String;
using GBufOut = System.Text.StringBuilder;
using GBufIn = System.String;
using GStatus = System.Byte;
// IMPORTANT! Be sure that the paths below are correct 
public static class LibraryPath
{
    public const string GclibDllPath_ = "C:\\Program Files (x86)\\Galil\\gclib\\dll\\x86\\gclib.dll";
    public const string GcliboDllPath_ = "C:\\Program Files (x86)\\Galil\\gclib\\dll\\x86\\gclibO.dll";
}

#elif x64
using GReturn = System.Int32;
using GCon = System.IntPtr;
using GSize = System.UInt32;
using GOption = System.Int32;
using GCStringOut = System.Text.StringBuilder;
using GCStringIn = System.String;
using GBufOut = System.Text.StringBuilder;
using GBufIn = System.String;
using GStatus = System.Byte;
// IMPORTANT! Be sure that the paths below are correct 
public static class LibraryPath
{
    public const string GclibDllPath_ = "C:\\Program Files (x86)\\Galil\\gclib\\dll\\x64\\gclib.dll";
    public const string GcliboDllPath_ = "C:\\Program Files (x86)\\Galil\\gclib\\dll\\x64\\gclibo.dll";
}

#endif

/** @addtogroup cs_api
  * @brief Files included in the C#/VB API
  *
  * @{
  */
/// <summary>
/// Provides a class that binds to gclib's unmanaged dll. Wraps each call and provides a more user-friendly interface for use in C#.
/// </summary>
/// <remarks>
/// The Gclib class assumes the default installation of gclib, "C:\Program Files (x86)\Galil\gclib\". 
/// If the dlls are elsewhere, change the path strings GclibDllPath_, and GcliboDllPath_.
/// </remarks>
public class gclib
{
    #region "C# wrappers of gclib C calls"

    #region "Private properties"
    private const int BufferSize_ = 500000; //size of "char *" buffer. Big enough to fit entire 4000 program via UL/LS, or 24000 elements of array data.
    private GCStringOut Buffer_ = new System.Text.StringBuilder(BufferSize_); //used to pass a "char *" to gclib.
    private byte[] ByteArray_ = new byte[512]; //byte array to hold data record and response to GRead
    private GCon ConnectionHandle_; //keep track of the gclib connection handle.
    private bool ConnectionStatus_ = false; //keep track of the status of gclib's connection
    #endregion

    
    /// <summary>
    /// Constructor of the gclib wrapper class.
    /// </summary>
    /// <remarks>Checks to ensure gclib dlls are in the correct location.</remarks>
    /// <exception cref="System.Exception">Will throw an exception if either dll isn't found.</exception>
    public gclib()
    {
        if (!File.Exists(LibraryPath.GclibDllPath_))
            throw new System.Exception("Could not find gclib dll at " + LibraryPath.GclibDllPath_);

        if (!File.Exists(LibraryPath.GcliboDllPath_))
            throw new System.Exception("Could not find gclibo dll at " + LibraryPath.GcliboDllPath_);

    }
    
    /// <summary>
    /// Return a string array of available connection addresses.
    /// </summary>
    /// <returns>String array containing all available Galil Ethernet controllers, PCI controllers, and COM ports.</returns>
    /// <remarks>Wrapper around gclib GAddresses(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a6a6114683ed5749519b64f19512c24d6
    /// An empty array is returned on error.</remarks>
    public string[] GAddresses()
    {
        GReturn rc = DllGAddresses(Buffer_, BufferSize_);
        if (rc == G_NO_ERROR)
        {
            char[] delimiters = new char[] { '\r', '\n' };
            return Buffer_.ToString().Split(delimiters, System.StringSplitOptions.RemoveEmptyEntries);
        }
        else
            return new string[0];

    }

    /// <summary>
    /// Downloads array data to a pre-dimensioned array in the controller's array table. 
    /// </summary>
    /// <param name="array_name">String containing the name of the array to download. Must match the array name used in DM.</param>
    /// <param name="data">A list of doubles, to be downloaded.</param>
    /// <param name="first">The first element of the array for sub-array downloads.</param>
    /// <param name="last">The last element of the array for sub-array downloads.</param>
    /// <remarks>Wrapper around gclib GArrayDownload(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a6ea5ae6d167675e4c27ccfaf2f240f8a 
    /// The array must already exist on the controller, see DM and LA.</remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GArrayDownload(string array_name, ref List<double> data, Int16 first = -1, Int16 last = -1)
    {
        System.Text.StringBuilder ArrayData = new System.Text.StringBuilder(BufferSize_); //for converting to ASCII
        int len = data.Count();
        for (int i = 0; i <= len - 1; i++)
        {
            ArrayData.Append(data[i].ToString("F4")); //format to fixed point
            if (i < len - 1)
            {
                ArrayData.Append(","); //delimiter
            }
        }
        GReturn rc = DllGArrayDownload(ConnectionHandle_, array_name, first, last, ArrayData.ToString());
        if (!(rc == G_NO_ERROR))
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Allows downloading of a program array file to the controller.
    /// </summary>
    /// <param name="Path">The full filepath of the array csv file.</param>
    /// <remarks>Wrapper around gclib GArrayDownload(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a14b448ab8c7e6cf495865af301be398e
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GArrayDownloadFile(string Path)
    {
        GReturn rc = DllGArrayDownloadFile(ConnectionHandle_, Path);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Uploads array data from the controller's array table. 
    /// </summary>
    /// <param name="array_name">String containing the name of the array to upload.</param>
    /// <param name="first">The first element of the array for sub-array uploads.</param>
    /// <param name="last">The last element of the array for sub-array uploads.</param>
    /// <returns>The desired array as a list of doubles.</returns>
    /// <remarks>Wrapper around gclib GArrayUpload(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#af215806ec26ba06ed3f174ebeeafa7a7
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public List<double> GArrayUpload(string array_name, Int16 first = -1, Int16 last = -1)
    {
        List<double> array = new List<double>();
        GReturn rc = DllGArrayUpload(ConnectionHandle_, array_name, first, last, 1, Buffer_, BufferSize_);
        //1 = comma delim
        if (!(rc == G_NO_ERROR))
        {
            throw new System.Exception(GError(rc));
        }
        char[] delimiters = new char[] { ',' };

        string[] tokens = Buffer_.ToString().Split(delimiters, System.StringSplitOptions.RemoveEmptyEntries);
        double value = 0;
        foreach (string s in tokens)
        {
            if (!double.TryParse(s, out value))
            {
                throw new System.Exception("Could not parse " + s + " into double");
            }
            array.Add(value);
        }
        return array;
    }

    /// <summary>
    /// Allows uploading of a program array file from the controller to an array CSV file.
    /// </summary>
    /// <param name="Path">The full filepath of the array csv file to save.</param>
    /// <param name="Names">A space separated list of the array names to upload. A null string uploads all arrays in the array table (LA).</param>
    /// <remarks>Wrapper around gclib GArrayUpload().
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#af215806ec26ba06ed3f174ebeeafa7a7
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GArrayUploadFile(string Path, string Names)
    {
        GReturn rc = DllGArrayUploadFile(ConnectionHandle_, Path, Names);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Assigns IP address over the Ethernet to a controller at a given MAC address. 
    /// </summary>
    /// <param name="ip">The ip address to assign. The hardware should not yet have an IP address. </param>
    /// <param name="mac">The MAC address of the hardware.</param>
    /// <remarks>Wrapper around gclib GAssign(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#acc996b7c22cfed8e5573d096ef1ab759
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GAssign(string ip, string mac)
    {
        GReturn rc = DllGAssign(ip, mac);
        if (!(rc == G_NO_ERROR))
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Used to close a connection to Galil hardware.
    /// </summary>
    /// <remarks>Wrapper around gclib GClose(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a24a437bcde9637b0db4b94176563a052
    /// Be sure to call GClose() whenever a connection is finished.</remarks>
    public void GClose()
    {
        if(ConnectionStatus_)
            DllGClose(ConnectionHandle_);

        ConnectionStatus_ = false;
    }

    /// <summary>
    /// Used for command-and-response transactions.
    /// </summary>
    /// <param name="Command">The command to send to the controller. Do not append a carriage return. Use only ASCII-based commmands.</param>
    /// <param name="Trim">If true, the response will be trimmed of the trailing colon and any leading or trailing whitespace.</param>
    /// <returns>The command's response.</returns>
    /// <remarks>Wrapper around gclib GCommand(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a5ac031e76efc965affdd73a1bec084a8
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public string GCommand(string Command, bool Trim = true)
    {        
        GSize bytes_read = 0;
        GReturn rc = DllGCommand(ConnectionHandle_, Command, Buffer_, BufferSize_, ref bytes_read);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
        if (Trim)
        {
            string r = Buffer_.ToString();
            if (r[r.Count() - 1] == ':')
            {
                r = r.Substring(0, r.Count() - 1);
            }
            return r.Trim();
        }
        else
        {
            return Buffer_.ToString();
        }
    }

    /// <summary>
    /// Used for command-and-response transactions.
    /// </summary>
    /// <param name="Command">The command to send to the controller. Do not append a carriage return. Use only ASCII-based commmands.</param>
    /// <returns>The command's response parsed as an integer.</returns>
    /// <remarks>Wrapper around gclib GCmdI(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a5ac031e76efc965affdd73a1bec084a8
    /// </remarks>
    public Int16 GCmdI(string Command)
    {
        return Convert.ToInt16(Convert.ToDouble(GCommand(Command)));
    }

    /// <summary>
    /// Used for command-and-response transactions.
    /// </summary>
    /// <param name="Command">The command to send to the controller. Do not append a carriage return. Use only ASCII-based commmands.</param>
    /// <returns>The command's response parsed as a double.</returns>
    /// <remarks>Wrapper around gclib GCmdD(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a5ac031e76efc965affdd73a1bec084a8
    /// </remarks>
    public double GCmdD(string Command)
    {
        return Convert.ToDouble(GCommand(Command));
    }

    /// <summary>
    /// Provides a human-readable error message from a gclib error code.
    /// </summary>
    /// <param name="ErrorCode">The gclib error code, as returned from a call to the gclib.</param>
    /// <returns>Error message string.</returns>
    /// <remarks>
    /// Wrapper around gclib GError(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#afef1bed615bd72134f3df6d3a5723cba
    ///  This function is private, all public calls that throw errors use this command for setting the exception message.
    /// </remarks>
    private string GError(GReturn ErrorCode)
    {
        DllGError(ErrorCode, Buffer_, BufferSize_);
        return ErrorCode.ToString() + " " + Buffer_.ToString() + "\n";
    }

    /// <summary>
    /// Upgrade firmware. 
    /// </summary>
    /// <param name="filepath ">The full filepath of the firmware hex file.</param>
    /// <remarks>Wrapper around gclib GFirmwareDownload(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a1878a2285ff17897fa4fb20182ba6fdf
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GFirmwareDownload(string filepath)
    {
        GReturn rc = DllGFirmwareDownload(ConnectionHandle_, filepath);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>Provides a useful connection string.</summary>
    /// <remarks>Wrapper around gclib GInfo(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a08abfcff8a1a85a01987859473167518
    /// </remarks>
    /// <returns>String containing connection information, e.g. "192.168.0.43, DMC4020 Rev 1.2c, 291". A null string indicates an error was returned from the library.</returns>
    public string GInfo()
    {
        GReturn rc = DllGInfo(ConnectionHandle_, Buffer_, BufferSize_);
        if (rc == G_NO_ERROR)
        {
            return Buffer_.ToString();
        }
        else
        {
            return "";
        }
    }

    /// <summary>
    /// Provides access to PCI and UDP interrupts from the controller. 
    /// </summary>
    /// <returns>The status byte from the controller. Zero will be returned if a status byte is not read.</returns>
    /// <remarks>Wrapper around gclib GInterrupt(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a5bcf802404a96343e7593d247b67f132
    /// -s ALL or -s EI must be specified in the address argument of GOpen() to receive interrupts.</remarks>
    public byte GInterrupt()
    {
        byte StatusByte = 0;
        GReturn rc = DllGInterrupt(ConnectionHandle_, ref StatusByte);
        if (rc == G_NO_ERROR)
        {
            return StatusByte;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>
    /// Provides a list of all Galil controllers requesting IP addresses via BOOT-P or DHCP. 
    /// </summary>
    /// <returns>Each line of the returned data will be of the form "model, serial_number, mac". </returns>
    /// <remarks>Wrapper around gclib GIpRequests(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a0afb4c82642a4ef86f997c39a5518952
    /// An empty array is returned on error.
    /// Call will take roughly 5 seconds to return.</remarks>
    public string[] GIpRequests()
    {
        GReturn rc = DllGIpRequests(Buffer_, BufferSize_);
        if (rc == G_NO_ERROR)
        {
            char[] delimiters = new char[] { '\r', '\n' };
            return Buffer_.ToString().Split(delimiters, System.StringSplitOptions.RemoveEmptyEntries);
        }
        else
            return new string[0];

    }

    /// <summary>
    /// Provides access to unsolicited messages.
    /// </summary>
    /// <returns>String containing all messages received by controller.</returns>
    /// <remarks>Wrapper around gclib GMessage(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#aabc5eaa09ddeca55ab8ee048b916cbcd
    ///An empty string is returned on error.
    /// -s ALL or -s MG must be specified in the address argument of GOpen() to receive messages.</remarks>
    public string GMessage()
    {
        GReturn rc = DllGMessage(ConnectionHandle_, Buffer_, BufferSize_);
        if (rc == G_NO_ERROR)
        {
            return Buffer_.ToString();
        }
        else
        {
            return "";
        }
    }

    /// <summary>
    /// Blocking call that returns once all axes specified have completed their motion. 
    /// </summary>
    /// <param name="axes">A string containing a multiple-axes mask. Every character in the string should be a valid argument to MG_BGm, i.e. XYZWABCDEFGHST.</param>
    /// <remarks>Wrapper around gclib GMotionComplete(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a19c220879442987970706444197f397a
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GMotionComplete(string axes)
    {
        GReturn rc = DllGMotionComplete(ConnectionHandle_, axes);
        if (!(rc == G_NO_ERROR))
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Used to open a connection to Galil hardware.
    /// </summary>
    /// <param name="address">Address string including any connection switches. See gclib documentation for GOpen().</param>
    /// <remarks>Wrapper around gclib GOpen(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#aef4aec8a85630eed029b7a46aea7db54
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GOpen(string address)
    {
        GReturn rc = DllGOpen(address, ref ConnectionHandle_);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
        else
            ConnectionStatus_ = true;
    }

    /// <summary>
    /// Allows downloading of a DMC program from a string buffer.
    /// </summary>
    /// <param name="program">The program to download.</param>
    /// <param name="preprocessor">Preprocessor directives. Use nullstring for none.</param>
    /// <remarks>Wrapper around gclib GProgramDownload(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#acafe19b2dd0537ff458e3c8afe3acfeb
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GProgramDownload(string program, string preprocessor = "")
    {
        GReturn rc = DllGProgramDownload(ConnectionHandle_, program, preprocessor);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Allows downloading of a DMC program from file.
    /// </summary>
    /// <param name="file_path">The full filepath of the DMC file.</param>
    /// <param name="preprocessor">Preprocessor directives. Use nullstring for none.</param>
    /// <remarks>Wrapper around gclib GProgramDownloadFile(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a8e44e2e321df9e7b8c538bf2d640633f
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GProgramDownloadFile(string file_path, string preprocessor = "")
    {
        GReturn rc = DllGProgramDownloadFile(ConnectionHandle_, file_path, preprocessor);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Allows uploading of a DMC program to a string.
    /// </summary>
    /// <remarks>Wrapper around gclib GProgramUpload(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a80a653ce387a2bd16bde2793c6de77e9
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public string GProgramUpload()
    {
        GReturn rc = DllGProgramUpload(ConnectionHandle_, Buffer_, BufferSize_);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
        else
        {
            return Buffer_.ToString();
        }
    }

    /// <summary>
    /// Allows uploading of a DMC program to a file.
    /// </summary>
    /// <param name="file_path">The full filepath of the DMC file to save.</param>
    /// <remarks>Wrapper around gclib GProgramUploadFile(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a38c5565afc11762fa19d37fbaa3c9aa3
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GProgramUploadFile(string file_path)
    {
        GReturn rc = DllGProgramUploadFile(ConnectionHandle_, file_path);
        if (rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Performs a read on the connection. 
    /// </summary>
    /// <returns>String containing the read data, or a nullstring if nothing was read or an error occured.</returns>
    /// <remarks>Wrapper around gclib GRead(), 
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#adab6ec79b7e1bc7f0266684dd3434923
    /// </remarks>
    public byte[] GRead()
    {
        GSize read = 0;
        GReturn rc = DllGRead(ConnectionHandle_, ByteArray_, (uint)ByteArray_.Length, ref read);
        if (rc == G_NO_ERROR)
        {
            byte[] ReturnData = new byte[read];
            //create an array of the correct size
            for (GSize i = 0; i <= read - 1; i++)
            {
                ReturnData[i] = ByteArray_[i];
                //copy over the data
            }
            return ReturnData;
        }
        else
            return new byte[0];
    }

    /// <summary>
    /// Used for retrieving data records from the controller.
    /// </summary>
    /// <returns>A struct containing the information from the retrieved data record.</returns>
    /// <param name="async">False to user QR, True to use DR.</param>
    /// <remarks>Wrapper around gclib GRecord(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a1f39cd57dcfa55d065c972a020b1f8ee
    /// To use async, -s ALL or -s DR must be specified in the address argument of GOpen(),
    /// and the records must be started via DR or RecordRate().
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public T GRecord<T>(bool async)
            where T : struct, GDataRecord
    {
        ushort method = 0;
        if (async)
            method = 1;

        GReturn rc = DllGRecord(ConnectionHandle_, ByteArray_, method);
        if (rc != G_NO_ERROR)
            throw new System.Exception(GError(rc));

        return ByteArrayToDataRecord<T>(ByteArray_);
    }

    /// <summary>
    /// Sets the asynchronous data record to a user-specified period via DR. 
    /// </summary>
    /// <param name="period_ms">Period, in milliseconds, to set up for the asynchronous data record.</param>
    /// <remarks>Wrapper around gclib GRecordRate(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#ada86dc9d33ac961412583881963a1b8a
    /// Takes TM and product type into account and sets the DR period to the period requested by the user, if possible.</remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GRecordRate(double period_ms)
    {
        GReturn rc = DllGRecordRate(ConnectionHandle_, period_ms);
        if (!(rc == G_NO_ERROR))
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Set the timeout of communication transactions. Use -1 to set the original timeout from GOpen().
    /// </summary>
    /// <param name="timeout_ms ">New timeout in miliseconds.</param>
    /// <remarks>Wrapper around gclib GTimeout(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a179aa2d1b8e2227944cc06a7ceaf5640
    /// </remarks>
    public void GTimeout(Int16 timeout_ms)
    {
        DllGTimeout(ConnectionHandle_, timeout_ms);
    }

    /// <summary>Used to get the gclib version.</summary>
    /// <returns>The library version, e.g. "104.73.179". A null string indicates an error was returned from the library.</returns>
    /// <remarks>Wrapper around gclib GVersion(),
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a1784b39416b77af20efc98a05f8ce475
    /// </remarks>
    public string GVersion()
    {
        GReturn rc = DllGVersion(Buffer_, BufferSize_);
        if (rc == G_NO_ERROR)
        {
            return Buffer_.ToString();
        }
        else
        {
            return "";
        }
    }

    /// <summary>
    /// Performs a write on the connection. 
    /// </summary>
    /// <param name="buffer">The user's write buffer. To send a Galil command, a terminating carriage return is usually required. </param>
    /// <remarks>Wrapper around gclib GWrite(), 
    /// http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#abe28ebaecd5b3940adf4e145d40e5456 
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GWrite(string buffer)
    {
        GReturn rc = DllGWrite(ConnectionHandle_, buffer, (uint) buffer.Length);
        if (!(rc == G_NO_ERROR))
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Allows downloading of a Galil compressed backup (gcb) file to the controller.
    /// </summary>
    /// <param name="path">The full filepath of the gcb file.</param>
    /// <param name="options">A bit mask indicating which sectors of the gcb file to restore to the controller.</param>
    /// <returns>The controller information stored in the gcb file.</returns>
    /// <remarks>Wrapper around gclib GSetupDownloadFile(),
    /// 
    /// If options is specified as 0, the return string will have a number appended corresponding to a bit mask of the available gcb sectors
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public string[] GSetupDownloadFile(string path, Int32 options)
	{
		GReturn rc = DllGSetupDownloadFile(ConnectionHandle_, path, options, Buffer_, BufferSize_);

		string ret_buf = Buffer_.ToString();
        ret_buf = ret_buf.Replace("\r\n", ", ");
		
		if (options != 0)
		{
			if (rc != G_NO_ERROR)
			{
				throw new System.Exception(GError(rc));
			}
		}
		else
		{
			ret_buf += "\"options\"," + rc + "\n";
		}

        char[] delimiters = new char[] { '\n' };
        return ret_buf.ToString().Split(delimiters, System.StringSplitOptions.RemoveEmptyEntries);
	}

    /// <summary>
    /// Connects gclib to a new gcaps server
    /// </summary>
    /// <param name="server_name">Name of the server to connect.</param>
    /// <remarks>Wrapper around gclib GSetServer(),
    /// Call GSetServer("Local") to connect gclib back to local gcaps server
    /// </remarks>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GSetServer(string server_name)
    {
        GReturn rc = DllGSetServer(server_name);

        if(rc != G_NO_ERROR)
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Retrieves the name of your local gcaps server and whether or not it is currently published
    /// </summary>
    /// <returns>A string in the form "<server_name>, <isPublished>"</returns>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public string GServerStatus()
    {
        GReturn rc = DllGServerStatus(Buffer_, BufferSize_);

        if(rc == G_NO_ERROR)
            return Buffer_.ToString();
        else
            throw new System.Exception(GError(rc));
    }

    /// <summary>
    /// Retrieves a list of gcaps servers that are advertising themselves on the local network
    /// </summary>
    /// <returns>A list of available gcaps server names</returns>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public string[] GListServers()
    {
        GReturn rc = DllGListServers(Buffer_, BufferSize_);

        if(rc == G_NO_ERROR)
        {
            char[] delimiters = new char[] { '\n' };
            return Buffer_.ToString().Split(delimiters, System.StringSplitOptions.RemoveEmptyEntries);
        }
        else
        {
            throw new System.Exception(GError(rc));
        }
    }

    /// <summary>
    /// Publishes or removes local gcaps server from the network
    /// </summary>
    /// <param name="server_name">Name to publish server under.</param>
    /// <param name="publish">True=publish server, False=remove server.</param>
    /// <param name="save">Save this configuration for future server reboots.</param>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public void GPublishServer(string server_name, bool publish, bool save)
    {
        GReturn rc = DllGPublishServer(server_name, Convert.ToInt16(publish), Convert.ToInt16(save));

        if (rc != G_NO_ERROR)
            throw new System.Exception(GError(rc));
    }

    /// <summary>
    /// Returns a list of IP Addresses that currently have an open connection to your hardware.
    /// </summary>
    /// <returns>Returns a list of IP Addresses that currently have an open connection to your hardware.</returns>
    /// <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    public string[] GRemoteConnections()
    {
        GReturn rc = DllGRemoteConnections(Buffer_, BufferSize_);

        if(rc == G_NO_ERROR)
        {
            char[] delimiters = new char[] { '\n' };
            return Buffer_.ToString().Split(delimiters, System.StringSplitOptions.RemoveEmptyEntries);
        }
        else
        {
            throw new System.Exception(GError(rc));
        }
    }

    #endregion

    #region "DLL Imports"
    //Import declarations for gclib functions. Functions are private to this class and are prefixed with "Dll" to distinguish from C# functions.

    #region "Error Codes"
    /// <summary>
    /// Functions are checked for G_NO_ERROR.<
    /// </summary>
    /// <remarks>Some functions throw exceptions if G_NO_ERROR is not returned.</remarks>
    private const Int32 G_NO_ERROR = 0;
    #endregion

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GAddresses", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGAddresses(GCStringOut addresses, GSize addresses_len);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GArrayDownload", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGArrayDownload(GCon g, GCStringIn array_name, GOption first, GOption last, GCStringIn buffer);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GArrayDownloadFile", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGArrayDownloadFile(GCon g, GCStringIn path);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GArrayUpload", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGArrayUpload(GCon g, GCStringIn array_name, GOption first, GOption last, GOption delim, GCStringOut buffer, GSize bufferLength);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GArrayUploadFile", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGArrayUploadFile(GCon g, GCStringIn path, GCStringIn names);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GAssign", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGAssign(GCStringIn ip, GCStringIn mac);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GClose", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGClose(GCon g);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GCommand", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGCommand(GCon g, GCStringIn command, GCStringOut buffer, GSize bufferLength, ref GSize bytesReturned);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GError", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern void DllGError(GReturn error_code, GCStringOut errorbuf, GSize error_len);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GFirmwareDownload", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGFirmwareDownload(GCon g, GCStringIn path);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GInfo", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGInfo(GCon g, GCStringOut info, GSize infoLength);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GInterrupt", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGInterrupt(GCon g, ref GStatus status_byte);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GIpRequests", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGIpRequests(GCStringOut requests, GSize requests_len);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GMessage", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGMessage(GCon g, GCStringOut buffer, GSize bufferLength);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GMotionComplete", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGMotionComplete(GCon g, GCStringIn axes);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GOpen", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGOpen(GCStringIn address, ref GCon g);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GProgramDownload", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGProgramDownload(GCon g, GCStringIn program, GCStringIn preprocessor);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GProgramDownloadFile", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGProgramDownloadFile(GCon g, GCStringIn path, GCStringIn preprocessor);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GProgramUpload", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGProgramUpload(GCon g, GCStringOut buffer, GSize bufferLength);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GProgramUploadFile", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGProgramUploadFile(GCon g, GCStringIn path);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GRead", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGRead(GCon g, byte[] record, GSize buffer_len, ref GSize bytes_read);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GRecord", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGRecord(GCon g, byte[] record, GOption method);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GRecordRate", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGRecordRate(GCon g, double period_ms);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GTimeout", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern void DllGTimeout(GCon g, GOption timeoutMs);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GVersion", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGVersion(GCStringOut ver, GSize ver_len);

    [DllImport(LibraryPath.GclibDllPath_, EntryPoint = "GWrite", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGWrite(GCon g, GCStringIn buffer, GSize buffer_len);
	
	[DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GSetupDownloadFile", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGSetupDownloadFile(GCon g, GCStringIn file_path, GOption options, GCStringOut info, GSize info_len);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GSetServer", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGSetServer(GCStringIn server_name);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GServerStatus", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGServerStatus(GCStringOut status, GSize status_len);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GListServers", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGListServers(GCStringOut servers, GSize servers_len);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GPublishServer", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGPublishServer(GCStringIn name, GOption publish, GOption save);

    [DllImport(LibraryPath.GcliboDllPath_, EntryPoint = "GRemoteConnections", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
    private static extern GReturn DllGRemoteConnections(GCStringOut connections, GSize connections_len);

    #endregion

    #region "Data Record"

    private T ByteArrayToDataRecord<T>(byte[] array)
        where T : struct, GDataRecord
    {
        GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
        try
        {
            return Marshal.PtrToStructure<T>(handle.AddrOfPinnedObject());
        }
        catch
        {
            return default(T);
        }
        finally
        {
            handle.Free();
        }
    }

    public interface GDataRecord
    {
        //! Returns the data record as a byte array and allows for access to individual bytes.
        byte[] byte_array();
    }

    private static byte[] StructToByteArray(GDataRecord record) //Returns this DataRecord as a byte[]
    {
        int size = Marshal.SizeOf(record);
        byte[] arr = new byte[size];

        IntPtr ptr = Marshal.AllocHGlobal(size);
        Marshal.StructureToPtr(record, ptr, true);
        Marshal.Copy(ptr, arr, 0, size);
        Marshal.FreeHGlobal(ptr);
        return arr;
    }


    //! Data record struct for DMC-4000 controllers, including 4000, 4200, 4103, and 500x0.
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord4000 : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }
        /*Offset   type name        description*/

        /*00*/      public UB	header_0; //!<1st Byte of Header.
	    /*01*/	    public UB	header_1; //!<2nd Byte of Header.
	    /*02*/	    public UB	header_2; //!<3rd Byte of Header.
	    /*03*/	    public UB	header_3; //!<4th Byte of Header.
                     
	    /*04-05*/  	public UW	sample_number; //!<sample number.
                     
	    /*06*/	    public UB	input_bank_0; //!<general input bank 0 (inputs 1-8).
	    /*07*/	    public UB	input_bank_1; //!<general input bank 1 (inputs 9-16).
	    /*08*/	    public UB	input_bank_2; //!<general input bank 2 (inputs 17-24).
	    /*09*/	    public UB	input_bank_3; //!<general input bank 3 (inputs 25-32).
	    /*10*/    	public UB	input_bank_4; //!<general input bank 4 (inputs 33-40).
	    /*11*/	    public UB	input_bank_5; //!<general input bank 5 (inputs 41-48).
	    /*12*/	    public UB	input_bank_6; //!<general input bank 6 (inputs 49-56).
	    /*13*/   	public UB	input_bank_7; //!<general input bank 7 (inputs 57-64).
	    /*14*/   	public UB	input_bank_8; //!<general input bank 8 (inputs 65-72).
	    /*15*/	    public UB	input_bank_9; //!<general input bank 9 (inputs 73-80).
                     
	    /*16*/    	public UB	output_bank_0; //!<general output bank 0 (outputs 1-8).
	    /*17*/	    public UB	output_bank_1; //!<general output bank 1 (outputs 9-16).
	    /*18*/   	public UB	output_bank_2; //!<general output bank 2 (outputs 17-24).
	    /*19*/	    public UB	output_bank_3; //!<general output bank 3 (outputs 25-32).
	    /*20*/   	public UB	output_bank_4; //!<general output bank 4 (outputs 33-40).
	    /*21*/	    public UB	output_bank_5; //!<general output bank 5 (outputs 41-48).
	    /*22*/	    public UB	output_bank_6; //!<general output bank 6 (outputs 49-56).
	    /*23*/	    public UB	output_bank_7; //!<general output bank 7 (outputs 57-64).
	    /*24*/  	public UB	output_bank_8; //!<general output bank 8 (outputs 65-72).
	    /*25*/  	public UB	output_bank_9; //!<general output bank 9 (outputs 73-80).
                     
	    /*26-27*/	public SW  reserved_0; //!<Reserved.
	    /*28-29*/	public SW 	reserved_2; //!<Reserved.
	    /*30-31*/	public SW 	reserved_4; //!<Reserved.
	    /*32-33*/	public SW	reserved_6; //!<Reserved.
	    /*34-35*/	public SW	reserved_8; //!<Reserved.
	    /*36-37*/	public SW	reserved_10; //!<Reserved.
	    /*38-39*/	public SW	reserved_12; //!<Reserved.
	    /*40-41*/	public SW	reserved_14; //!<Reserved.
                     
	    /*42*/	    public UB	ethernet_status_a; //!<Ethernet Handle A Status.
	    /*43*/   	public UB	ethernet_status_b; //!<Ethernet Handle B Status.
	    /*44*/	    public UB	ethernet_status_c; //!<Ethernet Handle C Status.
	    /*45*/   	public UB	ethernet_status_d; //!<Ethernet Handle D Status.
	    /*46*/  	public UB	ethernet_status_e; //!<Ethernet Handle E Status.
	    /*47*/	    public UB	ethernet_status_f; //!<Ethernet Handle F Status.
	    /*48*/  	public UB	ethernet_status_g; //!<Ethernet Handle G Status.
	    /*49*/  	public UB	ethernet_status_h; //!<Ethernet Handle H Status.
                     
	    /*50*/	    public UB	error_code; //!<error code.
	    /*51*/  	public UB	thread_status; //!<thread status
	    /*52-55*/	public UL	amplifier_status; //!<Amplifier Status.
                     
	    /*56-59*/	public UL	contour_segment_count; //!<Segment Count for Contour Mode.
	    /*60-61*/	public UW	contour_buffer_available; //!<Buffer space remaining, Contour Mode.
                     
	    /*62-63*/	public UW	s_plane_segment_count; //!<segment count of coordinated move for S plane.
	    /*64-65*/	public UW	s_plane_move_status; //!<coordinated move status for S plane.
	    /*66-69*/	public SL	s_distance; //!<distance traveled in coordinated move for S plane.
	    /*70-71*/	public UW	s_plane_buffer_available; //!<Buffer space remaining, S Plane.
                     
	    /*72-73*/	public UW	t_plane_segment_count; //!<segment count of coordinated move for T plane.
	    /*74-75*/	public UW	t_plane_move_status; //!<Coordinated move status for T plane.
	    /*76-79*/	public SL	t_distance; //!<distance traveled in coordinated move for T plane.
	    /*80-81*/	public UW	t_plane_buffer_available; //!<Buffer space remaining, T Plane.
                     
	    /*82-83*/	public UW	axis_a_status; //!<A axis status.
	    /*84*/	    public UB	axis_a_switches; //!<A axis switches.
	    /*85*/	    public UB	axis_a_stop_code; //!<A axis stop code.
	    /*86-89*/	public SL	axis_a_reference_position; //!<A axis reference position.
	    /*90-93*/	public SL	axis_a_motor_position; //!<A axis motor position.
	    /*94-97*/	public SL	axis_a_position_error; //!<A axis position error.
	    /*98-101*/	public SL	axis_a_aux_position; //!<A axis auxiliary position.
	    /*102-105*/	public SL	axis_a_velocity; //!<A axis velocity.
	    /*106-109*/	public SL 	axis_a_torque; //!<A axis torque.
	    /*110-111*/	public UW  axis_a_analog_in; //!<A axis analog input.
	    /*112*/	    public UB	axis_a_halls; //!<A Hall Input Status.
	    /*113*/  	public UB	axis_a_reserved; //!<Reserved.
	    /*114-117*/	public SL	axis_a_variable; //!<A User-defined variable (ZA).
                     
	    /*118-119*/	public UW	axis_b_status; //!<B axis status.
	    /*120*/	    public UB	axis_b_switches; //!<B axis switches.
	    /*121*/	    public UB	axis_b_stop_code; //!<B axis stop code.
	    /*122-125*/	public SL	axis_b_reference_position; //!<B axis reference position.
	    /*126-129*/	public SL	axis_b_motor_position; //!<B axis motor position.
	    /*130-133*/	public SL	axis_b_position_error; //!<B axis position error.
	    /*134-137*/	public SL	axis_b_aux_position; //!<B axis auxiliary position.
	    /*138-141*/	public SL	axis_b_velocity; //!<B axis velocity.
	    /*142-145*/	public SL	axis_b_torque; //!<B axis torque.
	    /*146-147*/	public UW  axis_b_analog_in; //!<B axis analog input.
	    /*148*/   	public UB	axis_b_halls; //!<B Hall Input Status.
	    /*149*/	    public UB	axis_b_reserved; //!<Reserved.
	    /*150-153*/	public SL	axis_b_variable; //!<B User-defined variable (ZA).
                     
	    /*154-155*/	public UW	axis_c_status; //!<C axis status.
	    /*156*/  	public UB	axis_c_switches; //!<C axis switches.
	    /*157*/ 	public UB	axis_c_stop_code; //!<C axis stop code.
	    /*158-161*/	public SL	axis_c_reference_position; //!<C axis reference position.
	    /*162-165*/	public SL	axis_c_motor_position; //!<C axis motor position.
	    /*166-169*/	public SL	axis_c_position_error; //!<C axis position error.
	    /*170-173*/	public SL	axis_c_aux_position; //!<C axis auxiliary position.
	    /*174-177*/	public SL	axis_c_velocity; //!<C axis velocity.
	    /*178-181*/	public SL	axis_c_torque; //!<C axis torque.
	    /*182-183*/	public UW 	axis_c_analog_in; //!<C axis analog input.
	    /*184*/	    public UB	axis_c_halls; //!<C Hall Input Status.
	    /*185*/	    public UB	axis_c_reserved; //!<Reserved.
	    /*186-189*/	public SL	axis_c_variable; //!<C User-defined variable (ZA).
                     
	    /*190-191*/	public UW	axis_d_status; //!<D axis status.
	    /*192*/	    public UB	axis_d_switches; //!<D axis switches.
	    /*193*/  	public UB	axis_d_stop_code; //!<D axis stop code.
	    /*194-197*/	public SL	axis_d_reference_position; //!<D axis reference position.
	    /*198-201*/	public SL	axis_d_motor_position; //!<D axis motor position.
	    /*202-205*/	public SL	axis_d_position_error; //!<D axis position error.
	    /*206-209*/	public SL	axis_d_aux_position; //!<D axis auxiliary position.
	    /*210-213*/	public SL	axis_d_velocity; //!<D axis velocity.
	    /*214-217*/	public SL	axis_d_torque; //!<D axis torque.
	    /*218-219*/ public UW  axis_d_analog_in; //!<D axis analog input.
	    /*220*/  	public UB	axis_d_halls; //!<D Hall Input Status.
	    /*221*/ 	public UB	axis_d_reserved; //!<Reserved.
	    /*222-225*/	public SL	axis_d_variable; //!<D User-defined variable (ZA).
                     
	    /*226-227*/	public UW	axis_e_status; //!<E axis status.
	    /*228*/	    public UB	axis_e_switches; //!<E axis switches.
	    /*229*/	    public UB	axis_e_stop_code; //!<E axis stop code.
	    /*230-233*/	public SL	axis_e_reference_position; //!<E axis reference position.
	    /*234-237*/	public SL	axis_e_motor_position; //!<E axis motor position.
	    /*238-241*/	public SL	axis_e_position_error; //!<E axis position error.
	    /*242-245*/	public SL	axis_e_aux_position; //!<E axis auxiliary position.
	    /*246-249*/	public SL	axis_e_velocity; //!<E axis velocity.
	    /*250-253*/	public SL	axis_e_torque; //!<E axis torque.
	    /*254-255*/	public UW  axis_e_analog_in; //!<E axis analog input.
	    /*256*/  	public UB	axis_e_halls; //!<E Hall Input Status.
	    /*257*/	    public UB	axis_e_reserved; //!<Reserved.
	    /*258-261*/	public SL	axis_e_variable; //!<E User-defined variable (ZA).
                     
	    /*262-263*/	public UW	axis_f_status; //!<F axis status.
	    /*264*/	    public UB	axis_f_switches; //!<F axis switches.
	    /*265*/	    public UB	axis_f_stop_code; //!<F axis stop code.
	    /*266-269*/	public SL	axis_f_reference_position; //!<F axis reference position.
	    /*270-273*/	public SL	axis_f_motor_position; //!<F axis motor position.
	    /*274-277*/	public SL	axis_f_position_error; //!<F axis position error.
	    /*278-281*/	public SL	axis_f_aux_position; //!<F axis auxiliary position.
	    /*282-285*/	public SL	axis_f_velocity; //!<F axis velocity.
	    /*286-289*/	public SL	axis_f_torque; //!<F axis torque.
	    /*290-291*/	public UW	axis_f_analog_in; //!<F axis analog input.
	    /*292*/	    public UB	axis_f_halls; //!<F Hall Input Status.
	    /*293*/	    public UB	axis_f_reserved; //!<Reserved.
	    /*294-297*/	public SL	axis_f_variable; //!<F User-defined variable (ZA).
                     
	    /*298-299*/	public UW	axis_g_status; //!<G axis status.
	    /*300*/  	public UB	axis_g_switches; //!<G axis switches.
	    /*301*/ 	public UB	axis_g_stop_code; //!<G axis stop code.
	    /*302-305*/	public SL	axis_g_reference_position; //!<G axis reference position.
	    /*306-309*/	public SL	axis_g_motor_position; //!<G axis motor position.
	    /*310-313*/	public SL	axis_g_position_error; //!<G axis position error.
	    /*314-317*/	public SL	axis_g_aux_position; //!<G axis auxiliary position.
	    /*318-321*/	public SL	axis_g_velocity; //!<G axis velocity.
	    /*322-325*/	public SL	axis_g_torque; //!<G axis torque.
	    /*326-327*/ public UW  axis_g_analog_in; //!<G axis analog input.
	    /*328*/	    public UB	axis_g_halls; //!<G Hall Input Status.
	    /*329*/	    public UB	axis_g_reserved; //!<Reserved.
	    /*330-333*/	public SL	axis_g_variable; //!<G User-defined variable (ZA).
                     
	    /*334-335*/	public UW	axis_h_status; //!<H axis status.
	    /*336*/  	public UB	axis_h_switches; //!<H axis switches.
	    /*337*/	    public UB	axis_h_stop_code; //!<H axis stop code.
	    /*338-341*/	public SL	axis_h_reference_position; //!<H axis reference position.
	    /*342-345*/	public SL	axis_h_motor_position; //!<H axis motor position.
	    /*346-349*/	public SL	axis_h_position_error; //!<H axis position error.
	    /*350-353*/	public SL	axis_h_aux_position; //!<H axis auxiliary position.
	    /*354-357*/	public SL	axis_h_velocity; //!<H axis velocity.
	    /*358-361*/	public SL	axis_h_torque; //!<H axis torque.
	    /*362-363*/	public UW  axis_h_analog_in; //!<H axis analog input.
	    /*364*/	    public UB	axis_h_halls; //!<H Hall Input Status.
	    /*365*/  	public UB	axis_h_reserved; //!<Reserved.
	    /*366-369*/	public SL	axis_h_variable; //!<H User-defined variable (ZA).
    }; //DataRecord4000

    //! Data record struct for DMC-52000 controller. Same as DMC-4000, with bank indicator added at byte 40.
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord52000 : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }
        /*Offset   type name        description*/

        /*00*/      public UB	header_0; //!<1st Byte of Header.
	    /*01*/	    public UB	header_1; //!<2nd Byte of Header.
	    /*02*/	    public UB	header_2; //!<3rd Byte of Header.
	    /*03*/	    public UB	header_3; //!<4th Byte of Header.
                     
	    /*04-05*/  	public UW	sample_number; //!<sample number.
                     
	    /*06*/	    public UB	input_bank_0; //!<general input bank 0 (inputs 1-8).
	    /*07*/	    public UB	input_bank_1; //!<general input bank 1 (inputs 9-16).
	    /*08*/	    public UB	input_bank_2; //!<general input bank 2 (inputs 17-24).
	    /*09*/	    public UB	input_bank_3; //!<general input bank 3 (inputs 25-32).
	    /*10*/    	public UB	input_bank_4; //!<general input bank 4 (inputs 33-40).
	    /*11*/	    public UB	input_bank_5; //!<general input bank 5 (inputs 41-48).
	    /*12*/	    public UB	input_bank_6; //!<general input bank 6 (inputs 49-56).
	    /*13*/   	public UB	input_bank_7; //!<general input bank 7 (inputs 57-64).
	    /*14*/   	public UB	input_bank_8; //!<general input bank 8 (inputs 65-72).
	    /*15*/	    public UB	input_bank_9; //!<general input bank 9 (inputs 73-80).
                     
	    /*16*/    	public UB	output_bank_0; //!<general output bank 0 (outputs 1-8).
	    /*17*/	    public UB	output_bank_1; //!<general output bank 1 (outputs 9-16).
	    /*18*/   	public UB	output_bank_2; //!<general output bank 2 (outputs 17-24).
	    /*19*/	    public UB	output_bank_3; //!<general output bank 3 (outputs 25-32).
	    /*20*/   	public UB	output_bank_4; //!<general output bank 4 (outputs 33-40).
	    /*21*/	    public UB	output_bank_5; //!<general output bank 5 (outputs 41-48).
	    /*22*/	    public UB	output_bank_6; //!<general output bank 6 (outputs 49-56).
	    /*23*/	    public UB	output_bank_7; //!<general output bank 7 (outputs 57-64).
	    /*24*/  	public UB	output_bank_8; //!<general output bank 8 (outputs 65-72).
	    /*25*/  	public UB	output_bank_9; //!<general output bank 9 (outputs 73-80).
                     
	    /*26-27*/	public SW  reserved_0; //!<Reserved.
	    /*28-29*/	public SW 	reserved_2; //!<Reserved.
	    /*30-31*/	public SW 	reserved_4; //!<Reserved.
	    /*32-33*/	public SW	reserved_6; //!<Reserved.
	    /*34-35*/	public SW	reserved_8; //!<Reserved.
	    /*36-37*/	public SW	reserved_10; //!<Reserved.
	    /*38-39*/	public SW	reserved_12; //!<Reserved.
	    /*40*/      public UB  ethercat_bank; //!< EtherCAT Bank Indicator.
	    /*41*/   	public UB	reserved_14; //!<Reserved.
                     
	    /*42*/	    public UB	ethernet_status_a; //!<Ethernet Handle A Status.
	    /*43*/   	public UB	ethernet_status_b; //!<Ethernet Handle B Status.
	    /*44*/	    public UB	ethernet_status_c; //!<Ethernet Handle C Status.
	    /*45*/   	public UB	ethernet_status_d; //!<Ethernet Handle D Status.
	    /*46*/  	public UB	ethernet_status_e; //!<Ethernet Handle E Status.
	    /*47*/	    public UB	ethernet_status_f; //!<Ethernet Handle F Status.
	    /*48*/  	public UB	ethernet_status_g; //!<Ethernet Handle G Status.
	    /*49*/  	public UB	ethernet_status_h; //!<Ethernet Handle H Status.
                     
	    /*50*/	    public UB	error_code; //!<error code.
	    /*51*/  	public UB	thread_status; //!<thread status
	    /*52-55*/	public UL	amplifier_status; //!<Amplifier Status.
                     
	    /*56-59*/	public UL	contour_segment_count; //!<Segment Count for Contour Mode.
	    /*60-61*/	public UW	contour_buffer_available; //!<Buffer space remaining, Contour Mode.
                     
	    /*62-63*/	public UW	s_plane_segment_count; //!<segment count of coordinated move for S plane.
	    /*64-65*/	public UW	s_plane_move_status; //!<coordinated move status for S plane.
	    /*66-69*/	public SL	s_distance; //!<distance traveled in coordinated move for S plane.
	    /*70-71*/	public UW	s_plane_buffer_available; //!<Buffer space remaining, S Plane.
                     
	    /*72-73*/	public UW	t_plane_segment_count; //!<segment count of coordinated move for T plane.
	    /*74-75*/	public UW	t_plane_move_status; //!<Coordinated move status for T plane.
	    /*76-79*/	public SL	t_distance; //!<distance traveled in coordinated move for T plane.
	    /*80-81*/	public UW	t_plane_buffer_available; //!<Buffer space remaining, T Plane.
                     
	    /*82-83*/	public UW	axis_a_status; //!<A axis status.
	    /*84*/	    public UB	axis_a_switches; //!<A axis switches.
	    /*85*/	    public UB	axis_a_stop_code; //!<A axis stop code.
	    /*86-89*/	public SL	axis_a_reference_position; //!<A axis reference position.
	    /*90-93*/	public SL	axis_a_motor_position; //!<A axis motor position.
	    /*94-97*/	public SL	axis_a_position_error; //!<A axis position error.
	    /*98-101*/	public SL	axis_a_aux_position; //!<A axis auxiliary position.
	    /*102-105*/	public SL	axis_a_velocity; //!<A axis velocity.
	    /*106-109*/	public SL 	axis_a_torque; //!<A axis torque.
	    /*110-111*/	public UW  axis_a_analog_in; //!<A axis analog input.
	    /*112*/	    public UB	axis_a_halls; //!<A Hall Input Status.
	    /*113*/  	public UB	axis_a_reserved; //!<Reserved.
	    /*114-117*/	public SL	axis_a_variable; //!<A User-defined variable (ZA).
                     
	    /*118-119*/	public UW	axis_b_status; //!<B axis status.
	    /*120*/	    public UB	axis_b_switches; //!<B axis switches.
	    /*121*/	    public UB	axis_b_stop_code; //!<B axis stop code.
	    /*122-125*/	public SL	axis_b_reference_position; //!<B axis reference position.
	    /*126-129*/	public SL	axis_b_motor_position; //!<B axis motor position.
	    /*130-133*/	public SL	axis_b_position_error; //!<B axis position error.
	    /*134-137*/	public SL	axis_b_aux_position; //!<B axis auxiliary position.
	    /*138-141*/	public SL	axis_b_velocity; //!<B axis velocity.
	    /*142-145*/	public SL	axis_b_torque; //!<B axis torque.
	    /*146-147*/	public UW  axis_b_analog_in; //!<B axis analog input.
	    /*148*/   	public UB	axis_b_halls; //!<B Hall Input Status.
	    /*149*/	    public UB	axis_b_reserved; //!<Reserved.
	    /*150-153*/	public SL	axis_b_variable; //!<B User-defined variable (ZA).
                     
	    /*154-155*/	public UW	axis_c_status; //!<C axis status.
	    /*156*/  	public UB	axis_c_switches; //!<C axis switches.
	    /*157*/ 	public UB	axis_c_stop_code; //!<C axis stop code.
	    /*158-161*/	public SL	axis_c_reference_position; //!<C axis reference position.
	    /*162-165*/	public SL	axis_c_motor_position; //!<C axis motor position.
	    /*166-169*/	public SL	axis_c_position_error; //!<C axis position error.
	    /*170-173*/	public SL	axis_c_aux_position; //!<C axis auxiliary position.
	    /*174-177*/	public SL	axis_c_velocity; //!<C axis velocity.
	    /*178-181*/	public SL	axis_c_torque; //!<C axis torque.
	    /*182-183*/	public UW 	axis_c_analog_in; //!<C axis analog input.
	    /*184*/	    public UB	axis_c_halls; //!<C Hall Input Status.
	    /*185*/	    public UB	axis_c_reserved; //!<Reserved.
	    /*186-189*/	public SL	axis_c_variable; //!<C User-defined variable (ZA).
                     
	    /*190-191*/	public UW	axis_d_status; //!<D axis status.
	    /*192*/	    public UB	axis_d_switches; //!<D axis switches.
	    /*193*/  	public UB	axis_d_stop_code; //!<D axis stop code.
	    /*194-197*/	public SL	axis_d_reference_position; //!<D axis reference position.
	    /*198-201*/	public SL	axis_d_motor_position; //!<D axis motor position.
	    /*202-205*/	public SL	axis_d_position_error; //!<D axis position error.
	    /*206-209*/	public SL	axis_d_aux_position; //!<D axis auxiliary position.
	    /*210-213*/	public SL	axis_d_velocity; //!<D axis velocity.
	    /*214-217*/	public SL	axis_d_torque; //!<D axis torque.
	    /*218-219*/ public UW  axis_d_analog_in; //!<D axis analog input.
	    /*220*/  	public UB	axis_d_halls; //!<D Hall Input Status.
	    /*221*/ 	public UB	axis_d_reserved; //!<Reserved.
	    /*222-225*/	public SL	axis_d_variable; //!<D User-defined variable (ZA).
                     
	    /*226-227*/	public UW	axis_e_status; //!<E axis status.
	    /*228*/	    public UB	axis_e_switches; //!<E axis switches.
	    /*229*/	    public UB	axis_e_stop_code; //!<E axis stop code.
	    /*230-233*/	public SL	axis_e_reference_position; //!<E axis reference position.
	    /*234-237*/	public SL	axis_e_motor_position; //!<E axis motor position.
	    /*238-241*/	public SL	axis_e_position_error; //!<E axis position error.
	    /*242-245*/	public SL	axis_e_aux_position; //!<E axis auxiliary position.
	    /*246-249*/	public SL	axis_e_velocity; //!<E axis velocity.
	    /*250-253*/	public SL	axis_e_torque; //!<E axis torque.
	    /*254-255*/	public UW  axis_e_analog_in; //!<E axis analog input.
	    /*256*/  	public UB	axis_e_halls; //!<E Hall Input Status.
	    /*257*/	    public UB	axis_e_reserved; //!<Reserved.
	    /*258-261*/	public SL	axis_e_variable; //!<E User-defined variable (ZA).
                     
	    /*262-263*/	public UW	axis_f_status; //!<F axis status.
	    /*264*/	    public UB	axis_f_switches; //!<F axis switches.
	    /*265*/	    public UB	axis_f_stop_code; //!<F axis stop code.
	    /*266-269*/	public SL	axis_f_reference_position; //!<F axis reference position.
	    /*270-273*/	public SL	axis_f_motor_position; //!<F axis motor position.
	    /*274-277*/	public SL	axis_f_position_error; //!<F axis position error.
	    /*278-281*/	public SL	axis_f_aux_position; //!<F axis auxiliary position.
	    /*282-285*/	public SL	axis_f_velocity; //!<F axis velocity.
	    /*286-289*/	public SL	axis_f_torque; //!<F axis torque.
	    /*290-291*/	public UW	axis_f_analog_in; //!<F axis analog input.
	    /*292*/	    public UB	axis_f_halls; //!<F Hall Input Status.
	    /*293*/	    public UB	axis_f_reserved; //!<Reserved.
	    /*294-297*/	public SL	axis_f_variable; //!<F User-defined variable (ZA).
                     
	    /*298-299*/	public UW	axis_g_status; //!<G axis status.
	    /*300*/  	public UB	axis_g_switches; //!<G axis switches.
	    /*301*/ 	public UB	axis_g_stop_code; //!<G axis stop code.
	    /*302-305*/	public SL	axis_g_reference_position; //!<G axis reference position.
	    /*306-309*/	public SL	axis_g_motor_position; //!<G axis motor position.
	    /*310-313*/	public SL	axis_g_position_error; //!<G axis position error.
	    /*314-317*/	public SL	axis_g_aux_position; //!<G axis auxiliary position.
	    /*318-321*/	public SL	axis_g_velocity; //!<G axis velocity.
	    /*322-325*/	public SL	axis_g_torque; //!<G axis torque.
	    /*326-327*/ public UW  axis_g_analog_in; //!<G axis analog input.
	    /*328*/	    public UB	axis_g_halls; //!<G Hall Input Status.
	    /*329*/	    public UB	axis_g_reserved; //!<Reserved.
	    /*330-333*/	public SL	axis_g_variable; //!<G User-defined variable (ZA).
                     
	    /*334-335*/	public UW	axis_h_status; //!<H axis status.
	    /*336*/  	public UB	axis_h_switches; //!<H axis switches.
	    /*337*/	    public UB	axis_h_stop_code; //!<H axis stop code.
	    /*338-341*/	public SL	axis_h_reference_position; //!<H axis reference position.
	    /*342-345*/	public SL	axis_h_motor_position; //!<H axis motor position.
	    /*346-349*/	public SL	axis_h_position_error; //!<H axis position error.
	    /*350-353*/	public SL	axis_h_aux_position; //!<H axis auxiliary position.
	    /*354-357*/	public SL	axis_h_velocity; //!<H axis velocity.
	    /*358-361*/	public SL	axis_h_torque; //!<H axis torque.
	    /*362-363*/	public UW  axis_h_analog_in; //!<H axis analog input.
	    /*364*/	    public UB	axis_h_halls; //!<H Hall Input Status.
	    /*365*/  	public UB	axis_h_reserved; //!<Reserved.
	    /*366-369*/	public SL	axis_h_variable; //!<H User-defined variable (ZA).
    }; //DataRecord52000

    //! Data record struct for DMC-1806 controller.
    /*!
    The 18x6 Data record is the same as 4000 except the following.
    -# No header bytes. Firmware strips it in DR. Software removes it from QR.
    -# No Ethernet status (bytes 42-49).
    -# No amplfifier status (bytes 52-55).
    -# No axis-specific hall input status.
    */
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord1806 : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }
        /*Offset   type name        description*/

        /*00-01*/   public UW	sample_number; //!<sample number.
                     
	    /*02*/	    public UB	input_bank_0; //!<general input bank 0 (inputs 1-8).
	    /*03*/	    public UB	input_bank_1; //!<general input bank 1 (inputs 9-16).
	    /*04*/	    public UB	input_bank_2; //!<general input bank 2 (inputs 17-24).
	    /*05*/	    public UB	input_bank_3; //!<general input bank 3 (inputs 25-32).
	    /*06*/    	public UB	input_bank_4; //!<general input bank 4 (inputs 33-40).
	    /*07*/	    public UB	input_bank_5; //!<general input bank 5 (inputs 41-48).
	    /*08*/	    public UB	input_bank_6; //!<general input bank 6 (inputs 49-56).
	    /*09*/   	public UB	input_bank_7; //!<general input bank 7 (inputs 57-64).
	    /*10*/   	public UB	input_bank_8; //!<general input bank 8 (inputs 65-72).
	    /*11*/	    public UB	input_bank_9; //!<general input bank 9 (inputs 73-80).
                     
	    /*12*/    	public UB	output_bank_0; //!<general output bank 0 (outputs 1-8).
	    /*13*/	    public UB	output_bank_1; //!<general output bank 1 (outputs 9-16).
	    /*14*/   	public UB	output_bank_2; //!<general output bank 2 (outputs 17-24).
	    /*15*/	    public UB	output_bank_3; //!<general output bank 3 (outputs 25-32).
	    /*16*/   	public UB	output_bank_4; //!<general output bank 4 (outputs 33-40).
	    /*17*/	    public UB	output_bank_5; //!<general output bank 5 (outputs 41-48).
	    /*18*/	    public UB	output_bank_6; //!<general output bank 6 (outputs 49-56).
	    /*19*/	    public UB	output_bank_7; //!<general output bank 7 (outputs 57-64).
	    /*20*/  	public UB	output_bank_8; //!<general output bank 8 (outputs 65-72).
	    /*21*/  	public UB	output_bank_9; //!<general output bank 9 (outputs 73-80).
                     
	    /*22-23*/	public SW  reserved_0; //!<Reserved.
	    /*24-25*/	public SW 	reserved_2; //!<Reserved.
	    /*26-27*/	public SW 	reserved_4; //!<Reserved.
	    /*28-29*/	public SW	reserved_6; //!<Reserved.
	    /*30-31*/	public SW	reserved_8; //!<Reserved.
	    /*32-33*/	public SW	reserved_10; //!<Reserved.
	    /*34-35*/	public SW	reserved_12; //!<Reserved.
	    /*36-37*/	public SW	reserved_14; //!<Reserved.
                     
	    /*38*/	    public UB	reserved_16; //!<Reserved.
	    /*39*/   	public UB	reserved_17; //!<Reserved.
	    /*40*/	    public UB	reserved_18; //!<Reserved.
	    /*41*/   	public UB	reserved_19; //!<Reserved.
	    /*42*/  	public UB	reserved_20; //!<Reserved.
	    /*43*/	    public UB	reserved_21; //!<Reserved.
	    /*44*/  	public UB	reserved_22; //!<Reserved.
	    /*45*/  	public UB	reserved_23; //!<Reserved.
                     
	    /*46*/	    public UB	error_code; //!<error code.
	    /*47*/  	public UB	thread_status; //!<thread status.
	    /*48-51*/	public UL	reserved_24; //!<Reserved.
                     
	    /*52-55*/	public UL	contour_segment_count; //!<Segment Count for Contour Mode.
	    /*56-57*/	public UW	contour_buffer_available; //!<Buffer space remaining, Contour Mode.
                     
	    /*58-59*/	public UW	s_plane_segment_count; //!<segment count of coordinated move for S plane.
	    /*60-61*/	public UW	s_plane_move_status; //!<coordinated move status for S plane.
	    /*62-65*/	public SL	s_distance; //!<distance traveled in coordinated move for S plane.
	    /*66-67*/	public UW	s_plane_buffer_available; //!<Buffer space remaining, S Plane.
                     
	    /*68-69*/	public UW	t_plane_segment_count; //!<segment count of coordinated move for T plane.
	    /*70-71*/	public UW	t_plane_move_status; //!<Coordinated move status for T plane.
	    /*72-75*/	public SL	t_distance; //!<distance traveled in coordinated move for T plane.
	    /*76-77*/	public UW	t_plane_buffer_available; //!<Buffer space remaining, T Plane.
                     
	    /*78-79*/	public UW	axis_a_status; //!<A axis status.
	    /*80*/	    public UB	axis_a_switches; //!<A axis switches.
	    /*81*/	    public UB	axis_a_stop_code; //!<A axis stop code.
	    /*82-85*/	public SL	axis_a_reference_position; //!<A axis reference position.
	    /*86-89*/	public SL	axis_a_motor_position; //!<A axis motor position.
	    /*90-93*/	public SL	axis_a_position_error; //!<A axis position error.
	    /*94-97*/	public SL	axis_a_aux_position; //!<A axis auxiliary position.
	    /*98-101*/	public SL	axis_a_velocity; //!<A axis velocity.
	    /*102-105*/	public SL 	axis_a_torque; //!<A axis torque.
	    /*106-107*/	public UW  axis_a_analog_in; //!<A axis analog input.
	    /*108*/	    public UB	axis_a_reserved_0; //!<Reserved.
	    /*109*/  	public UB	axis_a_reserved_1; //!<Reserved.
	    /*110-113*/	public SL	axis_a_variable; //!<A User-defined variable (ZA).
                     
	    /*114-115*/	public UW	axis_b_status; //!<B axis status.
	    /*116*/	    public UB	axis_b_switches; //!<B axis switches.
	    /*117*/	    public UB	axis_b_stop_code; //!<B axis stop code.
	    /*118-121*/	public SL	axis_b_reference_position; //!<B axis reference position.
	    /*122-125*/	public SL	axis_b_motor_position; //!<B axis motor position.
	    /*126-129*/	public SL	axis_b_position_error; //!<B axis position error.
	    /*130-133*/	public SL	axis_b_aux_position; //!<B axis auxiliary position.
	    /*134-137*/	public SL	axis_b_velocity; //!<B axis velocity.
	    /*138-141*/	public SL	axis_b_torque; //!<B axis torque.
	    /*142-143*/	public UW  axis_b_analog_in; //!<B axis analog input.
	    /*144*/   	public UB	axis_b_reserved_0; //!<Reserved.
	    /*145*/	    public UB	axis_b_reserved_1; //!<Reserved.
	    /*146-149*/	public SL	axis_b_variable; //!<B User-defined variable (ZA).
                     
	    /*150-151*/	public UW	axis_c_status; //!<C axis status.
	    /*152*/  	public UB	axis_c_switches; //!<C axis switches.
	    /*153*/ 	public UB	axis_c_stop_code; //!<C axis stop code.
	    /*154-157*/	public SL	axis_c_reference_position; //!<C axis reference position.
	    /*158-161*/	public SL	axis_c_motor_position; //!<C axis motor position.
	    /*162-165*/	public SL	axis_c_position_error; //!<C axis position error.
	    /*166-169*/	public SL	axis_c_aux_position; //!<C axis auxiliary position.
	    /*170-173*/	public SL	axis_c_velocity; //!<C axis velocity.
	    /*174-177*/	public SL	axis_c_torque; //!<C axis torque.
	    /*178-179*/	public UW 	axis_c_analog_in; //!<C axis analog input.
	    /*180*/	    public UB	axis_c_reserved_0; //!<Reserved.
	    /*181*/	    public UB	axis_c_reserved_1; //!<Reserved.
	    /*182-185*/	public SL	axis_c_variable; //!<C User-defined variable (ZA).
                     
	    /*186-187*/	public UW	axis_d_status; //!<D axis status.
	    /*188*/	    public UB	axis_d_switches; //!<D axis switches.
	    /*189*/  	public UB	axis_d_stop_code; //!<D axis stop code.
	    /*190-193*/	public SL	axis_d_reference_position; //!<D axis reference position.
	    /*194-197*/	public SL	axis_d_motor_position; //!<D axis motor position.
	    /*198-201*/	public SL	axis_d_position_error; //!<D axis position error.
	    /*202-205*/	public SL	axis_d_aux_position; //!<D axis auxiliary position.
	    /*206-209*/	public SL	axis_d_velocity; //!<D axis velocity.
	    /*210-213*/	public SL	axis_d_torque; //!<D axis torque.
	    /*214-215*/ public UW  axis_d_analog_in; //!<D axis analog input.
	    /*216*/  	public UB	axis_d_reserved_0; //!<Reserved.
	    /*217*/ 	public UB	axis_d_reserved_1; //!<Reserved.
	    /*218-221*/	public SL	axis_d_variable; //!<D User-defined variable (ZA).
                     
	    /*222-223*/	public UW	axis_e_status; //!<E axis status.
	    /*224*/	    public UB	axis_e_switches; //!<E axis switches.
	    /*225*/	    public UB	axis_e_stop_code; //!<E axis stop code.
	    /*226-229*/	public SL	axis_e_reference_position; //!<E axis reference position.
	    /*230-233*/	public SL	axis_e_motor_position; //!<E axis motor position.
	    /*234-237*/	public SL	axis_e_position_error; //!<E axis position error.
	    /*238-241*/	public SL	axis_e_aux_position; //!<E axis auxiliary position.
	    /*242-245*/	public SL	axis_e_velocity; //!<E axis velocity.
	    /*256-249*/	public SL	axis_e_torque; //!<E axis torque.
	    /*250-251*/	public UW  axis_e_analog_in; //!<E axis analog input.
	    /*252*/  	public UB	axis_e_reserved_0; //!<Reserved.
	    /*253*/	    public UB	axis_e_reserved_1; //!<Reserved.
	    /*254-257*/	public SL	axis_e_variable; //!<E User-defined variable (ZA).
                     
	    /*258-259*/	public UW	axis_f_status; //!<F axis status.
	    /*260*/	    public UB	axis_f_switches; //!<F axis switches.
	    /*261*/	    public UB	axis_f_stop_code; //!<F axis stop code.
	    /*262-265*/	public SL	axis_f_reference_position; //!<F axis reference position.
	    /*266-269*/	public SL	axis_f_motor_position; //!<F axis motor position.
	    /*270-273*/	public SL	axis_f_position_error; //!<F axis position error.
	    /*274-277*/	public SL	axis_f_aux_position; //!<F axis auxiliary position.
	    /*278-281*/	public SL	axis_f_velocity; //!<F axis velocity.
	    /*282-285*/	public SL	axis_f_torque; //!<F axis torque.
	    /*286-287*/	public UW	axis_f_analog_in; //!<F axis analog input.
	    /*288*/	    public UB	axis_f_reserved_0; //!<Reserved.
	    /*289*/	    public UB	axis_f_reserved_1; //!<Reserved.
	    /*290-293*/	public SL	axis_f_variable; //!<F User-defined variable (ZA).
                     
	    /*294-295*/	public UW	axis_g_status; //!<G axis status.
	    /*296*/  	public UB	axis_g_switches; //!<G axis switches.
	    /*297*/ 	public UB	axis_g_stop_code; //!<G axis stop code.
	    /*298-301*/	public SL	axis_g_reference_position; //!<G axis reference position.
	    /*302-305*/	public SL	axis_g_motor_position; //!<G axis motor position.
	    /*306-309*/	public SL	axis_g_position_error; //!<G axis position error.
	    /*310-313*/	public SL	axis_g_aux_position; //!<G axis auxiliary position.
	    /*314-317*/	public SL	axis_g_velocity; //!<G axis velocity.
	    /*318-321*/	public SL	axis_g_torque; //!<G axis torque.
	    /*322-323*/ public UW  axis_g_analog_in; //!<G axis analog input.
	    /*324*/	    public UB	axis_g_reserved_0; //!<Reserved.
	    /*325*/	    public UB	axis_g_reserved_1; //!<Reserved.
	    /*326-329*/	public SL	axis_g_variable; //!<G User-defined variable (ZA).
                     
	    /*330-331*/	public UW	axis_h_status; //!<H axis status.
	    /*332*/  	public UB	axis_h_switches; //!<H axis switches.
	    /*333*/	    public UB	axis_h_stop_code; //!<H axis stop code.
	    /*334-337*/	public SL	axis_h_reference_position; //!<H axis reference position.
	    /*338-341*/	public SL	axis_h_motor_position; //!<H axis motor position.
	    /*342-345*/	public SL	axis_h_position_error; //!<H axis position error.
	    /*346-349*/	public SL	axis_h_aux_position; //!<H axis auxiliary position.
	    /*350-353*/	public SL	axis_h_velocity; //!<H axis velocity.
	    /*354-357*/	public SL	axis_h_torque; //!<H axis torque.
	    /*358-359*/	public UW  axis_h_analog_in; //!<H axis analog input.
	    /*360*/	    public UB	axis_h_reserved_0; //!<Reserved.
	    /*361*/  	public UB	axis_h_reserved_1; //!<Reserved.
	    /*362-365*/	public SL	axis_h_variable; //!<H User-defined variable (ZA).
    }; //DataRecord1806

    //! Data record struct for DMC-2103 controllers.
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord2103 : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }

        /*Offset   type name        description*/

        /*00*/      public UB	header_0; //!<1st Byte of Header.
	    /*01*/	    public UB	header_1; //!<2nd Byte of Header.
	    /*02*/	    public UB	header_2; //!<3rd Byte of Header.
	    /*03*/	    public UB	header_3; //!<4th Byte of Header.
                     
	    /*04-05*/  	public UW	sample_number; //!<sample number.
                     
	    /*06*/	    public UB	input_bank_0; //!<general input bank 0 (inputs 1-8).
	    /*07*/	    public UB	input_bank_1; //!<general input bank 1 (inputs 9-16).
	    /*08*/	    public UB	input_bank_2; //!<general input bank 2 (inputs 17-24).
	    /*09*/	    public UB	input_bank_3; //!<general input bank 3 (inputs 25-32).
	    /*10*/    	public UB	input_bank_4; //!<general input bank 4 (inputs 33-40).
	    /*11*/	    public UB	input_bank_5; //!<general input bank 5 (inputs 41-48).
	    /*12*/	    public UB	input_bank_6; //!<general input bank 6 (inputs 49-56).
	    /*13*/   	public UB	input_bank_7; //!<general input bank 7 (inputs 57-64).
	    /*14*/   	public UB	input_bank_8; //!<general input bank 8 (inputs 65-72).
	    /*15*/	    public UB	input_bank_9; //!<general input bank 9 (inputs 73-80).
                     
	    /*16*/    	public UB	output_bank_0; //!<general output bank 0 (outputs 1-8).
	    /*17*/	    public UB	output_bank_1; //!<general output bank 1 (outputs 9-16).
	    /*18*/   	public UB	output_bank_2; //!<general output bank 2 (outputs 17-24).
	    /*19*/	    public UB	output_bank_3; //!<general output bank 3 (outputs 25-32).
	    /*20*/   	public UB	output_bank_4; //!<general output bank 4 (outputs 33-40).
	    /*21*/	    public UB	output_bank_5; //!<general output bank 5 (outputs 41-48).
	    /*22*/	    public UB	output_bank_6; //!<general output bank 6 (outputs 49-56).
	    /*23*/	    public UB	output_bank_7; //!<general output bank 7 (outputs 57-64).
	    /*24*/  	public UB	output_bank_8; //!<general output bank 8 (outputs 65-72).
	    /*25*/  	public UB	output_bank_9; //!<general output bank 9 (outputs 73-80).
                     
	    /*26*/	    public UB	error_code; //!<error code.
	    /*27*/  	public UB	general_status; //!<general status
                     
	    /*28-29*/	public UW	s_plane_segment_count; //!<segment count of coordinated move for S plane.
	    /*30-31*/	public UW	s_plane_move_status; //!<coordinated move status for S plane.
	    /*32-35*/	public SL	s_distance; //!<distance traveled in coordinated move for S plane.
                     
	    /*36-37*/	public UW	t_plane_segment_count; //!<segment count of coordinated move for T plane.
	    /*38-39*/	public UW	t_plane_move_status; //!<Coordinated move status for T plane.
	    /*40-43*/	public SL	t_distance; //!<distance traveled in coordinated move for T plane.
                     
	    /*44-45*/	public UW	axis_a_status; //!<A axis status.
	    /*46*/	    public UB	axis_a_switches; //!<A axis switches.
	    /*47*/	    public UB	axis_a_stop_code; //!<A axis stop code.
	    /*48-51*/	public SL	axis_a_reference_position; //!<A axis reference position.
	    /*52-55*/	public SL	axis_a_motor_position; //!<A axis motor position.
	    /*56-59*/	public SL	axis_a_position_error; //!<A axis position error.
	    /*60-63*/	public SL	axis_a_aux_position; //!<A axis auxiliary position.
	    /*64-67*/	public SL	axis_a_velocity; //!<A axis velocity.
	    /*68-69*/	public SW 	axis_a_torque; //!<A axis torque.
	    /*70-71*/	public UW  axis_a_analog_in; //!<A axis analog input.
                     
	    /*72-73*/	public UW	axis_b_status; //!<B axis status.
	    /*74*/	    public UB	axis_b_switches; //!<B axis switches.
	    /*75*/	    public UB	axis_b_stop_code; //!<B axis stop code.
	    /*76-79*/	public SL	axis_b_reference_position; //!<B axis reference position.
	    /*80-83*/	public SL	axis_b_motor_position; //!<B axis motor position.
	    /*84-87*/	public SL	axis_b_position_error; //!<B axis position error.
	    /*88-91*/	public SL	axis_b_aux_position; //!<B axis auxiliary position.
	    /*92-95*/	public SL	axis_b_velocity; //!<B axis velocity.
	    /*96-97*/	public SW	axis_b_torque; //!<B axis torque.
	    /*98-99*/	public UW  axis_b_analog_in; //!<B axis analog input.
                     
	    /*100-101*/	public UW	axis_c_status; //!<C axis status.
	    /*102*/  	public UB	axis_c_switches; //!<C axis switches.
	    /*103*/ 	public UB	axis_c_stop_code; //!<C axis stop code.
	    /*104-107*/	public SL	axis_c_reference_position; //!<C axis reference position.
	    /*108-111*/	public SL	axis_c_motor_position; //!<C axis motor position.
	    /*112-115*/	public SL	axis_c_position_error; //!<C axis position error.
	    /*116-119*/	public SL	axis_c_aux_position; //!<C axis auxiliary position.
	    /*120-123*/	public SL	axis_c_velocity; //!<C axis velocity.
	    /*124-125*/	public SW	axis_c_torque; //!<C axis torque.
	    /*126-127*/	public UW 	axis_c_analog_in; //!<C axis analog input.
                     
	    /*128-129*/	public UW	axis_d_status; //!<D axis status.
	    /*130*/	    public UB	axis_d_switches; //!<D axis switches.
	    /*131*/  	public UB	axis_d_stop_code; //!<D axis stop code.
	    /*132-135*/	public SL	axis_d_reference_position; //!<D axis reference position.
	    /*136-139*/	public SL	axis_d_motor_position; //!<D axis motor position.
	    /*140-143*/	public SL	axis_d_position_error; //!<D axis position error.
	    /*144-147*/	public SL	axis_d_aux_position; //!<D axis auxiliary position.
	    /*148-151*/	public SL	axis_d_velocity; //!<D axis velocity.
	    /*152-153*/	public SW	axis_d_torque; //!<D axis torque.
	    /*154-155*/ public UW  axis_d_analog_in; //!<D axis analog input.
                     
	    /*156-157*/	public UW	axis_e_status; //!<E axis status.
	    /*158*/	    public UB	axis_e_switches; //!<E axis switches.
	    /*159*/	    public UB	axis_e_stop_code; //!<E axis stop code.
	    /*160-163*/	public SL	axis_e_reference_position; //!<E axis reference position.
	    /*164-167*/	public SL	axis_e_motor_position; //!<E axis motor position.
	    /*168-171*/	public SL	axis_e_position_error; //!<E axis position error.
	    /*172-175*/	public SL	axis_e_aux_position; //!<E axis auxiliary position.
	    /*176-179*/	public SL	axis_e_velocity; //!<E axis velocity.
	    /*180-181*/	public SW	axis_e_torque; //!<E axis torque.
	    /*182-183*/	public UW  axis_e_analog_in; //!<E axis analog input.
                     
	    /*184-185*/	public UW	axis_f_status; //!<F axis status.
	    /*186*/	    public UB	axis_f_switches; //!<F axis switches.
	    /*187*/	    public UB	axis_f_stop_code; //!<F axis stop code.
	    /*188-191*/	public SL	axis_f_reference_position; //!<F axis reference position.
	    /*192-195*/	public SL	axis_f_motor_position; //!<F axis motor position.
	    /*196-199*/	public SL	axis_f_position_error; //!<F axis position error.
	    /*200-203*/	public SL	axis_f_aux_position; //!<F axis auxiliary position.
	    /*204-207*/	public SL	axis_f_velocity; //!<F axis velocity.
	    /*208-209*/	public SW	axis_f_torque; //!<F axis torque.
	    /*210-211*/	public UW	axis_f_analog_in; //!<F axis analog input.
                     
	    /*212-213*/	public UW	axis_g_status; //!<G axis status.
	    /*214*/  	public UB	axis_g_switches; //!<G axis switches.
	    /*215*/ 	public UB	axis_g_stop_code; //!<G axis stop code.
	    /*216-219*/	public SL	axis_g_reference_position; //!<G axis reference position.
	    /*220-223*/	public SL	axis_g_motor_position; //!<G axis motor position.
	    /*224-227*/	public SL	axis_g_position_error; //!<G axis position error.
	    /*228-231*/	public SL	axis_g_aux_position; //!<G axis auxiliary position.
	    /*232-235*/	public SL	axis_g_velocity; //!<G axis velocity.
	    /*236-237*/	public SW	axis_g_torque; //!<G axis torque.
	    /*238-239*/ public UW  axis_g_analog_in; //!<G axis analog input.
                     
	    /*240-241*/	public UW	axis_h_status; //!<H axis status.
	    /*242*/  	public UB	axis_h_switches; //!<H axis switches.
	    /*243*/	    public UB	axis_h_stop_code; //!<H axis stop code.
	    /*244-247*/	public SL	axis_h_reference_position; //!<H axis reference position.
	    /*248-251*/	public SL	axis_h_motor_position; //!<H axis motor position.
	    /*252-255*/	public SL	axis_h_position_error; //!<H axis position error.
	    /*256-259*/	public SL	axis_h_aux_position; //!<H axis auxiliary position.
	    /*260-263*/	public SL	axis_h_velocity; //!<H axis velocity.
	    /*264-265*/	public SW	axis_h_torque; //!<H axis torque.
	    /*266-267*/	public UW  axis_h_analog_in; //!<H axis analog input.
    }; //DataRecord2013

    /// <summary>Data record struct for DMC-1802 controllers.</summary> 
    /// <remarks>
    /// The 18x2 Data record is the Same as 2103 except the following.
    /// -# No header bytes. Software removes it from QR.
    /// -# No analog in axis data.
    /// </remarks>    
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord1802 : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }

        /*Offset   type name        description*/

        /*00-01*/   public UW	sample_number; //!<sample number.
                     
	    /*02*/	    public UB	input_bank_0; //!<general input bank 0 (inputs 1-8).
	    /*03*/	    public UB	input_bank_1; //!<general input bank 1 (inputs 9-16).
	    /*04*/	    public UB	input_bank_2; //!<general input bank 2 (inputs 17-24).
	    /*05*/	    public UB	input_bank_3; //!<general input bank 3 (inputs 25-32).
	    /*06*/    	public UB	input_bank_4; //!<general input bank 4 (inputs 33-40).
	    /*07*/	    public UB	input_bank_5; //!<general input bank 5 (inputs 41-48).
	    /*08*/	    public UB	input_bank_6; //!<general input bank 6 (inputs 49-56).
	    /*09*/   	public UB	input_bank_7; //!<general input bank 7 (inputs 57-64).
	    /*10*/   	public UB	input_bank_8; //!<general input bank 8 (inputs 65-72).
	    /*11*/	    public UB	input_bank_9; //!<general input bank 9 (inputs 73-80).
                     
	    /*12*/    	public UB	output_bank_0; //!<general output bank 0 (outputs 1-8).
	    /*13*/	    public UB	output_bank_1; //!<general output bank 1 (outputs 9-16).
	    /*14*/   	public UB	output_bank_2; //!<general output bank 2 (outputs 17-24).
	    /*15*/	    public UB	output_bank_3; //!<general output bank 3 (outputs 25-32).
	    /*16*/   	public UB	output_bank_4; //!<general output bank 4 (outputs 33-40).
	    /*17*/	    public UB	output_bank_5; //!<general output bank 5 (outputs 41-48).
	    /*18*/	    public UB	output_bank_6; //!<general output bank 6 (outputs 49-56).
	    /*19*/	    public UB	output_bank_7; //!<general output bank 7 (outputs 57-64).
	    /*20*/  	public UB	output_bank_8; //!<general output bank 8 (outputs 65-72).
	    /*21*/  	public UB	output_bank_9; //!<general output bank 9 (outputs 73-80).
                     
	    /*22*/	    public UB	error_code; //!<error code.
	    /*23*/  	public UB	general_status; //!<general status
                     
	    /*24-25*/	public UW	s_plane_segment_count; //!<segment count of coordinated move for S plane.
	    /*26-27*/	public UW	s_plane_move_status; //!<coordinated move status for S plane.
	    /*28-31*/	public SL	s_distance; //!<distance traveled in coordinated move for S plane.
                     
	    /*32-33*/	public UW	t_plane_segment_count; //!<segment count of coordinated move for T plane.
	    /*34-35*/	public UW	t_plane_move_status; //!<Coordinated move status for T plane.
	    /*36-39*/	public SL	t_distance; //!<distance traveled in coordinated move for T plane.
                     
	    /*40-41*/	public UW	axis_a_status; //!<A axis status.
	    /*42*/	    public UB	axis_a_switches; //!<A axis switches.
	    /*43*/	    public UB	axis_a_stop_code; //!<A axis stop code.
	    /*44-47*/	public SL	axis_a_reference_position; //!<A axis reference position.
	    /*48-51*/	public SL	axis_a_motor_position; //!<A axis motor position.
	    /*52-55*/	public SL	axis_a_position_error; //!<A axis position error.
	    /*56-59*/	public SL	axis_a_aux_position; //!<A axis auxiliary position.
	    /*60-63*/	public SL	axis_a_velocity; //!<A axis velocity.
	    /*64-65*/	public SW 	axis_a_torque; //!<A axis torque.
	    /*66*/	    public UB	axis_a_reserved_0; //!<Reserved.
	    /*67*/      public UB	axis_a_reserved_1; //!<Reserved.
                     
	    /*68-69*/   public UW	axis_b_status; //!<B axis status.
	    /*70*/	    public UB	axis_b_switches; //!<B axis switches.
	    /*71*/	    public UB	axis_b_stop_code; //!<B axis stop code.
	    /*72-75*/	public SL	axis_b_reference_position; //!<B axis reference position.
	    /*76-79*/	public SL	axis_b_motor_position; //!<B axis motor position.
	    /*80-83*/	public SL	axis_b_position_error; //!<B axis position error.
	    /*84-87*/	public SL	axis_b_aux_position; //!<B axis auxiliary position.
	    /*88-91*/	public SL	axis_b_velocity; //!<B axis velocity.
	    /*92-93*/	public SW	axis_b_torque; //!<B axis torque.
	    /*94*/	    public UB	axis_b_reserved_0; //!<Reserved.
	    /*95*/      public UB	axis_b_reserved_1; //!<Reserved.
                     
	    /*96-97*/	public UW	axis_c_status; //!<C axis status.
	    /*98*/  	public UB	axis_c_switches; //!<C axis switches.
	    /*99*/		public UB	axis_c_stop_code; //!<C axis stop code.
	    /*100-103*/	public SL	axis_c_reference_position; //!<C axis reference position.
	    /*104-107*/	public SL	axis_c_motor_position; //!<C axis motor position.
	    /*108-111*/	public SL	axis_c_position_error; //!<C axis position error.
	    /*112-115*/	public SL	axis_c_aux_position; //!<C axis auxiliary position.
	    /*116-119*/	public SL	axis_c_velocity; //!<C axis velocity.
	    /*120-121*/	public SW	axis_c_torque; //!<C axis torque.
	    /*122*/	    public UB	axis_c_reserved_0; //!<Reserved.
	    /*123*/     public UB	axis_c_reserved_1; //!<Reserved.
                     
	    /*124-125*/	public UW	axis_d_status; //!<D axis status.
	    /*126*/	    public UB	axis_d_switches; //!<D axis switches.
	    /*127*/  	public UB	axis_d_stop_code; //!<D axis stop code.
	    /*128-131*/	public SL	axis_d_reference_position; //!<D axis reference position.
	    /*132-135*/	public SL	axis_d_motor_position; //!<D axis motor position.
	    /*136-139*/	public SL	axis_d_position_error; //!<D axis position error.
	    /*140-143*/	public SL	axis_d_aux_position; //!<D axis auxiliary position.
	    /*144-147*/	public SL	axis_d_velocity; //!<D axis velocity.
	    /*148-149*/	public SW	axis_d_torque; //!<D axis torque.
	    /*150*/	    public UB	axis_d_reserved_0; //!<Reserved.
	    /*151*/     public UB	axis_d_reserved_1; //!<Reserved.

    }; //DataRecord1802

    //! Data record struct for DMC-30010 controllers. 
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord30000 : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }

        /*Offset   type name        description*/

        /*00*/      public UB	header_0; //!<1st Byte of Header.
	    /*01*/	    public UB	header_1; //!<2nd Byte of Header.
	    /*02*/	    public UB	header_2; //!<3rd Byte of Header.
	    /*03*/	    public UB	header_3; //!<4th Byte of Header.
                     
	    /*04-05*/  	public UW	sample_number; //!<sample number.
                     
	    /*06*/	    public UB	input_bank_0; //!<general input bank 0 (inputs 1-8).
	    /*07*/	    public UB	input_bank_1; //!<general input bank 1 (inputs 9-16).
                     
	    /*08*/    	public UB	output_bank_0; //!<general output bank 0 (outputs 1-8).
	    /*09*/	    public UB	output_bank_1; //!<general output bank 1 (outputs 9-16).
                     
	    /*10*/	    public UB	error_code; //!<error code.
	    /*11*/  	public UB	thread_status; //!<thread status.
                     
	    /*12-13*/	public UW	input_analog_2; //!< Analog input 2. 1 is in axis data, see axis_a_analog_in.
                     
	    /*14-15*/	public UW	output_analog_1; //!< Analog output 1.
	    /*16-17*/	public UW	output_analog_2; //!< Analog output 2.
                     
	    /*18-21*/	public UL	amplifier_status; //!<Amplifier Status.
                     
	    /*22-25*/	public UL	contour_segment_count; //!<Segment Count for Contour Mode.
	    /*26-27*/	public UW	contour_buffer_available; //!<Buffer space remaining, Contour Mode.
                     
	    /*28-29*/	public UW	s_plane_segment_count; //!<segment count of coordinated move for S plane.
	    /*30-31*/	public UW	s_plane_move_status; //!<coordinated move status for S plane.
	    /*32-35*/	public SL	s_distance; //!<distance traveled in coordinated move for S plane.
	    /*36-37*/	public UW	s_plane_buffer_available; //!<Buffer space remaining, S Plane.
                     
	    /*38-39*/	public UW	axis_a_status; //!<A axis status.
	    /*40*/	    public UB	axis_a_switches; //!<A axis switches.
	    /*41*/	    public UB	axis_a_stop_code; //!<A axis stop code.
	    /*42-45*/	public SL	axis_a_reference_position; //!<A axis reference position.
	    /*46-49*/	public SL	axis_a_motor_position; //!<A axis motor position.
	    /*50-53*/	public SL	axis_a_position_error; //!<A axis position error.
	    /*54-57*/	public SL	axis_a_aux_position; //!<A axis auxiliary position.
	    /*58-61*/	public SL	axis_a_velocity; //!<A axis velocity.
	    /*62-65*/	public SL 	axis_a_torque; //!<A axis torque.
	    /*66-67*/	public UW  axis_a_analog_in; //!<A axis analog input.
	    /*68*/	    public UB	axis_a_halls; //!<A Hall Input Status.
	    /*69*/  	public UB	axis_a_reserved; //!<Reserved.
	    /*70-73*/	public SL	axis_a_variable; //!<A User-defined variable (ZA).
    }; //DataRecord30000

    //! Data record struct for RIO-471xx and RIO-472xx PLCs. Includes encoder fields.
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord47000_ENC : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }

        /*Offset   type name        description*/

        /*00*/      public UB	header_0; //!< 1st Byte of Header.
	    /*01*/	    public UB	header_1; //!< 2nd Byte of Header.
	    /*02*/	    public UB	header_2; //!< 3rd Byte of Header.
	    /*03*/	    public UB	header_3; //!< 4th Byte of Header.
                     
	    /*04-05*/  	public UW	sample_number; //!< Sample number.
	    /*06*/	    public UB	error_code; //!< Error code.
	    /*07*/  	public UB	general_status; //!< General status.
                     
	    /*08-09*/	public UW	output_analog_0; //!< Analog output 0.
	    /*10-11*/	public UW	output_analog_1; //!< Analog output 1.
	    /*12-13*/	public UW	output_analog_2; //!< Analog output 2.
	    /*14-15*/	public UW	output_analog_3; //!< Analog output 3.
	    /*16-17*/	public UW	output_analog_4; //!< Analog output 4.
	    /*18-19*/	public UW	output_analog_5; //!< Analog output 5.
	    /*20-21*/	public UW	output_analog_6; //!< Analog output 6.
	    /*22-23*/	public UW	output_analog_7; //!< Analog output 7.
                     
	    /*24-25*/	public UW	input_analog_0; //!< Analog input 0.
	    /*26-27*/	public UW	input_analog_1; //!< Analog input 1.
	    /*28-29*/	public UW	input_analog_2; //!< Analog input 2.
	    /*30-31*/	public UW	input_analog_3; //!< Analog input 3.
	    /*32-33*/	public UW	input_analog_4; //!< Analog input 4.
	    /*34-35*/	public UW	input_analog_5; //!< Analog input 5.
	    /*36-37*/	public UW	input_analog_6; //!< Analog input 6.
	    /*38-39*/	public UW	input_analog_7; //!< Analog input 7.
                     
	    /*40-41*/	public UW	output_bank_0; //!< Digital outputs 0-15;
                     
	    /*42-43*/	public UW	input_bank_0; //!< Digital inputs 0-15;
                     
	    /*44-47*/	public UL	pulse_count_0; //!< Pulse counter (see PC).
	    /*48-51*/	public SL	zc_variable; //!< ZC User-defined variable (see ZC). 
	    /*52-55*/	public SL	zd_variable; //!< ZD User-defined variable (see ZD).
                     
	    /*56-59*/	public SL	encoder_0; //!< Encoder channel 0. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*60-63*/	public SL	encoder_1; //!< Encoder channel 1. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*64-67*/	public SL	encoder_2; //!< Encoder channel 2. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*68-71*/	public SL	encoder_3; //!< Encoder channel 3. Data only valid for parts with -BISS, -QUAD, or -SSI.

    }; //GDataRecord47000_ENC

    //! Data record struct for RIO-47300. Includes encoder fields.
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord47300_ENC : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }

        /*Offset   type name        description*/

        /*00*/      public UB	header_0; //!< 1st Byte of Header.
	    /*01*/	    public UB	header_1; //!< 2nd Byte of Header.
	    /*02*/	    public UB	header_2; //!< 3rd Byte of Header.
	    /*03*/	    public UB	header_3; //!< 4th Byte of Header.
                     
	    /*04-05*/  	public UW	sample_number; //!< Sample number.
	    /*06*/	    public UB	error_code; //!< Error code.
	    /*07*/  	public UB	general_status; //!< General status.
                     
	    /*08-09*/	public UW	output_analog_0; //!< Analog output 0.
	    /*10-11*/	public UW	output_analog_1; //!< Analog output 1.
	    /*12-13*/	public UW	output_analog_2; //!< Analog output 2.
	    /*14-15*/	public UW	output_analog_3; //!< Analog output 3.
	    /*16-17*/	public UW	output_analog_4; //!< Analog output 4.
	    /*18-19*/	public UW	output_analog_5; //!< Analog output 5.
	    /*20-21*/	public UW	output_analog_6; //!< Analog output 6.
	    /*22-23*/	public UW	output_analog_7; //!< Analog output 7.
                     
	    /*24-25*/	public UW	input_analog_0; //!< Analog input 0.
	    /*26-27*/	public UW	input_analog_1; //!< Analog input 1.
	    /*28-29*/	public UW	input_analog_2; //!< Analog input 2.
	    /*30-31*/	public UW	input_analog_3; //!< Analog input 3.
	    /*32-33*/	public UW	input_analog_4; //!< Analog input 4.
	    /*34-35*/	public UW	input_analog_5; //!< Analog input 5.
	    /*36-37*/	public UW	input_analog_6; //!< Analog input 6.
	    /*38-39*/	public UW	input_analog_7; //!< Analog input 7.
                     
	    /*40-41*/	public UW	output_bank_0; //!< Digital outputs 0-15;
	    /*42-43*/	public UW	output_bank_1; //!< Digital outputs 16-23;
                     
	    /*44-45*/	public UW	input_bank_0; //!< Digital inputs 0-15;
	    /*46-47*/	public UW	input_bank_1; //!< Digital inputs 16-23;
                     
	    /*48-51*/	public UL	pulse_count_0; //!< Pulse counter (see PC).
	    /*52-55*/	public SL	zc_variable; //!< ZC User-defined variable (see ZC). 
	    /*56-59*/	public SL	zd_variable; //!< ZD User-defined variable (see ZD).
                     
	    /*60-63*/	public SL	encoder_0; //!< Encoder channel 0. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*64-67*/	public SL	encoder_1; //!< Encoder channel 1. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*68-71*/	public SL	encoder_2; //!< Encoder channel 2. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*72-75*/	public SL	encoder_3; //!< Encoder channel 3. Data only valid for parts with -BISS, -QUAD, or -SSI.

    }; //GDataRecord47300_ENC

    //! Data record struct for RIO-47300 with 24EX I/O daughter board.
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord47300_24EX : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }

        /*Offset   type name        description*/

        /*00*/      public UB	header_0; //!< 1st Byte of Header.
	    /*01*/	    public UB	header_1; //!< 2nd Byte of Header.
	    /*02*/	    public UB	header_2; //!< 3rd Byte of Header.
	    /*03*/	    public UB	header_3; //!< 4th Byte of Header.
                     
	    /*04-05*/  	public UW	sample_number; //!< Sample number.
	    /*06*/	    public UB	error_code; //!< Error code.
	    /*07*/  	public UB	general_status; //!< General status.
                     
	    /*08-09*/	public UW	output_analog_0; //!< Analog output 0.
	    /*10-11*/	public UW	output_analog_1; //!< Analog output 1.
	    /*12-13*/	public UW	output_analog_2; //!< Analog output 2.
	    /*14-15*/	public UW	output_analog_3; //!< Analog output 3.
	    /*16-17*/	public UW	output_analog_4; //!< Analog output 4.
	    /*18-19*/	public UW	output_analog_5; //!< Analog output 5.
	    /*20-21*/	public UW	output_analog_6; //!< Analog output 6.
	    /*22-23*/	public UW	output_analog_7; //!< Analog output 7.
                     
	    /*24-25*/	public UW	input_analog_0; //!< Analog input 0.
	    /*26-27*/	public UW	input_analog_1; //!< Analog input 1.
	    /*28-29*/	public UW	input_analog_2; //!< Analog input 2.
	    /*30-31*/	public UW	input_analog_3; //!< Analog input 3.
	    /*32-33*/	public UW	input_analog_4; //!< Analog input 4.
	    /*34-35*/	public UW	input_analog_5; //!< Analog input 5.
	    /*36-37*/	public UW	input_analog_6; //!< Analog input 6.
	    /*38-39*/	public UW	input_analog_7; //!< Analog input 7.
                     
	    /*40-41*/	public UW	output_bank_0; //!< Digital outputs 0-15.
	    /*42-43*/	public UW	output_bank_1; //!< Digital outputs 16-23.
                     
	    /*44-45*/	public UW	input_bank_0; //!< Digital inputs 0-15.
	    /*46-47*/	public UW	input_bank_1; //!< Digital inputs 16-23.
                     
	    /*48-51*/	public UL	pulse_count_0; //!< Pulse counter (see PC)8.
	    /*52-55*/	public SL	zc_variable; //!< ZC User-defined variable (see ZC). 
	    /*56-59*/	public SL	zd_variable; //!< ZD User-defined variable (see ZD).
                     
	    /*60-61*/	public UW output_bank_2; //!< Digital outputs 24-39. Data only valid for parts with 24EXOUT.
	    /*62-63*/	public UW output_back_3; //!< Digital outputs 40-47. Data only valid for parts with 24EXOUT.
                     
	    /*64-65*/	public UW input_bank_2; //!< Digital inputs 24-39. Data only valid for parts with 24EXIN.
	    /*66-67*/	public UW input_bank_3; //!< Digital inputs 40-47. Data only valid for parts with 24EXIN.

    }; //GDataRecord47300_24EX

    //! Data record struct for RIO-47162.
    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct GDataRecord47162 : GDataRecord
    {
        public byte[] byte_array() { return StructToByteArray(this); }
        /*Offset   type name        description*/

        /*00*/      public UB	header_0; //!< 1st Byte of Header.
	    /*01*/	    public UB	header_1; //!< 2nd Byte of Header.
	    /*02*/	    public UB	header_2; //!< 3rd Byte of Header.
	    /*03*/	    public UB	header_3; //!< 4th Byte of Header.
                     
	    /*04-05*/  	public UW	sample_number; //!< Sample number.
	    /*06*/	    public UB	error_code; //!< Error code.
	    /*07*/  	public UB	general_status; //!< General status.
                     
	    /*08-09*/	public UW	output_analog_0; //!< Analog output 0.
	    /*10-11*/	public UW	output_analog_1; //!< Analog output 1.
	    /*12-13*/	public UW	output_analog_2; //!< Analog output 2.
	    /*14-15*/	public UW	output_analog_3; //!< Analog output 3.
	    /*16-17*/	public UW	output_analog_4; //!< Analog output 4.
	    /*18-19*/	public UW	output_analog_5; //!< Analog output 5.
	    /*20-21*/	public UW	output_analog_6; //!< Analog output 6.
	    /*22-23*/	public UW	output_analog_7; //!< Analog output 7.
                     
	    /*24-25*/	public UW	input_analog_0; //!< Analog input 0.
	    /*26-27*/	public UW	input_analog_1; //!< Analog input 1.
	    /*28-29*/	public UW	input_analog_2; //!< Analog input 2.
	    /*30-31*/	public UW	input_analog_3; //!< Analog input 3.
	    /*32-33*/	public UW	input_analog_4; //!< Analog input 4.
	    /*34-35*/	public UW	input_analog_5; //!< Analog input 5.
	    /*36-37*/	public UW	input_analog_6; //!< Analog input 6.
	    /*38-39*/	public UW	input_analog_7; //!< Analog input 7.
                     
	    /*40*/	    public UB	output_byte_0; //!< Digital outputs 0-7.
	    /*41*/	    public UB	output_byte_1; //!< Digital outputs 8-15.
	    /*42*/	    public UB	output_byte_2; //!< Digital outputs 16-23.
	                 
	    /*43*/	    public UB	input_byte_0; //!< Digital inputs 0-7.
	    /*44*/	    public UB	input_byte_1; //!< Digital inputs 8-15.
	    /*45*/	    public UB	input_byte_2; //!< Digital inputs 16-23.
	    /*46*/	    public UB	input_byte_3; //!< Digital inputs 24-31.
	    /*47*/	    public UB	input_byte_4; //!< Digital inputs 32-39.
                     
	    /*48-51*/	public UL	pulse_count_0; //!< Pulse counter (see PC).
	    /*52-55*/	public SL	zc_variable; //!< ZC User-defined variable (see ZC). 
	    /*56-59*/	public SL	zd_variable; //!< ZD User-defined variable (see ZD).
                     
	    /*60-63*/	public SL	encoder_0; //!< Encoder channel 0. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*64-67*/	public SL	encoder_1; //!< Encoder channel 1. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*68-71*/	public SL	encoder_2; //!< Encoder channel 2. Data only valid for parts with -BISS, -QUAD, or -SSI.
	    /*72-75*/	public SL	encoder_3; //!< Encoder channel 3. Data only valid for parts with -BISS, -QUAD, or -SSI.

    }; //GDataRecord47162

    #endregion
} //gclib class

/** @}*/
