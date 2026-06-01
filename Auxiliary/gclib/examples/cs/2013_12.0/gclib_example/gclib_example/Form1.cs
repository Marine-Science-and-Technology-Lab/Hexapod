using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace gclib_example
{
    /// <summary>
    /// Demonstrates using gclib in a Windows Form, including using a second thread to free the GUI.
    /// </summary>
    public partial class MainForm : Form
    {

        #region "UI"

        //form's ctor
        public MainForm()
        {
            InitializeComponent();
        }

        //Runs when form loads
        private void MainForm_Load(object sender, EventArgs e)
        {
            PrintOutput("Enter a FULL GOpen() address above and click Go", PrintStyle.Instruction);
            PrintOutput("NOTE: This demo will attempt to move Axis A", PrintStyle.Instruction);
        }

        // Opens Galil's help to show GOpen() options
        private void HelpLabel_Click(object sender, EventArgs e)
        {
            //link to GOpen() documentation.
            System.Diagnostics.Process.Start("http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h_aef4aec8a85630eed029b7a46aea7db54.html#aef4aec8a85630eed029b7a46aea7db54");
        }

        //Runs when user clicks Go button
        private void GoButton_Click(object sender, EventArgs e)
        {
            if (AddressTextBox.Text.Length == 0)
            {
                PrintOutput("Enter a FULL GOpen() address above and click Go", PrintStyle.Instruction);
                return;
            }
            RunDemo(AddressTextBox.Text);
        }

        //Various print styles.
        private enum PrintStyle
        {
            Instruction,
            Normal,
            GalilData,
            GclibData,
            Err,
        }

        /// <summary>
        /// Thread safe printing call.
        /// </summary>
        /// <param name="Message">The message to print</param>
        /// <param name="Style">The style enum to indicate how to print</param>
        /// <param name="SuppressCrLf">If true, the string will be printed without a trailing cr+lf</param>
        private void PrintOutput(string Message, PrintStyle Style = PrintStyle.Normal, bool SuppressCrLf = false)
        {
            if (Output.InvokeRequired)
            {
                Output.Invoke(new Printer(PrintOutput), new object[] { Message, Style, SuppressCrLf });
            }
            else
            {
                Color color;

                switch (Style)
                {
                    case PrintStyle.Instruction:
                        color = Color.Black;
                        break;
                    case PrintStyle.GalilData:
                        color = Color.Green;
                        break;
                    case PrintStyle.Normal:
                        color = Color.Blue;
                        break;
                    case PrintStyle.Err:
                        color = Color.Red;
                        break;
                    case PrintStyle.GclibData:
                        color = Color.Magenta;
                        break;
                    default:
                        color = Color.Blue;
                        break;
                }//switch

                Output.SelectionStart = Output.Text.Length;
                Output.SelectionColor = color;
                Output.AppendText(Message);

                if (!SuppressCrLf)
                    Output.AppendText("\r\n");

            }//invoke check
        }

        #endregion

        #region "Threading"

        /// <summary>
        /// Delegate used to print status when the status is generated in a thread other than the UI thread.
        /// </summary>
        /// <param name="Message">Message to print</param>
        /// <param name="Style">Print Style</param>
        /// <param name="SuppressCrLf">If true, the string will be printed without a trailing cr+lf</param>
        private delegate void Printer(string Message, PrintStyle Style, bool SuppressCrLf);

        /// <summary>
        /// Fires up the demo via the background worker thread
        /// </summary>
        /// <param name="address">The full GOpen() addresss<</param>
        /// <remarks>Runs in UI thread</remarks>
        private void RunDemo(string address)
        {
            MainToolStrip.Enabled = false;
            Output.Clear();
            GclibBackgroundWorker.RunWorkerAsync(address);
        }


        /// <summary>
        /// Runs in second thread to call the demo.
        /// </summary>
        private void GclibBackgroundWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            PrintOutput("Running Demo with address " + e.Argument, PrintStyle.Normal);
            TheDemo((string)e.Argument); //call the actual demo code
        }

        /// <summary>
        /// Runs in the main thread after the second thread returns.
        /// </summary>
        private void GclibBackgroundWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            PrintOutput("Demo thread done.", PrintStyle.Normal);
            MainToolStrip.Enabled = true;
        }

        #endregion

        #region "Demo Code"

        private void TheDemo(string address)
        {
            gclib gclib = null;
            try
            {
                gclib = new gclib(); //constructor can throw, so keep it in a Try block

                PrintOutput("gclib version: ", PrintStyle.Normal, true);
                PrintOutput(gclib.GVersion(), PrintStyle.GclibData);

                //*** Uncomment below for network utilities ***
                //PrintOutput("Controllers requesting IP addresses...");
                //string[] macs = gclib.GIpRequests();
                //if (macs.Length == 0)
                //    PrintOutput("None");
                //else
                //    foreach (string m in macs)
                //        PrintOutput(m);

                //gclib.GAssign("192.168.0.42", "00:50:4c:20:01:23"); //Assign an IP to an unassigned controller

                PrintOutput("Available connections:");
                string[] addrs = gclib.GAddresses();
                if (addrs.Length == 0)
                {
                    PrintOutput("None");
                }
                else
                {
                    foreach (string a in addrs)
                    {
                        PrintOutput(a, PrintStyle.GclibData);
                    }
                }

                PrintOutput("Opening connection to \"" + address + "\"... ", PrintStyle.Normal, true);
                gclib.GOpen(address);
                PrintOutput("Connected.", PrintStyle.Normal);
                PrintOutput(gclib.GInfo(), PrintStyle.GalilData);

               // gclib.GCommand("BN"); //send BN if IP address was assigned above

                PrintOutput("Sending \"MG TIME\"", PrintStyle.Normal);
                PrintOutput(gclib.GCommand("MG TIME", false), PrintStyle.GalilData);

                PrintOutput("Downloading Program... ", PrintStyle.Normal, true);
                gclib.GProgramDownload("i=0\r#A;MG i{N};i=i+1;WT10;JP#A,i<10;EN", "");

                PrintOutput("Uploading Program");
                PrintOutput(gclib.GProgramUpload(), PrintStyle.GalilData);

                PrintOutput("Blocking GMessage call");
                gclib.GCommand("XQ");
                System.Threading.Thread.Sleep(200);
                //wait a bit to queue up some messages
                PrintOutput(gclib.GMessage(), PrintStyle.GalilData);
                //get them all in one blocking read

                PrintOutput("Downloading Program... ", PrintStyle.Normal, true);
                gclib.GProgramDownload("WT 1000; MG TIME; EN", "");
                //prints a messsage after 1 second

                PrintOutput("Uploading Program");
                PrintOutput(gclib.GProgramUpload(), PrintStyle.GalilData);

                PrintOutput("Non-blocking GMessage call", PrintStyle.Normal, true);
                gclib.GCommand("XQ");
                gclib.GTimeout(0);
                //set a zero timeout for a non-blocking read
                string msg = "";
                while ((string.IsNullOrEmpty(msg)))
                {
                    msg = gclib.GMessage();
                    PrintOutput(".", PrintStyle.Normal, true);
                    System.Threading.Thread.Sleep(20);
                    //do something useful here...
                }
                PrintOutput("Message: ", PrintStyle.Normal, true);
                PrintOutput(msg.Trim(), PrintStyle.GalilData);
                gclib.GTimeout(-1);
                //put the timeout back
                //NOTE: Both GRecord and GInterrupt also have non-blocking mode with 0 timeout.

                PrintOutput("Downloading Program... ", PrintStyle.Normal, true);
                gclib.GProgramDownload("WT 1000; UI 8; EN", "");
                //fires an interrupt after 1 second

                PrintOutput("Uploading Program");
                PrintOutput(gclib.GProgramUpload(), PrintStyle.GalilData);

                PrintOutput("Non-blocking GInterrupt call", PrintStyle.Normal, true);
                gclib.GCommand("XQ");
                gclib.GTimeout(0);
                //set a zero timeout for a non-blocking read
                byte b = 0;
                while ((b == 0))
                {
                    b = gclib.GInterrupt();
                    PrintOutput(".", PrintStyle.Normal, true);
                    System.Threading.Thread.Sleep(20);
                    //do something useful here...
                }
                PrintOutput("Byte: ", PrintStyle.Normal, true);
                PrintOutput(b.ToString("X02"), PrintStyle.GalilData);
                gclib.GTimeout(-1);
                //put the timeout back

                PrintOutput("Getting some synchronous data records");
                gclib.GDataRecord4000 DataRecord;
                for (int i = 0; i <= 10; i++)
                {
                    DataRecord = gclib.GRecord<gclib.GDataRecord4000>(false);
                    PrintOutput(DataRecord.sample_number + " ", PrintStyle.GalilData, true);
                    //need help accessing the data record? Contact softwaresupport@galil.com
                    System.Threading.Thread.Sleep(10);
                }
                PrintOutput("");

                PrintOutput("Getting some asynchronous data records");
                gclib.GRecordRate(10);
                //set up data records every 10 ms
                for (int i = 0; i <= 10; i++)
                {
                    DataRecord = gclib.GRecord<gclib.GDataRecord4000>(true);
                    PrintOutput(DataRecord.sample_number + " ", PrintStyle.GalilData, true);
                    //no need to delay, asynchronous mode is dispatched by the Galil's RTOS.
                }
                gclib.GRecordRate(0);
                //turn off data records
                PrintOutput("");

                PrintOutput("Downloading an array... ", PrintStyle.Normal, true);
                List<double> array = new List<double>();
                for (double i = 0; i <= 9; i++)
                {
                    array.Add(i * 2);
                }
                gclib.GCommand("DA *[];DM array[10]");
                //arrays must be dimensioned prior to download
                gclib.GArrayDownload("array", ref array);

                PrintOutput("Ok. Uploading array");
                array = gclib.GArrayUpload("array");
                foreach (double d in array)
                {
                    PrintOutput(d.ToString("F4") + " ", PrintStyle.GalilData, true);
                }
                PrintOutput("");

                PrintOutput("Performing a write... ", PrintStyle.Normal, true);
                gclib.GWrite("QR\r");
                //QR returns the binary data record
                PrintOutput("Ok. Reading binary data... ", PrintStyle.Normal, true);
                byte[] data = gclib.GRead();
                PrintOutput("Ok. Read " + data.Length + " bytes.");

                PrintOutput("Preparing A axis. This could cause errors if the axis is not initialized...", PrintStyle.Normal, true);
                gclib.GCommand("AB;MO;SHA");
                //compound commands are possible though typically not recommended
                PrintOutput("Ok");
                gclib.GCommand("PRA=5000");
                gclib.GCommand("SPA=5000");
                PrintOutput("Profiling a move on axis A... ", PrintStyle.Normal, true);
                gclib.GCommand("BGA");
                PrintOutput("Waiting for motion to complete... ", PrintStyle.Normal, true);
                gclib.GMotionComplete("A");
                PrintOutput("done");
                PrintOutput("Going back... ", PrintStyle.Normal, true);
                gclib.GCommand("PRA=-5000");
                gclib.GCommand("BGA");
                gclib.GMotionComplete("A");
                PrintOutput("done");
            }
            catch (Exception ex)
            {
                PrintOutput("Error: " + ex.Message, PrintStyle.Err);
            }
            finally
            {
                if (gclib != null)
                    gclib.GClose(); //don't forget to close the connection
            }
        }
        #endregion
    }
}
