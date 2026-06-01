package gclibtest;

import gclibjava.GclibJava; //the java gclib wrapper
import gclibjava.GclibJavaException; //the Galil exception class
import java.util.ArrayList;
import java.util.List;

/**
 * A test of the Java gclib wrapper.
 */
public class GclibTest {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {              
 
        System.out.println("Java gclib Test");
        test();
        System.out.println("Done"); 
         
    }
    
    static void test()
    {
         GclibJava gclib = new GclibJava(); //our instance of the GclibJava class
        
         try
         {
             System.out.println("gclib version " + gclib.GVersion());
             
             /*
             System.out.println("Hardware Requesting IP Addresses:");
             System.out.print(gclib.GIpRequests());
             System.out.println();
             */
             
             /*
             System.out.println("Available addresses:");
             System.out.print(gclib.GAddresses());
             System.out.println();
             */
             
             /*
             System.out.println("Assigning IP Address");
             gclib.GAssign("192.168.0.42", "00-50-4C-20-01-23");
             */
             
             System.out.println("Connecting");
             gclib.GOpen("192.168.0.42 -s ALL"); //connect to hardware, through gcaps
             //see http://galil.com/sw/pub/all/doc/gclib/html/gcaps.html    
             
             System.out.println(gclib.GInfo());
                        
             System.out.println("MG TIME: " + gclib.GCommand("MG TIME"));    
                         
             System.out.println("Downloading program");
             gclib.GProgramDownload("#A\rWT100\rMG\"Hello from program!\",TIME\rEN");
             //gclib.GProgramDownloadFile("c:\\temp\\test.dmc");
             
             System.out.println("Program now on hardware\n********************");
             System.out.print(gclib.GProgramUpload());
             //gclib.GProgramUploadFile("c:\\temp\\uploaded.dmc");
             System.out.println("\n********************");
             
             /*
             gclib.GTimeout((short) 10000); //adjust timeout for long command
             gclib.GCommand("BP"); //burn program
             gclib.GTimeout((short) -1); //use initial GOpen() timeout (--timeout)
             */
             
             System.out.println("Executing program");
             gclib.GCommand("XQ"); //execute program that was just downloaded
             
             System.out.println("Reading for a message");
             System.out.println(gclib.GMessage()); //print the message the program generated
             
             System.out.println("Downloading array data");
             List<Double> doubleList = new ArrayList();
             for (double i = 0; i < 10.0; ++i)
             {
                 doubleList.add(i);
             }
             gclib.GCommand("DA*[]"); //deallocate all array data
             gclib.GCommand("DM array[10]"); //dimension the array
             gclib.GArrayDownload("array", doubleList);
             //gclib.GArrayDownloadFile("c:\\temp\\arrays.csv");
             
             System.out.println("Uploading array data");
             List<Double> arrayUpload = gclib.GArrayUpload("array");
             //gclib.GArrayUploadFile("c:\\temp\\upload_arrays.csv", "array");
             for (Double d : arrayUpload)
             {
                 System.out.println(d.toString());
             }
             
             /*
             gclib.GCommand("UI 1"); //should evoke $F1 from products that 
             //support interrupts. Look for the existance of UI in the hardware
             //command reference.
             
             System.out.println("Interrupt status byte: " +
                     String.format("%02X", gclib.GInterrupt()));
             */
                                                                
             /*
             //loading firmware
             System.out.println("Firmware loader test");
             System.out.println("Currently loaded firmware: " + gclib.GCommand("\u0012\u0016")); //^R^V
             gclib.GFirmwareDownload("c:\\temp\\dmc-4000-r12h-cer.hex");
             System.out.println("Currently loaded firmware: " + gclib.GCommand("\u0012\u0016")); //^R^V
             gclib.GFirmwareDownload("c:\\temp\\dmc-4000-r12h.hex");
             System.out.println("Currently loaded firmware: " + gclib.GCommand("\u0012\u0016")); //^R^V
              */
                          
         }
         catch (GclibJavaException e)
         {
              System.out.println(Integer.toString(e.getErrorCode()) + " " + e.getMessage()); //print the message
         }
         finally
         {
              gclib.GClose(); //must call close
         } 
         
    }//test()
    
}

