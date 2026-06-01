package gclibjava;

public class GclibJavaException extends Exception {
    int myErrorCode = 0;    
    public GclibJavaException(int errorCode, String message) {
        super(message);
        myErrorCode = errorCode;
    }
    public int getErrorCode()
    {
        return myErrorCode;
    }
}
