function [image] = BaslerCamCaptureImg(v)
% Function to capture an image using the BaslerCam
    %v = videoinput("gentl", 1, "Mono8");
    src = getselectedsource(v);
    src.TriggerMode = "Off";
    src.TriggerSource = "Software";
    image = getsnapshot(v);
    
    filelocation = "Cam-Track";
    baseFilename = "snapshot";
    currentDateTime = datetime('now');
    dateTimeString = datestr(currentDateTime,'yyyy-mm-dd_HH-MM-SS')

    filename = baseFilename+"_"+dateTimeString + ".png";
    fullFilename =fullfile(filelocation,filename);
    imwrite(image,fullFilename)

    %delete(v)
    %clear src v
    %delete(imaqfind)

end

