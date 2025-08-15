function [v] = BaslerCamCaptureRec(v,fpath)
   
    %v = videoinput("gentl", "1","Mono8");
    if isvalid(v) && strcmp(v.Running, 'on')
        stop(v);
    end

    src = getselectedsource(v);
    src.CounterDuration = 0;
    src.TriggerMode = "On";
    src.TriggerSource = "Line2";
    
    filelocation = fpath;
    baseFilename = "snapshot";
    currentDateTime = datetime('now');
    dateTimeString = datestr(currentDateTime,'yyyy-mm-dd_HH-MM-SS')
    
    filename = baseFilename+"_"+dateTimeString + ".avi";
    fullFilename =fullfile(filelocation,filename);
    
    v.FramesPerTrigger = 1;
    v.TriggerRepeat = Inf;

    % Create and configure the video writer
    %logfile = VideoWriter(fullFilename, "Motion JPEG AVI");
    %logfile.FrameRate = 32;
    
    % Configure the device to log to disk using the video writer
    %v.LoggingMode = "disk";
    %v.DiskLogger = logfile;
    
    %framesPerTrigger = 32;
    %numTriggers = time(end)*framesPerTrigger;
    %numTriggers = Inf;
    %triggerCondition = "DeviceSpecific";
    %triggerSource = "DeviceSpecific";
    
    triggerconfig(v, "hardware", "RisingEdge", "Line2");
    %v.TriggerRepeat = numTriggers - 1;
    

    % try
    %     start(v);
    % catch
    %     disp('Failed to start recording')
    %     return
    % end
    % 
    % % Use INPUT to pause before ending acquisition.
    % %input("Press ENTER to end acquisition.");
    % %pause(20)
    % 
    % stop(v);
    % 
    % 
    % % Wait for all frames to be written to diskstop(v)stop(Camers
    % while v.FramesAcquired ~= v.DiskLoggerFrameCount
    %     pause(.1);
    % end
    % 
    %  % Check if frames were acquired
    % if v.FramesAcquired > 0
    %     disp(['Frames acquired: ' num2str(v.FramesAcquired)]);
    %     disp(['Video saved to: ' fullFilename]);
    %     msgbox(['Recording complete. Frames acquired: ' num2str(v.FramesAcquired)], 'Recording Status');
    % else
    %     disp('No frames were acquired.');
    %     msgbox('No frames were acquired.', 'Recording Status');
    % end
    % 
    % 
    % 
    % %delete(v)
    % %clear src v
    % %delete(imaqfind)

end

