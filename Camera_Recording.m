% Connecting to device
v = videoinput("gentl", 1, "Mono8");

% Configure Device
src = getselectedsource(v);
src.ExposureMode = "Timed";
src.TriggerMode = "On";

% File Logging 
filelocation = "C:\Users\juruiz\Desktop\TestBasler";
filename = "recording5.avi";
fullFilename = fullfile(filelocation, filename);


% Create and configure the video writer
logfile = VideoWriter(fullFilename, "Motion JPEG AVI");
logfile.FrameRate = 25;

% Configure the device to log to disk using the video writer
v.LoggingMode = "disk";
v.DiskLogger = logfile;

% Configure Trigger
framesPerTrigger = 1;
numTriggers = Inf;
triggerCondition = "DeviceSpecific";
triggerSource = "DeviceSpecific";

triggerconfig(v, "hardware", triggerCondition, triggerSource);
v.FramesPerTrigger = framesPerTrigger;
v.TriggerRepeat = numTriggers - 1;

% Record with hardware trigger
v.FramesPerTrigger = Inf;
start(v);

% Use INPUT to pause before ending acquisition.
input("Press ENTER to end acquisition.");
stop(v);

% Wait for all frames to be written to disk
while v.FramesAcquired ~= v.DiskLoggerFrameCount
    pause(.1);
end

%Show Recording 
reader = VideoReader(fullFilename);
videoData = read(reader);
implay(videoData);

% CleanUp 
delete(v)
clear src v
delete(imaqfind)

