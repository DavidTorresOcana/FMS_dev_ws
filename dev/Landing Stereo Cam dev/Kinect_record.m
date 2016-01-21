%------------------------------------------------
%------------------------------------------------
%Code to record kinect colour and sensor data
%using code supplied on http://www.mathworks.co.uk/help/imaq/examples/using-the-                kinect-r-for-windows-r-from-image-acquisition-toolbox-tm.html
%and http://www.mathworks.co.uk/help/imaq/examples/logging-data-to-disk.html
%------------------------------------------------
%------------------------------------------------

imaqreset %deletes any image acquisition objects that exsist in memory and uploads         all adaptors loaded by the toolbox. As a result, image acquisition hardware is reset

%------------------------------------------------
%setting up video streams
%------------------------------------------------
disp('Setting up video streams');

%Call up dicertory containing utility functions
utilpath = fullfile(matlabroot, 'toolbox', 'imaq', 'imaqdemos', 'html', 'KinectForWindows');
addpath(utilpath);

%Create the videoinput objects for the colour and depth streams
colourVid = videoinput('kinect', 1, 'RGB_640x480');
%preview(colourVid);
depthVid = videoinput('kinect', 2, 'Depth_640x480');

% Set the depth mode to near.
srcDepth = getselectedsource(depthVid);
srcColor = getselectedsource(colourVid);
% set(srcDepth, 'DepthMode' , 'Near');
set(srcDepth, 'CameraElevationAngle', 5);

%set backlight compensation with centre priority
set(srcColor, 'BacklightCompensation', 'CenterPriority');

disp('Video stream set-up complete');

%------------------------------------------------
%setting up record
%------------------------------------------------

% set the data streams to logging mode and to disk
set(colourVid, 'LoggingMode', 'Disk&Memory');
set(depthVid, 'LoggingMode', 'Disk&Memory');

%Set a video timeout property limit to 50 seconds from
%www.mathworks.com/matlabcentral/answers/103543-why-do-i-receive-the-error-getdata-timed-out-before-frames-were-available-when-using-getdata-in-im
set(colourVid, 'Timeout',90);
set(depthVid, 'Timeout',90);

%Creat a VideoReader object
colourLogfile = VideoWriter('colourTrial5.mj2', 'Motion JPEG 2000');
depthLogfile = VideoWriter('depthTrial5.mj2', 'Archival');

%configure the video input object to use the VideoWriter object
colourVid.DiskLogger = colourLogfile;
depthVid.DiskLogger = depthLogfile;

%set the triggering mode to 'manual'
triggerconfig([colourVid depthVid], 'manual');

%set the FramePerTrigger property of the VIDEOINPUT objects to 100 to
%acquire 100 frames per trigger.
set([colourVid depthVid], 'FramesPerTrigger', 300);

disp('Video record set-up complete');

%------------------------------------------------
%Initiating the aquisition
%------------------------------------------------
disp('Starting Steam');

%Start the colour and depth device. This begins acquisition, but does not
%start logging of acquired data
start([colourVid depthVid]);

pause(2); %allow time for both streams to start

disp('Starting Depth Stream');

%Trigger the devices to start logging of data.
trigger([colourVid depthVid]);

%Retrieve the acquired data
[colourFrameData, colourTimeData, colourMetaData] = getdata(colourVid);
[depthFrameData, depthTimeData, depthMetaData] = getdata(depthVid);

stop([colourVid depthVid])

disp('Recording Complete')
%%




[xyzPoints,flippedDepthImage] = depthToPointCloud(depthFrameData,depthVid);






%------------------------------------------------
%Play back recordings
%------------------------------------------------
disp('Construct playback objects')

colourPlayback = VideoReader('colourTrial5.mj2');
depthPlayback = VideoReader('depthTrial5.mj2');

%Set colour(c) playback parameters
cFrames = colourPlayback.NumberOfFrames;
cHeight = colourPlayback.Height;
cWidth = colourPlayback.Width;

%Preallocate movie structure
colourMov(1:cFrames)=struct('cdata', zeros(cHeight,cWidth,3,'uint8'),'colormap',[]);

disp('Reading colour frames one by one')

%read one frame at a time
for k = 1:cFrames
    colourMov(k).cdata=read(colourPlayback,k);
end

disp('Sizing figure for colour playback')

%Size a figure based on the video's width and height
hf1=figure;
set(hf1,'position',[150 150 cWidth cHeight])

disp('Playing Colour recording')

%play back the movie once at the video's frame rate
movie(hf1,colourMov,1,colourPlayback.FrameRate);

%Set depth(d) playback parameters
dFrames = depthPlayback.NumberOfFrames;
dHeight = depthPlayback.Height;
dWidth = depthPlayback.Width;

%Preallocate movie structure
depthMov(1:dFrames)=struct('cdata', zeros(dHeight,dWidth,3,'uint8'),'colormap',gray(256));

disp('Reading depth frames one by one')

%read one frame at a time
maxDistFromCamera = 1600; % 1600 millimeters
for k = 1:dFrames
    % Depth frames are int16.
    depthFrame = read(depthPlayback,k); 
    % We'll rescale the image from [0,maxDistFromCamera] to [0,255]
    depthFrame = 255.0*single(depthFrame)/maxDistFromCamera;
    % And then recast it to uint8 for display.
    depthMov(k).cdata=uint8(depthFrame);
end

disp('Sizing figure for depth playback')

%Size a figure based on the video's width and height
hf2=figure;
set(hf2,'position',[150 150 dWidth dHeight])

disp('Playing Depth recording')

%play back the movie once at the video's frame rate
movie(hf2,depthMov,1,depthPlayback.FrameRate);

%clear videos from workspace
delete([colourVid depthVid])
clear [colourVid depthVid]

close all;