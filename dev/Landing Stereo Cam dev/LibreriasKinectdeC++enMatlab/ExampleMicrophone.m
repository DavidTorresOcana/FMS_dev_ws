% Open Kinect Microphone Device
KinectAudioHandles=mxMSAudioStart;

% Record Audio (Single channel)
[S,P]=mxMSAudioRecord(KinectAudioHandles,10);

% Normalize audio from int16 to double
S=double(S)/(2^15-1);

% Play the recorder Audio
sound(S,16000,16)

% Close the Kinect Microphone Device
mxMSAudioClose(KinectAudioHandles);
