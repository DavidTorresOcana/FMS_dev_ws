% This function mxMSAudioRecord, records audio from the kinect microphone
% array
%
% 	[S,P]=mxMSAudioRecord(KinectAudioHandles, T);
%
% inputs,
%   KinectAudioHandles : Array with pointers to Kinect Microphone objects
%   T : Number of Seconds to record
%
% outputs,
%   S : Array of type Int16 with the recorded audio
%   P : Array with the sound position M x 3 [Angle, Conf, BeamAngle]
% 
% See also mxMSAudioStart,  mxMSAudioClose
%
% Mex-Wrapper is written by D.Kroon University of Twente (September 2011)  
