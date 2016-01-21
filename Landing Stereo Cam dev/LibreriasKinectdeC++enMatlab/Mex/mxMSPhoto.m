% This function mxMSPhoto, takes the Current-Photo (movie) frame from 
%   the Kinect-Stream, and makes it available as matlab variable
%
% 	I = mxMSPhoto(KinectHandles);
%
% inputs,
%   KinectHandles : Array with pointers to kinect node objects generated by
%          mxMSCreateContext, such as main, image, IR, Depth and User node.
%
% outputs,
%   I : A matrix of type Uint8 with sizes [sizex sizey 3] with the 
%       current RGB photo of the camera stream
%
% See also mxMSCreateContext, mxMSDepth, mxMSSkeleton, mxMSDeleteContext
%
% Mex-Wrapper is written by D.Kroon University of Twente (September 2011)  
