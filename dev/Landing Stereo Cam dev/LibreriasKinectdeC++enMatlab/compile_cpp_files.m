function compile_cpp_files(MicrosoftSDKPath)
% This function compile_cpp_files will compile the c++ code files
% which wraps OpenNI for the Kinect in Matlab.
%
% Please install first on your computer:
% KinectSDK64.msi or KinectSDK32.msi
%
% Just execute by:
%
%   compile_c_files
%
% or with specifying the Kinect SDK\ path
%
%   compile_cpp_files('C:\Program Files (x86)\Microsoft Research KinectSDK\');
%
%

% Detect 32/64bit and Linux/Mac/PC
c = computer;
is64=length(c)>2&&strcmp(c(end-1:end),'64');

if(nargin<1)
    MicrosoftSDKPath=getenv('KINECTSDK_DIR');
end
if(is64)
    MicrosoftSDKPathLib=[MicrosoftSDKPath '\lib\amd64'];
else
    MicrosoftSDKPathLib=[MicrosoftSDKPath '\lib\x86'];
end
MicrosoftSDKPathInclude=[MicrosoftSDKPath '\inc'];

% Get the folder of the Mex files
functionname='compile_cpp_files.m';
functiondir=which(functionname);
functiondir=functiondir(1:end-length(functionname));
currentdir=cd;
cd([functiondir '\Mex']);
addpath([functiondir '\Mex']);

% Compile all .cpp files to mex files
files=dir('*.cpp');
for i=1:length(files)
    Filename=files(i).name;
    if(strcmpi(Filename(1:min(end,9)),'mxMSAudio'))
        mex('-v',['-L' MicrosoftSDKPathLib],'-lMSRKinectNUI','-lMsdmo','-ldmoguids','-luuid','-lamstrmid','-lavrt',['-I' MicrosoftSDKPathInclude],Filename);
    else
        mex('-v',['-L' MicrosoftSDKPathLib],'-lMSRKinectNUI',['-I' MicrosoftSDKPathInclude],Filename);
    end
end
% Go back to the original folder
cd(currentdir);
