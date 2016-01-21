addpath('Mex')

% Start the Kinect Process
if(~exist('KinectHandles','var'))
    KinectHandles=mxMSCreateContext([2 1]);
end

figure;
% Get Photo and Depth-frame
I=mxMSPhoto(KinectHandles); 
D=mxMSDepth(KinectHandles);

% Normalize Depth map to 0..1 range
D=double(D); D=D-min(D(:)); D=D./(max(D(:))+eps);

subplot(1,2,1), h1=imshow(I);
subplot(1,2,2), h2=imshow(D);

% Display 900 Frames
for i=1:900
    I=mxMSPhoto(KinectHandles); 
    D=mxMSDepth(KinectHandles); D=double(D);  D=D-min(D(:)); D=D./max(D(:));
    set(h1,'CDATA',I);
    set(h2,'CDATA',D);
    pause(0.01);
    drawnow; 
end

% Stop the Kinect Process
%mxMSDeleteContext(KinectHandles);
