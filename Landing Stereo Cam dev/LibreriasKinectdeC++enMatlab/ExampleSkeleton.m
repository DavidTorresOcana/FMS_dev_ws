addpath('Mex')

% Start the Kinect Process
if(~exist('KinectHandles','var'))
    KinectHandles=mxMSCreateContext();
end

% Show the Video-Frames until a player is found
figure,
Pos= mxMSSkeleton(KinectHandles); 
I=mxMSPhoto(KinectHandles); 
h=imshow(I);
while(Pos(1)==0);
    I=mxMSPhoto(KinectHandles); 
    Pos= mxMSSkeleton(KinectHandles); 
    set(h,'Cdata',I); drawnow;
end

% Show the Skeleton of the player, until the player is lost
hh=zeros(1,7);
while(Pos(1)>0)
    I=mxMSPhoto(KinectHandles);
    set(h,'Cdata',I); drawnow;
    Pos= mxMSSkeleton(KinectHandles,1); 
    if(hh(1)>0);
        for i=1:7, delete(hh(i)); end
    end
    
    hold on
    x=(Pos(1:20,5)-0.5)*2;
    y=(Pos(1:20,6)-0.5)*2;
    hh(1)=plot(x,y,'r.');
    hh(2)=plot(x([4 3]),y([4 3]),'g','LineWidth',2);
    hh(3)=plot(x([3 5 6 7 8 ]),y([3 5 6 7 8 ]),'c','LineWidth',2);
    hh(4)=plot(x([3 9 10 11 12]),y([3 9 10 11 12]),'m','LineWidth',2);
    hh(5)=plot(x([3 2 1]),y([3 2 1]),'k','LineWidth',2);
    hh(6)=plot(x([1 13 14 15 16]),y([1 13 14 15 16]),'b','LineWidth',2);
    hh(7)=plot(x([1 17 18 19 20]),y([1 17 18 19 20]),'b','LineWidth',2);
    pause(0.1);
    drawnow
end            
 
% Stop the Kinect Process
%mxMSDeleteContext(KinectHandles);
   


    