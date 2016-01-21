cd(fileparts(mfilename('fullpath')))



%% Calculating cartesian coordinates

% the variables we know (set these yourself)
width = 512;
height = 424;
ycenter = 209.5835;
xcenter = 260.1922;
HfoclenInPixels = 365.5953;
VfoclenInPixels = 365.5953;


% create convinient matrices for the coordinate calculation (way faster than for-loops!)
yv = linspace(0,height-1, height);
yv = yv';
yv = yv - (ycenter);
yv = repmat(yv, 1, width);
yv = yv*-1;

xv = linspace(0,width-1, width);
xv = xv - xcenter;
repmat(xv, height, 1);
xv = repmat(xv, height, 1);



% beginning
i = 1;
framepath = 'frame1.mat';
load(framepath);

for i=1:size(depthTrial5:4)

    % make sure that the frame is compatible with the code
    if (size(depthTrial5,1)~= height)
        depthmat(:,:,1,:) = rot90(depthmat,-1);
    end
    if (~isfloat(depthmat))
       depthmat = cast(depthmat,'double'); 
    end

    % the actual coordinate calculation
    yw = (depthmat.*yv)/VfoclenInPixels;
    xw = (depthmat.*xv)/HfoclenInPixels;
    zw = depthmat;
    
    
    save(framepath , 'xw', 'yw', 'zw');
    
    framepath = strcat('frame',num2str(i),'.mat');
    try 
        load(strcat('frame',num2str(i),'.mat'));
    catch

    end
end


%% For Visualization

% v is used to reduce the number of points to show 
v=randi(height*width,10000,1);

% vectorizing the data, for input to scatter3
xs = xw(:);
ys = yw(:);
zs = zw(:);

% Plot
scatter3(xs(v), zs(v), ys(v),30,zs(v),'.');

% colomap and bar
colormap(gray);
colorbar;

axis equal;