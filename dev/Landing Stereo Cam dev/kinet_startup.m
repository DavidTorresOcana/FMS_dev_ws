
%% Init kinect adaptor
info = imaqhwinfo('Kinect')

%% Create videoinpout Object for Color stream
info.DeviceInfo(1)

colorVid = videoinput('kinect',1,'RGB_640x480')

preview(colorVid)


%% Create videoinpout Object for DEPTH stream
info.DeviceInfo(2)
depthVid = videoinput('kinect',2,'Depth_640x480')


preview(depthVid)

depthImg = getsnapshot(depthVid);
imshow(depthImg,[0 4000])

%% Getting data
FOV_V = deg2rad(25); %  FOV vertical de la camara de profucdidad
FOV_H = deg2rad(30); %  FOV_H horizontal  de la camara de profucdidad
close all
try
    release(depthVid);
end
pause(2)
depthVid = imaq.VideoDevice('kinect',2,'Depth_80x60');

depthImage = step(depthVid);
xyzPoints = depthToPointCloud(depthImage,depthVid);

figure;
h{1} = surf(xyzPoints(:,:,1),xyzPoints(:,:,2),xyzPoints(:,:,3),'EdgeColor','none');
hold on
h{2} = plot3(0,0,0,'b*');
h{3} = quiver3(0,0,0,0,0,0,'r','LineWidth',5);
axis equal
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
view(0,-90)

% Region of interest: Where we pretend to land
ROI_center =[0,0]; % Center (x,y) of the ROI
L=0.3;  % Side of the square of the ROI
% Init
Dist2ground = 10;

% PSO params
options.PopInitRange		= [];		% range of initial random seed
options.PopulationSize		= 5;		% number of particles in swarm
options.Generations			= 2;		% maximum number of generations
options.TimeLimit			= inf;		% terminate if time limit is reached
options.FitnessLimit		=-inf;		% terminate when fitness drops below this value
options.StallGenLimit		= 50;		% not change more than TolFun over StallGenLimit Generations
options.StallTimeLimit		= 0.02;		% terminate if change less than TolFun over StallTimeLimit
options.TolFun				= 1e-6;		% PSO terminates if global best value does
options.TolCon				= 1e-6;		% Acceptable constraint violation
options.HybridFcn			= [];		% invoke fmincon after pso. Set also the options
options.Display				= 'off';	% {'none'}|iter|final
options.OutputFcns			= [];		% call function(s) after each iteration
options.PlotFcns			= [];		% leave empty for no plot
options.InitialPopulation	= [];		% user specified initial positions
options.InitialVelocities	= [];		% user specified initial velocities
options.InitialGeneration	= 1;		% initial generation
options.PopInitBest			= [];		% specify initial personal best positions
options.CognitiveAttraction	= 0.5;		% acceleration constant, cognitive
options.SocialAttraction	= 0.1;		% acceleration constant, social
options.VelocityLimit		= [0.2,0.2];		% maximum velocity of particles
options.BoundaryMethod		= 'penalize'; % constraint handling method
options.Vectorized			= 'off';	% if fun is vectorized (faster)

Patch = [1,1;-1,1;-1,-1;1,-1]; % Shape for generation of points of evaluation

while -1
    % Get data
    depthImage = step(depthVid);
    xyzPoints = depthToPointCloud(depthImage,depthVid);
    
    % Optimization with fmincon
% %     [ROI_center_best,Cost] = fmincon(@(ROI_center) Landing_box_cost(ROI_center ,xyzPoints,L ),ROI_center,[],[],[],[],  [],[] ,[]);
% %     
            
    % Iterative searching POS
    My_pop=[ROI_center];
    for i=1:options.PopulationSize-1
        My_pop=[ My_pop; ROI_center + 0.02*(rand(1,2).*[2, 2]-[1, 1]) ]; % Random generation
%         My_pop=[ My_pop; ROI_center + 0.2*Patch(i,:) ]; % Square
    end
    options.InitialPopulation = My_pop;
%     options.InitialPopulation = [];
    
    options.PopInitBest = ROI_center;
    [ROI_center_best,Cost] = pso(@(ROI_center) Landing_box_cost(ROI_center ,xyzPoints,L ),2,...
            [],[],[],[],  [-tan(FOV_H)*Dist2ground+L,-tan(FOV_V)*Dist2ground+L],[tan(FOV_H)*Dist2ground-L,tan(FOV_V)*Dist2ground-L] ,[],options);

% % % %     % My OPT func
% % % %     My_pop=[ROI_center];
% % % %     for i=1:options.PopulationSize-1
% % % % %         My_pop=[ My_pop; ROI_center + 0.2*(rand(1,2).*[2, 2]-[1, 1]) ]; % Random generation
% % % %         My_pop=[ My_pop; ROI_center + 0.2*Patch(i,:) ]; % Square
% % % %     end
% % % %     for i=1:options.PopulationSize
% % % %         score(i) = Landing_box_cost(My_pop(i,:) ,xyzPoints,L );
% % % %     end
% % % %     [~,idx] = min(score);
% % % %     ROI_center_best = My_pop(idx,:);
    
    % Eval cost anf Box landing
    [Cost, PLane_2D,Dist2ground,Ground_angle, Rugosity_norm] = Landing_box_cost(ROI_center_best ,xyzPoints,L );
%     ROI_center = ROI_center_best;
        if abs(Ground_angle)<10 && Rugosity_norm<0.05
            ROI_center = ROI_center_best;
        else
            ROI_center = options.PopInitBest;
        end
    
    
    fprintf('\n Plano a %f m . Inclinacion de %f degs %f rugosidad norm ',Dist2ground,Ground_angle,Rugosity_norm);
    % Plot cloud surface and reduced plane
    set(h{1},'XData',xyzPoints(:,:,1),'YData',xyzPoints(:,:,2),'ZData',xyzPoints(:,:,3));
    if abs(Ground_angle)<10 && Rugosity_norm<0.01
        set(h{2},'XData',PLane_2D(:,1),'YData',PLane_2D(:,2),'ZData',PLane_2D(:,3),'Color',[0,1,0]);
    else
        set(h{2},'XData',PLane_2D(:,1),'YData',PLane_2D(:,2),'ZData',PLane_2D(:,3),'Color',[1,0,0]);
    end
    
    set(h{3},'UData',ROI_center(1),'VData',ROI_center(2),'WData',0);
    
    drawnow
end




