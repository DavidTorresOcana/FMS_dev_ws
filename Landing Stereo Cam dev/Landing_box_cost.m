function [ Cost, Plane_2D,Dist2ground,Ground_angle, Rugosity_norm] = Landing_box_cost(ROI_center ,xyzPoints,L )
Cost = 0;
INF_cost = 10000000;
Dist2ground = 0;
Ground_angle =0;
Rugosity_norm = 0;
Plane_2D = zeros(1,3);

% Select the square of interest
X = reshape( xyzPoints(:,:,1),60*80,1 );
Y = reshape( xyzPoints(:,:,2),60*80,1 );
Z = reshape( xyzPoints(:,:,3),60*80,1 );
idxs = logical(  ( X > (ROI_center(1)-L) ).*( X < (ROI_center(1)+L) ).*( Y > (ROI_center(2)-L) ).*( Y < (ROI_center(2)+L) )  );

Box_3D =[X(idxs),Y(idxs),Z(idxs)]; % Array of all point defining the 3D point cloud

% Normalize the cloud
Box_center = mean(Box_3D,1);
Box_3D_norm = [Box_3D(:,1) - Box_center(1),Box_3D(:,2) - Box_center(2),Box_3D(:,3) - Box_center(3)];
% Compute the covariance
Box_Sigma = Box_3D_norm'*Box_3D_norm./size(Box_3D_norm,1);
% Get eigen values and vectors
try
    [U,S] = svd(Box_Sigma);
catch
    Plane_2D = zeros(1,3);
    Dist2ground = 0;
    Ground_angle = 90;
    Rugosity_norm = 100;
    Cost = INF_cost;
    return
end
% Inclinaciones y Rugosidad
theta = rad2deg( atan(sqrt(U(1,3)^2+U(2,3)^2)/U(3,3)) ); % Angulo de inclinacion en degs

% Get the reduced (proeycted) plane 3D points
Plane_2D =  (U(:,1:2)'*Box_3D_norm' )'*U(:,1:2)' + repmat([Box_center(1:2),Box_center(3)-0.1],size(Box_3D,1),1);

Dist2ground = norm(Box_center);
Ground_angle = theta;
Rugosity_norm = S(3,3)*100/norm(Box_center);

% Landing suitabiity criterion
if abs(Ground_angle)<10 && Rugosity_norm<0.05
    % Cost of pure suitability
    lambda = 1000;
    Cost  =  Ground_angle + lambda*Rugosity_norm;
    
    % Add cost of deviation
    Cost = Cost + 100*norm(ROI_center);
else
    Cost = INF_cost;
end

Cost = double(Cost);

end

