clear all; clc; close all
visualizeWind = VisualizeWindData();
lons = visualizeWind.lons;
lats = visualizeWind.lats;
coordinates = visualizeWind.generateCoordinates(lons, lats);
longitudes = coordinates(:,1);
latitudes = coordinates(:,2);
[m_lat, m_lon] = visualizeWind.latLonToMeters(latitudes, longitudes);

x = linspace(-100,700,500);
y = linspace(-100,450,500);

% Get wind components
[Wx, Wy] = visualizeWind.trigWindFunc(x, y);

[lat_traj, lon_traj] = visualizeWind.metersToLatLon(x, y);

% Plot the wind data and trajectory
visualizeWind.plotTrigWind(deg2rad(lon_traj),deg2rad(lat_traj), Wx, Wy);

%%
% xWind0 = load('Friscosmoothness0wind0XTrajectory_02.mat');
% yWind0 = load('Friscosmoothness0wind0YTrajectory_02.mat');
% zWind0 = load('Friscosmoothness0wind0ZTrajectory_02.mat');
% 
% xWind1 = load('Friscosmoothness0.1wind1XTrajectory_02.mat');
% yWind1 = load('Friscosmoothness0.1wind1YTrajectory_02.mat');
% zWind1 = load('Friscosmoothness0.1wind1ZTrajectory_02.mat');
% 
% xWind3 = load('Friscosmoothness0wind3XTrajectory_02.mat');
% yWind3 = load('Friscosmoothness0wind3YTrajectory_02.mat');
% zWind3 = load('Friscosmoothness0wind3ZTrajectory_02.mat');


% uav =1;
% plot3(xWind0.xTraj(uav,:),yWind0.yTraj(uav,:),zWind0.zTraj(uav,:))
% hold on
% % plot3(xWind1.xTraj(uav,:),yWind1.yTraj(uav,:),zWind1.zTraj(uav,:))
% plot3(xWind3.xTraj(uav,:),yWind3.yTraj(uav,:),zWind3.zTraj(uav,:))
%%%%%%%%%%%%%%%%%%%%%
%%
clear all; clc; close all
visualizeWind = VisualizeWindData();
lons = visualizeWind.lons;
lats = visualizeWind.lats;
coordinates = visualizeWind.generateCoordinates(lons, lats);
longitudes = coordinates(:,1);
latitudes = coordinates(:,2);
[m_lat, m_lon] = visualizeWind.latLonToMeters(latitudes, longitudes);

x = linspace(-100,700,500);
y = linspace(-100,450,500);


% 
[Us, Vs] = visualizeWind.calculateWindComponents(m_lon, m_lat);

visualizeWind.plotWindDataAndTrajectory(deg2rad(longitudes), deg2rad(latitudes), Us, Vs)

%%
if false
load('futureTraj.mat')
nextStep = 10;

% Extract One Step and Future States
oneStepStates = futureTraj(nextStep, :, :); % Change this to 100 for optimal performance
futIndex = floor(linspace(1, length(futureTraj(:, 1, 1)), 10));
futureStates = futureTraj(futIndex, :, :); % Ensure size assertion as needed

index = 35;

X = futureStates(:,1:index:end,1);
Y = futureStates(:,1:index:end,2);
Z = futureStates(:,1:index:end,3);
velocity = futureStates(:,1:index:end,4:6);
% plot3(X,Y,Z,'o')

% Assuming X, Y, Z are 10x1144 and velocity is 10x1144x3
[nTimeStamps, nTrajectories] = size(X); % Get the size of the trajectories

% Prepare the figure
figure;
hold on;
grid on;
axis equal;

xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trajectories with Velocity Vectors');

% Loop through each trajectory and timestamp
for i = 1:nTrajectories
    for j = 1:nTimeStamps
        % Plot each point in the trajectory
        plot3(X(j,i), Y(j,i), Z(j,i), '--or', 'Color','red');
        
        % Extract the velocity components for the current point
        vx = velocity(j,i,1);
        vy = velocity(j,i,2);
        vz = velocity(j,i,3);
        
        % Normalize the velocity vector to get the unit vector
        % This step ensures the vector is only used to indicate direction
        vMagnitude = sqrt(vx^2 + vy^2 + vz^2);
        if vMagnitude ~= 0 % Avoid division by zero for zero velocity
            vx_unit = 8*vx / vMagnitude;
            vy_unit = 8*vy / vMagnitude;
            vz_unit = 8*vz / vMagnitude;
            
            % Plot the velocity vector as an arrow starting from the current point
            quiver3(X(j,i), Y(j,i), Z(j,i), vx_unit, vy_unit, vz_unit, 1.5, 'MaxHeadSize', 2, 'Color','blue');
        end
    end
end

% Adjusting the view for better visualization
view(3); % Sets the view to 3D

end


