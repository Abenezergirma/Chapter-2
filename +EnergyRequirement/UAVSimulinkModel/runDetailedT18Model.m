function [SOC,totali,voltage,postraj,refpostraj,time,timeb] = runDetailedT18Model(wayPoint)
% clear all; close all; clc
addContainingDirAndSubDir;


%% load UAV parameters, map and trajectories
octomodel=load('params/TarotT18.mat').octomodel;
Motor=load('params/KDE4213XF-360_performancedata.mat').Motor;
battery=load('params/battery_Tattu.mat').battery;
battery.sampleTime = 0.1;
windspecs=load('params/TarotT18_windspecs.mat').windspecs;
center_lon = (-96.94 + -96.7600)/2;
center_lat = (33.06 + 33.22)/2;


data = load('filtered_wind_data.mat');
x_grid = linspace(min(data.x), max(data.x), 1000);  % Adjust the grid size as needed
y_grid = linspace(min(data.y), max(data.y), 1000);
[X_grid, Y_grid] = meshgrid(x_grid, y_grid);
X_velocity_grid = griddata(data.x, data.y, data.x_velocity, X_grid, Y_grid, 'linear');
Y_velocity_grid = griddata(data.x, data.y, data.y_velocity, X_grid, Y_grid, 'linear');

% Save gridded wind data to workspace
assignin('base', 'X_velocity_grid', X_velocity_grid);
assignin('base', 'Y_velocity_grid', Y_velocity_grid);
assignin('base', 'x_grid', x_grid);
assignin('base', 'y_grid', y_grid);



%% trajectory settings/ mission plan / initial conditions

% mission velocity cruise speed, desired and max
maxleashvelocity=5; % in m/s  15 M/S is max nominal speed for the Tarot

% transform lat long to xy in meters
% xyzENU = lla2enu(waypoints,waypointsorign,'flat');
xyzENU = wayPoint;
xyzref=xyzENU(2:end,:);
xyzic=xyzENU(1,:);
xyzic(3)=xyzENU(1,3);
save('missionplanner/xyzic.mat','xyzic');
save('missionplanner/xyzref.mat','xyzref');

%define initial conditions for the vehicle
IC= load('params/initialcond/IC_HoverAt30mOcto.mat').IC;
IC.X=xyzic(1);
IC.Y=xyzic(2);
IC.Z=xyzic(3);

%rework altitude reference based on initial position
xyzref(:,3)=xyzref(:,3);
% time required to complete the whole mission, currently using this for
% every waypoint, should refine for each waypoint
waypoints=xyzref(:,1:2);
totaldistancei = calculatedistance(waypoints);
timeinterval = calculatetime(totaldistancei,maxleashvelocity);
stoptimetotal=timeinterval(2)+0.35*timeinterval(2);

% time travel between waypoints
waypoints=xyzref;
nWayPoints = length(waypoints);

% Calculate the distance between waypoints
% currently unused, could be used to estimate max time to complete each
% waypoint trajectory
distance = zeros(1,nWayPoints);
for i = 2:nWayPoints
    distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
end


%% wind parameters
turbenabled=0; % 1=enabled, 0=disabled

%% simulate flight from waypoint to waypoint

% variables initialization
postraj=[];
veltraj=[];
atttraj=[];
rpmtraj=[];
refpostraj=[];
batvartraj=[];


for waypts=1:length(xyzref)
    if waypts==6
        see=1;
    end
    % read next waypoint
    xref=xyzref(waypts,1);
    yref=xyzref(waypts,2);
    zref=xyzref(waypts,3);

    % simulate until reaching next waypoint
    w=warning('off','all');
    sim_options = simset('SrcWorkspace', 'current');
    out=sim('dynamicsquat',[],sim_options);

    % obtain simulation data
    pos=state.Data(:,1:3);
    vel=state.Data(:,4:6);
    att=state.Data(:,7:9);
    rpm=motorsrpm.Data(:,1:8);
    refpos=refposition.Data(:,1:3);
    batvar=battery_data.Data(:,1:3);

    % save variables
    postraj=[postraj;pos];
    veltraj=[veltraj;vel];
    atttraj=[atttraj;att];
    rpmtraj=[rpmtraj;rpm];
    refpostraj=[refpostraj;refpos];
    batvartraj=[batvartraj;batvar];

    %reset initial conditions to start from last simulation step
    [IC,battery] = resetinitial(IC,battery,state.Data,battery_variables.Data);
end
%% visualization of results- can be commented for continuous simulation purposes

%create time vector
time=0:octomodel.sampletime:(length(postraj)-1)*octomodel.sampletime;
timeb=0:battery.sampleTime:(length(batvartraj)-1)*battery.sampleTime;

% obtain current results
totali=batvartraj(:,1);
voltage=batvartraj(:,2);
SOC=batvartraj(:,3);

end

function [grid_U, grid_V, X_grid, Y_grid] = processWindData(jsonPath, center_lon, center_lat)
% Load wind data from a JSON file
data = jsondecode(fileread(jsonPath));

% Earth's radius in meters
R = 6371000;

% Assume wind_data contains fields for coordinates, wind_speed, and wind_direction
wind_data = data.wind_data;

% Extract longitude, latitude, wind speed, and direction
longitudes = cellfun(@(c) c(1), {wind_data.coordinates});
latitudes = cellfun(@(c) c(2), {wind_data.coordinates});
wind_speeds = [wind_data.wind_speed];
wind_directions = [wind_data.wind_direction]; % Assuming this is in degrees from north

% Convert differences in latitudes and longitudes from degrees to meters
delta_lon_m = (longitudes - center_lon) .* cosd(center_lat) * (pi * R / 180);
delta_lat_m = (latitudes - center_lat) * (pi * R / 180);

% Convert wind speed and direction into north and east components
U = wind_speeds .* cosd(wind_directions); % North component
V = wind_speeds .* sind(wind_directions); % East component

% Create grid for interpolation
x_range = linspace(min(delta_lon_m), max(delta_lon_m), 100);
y_range = linspace(min(delta_lat_m), max(delta_lat_m), 100);
[X_grid, Y_grid] = meshgrid(x_range, y_range);

% Interpolate U and V components onto the grid
grid_U = griddata(delta_lon_m, delta_lat_m, U, X_grid, Y_grid, 'linear');
grid_V = griddata(delta_lon_m, delta_lat_m, V, X_grid, Y_grid, 'linear');

% The function returns the gridded U and V components, along with the grid coordinates
end