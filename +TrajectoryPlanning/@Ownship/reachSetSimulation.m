clear all;
initialState = zeros(1,12);

center_lon = -96.94;
center_lat = 33.06;
windDataPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/Wind Forecasts/smaller_area_wind_data.json';
aircraft = TrajectoryPlanning.Ownship(windDataPath,center_lon,center_lat);
trajWind = TrajectoryPlanning.Ownship.OctocopterModelWind(aircraft,aircraft.aircraftActions,aircraft.timeStep, aircraft.numSteps);

X = trajWind(:,1:2:end,1);
Y = trajWind(:,1:2:end,2);
Z = trajWind(:,1:2:end,3);

trajNoWind = TrajectoryPlanning.Ownship.OctocopterModelWind(aircraft,aircraft.aircraftActions,aircraft.timeStep, aircraft.numSteps);
noWindX = trajNoWind(:,1:2:end,1);
noWindY = trajNoWind(:,1:2:end,2);
noWindZ = trajNoWind(:,1:2:end,3);

trajIndex = 100;
plot3(X(:,trajIndex),Y(:,trajIndex),Z(:,trajIndex))
hold on
plot3(noWindX(:,trajIndex),noWindY(:,trajIndex),noWindZ(:,trajIndex))

DryVR_obj = TrajectoryPlanning.DryVR(aircraft.timeStep, aircraft.numSteps);

computeReachTube(aircraft, DryVR_obj);

%% 
reachTube = aircraft.reachSet;
time = 0:0.01:(5)-0.04;
numTrace = DryVR_obj.numTraces;
xState = aircraft.Traces(4:end,:,1);
xStateBounds = reachTube(:,:,2);

yState = aircraft.Traces(4:end,:,2);
yStateBounds = reachTube(:,:,3);

zState = aircraft.Traces(4:end,:,2);
zStateBounds = reachTube(:,:,4);

h = zeros(1,3);
figure(1)
plot(time, xState, 'LineWidth',0.5,'Color','blue');
hold on
h(2:end) = plot(time, xStateBounds, 'LineWidth',1.5, 'Color','black');
hold off
ylabel('x in meters')
xlabel('time in seconds')
legend(h(2:end),'Upper Bound','Lower Bound', 'Location','northwest')
title('Reachable set of the aircraft (state x)') 
print -dpng state_x.png

h = zeros(1,3);
figure(2)
plot(time, yState, 'LineWidth',0.5,'Color','blue');
hold on
h(2:end) = plot(time, yStateBounds, 'LineWidth',1.5, 'Color','black');
hold off
ylabel('y in meters')
xlabel('time in seconds')
legend(h(2:end),'Upper Bound','Lower Bound', 'Location','northwest')
title('Reachable set of the aircraft (state y)') 
print -dpng state_y.png

h = zeros(1,3);
figure(3) 
plot(time, zState, 'LineWidth',0.5,'Color','blue');
hold on
h(2:end) = plot(time, zStateBounds, 'LineWidth',1.5, 'Color','black');
hold off
ylabel('z in meters')
xlabel('time in seconds')
legend(h(2:end),'Upper Bound','Lower Bound', 'Location','northwest')
title('Reachable set of the aircraft (state z)') 
print -dpng state_z.png
%%
xReward = xTraj;
yReward = yTraj;
zReward = zTraj;

trajIndex = 2;
plot3(xTraj(trajIndex,:), yTraj(trajIndex,:), zTraj(trajIndex,:)) % Plot the trajectory
hold on
plot3(xReward(trajIndex,:), yReward(trajIndex,:), zReward(trajIndex,:)) % Plot the rewards

% Add legends
legend('without R', 'With R')





