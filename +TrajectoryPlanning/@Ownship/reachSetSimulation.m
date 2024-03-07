clear all;
initialState = zeros(1,12);

center_lon = -96.94;
center_lat = 33.06;
windDataPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/Wind Forecasts/smaller_area_wind_data.json';
aircraft = TrajectoryPlanning.Ownship(windDataPath,center_lon,center_lat);
trajNoWind = TrajectoryPlanning.Ownship.OctocopterModelWind(aircraft,aircraft.aircraftActions,aircraft.timeStep, aircraft.numSteps);

X = trajNoWind(:,1:2:end,1);
Y = trajNoWind(:,1:2:end,2);
Z = trajNoWind(:,1:2:end,3);

plot3(X,Y,Z)

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