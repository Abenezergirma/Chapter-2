clear all;
initialState = zeros(1,12);

center_lon = -96.94;
center_lat = 33.06;
windDataPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/Wind Forecasts/smaller_area_wind_data.json';
aircraft = TrajectoryPlanning.Ownship(windDataPath,center_lon,center_lat);
trajNoWind = TrajectoryPlanning.Ownship.OctocopterModelWind(aircraft, aircraft.aircraftActions,aircraft.timeStep, aircraft.numSteps);

X = trajNoWind(:,1:2:end,1);
Y = trajNoWind(:,1:2:end,2);
Z = trajNoWind(:,1:2:end,3);

plot3(X,Y,Z)

DryVR_obj = DryVR(aircraft.timeStep, aircraft.numSteps);

computeReachTube(aircraft, DryVR_obj);

