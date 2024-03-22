clear all; clc; close all
visualizeWind = VisualizeWindData();
lons = visualizeWind.lons;
lats = visualizeWind.lats;
coordinates = visualizeWind.generateCoordinates(lons, lats);
longitudes = coordinates(:,1);
latitudes = coordinates(:,2);
[m_lat, m_lon] = visualizeWind.latLonToMeters(latitudes, longitudes);

x = linspace(100,700,500);%linspace(-100,1000,500);
y = linspace(100,450,500);

[Us, Vs] = visualizeWind.calculateWindComponents(m_lon, m_lat);
% [u,v] = calculateWindComponents(coefs_u, intercept_u, coefs_v, intercept_v, 1000, 1000, degree)
[lat_traj, lon_traj] = visualizeWind.metersToLatLon(x, y);

trajectory = [NaN,NaN];%[lon_traj;lat_traj]';
% plotWindData(deg2rad(longitudes), deg2rad(latitudes), Us, Vs)
visualizeWind.plotWindDataAndTrajectory(deg2rad(longitudes), deg2rad(latitudes), Us, Vs, deg2rad(trajectory))
% visualizeWind.plotUavTrajectoriesOnWindContour(directoryPath)