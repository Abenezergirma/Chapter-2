clear all; clc; close all;
% Load JSON wind data
data = jsondecode(fileread('smaller_area_wind_data.json'));
% Center point (longitude, latitude)
center_lon = -96.94;
center_lat = 33.06;
% Convert all coordinates to meters relative to the center
% Earth's radius in meters
R = 6371000; 
% Conversion factor for degrees to radians
deg_to_rad = pi / 180;

% Assume wind_data contains fields for coordinates, wind_speed, and wind_direction
wind_data = data.wind_data;

% Extract longitude, latitude, wind speed, and direction
longitudes = cellfun(@(c) c(1), {wind_data.coordinates});
latitudes = cellfun(@(c) c(2), {wind_data.coordinates});

% Convert differences in latitudes and longitudes to meters
delta_lon_m = (longitudes - center_lon) .* cosd(center_lat) * (pi * R / 180);
delta_lat_m = (latitudes - center_lat) * (pi * R / 180);

wind_speeds = [wind_data.wind_speed];
wind_directions = [wind_data.wind_direction]; % Assuming this is in degrees from north

% Convert wind speed and direction into north and east components
U = wind_speeds .* cosd(wind_directions); % North component
V = wind_speeds .* sind(wind_directions); % East component


% Prepare data for regression
X = [delta_lon_m', delta_lat_m']; % Design matrix with coordinates
Y_U = U'; % Response variable for U
Y_V = V'; % Response variable for V

% Linear regression model for U
mdl_U = fitlm(X, Y_U);

% Linear regression model for V
mdl_V = fitlm(X, Y_V);

x_new = 100; 
y_new = 100;
% To predict using the models, for example:
U_pred = predict(mdl_U, [x_new, y_new]);
V_pred = predict(mdl_V, [x_new, y_new]);

mdl_U = -9.0565 + 3.5107e-06*x_new - 3.4781e-06*y_new;
mdl_V = -5.2024 - 4.6088e-07*x_new - 3.2683e-06*y_new;

