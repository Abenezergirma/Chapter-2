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


% Create grid for interpolation
% Note: You might need to adjust the resolution and range depending on your specific area
x_range = linspace(min(delta_lon_m), max(delta_lon_m), 100);
y_range = linspace(min(delta_lat_m), max(delta_lat_m), 100);
[X_grid, Y_grid] = meshgrid(x_range, y_range);

% Grid the data
grid_U = griddata(delta_lon_m, delta_lat_m, U, X_grid, Y_grid, 'linear');
grid_V = griddata(delta_lon_m, delta_lat_m, V, X_grid, Y_grid, 'linear');

% Aircraft position in meters from the center
aircraft_x = 10; % Define based on your simulation
aircraft_y = 12;

% Interpolate wind speed components at aircraft position
interpolated_U = interp2(X_grid, Y_grid, grid_U, aircraft_x, aircraft_y, 'linear');
interpolated_V = interp2(X_grid, Y_grid, grid_V, aircraft_x, aircraft_y, 'linear');

