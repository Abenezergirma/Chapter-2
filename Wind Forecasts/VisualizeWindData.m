classdef VisualizeWindData
    properties
        % clear all; clc;
        % define area
        directoryPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';

        lower_left = [-96.84, 33.14];
        upper_right = [-96.81,33.17];
        num_points = 1000;
        lons
        lats

        % coefs_u = [ 0.00000000e+00  1.61094414e-10 -7.37058611e-03 -1.48809651e-06...
        %     -3.68089481e-13 -7.95141020e-06  2.10701150e-17  9.01146727e-10...
        %     1.59724993e-17  6.10108129e-10  7.28957673e-14  8.49416234e-19...
        %     7.23116917e-13  4.75030720e-19  1.14705027e-12];
        % intercept_u = 10.362042657397877;
        % 
        % coefs_v = [ 0.00000000e+00 -2.00488263e-10  8.08528198e-03 -7.92842850e-07...
        %     4.03295347e-13  3.32875723e-06 -1.06781708e-17 -7.96012216e-11...
        %     -1.33310503e-17 -4.23514366e-09  7.95707308e-14 -4.36936906e-19...
        %     -5.13247109e-14 -4.72047951e-19 -1.17203841e-12];
        % intercept_v = 3.69631689881155;
        % degree = 4;

        coefs_u = [ 0.00000000e+00  8.85299837e-05 -1.13459271e-04  3.72603698e-08...
                 -2.06693840e-08  1.56401736e-08 -2.32943787e-11  2.45024828e-11...
                 -6.36089363e-12  1.98028390e-12 -8.06354046e-15  1.52094035e-14...
                 -6.80429453e-15 -1.21078920e-15 -2.41096760e-15];
        intercept_u = -10.049159427402843;

        coefs_v = [ 0.00000000e+00 -1.23492606e-05 -2.62623724e-05 -1.47963467e-08...
                 -1.09706422e-08  3.98864779e-09  5.11118620e-12 -8.34580312e-12...
                  8.42685796e-13  2.60195956e-12  4.01355219e-15  3.28455280e-15...
                 -6.04056136e-15 -1.79591695e-15 -9.53309220e-16];
        intercept_v = -0.7145107563634218;
        degree = 4;


        
        center_lon = (-96.84 + -96.81) / 2;
        center_lat = (33.14 + 33.17) / 2;
    end
    methods
        function obj = VisualizeWindData()
            obj.lons = linspace(obj.lower_left(1), obj.upper_right(1), round(sqrt(1000)));
            obj.lats = linspace(obj.lower_left(2), obj.upper_right(2), round(sqrt(1000)));
        end
        function [Us, Vs] = calculateWindComponents(obj, xs, ys)
            % Calculate wind components in MATLAB, correctly handling intercepts.

            % Ensure xs and ys are column vectors
            xs = xs(:);
            ys = ys(:);

            % Combine xs and ys into a single matrix of shape (n_samples, n_features)
            features = [xs, ys];

            % Generate polynomial features
            polyFeatures = obj.generatePolynomialFeatures(features, obj.degree);

            % Calculate the wind components by dot product, then add the intercept
            Us = polyFeatures * obj.coefs_u' + obj.intercept_u;
            Vs = polyFeatures * obj.coefs_v' + obj.intercept_v;
        end

        function polyFeatures = generatePolynomialFeaturesOld(obj,features, degree)
            % Manually generate polynomial features up to a specified degree for 2D features
            [n_samples, ~] = size(features);

            % Initialize a list to hold all feature combinations
            featureList = {ones(n_samples, 1)}; % Start with the bias (intercept) term

            % Generate feature combinations
            for d = 1:degree
                for i = 0:d
                    j = d - i;
                    newFeature = (features(:,1).^i) .* (features(:,2).^j);
                    featureList{end+1} = newFeature;
                end
            end

            % Combine all features into a single matrix
            polyFeatures = horzcat(featureList{:});
        end

        function polyFeatures = generatePolynomialFeatures(obj,features, degree)
            % Calculate the total number of polynomial features including the bias term
            totalFeatures = (degree + 1) * (degree + 2) / 2;

            [n_samples, ~] = size(features);

            % Preallocate polyFeatures matrix with zeros
            polyFeatures = zeros(n_samples, totalFeatures);

            % Fill in the bias (intercept) term
            polyFeatures(:, 1) = ones(n_samples, 1);

            % Index to keep track of the current column in polyFeatures
            currentColumn = 2;

            % Generate feature combinations and fill in polyFeatures
            for d = 1:degree
                for i = 0:d
                    j = d - i;
                    newFeature = (features(:,1).^i) .* (features(:,2).^j);
                    polyFeatures(:, currentColumn) = newFeature;
                    currentColumn = currentColumn + 1;
                end
            end
        end


        function plotWindData(obj,longitudes, latitudes, U_pred, V_pred)
            longitudes = longitudes(:);
            latitudes = latitudes(:);
            U_pred = U_pred(:);
            V_pred = V_pred(:);
            % Calculate the magnitudes of original and predicted wind components
            % Normalize U_pred and V_pred
            norm_factor = sqrt(U_pred.^2 + V_pred.^2);
            U_normalized = U_pred ./ norm_factor;
            V_normalized = V_pred ./ norm_factor;

            % Create a mesh grid for contour plots
            [grid_x, grid_y] = meshgrid(linspace(min(longitudes), max(longitudes), 100), ...
                linspace(min(latitudes), max(latitudes), 100));

            % Interpolate magnitudes onto the grid
            grid_z_pred = griddata(longitudes, latitudes, norm_factor, grid_x, grid_y, 'linear');

            % Plotting original wind field with contours
            figure;

            contourf(grid_x, grid_y, grid_z_pred', 15, 'LineStyle', 'none');
            cbar = colorbar;
            cbar.Label.String = 'Wind Speed (magnitude)'; % Corrected label setting
            hold on;
            % quiver(longitudes, latitudes, U_normalized, V_normalized, 0.5, 'b');
            quiver(longitudes, latitudes, U_normalized, V_normalized, 0.5, 'b');
            title('Predicted Wind Field');
            xlabel('Longitude');
            ylabel('Latitude');
            grid on;
        end
        function plotWindDataAndTrajectory(obj,longitudes, latitudes, U_pred, V_pred, trajectory)
            longitudes = longitudes(:);
            latitudes = latitudes(:);
            U_pred = U_pred(:);
            V_pred = V_pred(:);
            % Calculate the magnitudes of original and predicted wind components
            % Normalize U_pred and V_pred
            norm_factor = sqrt(U_pred.^2 + V_pred.^2);
            U_normalized = U_pred ./ norm_factor;
            V_normalized = V_pred ./ norm_factor;

            % Create a mesh grid for contour plots
            [grid_x, grid_y] = meshgrid(linspace(min(longitudes), max(longitudes), 100), ...
                linspace(min(latitudes), max(latitudes), 100));

            % Interpolate magnitudes onto the grid
            grid_z_pred = griddata(longitudes, latitudes, norm_factor, grid_x, grid_y, 'linear');

            % Plotting original wind field with contours
            figure;

            contourf(grid_x, grid_y, grid_z_pred', 15, 'LineStyle', 'none');
            cbar = colorbar;
            cbar.Label.String = 'Wind Speed (magnitude)'; % Corrected label setting
            hold on;
            quiver(longitudes, latitudes, U_normalized, V_normalized, 0.5, 'b');

            % Plot the trajectory
            % Assuming the trajectory is an array with rows as [x, y] coordinates
            plot(trajectory(:,1), trajectory(:,2), 'r', 'LineWidth', 2);
            % Plot UAV trajectory
            [longitudes, latitudes] = obj.extractUAVTrajectories;
            % Loop through each UAV in results
            for uav = 1:size(latitudes, 2)


                plot(longitudes{uav}, latitudes{uav}, '-', 'LineWidth', 2, 'Color', 'red', 'DisplayName', sprintf('UAV %d Trajectory', uav));
                % Mark the initial point
                scatter(longitudes{uav}(1), latitudes{uav}(1), 100, 'green', 'o', 'filled', 'ZData', 5);
                % Mark the final point
                scatter(longitudes{uav}(end), latitudes{uav}(end), 100, 'green', 'x', 'LineWidth', 2, 'ZData', 5);
            end
            title('Wind Field');
            xlabel('Longitude');
            ylabel('Latitude');
            grid on;
        end

        function [Wx, Wy] = trigWindFunc(obj, x, y)
            % Interpolate wind speed components at aircraft position using trigonometric functions

            % Parameters for the trigonometric functions
            a_u = 5;
            b_u = 0.01;
            c_u = 2;
            d_u = 3;
            e_u = 0.05;
            f_u = 0.005;

            a_v = 3;
            b_v = 0.01;
            c_v = 1.5;
            d_v = 3;
            e_v = .05;
            f_v = 0.005;

            % Ensure x and y are column vectors
            x = x(:);
            y = y(:);

            % Create mesh grid
            [X, Y] = meshgrid(x, y);

            % Trigonometric functions to generate wind components
            Wx = a_u * sin(b_u * X + c_u) + d_u * cos(e_u * Y + f_u);
            Wy = a_v * sin(b_v * Y + c_v) + d_v * cos(e_v * X + f_v);
        end

        % Function to plot the wind data and trajectory
        function plotTrigWind(obj, longitudes, latitudes, V_pred, U_pred)
            % Normalize U_pred and V_pred
            norm_factor = sqrt(U_pred.^2 + V_pred.^2);
            U_normalized = U_pred ./ norm_factor;
            V_normalized = V_pred ./ norm_factor;

            % Create a mesh grid for contour plots
            [grid_x, grid_y] = meshgrid(longitudes, latitudes);

            % Interpolate magnitudes onto the grid
            grid_z_pred = griddata(grid_x(:), grid_y(:), norm_factor(:), grid_x, grid_y, 'linear');

            % Plotting original wind field with contours
            figure;

            contourf(grid_x, grid_y, grid_z_pred, 15, 'LineStyle', 'none');
            cbar = colorbar;
            cbar.Label.String = 'Wind Speed (magnitude)';
            hold on;
            % quiver(grid_x, grid_y, U_normalized, V_normalized, 0.5, 'b');
            quiver(longitudes(1:10:end), latitudes(1:10:end),V_normalized(1:10:end,1:10:end), U_normalized(1:10:end,1:10:end), 0.75, 'b');

            
            % Plot UAV trajectory
            [longitudes, latitudes, fileNames] = obj.extractUAVTrajectories();
            % Generate unique colors for each experiment
            uniqueFiles = unique(fileNames);
            colorMap = lines(length(uniqueFiles));
            
            % Plot each UAV trajectory with different color based on file name
            for uav = 1:size(latitudes, 2)
                fileIdx = find(strcmp(fileNames{uav}, uniqueFiles));
                plot(longitudes{uav}, latitudes{uav}, '-', 'LineWidth', 2, 'Color', colorMap(fileIdx, :), 'DisplayName', sprintf('UAV %d Trajectory', uav));
                
                % Mark the initial point
                scatter(longitudes{uav}(1), latitudes{uav}(1), 100, 'green', 'o', 'filled', 'ZData', 5);
                
                % Mark the final point
                scatter(longitudes{uav}(end), latitudes{uav}(end), 100, 'green', 'x', 'LineWidth', 2, 'ZData', 5);
            end

            title('Wind Field');
            xlabel('Longitude');
            ylabel('Latitude');
            grid on;
        end

        function [m_lat, m_lon] = latLonToMeters(obj,lat, lon)
            % Earth's radius in meters
            R = 6371000;

            % Convert latitude and longitude differences to radians
            delta_lat = (lat - obj.center_lat) * pi / 180;
            delta_lon = (lon - obj.center_lon) * pi / 180;

            % Approximate conversion to meters
            m_lat = delta_lat * R;
            m_lon = delta_lon * R * cos(obj.center_lat * pi / 180);
        end

        function coordinates = generateCoordinates(obj,lons, lats)
            % Initialize an empty array to hold the coordinate pairs
            coordinates = [];

            % Iterate through each lon and lat value to create [lon, lat] pairs
            for i = 1:length(lons)
                for j = 1:length(lats)
                    % Append the [lon, lat] pair to the coordinates array
                    coordinates = [coordinates; [lons(i), lats(j)]];
                end
            end
        end

        function [lat, lon] = metersToLatLon(obj,x, y)
            % Constants
            meters_per_degree_lat = 111319;
            % Convert center latitude from degrees to radians for the cosine function
            meters_per_degree_lon = meters_per_degree_lat * cosd(obj.center_lat);

            % Convert
            delta_lat = y / meters_per_degree_lat;
            delta_lon = x / meters_per_degree_lon;

            lat = obj.center_lat + delta_lat;
            lon = obj.center_lon + delta_lon;
        end

        function [longitudes, latitudes, fileNames] = extractUAVTrajectories(obj)
            % Initialize cell arrays to hold latitudes and longitudes for all UAVs
            latitudes = {};
            longitudes = {};
            fileNames = {};


            % List all .mat files in the directory
            matFiles = dir(fullfile(obj.directoryPath, '*.mat'));

            % Loop through each .mat file
            for i = 1:length(matFiles)
                filePath = fullfile(obj.directoryPath, matFiles(i).name);
                data = load(filePath);
                results = data.results;

                % Loop through each UAV in results
                for uav = 1:size(results, 1)
                    trajectory = results(uav, 4); % Assuming trajectory is stored in the 4th column
                    % Assuming metersToLatLon is a method that can handle vectors
                    [lat, lon] = obj.metersToLatLon(trajectory{1}(:, 1), trajectory{1}(:, 2));
                    % Convert from meters to geographic coordinates and store in cell arrays
                    latitudes{end+1} = deg2rad(lat);
                    longitudes{end+1} = deg2rad(lon);
                    % Store the file name for color coding
                    fileNames{end+1} = matFiles(i).name;
                end
            end
        end

    end

end

