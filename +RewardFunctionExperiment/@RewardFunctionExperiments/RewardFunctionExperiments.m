classdef RewardFunctionExperiments %< TrajectoryPlanning.Ownship & EnergyRequirement.SimulinkUAVWrapper
    properties
        % Use the concept of composition to import the two vehicle models
        % lowFidelityModel  = TrajectoryPlanning.Ownship(); % Low-fidelity UAV model
        % timeStep = 0.1;

        highFidelityModelPath
        highFidelityModelRunner
        highFidelityModel % High-fidelity Simulink model
        BatteryParams % full name for the .mat that stores the battery related parameters
        energyRequirement % full name for the .mat file that stores the energy requirements of the mission
        resultsPath
        experimentName
        maxTotalPower  % Maximum total power for normalization
    end

    methods
        function obj = RewardFunctionExperiments(maxTotalPower)
            % Constructor for the class
            % obj@TrajectoryPlanning.Ownship(windDataPath);
            % obj@EnergyRequirement.SimulinkUAVWrapper(modelPath)

            obj.maxTotalPower = maxTotalPower;
        end

        function controlCombinations = generateControlCombinations(obj)
            % Generate all combinations of control actions
            % Return an array of control action combinations
            controlCombinations = obj.lowFidelityModel.T18Actions;
        end

        function trajectories = runLowFidelityModel(obj, controlCombinations, simulationTime)
            % Run low-fidelity model simulation
            % Return a set of trajectories for each control action
            % timeStep = obj.lowFidelityModel.timeStep;
            trajectories = obj.lowFidelityModel.discreteOctocopterModel(obj.lowFidelityModel,controlCombinations,obj.timeStep, ...
                simulationTime/obj.timeStep);
        end

        function waypoints = convertTrajectoriesToWaypoints(obj, trajectories, numWaypoints)
            % Converts the set of trajectories generated from the control
            % actions to waypoints. The trajectories array is a 3D array of size NxMx3,
            % where N represents the number of trajectories,
            %       M represents the number of data points in each traj
            %
            %

            % Check input dimensions
            [N, M, numDimensions] = size(trajectories);
            if numDimensions ~= 3
                error('Input trajectories must be in [x, y, z] format.');
            end

            % Calculate step size for each trajectory
            stepSize = floor((N - 1) / (numWaypoints - 2));

            % Calculate the indices for waypoints
            waypointIndices = [1, (1:numWaypoints-2) * stepSize + 1, N]; % Include first and last points

            % Initialize waypoints matrix
            waypoints = zeros(numWaypoints, M, numDimensions);

            % Extract waypoints from trajectories using vectorized operations
            for i = 1:M
                waypointIndices_i = waypointIndices ;  % Adjust indices for each trajectory
                waypoints(:, i, :) = reshape(trajectories(waypointIndices_i,i, :), [numWaypoints, 1, numDimensions]);
            end

        end

        function smoothedTrajectory = smoothTrajectory(obj, originalTrajectory, smoothness)
            % originalTrajectory: Nx3 matrix containing [x, y, z] positions of the UAV
            % smoothness: scalar indicating the number of points for interpolation (higher value = smoother)
            % smoothedTrajectory: Mx3 matrix containing smoothed [x, y, z] positions

            % Ensure originalTrajectory has enough points
            if size(originalTrajectory, 1) < 2
                error('originalTrajectory must contain at least two points.');
            end

            % Extract x, y, z
            x = originalTrajectory(:, 1);
            y = originalTrajectory(:, 2);
            z = originalTrajectory(:, 3);

            % Create original parameterization
            tOriginal = 1:length(x);

            % Create smooth parameterization
            tSmooth = linspace(1, length(x), smoothness);

            % Spline interpolation
            xSmooth = spline(tOriginal, x, tSmooth);
            ySmooth = spline(tOriginal, y, tSmooth);
            zSmooth = spline(tOriginal, z, tSmooth);

            % Combine smoothed x, y, z
            smoothedTrajectory = [xSmooth', ySmooth', zSmooth'];
        end

        function plotAllTrajectoryAndWaypoints(obj, trajectories, waypoints)
            % Ensure that trajectories and waypoints have the correct shape
            if size(trajectories, 3) ~= 3 || size(waypoints, 3) ~= 3
                error('Trajectories and waypoints must have a size of MxNx3.');
            end

            % Number of UAVs
            numUAVs = size(trajectories, 2);

            % Create a new figure
            figure;
            hold on; % Keep the current plot when adding new plots

            % Define colors and markers for better distinction between different UAVs
            colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k'];
            markers = ['o', 's', 'd', '^', 'v', '>', '<', 'p', 'h'];
            % Loop through each UAV
            for uavIdx = 1:numUAVs
                % Extract x, y, and z coordinates from trajectories and waypoints
                x_traj = trajectories(:, uavIdx, 1);
                y_traj = trajectories(:, uavIdx, 2);
                z_traj = trajectories(:, uavIdx, 3);

                x_waypoints = waypoints(:, uavIdx, 1);
                y_waypoints = waypoints(:, uavIdx, 2);
                z_waypoints = waypoints(:, uavIdx, 3);

                % Plot the trajectory in blue
                plot3(x_traj, y_traj, z_traj, 'b-', 'LineWidth', 1.5);

                % Plot the waypoints in red
                scatter3(x_waypoints, y_waypoints, z_waypoints, 50, 'ro', 'filled');
            end

            % % Loop through each UAV
            % for uavIdx = 1:numUAVs
            %     % Extract x, y, and z coordinates from trajectories and waypoints
            %     x_traj = trajectories(:, uavIdx, 1);
            %     y_traj = trajectories(:, uavIdx, 2);
            %     z_traj = trajectories(:, uavIdx, 3);
            %
            %     x_waypoints = waypoints(:, uavIdx, 1);
            %     y_waypoints = waypoints(:, uavIdx, 2);
            %     z_waypoints = waypoints(:, uavIdx, 3);
            %
            %     % Choose color and marker
            %     color = colors(mod(uavIdx-1, length(colors)) + 1);
            %     marker = markers(mod(uavIdx-1, length(markers)) + 1);
            %
            %     % Plot the trajectory
            %     plot3(x_traj, y_traj, z_traj, [color, '-'], 'LineWidth', 1.5);
            %
            %     % Plot the waypoints
            %     scatter3(x_waypoints, y_waypoints, z_waypoints, 100, color, marker, 'filled');
            % end

            % Label axes and title the plot
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Trajectories and Waypoints');
            grid on;

            % Adjust view and display the plot
            view(3);
            axis equal;
            hold off;

            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath, 'allTrajectoriesAndWaypoints.png');

            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng'); % '-dpng' specifies the format
        end

        function smoothedTrajectories = smoothTrajectoryVectorized(obj, originalTrajectories, smoothness)
            % originalTrajectories: NxMx3 matrix containing [x, y, z] positions of M UAVs
            % smoothness: scalar indicating the number of points for interpolation (higher value = smoother)
            % smoothedTrajectories: NxMx3 matrix containing smoothed [x, y, z] positions for each UAV

            % Ensure originalTrajectories has enough points
            if size(originalTrajectories, 1) < 2
                error('originalTrajectories must contain at least two points.');
            end

            % Get the number of waypoints (N), the number of aircraft (M), and the dimensionality (3)
            [N, M, dim] = size(originalTrajectories);

            % Create original parameterization
            tOriginal = 1:N;

            % Create smooth parameterization
            tSmooth = linspace(1, N, smoothness);

            % Initialize the matrix to store smoothed trajectories
            smoothedTrajectories = zeros(smoothness, M, dim);

            % Loop through each UAV
            for i = 1:M
                % Extract x, y, z for the current UAV
                trajectory = originalTrajectories(:, i, :);

                % Initialize matrices for smoothed dimensions
                smoothedDimensions = zeros(smoothness, dim);

                % Spline interpolation for x, y, z
                for d = 1:dim
                    % Perform spline interpolation
                    smoothedDimensions(:, d) = spline(tOriginal, trajectory(:, d), tSmooth);
                end

                % Assign the smoothed dimensions to the output
                smoothedTrajectories(:, i, :) = smoothedDimensions;
            end
        end




        function waypoints = NewconvertTrajectoriesToWaypoints(obj, trajectories, numWaypoints)
            % Converts the set of trajectories generated from the control
            % actions to waypoints. The trajectories array is a 3D array of size NxMx3,
            % where N represents the number of trajectories,
            %       M represents the number of data points in each traj
            %

            % Check input dimensions
            [N, M, numDimensions] = size(trajectories);
            if numDimensions ~= 3
                error('Input trajectories must be in [x, y, z] format.');
            end

            % Initialize waypoints matrix
            waypoints = zeros(numWaypoints, M, numDimensions);

            % Define the parameter values for the original and interpolated trajectories
            origParamValues = linspace(0, 1, N);
            interpParamValues = linspace(0, 1, numWaypoints);

            % Extract waypoints from trajectories using interpolation
            for i = 1:M
                for j = 1:numDimensions
                    waypoints(:, i, j) = interp1(origParamValues, trajectories(:, i, j), interpParamValues, 'linear');
                end
            end
        end


        function plotTrajectoryAndWaypoints(obj,trajectories, waypoints)
            % Extract x, y, and z coordinates from trajectories and waypoints
            x_traj = trajectories(:, 1);
            y_traj = trajectories(:, 2);
            z_traj = trajectories(:, 3);

            x_waypoints = waypoints(:, 1);
            y_waypoints = waypoints(:, 2);
            z_waypoints = waypoints(:, 3);

            % Plot the trajectory
            figure;
            plot3(x_traj, y_traj, z_traj, 'b-', 'LineWidth', 1.5);
            hold on;

            % Plot the waypoints
            scatter3(x_waypoints, y_waypoints, z_waypoints, 100, 'ro', 'filled');


            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Trajectory and Waypoints');
            grid on;

            % Adjust view and display the plot
            view(3);
            axis equal;
            hold off;
            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath,'trajectoryAndWaypoint.png');

            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng'); % '-dpng' specifies the format, '-r300' sets the resolution (dpi)
        end


        function obj = getPowerConsumption(obj)
            % add a for loop here
            % obj.highFidelityModel = SimulinkUAVWrapper(obj.highFidelityModelPath, obj.highFidelityModelRunner);
            % experimentPath = obj.highFidelityModel.resultsPath;
            % resultFileName = sprintf('rewardExperiment.mat');
            % resultFilePath = fullfile(experimentPath, resultFileName);
            % Load energy related UAV parameters from the .mat file
            load(obj.BatteryParams);%, 'SOC', 'totalCurrent','voltage');
            reqEnergy = [];

            numExperiments  = size(results,1);
            for k = 1:numExperiments
                SOC = results{k,1};
                totalCurrent = results{k,2};
                voltage = results{k,3};
                % totalEnergy = computeTotalEnergy(obj, totalCurrent, voltage, SOC);
                energyAtPoints = computeEnergyAtPoints(obj, totalCurrent, voltage, SOC);
                reqEnergy = [reqEnergy;energyAtPoints];
            end
            energyFileName = obj.energyRequirement;
            energyFilePath = fullfile(obj.resultsPath, obj.energyRequirement);
            save(energyFilePath,"reqEnergy");
        end

        function totalEnergy = computeTotalEnergy(obj, current, voltage, soc)
            % Check if input arrays have the same length
            if numel(current) ~= numel(voltage) || numel(current) ~= numel(soc)
                error('Input arrays must have the same length.');
            end

            % Convert current and voltage to Watts
            power = current .* voltage;

            % Replace with the actual time step (in seconds) if needed
            % timeStep = obj.highFidelityModel.timeStep;

            % Compute energy using trapezoidal integration
            energy = trapz(power) * obj.timeStep;

            % Convert energy to Watt-hours
            energyWh = energy / 3600; % 1 Wh = 3600 J

            totalEnergy = energy;
        end

        function energyAtPoints = computeEnergyAtPoints(obj, current, voltage, soc)
            % Check if input arrays have the same length
            if numel(current) ~= numel(voltage) || numel(current) ~= numel(soc)
                error('Input arrays must have the same length.');
            end

            % Convert current and voltage to Watts
            power = current .* voltage;

            % Generate 10 equally spaced indices along the trajectory
            indices = round(linspace(1, numel(current), 10));

            % Initialize array to store energy at 10 points
            energyAtPoints = zeros(1, 10);

            % Compute energy at each point using trapezoidal integration
            for i = 1:10
                if i == 1
                    % For the first point, integrate from the start to the first index
                    energyAtPoints(i) = trapz(power(1:indices(i))) * obj.timeStep;
                else
                    % For subsequent points, integrate from the previous index to the current index
                    energyAtPoints(i) = trapz(power(indices(i-1):indices(i))) * obj.timeStep;
                end
                % Convert energy to Watt-hours
                % energyAtPoints(i) = energyAtPoints(i) / 3600; % 1 Wh = 3600 J
            end
        end



        function obj = runHighFidelityModel(obj, setWaypoints)

            obj.highFidelityModel = EnergyRequirement.SimulinkUAVWrapper(obj.highFidelityModelPath);
            obj.highFidelityModel.modelScript = obj.highFidelityModelRunner;
            obj.highFidelityModel.experimentName = obj.experimentName;
            obj.highFidelityModel.resultsPath = obj.resultsPath;

            disp('This experiment is to generate energy-related reward function');
            obj.highFidelityModel.runModel(setWaypoints);

        end

        function fittedFunction = createFittedEnergyFunction(obj,sampleEnegry)
            % Fit a curve to sampleEnegry
            x = 1:20:10000;
            fittedCurve = fit(x', sampleEnegry, 'smoothingspline');

            % Create a function that takes an experiment number and returns the values
            fittedFunction = @(experimentNumber) feval(fittedCurve, experimentNumber);
        end

        function fittedFunctions = createFittedEnergyFunctions(obj, sampleEnergy)
            % Check the size of sampleEnergy
            [numExperiments, numTimeSteps] = size(sampleEnergy);

            if numTimeSteps ~= 10
                error('The second dimension of sampleEnergy must be 10.');
            end

            % Initialize a cell array to store the fitted functions for each time step
            fittedFunctions = cell(1, numTimeSteps);

            % Define x - assuming each experiment corresponds to a point in x
            x = 1:20:10000;

            % Loop through each time step
            for i = 1:numTimeSteps
                % Extract the energy values for the current time step
                currentEnergy = sampleEnergy(:, i);

                % Fit a curve to the current energy values
                fittedCurve = fit(x', currentEnergy, 'smoothingspline');

                % Store the fitted function in the cell array
                fittedFunctions{i} = @(experimentNumber) feval(fittedCurve, experimentNumber);
            end
        end

        function plotPowerProfiles(obj, allData, allNames, uav)
            % Prepare the figure
            % Prepare the figure
            figure('Color', 'w', 'Position', [100, 100, 1200, 600]);

            % Define line styles
            lineStyles = {'-', '--', ':', '-.'};
            numStyles = length(lineStyles);

            % Generate a colormap and select a subset of colors
            totalPlots = length(allData);
            myColors = lines(totalPlots); % 'lines' is one of MATLAB's built-in colormaps

            % Subplot for Power Profiles
            subplot(2,1,1); % This specifies a 2-row, 1-column grid of subplots, and activates the first subplot for the power profiles.
            hold on;

            for idx = 1:length(allData)
                % Extract the dataset
                data = allData{idx};
                fullName = allNames{idx}; % Assuming each dataset's name is stored in `allNames`
                [SOC, totali, voltage, ~, ~, time, timeb] = deal(data.results{uav,1}, data.results{uav,2}, data.results{uav,3}, ...
                    data.results{uav,4}, data.results{uav,5}, data.results{uav,6}, data.results{uav,7});


                % Ensure column vectors
                timeb = timeb(:);
                totali = totali(:);
                voltage = voltage(:);

                % Calculate the power profile
                powerProfile = totali .* voltage;

                color = myColors(idx, :);
                lineStyle = lineStyles{mod(idx-1, numStyles) + 1};


                % Plot the power profile
                plot(timeb, powerProfile, 'LineWidth', 2, 'DisplayName', fullName, 'Color', color, 'LineStyle', lineStyle);
            end

            xlabel('Time (s)');
            ylabel('Power (W)');
            title('Comparison of Power Profiles Across Conditions');
            legend('Location', 'Best');
            grid on;
            hold off;

            % Subplot for Cumulative Energy
            subplot(2,1,2); % Activates the second subplot for the cumulative energy.
            hold on;

            for idx = 1:length(allData)
                data = allData{idx};
                fullName = allNames{idx}; % Using the name as the legend label
                [SOC, totali, voltage, ~, ~, time, timeb] = deal(data.results{uav,1}, data.results{uav,2}, data.results{uav,3}, ...
                    data.results{uav,4}, data.results{uav,5}, data.results{uav,6}, data.results{uav,7});


                % Ensure column vectors
                timeb = timeb(:);
                totali = totali(:);
                voltage = voltage(:);

                % Calculate the power profile
                powerProfile = totali .* voltage;

                % Compute cumulative energy
                energyProfile = cumtrapz(timeb, powerProfile);

                color = myColors(idx, :);
                lineStyle = lineStyles{mod(idx-1, numStyles) + 1};

                % Plot the cumulative energy
                plot(timeb, energyProfile, 'LineWidth', 2, 'DisplayName', fullName, 'Color', color, 'LineStyle', lineStyle);
            end

            xlabel('Time (s)');
            ylabel('Energy (J)');
            title('Comparison of Cumulative Energy Across Conditions');
            legend('Location', 'Best');
            grid on;
            hold off;

            % Save the figure as before
            filename = fullfile(obj.resultsPath, 'PowerAndEnergyComparison.png');
            print(filename, '-dpng'); % '-dpng' specifies the format
        end

        function plotPowerProfilesWithBar(obj, allData, allNames, uav)
            % Prepare the figure
            figure('Color', 'w', 'Position', [100, 100, 1200, 800]);

            % Define line styles and generate a colormap for distinct plots
            lineStyles = {'-', '--', ':', '-.'};
            numStyles = length(lineStyles);
            totalPlots = length(allData);
            myColors = lines(totalPlots); % Generate distinct colors

            % First subplot for cumulative energy line plots
            subplot(2,1,1); % Use for cumulative energy plots
            hold on;
            finalEnergyValues = zeros(totalPlots, 1); % To store final cumulative energy values

            for iPlot = 1:totalPlots
                data = allData{iPlot};
                fullName = allNames{iPlot}; % Assuming you have this variable prepared
                [SOC, totali, voltage, ~, ~, time, timeb] = deal(data.results{uav,1}, data.results{uav,2}, data.results{uav,3}, ...
                    data.results{uav,4}, data.results{uav,5}, data.results{uav,6}, data.results{uav,7});

                timeb = timeb(:); totali = totali(:); voltage = voltage(:);
                powerProfile = totali .* voltage;
                energyProfile = cumtrapz(timeb, powerProfile);
                finalEnergyValues(iPlot) = energyProfile(end); % Store last value for bar chart

                color = myColors(iPlot, :);
                lineStyle = lineStyles{mod(iPlot-1, numStyles) + 1};
                plot(timeb, powerProfile, 'LineWidth', 2, 'DisplayName', fullName, 'Color', color, 'LineStyle', lineStyle);
            end

            xlabel('Time (s)');
            ylabel('Cumulative Energy (J)');
            title('Cumulative Energy Profiles');
            legend('Location', 'Best');
            grid on;
            hold off;

            % Second subplot for final cumulative energy values as a bar chart
            subplot(2,1,2); % Use for the bar chart of final values
            hold on; % Enable holding multiple plots

            % Plot each bar individually to set colors
            for iBar = 1:length(finalEnergyValues)
                bar(iBar, finalEnergyValues(iBar), 'FaceColor', myColors(iBar, :));
            end

            % Adding labels to each bar with its value
            for i = 1:length(finalEnergyValues)
                text(i, finalEnergyValues(i), sprintf('%.2f J', finalEnergyValues(i)), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'bottom');
            end

            % Set the x-tick labels to the names of the datasets
            set(gca, 'xtick', 1:totalPlots, 'xticklabel', allNames);
            xtickangle(45); % Angle the labels for better readability

            ylabel('Final Cumulative Energy (J)');
            title('Final Cumulative Energy Values');
            grid on;

            % Adjust the legend to be outside the plot for the bar chart as well
            % Since we're adding bars in a loop, the legend call should reference
            % dummy objects for clarity in legend creation
            legendEntries = arrayfun(@(x) plot(NaN, NaN, 'Color', myColors(x, :)), 1:totalPlots, 'UniformOutput', false);
            legend([legendEntries{:}], allNames, 'Location', 'eastoutside');

            hold off;


            % Save the figure as before
            filename = fullfile(obj.resultsPath, 'CumulativeEnergyAndBarChart.png');
            print(filename, '-dpng'); % '-dpng' specifies the format
        end

        function plotPowerProfilesOld(obj, withoutEnergyReward, withEnergyReward, uav)

            % for uav = 1:6
            [SOC,totali,voltage,postraj,refpostraj,time,timeb] = deal(withoutEnergyReward{uav,1},withoutEnergyReward{uav,2},withoutEnergyReward{uav,3}, ...
                withoutEnergyReward{uav,4},withoutEnergyReward{uav,5},withoutEnergyReward{uav,6},withoutEnergyReward{uav,7});

            [SOC1,totali1,voltage1,postraj1,refpostraj1,time1,timeb1] = deal(withEnergyReward{uav,1},withEnergyReward{uav,2},withEnergyReward{uav,3}, ...
                withEnergyReward{uav,4},withEnergyReward{uav,5},withEnergyReward{uav,6},withEnergyReward{uav,7});

            % plot3(postraj1(:,1),postraj1(:,2),postraj1(:,3))
            % hold on
            % plot3(refpostraj1(:,1),refpostraj1(:,2),refpostraj1(:,3))
            %
            % plot3(postraj(:,1),postraj(:,2),postraj(:,3))
            % hold on
            % plot3(refpostraj(:,1),refpostraj(:,2),refpostraj(:,3))
            % end

            powerProfile1 = totali.*voltage;
            powerProfile2 = totali1.*voltage1;
            % Ensure column vectors
            timeb = timeb(:);
            powerProfile1 = powerProfile1(:);
            timeb1 = timeb1(:);
            powerProfile2 = powerProfile2(:);

            % Create Figure
            figure('Color', 'w', 'Position', [100, 100, 800, 400]);

            % Main Plot: Power Profiles
            subplot(2,1,1);
            hold on;
            plot(timeb, powerProfile1, 'LineWidth', 2, 'DisplayName', 'Profile 1', 'Color', [0 0 1]);
            plot(timeb1, powerProfile2, 'LineWidth', 2, 'DisplayName', 'Profile 2', 'Color', [1 0 0]);
            xlabel('Time (s)');
            ylabel('Power (W)');
            title('Comparison of Power Profiles');
            legend('Location', 'Best');
            grid on;
            hold off;

            % Compute and Plot Cumulative Energy
            energyProfile1 = cumtrapz(timeb, powerProfile1);
            energyProfile2 = cumtrapz(timeb1, powerProfile2);

            subplot(2,1,2);
            hold on;
            plot(timeb, energyProfile1, 'LineWidth', 2, 'DisplayName', 'Energy without reward', 'Color', [0 0 1]);
            plot(timeb1, energyProfile2, 'LineWidth', 2, 'DisplayName', 'Energy with reward', 'Color', [1 0 0]);
            xlabel('Time (s)');
            ylabel('Energy (J)');
            title('Cumulative Energy Comparison');
            legend('Location', 'Best');
            grid on;

            % Display the actual accumulated energy at the last points
            text(timeb(end), energyProfile1(end), sprintf('%.2f J', energyProfile1(end)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 11, 'Color', [0 0 1]);
            text(timeb1(end), energyProfile2(end), sprintf('%.2f J', energyProfile2(end)), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'FontSize', 11, 'Color', [1 0 0]);
            hold off;

            filename = fullfile(obj.resultsPath,'EnergyComparison.png');
            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng'); % '-dpng' specifies the format, '-r300' sets the resolution (dpi)
        end



        function plotDataPoints(obj, originalData, interpolatedData, experimentNumbers, timeSteps)
            % Check if originalData and interpolatedData are numeric matrices
            if ~isnumeric(originalData) || ~isnumeric(interpolatedData)
                error('originalData and interpolatedData must be numeric matrices.');
            end

            % Check if experimentNumbers and timeSteps are provided and are numeric vectors
            if nargin < 3
                experimentNumbers = 1:20:10000; % Default if not provided
            end
            if nargin < 4
                timeSteps = 1:10; % Default if not provided
            end
            if ~isnumeric(experimentNumbers) || ~isnumeric(timeSteps)
                error('experimentNumbers and timeSteps must be numeric vectors.');
            end

            % Create a grid of experiment numbers and time steps for original data
            [X_original, Y_original] = meshgrid(linspace(1, 10000, size(originalData, 1)), timeSteps);

            % Create a grid of experiment numbers and time steps for interpolated data
            [X_interpolated, Y_interpolated] = meshgrid(experimentNumbers, timeSteps);

            % Reshape the data matrices to be column vectors for plotting
            Z_original = reshape(originalData', [], 1);
            Z_interpolated = reshape(interpolatedData', [], 1);
            X_original = reshape(X_original', [], 1);
            Y_original = reshape(Y_original', [], 1);
            X_interpolated = reshape(X_interpolated', [], 1);
            Y_interpolated = reshape(Y_interpolated', [], 1);

            % Create a figure and axes
            figure('Color', 'w'); % Set background color to white
            ax = axes;
            hold(ax, 'on');

            % Plot the original data points
            scatter3(ax, X_original, Y_original, Z_original, 25, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.8 0.8 0.8], 'DisplayName', 'Original', 'LineWidth', 0.5);

            % Plot the interpolated data points
            scatter3(ax, X_interpolated, Y_interpolated, Z_interpolated, 10, '.', 'MarkerEdgeColor', [0.1 0.6 0.9], 'DisplayName', 'Interpolated');

            % Label axes and add legend
            xlabel(ax, 'Experiment Number', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel(ax, 'Time Step', 'FontSize', 12, 'FontWeight', 'bold');
            zlabel(ax, 'Energy Req', 'FontSize', 12, 'FontWeight', 'bold');
            legend(ax, 'Location', 'best', 'FontSize', 10);
            title(ax, 'Original and Interpolated Energy Reqs', 'FontSize', 14, 'FontWeight', 'bold');
            grid(ax, 'on');
            ax.GridAlpha = 0.3; % Make grid lines less prominent
            ax.BoxStyle = 'full'; % Full box around plot
            ax.LineWidth = 1.5; % Make box lines thicker
            hold(ax, 'off');
            view(45, 30);
            filename = fullfile(obj.resultsPath,'InterpolatedEnergyReqs.png');

            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng'); % '-dpng' specifies the format, '-r300' sets the resolution (dpi)
        end


        function fittedValues = evaluateFittedFunctions(obj, fittedFunctions, experimentNumbers, timeSteps)
            % Check if fittedFunctions is a cell array of function handles
            if ~iscell(fittedFunctions) || ~all(cellfun(@isa, fittedFunctions, repmat({'function_handle'}, size(fittedFunctions))))
                error('fittedFunctions must be a cell array of function handles.');
            end

            % Check if experimentNumbers and timeSteps are numeric arrays
            if ~isnumeric(experimentNumbers) || ~isnumeric(timeSteps)
                error('experimentNumbers and timeSteps must be numeric arrays.');
            end

            % Initialize a matrix to store the fitted values
            fittedValues = zeros(length(experimentNumbers), length(timeSteps));

            % Loop through each time step
            for i = 1:length(timeSteps)
                % Get the fitted function for the current time step
                currentFittedFunction = fittedFunctions{i};

                % Loop through each experiment number
                for j = 1:length(experimentNumbers)
                    % Evaluate the fitted function at the current experiment number
                    fittedValues(j, i) = currentFittedFunction(experimentNumbers(j));
                end
            end
        end



        function fittedFunction = create2DFittedEnergyFunction(obj, sampleEnergy)
            % Check the size of sampleEnergy
            [numExperiments, numTimeSteps] = size(sampleEnergy);

            if numTimeSteps ~= 10
                error('The second dimension of sampleEnergy must be 10.');
            end

            % Define x and y
            x = 1:20:10000; % Original experiment numbers
            y = 1:numTimeSteps;

            % Create a grid of x and y values
            [X, Y] = meshgrid(x, y);

            % Reshape sampleEnergy to be a column vector
            Z = reshape(sampleEnergy', [], 1);

            % Reshape X and Y to be column vectors
            X = reshape(X, [], 1);
            Y = reshape(Y, [], 1);

            % Fit a 2D surface to the data
            [fittedSurface, gof] = fit([X, Y], Z, 'biharmonicinterp'); % try a different model

            % Create a function that takes an experiment number and time step and returns the values
            fittedFunction = @(experimentNumber, timeStep) feval(fittedSurface, [experimentNumber, timeStep]);

            % Optional: Display the fitted surface
            figure;
            plot(fittedSurface, [X, Y], Z);
            title('Fitted Surface');
            xlabel('Experiment Number');
            ylabel('Time Step');
            zlabel('Energy');
            legend('sampleEnergy', 'Fitted Surface');
            grid on;
        end

        % function plotFittedSurface(obj, fittedFunction)
        % end


        function rewards = calculateRewards(obj, powerConsumptionData)
            rewards = zeros(length(powerConsumptionData), 1);
            for i = 1:length(powerConsumptionData)
                totalPower = powerConsumptionData{i}.totalPower;
                normalizedPower = totalPower / obj.maxTotalPower;

                % Calculate penalty and reward
                penalty = 1 - normalizedPower;
                reward = normalizedPower;

                rewards(i) = reward;
            end
        end
    end
end
