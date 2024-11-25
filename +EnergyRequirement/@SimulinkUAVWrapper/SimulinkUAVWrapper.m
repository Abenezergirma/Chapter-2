classdef SimulinkUAVWrapper < handle 
    properties
        timeStep = 0.1;
        modelPath
        modelScript
        experimentName
        resultsPath
        totalCurrent
        BatteryParams
        SOC
        voltage
        actualPosTraj
        refPosTraj
        time
        timeb
        batterySampleTime
        uavSampleTime
    end

    methods
        function obj = SimulinkUAVWrapper(modelPath)
            % Constructor
            obj.modelPath = modelPath;
            % obj.modelScript = modelScript;
            % obj.experimentName = experimentName;
            % obj.resultsPath = resultsPath;
        end


        function runModel(obj, setWaypoints)
            % Run the Simulink model with the specified inputs
            cd(obj.modelPath);

            numWaypoints = length(setWaypoints(1,:,1));

            % Preallocate a cell array to store the results
            results = cell(numWaypoints,7);

            for i = 1:numWaypoints
                fprintf('Currently running %d-th experiment\n', i);
                wayPoint = squeeze(setWaypoints(:,i,:));
                [results{i,1},results{i,2},results{i,3},results{i,4},results{i, 5},results{i,6},results{i,7}]...
                    = runDetailedT18Model(wayPoint);
            end

            % Save the output of simulation
            cd(fileparts(mfilename('fullpath')));
            resultFileName = obj.experimentName;
            resultFilePath = fullfile(obj.resultsPath, resultFileName);
            save(resultFilePath, 'results');
        end

        function saveOutputData(obj, resultFilePath)
            % This method saves all important properties of the object for future experiments
            SOC  =  obj.SOC;
            totalCurrent = obj.totalCurrent;
            voltage = obj.voltage;
            actualPosTraj = obj.actualPosTraj;
            refPosTraj = obj.refPosTraj;
            time = obj.time;
            timeb = obj.timeb;
            batterySampleTime = obj.batterySampleTime;
            uavSampleTime = obj.uavSampleTime;
            save(resultFilePath, 'totalCurrent','voltage','SOC','actualPosTraj', ...
                'refPosTraj','time','timeb',"batterySampleTime","uavSampleTime");
        end

        function [x_combined, y_combined, z_combined] = preprocessTrajectories(obj, xTraj, yTraj, zTraj)
            % Preprocess trajectories by combining delivery and return missions, and
            % adding takeoff and landing waypoints.
            %
            % Input:
            %   - xTraj: MxN matrix of x coordinates (M is number of aircraft, N is number of timestamps)
            %   - yTraj: MxN matrix of y coordinates
            %   - zTraj: MxN matrix of z coordinates (altitude)
            %
            % Output:
            %   - x_combined: Combined x trajectories with added waypoints
            %   - y_combined: Combined y trajectories with added waypoints
            %   - z_combined: Combined z trajectories with takeoff and landing waypoints

            % Number of timestamps
            [numAircraft, numTimestamps] = size(xTraj);

            % Check if the number of aircraft is even (required for pairing delivery and return missions)
            if numAircraft ~= 4
                error('The number of aircraft must be 4 to properly pair the missions.');
            end

            % Preallocate arrays for the combined trajectories
            x_combined = [];
            y_combined = [];
            z_combined = [];

            % Combine aircraft 1 and 3, and aircraft 2 and 4
            for i = 1:2:numAircraft
                % Combine the delivery and return mission (aircraft i and i+1)

                % Add takeoff waypoint at 70 meters altitude for the first segment (delivery)
                takeoffWaypointX1 = xTraj(i, 1); % Starting x position
                takeoffWaypointY1 = yTraj(i, 1); % Starting y position
                takeoffWaypointZ1 = 70;          % Takeoff altitude

                % Add landing waypoint at 70 meters altitude for the first segment (delivery)
                landingWaypointX1 = xTraj(i, end); % Ending x position of delivery
                landingWaypointY1 = yTraj(i, end); % Ending y position of delivery
                landingWaypointZ1 = 70;            % Landing altitude after delivery

                % Add takeoff waypoint at 70 meters altitude for the second segment (return)
                takeoffWaypointX2 = xTraj(i+1, 1); % Starting x position for return
                takeoffWaypointY2 = yTraj(i+1, 1); % Starting y position for return
                takeoffWaypointZ2 = 70;            % Takeoff altitude for return

                % Add landing waypoint at 70 meters altitude for the second segment (return)
                landingWaypointX2 = xTraj(i+1, end); % Ending x position of return
                landingWaypointY2 = yTraj(i+1, end); % Ending y position of return
                landingWaypointZ2 = 70;              % Landing altitude after return

                % Combine the waypoints with each mission's trajectory (delivery and return)
                x_mission = [takeoffWaypointX1, xTraj(i, :), landingWaypointX1, takeoffWaypointX2, xTraj(i+1, :), landingWaypointX2];
                y_mission = [takeoffWaypointY1, yTraj(i, :), landingWaypointY1, takeoffWaypointY2, yTraj(i+1, :), landingWaypointY2];
                z_mission = [takeoffWaypointZ1, zTraj(i, :), landingWaypointZ1, takeoffWaypointZ2, zTraj(i+1, :), landingWaypointZ2];

                % Add the combined trajectory to the final arrays
                x_combined = [x_combined; x_mission];
                y_combined = [y_combined; y_mission];
                z_combined = [z_combined; z_mission];
            end
        end




        function smoothedTrajectory = smoothTrajectory(obj, originalTrajectory, smoothness)
            % originalTrajectory: Nx3 matrix containing [x, y, z] positions of the UAV
            % smoothness: scalar indicating the number of points for interpolation (higher value = smoother)
            % smoothedTrajectory: Mx3 matrix containing smoothed [x, y, z] positions

            % Ensure originalTrajectory has enough points
            if size(originalTrajectory, 1) < 2
                error('original Trajectory must contain at least two points.');
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

        function [x_combined, y_combined, z_combined] = sampleAndCombineTrajectories(obj, xTraj, yTraj, zTraj, numWaypoints)
            % This function samples waypoints for each segment (delivery and return),
            % adds takeoff and landing waypoints, and then merges the segments.
            %
            % Input:
            %   - xTraj: MxN matrix of x coordinates (M is number of aircraft, N is number of timestamps)
            %   - yTraj: MxN matrix of y coordinates
            %   - zTraj: MxN matrix of z coordinates (altitude)
            %   - numWaypoints: number of waypoints to sample for each trajectory segment
            %
            % Output:
            %   - x_combined: Combined x trajectories with added waypoints
            %   - y_combined: Combined y trajectories with added waypoints
            %   - z_combined: Combined z trajectories with takeoff and landing waypoints

            % Number of aircraft and timestamps
            [numAircraft, numTimestamps] = size(xTraj);

            % Check if the number of aircraft is even (required for pairing delivery and return missions)
            if numAircraft ~= 4
                error('The number of aircraft must be 4 to properly pair the missions.');
            end

            % Preallocate arrays for the combined trajectories
            x_combined = [];
            y_combined = [];
            z_combined = [];

            % Loop through each pair of aircraft (1-3, 2-4)
            for i = 1:2:numAircraft
                % Sample waypoints for the first segment (delivery)
                x_delivery = obj.sampleTrajectory(xTraj(i, :), numWaypoints);
                y_delivery = obj.sampleTrajectory(yTraj(i, :), numWaypoints);
                z_delivery = obj.sampleTrajectory(zTraj(i, :), numWaypoints);

                % Sample waypoints for the second segment (return)
                x_return = obj.sampleTrajectory(xTraj(i+1, :), numWaypoints);
                y_return = obj.sampleTrajectory(yTraj(i+1, :), numWaypoints);
                z_return = obj.sampleTrajectory(zTraj(i+1, :), numWaypoints);

                % Add takeoff and landing waypoints for the delivery segment
                takeoffWaypointX1 = xTraj(i, 1); % Starting x position
                takeoffWaypointY1 = yTraj(i, 1); % Starting y position
                takeoffWaypointZ1 = 20;          % Takeoff altitude

                landingWaypointX1 = xTraj(i, end); % Ending x position of delivery
                landingWaypointY1 = yTraj(i, end); % Ending y position of delivery
                landingWaypointZ1 = 20;            % Landing altitude after delivery

                % Add takeoff and landing waypoints for the return segment
                takeoffWaypointX2 = xTraj(i+1, 1); % Starting x position for return
                takeoffWaypointY2 = yTraj(i+1, 1); % Starting y position for return
                takeoffWaypointZ2 = 20;            % Takeoff altitude for return

                landingWaypointX2 = xTraj(i+1, end); % Ending x position of return
                landingWaypointY2 = yTraj(i+1, end); % Ending y position of return
                landingWaypointZ2 = 20;              % Landing altitude after return

                % Combine the takeoff, delivery segment, landing for the first part
                x_combined_delivery = [takeoffWaypointX1, x_delivery, landingWaypointX1];
                y_combined_delivery = [takeoffWaypointY1, y_delivery, landingWaypointY1];
                z_combined_delivery = [takeoffWaypointZ1, z_delivery, landingWaypointZ1];

                % Combine the takeoff, return segment, landing for the second part
                x_combined_return = [takeoffWaypointX2, x_return, landingWaypointX2];
                y_combined_return = [takeoffWaypointY2, y_return, landingWaypointY2];
                z_combined_return = [takeoffWaypointZ2, z_return, landingWaypointZ2];

                % Finally, merge the delivery and return segments into one mission
                x_combined = [x_combined; x_combined_delivery, x_combined_return];
                y_combined = [y_combined; y_combined_delivery, y_combined_return];
                z_combined = [z_combined; z_combined_delivery, z_combined_return];
            end
        end

        % Helper function to sample waypoints from a trajectory
        function sampledTrajectory = sampleTrajectory(obj, trajectory, numWaypoints)
            % Create a parameterization for the original trajectory
            tOriginal = 1:length(trajectory);

            % Create a parameterization for the sampled waypoints
            tSampled = linspace(1, length(trajectory), numWaypoints);

            % Sample the trajectory using linear interpolation
            sampledTrajectory = interp1(tOriginal, trajectory, tSampled, 'linear');
        end


        function sampledWaypoints = smoothAndSampleTrajectory(obj, originalTrajectories, smoothness, sigma, numWaypoints, plotTrajectories)
            % originalTrajectories: NxMx3 matrix containing [x, y, z] positions of M UAVs
            % smoothness: scalar indicating the number of points for interpolation (higher value = smoother)
            % sigma: standard deviation for Gaussian smoothing (larger value = smoother)
            % numWaypoints: scalar indicating the number of waypoints to sample
            % plotTrajectories: boolean to indicate whether to plot the trajectories
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

            % Initialize the matrix to store sampled waypoints
            sampledWaypoints = zeros(numWaypoints, M, dim);

            % Define Gaussian filter parameters
            gaussFilter = fspecial('gaussian', [smoothness, 1], sigma);

            % Loop through each UAV
            for i = 1:M
                % Extract x, y, z for the current UAV
                trajectory = squeeze(originalTrajectories(:, i, :));

                % Initialize matrices for smoothed dimensions
                smoothedDimensions = zeros(smoothness, dim);

                % Apply Gaussian filtering for x, y, z
                for d = 1:dim
                    % Perform linear interpolation to ensure smooth trajectory sampling
                    interpTrajectory = interp1(tOriginal, trajectory(:, d), tSmooth, 'linear');
                    % Apply Gaussian smoothing filter
                    smoothedDimensions(:, d) = imfilter(interpTrajectory, gaussFilter, 'replicate');
                end

                % Assign the smoothed dimensions to the output
                smoothedTrajectories(:, i, :) = smoothedDimensions;

                % Sample waypoints from the smoothed trajectory
                % Compute the cumulative arc length of the smoothed trajectory
                arcLength = [0; cumsum(sqrt(sum(diff(smoothedDimensions).^2, 2)))];

                % Ensure arcLength has unique values
                [arcLength, uniqueIdx] = unique(arcLength, 'stable');
                smoothedDimensions = smoothedDimensions(uniqueIdx, :);

                % Interpolate to get waypoints that are evenly spaced in arc length
                samplePoints = linspace(0, arcLength(end), numWaypoints);
                for d = 1:dim
                    sampledWaypoints(:, i, d) = interp1(arcLength, smoothedDimensions(:, d), samplePoints, 'linear');
                end
            end

            % Plotting (if plotTrajectories is true)
            if plotTrajectories
                figure;
                hold on;
                for i = 1:M
                    % Plot original trajectory
                    plot3(originalTrajectories(:, i, 1), originalTrajectories(:, i, 2), originalTrajectories(:, i, 3), 'r--', 'DisplayName', 'Original Trajectory','LineWidth',2);
                    % Plot smoothed trajectory
                    plot3(smoothedTrajectories(:, i, 1), smoothedTrajectories(:, i, 2), smoothedTrajectories(:, i, 3), 'b-', 'DisplayName', 'Smoothed Trajectory', 'LineWidth',2);
                    % Plot sampled waypoints
                    plot3(sampledWaypoints(:, i, 1), sampledWaypoints(:, i, 2), sampledWaypoints(:, i, 3), 'go', 'MarkerSize', 8, 'DisplayName', 'Sampled Waypoints', 'LineWidth',2);
                    % Connect sampled waypoints with straight lines
                    plot3(sampledWaypoints(:, i, 1), sampledWaypoints(:, i, 2), sampledWaypoints(:, i, 3), 'k-', 'DisplayName', 'Waypoint Connections');
                end
                hold off;
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                grid on;
                legend show;
                title('Trajectory Smoothing and Waypoint Sampling');
                % Save the plot
                saveas(gcf,  '2S_trajectory.png');
                close(gcf);  % Close the figure after saving
            end
        end

        function sampledWaypoints = smoothAndSampleTrajectoryNEW(obj, originalTrajectories, smoothness, sigma, numWaypoints, plotTrajectories)
            % originalTrajectories: NxMx3 matrix containing [x, y, z] positions of M UAVs
            % smoothness: scalar indicating the number of points for interpolation (higher value = smoother)
            % sigma: standard deviation for Gaussian smoothing (larger value = smoother)
            % numWaypoints: scalar indicating the number of waypoints to sample
            % plotTrajectories: boolean to indicate whether to plot the trajectories
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

            % Initialize the matrix to store sampled waypoints
            sampledWaypoints = zeros(numWaypoints, M, dim);

            % Define Gaussian filter parameters
            gaussFilter = fspecial('gaussian', [smoothness, 1], sigma);

            % Directory to save the plots
            saveDir = 'trajectory_plots/';
            if ~exist(saveDir, 'dir')
                mkdir(saveDir);  % Create directory if it doesn't exist
            end

            % Loop through each UAV
            for i = 1:M
                % Extract x, y, z for the current UAV
                trajectory = squeeze(originalTrajectories(:, i, :));

                % Initialize matrices for smoothed dimensions
                smoothedDimensions = zeros(smoothness, dim);

                % Apply Gaussian filtering for x, y, z
                for d = 1:dim
                    % Perform linear interpolation to ensure smooth trajectory sampling
                    interpTrajectory = interp1(tOriginal, trajectory(:, d), tSmooth, 'linear');
                    % Apply Gaussian smoothing filter
                    smoothedDimensions(:, d) = imfilter(interpTrajectory, gaussFilter, 'replicate');
                end

                % Assign the smoothed dimensions to the output
                smoothedTrajectories(:, i, :) = smoothedDimensions;

                % Sample waypoints from the smoothed trajectory
                % Compute the cumulative arc length of the smoothed trajectory
                arcLength = [0; cumsum(sqrt(sum(diff(smoothedDimensions).^2, 2)))];

                % Ensure arcLength has unique values
                [arcLength, uniqueIdx] = unique(arcLength, 'stable');
                smoothedDimensions = smoothedDimensions(uniqueIdx, :);

                % Interpolate to get waypoints that are evenly spaced in arc length
                samplePoints = linspace(0, arcLength(end), numWaypoints);
                for d = 1:dim
                    sampledWaypoints(:, i, d) = interp1(arcLength, smoothedDimensions(:, d), samplePoints, 'linear');
                end

                % Plotting (if plotTrajectories is true)
                if plotTrajectories
                    figure;
                    hold on;
                    % Plot original trajectory
                    plot3(originalTrajectories(:, i, 1), originalTrajectories(:, i, 2), originalTrajectories(:, i, 3), ...
                        'r--', 'DisplayName', 'Original Trajectory', 'LineWidth', 2);
                    % Plot smoothed trajectory
                    plot3(smoothedTrajectories(:, i, 1), smoothedTrajectories(:, i, 2), smoothedTrajectories(:, i, 3), ...
                        'b-', 'DisplayName', 'Smoothed Trajectory', 'LineWidth', 3);
                    % Plot sampled waypoints
                    plot3(sampledWaypoints(:, i, 1), sampledWaypoints(:, i, 2), sampledWaypoints(:, i, 3), ...
                        'go', 'MarkerSize', 10, 'DisplayName', 'Sampled Waypoints', 'LineWidth', 2);
                    % Connect sampled waypoints with straight lines
                    plot3(sampledWaypoints(:, i, 1), sampledWaypoints(:, i, 2), sampledWaypoints(:, i, 3), ...
                        'k-', 'DisplayName', 'Waypoint Connections', 'LineWidth', 2);

                    hold off;
                    xlabel('X');
                    ylabel('Y');
                    zlabel('Z');
                    grid on;
                    title(['Trajectory for Aircraft ', num2str(i)]);
                    legend('show');
                    set(gca, 'FontSize', 14);
                    set(gcf, 'Position', [100, 100, 800, 600]);  % Adjust figure size

                    % Save the plot
                    saveas(gcf, fullfile(saveDir, ['aircraft_', num2str(i), '_trajectory.png']));
                    close(gcf);  % Close the figure after saving
                end
            end
        end



        function plotTrajectoriesAndWaypoints(obj, trajectory1, waypoints1, trajectory2, waypoints2)
            % Set up figure
            figure;
            hold on;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Trajectories and Waypoints');
            grid on;

            % Initialize legend information
            h = []; % Handles for the plots
            labels = {}; % Labels for the plots

            % Function to plot each trajectory and waypoints
            function plotSet(trajectory, waypoints, trajColor, wpColor, label)
                uav = 6;
                x_traj = trajectory(:, uav,1);
                y_traj = trajectory(:, uav,2);
                z_traj = trajectory(:, uav,3)-950;

                x_waypoints = waypoints(:, uav,1);
                y_waypoints = waypoints(:, uav,2);
                z_waypoints = waypoints(:, uav, 3)-950;

                % Plot the trajectory
                h_traj = plot3(x_traj, y_traj, z_traj, trajColor, 'LineWidth', 1.5);
                % Plot the waypoints
                scatter3(x_waypoints, y_waypoints, z_waypoints, 100, wpColor, 'filled');

                % Collect the handles and labels for legend
                h(end+1) = h_traj;
                labels{end+1} = label;
            end

            % Plot the first trajectory and waypoints with label for "Traj without reward"
            plotSet(trajectory1, waypoints1, 'b-', 'ro', 'Traj without reward');

            % Plot the second trajectory and waypoints with label for "Traj with reward"
            plotSet(trajectory2, waypoints2, 'g-', 'mo', 'Traj with reward');

            % Add legend
            legend(h, labels, 'Location', 'best');

            % Adjust view and display the plot
            view(3);
            axis equal;
            hold off;

            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath,'trajectoryAndWaypoints.png');

            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng'); % '-dpng' specifies the format
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

        function generateBatteryParametersPlot(obj,results,uav)
            % Unpack the data
            [SOC,totali,voltage,postraj,refpostraj,time,timeb] = deal(results{uav,1},results{uav,2},results{uav,3}, ...
                results{uav,4},results{uav,5},results{uav,6},results{uav,7});
            % Create a new figure for the first subplot
            figure;

            % Subplot for SOC and voltage
            subplot(2, 1, 1);
            plot(timeb, voltage, 'r', 'LineWidth', 1.5);
            ylabel('Voltage');
            xlabel('Time (s)');
            legend('Voltage', 'Location', 'northeast');
            % yyaxis left;
            % plot(timeb, SOC*100, 'b', 'LineWidth', 1.5);
            % ylabel('SOC');
            % ylim([0, 100]);
            % yyaxis right;
            % plot(timeb, voltage, 'r', 'LineWidth', 1.5);
            % ylabel('Voltage');
            % xlabel('Time (s)');
            % legend('SOC', 'Voltage', 'Location', 'southwest');
            % legend('Totali', 'Location', 'Best'); % Adjust legend location
            title('Voltage');

            % Subplot for totali
            subplot(2, 1, 2);
            plot(timeb, totali, 'g', 'LineWidth', 1.5);
            ylabel('TotalCurrent');
            xlabel('Time (s)');
            title('TotalCurrent');
            legend('Current', 'Location', 'northeast');

            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath,'TotalCurrent_noenergyReward.png');
            print(filename, '-dpng');

            % Create a new figure for the second subplot
            figure;

            % Plot postraj and refpostraj
            plot3(postraj(:, 1), postraj(:, 2), postraj(:, 3)-950, 'b', 'LineWidth', 1.5);
            hold on;
            % plot3(refpostraj(:, 1), refpostraj(:, 2), refpostraj(:, 3)-950, 'r', 'LineWidth', 1.5);
            hold off;
            xlabel('X, m');
            ylabel('Y, m');
            zlabel('Z, m');
            legend('actualTraj', 'refTraj', 'Location', 'Best');
            title('Octo-copter Trajectory');

            % Adjust the figure layout
            axis equal;
            grid on;
            % Set the filename for the PNG image
            filename = fullfile(obj.resultsPath,'T18-Trajectories_noenergyReward.png');
            print(filename, '-dpng');
        end

    end
end