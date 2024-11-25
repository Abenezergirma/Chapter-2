classdef EnergyEfficiency < handle
    methods
        function obj = EnergyEfficiency()
            % Constructor for the EnergyEfficiency class
            % No specific initialization is needed for now
        end

        function normalizedReward = normalizeReward(obj, rewardArray, desiredRange)
            % Normalizes reward array to a specified range [min, max]
            minReward = min(rewardArray(:));
            maxReward = max(rewardArray(:));
            normalizedReward = (rewardArray - minReward) / (maxReward - minReward);
            rangeSpan = desiredRange(2) - desiredRange(1);
            normalizedReward = normalizedReward * rangeSpan + desiredRange(1);
        end

        function R_length = computeTrajectoryLengthReward(obj, ownship, future_states)
            % Computes trajectory length reward for each future state
            s_G = ownship.goal;
            s_0 = ownship.traveledPath(1, 1:3);
            past_trajectory = ownship.traveledPath;
            delta_t = ownship.timeStep*10;
            T = size(future_states, 1);
            num_actions = size(future_states, 2);
            R_length = zeros(T, num_actions);

            diffs_past = diff(past_trajectory, 1, 1);
            L_past = sum(vecnorm(diffs_past, 2, 2));

            for t = 1:T
                if t < T
                    diffs_horizon = diff(future_states(t:end, :, :), 1, 1);
                    L_horizon = sum(vecnorm(diffs_horizon, 2, 3), 1);
                else
                    L_horizon = zeros(1, num_actions);
                end
                s_T = squeeze(future_states(T, :, :));
                L_goal = vecnorm(s_G - s_T, 2, 2)';
                L_total = L_past + L_horizon + L_goal;
                R_length(t, :) = -L_total;
            end
            % num_samples = 10;  % Choose how many future states to sample
            % visualizeTrajectoryLength3D(obj, ownship, future_states, num_samples);
        end

        function visualizeTrajectoryLength3D(obj, ownship, future_states, num_samples)
            % Visualizes the trajectory length components in 3D with a subset of future states
            s_0 = ownship.traveledPath(1, 1:3);  % Initial state
            s_G = ownship.goal;  % Goal state
            past_trajectory = ownship.traveledPath;  % Past traveled path
            T = size(future_states, 1);
            num_actions = size(future_states, 2);

            % Choose a time step to visualize (for instance, T=1 for current state)
            t_vis = 1;  % You can change this to visualize different time steps
            future_state_t = squeeze(future_states(t_vis, :, :));  % Current future state for actions

            % Sample a subset of the future states to plot
            sample_indices = round(linspace(1, num_actions, num_samples));

            % Create 3D plot
            figure;
            hold on;

            % Plot the past trajectory (Initial to current state)
            plot3(past_trajectory(:, 1), past_trajectory(:, 2), past_trajectory(:, 3), '-b', 'LineWidth', 2);
            scatter3(s_0(1), s_0(2), s_0(3), 100, 'filled', 'r');  % Initial state in red

            % Plot sampled future states (Current state to sampled future states)
            for i = sample_indices
                % Line between current state and future states
                plot3([past_trajectory(end, 1), future_state_t(i, 1)], ...
                    [past_trajectory(end, 2), future_state_t(i, 2)], ...
                    [past_trajectory(end, 3), future_state_t(i, 3)], '--g');
                scatter3(future_state_t(i, 1), future_state_t(i, 2), future_state_t(i, 3), 50, 'filled', 'g');  % Sampled future states in green

                % Line from future state to goal
                plot3([future_state_t(i, 1), s_G(1)], ...
                    [future_state_t(i, 2), s_G(2)], ...
                    [future_state_t(i, 3), s_G(3)], '--r');
            end

            % Plot goal state
            scatter3(s_G(1), s_G(2), s_G(3), 100, 'filled', 'k');  % Goal state in black

            % Labels and legend
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('3D Visualization of Sampled Trajectory Lengths');
            legend('Past Trajectory', 'Initial State', 'Future States', 'Distance to Future', 'Goal State', 'Distance to Goal');

            grid on;
            axis equal;
            hold off;
        end


        function deviations = OldcomputeTrajectoryDeviationReward(obj, ownship, states)
            % Computes deviation reward based on future states
            s_G = ownship.goal;
            s_t = states;
            v = 20;
            delta_t = ownship.timeStep*10;
            s_0 = ownship.traveledPath(1, 1:3);
            pastT = length(ownship.traveledPath(:, 1));
            dist_0_G = norm(s_G - s_0);
            T = dist_0_G / (v * delta_t);
            [num_time_steps, num_actions, ~] = size(s_t);
            t = (1:num_time_steps)' + pastT;
            s_ref = s_0 + (t / T) * (s_G - s_0);
            s_ref_repeated = repmat(s_ref, 1, num_actions, 1);
            s_ref_repeated = reshape(s_ref_repeated, num_time_steps, num_actions, 3);
            deviations = sqrt(sum((s_t - s_ref_repeated).^2, 3));

            num_total_steps = 100;
            visualizeTrajectoryDeviation(obj, ownship, states, num_total_steps, s_ref);

            plot(ownship.currentStates(1), ownship.currentStates(2), 'rx', 'MarkerSize', 10, 'DisplayName', 'Goal Locations');
            yaw = ownship.currentStates(9);
            hold on
            plot(s_ref(:,1), s_ref(:,2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Initial Locations');
            quiver(ownship.currentStates(1), ownship.currentStates(2), cos(yaw), sin(yaw), 3.3, 'k', 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Yaw Direction');
            plot(states(:,1:1:100,1), states(:,1:1:100,2));


            % visualizeDeviationRewards(obj, ownship, states, deviations);
        end

        function deviations = computeTrajectoryDeviationReward(obj, ownship, states)
            % Computes deviation reward based on angle between state vectors and goal vector
            s_G = ownship.goal;  % Goal state [1x3]
            s_t = states;  % Future states [10x3087x3]
            s_0 = ownship.traveledPath(1, 1:3);  % Current state [1x3]
            % Number of time steps and actions for future states
            [num_time_steps, num_actions, ~] = size(states);

            % Create the vector representing the straight line from the current state to the goal
            goal_vector = s_G - s_0;  % Vector from current state to goal [1x3]

            % Normalize the goal vector
            norm_goal_vector = sqrt(sum(goal_vector.^2));  % Norm of the goal vector [scalar]
            goal_vector_normalized = goal_vector / norm_goal_vector;  % Normalized goal vector [1x3]

            % Create vectors from the current state to each future state
            vectors_to_states = s_t - reshape(s_0, 1, 1, 3);  % Vectors from s_0 to each future state [10x3087x3]

            % Compute the norms of the vectors to future states
            norms_states = sqrt(sum(vectors_to_states.^2, 3));  % Norms of each future state vector [10x3087]

            % Normalize the vectors to future states
            vectors_to_states_normalized = vectors_to_states ./ norms_states;  % Normalized future state vectors [10x3087x3]

            % Replicate goal_vector_normalized across the dimensions of vectors_to_states_normalized
            goal_vector_normalized_repeated = repmat(reshape(goal_vector_normalized, 1, 1, 3), num_time_steps, num_actions, 1);


            % Compute the dot product between the normalized goal vector and each normalized future state vector
            % This computes the cosine of the angle between the two vectors
            cos_theta = sum(vectors_to_states_normalized .* goal_vector_normalized_repeated, 3);  % Dot product [10x3087]

            % Clip cos_theta to the range [-1, 1] to avoid issues with acos
            cos_theta = min(max(cos_theta, -1), 1);  % Ensure values are within [-1, 1] [10x3087]

            % Compute the angles (in radians)
            deviations = acos(cos_theta);  % Deviations as angles [10x3087]

            % plot_3D_trajectories_and_goal(obj, s_0, s_G, states, 'trajectory_goal.png');  % Save the plot as 'trajectory_goal.png'.
            % plot_3D_trajectory_deviation_heatmap(obj, s_0, s_G, states, 'trajectory_goal_heatmap.png')

            % Visualization step
            % num_total_steps = 100;
            % visualizeTrajectoryDeviation(obj, ownship, states, num_total_steps, s_G);
            % visualizeDeviationRewards(obj, ownship, states, deviations, s_G);
        end




        function visualizeTrajectoryDeviation(obj, ownship, states, num_total_steps, ten_ref)
            % Extract key variables from ownship
            s_G = ownship.goal;           % Goal position
            s_0 = ownship.traveledPath(1, 1:3);  % Initial position
            past_trajectory = ownship.traveledPath;  % Past traveled path
            delta_t = ownship.timeStep;    % Time step duration
            v = 5;  % Assumed average velocity

            % Compute the total distance from start to goal
            dist_0_G = norm(s_G - s_0);

            % Number of time steps and actions for future states
            [num_time_steps, num_actions, ~] = size(states);

            % Generate the reference trajectory for the exact number of time steps
            t = linspace(0, dist_0_G / (v * delta_t), num_time_steps)';
            s_ref = s_0 + (t / t(end)) * (s_G - s_0);  % Ideal reference trajectory (num_time_steps x 3)

            % Repeat s_ref to match the size of states (num_time_steps x num_actions x 3)
            s_ref_repeated = repmat(s_ref, [1, num_actions, 1]);
            s_ref_repeated = reshape(s_ref_repeated, num_time_steps, num_actions, 3);


            % Compute the deviations between the future states and reference trajectory
            deviations = sqrt(sum((states - s_ref_repeated).^2, 3)) * 100;

            % Visualize past trajectory, reference trajectory, and future states
            figure;
            hold on;
            grid on;

            % Plot past trajectory
            plot3(past_trajectory(:, 1), past_trajectory(:, 2), past_trajectory(:, 3), 'b-o', 'LineWidth', 2, 'DisplayName', 'Past Trajectory');

            % Plot reference trajectory (which is a straight line from start to goal)
            % plot3(s_ref(:, 1), s_ref(:, 2), s_ref(:, 3), 'g--', 'LineWidth', 2, 'DisplayName', 'Reference Trajectory');

            % Highlight the first 10 time steps of the reference trajectory
            if num_time_steps >= 10
                plot3(ten_ref(:, 1), ten_ref(:, 2), ten_ref(:, 3), 'r', 'LineWidth', 2, 'DisplayName', 'First 10 Steps of Ref. Trajectory');
            else
                plot3(ten_ref(:, 1), ten_ref(:, 2), ten_ref(:, 3), 'g-', 'LineWidth', 2, 'DisplayName', 'First Steps of Ref. Trajectory');
            end

            % Plot future states (for each action)
            for i = 1:num_actions
                % Extract the future state trajectory for each action
                % future_traj = squeeze(states(:, i, :));
                % Plot the connecting line
                % plot3(future_traj(:, 1), future_traj(:, 2), future_traj(:, 3), 'r--', 'LineWidth', 0.5);
            end

            % Visualize deviations (deviation between future states and reference trajectory)
            futureStateFlat = reshape(states, [], 3);  % Flatten to a (num_time_steps*num_actions)x3 matrix
            deviationRewardsFlat = reshape(deviations, [], 1);  % Flatten to a (num_time_steps*num_actions)x1 vector

            scatter3(futureStateFlat(:, 1), futureStateFlat(:, 2), futureStateFlat(:, 3), 8, deviationRewardsFlat, 'filled');
            plot3(states(:,:,1),states(:,:,2),states(:,:,3),'black')
            % Mark goal position
            % plot3(s_G(1), s_G(2), s_G(3), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Goal');

            % Add labels and legend
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title('Trajectory Deviation Visualization');
            % legend('Location', 'best');
            colorbar;
            caxis([min(deviationRewardsFlat), max(deviationRewardsFlat)]);

            % Set axis ranges for consistent debugging
            % axis_range = [-100, 100, -100, 100, -100, 100];  % Define fixed axis limits [X_min, X_max, Y_min, Y_max, Z_min, Z_max]
            % axis(axis_range);  % Set fixed axis limits

            % Set 3D view
            view(3);
            hold off;
        end



        function distances = calculateDistances(obj, ownshipCurrentStates, states)
            % Extract sizes
            numExperiments = size(states, 2);
            numWaypoints = size(states, 1);

            % Expand the reference state to match the dimensions of states
            expandedReference = repmat(ownshipCurrentStates, [numWaypoints, 1, numExperiments]);

            % Permute expandedReference to match the desired dimensions
            expandedReference = permute(expandedReference, [1, 3, 2]);

            % Calculate the change in position (deltaPosition) relative to the reference
            deltaPosition = states - expandedReference;

            % Calculate the Euclidean distances with altitude sign adjustment
            distances = sqrt(sum(deltaPosition.^2, 3));
        end

        function totalReward = calculateWindEnergyReward(obj, vx, vy, wx, wy)
            % Calculates wind energy reward based on velocity vectors
            dotProducts = vx .* wx + vy .* wy;
            magsV = sqrt(vx.^2 + vy.^2);
            magsW = sqrt(wx.^2 + wy.^2);
            cosTheta = dotProducts ./ (magsV .* magsW);
            cosTheta = max(min(cosTheta, 1), -1);
            A = 200;
            B = 300;
            totalReward = (A * magsW .* cosTheta + B);
        end

        function energyArray = predictEnergy(obj, distances)
            % Predicts energy consumption based on distances
            aCoeffs = [0, 520.699, 203.742, 127.606, 86.3024, 71.8159, 63.1053, 57.9294, 50.7294, 46.399];
            bCoeffs = [0, 369.168, 869.01, 1119.61, 1459.52, 1368.97, 1279.89, 1128.54, 1144.29, 1183.82];
            energyArray = aCoeffs' .* distances + bCoeffs';
        end

        function visualizeDeviationRewards(obj, ownship, futureState, deviationRewards, s_G)
            % previousState: 1x3 array containing the [x, y, z] position of the aircraft at the previous time step
            % currentState: 1x3 array containing the [x, y, z] position of the aircraft at the current time step
            % futureState: 10x3430x3 array containing the [x, y, z] positions of the aircraft for 3430 possible future trajectories over 10 time steps
            % deviationRewards: 10x3430 array containing the deviation penalty for each future state
            previousState = ownship.traveledPath(end-1,1:3);
            currentState = ownship.currentStates(1:3);

            % Flatten the futureState and deviationRewards for easier plotting
            numTrajectories = size(futureState, 2);
            futureStateFlat = reshape(futureState, [], 3);  % Flatten to a 34300x3 matrix
            deviationRewardsFlat = reshape(deviationRewards, [], 1);  % Flatten to a 34300x1 vector

            % Normalize the deviation rewards for color mapping
            normDeviationRewards = (deviationRewardsFlat - min(deviationRewardsFlat)) / (max(deviationRewardsFlat) - min(deviationRewardsFlat));

            % Create a figure for the 3D plot with a smaller size
            % figureHandle = figure;
            % set(figureHandle, 'Units', 'Inches', 'Position', [1, 1, 3.5, 2.5]);  % Width = 6.5 inches, Height = 3.5 inches

            hold on;
            grid on;

            % Plot the previous and current states
            % plot3(previousState(1), previousState(2), previousState(3), 'bo', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Previous State');
            % plot3(currentState(1), currentState(2), currentState(3), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Current State');
            % Plot the black thin line for each trajectory
            numTrajectories = size(futureState, 2);
            for i = 1:numTrajectories
                % Extract the trajectory points
                trajPoints = squeeze(futureState(:, i, :));

                % Plot the connecting line
                plot3(trajPoints(:, 1), trajPoints(:, 2), trajPoints(:, 3), 'k-', 'LineWidth', 0.5);
            end
            % Scatter plot for all future trajectories using color based on actual deviation rewards
            scatter3(futureStateFlat(:, 1), futureStateFlat(:, 2), futureStateFlat(:, 3), 8, deviationRewardsFlat, 'filled');

            % Plot the goal state in red
            scatter3(s_G(1), s_G(2), s_G(3), 100, 'r', 'filled', 'DisplayName', 'Goal State');

            % Add labels and legend
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            % title('Future Trajectories and Deviation Rewards');
            % legend('Location', 'best');

            % Optional: add a colorbar to represent the reward scale
            colormap(jet);
            c = colorbar;
            c.Label.String = 'Deviation';%Deviation Reward';
            caxis([min(deviationRewardsFlat), max(deviationRewardsFlat)]);

            % Adjust view
            % view([145, 30]);
            view(3);

            % Set the filename for the PNG image
            filename = 'Future_Trajectories_Deviation_Rewards.png';

            % Save the figure with transparency
            saveas(gcf, 'Deviation_Rewards', 'png');
            set(gcf,'InvertHardcopy','off');  % Ensure the figure is saved with transparency

            hold off;
            % Set the filename for the PNG image
            filename = 'Future_Trajectories_Deviation_Rewards.png';

            % Use the print function to save the figure as a PNG image
            print(filename, '-dpng', '-r300'); % '-dpng' specifies the format
        end

        function plot_3D_trajectories_and_goal(obj, s_0, s_G, states, output_file)
            % Plot 3D trajectories of the aircraft and the goal position with transparency.
            % Inputs:
            % - s_0: Initial UAV state [1x3].
            % - s_G: Goal state [1x3].
            % - states: Future states [10x3087x3].
            % - output_file: Name of the file to save the figure (e.g., 'trajectory_goal.png').

            figure;
            hold on;
            grid on;

            % Plot the initial state in black
            scatter3(s_0(1), s_0(2), s_0(3), 100, 'k', 'filled', 'DisplayName', 'Initial State');

            % Plot the goal state in red
            scatter3(s_G(1), s_G(2), s_G(3), 100, 'r', 'filled', 'DisplayName', 'Goal State');

            % Plot the trajectories in black with transparency
            alpha_value = 0.3;  % Transparency value (0: fully transparent, 1: fully opaque)
            [num_time_steps, num_actions, ~] = size(states);

            % Use vectorized plotting to improve speed
            all_lines_x = reshape([repmat(s_0(1), num_time_steps*num_actions, 1), reshape(states(:, :, 1), [], 1)]', 2, []);
            all_lines_y = reshape([repmat(s_0(2), num_time_steps*num_actions, 1), reshape(states(:, :, 2), [], 1)]', 2, []);
            all_lines_z = reshape([repmat(s_0(3), num_time_steps*num_actions, 1), reshape(states(:, :, 3), [], 1)]', 2, []);

            % Plot the lines in a single call
            plot3(all_lines_x, all_lines_y, all_lines_z, 'k-', 'LineWidth', 1, 'Color', [0, 0, 0, alpha_value]);

            % Set labels and title
            xlabel('X Position');
            ylabel('Y Position');
            zlabel('Z Position');
            title('3D Trajectories of Aircraft and Goal Position');
            % legend('Location', 'best');
            axis tight;
            view(3);

            % Save the figure with transparency
            saveas(gcf, output_file, 'png');
            set(gcf,'InvertHardcopy','off');  % Ensure the figure is saved with transparency

            hold off;
        end


    end
end
%%
% Set figure size and initialize
% figure('Position', [100, 100, 400, 400]);  % 4x4 inch plot
%
% % Identify tailwind and headwind regions
% tailwind = windReward > 0;  % Tailwind: Positive wind reward
% headwind = windReward < 0;  % Headwind: Negative wind reward
%
% % Scatter plot for tailwind zones (light green, with some transparency)
% scatter(x(tailwind), y(tailwind), 15, 'MarkerFaceColor', [0.3, 0.8, 0.3], 'MarkerEdgeColor', 'none', ...
%         'DisplayName', 'Tailwind', 'MarkerFaceAlpha', 0.7);
% hold on;
%
% % Scatter plot for headwind zones (light red, with some transparency)
% scatter(x(headwind), y(headwind), 15, 'MarkerFaceColor', [0.9, 0.3, 0.3], 'MarkerEdgeColor', 'none', ...
%         'DisplayName', 'Headwind', 'MarkerFaceAlpha', 0.7);
%
% % Plot the (x, y) trajectories as lines with alpha set to 0.5 (no legend for this)
% plot(x, y, 'k-', 'LineWidth', .5, 'Color', [0, 0, 0, 0.01]);  % Black lines with alpha = 0.5 for transparency
%
% % Axis labels and title
% xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
% title('Tailwind and Headwind Zones', 'FontSize', 14, 'FontWeight', 'bold');
%
% % Add legend inside the figure (no legend for the black trajectory lines)
% legend('Tailwind', 'Headwind', 'Location', 'best');
%
% % Grid and axis settings
% grid on;
% set(gca, 'GridLineStyle', '--', 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.7);  % Light grid
% axis tight;  % Ensure the axis fits well
%
%
% % Set font size for axes
% set(gca, 'FontSize', 10, 'LineWidth', 2);  % Increase axis border LineWidth
%
% % Set bold axis border
% set(gca, 'box', 'on', 'LineWidth', 1.5);  % Ensure that the box around the axes is on and bold
%
% % Adjust tick parameters to make axis lines more prominent
% % set(gca, 'TickDir', 'out', 'TickLength', [0.02 0.02], 'LineWidth', 2);  % Makes the ticks and border bold
%
% % Adjust figure layout to maintain proper sizing
% set(gca, 'Position', [0.15, 0.15, 0.75, 0.75]);  % Adjust position to fit the plot and legend
%
% % Save the figure as a transparent PNG
% saveas(gcf, 'Tailwind_Headwind_Zones.png');

% % Set figure size and initialize
% figure('Position', [100, 100, 400, 400]);  % 4x4 inch plot
%
% % Subsample the data to reduce arrow density
% subsample_rate = 5;  % Adjust this value to control density
% x_sub = x(1:subsample_rate:end, 1:subsample_rate:end);
% y_sub = y(1:subsample_rate:end, 1:subsample_rate:end);
% Wx_sub = Wx(1:subsample_rate:end, 1:subsample_rate:end);
% Wy_sub = Wy(1:subsample_rate:end, 1:subsample_rate:end);
% Vx_sub = Vx(1:subsample_rate:end, 1:subsample_rate:end);
% Vy_sub = Vy(1:subsample_rate:end, 1:subsample_rate:end);
%
% % Plot wind field
% quiver(x_sub, y_sub, Wx_sub, Wy_sub, 0.25, 'b', 'LineWidth', 1.0, 'DisplayName', 'Wind Field');  % Wind field in blue
% hold on;
%
% % Plot vehicle speed field
% quiver(x_sub, y_sub, Vx_sub, Vy_sub, 0.25, 'r', 'LineWidth', 1.0, 'DisplayName', 'Vehicle Speed');  % Vehicle speed in red
%
% % Plot the (x, y) trajectories as lines with alpha set to 0.5 (no legend for this)
% plot(x_sub, y_sub, 'k-', 'LineWidth', 0.1, 'Color', [0, 0, 0, 0.2]);  % Black lines with alpha = 0.5 for transparency
%
% % Axis labels and title
% xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
% ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
% title('Wind and Aircraft Speed Field', 'FontSize', 14, 'FontWeight', 'bold');
%
% % Add legend inside the figure (no legend for the black trajectory lines)
% legend('Wind Field', 'Aircraft Speed', 'Location', 'northeast');
%
% % Grid and axis settings
% grid on;
% set(gca, 'GridLineStyle', '--', 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.7);  % Light grid
% axis tight;  % Ensure the axis fits well
%
% % Set font size for axes
% set(gca, 'FontSize', 10, 'LineWidth', 1.5);
%
% % Adjust figure layout to maintain proper sizing
% set(gca, 'Position', [0.15, 0.15, 0.75, 0.75]);  % Adjust position to fit the plot and legend
%
% % Save the figure as a transparent PNG
% saveas(gcf, 'Wind_VehicleSpeed_Field_with_Trajectories.png');
% %%

% visualizeDeviationRewards(obj, ownship, states(:,:,1:3), windReward)

% function normalizedReward = normalizeReward(obj, rewardArray, desiredRange)
%     % rewardArray: 10x3430 array containing the rewards to be normalized
%     % desiredRange: Two-element vector specifying the desired range [min, max]
%     % normalizedReward: 10x3430 array containing the normalized rewards
%
%     minReward = min(rewardArray(:));
%     maxReward = max(rewardArray(:));
%
%     % Normalize to [0, 1]
%     normalizedReward = (rewardArray - minReward) / (maxReward - minReward);
%
%     % Scale to desired range
%     rangeSpan = desiredRange(2) - desiredRange(1);
%     normalizedReward = normalizedReward * rangeSpan + desiredRange(1);
% end
%
% function R_length = computeTrajectoryLengthReward(obj, ownship, future_states)
%     % This function computes the trajectory length reward.
%     % Inputs:
%     %   ownship - object containing the goal, traveledPath, and timeStep properties
%     %   future_states - array of future states (10x3430x3, where 10 is the time step,
%     %                  3430 is the number of actions, and 3 is the x, y, z position)
%     %
%     % Output:
%     %   R_length - array of trajectory length rewards for all future states (10x3430)
%
%     % Extract relevant data from the ownship object
%     s_G = ownship.goal;  % Goal position (1x3 array)
%     s_0 = ownship.traveledPath(1, 1:3);  % Initial position from traveled path
%     past_trajectory = ownship.traveledPath;  % Array of all past positions
%     delta_t = ownship.timeStep;  % Time step duration
%     T = size(future_states, 1);  % Horizon length (time steps)
%
%     % Number of actions (3430)
%     num_actions = size(future_states, 2);
%
%     % Initialize R_length to store results for each time step and action
%     R_length = zeros(T, num_actions);
%
%     % 1. Compute L_past: cumulative distance of past trajectory
%     diffs_past = diff(past_trajectory, 1, 1);  % Differences in past trajectory
%     L_past = sum(vecnorm(diffs_past, 2, 2));   % Sum of Euclidean distances in past trajectory
%
%     % Loop over each time step to compute the trajectory length reward
%     for t = 1:T
%         % 2. Compute L_horizon: predicted distance from time step t to T
%         if t < T
%             diffs_horizon = diff(future_states(t:end, :, :), 1, 1);  % Differences in future states starting from t
%             L_horizon = sum(vecnorm(diffs_horizon, 2, 3), 1);  % Horizon distance for each action
%         else
%             L_horizon = zeros(1, num_actions);  % No horizon distance at final time step
%         end
%
%         % 3. Compute L_goal: straight-line distance from state at horizon T to goal
%         s_T = squeeze(future_states(T, :, :));  % Final state at horizon for each action
%         L_goal = vecnorm(s_G - s_T, 2, 2)';  % Euclidean distance from horizon state to goal
%
%         % Total trajectory length for each action at time step t
%         L_total = L_past + L_horizon + L_goal;
%
%         % Trajectory length reward for time step t (negative of total length)
%         R_length(t, :) = -L_total;
%     end
% end
%
%
%
%
%
% function deviations = computeTrajectoryDeviationReward(obj, ownship, states)
%     % This function computes the trajectory deviation reward for future states.
%     % Inputs:
%     %   ownship - object containing the goal, traveledPath, and timeStep properties
%     %   states - array of future states (10x3430x3)
%     %
%     % Output:
%     %   deviations - array of deviations for all future states (10x3430 array)
%
%     % Extract relevant data from the ownship object
%     s_G = ownship.goal;  % Goal position (1x3 array)
%     s_t = states;        % Future states (10x3430x3 array)
%     v = 5;               % Estimated average speed (constant)
%     delta_t = ownship.timeStep;  % Time step duration
%     s_0 = ownship.traveledPath(1, 1:3);  % Initial position from traveled path
%     pastT = length(ownship.traveledPath(:, 1));  % Length of the traveled path
%
%     % Calculate the Euclidean distance between start and goal positions
%     dist_0_G = norm(s_G - s_0);
%
%     % Estimate the total number of time steps to reach the goal
%     T = dist_0_G / (v * delta_t);
%
%     % Number of time steps and actions
%     [num_time_steps, num_actions, ~] = size(s_t);
%
%     % Precompute reference trajectory points for all time steps
%     t = (1:num_time_steps)' + pastT;  % Adjust time step by adding pastT
%     s_ref = s_0 + (t / T) * (s_G - s_0);  % Interpolate reference trajectory for all steps
%
%     % Repeat the reference trajectory for all actions (10x3430x3)
%     s_ref_repeated = repmat(s_ref, 1, num_actions, 1);
%     s_ref_repeated = reshape(s_ref_repeated, num_time_steps, num_actions, 3);
%
%     % Compute the deviations from the reference trajectory (10x3430 array)
%     deviations = sqrt(sum((s_t - s_ref_repeated).^2, 3))*100;
% end
%
%
% function distances = calculateDistances(obj, ownshipCurrentStates, states)
%     % Extract sizes
%     numExperiments = size(states, 2);
%     numWaypoints = size(states, 1);
%
%     % Expand the reference state to match the dimensions of states
%     expandedReference = repmat(ownshipCurrentStates, [numWaypoints, 1, numExperiments]);
%
%     % Permute expandedReference to match the desired dimensions
%     expandedReference = permute(expandedReference, [1, 3, 2]);
%
%     % Calculate the change in position (deltaPosition) relative to the reference
%     deltaPosition = states - expandedReference;
%
%     % Calculate the Euclidean distances with altitude sign adjustment
%     distances = sqrt(sum(deltaPosition.^2, 3));
% end
%
% function totalReward = calculateWindEnergyReward(obj, vx, vy, wx, wy)
%     % Compute dot products and magnitudes
%     dotProducts = vx .* wx + vy .* wy;
%     magsV = sqrt(vx.^2 + vy.^2);
%     magsW = sqrt(wx.^2 + wy.^2);
%
%     % Compute cosine of the angle
%     cosTheta = dotProducts ./ (magsV .* magsW);
%
%     % Ensure the cosine values are within the valid range [-1, 1]
%     cosTheta = max(min(cosTheta, 1), -1);
%
%     % Constants
%     A = 200;
%     B = 300;
%
%     % plot(A*magsW.*cosTheta,'o')
%
%     % Calculate the reward
%     reward = (A * magsW .* cosTheta + B) ;
%
%     % Return the total reward
%     totalReward = reward;
% end
%
%
% function energyArray = predictEnergy(obj,distances)
%     % Define the coefficients for each step
%     aCoeffs = [0, 520.699, 203.742, 127.606, 86.3024, 71.8159, 63.1053, 57.9294, 50.7294, 46.399];
%     bCoeffs = [0, 369.168, 869.01, 1119.61, 1459.52, 1368.97, 1279.89, 1128.54, 1144.29, 1183.82];
%
%     % Calculate the energy using the linear function in a vectorized way
%     energyArray = aCoeffs' .* distances + bCoeffs';
% end
