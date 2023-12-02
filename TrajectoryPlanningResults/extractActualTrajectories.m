function [xTraj, yTraj, zTraj] = extractTrajectories(results)
    numUAVs = size(results, 1); % Number of UAVs

    % Determine the maximum trajectory length (adding 2 for the new points)
    maxLength = 0;
    for i = 1:numUAVs
        traj = results{i, 4}; % Assuming trajectory is in the 4th column
        maxLength = max(maxLength, size(traj, 1));
    end
    maxLength = maxLength + 2; % Add 2 for the new start and end points

    % Initialize 3D arrays for x, y, and z trajectories
    xTraj = NaN(maxLength, numUAVs);
    yTraj = NaN(maxLength, numUAVs);
    zTraj = NaN(maxLength, numUAVs);

    % Fill in the trajectories
    for i = 1:numUAVs
        traj = results{i, 4};
        currentLength = size(traj, 1);

        % Add new start and end points
        startPoint = [traj(1,1), traj(1,2), 0];
        endPoint = [traj(end,1), traj(end,2), 0];
        traj = [startPoint; traj; endPoint];

        % Pad shorter trajectories
        if currentLength + 2 < maxLength
            x = [traj(:,1); repmat(traj(end,1), maxLength - currentLength - 2, 1)];
            y = [traj(:,2); repmat(traj(end,2), maxLength - currentLength - 2, 1)];
            z = [traj(:,3); repmat(0, maxLength - currentLength - 2, 1)]; % Use zero for padding z
        else
            % No padding needed, trajectories are already at maxLength
            x = traj(1:maxLength, 1);
            y = traj(1:maxLength, 2);
            z = traj(1:maxLength, 3);
        end

        xTraj(:,i) = x;
        yTraj(:,i) = y;
        zTraj(:,i) = z;
    end
end
