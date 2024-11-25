classdef Planner < TrajectoryPlanning.Ownship & TrajectoryPlanning.EnergyEfficiency
    properties
        scenario
        totalAgents
        totalNMACs
        experimentName
        % smoothnessRewardRate
        % windRewardRate
        w_main
        w_wind
        w_energy
        w_path
        w_deviation
    end

    methods(Access = private)
        function hexagonPoints = generateHexagon(obj, centerPoint, apothem)
            % Generate hexagon points given a center and apothem
            angles = linspace(0, 2*pi, 7);
            angles = angles(1:end-1);
            x = centerPoint(1) + apothem * cos(angles);
            y = centerPoint(2) + apothem * sin(angles);
            z = ones(1,length(x))*1000;
            hexagonPoints = [x; y; z]';
        end
        function [initialStates, goals] = generateScenario(obj, N, scenarioType)
            % Generate scenario based on type and number of initial states
            switch lower(scenarioType)
                case {'hexagon'}
                    % Define Parameters
                    centerPoint = [0, 0];  % Center point
                    apothem = 1000;         % Length of points from the center

                    % Generate Hexagon Points
                    hexagonPoints = generateHexagon(obj, centerPoint, apothem);
                    goals = hexagonPoints;

                    % Initialize State Variables
                    numPoints = length(hexagonPoints);
                    X = zeros(numPoints, 1);
                    Y = zeros(numPoints, 1);
                    Z = hexagonPoints(:, 3);

                    % Define Orientation Angles
                    angleValue = 0;
                    psi = ones(numPoints, 1) * angleValue;
                    gamma = ones(numPoints, 1) * angleValue;
                    phi = ones(numPoints, 1) * angleValue;

                    % Combine Initial States
                    initialStates = [X, Y, Z, psi, gamma, phi, psi, gamma, phi, psi, gamma, phi];

                case {'circle'}
                    % N - Number Of vertiports
                    radius = 500;

                    % Generate Circle Points
                    points = linspace(0, 2*pi, N*10);
                    X = radius * cos(points(1:10:end))';
                    Y = radius * sin(points(1:10:end))' +500;
                    Z = repmat(100, length(X), 1);

                    % Define Orientation Angles
                    angleValue = -0.1;
                    psi = ones(length(X), 1) * angleValue;
                    gamma = psi;  % Same value as psi
                    phi = psi;     % Same value as psi
                    %         initialStates = [X', Y', Z', psi, gamma, phi, V]; %use this for fixed wing
                    initialStates = [X, Y, Z, psi, gamma, phi ,psi, gamma, phi, psi, gamma, phi];
                    goals = flipud(initialStates);
                case {'random'}
                    % N - Number of random initial points
                    hardDeck = 100;
                    lowAltitude = hardDeck + 200;
                    highAltitude = lowAltitude + 300;
                    upperX = 10000;
                    upperY = 10000;
                    upperZ = highAltitude;
                    lowerX = 100;
                    lowerY = 100;
                    lowerZ = lowAltitude;

                    % Generate Random Points
                    X = (lowerX + (upperX - lowerX) .* rand(1, N))';
                    Y = (lowerY + (upperY - lowerY) .* rand(1, N))';
                    Z = (lowerZ + (upperZ - lowerZ) .* rand(1, N))';

                    % Define Orientation Angles and Velocity
                    angleValue = 0.5;
                    psi = ones(N, 1) * angleValue;
                    gamma = psi;  % Same value as psi
                    phi = psi;    % Same value as psi
                    V = repmat(24.01, N, 1);
                    %         initialStates = [X', Y', Z', psi, gamma, phi, V]; %use this for fixed wing
                    initialStates = [X', Y', Z', psi, gamma, phi ,psi, gamma, phi, psi, gamma, phi];
                otherwise
                    error('scenario building on progress')
            end
        end
    end

    methods
        function obj = Planner(scenario, totalAgents, totalNMACs, experimentName,windDataPath)
            % Initialize the planner with scenario, total agents, and NMACs
            obj@TrajectoryPlanning.Ownship(windDataPath);
            obj@TrajectoryPlanning.EnergyEfficiency()   

            obj.scenario = scenario;
            obj.totalAgents = totalAgents;
            obj.totalNMACs = totalNMACs;
            obj.experimentName = experimentName;
        end

        function [initialStates, goals] = scenarioGenerator(obj, N, varargin)
            % Generate initial states and goals based on scenario type
            scenario = 'circle';
            if nargin > 1 && ischar(varargin{1})
                scenario = varargin{1};
            end
            [initialStates, goals] = obj.generateScenario(N, scenario);
        end
        % Walmart and destination locations (latitude, longitude)
        % walmartLocations = [
        %     -96.889458, 33.142295;  % Walmart 1
        %     -96.884394, 33.179801;  % Walmart 2
        %     -96.806245, 33.146822;  % Walmart 3
        %     -96.736346, 33.127893   % Walmart 4
        %     ];
        %
        % destinationLocations = [
        %     -96.879866, 33.145978;  % Destination 1
        %     -96.875167, 33.183636;  % Destination 2
        %     -96.795870, 33.148161;  % Destination 3
        %     -96.746260, 33.130678   % Destination 4
        %     ];
        function [assignedInitials, assignedGoals] = packageDeliveryScenario(obj)
            % Walmart and destination locations (latitude, longitude)
           walmartLocations = [
                -71.143513, 42.253617;  % Origin 
                -71.138921, 42.255935;  % Destination 1 
                -71.143513, 42.253617;  % Origin
                -71.144135, 42.249582   % Destination 2
                ];

            destinationLocations = [
                -71.138921, 42.255935;  % Destination 1
                -71.143513, 42.253617;  % Origin
                -71.144135, 42.249582;  % Destination 2
                -71.143513, 42.253617   % Origin
                ];


            % Bounding box for the area of interest
            lat_min = 42.230629;
            lat_max = 42.259;
            lon_min = -71.146753;
            lon_max = -71.137655;

            % Compute the center of the bounding box as the reference point (0, 0)
            center_lat = (lat_min + lat_max) / 2;
            center_lon = (lon_min + lon_max) / 2;

            % Average radius of Earth in meters
            R = 6371000;

            % Conversion factors (latitude and longitude to meters)
            metersPerDegreeLat = 111000;  % Approximate value for 1 degree of latitude in meters
            metersPerDegreeLon = metersPerDegreeLat * cos(deg2rad(center_lat));  % Adjust for longitude

            % Function to convert lat, lon to x, y (relative to center point)
            latLonToXY = @(lat, lon) [(lon - center_lon) * metersPerDegreeLon, (lat - center_lat) * metersPerDegreeLat];

            % Convert initial (Walmart) positions to x, y
            initialXY = zeros(size(walmartLocations, 1), 2);
            for i = 1:size(walmartLocations, 1)
                initialXY(i, :) = latLonToXY(walmartLocations(i, 2), walmartLocations(i, 1));
            end

            % Convert final (Destination) positions to x, y
            finalXY = zeros(size(destinationLocations, 1), 2);
            for i = 1:size(destinationLocations, 1)
                finalXY(i, :) = latLonToXY(destinationLocations(i, 2), destinationLocations(i, 1));
            end

            % Assigning initial (Walmart) positions and final (destination) positions with altitude
            assignedInitials = [initialXY, ones(size(initialXY, 1), 1) * 100];  % 100m altitude for all
            assignedGoals = [finalXY, ones(size(finalXY, 1), 1) * 100];  % 100m altitude for all

            % Plot initial and goal locations
            figure;
            hold on;
            plot(initialXY(:, 1), initialXY(:, 2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Initial Locations');
            plot(finalXY(:, 1), finalXY(:, 2), 'rx', 'MarkerSize', 10, 'DisplayName', 'Goal Locations');
            xlabel('X (meters)');
            ylabel('Y (meters)');
            title('Initial and Goal Locations in ENU Coordinates');
            legend('show');
            grid on;
            axis equal;
            hold off;
        end
        function [assignedInitials, assignedGoals, yawAngles] = packageDeliveryScenarioWithYaw(obj)
            % Walmart and destination locations (latitude, longitude)
            walmartLocations = [
                -71.138921, 42.255935;  % Origin 
                -71.143513, 42.253617;  % Destination 1 
                -71.138921, 42.255935; % Origin
                -71.144135, 42.249582   % Destination 2
                ];

            destinationLocations = [
                -71.143513, 42.253617;  % Destination 1
                -71.138921, 42.255935;  % Origin
                -71.144135, 42.249582;  % Destination 2
                -71.138921, 42.255935   % Origin
                ];

            % Bounding box for the area of interest
            lat_min = 42.230629;
            lat_max = 42.259;
            lon_min = -71.146753;
            lon_max = -71.137655;

            % Compute the center of the bounding box as the reference point (0, 0)
            center_lat = (lat_min + lat_max) / 2;
            center_lon = (lon_min + lon_max) / 2;

            % Average radius of Earth in meters
            R = 6371000;

            % Conversion factors (latitude and longitude to meters)
            metersPerDegreeLat = 111000;  % Approximate value for 1 degree of latitude in meters
            metersPerDegreeLon = metersPerDegreeLat * cos(deg2rad(center_lat));  % Adjust for longitude

            % Function to convert lat, lon to x, y (relative to center point)
            latLonToXY = @(lat, lon) [(lon - center_lon) * metersPerDegreeLon, (lat - center_lat) * metersPerDegreeLat];

            % Convert initial (Walmart) positions to x, y
            initialXY = zeros(size(walmartLocations, 1), 2);
            for i = 1:size(walmartLocations, 1)
                initialXY(i, :) = latLonToXY(walmartLocations(i, 2), walmartLocations(i, 1));
            end

            % Convert final (Destination) positions to x, y
            finalXY = zeros(size(destinationLocations, 1), 2);
            for i = 1:size(destinationLocations, 1)
                finalXY(i, :) = latLonToXY(destinationLocations(i, 2), destinationLocations(i, 1));
            end

            % Calculate yaw angles (in radians)
            yawAngles = zeros(size(initialXY, 1), 1);
            for i = 1:size(initialXY, 1)
                deltaX = finalXY(i, 1) - initialXY(i, 1);
                deltaY = finalXY(i, 2) - initialXY(i, 2);
                yawAngles(i) = atan2(deltaY, deltaX);  % Yaw angle
            end

            % Assigning initial (Walmart) positions and final (destination) positions with altitude
            assignedInitials = [initialXY, ones(size(initialXY, 1), 1) * 100];  % 100m altitude for all
            assignedGoals = [finalXY, ones(size(finalXY, 1), 1) * 100];  % 100m altitude for all

            % Plot initial and goal locations along with yaw directions
            figure;
            hold on;
            plot(initialXY(:, 1), initialXY(:, 2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Initial Locations');
            plot(finalXY(:, 1), finalXY(:, 2), 'rx', 'MarkerSize', 10, 'DisplayName', 'Goal Locations');

            % Plot yaw direction as arrows
            quiver(initialXY(:, 1), initialXY(:, 2), cos(yawAngles), sin(yawAngles), 0.3, 'k', 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Yaw Direction');

            xlabel('X (meters)');
            ylabel('Y (meters)');
            title('Initial and Goal Locations with Yaw Directions');
            legend('show');
            grid on;
            axis equal;
            hold off;
        end

        


        function yawAngles = initializeYaw(obj, assignedInitials,assignedGoals)
            % Assuming assignedInitials and assignedGoals are your arrays and are of size 6x3
            numUAVs = size(assignedInitials, 1);  % Number of UAVs
            yawAngles = zeros(numUAVs, 1);  % Initialize an array to store the yaw angles

            for i = 1:numUAVs
                % Calculate the difference in x and y coordinates between goals and initials
                deltaX = assignedGoals(i, 1) - assignedInitials(i, 1);
                deltaY = assignedGoals(i, 2) - assignedInitials(i, 2);

                % Calculate the yaw angle
                yaw = atan2(deltaY, deltaX);  % Yaw angle in radians

                % Store the yaw angle
                yawAngles(i) = yaw;
            end
        end

        function [assignedInitials, assignedGoals] = assignUAVPositions(obj, initialStates, goals)
            centerPosition = initialStates(1,1:3);
            % Number of UAVs
            numUAVs = 7;

            % Add the center position to the goals array to use it as one of the positions
            allPositions = [goals; centerPosition];

            % Ensure there are 7 unique positions
            if size(allPositions, 1) ~= numUAVs
                error('The number of unique positions must be equal to the number of UAVs');
            end

            % Initialize arrays to store assigned positions
            assignedInitials = zeros(numUAVs, 3);
            assignedGoals = zeros(numUAVs, 3);

            % Initialize a flag array to check if a UAV has been assigned
            assigned = false(numUAVs, 1);

            % Assign initial and goal positions
            while any(~assigned)
                shuffledIndices = randperm(numUAVs);
                for i = 1:numUAVs
                    if ~assigned(i)
                        initialIndex = shuffledIndices(i);
                        goalIndex = shuffledIndices(mod(i, numUAVs) + 1); % Ensures a different index for the goal

                        % Calculate the distance
                        distance = norm(allPositions(initialIndex, :) - allPositions(goalIndex, :));

                        % Check if the distance is within 1000 meters
                        if distance <= 1000
                            % Store the positions
                            assignedInitials(i, :) = allPositions(initialIndex, :);
                            assignedGoals(i, :) = allPositions(goalIndex, :);
                            assigned(i) = true;

                            % Print the assignments
                            fprintf('UAV %d: Initial Position = [%f, %f, %f], Goal Position = [%f, %f, %f], Distance = %f meters\n', ...
                                i, assignedInitials(i, 1), assignedInitials(i, 2), assignedInitials(i, 3), ...
                                assignedGoals(i, 1), assignedGoals(i, 2), assignedGoals(i, 3), distance);
                        end
                    end
                end
            end

            % Return the assigned initial and goal positions
            return;
        end

        function [positivePeaks, negativePeaks, droneList] = buildPeaks(obj, ownship, droneList, actions)
            % Builds a set of reward sources for a certain ownship

            % Extract Ownship Position
            [ownX, ownY, ownZ] = deal(ownship.position(1), ownship.position(2), ownship.position(3));

            % Initialize Negative Peaks
            negativePeaks = [];

            % Loop through Drone List
            for i = 1:numel(droneList)
                drone = droneList{i};

                % Skip if Ownship ID and Intruder ID are the same or Drone is Dead
                if drone.aircraftID == ownship.aircraftID || drone.dead
                    continue;
                end

                % Compute Negative Peaks if condition is met
                if false  % Replace with actual condition if needed
                    [intruderX, intruderY, intruderZ] = deal(drone.position(1), drone.position(2), drone.position(3));
                    distance = sqrt((ownX - intruderX)^2 + (ownY - intruderY)^2 + (ownZ - intruderZ)^2);

                    if distance < 2000
                        drone = computeIntruderTrajectory(obj, drone, actions);
                        [negativePeaks, drone] = computeNegativePeaks(obj, drone, negativePeaks);
                    end
                    droneList{i} = drone;
                end
            end

            % Compute Positive Peaks
            [goalX, goalY, goalZ] = deal(ownship.goal(1), ownship.goal(2), ownship.goal(3));
            positivePeaks = [500, goalX, goalY, goalZ, 0.999, inf];
        end

        function drone = computeIntruderTrajectory(obj, drone, actions)
            % Compute Intruder Trajectory if Traces are Zero
            if drone.Traces == 0
                drone = forwardSimulate(drone, actions, drone.timeStep, drone.numSteps);
            end
        end

        function [negativePeaks, drone] = computeNegativePeaks(obj, drone, negativePeaks)
            % Compute Negative Peaks based on Intruder Trajectory
            intruderTraj = drone.Traces;

            [Xmin, Xmax, Ymin, Ymax, Zmin, Zmax] = computeTrajectoryBounds(obj, intruderTraj);

            xRange = (Xmax - Xmin);
            yRange = (Ymax - Ymin);
            zRange = (Zmax - Zmin);
            center = [Xmin + xRange/2, Ymin + yRange/2, Zmin + zRange/2];
            radius = sqrt(xRange.^2 + yRange.^2 + zRange.^2) * 0.5;

            reward = linspace(500, 1500, length(xRange))';

            negativePeaks = [negativePeaks; [reward, center, repmat(0.97, length(xRange), 1), radius]];
        end

        function [Xmin, Xmax, Ymin, Ymax, Zmin, Zmax] = computeTrajectoryBounds(obj, intruderTraj)
            % Compute Bounds of Intruder Trajectory
            [Xmin, Xmax] = deal(min(intruderTraj(:, :, 1),[],'all'), max(intruderTraj(:, :, 1),[],'all'));
            [Ymin, Ymax] = deal(min(intruderTraj(:, :, 2),[],'all'), max(intruderTraj(:, :, 2),[],'all'));
            [Zmin, Zmax] = deal(min(intruderTraj(:, :, 3),[],'all'), max(intruderTraj(:, :, 3),[],'all'));
        end

        function [futureStates, oneStepStates, futureActions] = neighboringStates(obj, ownship, actions)
            % A function that returns next one step and few future states of the
            % aircraft
            % Input:
            %   - obj: [Object] The object calling this method
            %   - ownship: [Object] The aircraft whose future states are to be computed
            %   - actions: [Array] The actions to be taken
            % Output:
            %   - futureStates: [Array] The future states of the aircraft
            %   - oneStepStates: [Array] The state of the aircraft in the next time step
            %   - futureActions: [Array] The future actions of the aircraft
            % Usage:
            %   [futureStates, oneStepStates, futureActions] = neighboringStates(obj, ownship, actions)

            % Check for Collision
            if ownship.hit
                % Handle collision case here if needed
                % For example: assign zero thrust or modify actions
                ownship.Traces = 0;
                % Compute Future Trajectory
                futureTraj = computeFutureTrajectory(obj,ownship, actions);        
            elseif ownship.Traces == 0
                % Compute Future Trajectory
                futureTraj = computeFutureTrajectory(obj,ownship, actions);
            else
                futureTraj = ownship.Traces;
            end

            % save('futureTraj.mat','futureTraj')
            % 
            % X = futureTraj(:,1:2:end,1);
            % Y = futureTraj(:,1:2:end,2);
            % Z = futureTraj(:,1:2:end,3);
            % 
            % plot3(X,Y,Z)

            % if obj.smoothnessRewardRate == 0
            %     nextStep = 10;
            % else
            %     nextStep = 10;
            % end
            nextStep = 10;

            % Extract One Step and Future States
            oneStepStates = futureTraj(nextStep, :, :); % Change this to 100 for optimal performance 
            futIndex = floor(linspace(1, length(futureTraj(:, 1, 1)), 10));
            futureStates = futureTraj(futIndex, :, :); % Ensure size assertion as needed

            % Reshape Arrays for Next Operations
            futureActions = permute(repmat(actions, 1, 1, 10), [3 1 2]);
            oneStepStates = permute(repmat(oneStepStates, 10, 1, 1), [1 2 3]);
        end

        function futureTraj = computeFutureTrajectory(obj,ownship, actions)
            % Compute Future Trajectory of the Ownship

            if ownship.Traces == 0
                ownship = forwardSimulate(ownship, actions, ownship.timeStep, ownship.numSteps);
                futureTraj = ownship.Traces;
            else
                futureTraj = ownship.Traces;
            end
        end

        function [totalValues] = valueFunction(obj, ownship, states, positivePeaks, negativePeaks)
            % determine the value based on position
            % parameters;
            %indices for later use in value calculation
            RWD  = 1;
            LOCX = 2;
            LOCY = 3;
            LOCZ = 4;
            DISC = 5;
            LIM  = 6;

            numFuturePoints = length(states(:,1,1));
            lenFutureTraj = length(states(1,:,1));
            numStates = length(states(:,:,1));


            if ~isempty(positivePeaks)
                peakLocations = positivePeaks(:,[LOCX, LOCY, LOCZ]);
                rewardLimits = positivePeaks(:,LIM);
                discountFactor = positivePeaks(:, DISC);
                rewards = positivePeaks(:, RWD);
                A = states(:,:,1:3);

                reshapedStates = reshape(permute(A,[2 1 3]),[],size(A,3),1);
                positiveDistance = pdist2(reshapedStates, peakLocations(:,1:3));%computes pairwise distance b/n states and peaks
                [positiveValues,~] = obj.valueOptimized(positiveDistance, rewardLimits, discountFactor, rewards);
            else
                positiveValues = zeros(length(states));
            end

            if ~isempty(negativePeaks)
                peakLocations = negativePeaks(:,[LOCX, LOCY, LOCZ]);
                rewardLimits = negativePeaks(:,LIM)'; %transpose for value optimization
                discountFactor = negativePeaks(:, DISC)';
                rewards = negativePeaks(:, RWD)';
                B = states(:,:,1:3);

                reshapedStates = reshape(permute(B,[2 1 3]),[],size(B,3),1);
                negativeDistance = pdist2(reshapedStates, peakLocations(:,1:3));%
                %     negativeDistance = pdist2(reshape(states(:,:,1:3),[],size(states(:,:,1:3),3),1), peakLocations(:,1:3)); %change states to (:,:,1:3)
                [negativeValues, ~] = obj.valueOptimized(negativeDistance, rewardLimits, discountFactor, rewards);

                totalValues = positiveValues - negativeValues;

            else
                totalValues = positiveValues;
            end

            if true
                altitude = states(:,:,3)';
                altitude = altitude(:);
                belowLevel = altitude < 95;
                aboveLevel = altitude > 115;        
                penalityFunc_below = 5000 - altitude;
                % Define penalty for above 115m based on how much the altitude exceeds 115m
                scale_factor = 500;  % Adjust this to control penalty magnitude
                penalityFunc_above = scale_factor * (altitude - 115);  % Proportional to height increase above 115m

                % Apply penalties
                totalValues(belowLevel) = totalValues(belowLevel) - penalityFunc_below(belowLevel);
                totalValues(aboveLevel) = totalValues(aboveLevel) - penalityFunc_above(aboveLevel);
            end

            % This is the reward the deals with collision avoidance and
            % goal navigation
            mainReward = reshape(totalValues, lenFutureTraj, numFuturePoints)';

            % extract x and y pos and speeds of the aircraft for wind energy reward 
            x = states(:,:,1);
            y = states(:,:,2);
            Vx = states(:,:,4);
            Vy = states(:,:,5);
            
            % Interpolate wind speed components at aircraft position
            [Wx, Wy] = ownship.get_wind_at(x(:), y(:));
            Wx = reshape(Wx, lenFutureTraj, numFuturePoints)';
            Wy =  reshape(Wy, lenFutureTraj, numFuturePoints)';


            % windEnergyReward = calculateWindEnergyReward(obj, Vx, Vy, Wx, Wy);

            if true %obj.energyRewardRate ~= 0
                % Here, we handle energy cost related rewards
                distances = calculateDistances(obj,ownship.currentStates(1:3), states(:,:,1:3));
                energyReq = predictEnergy(obj,distances);
                energyReward = energyReq; %.*linspace(0,obj.smoothnessRewardRate,size(energyReq,1))'; %change energyRewardRate
                normalized_pathlengthReward = 0;
                normalized_deviationReward = 0;
                
                % Here, we hadle the deviation and path length rewards
                if size(ownship.traveledPath, 1) >= 2
                    deviationReward = computeTrajectoryDeviationReward(obj, ownship, states(:,:,1:3));

                    % deviationReward = computeDeviationReward(obj, ownship, states(:,:,1:3));
                    pathLengthReward = computeTrajectoryLengthReward(obj, ownship, states(:,:,1:3));
                    normalized_pathlengthReward = obj.normalizeReward(pathLengthReward, [-10000, 10000]);
                    normalized_deviationReward = obj.normalizeReward(-deviationReward, [-10000, 10000]);%.*logspace(0,-10,size(energyReward,1))';

                    debugReward = false;
                    if debugReward
                        % visualizeDeviationRewards(obj, ownship, states(:,:,1:3), calculateWindEnergyReward(obj, Vx, Vy, Wx, Wy));
                        % visualizeDeviationRewards(obj, ownship, states(:,:,1:3), -deviationReward)
                        % visualizeDeviationRewards(obj, ownship, states(:,:,1:3), smoothnessReward);
                        % visualizeDeviationRewards(obj, ownship, states(:,:,1:3), pathLengthRewards);
                        % visualizeDeviationRewards(obj, ownship, states(:,:,1:3), mainReward);
                    end
                end
                windReward = calculateWindEnergyReward(obj, Vx, Vy, Wx, Wy);
                
            else
                energyReward = 0;
            end

            % Now, let's normalize all rewards for easy tuning
            % Normalize each reward to [0, 1] or [-1, 1]
            normalized_mainReward = mainReward;%obj.normalizeReward(mainReward, [0, 100]);
            normalized_windReward = obj.normalizeReward(windReward, [-10000, 10000]);
            normalized_energyReward = obj.normalizeReward(energyReward, [-10000, 10000]);%.*logspace(0,-10,size(energyReward,1))';

            totalValues = obj.w_main * normalized_mainReward + ...
                obj.w_wind * normalized_windReward + ...
                obj.w_energy * normalized_energyReward + ...
                obj.w_path * normalized_pathlengthReward + ...
                obj.w_deviation * normalized_deviationReward;
        end

 

        function [optimizedValues, allValues] = valueOptimized(obj,distanceToPeaks, rewardLimits,discountFactor, rewards)

            propagationTable = distanceToPeaks < rewardLimits; %a boolean table

            minCycle = 2; % because its a grid world fundamentally

            peakValues = rewards ./ (1.0 - discountFactor.^(minCycle));

            allValues = propagationTable .* (peakValues .* discountFactor.^distanceToPeaks); %this is the key to all

            optimizedValues = max(allValues, [], 2);

        end

        function [terminal, NMACs, droneList] = terminalDetection(obj,droneList,stepCounter)
            NMACs = 0;
            NMACsPair = zeros(numel(droneList),numel(droneList));
            for own = 1:numel(droneList)

                ownship = droneList{own};

                %check if the ownship is dead
                if ownship.dead
                    continue
                end

                %check if the ownship violates the hit threshold
                ownship.hit = false;

                if ownship.hit
                    ownship.hitCounter = ownship.hitCounter + 1;
                end


                %check with respect to other intruders
                for intr = 1:numel(droneList)
                    if own == intr %intr is the index for intruders
                        continue
                    end

                    intruder = droneList{intr};

                    if intruder.dead
                        continue
                    end
                    %         distance = norm(ownship.currentStates(1:3) - intruder.currentStates(1:3));
                    hit_distance = norm(ownship.currentStates(1:3) - intruder.currentStates(1:3));
                    if hit_distance < 1000 %one step filter for n_x selection

                        ownship.hit = true;
                        if hit_distance < 100
                            % check if the case is already identified
                            if NMACsPair(ownship.aircraftID, intruder.aircraftID) == true || NMACsPair( intruder.aircraftID, ownship.aircraftID) == true
                                continue
                            end
                            % give a 100 meters allowance during initializing
                            if  stepCounter < 50
                                continue
                            end

                            NMACs = NMACs + 1;
                            NMACsPair(ownship.aircraftID, intruder.aircraftID) = true;

                        end
                    end

                end

                droneList{own} = ownship;
            end

            terminal = true;

            for drone = 1:numel(droneList)
                if ~droneList{drone}.dead
                    terminal = false;
                end
            end

        end
    end
end
