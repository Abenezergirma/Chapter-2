classdef Planner < TrajectoryPlanning.Ownship
    properties
        scenario
        totalAgents
        totalNMACs
        experimentName
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
                    apothem = 500;         % Length of points from the center

                    % Generate Hexagon Points
                    hexagonPoints = generateHexagon(obj, centerPoint, apothem);
                    goals = hexagonPoints;

                    % Initialize State Variables
                    numPoints = length(hexagonPoints);
                    X = zeros(numPoints, 1);
                    Y = zeros(numPoints, 1);
                    Z = hexagonPoints(:, 3);

                    % Define Orientation Angles
                    angleValue = -0.1;
                    psi = ones(numPoints, 1) * angleValue;
                    gamma = ones(numPoints, 1) * angleValue;
                    phi = ones(numPoints, 1) * angleValue;

                    % Combine Initial States
                    initialStates = [X, Y, Z, psi, gamma, phi, psi, gamma, phi, psi, gamma, phi];

                case {'circle'}
                    % N - Number Of vertiports
                    radius = 1500;

                    % Generate Circle Points
                    points = linspace(0, 2*pi, N*10);
                    X = radius * cos(points(1:10:end))';
                    Y = radius * sin(points(1:10:end))';
                    Z = repmat(100, length(X), 1);

                    % Define Orientation Angles
                    angleValue = -0.1;
                    psi = ones(length(X), 1) * angleValue;
                    gamma = psi;  % Same value as psi
                    phi = psi;     % Same value as psi
                    %         initialStates = [X', Y', Z', psi, gamma, phi, V]; %use this for fixed wing
                    initialStates = [X, Y, Z, psi, gamma, phi ,psi, gamma, phi, psi, gamma, phi];

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
        function obj = Planner(scenario, totalAgents, totalNMACs, experimentName)
            % Initialize the planner with scenario, total agents, and NMACs
            obj@TrajectoryPlanning.Ownship();
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
                if true  % Replace with actual condition if needed
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
            positivePeaks = [200, goalX, goalY, goalZ, 0.999, inf];
        end

        function drone = computeIntruderTrajectory(obj, drone, actions)
            % Compute Intruder Trajectory if Traces are Zero
            if drone.Traces == 0
                drone = forwardSimulate(drone, actions, 0.01, 500);
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
            [Xmin, Xmax] = deal(min(intruderTraj(end, :, 1)), max(intruderTraj(end, :, 1)));
            [Ymin, Ymax] = deal(min(intruderTraj(end, :, 2)), max(intruderTraj(end, :, 2)));
            [Zmin, Zmax] = deal(min(intruderTraj(end, :, 3)), max(intruderTraj(end, :, 3)));
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
            end

            % Compute Future Trajectory
            futureTraj = computeFutureTrajectory(obj,ownship, actions);

            % Extract One Step and Future States
            oneStepStates = futureTraj(10, :, :); % Ensure size assertion as needed
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
                belowLevel = altitude < 900;
                penalityFunc = 5000 - altitude;
                totalValues(belowLevel) = totalValues(belowLevel) - penalityFunc(belowLevel);
            end
            if strcmp(obj.experimentName,'withEnergyReward')
                % Add battery energy related rewards
                distances = calculateDistances(obj,ownship.currentStates(1:3), states(:,:,1:3));
                energyReq = 2067*exp(0.0128*distances);

                energyReward = energyReq.*linspace(0,0.1,size(energyReq,1))';
            else
                energyReward = 0;

                % totalValues = totalValues - altChange*1000*obj.energyRewardRate;
            end
            % energyReqs = obj.energyReqPerAction.interpolatedEnergyReqs*obj.energyRewardRate;

            totalValues = reshape(totalValues, lenFutureTraj, numFuturePoints)' - energyReward;
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
        % function [xyDistances, altitudeChange] = calculateDistances(obj, ownshipCurrentStates, states)
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
        %     % Calculate the (x,y) Euclidean distances
        %     xyDistances = sqrt(sum(deltaPosition(:,:,1:2).^2, 3));
        % 
        %     % Calculate the change in altitude (z)
        %     altitudeChange = deltaPosition(:,:,3);
        % end



        function [optimizedValues, allValues] = valueOptimized(obj,distanceToPeaks, rewardLimits,discountFactor, rewards)

            propagationTable = distanceToPeaks < rewardLimits; %a boolean table

            minCycle = 2; % because its a grid world fundamentally

            peakValues = rewards ./ (1.0 - discountFactor.^(minCycle));

            allValues = propagationTable .* (peakValues .* discountFactor.^distanceToPeaks); %this is the key to all

            optimizedValues = max(allValues, [], 2);

        end

        function [terminal, NMACs, droneList] = terminalDetection(obj,droneList)
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
                            if  norm(ownship.currentStates(1:3) - ownship.traveledPath(1,1:3)) < 200
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
