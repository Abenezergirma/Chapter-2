classdef Planner < TrajectoryPlanning.Ownship
    properties
        scenario
        totalAgents
        totalNMACs
        experimentName
        smoothnessRewardRate
        windRewardRate
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
        function obj = Planner(scenario, totalAgents, totalNMACs, experimentName,windDataPath,center_lon,center_lat)
            % Initialize the planner with scenario, total agents, and NMACs
            obj@TrajectoryPlanning.Ownship(windDataPath,center_lon,center_lat);
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
        function [assignedInitials, assignedGoals] = packageDeliveryScenario(obj)
            % Setup DC package delivery scenario
            % GWU = [38.900529, -77.049058];
            % sichuanPavilion = [38.902443, -77.042052];
            % watergateHotel = [38.899612, -77.055408];
            % initialPositions = [GWU; sichuanPavilion; watergateHotel];
            % 
            % traderJos = [38.904193, -77.053129]; %GWU
            % tonicAtQuincy = [38.898055, -77.046471]; %GWU
            % foundingFarmers = [38.90028, -77.044415]; %sichuanPavilion
            % theRitzHotel = [38.904526, -77.049072]; %sichuanPavilion
            % fourSeason = [38.904269, -77.057294]; %watergateHotel
            % bakedAndWired = [38.903887, -77.060437]; %watergateHotel
            % finalPositions = [traderJos;foundingFarmers;tonicAtQuincy;theRitzHotel;fourSeason;bakedAndWired];
            Maydan = [38.920027, -77.03127];
            watergateHotel = [38.899612, -77.055408];
            JoesStake = [38.900027, -77.033922];
            initialPositions = [Maydan; JoesStake; watergateHotel];
            cornerLat = [ 38.9228   38.9228   38.8990   38.8990];
            cornerLon = [-77.0274  -77.0580  -77.0580  -77.0274];

            PhilipsCollection = [38.91151, -77.046854];%Maydan
            wholeFoolds = [38.909499, -77.033568];%Maydan
            Dumbarton = [38.910883, -77.055618];%watergateHotel
            foundingFarmers = [38.90028, -77.044415]; %watergateHotel
            agoralDC = [38.91063, -77.038251];%JoesStake
            % MuseumMansion = [38.908363, -77.045874];%JoesStake
            AperoLa = [38.909246, -77.055494];%JoesStake
            % finalPositions = [PhilipsCollection;wholeFoolds;Dumbarton;foundingFarmers;agoralDC;AperoLa];

            finalPositions = [PhilipsCollection;wholeFoolds;Dumbarton;foundingFarmers;agoralDC;AperoLa];
            Dupont = [38.910944, -77.042726, 10];

            origin = Dupont;
            % Average radius of Earth in meters
            R = 6371000;

            % Conversion factors
            metersPerDegreeLat = 111000;
            metersPerDegreeLon = metersPerDegreeLat * cos(deg2rad(origin(1)));

            % Function to convert lat, lon to x, y
            latLonToXY = @(lat, lon) [(lon - origin(2)) * metersPerDegreeLon, (lat - origin(1)) * metersPerDegreeLat];

            % Convert initial positions
            initialXY = zeros(size(initialPositions, 1), 2);
            for i = 1:size(initialPositions, 1)
                initialXY(i, :) = latLonToXY(initialPositions(i, 1), initialPositions(i, 2));
            end

            % Convert final positions
            finalXY = zeros(size(finalPositions, 1), 2);
            for i = 1:size(finalPositions, 1)
                finalXY(i, :) = latLonToXY(finalPositions(i, 1), finalPositions(i, 2));
            end

            assignedInitials = [initialXY(repmat(1:size(initialXY, 1), 2, 1), :),ones(6,1)*1000];
            assignedGoals = [finalXY,ones(6,1)*1000];
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

            if obj.smoothnessRewardRate == 0
                nextStep = 10;
            else
                nextStep = 100;
            end

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
                belowLevel = altitude < 80;
                penalityFunc = 5000 - altitude;
                totalValues(belowLevel) = totalValues(belowLevel) - penalityFunc(belowLevel);
            end
            % extract x and y pos and speeds of the aircraft for wind energy reward 
            x = states(:,:,1);
            y = states(:,:,2);
            Vx = states(:,:,4);
            Vy = states(:,:,5);
            

            % Interpolate wind speed components at aircraft position
            % Wx = interp2(ownship.X_grid, ownship.Y_grid, ownship.grid_U, x, y, 'linear');
            % Wy = interp2(ownship.X_grid, ownship.Y_grid, ownship.grid_V, x, y, 'linear');
            [Wx, Wy] = calculateWindComponents(ownship, x(:), y(:));
            Wx = reshape(Wx, lenFutureTraj, numFuturePoints)';
            Wy = reshape(Wy, lenFutureTraj, numFuturePoints)';


            % windEnergyReward = calculateWindEnergyReward(obj, Vx, Vy, Wx, Wy);

            if true %obj.energyRewardRate ~= 0
                % Add battery energy related rewards
                distances = calculateDistances(obj,ownship.currentStates(1:3), states(:,:,1:3));
                % % energyReq = 2067*exp(0.0128*distances);
                energyReq = predictEnergy(obj,distances);
                % 
                smoothnessReward = energyReq.*linspace(0,obj.smoothnessRewardRate,size(energyReq,1))'; %change energyRewardRate
                
                windReward = calculateWindEnergyReward(obj, Vx, Vy, Wx, Wy)*obj.windRewardRate;
                energyReward = windReward - smoothnessReward; %double check the +/- logic here 
            else
                energyReward = 0;

                % totalValues = totalValues - altChange*1000*obj.energyRewardRate;
            end
            % energyReqs = obj.energyReqPerAction.interpolatedEnergyReqs*obj.energyRewardRate;

            totalValues = reshape(totalValues, lenFutureTraj, numFuturePoints)' + energyReward;
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
            % Compute dot products and magnitudes
            dotProducts = vx .* wx + vy .* wy;
            magsV = sqrt(vx.^2 + vy.^2);
            magsW = sqrt(wx.^2 + wy.^2);

            % Compute cosine of the angle
            cosTheta = dotProducts ./ (magsV .* magsW);

            % Ensure the cosine values are within the valid range [-1, 1]
            cosTheta = max(min(cosTheta, 1), -1);

            % Constants
            A = 500;
            B = 100;

            % Calculate the reward
            reward = (A .* cosTheta + B) ;

            % Return the total reward
            totalReward = reward;
        end


        function energyArray = predictEnergy(obj,distances)
            % Define the coefficients for each step
            aCoeffs = [0, 520.699, 203.742, 127.606, 86.3024, 71.8159, 63.1053, 57.9294, 50.7294, 46.399];
            bCoeffs = [0, 369.168, 869.01, 1119.61, 1459.52, 1368.97, 1279.89, 1128.54, 1144.29, 1183.82];

            % Calculate the energy using the linear function in a vectorized way
            energyArray = aCoeffs' .* distances + bCoeffs';
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
