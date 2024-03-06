classdef DryVR < handle 

    properties (Access = public)
        L = 0.9;  % Lift acceleration
        g = 9.81; % gravity
        Tf = 3; % Reachset time horizon

        timestep = 0.1; % time step for simulating the traces
        numSteps = 500; % N for simulating the traces using euler method
        numTraces = 20; % number of traces that needs to be generated to get the reachset

        DEG2RAD=0.0174533; %degree to rad coverting constant
        RAD2DEG = 57.29577951308232; %rad to degree converting constant
        MACH = 343; %Speed of sound

        % Aircraft performance parameters
        limits;
        actions;
        vaild_actions;
        fixedWingActions;

        % parameters for discrepancy calculation
        EPSILON = 1.0e-100;
        trueMIN = -10;
    end

    methods(Static)
        function dydt = FixedWing_vectorized(t,y,actions)
            % This method implements a 6-DOF guidance model of fixed wing
            % aircraft. The model is adopted from "Beard, R. W., and McLain, T. W., 
            % Small unmanned aircraft: Theory and practice, Princeton University Press, 2012.
            % https://doi.org/10.1515/9781400840601

            % The method is implemented so that it can take a vectorized
            % control action 

            % autopilot related parameters    
            b_gamma = 0.5;
            b_Va = 0.5;
            b_phi = 0.5;
            g = 9.81;

            % extract control actions 
            n = length(actions(:,1));
            y = reshape(y, n, []);
            [gamma_c, phi_c, Va_c] = deal(actions(:,1), actions(:,2), actions(:,3));
            
            % extract the initial states
            [~,~,~, chi, gamma, phi, Va] = deal(y(:,1), y(:,2), y(:,3), y(:,4), y(:,5), y(:,6), y(:,7));
            
            Vg = Va; % assuming no wind

            gamma_a = asin(Vg .*sin(gamma) ./Va);

            psi = chi;

            northRate = Va .* cos(psi) .* cos(gamma_a);

            eastRate = Va .* sin(psi) .* cos(gamma_a);

            heightRate = Va .* sin(gamma_a);

            chiRate = (g ./Vg) .* tan(phi) .* cos(chi - psi);

            gammaRate = b_gamma .* (gamma_c - gamma);

            VaRate = b_Va .* (Va_c - Va);

            phiRate = b_phi .* (phi_c - phi);

            dydt = [northRate; eastRate; heightRate; chiRate; gammaRate; phiRate; VaRate];
        end
    
    end


    methods 
        function obj = DryVR(timestep,numSteps)
            % class constructor 
            obj.timestep = timestep;
            obj.numSteps = numSteps;
        end

        function fixedWingActions = get.fixedWingActions(obj)
            % Builds an array that stores the action space of each team
            % Fixedwing --

            gamma_c = 2*linspace(-pi/9,pi/9,10);
            phi_c = gamma_c/2; 
            Va_c = 25:5:70; 

            % construct the joint action space that comprises the three action spaces
            fixedWingActions = {gamma_c, phi_c, Va_c};
            actionsD = fixedWingActions;
            [actionsD{:}] = ndgrid(fixedWingActions{:});
            teamActions = cell2mat(cellfun(@(m)m(:),actionsD,'uni',0));
            fixedWingActions = teamActions;
        end

        function futureTraj = updated_fixedWing_discrete(obj, currentState, actions, timestep, numSteps)
            % A method that implements the same dynamics but returns a set of next states with state constraints
            % applied

            %Note: this function is vectorzed to accept an array of control actions
            b_gamma = 0.5;
            b_Va = 0.5;
            b_phi = 0.5;
            g = 9.81;
            
            [north, east, height, chi, gamma, phi, Va] = deal(currentState(:,1),currentState(:,2),...
                currentState(:,3),currentState(:,4),currentState(:,5),currentState(:,6),currentState(:,7));

            [gamma_c, phi_c, Va_c] = deal(actions(:,1), actions(:,2), actions(:,3));

            futureTraj = zeros(length(actions(:,1)), numSteps, length(currentState(1,:))+1); %Preallocated memory for speed

            for i = 1:numSteps

                % get the parameters assuming no wind
                Vg = Va; % assuming no wind
                psi = chi; % assuming no wind

                % updating gamma air
                gamma_a = gamma;%asin(Vg .*sin(gamma) ./Va);

                % updating the airspeed
                Va_Dot = b_Va * (Va_c - Va);
                Va = Va + Va_Dot * timestep;

                % updating gamma
                gamma_Dot = b_gamma .* (gamma_c - gamma);
                gamma = gamma + gamma_Dot * timestep;

                % updating phi
                phi_Dot = b_phi .* (phi_c - phi);
                phi = phi + phi_Dot * timestep;

                % updating chi
                chi_Dot = (g ./Vg) .* tan(phi) .* cos(chi - psi);
                chi = chi + chi_Dot * timestep;

                % updating the velocities
                northRate = Va .* cos(psi) .* cos(gamma_a);
                eastRate = Va .* sin(psi) .* cos(gamma_a);
                heightRate = Va .* sin(gamma_a);

                % updating the positions
                north = north + northRate * timestep;
                east = east + eastRate * timestep;
                height = height + heightRate * timestep;

                nextState = [repmat(timestep*i,length(actions(:,1)),1), north, east, height, chi, gamma, phi, Va];
                futureTraj(:,i,:) = nextState;
            end


        end


        function Traces = GenerateTraces(obj, varargin)
            % A methods that returns a set of simulation traces, given the type of dynamic model
            % NOTE: it can accept different models but the default is
            % low fidelity fixed wing dynamics

            % Set defaults
            vehicle = 'fixedwing_vector';

            % Process optional inputs
            arg = varargin;
            if ischar(arg{1})
                vehicle = arg{1};
            end

            switch lower(vehicle)

                case{'random_action'}
                    currentState = arg{2};
                    controlActions = obj.fixedWingActions([floor(linspace(1,1000,obj.numTraces))],:);
                    futureTraj = updated_fixedWing_discrete(obj, currentState, controlActions, obj.timestep, obj.numSteps);
                    Traces = futureTraj;
                case {'fixedwing_vector'}
                    currentState = arg{2};
                    controlActions = obj.fixedWingActions;
                    vectorizedStates = currentState.*ones(length(controlActions),1);
                    [t1, traces] = ode45(@(t,y) DryVR.FixedWing_vectorized(t,y,controlActions),0:0.01:10,vectorizedStates);
                    Traces = permute(reshape(traces, length(t1), length(controlActions), length(currentState)), [1, 2, 3]);
                case {'fixedwing'}
                    %initial states
                    north = 10;
                    east = 10;
                    height = 10;
                    chi = 0.3;
                    gamma = 0.2;
                    phi = 0.2;
                    Va = 15;

                    initialUpper = [north+5, east+5, height+5, chi, gamma, phi, Va];
                    initialLower = [north-5, east-5, height-5, chi, gamma, phi, Va];
                    initialCenter = (initialUpper + initialLower)/2;
                    %                     timeStamps = 0:0.01:obj.Tf;
                    [t, centerTrace] = ode45(@DryVR.FixedWing, 0:0.01:5 ,initialCenter);

                    traceLength = length(centerTrace(:,1));
                    numStates = length(centerTrace(1,:));
                    Traces = zeros(obj.numTraces,traceLength, numStates + 1); % +1 is for the time axis
                    Traces(1,:,:) = [t, centerTrace];

                    for i = 2:obj.numTraces
                        initialNew = (initialUpper - initialLower).*rand(1,7) + initialLower; %Randomly generate initial set within the bound
                        [t,iTrace] = ode45(@DryVR.FixedWing, 0:0.01:5 ,initialNew);
                        Traces(i,:,:) = [t, iTrace];
                    end

                otherwise
                    error('Unrecognized vehicle type %s', vehicle)
            end
        end


        function initialRadii = computeInitialRadii(obj,Traces)
            % compute initial radii, which is the max difference between initial states among all traces
            numStates = length(Traces(1,1,:));

            initialRadii = zeros(1,numStates - 1);
            for i = 1:obj.numTraces % Find the max difference between initial states among all traces
                trace = Traces(i,:,:);
                for j = 2:(numStates)
                    initialRadii(j - 1) = max(initialRadii(j - 1), abs(Traces(1,1,j) - trace(1,1,j)));
                end
            end
        end

        function [chebDistance] = chebyshevDistance(obj, arrayA)
            % A method that returns chebyshev distance between points given
            % in array A
            arrayB = arrayA;
            [na,aDims] = size(arrayA);
            [nb,bDims] = size(arrayB);
            [j,i] = meshgrid(1:nb,1:na);
            delta = arrayA(i,:) - arrayB(j,:);
            cheby = zeros(na,nb);
            cheby(:) = max(abs(delta),[],2);
            chebDistance = cheby(j>i);
        end


        function muPoints = computeMuPoints(obj, Traces, initialRadii)
            % A method that returns the sensitivities for each state
            % trajectory

            numStates = length(Traces(1,1,:));
            traceLength = length(Traces(1,:,1));

            normalizingInitialRadii = repmat(initialRadii,1);
            muPoints = zeros(length(normalizingInitialRadii), traceLength-1); % exclude the initial point
            normalizingInitialRadii(find(normalizingInitialRadii==0)) = 1;

            normalizingInitialPoints = permute(Traces(:,1,2:end),[1 3 2]) ./normalizingInitialRadii;
            
            % Find the max distance between initial states using Chebyshev distance
            initialDistances = chebyshevDistance(obj, normalizingInitialPoints);

            for stateIndex = 2:numStates
                % Find the max distance between trajectories at each point and divide it by the max distance between initial set (aka sensitivity of trajectories)
                for timeIndex = 2:traceLength %mu_points are y_points

                    stateSensitivity =  ( chebyshevDistance(obj, reshape(Traces(:,timeIndex,stateIndex), ...
                        [obj.numTraces, 1]))./normalizingInitialRadii(stateIndex - 1))./initialDistances;
                    muPoints(stateIndex - 1, timeIndex - 1) = max(stateSensitivity(~isinf(stateSensitivity)));

                end
            end

        end


        function allStateLinearSepartors = computeDiscrepancyParameters(obj, Traces, initialRadii)
            % A method that returns the discrepancy parameters that will be
            % used to define the linear separators

            numStates = length(Traces(1,1,:));
            traceLength = length(Traces(1,:,1));
            muPoints = obj.computeMuPoints(Traces, initialRadii);

            traceInitialTime = Traces(1,1,1);
            nuPoints = Traces(1,:,1) - traceInitialTime;
            points = zeros(numStates - 1, traceLength, 2);
            points(find(initialRadii ~= 0), 1, 2) = 1.0;
            points(:,:,1) = repmat(reshape(nuPoints, [1, length(nuPoints)])', [1,numStates-1])';
            points(:,2:end,2) = muPoints;



            points(:,:,2) = max(points(:,:,2), obj.EPSILON);
            points(:,:,2) = log(points(:,:,2)); % The log here is used to fit exponentials

            for stateIndex = 2:numStates
                newMin = min(min(points(stateIndex -1, 2:end, 2)) + obj.trueMIN, -10);
                if initialRadii(stateIndex - 1) == 0
                    newPoints = vertcat([points(stateIndex - 1,2,1), ...
                        newMin], [points(stateIndex - 1,end,1), newMin]);
                else
                    newPoints = vertcat(permute( points(stateIndex - 1, 1, :), [3,2,1])' ...
                        ,[points(stateIndex - 1,1,1), newMin] , ...
                        [points(stateIndex - 1,end,1), newMin]);

                end
                statePoints = [permute( points(stateIndex - 1,2:end,:), [2,3,1]); newPoints];
                statePointsHull = sortrows(convhulln(statePoints));
                linearSeparators = [];
                vertInds = statePointsHull;

                for k = 1:length(vertInds)
                    endInd = vertInds(k,1);
                    startInd = vertInds(k,2);

                    if statePoints(startInd, 2) ~= newMin &&  statePoints(endInd, 2 ) ~= newMin
                        slope = (statePoints(endInd, 2) - statePoints(startInd, 2)) / ...
                            (statePoints(endInd, 1) - statePoints(startInd, 1));

                        yIntercept = statePoints(startInd, 2) - statePoints(startInd, 1)*slope;

                        startTime = statePoints(startInd, 1);
                        endTime = statePoints(endInd, 1);

                        assert  (startTime < endTime)


                        if startTime == 0
                            linearSeparators = [linearSeparators; [startTime, endTime, slope, yIntercept, 0, endInd + 1 ]];
                        else
                            linearSeparators = [linearSeparators; [startTime, endTime, slope, yIntercept, startInd + 1, endInd + 1]];
                        end

                    end
                end
                linearSeparators = sortrows(linearSeparators);
                allStateLinearSepartors{:,:,stateIndex-1} =  linearSeparators;

            end

        end

        function centerTrace = computeCenterTrace(obj, Traces)
            center_point = [];
            for stateIndex = 2:4 %x,y,z states only
                [lower_point,lower_index] = min(Traces(:,end,stateIndex));
                [higher_point,higher_index] = max(Traces(:,end,stateIndex));

                center_point_i = (lower_point+higher_point)*0.5;
                center_point = [center_point,center_point_i];
            end
            points = squeeze(Traces(:,end,2:4));
            [index, distance] = dsearchn(points,center_point);
            centerTrace = permute(Traces(index,:,:), [2,3,1]);
        end

        function reachTube = getReachtube(obj, Traces, initialRadii, discrepancyParameters)
            % A method that returns the reach tube from the computed
            % discrepancy parameters

            centerTrace = computeCenterTrace(obj,Traces); 

            numStates = length(centerTrace(1,:));
            traceLength = length(centerTrace(:,1));

            normalizingInitialradii = repmat(initialRadii,1);

            normalizingInitialradii(find(normalizingInitialradii==0)) = 1;
            df = zeros(traceLength, numStates);
            allStateLinearSepartors = discrepancyParameters;

            for stateIndex = 2:numStates
%                 [lower_point,lower_index] = min(Traces(:,end,stateIndex));
%                 [higher_point,higher_index] = max(Traces(:,end,stateIndex));
% 
%                 center_point = (lower_point+higher_point)*0.5;
%                 [minValue,closestIndex] = min(abs(Traces(:,end,stateIndex) - center_point));
% 
%                 centerTrace = Traces(closestIndex,:,stateIndex); 
%                 centerTrace = firstTrace; to go back to the previous
%                 state uncomment this line and comment all the above

                prevVal = 0;
                if initialRadii(stateIndex - 1) == 0
                    prevInd = 1;
                else
                    prevInd = 0;
                end
                linearSeparators = cell2mat(allStateLinearSepartors(:,:,stateIndex - 1));

                if initialRadii(stateIndex - 1) ~= 0
                    df(1, stateIndex) = initialRadii(stateIndex - 1);
                end

                for linSep = 1:length(linearSeparators(:,1))
                    linearSeparator = linearSeparators(linSep,:);
                    [~, ~, slope, yIntercept, startInd, endInd] = ...
                        deal(linearSeparator(1), linearSeparator(2), linearSeparator(3), linearSeparator(4), linearSeparator(5), linearSeparator(6));

                    segment_t = centerTrace(startInd+1:endInd, 1);
                    segment_df = normalizingInitialradii(stateIndex - 1) * exp(yIntercept + slope*segment_t);
                    segment_df(1) = max(segment_df(1), prevVal);
                    df(startInd+1:endInd, stateIndex) = segment_df;
                    prevVal = segment_df(end);
                    prevInd = endInd;

                end
            end

            reachTube = zeros(traceLength - 1, 2, numStates);
            reachTube(:,1,:) = min(centerTrace(2:end,:) - df(2:end,:), centerTrace(1:end-1,:) - df(1:end-1,:));
            reachTube(:,2,:) = max(centerTrace(2:end,:) + df(2:end,:), centerTrace(1:end-1,:) + df(1:end-1,:));

        end

    end

end


