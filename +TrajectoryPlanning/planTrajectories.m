function plotTrajectories(experimentName,energyRewardRate)
% clear; close all; clc;
% Future work - Write test cases for the trajectory planner

%% Initialization
resultsPath = 'TrajectoryPlanningResults';
totalAgents = 2;
scenarioSelector = struct('circle','circle', 'random','random', 'hexagon','hexagon');
totalNMACs = 0;
center_lon = (-96.94 + -96.7600)/2;
center_lat = (33.06 + 33.22)/2;
windDataPath = 'Wind Forecasts/smaller_area_wind_data.json';
planner = TrajectoryPlanning.Planner(scenarioSelector.hexagon, totalAgents, totalNMACs,...
    experimentName, windDataPath,center_lon,center_lat);
planner.energyRewardRate = energyRewardRate;
%% Generate Initial States and Goals
% [initialStates, goals] = planner.scenarioGenerator(totalAgents, 'hexagon');

%% Instantiate Ownship Class for Each Aircraft
% [assignedInitials, assignedGoals] = planner.packageDeliveryScenario;
% yawAngles = planner.initializeYaw(assignedInitials,assignedGoals);
% 
% initialStates = [assignedInitials(1:totalAgents,:),zeros(totalAgents,9)];
% initialStates(:,9) = yawAngles(1:totalAgents,:);

[initialStates, assignedGoals] = planner.scenarioGenerator(totalAgents, 'circle');
droneList = cell(1, totalAgents);
for i = 1:totalAgents
    droneList{i} = initializeDrone(i, initialStates(i,:), assignedGoals(i,1:3));
end

%% Main Simulation Loop
teamActions = TrajectoryPlanning.Ownship.T18Actions;
numLeft = size(droneList, 2);
stepTimer = [];
j = 0;

while numLeft > 0
    tic  % Start timer

    % droneList = updateDrones(droneList, planner, teamActions);
    [droneList,numLeft] = updateDrones(droneList, planner, teamActions,numLeft);

    % Monitor the status of the game
    [terminal, NMACs, droneList] = planner.terminalDetection(droneList,j);
    totalNMACs = totalNMACs + NMACs;
    stepTimer = [stepTimer, toc];  % Record elapsed time
    j = j + 1;

    % Display Status
    displayStatus(j, toc, totalNMACs, numLeft);

    % Check for Terminal Condition
    if terminal
        break;  % Exit the loop
    end
end

%% Analyze and Save Results
analyzeAndSaveResults(stepTimer, totalNMACs,droneList, experimentName);

%% Supporting Functions
    function drone = initializeDrone(id, initState, goal)
        drone = TrajectoryPlanning.Ownship(windDataPath,center_lon,center_lat);
        drone.aircraftID = id;
        drone.goal = goal;
        drone = updateAircraftStates(drone, initState);
    end

    function [droneList,numLeft] = updateDrones(droneList, planner, teamActions,numLeft)
        % numLeft = size(droneList, 2);
        for k = 1:numel(droneList)
            ownship = droneList{k};
            if ~ownship.dead
                [positivePeaks, negativePeaks, droneList] = planner.buildPeaks(ownship, droneList, teamActions);
                ownship = droneList{k};  % Update ownship in case it was modified in buildPeaks
                [futureStates, oneStepStates, futureActions] = planner.neighboringStates(ownship, teamActions);
                totalValues = planner.valueFunction(ownship, futureStates, positivePeaks, negativePeaks);
                ownship = selectBestAction(ownship, totalValues, futureActions, futureStates, oneStepStates);
                ownship = updateAircraftStates(ownship, ownship.nextStates);
                ownship.Traces = 0;
                if norm(ownship.currentStates(1:3) - ownship.goal) < 20
                    ownship.dead = true;
                    disp(['ownship ', num2str(ownship.aircraftID), ' made it to goal, removed']);
                    numLeft = numLeft - 1;
                end
                droneList{k} = ownship;
            end
        end
    end

    function displayStatus(step, compTime, totalNMACs, numLeft)
        disp(['step ', num2str(step), ', computation time is ', num2str(compTime), ' sec']);
        fprintf('NMACs so far, %d\n', totalNMACs);
        fprintf('Num aircraft left, %d\n', numLeft);
    end

    function analyzeAndSaveResults(stepTimer, totalNMACs, droneList, experimentName)
        resultsPath = 'TrajectoryPlanningResults';
        allNMAC = totalNMACs;
        Mean = mean(stepTimer);
        Median = median(stepTimer);
        Std = std(stepTimer);
        Throughput = sum(stepTimer);

        numDrones = length(droneList);

        % Extract paths and compute maximum length
        cellstore = cellfun(@(x) x.traveledPath(:,1:3), droneList, 'UniformOutput', false);
        maxlength = max(cellfun(@length, cellstore));

        % Preallocate arrays
        xTraj = zeros(numDrones, maxlength);
        yTraj = zeros(numDrones, maxlength);
        zTraj = zeros(numDrones, maxlength);
        bestTrajectory = zeros(numDrones, 10, 3, maxlength);

        % Populate trajectories
        for i = 1:numDrones
            len = length(droneList{i}.traveledPath(:,1));
            xTraj(i,1:len) = droneList{i}.traveledPath(:,1);
            yTraj(i,1:len) = droneList{i}.traveledPath(:,2);
            zTraj(i,1:len) = droneList{i}.traveledPath(:,3);

            % Padding with the last element
            xTraj(i,xTraj(i,:)==0) = droneList{i}.traveledPath(end,1);
            yTraj(i,yTraj(i,:)==0) = droneList{i}.traveledPath(end,2);
            zTraj(i,zTraj(i,:)==0) = droneList{i}.traveledPath(end,3);

            % Organizing best trajectories
            bestTraject = droneList{i}.bestTrajectory;
            bestTrajectory(i,:,:,1:size(bestTraject,1)/10) = permute(reshape(bestTraject, 10, size(bestTraject,1)/10, 3), [1 3 2]);
        end
        % Create the full file path
        baseFileNameX = strcat(experimentName,'XTrajectory');
        baseFileNameY = strcat(experimentName,'YTrajectory');
        baseFileNameZ = strcat(experimentName,'ZTrajectory');
        fileNameX = sprintf('%s_%02d.mat', baseFileNameX, numDrones);
        fileNameY = sprintf('%s_%02d.mat', baseFileNameY, numDrones);
        fileNameZ = sprintf('%s_%02d.mat', baseFileNameZ, numDrones);
        filePathX = fullfile(resultsPath, fileNameX);
        filePathY = fullfile(resultsPath, fileNameY);
        filePathZ = fullfile(resultsPath, fileNameZ);

        % Save the array
        save(filePathX, 'xTraj');
        save(filePathY, 'yTraj');
        save(filePathZ, 'zTraj');
    end
end