function generateEnergyReqiurement(experimentName)
% This script is used to generate battery related parameters (such as voltage, current) for the full
% flight mission
% clear all; clc
projectPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2';

%% Load Trajectory Data
% Load x, y, and z trajectories from the specified files
trajectoriesPath = 'TrajectoryPlanningResults';
numDrones = 4;
% Create the full file path
% Create the full file path
baseFileNameX = strcat(experimentName,'XTrajectory');
baseFileNameY = strcat(experimentName,'YTrajectory');
baseFileNameZ = strcat(experimentName,'ZTrajectory');
fileNameX = sprintf('%s_%02d.mat', baseFileNameX, numDrones);
fileNameY = sprintf('%s_%02d.mat', baseFileNameY, numDrones);
fileNameZ = sprintf('%s_%02d.mat', baseFileNameZ, numDrones);
filePathX = fullfile(trajectoriesPath, fileNameX);
filePathY = fullfile(trajectoriesPath, fileNameY);
filePathZ = fullfile(trajectoriesPath, fileNameZ);
load(filePathX)
load(filePathY)
load(filePathZ)



%% Define File and Path Related Properties
% Define all file and path-related properties in a structured way
% Define model path
modelPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/+EnergyRequirement/UAVSimulinkModel';

% Create an instance of SimulinkUAVWrapper and set properties
T18Wrapper = EnergyRequirement.SimulinkUAVWrapper(modelPath);
T18Wrapper.modelScript = 'runDetailedT18Model';
T18Wrapper.experimentName = strcat(experimentName,'fullMissionBatteryParams.mat');
T18Wrapper.resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';

% Construct the full path for battery parameters
T18Wrapper.BatteryParams = fullfile(T18Wrapper.resultsPath,T18Wrapper.experimentName);

combineTrajectories = true;
if combineTrajectories
    % [xTraj, yTraj, zTraj] = T18Wrapper.preprocessTrajectories(xTraj, yTraj, zTraj);
    numWaypoints = 40;
    [xTraj, yTraj, zTraj] = T18Wrapper.sampleAndCombineTrajectories(xTraj, yTraj, zTraj, numWaypoints);
    wayPoints = cat(3, xTraj', yTraj', zTraj');
else
    numWaypoints = 32;
    % Concatenate trajectories and smooth them
    lambda = 0.3;
    trajectories = cat(3, xTraj', yTraj', zTraj');
    smoothness = 80;
    plotTrajectories = true;
    wayPoints = T18Wrapper.smoothAndSampleTrajectory(trajectories, smoothness, lambda, numWaypoints, plotTrajectories);
end


%%
% plotTrajectoriesAndWaypoints(T18Wrapper, trajectories1, wayPoints1, trajectories2, wayPoints2)
%% Visualization
% Specify the UAV to visualize and plot its trajectory and waypoints
% for uav =1:numDrones
% T18Wrapper.plotTrajectoryAndWaypoints([xTraj(uav,:)', yTraj(uav,:)', zTraj(uav,:)'], wayPoints(:,uav,:))
% end
%% Model Execution
% Run the Simulink model
if true
T18Wrapper.runModel(wayPoints);
end

%% Load and Visualize Results
% Load the results from the executed model and generate a plot for battery parameters
cd(projectPath);

load(T18Wrapper.BatteryParams)
% T18Wrapper.generateBatteryParametersPlot(results,uav)
end
