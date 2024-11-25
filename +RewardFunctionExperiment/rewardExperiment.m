% This script is used to run reward function related experiments
clear all; clc; close all;
projectPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2';
maxPower = 10;
rewardExpts = RewardFunctionExperiment.RewardFunctionExperiments(maxPower);
% timestep = rewardExpts.timeStep;
simulationTime = 10;
% contorlActions = rewardExpts.generateControlCombinations;
% trajectories = rewardExpts.runLowFidelityModel(contorlActions, simulationTime);
% setWaypoints = rewardExpts.smoothTrajectoryVectorized(trajectories(:,:,1:3),6);

% define all file and path related properties 
rewardExpts.highFidelityModelPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/+EnergyRequirement/UAVSimulinkModel';
rewardExpts.highFidelityModelRunner = 'runDetailedT18Model';
rewardExpts.resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';
rewardExpts.experimentName = 'rewardExperimentBatteryParams.mat';
rewardExpts.BatteryParams = fullfile(rewardExpts.resultsPath,rewardExpts.experimentName);
rewardExpts.energyRequirement = 'reqEnergy.mat';

% sampleWaypoints = setWaypoints;%(:,1:20:10000,:);
trajID = 1:2:100;

load("futureTraj.mat")
% rewardExpts.plotAllTrajectoryAndWaypoints(trajectories(:,:,1:3),setWaypoints(:,:,:))
% 
% 
% plot3(trajectories(:,trajID,1),trajectories(:,trajID,2),trajectories(:,trajID,3) )
% hold on
% plot3(setWaypoints(:,trajID,1),setWaypoints(:,trajID,2),setWaypoints(:,trajID,3) )
if false % this is intentional because it takes sometime to run
    rewardExpts.runHighFidelityModel(sampleWaypoints);
end
numWaypoints = 6;

cd(projectPath);
rewardExpts.getPowerConsumption;
load(fullfile(rewardExpts.resultsPath,rewardExpts.energyRequirement));
fittedFunctions = rewardExpts.createFittedEnergyFunctions(reqEnergy);

experimentNumbers = 1:1:10000;
timeSteps = 1:1:10;
% Get the fitted values
% interpolatedEnergyReqs = reqEnergy;
interpolatedEnergyReqs = rewardExpts.evaluateFittedFunctions(fittedFunctions, experimentNumbers, timeSteps);
rewardExpts.plotDataPoints(reqEnergy, interpolatedEnergyReqs, experimentNumbers, timeSteps)

save("RewardFunctionExperimentResults/interpolatedEnergyReqs.mat",'interpolatedEnergyReqs');


% rewardExperimentWaypoints = rewardExpts.convertTrajectoriesToWaypoints(trajectories(:,:,1:3), numWaypoints);
% NewrewardExperimentWaypoints = rewardExpts.NewconvertTrajectoriesToWaypoints(trajectories(:,:,1:3), numWaypoints);
% waypointPath = fullfile('UAV model v3.4.1','rewardExperimentWaypoints.mat');
% save(waypointPath,'rewardExperimentWaypoints')
% traj = 1;
% rewardExpts.plotTrajectoryAndWaypoints(squeeze(trajectories(:,traj,1:3)), squeeze(rewardExperimentWaypoints(:,traj,1:3)));
% hold on 
% rewardExpts.plotTrajectoryAndWaypoints(squeeze(trajectories(:,traj,1:3)), squeeze(NewrewardExperimentWaypoints(:,traj,1:3)));
% rewardExpts.plotTrajectoryAndWaypoints(squeeze(trajectories(:,traj,1:3)), squeeze(sampleWaypoints(:,traj,1:3)));






