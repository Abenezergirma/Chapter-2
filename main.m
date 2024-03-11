% The main script that runs the whole Matlab code
clear all; clc
% experimentName = {'withoutEnergyReward', 'withEnergyReward'};
experimentName = 'packageEnergyReward';
energyRewardRate = [0,0.3];% [0,0.1,0.2,0.3,0.4,0.5];

for k = 1:length(energyRewardRate)
    fullName = strcat(experimentName,'rewardRate',num2str(energyRewardRate(k)));
    % TrajectoryPlanning.planTrajectories(fullName,energyRewardRate(k))
    EnergyRequirement.generateEnergyRequirement(fullName)
end

resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';
withoutEnergyReward = load(fullfile(resultsPath,'packageEnergyRewardrewardRate0fullMissionBatteryParams.mat'));
withEnergyReward = load(fullfile(resultsPath,'packageEnergyRewardrewardRate0.3fullMissionBatteryParams.mat'));

withoutEnergyReward = withoutEnergyReward.results;
withEnergyReward = withEnergyReward.results;
maxPower = 10e6;

for uav = 1:2
    rewardExpts = RewardFunctionExperiment.RewardFunctionExperiments(maxPower);
    rewardExpts.resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';
    rewardExpts.experimentName = 'rewardExperimentBatteryParams.mat';
    rewardExpts.plotPowerProfiles(withoutEnergyReward,withEnergyReward,uav)
end

% waypoints = rewardExpts.NewconvertTrajectoriesToWaypoints(futureTraj(:,1:5:end,1:3), 10);
% rewardExpts.plotAllTrajectoryAndWaypoints(futureTraj(:,1:5:end,1:3),waypoints(:,:,:))


% UAV 1: Initial Position = [1000.000000, 0.000000, 1000.000000], Goal Position = [-1000.000000, 0.000000, 1000.000000]
% UAV 2: Initial Position = [-1000.000000, 0.000000, 1000.000000], Goal Position = [0.000000, 0.000000, 1000.000000]
% UAV 3: Initial Position = [0.000000, 0.000000, 1000.000000], Goal Position = [-500.000000, -866.025404, 1000.000000]
% UAV 4: Initial Position = [-500.000000, -866.025404, 1000.000000], Goal Position = [-500.000000, 866.025404, 1000.000000]
% UAV 5: Initial Position = [-500.000000, 866.025404, 1000.000000], Goal Position = [500.000000, -866.025404, 1000.000000]
% UAV 6: Initial Position = [500.000000, -866.025404, 1000.000000], Goal Position = [500.000000, 866.025404, 1000.000000]
% UAV 7: Initial Position = [500.000000, 866.025404, 1000.000000], Goal Position = [1000.000000, 0.000000, 1000.000000]



