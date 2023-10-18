% The main script that runs the whole Matlab code
clear all; clc
experimentName = {'withoutEnergyReward', 'withEnergyRewardNew'};
energyRewardRate = [0,1];
for k = 2:length(experimentName)
TrajectoryPlanning.planTrajectories(experimentName{k},energyRewardRate(k))
EnergyRequirement.generateEnergyRequirement(experimentName{k})
end

resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter 2/EnergyRequirementResults';
withoutEnergyReward = load(fullfile(resultsPath,'withoutEnergyRewardfullMissionBatteryParams.mat'));
withEnergyReward = load(fullfile(resultsPath,'withEnergyRewardfullMissionBatteryParams.mat'));


withoutEnergyReward = withoutEnergyReward.results;
withEnergyReward = withEnergyReward.results;
maxPower = 10e6;

uav = 1;
rewardExpts = RewardFunctionExperiment.RewardFunctionExperiments(maxPower);
rewardExpts.resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter 2/EnergyRequirementResults';
rewardExpts.experimentName = 'rewardExperimentBatteryParams.mat';
rewardExpts.plotPowerProfiles(withoutEnergyReward,withEnergyReward,uav)


