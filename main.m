% The main script that runs the whole Matlab code
clear all; clc
experimentName = {'withoutEnergyReward', 'withEnergyReward'};
energyRewardRate = [0,1];
for k = 2:length(experimentName)
TrajectoryPlanning.planTrajectories(experimentName{k})
EnergyRequirement.generateEnergyRequirement(experimentName{k})
end

resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';
withoutEnergyReward = load(fullfile(resultsPath,'withoutEnergyRewardfullMissionBatteryParams.mat'));
withEnergyReward = load(fullfile(resultsPath,'withEnergyRewardfullMissionBatteryParams.mat'));


withoutEnergyReward = withoutEnergyReward.results;
withEnergyReward = withEnergyReward.results;
maxPower = 10e6;

uav = 6;
rewardExpts = RewardFunctionExperiment.RewardFunctionExperiments(maxPower);
rewardExpts.resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';
rewardExpts.experimentName = 'rewardExperimentBatteryParams.mat';
rewardExpts.plotPowerProfiles(withoutEnergyReward,withEnergyReward,uav)

% rew = 251317+252544+242687+256604+239865+241315
% 
% norew = 246540+262906+234203+261189.5+257797+235974
