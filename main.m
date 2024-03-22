% The main script that runs the whole Matlab code
clear all; clc
% experimentName = {'withoutEnergyReward', 'withEnergyReward'};
experimentName = 'Frisco';
energyRewardRate = [0,0.3];% [0,0.2,0.4,0.6,0.8,1];
windRewardRate = [0,1];%linspace(10,10,1);
smoothnessRewardRate = [0,0.1];%[0,0.1,0.2,0.3];%linspace(0.1,0.1,1);

for i = 1:length(windRewardRate)
    for k = 1:length(smoothnessRewardRate)
        fullName = strcat(experimentName,'smoothness',num2str(smoothnessRewardRate(k)),...
            'wind',num2str(windRewardRate(i)));
        TrajectoryPlanning.planTrajectories(fullName,smoothnessRewardRate(k),windRewardRate(i))
        EnergyRequirement.generateEnergyRequirement(fullName)
    end
end

maxPower = 10e6;
rewardExpts = RewardFunctionExperiment.RewardFunctionExperiments(maxPower);
rewardExpts.resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';
rewardExpts.experimentName = experimentName;
uav = 1;

% Initialize a cell array to hold all loaded data
allData = {};
allNames = {};
for i = 1:length(windRewardRate)
    for k = 1:length(smoothnessRewardRate)
        % if (i==2 && k==2)%(i==1 && k==2) %|| (i==2 && k==1)
        %     continue
        % end

        fullName = strcat(rewardExpts.experimentName,'smoothness',num2str(smoothnessRewardRate(k)), ...
            'wind',num2str(windRewardRate(i)), 'fullMissionBatteryParams.mat');

        % Check if the file exists. The function 'exist' returns 2 if the file exists.
        if exist(fullfile(rewardExpts.resultsPath,fullName), 'file') ~= 2
            % The file does not exist, so you can continue with your operation here
            disp(['File ', fullName, ' does not exist, continuing...']);
            continue

            % Your code to execute when the file does not exist goes here
        else
            % File exists
            disp(['File ', fullName, ' already exists.']);
        end
        % Load the data for this configuration
        loadedData = load(fullfile(rewardExpts.resultsPath,fullName));
        % Store the loaded data in the cell array
        allData{end+1} = loadedData;
        allNames{end+1} = strcat(rewardExpts.experimentName,'smoothness',num2str(smoothnessRewardRate(k)), ...
            'wind',num2str(windRewardRate(i)));

    end
end

% Now pass all the loaded data for plotting
% plotPowerProfiles(rewardExpts, allData, allNames, uav)

plotPowerProfilesWithBar(rewardExpts, allData, allNames, uav)






left_up = [0.579, -1.690058];
right_down = [0.577,-1.69];
left_corner = [ 33.17, -96.83];
right_corner = [33.05, -96.82];












