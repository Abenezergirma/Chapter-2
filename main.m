% The main script that runs the whole Matlab code
clear all; clc
% experimentName = {'withoutEnergyReward', 'withEnergyReward'};
experimentName = 'Journal';

% Define the ranges for each weight factor
w_main_range = [1.0];
w_wind_range = [0.0, 0.5, 1];
w_energy_range = [0.0, 0.5, 1];
w_path_range = [0.0];
w_deviation_range = [0, 0.5, 1];%[0,0.1, 0.2, 0.3];

% Loop through all combinations of the weight factors
for wm = 1:length(w_main_range)
    for ww = 1:length(w_wind_range)
        for we = 1:length(w_energy_range)
            for wp = 1:length(w_path_range)
                for wd = 1:length(w_deviation_range)
                    % Create the experiment name based on the weight factors
                    fullName = strcat(experimentName, ...
                        '_main', num2str(w_main_range(wm)), ...
                        '_wind', num2str(w_wind_range(ww)), ...
                        '_energy', num2str(w_energy_range(we)), ...
                        '_path', num2str(w_path_range(wp)), ...
                        '_deviation', num2str(w_deviation_range(wd)));

                    % Call the trajectory planning function with the current weights
                    TrajectoryPlanning.planTrajectories(fullName, ...
                        w_main_range(wm), ...
                        w_wind_range(ww), ...
                        w_energy_range(we), ...
                        w_path_range(wp), ...
                        w_deviation_range(wd));

                    % Call the energy requirement function with error handling
                    try
                        % Only run the energy requirement experiment
                        EnergyRequirement.generateEnergyRequirement(fullName);
                    catch ME
                        % Handle the error gracefully and continue
                        fprintf('Error in EnergyRequirement.generateEnergyRequirement for %s: %s\n', fullName, ME.message);
                        cd('/home/abenezertaye/Desktop/Research/Codes/Chapter-2');
                        continue;  % Continue to the next set of experiments
                    end
                end
            end
        end
   end
end


%%
maxPower = 10e6;
rewardExpts = RewardFunctionExperiment.RewardFunctionExperiments(maxPower);
rewardExpts.resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults';
rewardExpts.experimentName = experimentName;
uav = 2; 

% Initialize a cell array to hold all loaded data and names
allData = {};
allNames = {};

% Loop through all combinations of the weight factors
for wm = 1:length(w_main_range)
    for ww = 1:length(w_wind_range)
        for we = 1:length(w_energy_range)
            for wp = 1:length(w_path_range)
                for wd = 1:length(w_deviation_range)
                    % Construct the full file name based on the weight factors
                    fullName = strcat(rewardExpts.experimentName, ...
                        '_main', num2str(w_main_range(wm)), ...
                        '_wind', num2str(w_wind_range(ww)), ...
                        '_energy', num2str(w_energy_range(we)), ...
                        '_path', num2str(w_path_range(wp)), ...
                        '_deviation', num2str(w_deviation_range(wd)), ...
                        'fullMissionBatteryParams.mat');

                    % Check if the file exists
                    if exist(fullfile(rewardExpts.resultsPath, fullName), 'file') ~= 2
                        % The file does not exist, skip to the next iteration
                        disp(['File ', fullName, ' does not exist, continuing...']);
                        continue;
                    else
                        % File exists
                        disp(['File ', fullName, ' already exists.']);
                    end

                    % Load the data for this configuration
                    loadedData = load(fullfile(rewardExpts.resultsPath, fullName));

                    % Store the loaded data and name in the cell arrays
                    allData{end+1} = loadedData;
                    allNames{end+1} = strcat(rewardExpts.experimentName, ...
                        'main', num2str(w_main_range(wm)), ...
                        'wind', num2str(w_wind_range(ww)), ...
                        'energy', num2str(w_energy_range(we)), ...
                        'path', num2str(w_path_range(wp)), ...
                        'deviation', num2str(w_deviation_range(wd)));

                end
            end
        end
    end
end

rewardExpts.plotPowerProfilesWithBar(allData, allNames, uav)












