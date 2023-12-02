% animate the trajectories 
clear all; clc
% import trajectory planner results
load('packageEnergyRewardrewardRate0.3XTrajectory_06.mat') 
load('packageEnergyRewardrewardRate0.3YTrajectory_06.mat')
load('packageEnergyRewardrewardRate0.3ZTrajectory_06.mat')
% import T18 results 
load('/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults/packageEnergyRewardrewardRate0fullMissionBatteryParams.mat')



% Create the array
refCoordinate =[32.863498, -96.803123, 40]; %DFW

[xTrajActual, yTrajActual, zTrajActual] = extractActualTrajectories(results);
[latTraj, lonTraj, altTraj] = convertTrajectories(xTrajActual', yTrajActual', zTrajActual', refCoordinate);

% for uav = 1:6
% plot3(latTraj(uav,:), lonTraj(uav,:), altTraj(uav,:))
% % [latTraj, lonTraj, altTraj] = convertTrajectories(xTraj, yTraj, zTraj, refCoordinate);
% hold on
% end

for uavNumber = 1:6
    createKMLFile(latTraj(uavNumber, :), lonTraj(uavNumber, :), altTraj(uavNumber, :)*0.04, uavNumber);
end

% centerCoord = [38.910944, -77.042726, 40]; % %Washington DC
centerCoord = [32.863498, -96.803123, 40]; %DFW 
[cornersLat, cornersLon] = createRectangleKML(centerCoord);







