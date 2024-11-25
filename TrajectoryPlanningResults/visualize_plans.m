experimentName = 'Frisco';
windRewardRate = [0,1];%linspace(10,10,1);
smoothnessRewardRate = [0,0.1];%[0,0.1,0.2,0.3];%linspace(0.1,0.1,1);
numDrones = 2;
resultsPath = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/TrajectoryPlanningResults';
for i = 1:length(windRewardRate)
    for k = 1:length(smoothnessRewardRate)
        fullName = strcat(experimentName,'smoothness',num2str(smoothnessRewardRate(k)),...
            'wind',num2str(windRewardRate(i)));
                % Create the full file path
        baseFileNameX = strcat(fullName,'XTrajectory');
        baseFileNameY = strcat(fullName,'YTrajectory');
        baseFileNameZ = strcat(fullName,'ZTrajectory');
        fileNameX = sprintf('%s_%02d.mat', baseFileNameX, numDrones);
        fileNameY = sprintf('%s_%02d.mat', baseFileNameY, numDrones);
        fileNameZ = sprintf('%s_%02d.mat', baseFileNameZ, numDrones);
        X = load(fullfile(resultsPath, fileNameX));
        Y = load(fullfile(resultsPath, fileNameY));
        Z = load(fullfile(resultsPath, fileNameZ)); 
        [X_smooth,Y_smooth,Z_smooth] = smoothenPlannedTrajectories(X.xTraj,Y.yTraj,Z.zTraj);
        plot3(X.xTraj',Y.yTraj',Z.zTraj','DisplayName','original')
        hold on
        plot3(X_smooth',Y_smooth',Z_smooth','DisplayName','smooth','Color','black')
        legend
        grid on


    end
end

Xs0w0 = xTraj; 
Ys0w0 = yTraj; 
Zs0w0 = zTraj; 

Xs0w1 = xTraj; 
Ys0w1 = yTraj; 
Zs0w1 = zTraj; 

Xs01w0 = xTraj; 
Ys01w0 = yTraj; 
Zs01w0 = zTraj; 

Xs01w1 = xTraj; 
Ys01w1 = yTraj; 
Zs01w1 = zTraj; 

% plot3(Xs0w0',Ys0w0',Zs0w0','DisplayName','s0w0')
% hold on
% plot3(Xs0w1',Ys0w1',Zs0w1','DisplayName','s0w1')
% plot3(Xs01w0',Ys01w0',Zs01w0','DisplayName','s01w0')
% plot3(Xs01w1',Ys01w1',Zs01w1','DisplayName','s01w1')
% legend
% grid on

function [X_smooth,Y_smooth,Z_smooth] = smoothenPlannedTrajectories(X,Y,Z)
% Parameters for Savitzky-Golay filter
polyOrder = 4; % Polynomial order
frameSize = 67; % Frame size for the filter (must be odd)

% Initialize smoothed trajectory matrices
X_smooth = zeros(size(X));
Y_smooth = zeros(size(Y));
Z_smooth = zeros(size(Z));

% Apply the filter to each UAV's trajectory
for i = 1:size(X,1) % Loop through each UAV
    X_smooth(i,:) = sgolayfilt(X(i,:), polyOrder, frameSize);
    Y_smooth(i,:) = sgolayfilt(Y(i,:), polyOrder, frameSize);
    Z_smooth(i,:) = sgolayfilt(Z(i,:), polyOrder, frameSize);
end

end
