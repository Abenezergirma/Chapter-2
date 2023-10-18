% This script simulates the aircraft with fading past trajectories 
% Future work: covert this script to class 
clear; close all; clc;
load('T18_5.mat');

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

% Define fading parameters
N = 1500;  % Number of steps to show fading effect
fadeColor = [0 0 1 1]; % Initialize color [R G B Alpha]
alphaStep = 1/N; % Alpha reduction per step


% Plotting
figure('units','pixels','position',[0 0 1920 1080]);
% figure('units','inches','position',[0 0 8 6]); %uncomment this line if
% you want to simulate in default matlab figure size

hold on;
for vehicle = 1:numDrones
    aircraftHandle(vehicle) = plot3(xTraj(vehicle,1),yTraj(vehicle,1),zTraj(vehicle,1),'o','MarkerFaceColor','red');
    pathHandle{vehicle} = [];%plot3(xTraj(vehicle,1), yTraj(vehicle,1), zTraj(vehicle,1), 'LineWidth',1.2,'Color','blue');
    bestTrajHandle(vehicle) = plot3(xTraj(vehicle,1), yTraj(vehicle,1), zTraj(vehicle,1), 'LineWidth',1.5, 'Color','black');
    plot3(droneList{vehicle}.goal(1),droneList{vehicle}.goal(2),droneList{vehicle}.goal(3),'-s','LineWidth',5,'Color','black');
end

% Set plot limits and labels
xlim([min(xTraj,[],'all') - 1000, max(xTraj,[],'all') + 5000 ]);
ylim([min(yTraj,[],'all') - 1000, max(yTraj,[],'all') + 5000 ]);
zlim([min(zTraj,[],'all') - 500, max(zTraj,[],'all') + 500]);
xlabel('x, m');
ylabel('y, m');
zlabel('z, m');
view(3);
grid on;

stepCounter = title(sprintf('step = %.2f', 1));
% text(-25000,-20000,-1500,'Navigation with ' + string(numDrones) +' agents','FontSize',14);

% Video capture
wobj = VideoWriter('test1.avi', 'Uncompressed AVI');
% wobj.Quality = 100;  % Adjust for desired video quality. 100 is the maximum and represents lossless.
wobj.FrameRate = 30;
open(wobj);

set(gcf, 'renderer', 'zbuffer');
% comment out this line if you want to simulate in default matlab figure
set(gcf, 'units','normalized','outerposition',[0 0 1 1]); 

% Dynamic Camera Initial Configuration
initAzim = 45; % starting azimuthal angle
azimRate = 0.05; % how much the angle changes per step
initElev = 30;
elevRate = 0.01;


for p = 1:maxlength
    for aircraft = 1:numDrones
        set(stepCounter, 'String', sprintf('step = %.2f',p));
        set(aircraftHandle(aircraft), 'XData', xTraj(aircraft,p), 'YData', yTraj(aircraft,p), 'ZData', zTraj(aircraft,p));

        % Update paths using patch for fading effect
        if p > 1
            xSegment = [xTraj(aircraft, p-1), xTraj(aircraft, p)];
            ySegment = [yTraj(aircraft, p-1), yTraj(aircraft, p)];
            zSegment = [zTraj(aircraft, p-1), zTraj(aircraft, p)];
            % Set the EdgeColor to blue
            newPath = patch([xSegment, NaN], [ySegment, NaN], [zSegment, NaN], 'b', 'EdgeAlpha', 1, 'EdgeColor', 'b', 'FaceColor', 'none', 'LineWidth', 1.2);

            pathHandle{vehicle} = [{newPath}, pathHandle{vehicle}]; % Add the new path segment to the beginning of the handle list

            % Fade out older path segments
            if length(pathHandle{vehicle}) > N
                delete(pathHandle{vehicle}{end});
                pathHandle{vehicle}(end) = [];
            end

            % Update alpha values
            for i = 1:length(pathHandle{vehicle})
                currAlpha = get(pathHandle{vehicle}{i}, 'EdgeAlpha');
                set(pathHandle{vehicle}{i}, 'EdgeAlpha', abs(currAlpha - alphaStep));
            end
        end
        % Update best trajectories
        set(bestTrajHandle(aircraft), 'XData', bestTrajectory(aircraft,1:3,1,p), 'YData', bestTrajectory(aircraft,1:3,2,p), 'ZData', bestTrajectory(aircraft,1:3,3,p));
    end

    % Dynamic Camera View
    currAzim = initAzim + azimRate * p;
    currElev = initElev + elevRate * p;
    view(currAzim, currElev); % fixed elevation, rotating azimuth

    drawnow;
    frame = getframe(gcf);
    writeVideo(wobj, frame);
    
    if p == 116
        % Set the figure size
        % set(gcf, 'Position', [0 0 900 600]);
        
        % Save the current figure as an EPS file
        print('-depsc', 'frame_116.eps');

        % Save the current figure as a PNG file
        print('-dpng', 'frame_116.png');

        % Reset figure to its previous size 
        % set(gcf, 'Position', [0 0 1920 1080]);
    end
end

close(wobj);