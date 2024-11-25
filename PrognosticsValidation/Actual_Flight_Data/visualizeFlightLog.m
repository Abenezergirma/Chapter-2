clear all; close all; clc
% load('useless1.mat'); % Long mission
% load('ShortMission.mat'); % Short mission
load('ShortMission.mat'); % Long mission

BatteryData = BAT_0;
GPSData = POS;
FlightLog = FlightLogAnalysis(BatteryData,GPSData,'LongMissionExtracted.mat');
FlightLog = FlightLog.extractFlightLog;
FlightLog.saveFlightLog
%%
% longFlightPlan = FlightLog.longFlightPlan;
% shortFlightPlan = FlightLog.shortFlightPlan;
% modelPath = '/home/abenezertaye/Desktop/Research/Codes/NASA/Flight test related/UAVSimulinkModel';
% T18Wrapper = SimulinkUAVWrapper(modelPath);
% T18Wrapper.modelScript = 'runDetailedT18Model';
% T18Wrapper.experimentName = strcat('longMission','SimulationParams.mat');
% T18Wrapper.resultsPath = '/home/abenezertaye/Desktop/Research/Codes/NASA/Flight test related';
% xyz = lla2ecef([longFlightPlan(:,1),longFlightPlan(:,2),longFlightPlan(:,3)]);
% [x, y, z] = deal(xyz(:,1)-xyz(1,1),xyz(:,2)-xyz(1,2),xyz(:,3)-xyz(1,3));
% T18Wrapper.runModel([x,y,longFlightPlan(:,3)+llaTraj(1,3)]);

% T18Wrapper.plotTrajectoryAndWaypoints([FlightLog.latitude,FlightLog.longitude,FlightLog.altitude],shortFlightPlan+[0,0,FlightLog.altitude(1)])
% 
% T18Wrapper.plotTrajectoryAndWaypoints(llaTraj,shortFlightPlan+[0,0,FlightLog.altitude(1)])
% 
% T18Wrapper.generateBatteryParametersPlot(results,1)
%%
figure;
% Subplot for SOC and voltage
subplot(2, 1, 1);
yyaxis left;
plot(SOC, 'b', 'LineWidth', 1.5);
ylabel('SOC');
ylim([0, 100]);
yyaxis right;
plot(voltage, 'r', 'LineWidth', 1.5);

ylabel('Voltage');
xlabel('Time (s)');
legend('SOC', 'Voltage', 'Location', 'southwest');
% legend('Totali', 'Location', 'Best'); % Adjust legend location
title('SOC and Voltage');
% Subplot for totali
subplot(2, 1, 2);
plot(totalCurrent, 'g', 'LineWidth', 1.5);
ylabel('TotalCurrent');
xlabel('Time (s)');
title('TotalCurrent');
% Set the filename for the PNG image
filename = fullfile('TotalCurrentShortFlight.png');
print(filename, '-dpng');
%%
% Adjust view and display the plot
view(3);
% axis equal;
hold off;



plot3(results{4}(:,1),results{4}(:,2),results{4}(:,3)-results{4}(1,3))
hold on
plot3(xyzTraj(:,1),xyzTraj(:,2),xyzTraj(:,3))

plot(voltage)
hold on
plot(results{3})

plot(results{6})
hold on
plot(time)
% Use both currents for prognostics and compare the results 



preProgPath = '/home/abenezertaye/Desktop/Research/Codes/NASA/eVTOL-energy-consumption/Post PSOPT/Results/PreProgResults';
postProgPath = '/home/abenezertaye/Desktop/Research/Codes/NASA/eVTOL-energy-consumption/Post PSOPT/Results/PostProgResults';
