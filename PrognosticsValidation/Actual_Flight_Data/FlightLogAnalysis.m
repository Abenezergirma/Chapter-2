classdef FlightLogAnalysis
    properties
        BatteryData
        time
        voltage
        current
        energyTotal
        SOC
        GPSData
        latitude
        longitude
        altitude
        x
        y
        z
        shortMissionIndex
        % batteryIndex = 585; % for long mission
        % posIndex = 2000; % for long mission
        batteryIndex  % for short mission
        posIndex  % for short mission
        newCurrent
        newVoltage
        newSOC
        fileName
    end
    methods
        function obj = FlightLogAnalysis(BatteryData,GPSData,fileName)
            obj.BatteryData = BatteryData;
            obj.GPSData = GPSData;
            obj.fileName = fileName;
            if strcmp(fileName,'ShortMissionExtracted.mat')
                obj.batteryIndex = 654;
                obj.posIndex = 2231;

            elseif strcmp(fileName,'LongMissionExtracted.mat')
                obj.batteryIndex = 585;
                obj.posIndex = 2000;
            end
        end

        function obj = extractFlightLog(obj)
            % Battery related params
            TimesUS = obj.BatteryData(:,2);
            obj.time = (TimesUS - TimesUS(1))/1e6;
            obj.voltage = obj.BatteryData(:,4);
            obj.current = obj.BatteryData(:,6);
            obj.energyTotal = obj.BatteryData(:,8);
            obj.SOC = obj.BatteryData(:,11);
            % GPS related params
            obj.latitude = obj.GPSData(:,3);
            obj.longitude = obj.GPSData(:,4);
            obj.altitude = obj.GPSData(:,5);

            %Preprocess the extracted data
            %This step is mainly due to the fact that the UAV acts anomaly
            %in the begining of both flights
            obj.newCurrent = obj.current(obj.batteryIndex:end);
            obj.newVoltage = obj.voltage(obj.batteryIndex:end);
            obj.newSOC = obj.SOC(obj.batteryIndex:end);

            xyz = lla2ecef([obj.latitude(obj.posIndex:end), ...
                obj.longitude(obj.posIndex:end), obj.altitude(obj.posIndex:end)]);

            [x, y, z] = deal(xyz(:,1),xyz(:,2),xyz(:,3));
            obj.x = x-x(1);
            obj.y = y-y(1);
            obj.z = obj.altitude(obj.posIndex:end)-obj.altitude(1);
        end

        function saveFlightLog(obj)
            time = obj.time;
            SOC = obj.newSOC;
            totalCurrent = obj.newCurrent;
            voltage = obj.newVoltage;
            xyzTraj = [obj.x,obj.y,obj.z];
            llaTraj = [obj.latitude(obj.posIndex:end), ...
                obj.longitude(obj.posIndex:end), obj.altitude(obj.posIndex:end)];
            save(obj.fileName,'time','SOC',"totalCurrent","voltage","llaTraj","xyzTraj");

        end

        function shortFlightPlan = shortFlightPlan(obj)
            homePos = [38.83522906,-77.50622338,0];
            homePos1 = [38.83522906,-77.50622338,30];
            wayPoint1 = [38.83495471,-77.50679733,30];
            wayPoint2 = [38.83529002,-77.50708431,30];
            wayPoint3 = [38.83578181,-77.50593902,40];
            wayPoint4 = [38.83593422,-77.50536246,30];
            wayPoint5 = [38.83563143,-77.50514592,30];
            wayPoint6 = [38.83539366,-77.50562074,30];

            shortFlightPlan = [homePos;homePos1;wayPoint1;wayPoint2;wayPoint3;wayPoint4;wayPoint5;wayPoint6;homePos1;homePos];
            % plot3(shortFlightPlan(:,1),shortFlightPlan(:,2),shortFlightPlan(:,3))
            % hold on
            % plot3(llaTraj(:,1),llaTraj(:,2),llaTraj(:,3)-llaTraj(1,3))
        end

        function longFlightPlan = longFlightPlan(obj)

            wayPoint1 = [38.8352511,-77.5062340,56.31]; %idx 0
            wayPoint2 = [38.83525160	-77.50623230	96.720];%idx 1254
            wayPoint3 = [38.8348	-77.5073	96.64];%idx 1254
            wayPoint4 = [38.8351	-77.5074	96.64];%idx 1254
            wayPoint5 = [38.8357	-77.5067	96.64];%idx 1254
            wayPoint6 = [38.836	-77.5061	96.64];%idx 1254
            wayPoint7 = [38.8357	-77.5055	96.64];%idx 1254
            wayPoint8 = [38.8355	-77.5054	96.64];%idx 1254
            wayPoint9 = [38.8353	-77.5058	96.64];%idx 1254
            wayPoint10 = [38.8357	-77.5061	96.64];%idx 1254
            wayPoint11 = [38.8353	-77.5069	96.64];%idx 1254
            wayPoint12 = [38.8352	-77.5062	96.69];%idx 1254
            wayPoint13 = [38.8352	-77.5062	56.64];%idx 1254

            longFlightPlan = [wayPoint1;wayPoint2;wayPoint3;wayPoint4;wayPoint5;wayPoint6;...
                wayPoint7;wayPoint8;wayPoint9;wayPoint10;wayPoint11;wayPoint12;wayPoint13];
        end

    end

end