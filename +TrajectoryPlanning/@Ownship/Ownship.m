classdef Ownship < TrajectoryPlanning.WindInterpolator
    %This class implements a point-mass dynamic model of T18 UAV aircraft
    %The model is adopted from Corbetta, M., Banerjee, P., Okolo, W.,
    % Gorospe, G., and Luchinsky, D. G., “Real-time uav trajectory prediction
    % for safety monitoring in low-altitude airspace," AIAA AVIATION 2019 forum, 2019, p. 3514.

    % This model is first implemented in my AIAA SCITECH paper as part of
    % my second chapter of my PhD dissertation

    % The main role of this model in this project is to desribe the
    % behavior of the aircraft in the trajectory planning stage of the
    % framework

    % Important properties:
    %      aircraftActions - Action set of the aircrft used to train the
    %      MDP Agent
    %      currentStates - 12 states of the aircraft at time t
    %      nextStates - Projected best states of the aircraft at t+1
    %      traveledPath - Past [x, y, z] positions of aircraft
    %      bestTrajectory - Projected best [x, y, z] of the aircraft

    % Important methods:
    %      continiousOctocopterModel
    %      discreteOctocopterModel - a discrete model of the aircraft
    %      forwardSimulate - simulates the future states of the aircraft
    %      T18Actions - generates an array that contains the control set
    %      selectBestAction - picks the best action and state for t+1

    properties (Access = public)
        % general Aircraft properties
        g = 9.81
        timeStep = 0.01;
        numSteps = 500;
        aircraftActions;

        % Aircraft states
        currentStates = [100,100,100,zeros(1,3),ones(1,3)*1.7,zeros(1,3)];
        x = zeros(1,1);
        y = zeros(1,1);
        z = zeros(1,1);
        xDot = zeros(1,1);
        yDot = zeros(1,1);
        zDot = zeros(1,1);
        phi = zeros(1,1);
        theta = zeros(1,1);
        psi = zeros(1,1);
        p = zeros(1,1);
        q = zeros(1,1);
        r = zeros(1,1);
        yaw = zeros(1,1);

        position = zeros(1,3);
        velocity = zeros(1,3);
        nextStates = zeros(1,12);

        traveledPath; % stores the path traveled by the aircraft so far
        pastControls; % stores the control actions applied by the aircraft so far
        bestTrajectory;% stores the best trajectories picked by the aircraft so far
        values;
        bestVal;

        %control actions
        T = zeros(1,1);
        tauPhi = zeros(1,1);
        tauTheta = zeros(1,1);
        tauPsi = zeros(1,1);
        controlActions;

        %Vehicle identifiers
        aircraftID;
        aircraftTeam;
        dead = false;
        baseLatitude;
        baseAltitude;
        goal;

        %vehicle performace parameters
        hit = false;
        hitCounter;
        Traces = 0;
        reachSet = 0;

    end

    methods(Static)
        function dydt =  continiousOctocopterModel(t,y,actions)
            Ixx = 0.2506;
            Iyy = 0.2506;
            Izz = 0.4538; %
            l = 1; % arm length
            mt = 15; % total mass of the UAV
            g = 9.81;

            % extract control actions
            n = length(actions(:,1));
            y = reshape(y, n, []);
            [T, tauPhi, tauTheta, tauPsi] = deal(actions(:,1), actions(:,2), actions(:,3), actions(:,4));
            [~, ~, ~, xDot, yDot, zDot, phi, theta, psi, p, q, r] = deal(y(:,1), ...
                y(:,2), y(:,3), y(:,4), y(:,5), y(:,6), y(:,7), y(:,8), y(:,9), y(:,10), y(:,11), y(:,12));

            % implement the dynamics assuming no wind
            xRate = xDot;
            yRate = yDot;
            zRate = zDot;
            xDRate = (sin(theta).*cos(psi).*cos(phi) + sin(phi).*sin(psi)).*T/mt;
            yDRate = (sin(theta).*sin(psi).*cos(phi) - sin(phi).*sin(psi)).*T/mt;
            zDRate = -g + (cos(psi).*cos(theta)).*T/mt;
            phiRate = p + q.*sin(phi).*tan(theta) + r.*cos(phi).*tan(theta);
            thetaRate = q.*cos(phi) - r.*sin(phi);
            psiRate = q.*(sin(phi)./cos(theta)) + r.*(cos(phi)./cos(theta));
            pRate = ((Iyy - Izz)/Ixx).*q.*r + (l/Ixx).*tauPhi;
            qRate = ((Izz - Ixx)/Iyy).*p.*r + (l/Iyy).*tauTheta;
            rRate = ((Ixx - Iyy)/Izz).*q.*r + (l/Izz).*tauPsi;
            dydt = [xRate; yRate; zRate; xDRate; yDRate; zDRate; phiRate;thetaRate;psiRate;pRate;qRate;rRate ];
        end


        function futureTraj = OctocopterModelWind(ownship, actions, timestep, numSteps)
            % A function that returns a set of next states with state constraints
            % and wind effect applied
            % This model assumes the wind acceleration is zero (wx_dot,wy_dot,wz_dot = 0)
            Ixx = 0.2506;
            Iyy = 0.2506;
            Izz = 0.2506*4; %
            l = 0.635; % arm length
            mt = 12.66; % total mass of the UAV
            g = 9.81;

            % Extract state variables
            currentState = ownship.currentStates;
            
            deltaX = ownship.goal(1) - currentState(1);
            deltaY = ownship.goal(2) - currentState(2);
            % Calculate the yaw angle

            yaw = atan2(deltaY ,deltaX); 
            % hold on;
            % 
            % plot(currentState(1), currentState(2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Initial Locations');
            % plot(ownship.goal(1),ownship.goal(2), 'rx', 'MarkerSize', 10, 'DisplayName', 'Goal Locations');
            % hold on
            % quiver(currentState(1), currentState(2), cos(yaw), sin(yaw), 3.3, 'k', 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Yaw Direction');

            [x, y, z, xDot,yDot,zDot, phi, theta, psi, p, q, r] = deal(currentState(:,1), ...
                currentState(:,2),currentState(:,3),min(max(currentState(:,4), -3), 3),...
                min(max(currentState(:,5),-3),3),min(max(currentState(:,6),-2.5),2.5),...
                min(max(currentState(:,7), deg2rad(-35)), deg2rad(35)), ...
                min(max(currentState(:,8), deg2rad(-35)), deg2rad(35)), ...
                yaw,0,0,0); 

            % %Pre-compute trig values
            % sTheta = sin(theta);
            % cTheta = cos(theta);
            % tTheta = tan(theta + 0.0001);
            % sPsi = sin(psi);
            % cPsi = cos(psi);
            % sPhi = sin(phi);
            % cPhi = cos(phi);
            % 

            % Extract control variables
            [T, tauTheta, tauPhi, tauPsi] = deal(actions(:,1), actions(:,2), actions(:,3), actions(:,4));

            % Preallocate memory for speed
            futureTraj = zeros(numSteps, length(actions(:,1)), length(currentState(1,:))); %Preallocated memory for speed

            for i = 1:numSteps
                % compute drag forces
                % Dx = max(min(Dx, 15), -15);
                % Dy = max(min(Dy, 15), -15);
                % Dz = max(min(Dz, 15), -15);
                % Dx = 0; Dy = 0; Dz =0;

                %Pre-compute trig values
                sTheta = sin(theta);
                cTheta = cos(theta);
                tTheta = tan(theta + 0.0001);
                sPsi = sin(psi);
                cPsi = cos(psi);
                sPhi = sin(phi);
                cPhi = cos(phi);

                % vBody = ownship.rot_eart2body_fast_vectorized(sPhi, cPhi, sTheta, cTheta, sPsi, cPsi)*[xDot,yDot,zDot]';

                bodyVel = ownship.transform_velocity_to_body_frame(sPhi, cPhi, sTheta, cTheta, sPsi, cPsi, xDot, yDot, zDot);

                [wX, wY] = ownship.get_wind_at(x, y);
                wZ = zeros(length(wX),1);
                wBody = ownship.transform_velocity_to_body_frame(sPhi, cPhi, sTheta, cTheta, sPsi, cPsi,wX, wY, wZ);

                vAir = bodyVel - wBody;
                [Dx_body, Dy_body, Dz_body] = ownship.computeDragForces(x,y,z,vAir(:,1),vAir(:,2),vAir(:,3));

                dragForces= ownship.transform_velocity_to_earth_frame(sPhi, cPhi, sTheta, cTheta, sPsi, cPsi,Dx_body, Dy_body, Dz_body);
                [Dx, Dy, Dz] = deal(dragForces(:,1), dragForces(:,2), dragForces(:,3));


                % Update the linear speeds [x4 - x6]
                xDot = xDot + timestep.*((sTheta.*cPsi.*cPhi + sPhi.*sPsi).*T/mt - Dx/mt);
                xDot = min( max(xDot, -9.5), 9.5);
                yDot = yDot + timestep.*((sTheta.*sPsi.*cPhi - sPhi.*sPsi).*T/mt - Dy/mt);
                yDot = min( max(yDot, -9.5), 9.5);
                zDot = zDot + timestep.*((-g + (cPhi.*cTheta).*T/mt) - Dz/mt);
                zDot = min( max(zDot, -9.5), 9.5);

                % Update the linear positions [x1 - x3]
                x = x + xDot.*timestep;
                y = y + yDot.*timestep;
                z = z + zDot.*timestep;

                % Update the angular speeds [x10 - x12]
                p = p + timestep.*(((Iyy - Izz)/Ixx).*q.*r + (l/Ixx).*tauPhi);
                p =  min( max(p, deg2rad(-45)), deg2rad(45));

                q = q + timestep.*(((Izz - Ixx)/Iyy).*p.*r + (l/Iyy).*tauTheta);
                q =  min( max(q, deg2rad(-45)), deg2rad(45));

                r = r + timestep.*(((Ixx - Iyy)/Izz).*q.*r + (1/Izz).*tauPsi);
                r =  min( max(r, deg2rad(-45)), deg2rad(45));

                % Update the angular pos [x7 - x9]
                phi = phi + timestep.*(p + q.*sPhi.*tTheta + r.*cPhi.*tTheta);
                phi =min( max(phi, deg2rad(-45)), deg2rad(45));%wrapTo2Pi(phi);%

                theta = theta + timestep.*(q.*cPhi - r.*sPhi);
                theta = min( max(theta, deg2rad(-45)), deg2rad(45));

                psi = psi + timestep.*(q.*(sPhi./cTheta) + r.*(cPhi./cTheta));
                psi =  min( max(psi, yaw + deg2rad(-20)), yaw + deg2rad(20));

                % R = ownship.rot_body2earth_fast_vectorized(sPhi, cPhi, sTheta, cTheta, sPsi, cPsi);
                % 
                % % Ensure that the velocity matrix is 4608x3
                % vel = [xDot, yDot, zDot];  % Shape should be 4608x3

                % v_ground = ownship.multiply_rotation_and_velocity(R, vel);

               
                nextState = [x, y, z, xDot, yDot, zDot, phi, theta, psi, p, q, r];

                % Store the states into an array
                futureTraj(i,:,:) = nextState;
            end
        end


        function T18Actions = T18Actions(varargin)
            % Builds an array that stores the action space of each team
            Thrust  = linspace(50,600,9); %10.^[linspace(0, log10(300), 5),linspace(log10(400), log10(700), 5)]; %0:25*1:200;
            % tauTheta = [-10.^linspace(0,log10(100), 5),10.^linspace(log10(10), log10(100), 5)];%-100:25*1:100;
            % tauPhi = [-10.^linspace(0,log10(100), 5),10.^linspace(log10(10), log10(100), 5)];%-100:25*1:100;
            % tauPsi = [-10.^linspace(0,log10(100), 5),10.^linspace(log10(10), log10(100), 5)];%-100:25*1:100;
            % tauTheta = [-100.0000 -56.2341  -31.6228  -17.7828 -10.0000 0 10.0000   17.7828   31.6228   56.2341  100.0000];
            % tauTheta = [-180.2341  -100.6228  -50.7828  0   50.7828   100.6228   180.2341 ];
            tauTheta = linspace(-150,150,7);
            % tauTheta = [-56.2341  -31.6228  -17.7828  0   17.7828   31.6228   56.2341 ];


            tauPhi = tauTheta;
            tauPsi = tauTheta;
            % construct the joint action space that comprises the three action spaces
            actions = {Thrust, tauTheta, tauPhi, tauPsi};
            actionsD = actions;
            [actionsD{:}] = ndgrid(actions{:});
            teamActions = cell2mat(cellfun(@(m)m(:),actionsD,'uni',0));
            T18Actions = teamActions;
        end
    end

    methods
        function obj = Ownship(windDataPath)
            % A constructir method that initializes each aircraft in the game
            load(windDataPath, 'x', 'y', 'x_velocity', 'y_velocity');
            x_data = x;
            y_data = y;
            wx_data = x_velocity;
            wy_data = y_velocity;
            obj@TrajectoryPlanning.WindInterpolator(x_data, y_data, wx_data, wy_data);
            obj.aircraftActions = TrajectoryPlanning.Ownship.T18Actions;
        end


        function R = rot_body2earth_fast_vectorized(obj, sphi, cphi, stheta, ctheta, spsi, cpsi)
            % Vectorized computation of rotation matrices to transform coordinates from
            % the body frame to the Earth-fixed (inertial) frame for multiple angles.
            %
            % Input: All inputs are vectors of size Nx1 (sine and cosine of roll, pitch, and yaw)
            % Output: R is a 3x3xN array where each slice is a rotation matrix

            n = length(sphi);  % Number of trajectories
            R = zeros(3, 3, n);  % Preallocate for the 3x3xN result

            % Compute each element of the rotation matrix in a vectorized way
            R(1, 1, :) = cpsi .* ctheta;
            R(1, 2, :) = cpsi .* stheta .* sphi - spsi .* cphi;
            R(1, 3, :) = cpsi .* stheta .* cphi + spsi .* sphi;

            R(2, 1, :) = spsi .* ctheta;
            R(2, 2, :) = spsi .* stheta .* sphi + cpsi .* cphi;
            R(2, 3, :) = spsi .* stheta .* cphi - cpsi .* sphi;

            R(3, 1, :) = -stheta;
            R(3, 2, :) = ctheta .* sphi;
            R(3, 3, :) = ctheta .* cphi;
        end


        function R = rot_eart2body_fast_vectorized(obj, sphi, cphi, stheta, ctheta, spsi, cpsi)
            % Vectorized computation of rotation matrices to transform coordinates from
            % the Earth-fixed (inertial) frame to the body frame for multiple angles.
            %
            % Input: All inputs are vectors of size Nx1 (sine and cosine of roll, pitch, and yaw)
            % Output: R is a 3x3xN array where each slice is a rotation matrix

            n = length(sphi);  % Number of trajectories
            R = zeros(3, 3, n);  % Preallocate for the 3x3xN result

            % Compute each element of the rotation matrix in a vectorized way
            R(1, 1, :) = ctheta .* cpsi;
            R(1, 2, :) = ctheta .* spsi;
            R(1, 3, :) = -stheta;

            R(2, 1, :) = -cphi .* spsi + sphi .* stheta .* cpsi;
            R(2, 2, :) = cphi .* cpsi + sphi .* stheta .* spsi;
            R(2, 3, :) = sphi .* ctheta;

            R(3, 1, :) = sphi .* spsi + cphi .* stheta .* cpsi;
            R(3, 2, :) = -sphi .* cpsi + cphi .* stheta .* spsi;
            R(3, 3, :) = cphi .* ctheta;
        end



        function bodyVel = transform_velocity_to_body_frame(obj, sphi, cphi, stheta, ctheta, spsi, cpsi, xDot, yDot, zDot)
            % Compute the rotation matrices using the vectorized function (3x3xN)
            R = rot_eart2body_fast_vectorized(obj, sphi, cphi, stheta, ctheta, spsi, cpsi);

            % Stack the velocity components into a 3x1xN array (for multiple trajectories)
            V_inertial = [xDot'; yDot'; zDot'];  % 3xN array
            V_inertial = reshape(V_inertial, 3, 1, []);  % Reshape to 3x1xN

            % Perform matrix multiplication between 3x3xN rotation matrices and 3x1xN velocity vectors
            bodyVel = pagemtimes(R, V_inertial);  % Result will be 3x1xN

            % Reshape bodyVel to 3xN and then transpose it to Nx3
            bodyVel = squeeze(bodyVel);  % Remove the singleton dimension
            bodyVel = bodyVel';  % Transpose to get Nx3

        end

        function earthVel = transform_velocity_to_earth_frame(obj, sphi, cphi, stheta, ctheta, spsi, cpsi, xDot, yDot, zDot)
            % Compute the rotation matrices using the vectorized function (3x3xN)
            R = rot_body2earth_fast_vectorized(obj, sphi, cphi, stheta, ctheta, spsi, cpsi);

            % Stack the velocity components into a 3x1xN array (for multiple trajectories)
            V_body = [xDot'; yDot'; zDot'];  % 3xN array
            V_body = reshape(V_body, 3, 1, []);  % Reshape to 3x1xN

            % Perform matrix multiplication between 3x3xN rotation matrices and 3x1xN velocity vectors
            earthVel = pagemtimes(R, V_body);  % Result will be 3x1xN

            % Reshape earthVel to 3xN and then transpose it to Nx3
            earthVel = squeeze(earthVel);  % Remove the singleton dimension
            earthVel = earthVel';  % Transpose to get Nx3
        end


        function [Dx, Dy, Dz] = computeDragForces(obj,x,y,z, Vx, Vy, Vz)

            Cd = 1; % drag coefficient
            Axy = 0.403; % apparent face of the vehicle
            Ayx = 0.403;
            Ayz = 0.403;
            rho = 1.223; % air density

            % Interpolate wind speed components at aircraft position
            % [Wx, Wy] = calculateWindComponents(obj, x, y);
            % tic
            % [Wx, Wy] = obj.get_wind_at(x, y);
            % toc

            % Vr_x = Vx - Wx;
            % Vr_y = Vy - Wy;
            % Vr_z = 0;
            Dx = (1/2) * Cd * Axy * Vx .* abs(Vx);
            Dy = (1/2) * Cd * Ayx * Vy .* abs(Vy);
            Dz = (1/2) * Cd * Ayz * Vz .* abs(Vz);
        end

        function [obj] = updateAircraftStates(obj,  currentStates)
            % A method that updates the states of the aircraft
            obj.x = currentStates(1);
            obj.y = currentStates(2);
            obj.z = currentStates(3);
            obj.xDot = currentStates(4);
            obj.yDot = currentStates(5);
            obj.zDot = currentStates(6);
            obj.phi = currentStates(7);
            obj.theta = currentStates(8);
            obj.psi = currentStates(9);
            obj.p = currentStates(10);
            obj.q = currentStates(11);
            obj.r = currentStates(12);
            obj.position = [obj.x, obj.y, obj.z];
            obj.currentStates = currentStates;

        end

        function [obj] = updateControlActions(obj,  bestActions)
            % A method that updates the control actions based on the
            % selected best actions
            obj.T = bestActions(1);
            obj.tauPhi= bestActions(2);
            obj.tauTheta = bestActions(3);
            obj.tauPsi = bestActions(4);
            obj.controlActions = bestActions;
        end

        function [obj] = selectBestAction(obj, totalValues, futureActions, futureStates, oneStepStates)
            % A method that selects the optimal action
            bestValue = max(totalValues, [],'all');

            [bestRow, bestColumn, ~] = find(totalValues==bestValue);
            randomIndexRow = 1;%abs(randi(length(bestRow))); % randomly pick one index with max value
            randomIndexCol = 1;%abs(randi(length(bestColumn)));
            bestActions = futureActions(bestRow(randomIndexRow), bestColumn(randomIndexCol),:);
            % bestActions = futureActions(bestRow(1),bestColumn(1),:);

            % select the best next state
            bestStep = squeeze(oneStepStates(bestRow(randomIndexRow), bestColumn(randomIndexCol),:));

            %select the best future trajectory for the next 10 sec
            bestTraj = squeeze(futureStates(:, bestColumn(randomIndexCol),1:3)) ;

            % update the states and control actions of the aircraft
            obj = obj.updateControlActions(bestActions);
            obj.nextStates = squeeze(bestStep)';

            %store the control actions and path traveled so far
            obj.traveledPath = [obj.traveledPath; obj.nextStates];
            obj.pastControls = [obj.pastControls; obj.controlActions];
            obj.bestTrajectory = [obj.bestTrajectory; bestTraj];
            % obj.bestTrajectory = [obj.bestTrajectory; obj.nextStates(1:3)];
            %             obj.values = [obj.values;totalValues];
            obj.bestVal = [obj.bestVal;bestValue];

        end

        function obj = forwardSimulate(obj, controlActions,timeStep, numSteps)
            obj.Traces = TrajectoryPlanning.Ownship.OctocopterModelWind(obj, controlActions,timeStep, numSteps);
        end

        function obj = computeReachTube(obj, DryVR_obj)
            if obj.Traces == 0

                if obj.hit == true
                    actions = obj.aircraftActions;
                    actions(:,3) = 24;
                    control_Actions = unique(actions, 'rows');
                else
                    control_Actions = obj.aircraftActions;
                end
                obj = forwardSimulate(obj, control_Actions,obj.timeStep, obj.numSteps);
            end
            traces = obj.Traces(3:end,floor(linspace(1,length(obj.Traces(1,:,1)),DryVR_obj.numTraces)),:);
            t = 0:obj.timeStep:(obj.numSteps-3)*obj.timeStep;

            traces(:,:,end+1) = repmat(t,1,1,DryVR_obj.numTraces); %DO this before the simulation
            traces = permute(traces(:,:,[end,1:end-1]), [2,1,3]);
            initialRadii = computeInitialRadii(DryVR_obj,traces);
            discrepancyParameters = computeDiscrepancyParameters(DryVR_obj, traces, initialRadii);
            reachTube = getReachtube(DryVR_obj, traces, initialRadii, discrepancyParameters);
            obj.reachSet = reachTube;
        end

    end
end