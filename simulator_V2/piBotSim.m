classdef piBotSim < handle
    %PIBOTSIM A piBot simulator
    
    % Private dynamic states
    properties(GetAccess = private, SetAccess=private)
        robotPosition = [0;0]; % The robot's position in the world (m).
        robotAngle = 0; % The robots angle w.r.t. the world x axis (rad).
        robotWheelVelocity = [0,0]; % The speeds of the robot's left and right wheels.
%         robotAccel = 0;
        commandDuration = 0; % The duration of the current wheel command.
        simTime = 0.0; % The time since the start of the simulation.
        simRobotTrail = []; % A vector storing the robot's position over time.
        simRobotTrailLine; % A plot of the robot's position over time.
        simRobotTriangle; % A triangle at the current position and angle of the robot.
        simLandmarkMarkers; % Square markers showing the positions of the landmarks.
        simLandmarkLabels; % Text labels showing the id numbers of the landmarks.
        simRobotViewCapture; % The shape of the robot's view of landmarks.
        simDelayTimer = nan; % The timer used to keep the simulation in real-time.
    end
    
    % States to be set upon instantiation of the simulator.
    properties(GetAccess = private, SetAccess = immutable)
        floorImage = imread("floor.jpg"); % The image used to make the floor.
        worldLandmarkNumber = 20; % The number of landmarks in the world
        worldLandmarkPositions; % The positions of the landmarks (2xn) (m)
    end
    
    % Static parameters of the world and robot
    properties(Constant)
        simTimeStep = 0.1; % The duration of each step of the simulator (s).
        robotWheelVelScale = 5.33e-3; % The scaling applied to wheel velocities (tk/s).
        robotWheelVelNoise = 2.0/50; % Variance of the Gaussian noise added to the wheel velocities.
        robotMeasureNoisePosition = 0.005; % Variance of the Gaussian noise added to position measurements.
        robotMeasureNoiseAngle = 0.08; % Variance of the Gaussian noise added to angle measurements.
        robotWheelTrack = 0.156; % The distance between the robot wheels (m).
        worldBoundaries = [0,5;0,5]; % The world is a 5m x 5m square.
        simRobotTrailLimit = 5000; % The maximum entries in the trail. Reduce to save memory.
        robotCameraK = [200 0 200; 0 200 0; 0 0 1]; % The robot camera matrix.
        robotCameraHeight = 0.1; % The height of the camera from the ground (m).
        robotCameraR = [0,-1,0;0,0,-1;1,0,0]'; % The orienation of the camera w.r.t. the robot.
        robotCameraRef = imref2d([200,400]); % Size of the camera output image.
        worldARUCOSize = 0.08; % Size of ARUCO landmarks (m).
        worldARUCONoise = 0.01; % Noise on projection measurement of ARUCO landmarks.
        worldLandmarkMinDist = 0.2; % Minimum distance between generated landmarks.
        % The following three parameters are used to determine which
        % landmarks are in view of the robot.
        robotLandmarkViewAngle = 45 * pi/180; % Maximum absolute angle of landmarks from the robot x axis (rad).
        robotLandmarkDistMin = 0.1; % Minimum distance of landmarks from the robot (m)
        robotLandmarkDistMax = 3.0; % Maximum distance of landmarks from the robot (m)
    end
    
    % The interface methods of the robot simulator
    methods
        function self = piBotSim(floorImageFName, landmarkPositions)
            % Constructor of the simulator
            if nargin >= 1
                % Read the provided floor image
                self.floorImage = imread(floorImageFName);
            end
            if nargin >= 2
                % Use the provided landmark number and positions
                self.worldLandmarkNumber = size(landmarkPositions,2);
                self.worldLandmarkPositions = landmarkPositions;
            else
                % Generate n random landmarks
                self.worldLandmarkPositions = rand(2,self.worldLandmarkNumber) .* self.worldBoundaries(:,2);
                self.worldLandmarkPositions = self.ensureMinimumDist(self.worldLandmarkPositions, self.worldLandmarkMinDist);
            end
            
            % Initialise the robot trail.
            self.simRobotTrail = NaN(3, self.simRobotTrailLimit);
            self.simRobotTrail(:,end) = [self.robotPosition; self.robotAngle];
            
            % Initialise drawing the world
            figure;
            self.drawWorld();
        end
        
        function simulate(self, varargin)
            %PiBotSim.simulate  Advance the simulation
            %
            % PB.simulate() integrates the dynamics of the simulation for a period of
            % PiBotSim.simTimeStep.
            %
            % PB.simulate(duration) integrates the dynamics of the simulation for
            % the duration specified in steps of length PiBotSim.simTimeStep.
            
            % Handle the duration being given.
            if length(varargin) == 1
                total_duration = varargin{1};
                while total_duration > 0
                    total_duration = total_duration - self.simTimeStep;
                    self.simulate()
                end
            end
            
            % Integrate the (noisy) kinematics of the robot
            self.integrateRobotKinematics(min(self.commandDuration, self.simTimeStep));
            self.commandDuration = max(self.commandDuration - self.simTimeStep, 0);
            
            % Update the robot trail
            self.simRobotTrail(:,1:end-1) = self.simRobotTrail(:,2:end);
            self.simRobotTrail(:,end) = [self.robotPosition; self.robotAngle];
            
            % check if the robot has crashed against the wall
            if ~self.positionInBounds(self.robotPosition)
                error("The robot has left the world!")
            end
            
            % unblock the simulation and update the time
            self.simTime = self.simTime + self.simTimeStep;
            
            % Draw the updated world
            self.updateDrawWorld();
            
            % Force the simulation to be no faster than realtime
            if isnan(self.simDelayTimer)
                self.simDelayTimer = tic;
            end
            pause(self.simTimeStep - toc(self.simDelayTimer));
            self.simDelayTimer = tic;
            
        end
                
        
        % set velocity command
        function setVelocity(self, varargin)
            %PiBotSim.setVelocity  Set the speeds of the motors
            %
            % PB.setVelocity(Vleft, Vright) sets the velocities of the two motors to
            % the values Vleft and Vright respectively.
            %
            % PB.setVelocity(VEL, T) sets the speeds of the two motors to the values in
            % the 2-vector VEL = [Vleft, Vright] and the motion runs for T seconds.
            % Timing is done locally on the RPi.
            %
            % PB.setVelocity(VEL, T, ACC) as above but the speed ramps up and down at
            % the end of the motion over ACC seconds.  The constant velocity part of
            % the motion lasts for T-ACC seconds, the total motion time is T+2*ACC
            % seconds.  This profile moves the same distance as a rectangular speed
            % profile over T seconds.
            %
            % Notes::
            % - The motor speed is 10V encoders per second.
            % - If T is given the total distance travelled is 10V*T encoders.
            % - If ACC is also given the total distance travelled is 10V*(T-ACC)
            %   encoders.
            %
            % See also PiBotSim.stop.
            
            % Set defaults first
            duration = Inf;
            accel = 0;
            vel = [0,0];
            
            % Parse the input arguments
            if length(varargin{1}) == 1
                % then (SA, SB) classic format
                vel(1) = varargin{1}; vel(2) = varargin{2};
                
                assert(all(isreal(vel)), 'arguments must be real');
                vel = round(vel);  % convert to int
                vel = min(max(vel, -100), 100);  % clip
                
            elseif length(varargin{1}) == 2
                % then (SPEED), (SPEED, T) or (SPEED, T, A)
                
                % Get the speed.
                vel = varargin{1};
                assert(all(isreal(vel)), 'arguments must be real');
                vel = round(vel);  % convert to int
                vel = min(max(vel, -100), 100);  % clip
                
                if length(varargin) >= 2
                    % If available, get the duration
                    duration = varargin{2};
                    assert(duration > 0, 'duration must be positive');
                    assert(duration < 20, 'duration must be < network timeout');
                    
                    
                    if length(varargin) >= 3
                        % If available, get the acceleration
                        error("Acceleration is not currently available in simulation.")
                        accel = varargin{3};
                        assert(accel < duration/2, 'accel must be < duration/2');
                    end
                end
            end
            
            % Set the relevant state variables
            self.robotWheelVelocity = vel;
            self.commandDuration = duration;
%             self.robotAccel = accel;
            
            % If the duration is set, update the robot state until the
            % command is complete.
            while self.commandDuration > 1e-3 && ~isinf(self.commandDuration)
                self.simulate()
            end
        end
        
        
        function stop(self)
            %PiBotSim.stop  Stop all motors
            %
            % PB.stop() stops all motors.
            %
            % See also PiBot.setVelocity.
            
            self.setVelocity(0, 0);
        end
        
        
        function place(self, position, angle)
            %PiBotSim.place  Place the robot in the world
            %
            % PB.place(position, angle) places the robot at the specified position and
            % angle w.r.t. the world frame.
            %
            % Notes::
            % - During testing of submitted code, this function will not be available.
            %   You may not change the position or angle of your robot by placing it
            %   when the robot is supposed to be in operation.
        
            % Place the robot somewhere in the world.
            assert(all(size(position) == [2,1]), "Position must be a 2x1 vector.");
            assert(numel(angle) == 1, "Angle must be a scalar.");
            assert(self.positionInBounds(position), "The robot cannot be placed outside the room.")
            
            self.robotPosition = position;
            self.robotAngle = angle;
            
            % Reset the robot position trail.
            self.simRobotTrail = NaN(3, self.simRobotTrailLimit);
            self.simRobotTrail(:,end) = [self.robotPosition; self.robotAngle];
            self.updateDrawWorld();
        end
        
        
        function img = getCamera(self)
            % Get an image from the robot camera
            
            % Compute the warp map.
            R = self.rotz(self.robotAngle)*self.robotCameraR;
            x = [self.robotPosition; self.robotCameraHeight];
            e3 = [0;0;1];
            H = self.robotCameraK * R' * (inv(self.floorImageK()) - e3*e3' - x*e3');
            
            % Apply the warp to the floor image.
            img = imwarp(self.floorImage, projective2d(H'), ...
            'OutputView', self.robotCameraRef, ...
            'FillValues', [220, 220, 220]);
        
            % Advance the world.
            self.simulate();
        end
        
        
        function [position, angle] = measure(self)
            %PiBotSim.measure  Measure the robot's pose in the world
            %
            % [position, angle] = PB.measure() measures the robot's current position
            % and angle w.r.t. the world frame. Note this measurement is noisy!
            %
            % Notes::
            % - This function will not be available for every coding task.
            %   For example, you may not measure the robot's position while
            %   it is following a line or performing visual odometry.
            
            position = self.robotPosition + randn(2,1) * self.robotMeasureNoisePosition;
            angle = self.robotAngle + randn() * self.robotMeasureNoiseAngle;
        end
        
        
        function saveTrail(self)
            %PiBotSim.saveTrail()  Save the trail of robot poses.
            %
            % PB.saveTrail() saves the trail of the robot poses to a file.
            %
            % Notes::
            % - This function will not be available for every coding task.
            %   For example, you may not save the robot trail while it is
            %   following a line or performing visual odometry.
            
            simRobotTrail = self.simRobotTrail;
            save("robot_trail.mat", "simRobotTrail");
        end
        
        function [landmarks, idnumbers] = measureLandmarks(self)
            %PiBotSim.measureLandmarks()  Measure landmarks from the robot POV.
            %
            % [landmarks, idnumbers] = PB.measureLandmarks() returns the landmark
            % measurements (in the robot frame) and the id numbers of the landmarks
            % that were measured.

            R = self.rmat(self.robotAngle);
            t = self.robotPosition;
            
            % Identify which landmarks are visible
            landmarks = R' * (self.worldLandmarkPositions - t);
            landmarkAngles = atan2(landmarks(2,:), landmarks(1,:));
            landmarkDistances = vecnorm(landmarks);
            viewAngle = 45*pi/180;
            landmarksInView = (abs(landmarkAngles) < self.robotLandmarkViewAngle) & ...
                    (landmarkDistances > self.robotLandmarkDistMin) & ...
                    (landmarkDistances < self.robotLandmarkDistMax);
            
            % Simulate a realistic measurements
            idnumbers = find(landmarksInView);
            
            for i = idnumbers
                landmarks(:,i) = self.applyARUCONoise(landmarks(:,i));
            end
            
            landmarks = landmarks(:, idnumbers);
        end
        
        function showLandmarks(self, flag)
            % Show landmarks if flag is true and hide if flag is false.
            self.simLandmarkMarkers.Visible = flag;
            for i = 1:self.worldLandmarkNumber
                self.simLandmarkLabels(i).Visible = flag;
            end
        end
        
        function showViewRange(self, flag)
           self.simRobotViewCapture.Visible = flag; 
        end
    end

    % Internal methods
    methods(Hidden, Access=protected)
        
        function integrateRobotKinematics(self, dt)
            % Integrate the kinematics of the robot.
            wheel_noise = self.wheelNoise(norm(self.robotWheelVelocity));
            [v,omega] = self.forwardKinematics(self.robotWheelVelocity + wheel_noise);

            theta_t = self.robotAngle + dt * omega;
            if (omega == 0)
                self.robotPosition(1) = self.robotPosition(1) + dt * v * cos(self.robotAngle);
                self.robotPosition(2) = self.robotPosition(2) + dt * v * sin(self.robotAngle);
            else
                self.robotPosition(1) = self.robotPosition(1) + v/omega * (sin(theta_t) - sin(self.robotAngle));
                self.robotPosition(2) = self.robotPosition(2) - v/omega * (cos(theta_t) - cos(self.robotAngle));
            end
            self.robotAngle = theta_t;
        end
        
        
        function drawWorld(self)
            % draw the current state of the world and the robot in it
            currentHold = ishold();
            
            % First draw the floor
            imshow(self.floorImage, imref2d([size(self.floorImage,1), size(self.floorImage,2)], [0,5], [0,5]));
            hold on
            
            % Draw the robot trail
            self.simRobotTrailLine = plot(self.simRobotTrail(1,:), self.simRobotTrail(2,:), 'r', 'LineWidth', 2);
            
            % Draw the robot itself
            trianglePts = 0.07*[cos(2*pi/3*(0:2));sin(2*pi/3*(0:2))];
            trianglePts = self.rmat(self.robotAngle) * trianglePts + self.robotPosition;
            self.simRobotTriangle = fill(trianglePts(1,:), trianglePts(2,:), 'y');
            
            % Draw landmarks
            self.simLandmarkMarkers = plot(self.worldLandmarkPositions(1,:), self.worldLandmarkPositions(2,:), 'ks', ...
                'MarkerSize',20, 'MarkerFaceColor','w');
            markerLabels = 1:self.worldLandmarkNumber;
            self.simLandmarkLabels = text(self.worldLandmarkPositions(1,:), self.worldLandmarkPositions(2,:), string(markerLabels), ...
                'HorizontalAlignment', 'center');
            
            % Draw robot view range
            self.simRobotViewCapture = plot(0,0, 'g');
            
            % Set axes appropriately
            axis equal;
            xlim(self.worldBoundaries(1,:));
            ylim(self.worldBoundaries(2,:));            
            
            ax = gca();
            ax.YDir = 'normal';
            
            if ~currentHold
                hold off
            end
            self.updateDrawWorld();
        end
        
        function updateDrawWorld(self)
            % Update the drawing objects.
            self.simRobotTrailLine.XData = self.simRobotTrail(1,:);
            self.simRobotTrailLine.YData = self.simRobotTrail(2,:);
            
            trianglePts = 0.07*[cos(2*pi/3*(0:2));sin(2*pi/3*(0:2))];
            trianglePts = self.rmat(self.robotAngle) * trianglePts + self.robotPosition;
            self.simRobotTriangle.Vertices = trianglePts';
            
            % Update the view range if enabled
            if self.simRobotViewCapture.Visible
                % Create the top arc followed by the bottom arc in reverse
                viewMaxArcPts = self.robotLandmarkDistMax * [cos(-self.robotLandmarkViewAngle:0.02:self.robotLandmarkViewAngle);
                                                             sin(-self.robotLandmarkViewAngle:0.02:self.robotLandmarkViewAngle)];
                viewMinArcPts = self.robotLandmarkDistMin * [cos(-self.robotLandmarkViewAngle:0.02:self.robotLandmarkViewAngle);
                                                             sin(-self.robotLandmarkViewAngle:0.02:self.robotLandmarkViewAngle)];
                viewArcPts = [viewMaxArcPts, viewMinArcPts(:, end:-1:1), viewMaxArcPts(:,1)];
                viewArcPts = self.rmat(self.robotAngle) * viewArcPts + self.robotPosition;
                self.simRobotViewCapture.XData = viewArcPts(1,:);
                self.simRobotViewCapture.YData = viewArcPts(2,:);
            end
            
            drawnow
        end
            
        
        function K = floorImageK(self)
            % Compute the camera matrix transform of the floor
            K = [size(self.floorImage,2)/piBotSim.worldBoundaries(1,2) 0 0;
                0 size(self.floorImage,1)/piBotSim.worldBoundaries(2,2) 0;
                0 0 1];
        end
        
    end
    
    % Static utility methods
    methods(Static, Hidden, Access=protected)
        function [v, omega] = forwardKinematics(wheels)
            % Compute the linear and angular velocity of the robot given
            % the wheel velocities.
            ul = wheels(1);
            ur = wheels(2);
            v = piBotSim.robotWheelVelScale * (ul + ur) * 0.5;
            omega = piBotSim.robotWheelVelScale * (ur - ul ) / piBotSim.robotWheelTrack;
        end
        
        function flag = positionInBounds(position)
            % Check that the given position lies within the world.
            flag = all(position >= piBotSim.worldBoundaries(:,1)) && all(position <= piBotSim.worldBoundaries(:,2));
        end
        
        function noise = wheelNoise(speed) 
            noise = randn(1,2) * piBotSim.robotWheelVelNoise * abs(speed);
        end
        
        function noisy_lm = applyARUCONoise(lm)
            % Apply realistic noise to a landmark measurement
            % Compute projected square sides
            squareprojL = (lm(2) - piBotSim.worldARUCOSize/2) / lm(1);
            squareprojR = (lm(2) + piBotSim.worldARUCOSize/2) / lm(1);
            % Add some noise
            squareprojL = squareprojL + randn()*piBotSim.worldARUCONoise;
            squareprojR = squareprojR + randn()*piBotSim.worldARUCONoise;
            % Compute the landmark from this
            lmproj = (squareprojL + squareprojR)/2;
            lm1 = piBotSim.worldARUCOSize / (squareprojR - squareprojL);
            % Make sure depth is in range
            if lm1 < 0 || lm1 > 10
                lm1 = 10;
            end
            
            noisy_lm = [lm1 ; lmproj * lm1];
        end
        
        function R = rmat(th)
            R = [cos(th), -sin(th); sin(th), cos(th)];
        end
        
        function R = rotz(th)
            R = [piBotSim.rmat(th), [0;0]; 0,0,1];
        end
        
        function points = ensureMinimumDist(points, d)
            % Ensure all points are at least d apart.
            % Otherwise, regenerate
            n = size(points,2);
            d2 = d*d;
            max_reps = 50;
            for i = 1:(n-1)
                for rep = 1:max_reps
                    % Check the distances
                    diffs = points(:,(i+1):end) - points(:,i);
                    distsSquared = diffs(1,:).^2 + diffs(2,:).^2;
                    if any(distsSquared < d2)
                        % Resample
                        points(:,i) = rand(2,1) .* piBotSim.worldBoundaries(:,2);
                    else
                        break
                    end
                end
                assert(rep ~= max_reps, "Maximum repetitions reached in distributing landmarks.");
            end
        end
        
    end
end

