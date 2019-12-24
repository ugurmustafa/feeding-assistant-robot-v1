%%
% This m-file creates the trajectory from the home position to first bowl
% with rotation. This code calculates the trajectory in task space.
% Writer: Mustafa Ugur 25th of Decmeber, 2019

%% LOAD AND DISPLAY ROBOT
% Load the robot and change it's DataFormat
robot = importrobot('../../MATLAB_feeding_robot_arm20.urdf');
robot.DataFormat = 'row';

% Assign the home position to show the robot properly
jointAnglesHome = [0;0;0;0];

%Create a set of desired wayPoints
waypoints = [0.29014 -0.02273 0.2457;
             0.35 0.068 0.050]; 
        
waypoints = transpose(waypoints);

numWaypoints = size(waypoints,2);
numJoints = numel(robot.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);

% Array of waypoint times
waypointTimes = 0:4:4;

% Use a small sample time for this example, so the difference between joint
% and task space is clear due to evaluation of IK in task space trajectory.
ts = 0.1;
trajTimes = 0:ts:waypointTimes(end);

% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/2;

% Initialize matrices for plots
qTask = zeros(numJoints,numel(trajTimes)); % Derived joint angles in task space trajectory
posJoint = zeros(3,numel(trajTimes)); % Derived end effector positions in joint space trajectory

%% SET UP PLOT
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(robot,jointAnglesHome','Frames','off','PreservePlot',false);
xlim([-0.15 0.5]), ylim([-0.3 0.3]), zlim([-0.1 0.4]) % Just for getting better figures
hold on

if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b--', 'LineWidth', 2);
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% ADDING END-EFFECTOR FRAME TO RIGIB BODY
eeOffset = 0.1433;
eeBody = robotics.RigidBody('end_effector1');
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
addBody(robot,eeBody,'fourth_link');
eeName = 'end_effector1';
%% INVERSE KINEMATIC OBJECT
ik = robotics.InverseKinematics('RigidBodyTree',robot);
ikWeights = [0.1 0.1 0 1 1 1];
ikInitGuess = jointAnglesHome';

%% TASK SPACE
tic
% Euler Angles (Z Y X) relative to the home orientation       
orientations = [0     0    0;
                pi/12  0    0]';
            
R0 = eul2quat(orientations(:,1)');
Rf = eul2quat(orientations(:,2)');
timeInterval = waypointTimes(1:2);

% Find the quaternions from trajectory generation
[R, omega, alpha] = rottraj(R0, Rf, timeInterval, trajTimes);

% Trajectory generation
[posTask,velTask,accelTask] = trapveltraj(waypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[3 1]), ...
    'EndTime',repmat(diff(waypointTimes),[3 1]));

plotTransforms(posTask',R','FrameSize',0.05)

% Trajectory evaluation
for idx = 1:numel(trajTimes) 
    % Solve IK
    tgtPose = trvec2tform(posTask(:,idx)') * quat2tform(R(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    qTask(:,idx) = config;
end

taskTime = toc;
disp(['Task space trajectory time : ' num2str(taskTime) ' s']);

%% SHOW THE TRAJECTORY WITH END-EFFECTOR ORIENTATION
for idx = 1:numel(trajTimes) 
    show(robot,qTask(:,idx)','Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow    
end

%% PLOT THE JOINT POSITIONS FOR EACH JOINT
for idx = 1:numJoints
    figure(idx+1), hold on;
    plot(trajTimes,qTask(idx,:),'b-');
    grid on
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' Trajectory']); 
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
end

