%%
% This m-file creates the trajectory from the home position to second bowl
% with rotation. This code calculates the trajectory both in task space and
% joint space. You can find some figure and plot options on the code to
% compare joint space and task space differences. Calculatatin of the
% trajectory in task space could take 2-3 mins.
% Writer: Mustafa Ugur 25th of Decmeber, 2019
%% LOAD AND DISPLAY ROBOT
clear
clc

% Load the robot and change it's DataFormat
robot = importrobot('../../MATLAB_feeding_robot_arm20.urdf');
robot.DataFormat = 'row';

% Assign the home position to show the robot properly
jointAnglesHome = [0;0;0;0];

%Create a set of desired wayPoints
waypoints = [0.29014 -0.02273 0.2457;
             0.37 -0.023 0.050]; 
        
waypoints = transpose(waypoints);

numWaypoints = size(waypoints,2);
numJoints = numel(robot.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);

% Array of waypoint times
waypointTimes = 0:4:4;

% Use a small sample time for this example, so the difference between joint
% and task space is clear due to evaluation of IK in task space trajectory.
ts = 0.02;
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
%% JOINT SPACE
tic
for idx = 1:numWaypoints
    tgtPose = trvec2tform(waypoints(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;   
    jointWaypoints(:,idx) = config';
end

% Trajectory Generation
[qJoint,qdJoint,qddJoint] = trapveltraj(jointWaypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
    'EndTime',repmat(diff(waypointTimes),[numJoints 1]));

% Trajectory evaluation (only needed to find end effector position)
for idx = 1:numel(trajTimes)  
    eeTform = getTransform(robot,qJoint(:,idx)',eeName); 
    posJoint(:,idx) = tform2trvec(eeTform)'; 
end

taskTime = toc;
disp(['Joint space trajectory time : ' num2str(taskTime) ' s']);

%% TASK SPACE
tic
% Trajectory generation
[posTask,velTask,accelTask] = trapveltraj(waypoints,numel(trajTimes), ...
    'AccelTime',repmat(waypointAccelTimes,[3 1]), ...
    'EndTime',repmat(diff(waypointTimes),[3 1]));

% Trajectory evaluation
for idx = 1:numel(trajTimes) 
    % Solve IK
    tgtPose = trvec2tform(posTask(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    qTask(:,idx) = config;
end

taskTime = toc;
disp(['Task space trajectory time : ' num2str(taskTime) ' s']);

%% SHOW FULL TRAJECTORY
if plotMode == 1
    set(hTraj,'xdata',posTask(1,:),'ydata',posTask(2,:),'zdata',posTask(3,:));
elseif plotMode == 2
    plotTransforms(posTask',repmat([1 0 0 0],[size(posTask,2) 1]),'FrameSize',0.05);
end

%% COMPARE JOINT POSITIONS
% Compare trajectories in Cartesian space
plot3(posTask(1,:),posTask(2,:),posTask(3,:),'b-');
plot3(posJoint(1,:),posJoint(2,:),posJoint(3,:),'r--');
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ko','LineWidth',2);
title('Trajectory Following'); 
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('Task Space Trajectory','Joint Space Trajectory','Waypoints');
grid on
view([45 45]);

%%
for idx = 1:numel(trajTimes) 
   % Show the robot
    show(robot,qJoint(:,idx)','Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow    
end

%%
% Compare joint angles
% Plot each joint trajectory
for idx = 1:numJoints
    figure(idx+1), hold on;
    plot(trajTimes,qTask(idx,:),'b-');
    plot(trajTimes,qJoint(idx,:),'r-');
    grid on
    for wIdx = 1:numWaypoints
       xline(waypointTimes(wIdx),'k--'); 
    end
    title(['Joint ' num2str(idx) ' Trajectory']); 
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
    legend('Task Space Trajectory','Joint Space Trajectory', 'Location', 'southeast');
end

%% RUN THIS PART TO PLOT JOINT VELOCITIES PRODUCED IN JOINT SPACE
figure
subplot(2,2,1);
title('Angular Velocity of Joint 1');
hold on
plot(trajTimes, qdJoint(1,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Angular Velocity [rad/s]');
hold off

subplot(2,2,2);
title('Angular Velocity of Joint 2');
hold on
plot(trajTimes, qdJoint(2,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Angular Velocity [rad/s]');
hold off

subplot(2,2,3);
title('Angular Velocity of Joint 3');
hold on
plot(trajTimes, qdJoint(3,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Angular Velocity [rad/s]');
hold off

subplot(2,2,4);
title('Angular Velocity of Joint 4');
hold on
plot(trajTimes, qdJoint(4,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Angular Velocity [rad/s]');
hold off

%% RUN THIS PART TO PLOT JOINT ACCELERATIONS PRODUCED IN JOINT SPACE
figure
subplot(2,2,1);
title('Angular Acceleration of Joint 1');
hold on
plot(trajTimes, qddJoint(1,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Angular Acceleration [rad/s^2]');
hold off

subplot(2,2,2);
title('Angular Acceleration of Joint 2');
hold on
plot(trajTimes, qddJoint(2,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Angular Acceleration [rad/s^2]');
hold off

subplot(2,2,3);
title('Angular Acceleration of Joint 3');
hold on
plot(trajTimes, qddJoint(3,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Angular Acceleration [rad/s^2]');
hold off

subplot(2,2,4);
title('Angular Acceleration of Joint 4');
hold on
plot(trajTimes, qddJoint(4,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Angular Acceleration [rad/s^2]');
hold off

%% RUN THIS PART TO PLOT LINEAR VELOCITIES IN ALL DIRECTIONS 
subplot(3,1,1);
title('Linear Velocity in x-direction');
hold on
plot(trajTimes, velTask(1,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Velocity [m/s]');
hold off

subplot(3,1,2);
title('Linear Velocity in y-direction');
hold on
plot(trajTimes, velTask(2,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Velocity [m/s]');
hold off

subplot(3,1,3);
title('Linear Velocity in z-direction');
hold on
plot(trajTimes, velTask(3,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Velocity [m/s]');
hold off

%% RUN THIS PART TO PLOT LINEAR ACCELERATIONS IN ALL DIRECTIONS 
subplot(3,1,1);
title('Linear Acceleration in x-direction');
hold on
plot(trajTimes, accelTask(1,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Acceleration [m/s^2]');
hold off

subplot(3,1,2);
title('Linear Acceleration in y-direction');
hold on
plot(trajTimes, accelTask(2,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Acceleration [m/s^2]');
hold off

subplot(3,1,3);
title('Linear Acceleration in z-direction');
hold on
plot(trajTimes, accelTask(3,:), 'r-');
grid on
xlabel('Time[s]');
ylabel('Acceleration [m/s^2]');
hold off