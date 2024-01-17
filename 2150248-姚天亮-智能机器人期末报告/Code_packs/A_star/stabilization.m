clear all; close all;  clc;
global x
global y
global theta

err=0.001;
dis_err = 0.1;
dis_theta = 0.01;
R=0.5; %轨迹半径
omiga=pi/2;%角频率
v_max=0.6; %最大速度
d=0.05;

% set(gcf, 'Renderer', 'painters');
% set(gcf, 'Position', [500, 50, 700, 700]);

% Environment map in 2D space
xStart = 1.0;
yStart = 1.0;
xTarget = 10.0;
yTarget = 10.0;
MAX_X = 10;
MAX_Y = 10;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

% Waypoint Generator Using the A*
path = A_star_search(map, MAX_X, MAX_Y);




% clear; close all; clc;

%setenv('ROS_MASTER_URI','http://localhost:11311'); % Replace with actual ROS_MASTER_URI
setenv('ROS_IP','10.1.1.104'); % Replace with your IP address

% Initialize ROS node
node_name = 'stabilization_demo_233333';
setenv('ROS_MASTER_URI', 'http://10.1.1.4:11311')
rosinit('NodeName', node_name);
x=0;
y=0;
theta=0;
% Create subscriber for odom topic
odom_sub = rossubscriber('/Qbot2e_121/odom', 'DataFormat', 'struct');
msg2 = receive(odom_sub, 10); % 接收里程计消息，最长等待时间为 10 秒
x = msg2.Pose.Pose.Position.X;
y = msg2.Pose.Pose.Position.Y;
quat = msg2.Pose.Pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
theta = angles(1);
x_flag = x;
y_flag=y;
theta_flag=theta;
% Create publisher for velocity commands
vel_pub = rospublisher('/Qbot2e_121/mobile_base/commands/velocity', 'DataFormat', 'struct');
vel = rosmessage(vel_pub);

% Set desired velocity values
% vel.Linear.X = 0.1; % Forward velocity
% vel.Angular.Z = 0.5; % Angular velocity

rate = robotics.Rate(10); % Rate of publishing velocity commands (10 Hz)
[i,le]=size(path);
while i > 1 % Publish velocity commands for 10 seconds
    disp("Hello!");
    % Send velocity command
    send(vel_pub, vel);

    % Wait for next iteration
    %rate.sleep();
    x_goal = path(i,1)-1;
    y_goal = path(i,2)-1;
    % theta_mflag=theta - theta_flag;
    % alpha=atan2(dy,dx+err);
    % alpha=atan2(sin(alpha),cos(alpha)+err);
    % while abs(theta-theta_mflag-alpha)<dis_theta
    %     vel.Linear.X=0;
    %     vel.Angular.Z=0.4;
    %     send(vel_pub, vel);
    %     msg2 = receive(odom_sub, 10); % 接收里程计消息，最长等待时间为 10 秒
    %     x = msg2.Pose.Pose.Position.X;
    %     y = msg2.Pose.Pose.Position.Y;
    %     quat = msg2.Pose.Pose.Orientation;
    %     angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    %     theta = angles(1);
    % end
    while abs(x-x_flag - x_goal) > dis_err || abs(y-y_flag - y_goal) > dis_err
        x-x_flag
        y-y_flag
        theta-theta_flag
        dx=x_goal-x+x_flag;
        dy=y_goal-y+y_flag;
        alpha=atan2(dy,dx+err)-theta;
        alpha=atan2(sin(alpha),cos(alpha)+err);
        % vel.Linear.X=0.1;
        % vel.Angular.Z=0.1;
        vel.Linear.X = v_max*cos(alpha);
        vel.Angular.Z = v_max*sin(alpha)/d;
        % vel.Linear.X=0.6;
        % vel.Angular.Z=0;
        send(vel_pub, vel);
        msg2 = receive(odom_sub, 10); % 接收里程计消息，最长等待时间为 10 秒
        x = msg2.Pose.Pose.Position.X;
        y = msg2.Pose.Pose.Position.Y;
        quat = msg2.Pose.Pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        theta = angles(1);

    end
    i=i-1;
end

% Stop the robot by sending zero velocity command

% vel.Linear.X = 0;
% vel.Angular.Z = 0;
% send(vel_pub, vel);

% Shutdown ROS
rosshutdown;

% Callback function for odom topic subscriber
% function odomCallback(src, msg)
%     % Process odom data
%     pose = msg.Pose.Pose;
%     position = pose.Position;
%     orientation = pose.Orientation;
%
%     disp(['Position: X=',num2str(position.X),' Y=',num2str(position.Y),' Z=',num2str(position.Z)]);
%     disp(['Orientation: X=',num2str(orientation.X),' Y=',num2str(orientation.Y),' Z=',num2str(orientation.Z),' W=',num2str(orientation.W)]);
% end