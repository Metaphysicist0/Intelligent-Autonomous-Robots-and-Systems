clear; close all; clc;

% you shoule set ROS_MASTER_URI and ROS_IP in the environment value
node_name = 'stabilization_demo';
rosinit('NodeName', node_name);

% topics should have been registered
% subscriber
odom_sub = rossubscriber('/odom', @odomCallback, 'DataFormat', 'struct');

% publisher
vel_pub = rospublisher('/mobile_base/commands/velocity', 'DataFormat', 'struct');
vel = rosmessage(vel_pub);
send(vel_pub, vel);  % send the message
