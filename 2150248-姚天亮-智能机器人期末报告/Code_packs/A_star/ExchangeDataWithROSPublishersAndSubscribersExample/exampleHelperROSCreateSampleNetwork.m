%exampleHelperROSCreateSampleNetwork Create an example ROS network
%   This network is representative of a realistic ROS network and is used 
%   throughout the MATLAB ROS examples. It features three nodes,
%   two publishers, three subscribers, and two services. Additionally we use a
%   timer to control publishing of ROS messages over the network.
%
%   See also exampleHelperROSShutDownSampleNetwork, exampleHelperROSSimTimer,
%   exampleHelperROSLoadRanges

%   Copyright 2014-2021 The MathWorks, Inc.

% Initialize three nodes (in addition to the global MATLAB node that is 
% created through rosinit). Note that the nodes will try to connect to the
% ROS master at 'localhost'. If you are connecting to an external master,
% you will have to use its IP address or hostname.
masterHost = 'localhost';
node_1 = ros.Node('node_1', masterHost);
node_2 = ros.Node('node_2', masterHost);
node_3 = ros.Node('node_3', masterHost);

% Example data from pre-existing messages
messageData = load('specialROSMessageData.mat','scan','tf');

% Create a publisher and subscriber for the '/pose' topic
twistPub = ros.Publisher(node_1,'/pose','geometry_msgs/Twist',...
    'DataFormat','struct');
twistPubmsg = rosmessage(twistPub);
twistSub = ros.Subscriber(node_2,'/pose','geometry_msgs/Twist',...
    'DataFormat','struct');

% Create publishers and subscribers for the '/scan' topic
scanPub = ros.Publisher(node_3,'/scan','sensor_msgs/LaserScan',...
    'DataFormat','struct');
scanSub1 = ros.Subscriber(node_1,'/scan','sensor_msgs/LaserScan',...
    'DataFormat','struct');
scanSub2 = ros.Subscriber(node_2,'/scan','sensor_msgs/LaserScan',...
    'DataFormat','struct');

% Create two service servers for the '/add' and '/reply' services
srv1 = ros.ServiceServer(node_3,'/add','roscpp_tutorials/TwoInts',...
    'DataFormat','struct');
srv2 = ros.ServiceServer(node_3,'/reply','std_srvs/Empty',...
    @exampleHelperROSEmptyCallback,'DataFormat','struct');

% Load sample data for inspecting messages
tf = messageData.tf;

% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of 10 Hz.
timerHandles.twistPub = twistPub;
timerHandles.twistPubmsg = twistPubmsg;
timerHandles.scanPub = scanPub;
timerHandles.scanPubmsg = messageData.scan;
simTimer = ExampleHelperROSTimer(0.1, {@exampleHelperROSSimTimer,timerHandles});

clear messageData
