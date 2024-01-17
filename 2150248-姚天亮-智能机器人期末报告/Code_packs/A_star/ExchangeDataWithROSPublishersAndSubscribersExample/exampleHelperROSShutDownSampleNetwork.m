%exampleHelperROSShutDownSampleNetwork Shut down the example ROS network
%   This script shuts down the nodes, publishers, subscribers, and services
%   created by exampleHelperROSCreateSampleNetwork. 
%
%   See also exampleHelperROSCreateSampleNetwork

%   Copyright 2014-2015 The MathWorks, Inc.

% Clear timer object first
clear('simTimer');

% Shut down publishers and subscribers of the /scan topic
clear('scanPub');
clear('scanPubmsg');
clear('scanSub1');
clear('scanSub2');

% Shut down publishers and subscribers of the /pose topic
clear('twistPub');
clear('twistPubmsg');
clear('twistSub');

% Shut down the service servers
clear('srv1');
clear('srv2');

% Finally, shut down the three nodes
clear('node_1');
clear('node_2');
clear('node_3');

% Delete data variables
clear('tf');
clear('timerHandles');
clear('masterHost');
