function main()
    clear all; close all; clc;

    % 设置 ROS 环境
    setenv('ROS_IP','10.1.1.104');
    setenv('ROS_MASTER_URI', 'http://10.1.1.4:11311');
    rosinit('NodeName', 'stabilization_demo_2333');

    % 获取起点、终点和地图信息
    xStart = 1.0; yStart = 1.0;
    xTarget = 10.0; yTarget = 10.0;
    MAX_X = 10; MAX_Y = 10;
    map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
    path = A_star_search(map, MAX_X, MAX_Y);

    % 初始化机器人位置和速度发布器
    [odom_sub, vel_pub, x, y, theta] = initRobot();

    % 设置控制参数
    v_max = 0.6;
    d = 0.05;
    err = 0.001;
    dis_err = 0.01;
    
    % 控制循环
    [i, ~] = size(path);
    while i > 1
        disp("Hello!");

        % 更新机器人位置
        [x, y, theta] = updateRobotPose(odom_sub, x, y, theta);

        % 计算目标位置和角度
        x_goal = path(i, 1) - 1;
        y_goal = path(i, 2) - 1;

        % 控制机器人移动
        controlRobot(x, y, theta, x_goal, y_goal, v_max, d, err, dis_err, vel_pub, odom_sub);

        % 更新迭代
        i = i - 1;
    end

    % 关闭 ROS
    rosshutdown;
end

function [odom_sub, vel_pub, x, y, theta] = initRobot()
    % 订阅 odom 主题
    odom_sub = rossubscriber('/Qbot2e_121/odom', 'DataFormat', 'struct');
    msg = receive(odom_sub, 10);
    x = msg.Pose.Pose.Position.X;
    y = msg.Pose.Pose.Position.Y;

    % 创建速度发布器
    vel_pub = rospublisher('/Qbot2e_121/mobile_base/commands/velocity', 'DataFormat', 'struct');
end

function [x, y, theta] = updateRobotPose(odom_sub, x, y, theta)
    % 接收 odom 数据
    msg = receive(odom_sub, 10);
    x = msg.Pose.Pose.Position.X;
    y = msg.Pose.Pose.Position.Y;
    quat = msg.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = angles(1);
end

function controlRobot(x, y, theta, x_goal, y_goal, v_max, d, err, dis_err, vel_pub, odom_sub)
    while abs(x - x_goal) > dis_err && abs(y - y_goal) > dis_err
        dx = x_goal - x;
        dy = y_goal - y;
        alpha = atan2(dy, dx + err) - theta;
        alpha = atan2(sin(alpha), cos(alpha) + err);

        % 计算线速度和角速度
        linear_vel = v_max * cos(alpha);
        angular_vel = v_max * sin(alpha) / d;

        % 发布速度命令
        sendVelocityCommand(vel_pub, linear_vel, angular_vel);

        % 更新机器人位置
        [x, y, theta] = updateRobotPose(odom_sub, x, y, theta);
    end
end

function sendVelocityCommand(vel_pub, linear_vel, angular_vel)
    % 发布速度命令
    vel = rosmessage(vel_pub);
    vel.Linear.X = linear_vel;
    vel.Angular.Z = angular_vel;
    send(vel_pub, vel);
end
