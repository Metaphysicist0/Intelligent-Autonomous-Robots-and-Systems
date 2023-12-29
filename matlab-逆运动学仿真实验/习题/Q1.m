%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : Q1.m
% brief : 二轴机械臂逆运动学求解
% data  : 2021.11.1 
% version : 1.0
% note  : 需要解决的问题
%          ① 给定雅可比空间中的一个坐标点，根据机械臂逆运动学求解对应关节角（填写IKrob.m）       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all   % 删除工作区变量
close all   % 关闭所有图

%% 二轴机械臂连杆长度定义
    l = [0.8 0.8];

%% Q1 给定点的逆运动学
    dot = [0.5 1.5];        % 给出机械臂需要到达的点
    theta = IKrob(dot, l);  % 解出对应的关节角 <------------------
    
    figure                                                                 % 画图
    axis equal
    hold on
    plot(dot(1),dot(2),'r*','LineWidth',10);                               % 画目标点
    midtraj = [l(1) * cos(theta(1)),l(1) * sin(theta(1))];                 % 计算关节1的位置
    
    h1 = line([0 midtraj(1)],[0 midtraj(2)],'LineWidth',3);                % 画杆1
    h2 = line([midtraj(1) dot(1)],[midtraj(2) dot(2)],'LineWidth',3);      % 画杆2
    plot(midtraj(1),midtraj(2),'bo','LineWidth',6);                        % 画关节1
    
    midtraj = [l(1) * sin(theta(1)+theta(2)),l(1) * cos(theta(1)+theta(2))];     % 计算关节1的位置
    h1= line([0 midtraj(1,1)],[0 midtraj(1,2)],'LineWidth',3);                % 画杆1
    h2= line([midtraj(1,1) dot(1)],[midtraj(1,2) dot(2)],'LineWidth',3);      % 画杆2
    plot(midtraj(1,1),midtraj(1,2),'bo','LineWidth',6);                         % 画关节1
