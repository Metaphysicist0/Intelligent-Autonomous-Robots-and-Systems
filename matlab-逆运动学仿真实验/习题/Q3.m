%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : Q3.m
% brief : 二轴机械臂逆运动学雅可比矩阵求解
% data  : 2021.11.1 
% version : 1.0
% note  : 需要解决的问题
%          ③ 给定雅可比空间中的一条轨迹(手写字母a)，根据逆雅可比矩阵求解关节空间中的轨迹（填写Jacobi.m），
%             并作出机械臂运动图（需要求关节1的位置,具体效果见Robotarm.avi，动画中杆长与下面定义不同）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all   % 删除工作区变量
close all   % 关闭所有图

%% 二轴机械臂连杆长度定义
    l = [1.1 1.1];
    
%% Q3 逆雅可比矩阵求关节空间轨迹
% 读取轨迹信息
   load a1.mat                                          % 轨迹数据对应名称为saveddata，包含坐标数据x,y等，轨迹为手写字母a的轨迹
   trajactory_length = size(saveddata.x,2);             % 读取轨迹长度
   trajcoord = [saveddata.x',saveddata.y'];             % 读取轨迹坐标
   trajspeed = [saveddata.vx',saveddata.vy'];           % 读取轨迹速度 
   dt = saveddata.times(2) - saveddata.times(1);        % 读取时间间隔
   trajcoord(:,1) = trajcoord(:,1) - 1;                 % 改变一下轨迹的位置，方便机械臂运动

%% 逆雅可比矩阵求解（可以先用逆运动学求机械臂初始姿态）<----------------
        
   thetaB(1,:) = IKrob(trajcoord(1,:), l);
   midtrajB(1,:) = [l(1) * sin(thetaB(1)),l(1) * cos(thetaB(1))];
   for i=1:trajactory_length
       JacobiMatrix=Jacobi(thetaB(i,:),l);
       dtheta=JacobiMatrix\trajspeed(i,:)';
       thetaB(i+1,:)=thetaB(i,:)+dtheta'*dt;
       midtrajB(i+1,:)=[l(1)*sin(thetaB(i+1,1)) l(1)*cos(thetaB(i+1,1))];
   end  
    
%% 画图
% 生成动画
    figure
    Robotarm = VideoWriter('Robotarm.avi');                     % 新建叫Robotarm.avi的文件
    open(Robotarm);                                             % 打开Robotarm.avi的文件

    axis([-2 0.5 -1.8 1.5])                                     % 固定坐标轴
    hold on
    plot(trajcoord(:,1),trajcoord(:,2),'r-','linewidth',2);     % 画出轨迹    

    M=moviein(trajactory_length);                               % 前面要有plot帮助moviein初始化
    
    for k=1:trajactory_length
        
        axis([-2 0.5 -1.8 1.5])
        h1 = line([0 midtrajB(k,1)],[0 midtrajB(k,2)],'LineWidth',3);                             % 画杆1
        h2 = line([midtrajB(k,1) trajcoord(k,1)],[midtrajB(k,2) trajcoord(k,2)],'LineWidth',3);   % 画杆2
        h3 = plot(midtrajB(k,1),midtrajB(k,2),'bo','LineWidth',6);                                % 画关节1

        M(:,k)=getframe;                                        % 抓取图形作为电影的画面
        writeVideo(Robotarm,M(:,k));
        delete(h1);
        delete(h2);
        delete(h3);
    end
    
%     movie(M,1,50);                                              % 以每秒30帧的速度播放1次
    close(Robotarm);                                            % 关闭