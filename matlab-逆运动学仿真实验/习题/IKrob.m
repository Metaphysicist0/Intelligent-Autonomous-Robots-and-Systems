%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : IKrob.m
% @function : [theta] = IKrob(coord,l)
% brief : 二轴机械臂逆运动学求解函数
% data  : 2021.11.1 
% version : 1.0
% input : coord ------------- 笛卡尔空间坐标
%         l     ------------- 连杆长度
% output: theta ------------- 机械臂关节角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [theta] = IKrob(coord,l)

%{
已知二轴机械臂正运动学
    x = l_1 * c1 + l_2 * c12 
    y = l_1 * s1 + l_2 * s12

求theta1及theta2
    theta2 = ？
    theta1 = ？
    
注意：笛卡尔空间中的一个点可能对应关节空间中的两组解
%}
%%temp_theta2 = abs(acos(((coord(1)^2+coord(2)^2)/(2*(l(1)^2)))-1)); 
%phi=abs(acos(sqrt(coord(1)^2+coord(2)^2)/(2*l(1))));
%belta=abs(atan(coord(2)/coord(1)));
%theta1=belta-phi;
%theta2=belta+phi;

x = coord(1);
y = coord(2);
L1 = l(1);

fai = abs(acos(sqrt(x.^2+y.^2)/(2*L1)));
beta = abs(atan(y./x));
theta2 = acos((x.^2+y.^2)/(2*L1^2)-1);
if(x >= 0 && y >= 0)
   theta1 =  pi/2 - (beta(1) + fai(1));    
elseif(x < 0 && y >= 0)
   theta1 = - pi/2 + (beta(1) - fai(1));
elseif(x < 0 && y < 0)
   theta1 = -pi/2 - (beta(1) - fai(1));
elseif(x >= 0 && y < 0)
   theta1 =  pi/2 + (beta(1) - fai(1));
end
theta = [theta1 theta2];
end

