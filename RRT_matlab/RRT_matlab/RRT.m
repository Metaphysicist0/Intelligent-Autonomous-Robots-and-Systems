%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ���̳�ʼ��
clc
clear all; close all;
x_I=1; y_I=1; % ���ó�ʼ��
x_G=650; y_G=650; % ����Ŀ��㣨�ɳ����޸��յ㣩
Thr=50; % ����Ŀ�����ֵ����ʾ��ǰ�ڵ㵽��Ŀ��㷽Բ50�ڣ�����������Ŀ���
Delta= 30; % ������չ����
x_goal = [x_G y_G];
%% ������ʼ��
T.v(1).x = x_I; % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T.v(1).y = y_I;
T.v(1).xPrev = x_I; % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T.v(1).yPrev = y_I;
T.v(1).dist=0; % �Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T.v(1).indPrev = 0; %
%% ��ʼ����������ҵ����
% getDistance
getDist = @(x1,y1,x2,y2) sqrt((x1-x2)^2+(y1-y2)^2);
% end of getDistance
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);%��ͼx�᳤��
yL=size(Imp,1);%��ͼy�᳤��
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% ��������Ŀ���
count=1;
bFind = false; 
tic

for iter = 1:30000
    
%Step 1: �ڵ�ͼ���������һ����x_rand�������Ϊͼ�е�����
%��ʾ���ã�x_rand(1),x_rand(2)����ʾ�����в����������
x_rand=[0 + (xL - 0) * rand(1,1), 0 + (yL - 0) * rand(1,1) ];

%Step 2: ���������������ҵ�����ڽ���x_near
%��ʾ��x_near�Ѿ�����T��
distArr = arrayfun(@(dot) getDist(x_rand(1),x_rand(2),dot.x,dot.y),T.v);
[~,index] = min(distArr);
x_near=[T.v(index).x,T.v(index).y];
%Step 3: ��չ�õ�x_new�ڵ�
%��ʾ��ע��ʹ����չ����Delta
theta = atan2(x_rand(2)-x_near(2),x_rand(1)-x_near(1));
x_new=[x_near(1)+Delta*cos(theta), x_near(2) + Delta*sin(theta)];
%���ڵ��Ƿ���collision-free
if ~collisionChecking(x_near,x_new,Imp)
continue;
end
count=count+1;

%Step 4: ��x_new������T
%��ʾ���½ڵ�x_new�ĸ��ڵ���x_near
T.v(count).x = x_new(1); 
T.v(count).y = x_new(2);
T.v(count).xPrev = x_near(1);
T.v(count).yPrev = x_near(2);
T.v(count).dist=Delta; 
T.v(count).indPrev = index;
%Step 5:����Ƿ񵽴�Ŀ��㸽��
%��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
if getDist(x_new(1),x_new(2),x_G,y_G) < Thr
plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'r');
hold on;
bFind = true;
break;
end

%Step 6:��x_near��x_new֮���·��������
%��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
%��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�x_near��x_new֮���·��������
plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'r');
hold on;
pause(0.05); %��ͣһ�ᣬʹ��RRT��չ�������׹۲�
end
toc
%% ·���Ѿ��ҵ��������ѯ
if bFind
path.pos(1).x = x_G; path.pos(1).y = y_G;
path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
pathIndex = T.v(end).indPrev; % �յ����·��
j=0;
while 1
path.pos(j+3).x = T.v(pathIndex).x;
path.pos(j+3).y = T.v(pathIndex).y;
pathIndex = T.v(pathIndex).indPrev;
if pathIndex == 1
break
end
j=j+1;
end % ���յ���ݵ����
path.pos(end+1).x = x_I; path.pos(end).y = y_I; % ������·��
for j = 2:length(path.pos)
plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
end
else
disp('Error, no path found!');
end