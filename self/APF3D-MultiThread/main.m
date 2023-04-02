clear 
clc
close

timer = tic;
%% 起始点位置
StartX = 0;                                    % 出发点位置
StartY = 0;
StartZ = 0;
DesX = 10;                                 % 终点位置
DesY = 10;
DesZ = 10;
n1 = 5;                                    % 球体障碍物个数
n2 = 5;                                    % 圆柱体障碍物个数
n3 = 5;                                    % 圆锥体障碍物个数
n = n1 + n2 + n3;                          
%% 超参数设置
Kaat = 1;                     % 引力尺度因子
Krep = 15;                     % 斥力尺度因子
P0 = 2;                        % 斥力作用范围
StepRate = 0.02;                % 步长
max_turn_angle = pi/6;          % 最大转向角
max_pitch_angle = pi/6;         % 最大俯仰角
max_step = 0.1;                    % 最大单段步长
power = 200;                   % 最大能量（可前进的总路程量）
Epoch = power/max_step;         % 最大迭代次数

%% 改进版方程定义
m = 0.5; % 改进方程参数定义
[f_attx,f_atty,f_attz,f_repx,f_repy,f_repz] = EquationDefinition(DesX,DesY,DesZ,m,Kaat,Krep,P0);

%% 画图
figure(1)
plot3(StartX,StartY,StartZ,'.','MarkerSize',4,'color','black');
view(45,30);
hold on
plot3(DesX,DesY,DesZ,'o','MarkerSize',10,'Color','yellow');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
hold on
% 坐标
[Sphere_Obs,Cylinder_Obs,Cone_Obs] = PlotObs(n1,n2,n3);
%% 计算
CountFlag = 0;
last_Fxy = [0,0];
last_xyz = [StartX,StartY,StartZ];
while(1)
    % 后续改进基于ADS-B可以获得无人机的飞行角度，避免航路交叉发生碰撞
   [StartX,StartY,StartZ,last_Fxy,last_xyz,isDone] = MultiThread(CountFlag, [StartX,StartY,StartZ],[DesX,DesY,DesZ] ,...
       n,[n1,n2,n3], ...
       P0,StepRate,max_step,max_turn_angle,max_pitch_angle,Epoch, ...
       f_attx,f_atty,f_attz,f_repx,f_repy,f_repz, ...
       Sphere_Obs,Cylinder_Obs,Cone_Obs, ...
       last_xyz,last_Fxy, ...
       'black');
   if(isDone)
       break;
   end
end


toc(timer);
display(CountFlag);


