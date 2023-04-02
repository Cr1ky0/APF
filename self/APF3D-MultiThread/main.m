clear
clc
close

%% 起始点位置

% 出发点位置
Start =[
    0 0 0;
    1 1 1;
    ];
% 终点位置
Des = [
    10 10 7;
    5 5 5;
    ];

n1 = 3;                                    % 球体障碍物个数
n2 = 3;                                    % 圆柱体障碍物个数
n3 = 3;                                    % 圆锥体障碍物个数
n = n1 + n2 + n3;
%% 超参数设置
Kaat = 1;                     % 引力尺度因子
Krep = 20;                     % 斥力尺度因子
P0 = 2;                        % 斥力作用范围
StepRate = 0.2;                % 步长
max_turn_angle = pi/8;          % 最大转向角
max_pitch_angle = pi/8;         % 最大俯仰角
max_step = 0.2;                    % 最大单段步长
power = 200;                   % 最大能量（可前进的总路程量）
Epoch = power/max_step;         % 最大迭代次数

%% 改进版方程定义
m = 0.5; % 改进方程参数定义
[f_attx,f_atty,f_attz,f_repx,f_repy,f_repz] = EquationDefinition(Des(1,1),Des(1,2),Des(1,3),m,Kaat,Krep,P0);

%% 画图
figure(1)
plot3(Start(1,1),Start(1,2),Start(1,3),'.','MarkerSize',4,'color','black');
view(45,30);
hold on
plot3(Des(1,1),Des(1,2),Des(1,3),'o','MarkerSize',10,'Color','yellow');
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
last_xyz = [Start(1,1),Start(1,2),Start(1,3)];
path_points = zeros(Epoch,3);
path_points(1,:) = Start(1,:);
while(1)
    % 后续改进基于ADS-B可以获得无人机的飞行角度，避免航路交叉发生碰撞
    [Start(1,1),Start(1,2),Start(1,3),last_Fxy,last_xyz,isDone,CountFlag] = MultiThread(CountFlag, [Start(1,1),Start(1,2),Start(1,3)],[Des(1,1),Des(1,2),Des(1,3)] ,...
        n,[n1,n2,n3], ...
        P0,StepRate,max_step,max_turn_angle,max_pitch_angle,Epoch, ...
        f_attx,f_atty,f_attz,f_repx,f_repy,f_repz, ...
        Sphere_Obs,Cylinder_Obs,Cone_Obs, ...
        last_xyz,last_Fxy, ...
        'black');
    path_points(CountFlag+1,:) = Start(1,:);
    if(isDone)
        break;
    end
end
% 曲线平滑（三次B样条）
[new_x,new_y,new_z] = ThreeBSpline(path_points(1:CountFlag+1,1),path_points(1:CountFlag+1,2),path_points(1:CountFlag+1,3));
plot3(new_x,new_y,new_z,'r');




