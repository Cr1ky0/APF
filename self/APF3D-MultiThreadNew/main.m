clear
clc
close
%% 起始点位置
% 出发点位置
Start =[
    0 0 0;
    0 2 10;
    2 0 0;
    10 0 2;
    % 1 1 1;
    ];
n_uav = length(Start(:,1));
% 终点位置
Des = [
    10 10 7;
    7 10 10;
    10 7 10;
    ];

n1 = 5;                                    % 球体障碍物个数
n2 = 5;                                    % 圆柱体障碍物个数
n3 = 5;                                    % 圆锥体障碍物个数
n = n1 + n2 + n3;
%% 超参数设置
Kaat = 1;                     % 引力尺度因子
Krep = 20;                     % 斥力尺度因子
P0 = 2;                        % 斥力作用范围
StepRate = 0.02;                % 步长
% 俯仰角转向角约束
max_turn_angle = pi/6;          % 最大转向角
max_pitch_angle = pi/6;         % 最大俯仰角
max_step = 0.02;                 % 最大单段步长
% 能量约束
power = 200;                   % 最大能量（可前进的总路程量）
Epoch = floor(power/max_step);         % 最大迭代次数

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
% 障碍物坐标以及障碍物绘制
[Sphere_Obs,Cylinder_Obs,Cone_Obs] = PlotObs(n1,n2,n3);

%% 单例计算部分
% 多线程
points = {};
bspline_points = {};
parfor i = 1 : n_uav
    [path_points] = MultiThread([Start(i,1),Start(i,2),Start(i,3)],[Des(1,1),Des(1,2),Des(1,3)], ...
        n,[n1,n2,n3],...
        P0,StepRate,max_step,max_turn_angle,max_pitch_angle,Epoch,...
        f_attx,f_atty,f_attz,f_repx,f_repy,f_repz, ...
        Sphere_Obs,Cylinder_Obs,Cone_Obs, ...
        i);
    points{i} = path_points;

    % 曲线平滑（三次B样条）
    % 先进行3次数据剔除，在进行插值，防止震荡并且平滑曲线
    [xi,yi,zi] = FilterList(path_points);
    [xi,yi,zi] = FilterList([xi,yi,zi]);
    [xi,yi,zi] = FilterList([xi,yi,zi]);
    [x,y,z] = N_BSpline(xi,yi,zi,3);
    bspline_points{i} = [x(1:end-1)',y(1:end-1)',z(1:end-1)']; % 最后一个0去除
end

for i = 1 : n_uav
    hold on;
    cur = points{i};
    cur_bspline = bspline_points{i};
    plot3(cur(:,1),cur(:,2),cur(:,3),'r','Color','black');
    plot3(cur_bspline(:,1),cur_bspline(:,2),cur_bspline(:,3),'r','Color','red');
end

% [x,y,z,path_points] = MultiThread([Start(1,1),Start(1,2),Start(1,3)],[Des(1,1),Des(1,2),Des(1,3)], ...
%         n,[n1,n2,n3],...
%         P0,StepRate,max_step,max_turn_angle,max_pitch_angle,Epoch,...
%         f_attx,f_atty,f_attz,f_repx,f_repy,f_repz, ...
%         Sphere_Obs,Cylinder_Obs,Cone_Obs, ...
%         1);
% plot3(x,y,z,'r');






