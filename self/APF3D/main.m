clear 
clc
close
%% 起始点位置
MyX = 0;                                    % 出发点位置
MyY = 0;
MyZ = 0;
DesX = 10;                                 % 终点位置
DesY = 10;
DesZ = 10;
n = 100;                                    % 障碍物个数

% 自定义障碍物
% a1 = [10,12,10];                               % 障碍物
% a2 = [95,95,95];
% a3 = [35,25,35];
% a4 = [50,45,50];
% a5 = [60,50,60];
% a6 = [85,70,85];
% a7 = [60,30,60];
% a8 = [90,50,90];
% a9 = [65,60,65];
% a10 = [45,10,45];
% 
% Obs = [a1;a2;a3;a4;a5;a6;a7;a8;a9;a10];     % 障碍物坐标矩阵

% Obs = rand(n,3)*10;
%  Obs = randi([0,DesX],n,3);               % 随机生成障碍物
%% 超参数设置
Kaat = 1;                     % 引力尺度因子
Krep = 25;                     % 斥力尺度因子
P0 = 2;                        % 斥力作用范围
StepRate = 0.01;                % 步长
Epoch = 1000;                   % 最大迭代次数
% OBS1障碍物数据集下的参数Kaat=1 Krep=25 P0=2 StepRate=0.01 Epoch=1000
Obs = load('OBS1.mat').Obs;
%% 方程定义
syms x y z obs_x obs_y obs_z;

% 引力定义部分
u_att(x,y,z) = 1/2*Kaat*((x-DesX)^2+(y-DesY)^2+(z-DesZ)^2); % 引力势能函数
% f_att(x,y) = - (diff(u_att,x,1) + diff(u_att,y,1) + diff(u_att,z,1)); % 吸引力函数
f_attx(x,y,z) = - diff(u_att,x,1);
f_atty(x,y,z) = - diff(u_att,y,1);
f_attz(x,y,z) = - diff(u_att,z,1);

% 斥力定义部分
R_obs = (x-obs_x)^2 + (y-obs_y)^2 + (z-obs_z)^2;
r_obs = sqrt(R_obs);
u_rep(x,y,z,obs_x,obs_y,obs_z) = 1/2*Krep*(1/r_obs-1/P0)^2;
% f_rep(x,y,obs_x,obs_y) = - (diff(u_rep,x,1) + diff(u_rep,y,1) + diff(u_rep,z,1));
f_repx(x,y,z,obs_x,obs_y,obs_z) = - diff(u_rep,x,1);
f_repy(x,y,z,obs_x,obs_y,obs_z) = - diff(u_rep,y,1);
f_repz(x,y,z,obs_x,obs_y,obs_z) = - diff(u_rep,z,1);

%% 画图
timer = tic;

figure(1)
plot3(MyX,MyY,MyZ,'.','MarkerSize',4,'color','black');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
hold on 
plot3(Obs(:,1),Obs(:,2),Obs(:,3),'.','MarkerSize',8);

%% 计算
CountFlag = 0;
while(1)
   [Fattx,Fatty,Fattz] = Attractive(MyX,MyY,MyZ,f_attx,f_atty,f_attz); % 引力计算

   Frepx = zeros(1,n);
   Frepy = zeros(1,n);
   Frepz = zeros(1,n);
   for i = 1:n
        [Frepx(1,i),Frepy(1,i),Frepy(1,i)] = Repulsive(MyX,MyY,MyZ,Obs(i,1),Obs(i,2),Obs(i,3),P0,f_repx,f_repy,f_repz);        % 斥力计算
   end

   Fxsum = Fattx + sum(Frepx);
   Fysum = Fatty + sum(Frepy);
   Fzsum = Fattz + sum(Frepz);
 
%    PositionAngle = atan(Fysum/Fxsum); % x,y轴速度方向

   MyX = MyX + StepRate*Fxsum;
   MyY = MyY + StepRate*Fysum;
   MyZ = MyZ + StepRate*Fzsum;

   hold on 
   plot3(MyX,MyY,MyZ,'.','MarkerSize',4,'color','black');
   pause(0.1) % 这个一定要加不然就不会实时绘图
%    pos(CountFlag+2,1) = MyX;
%    pos(CountFlag+2,2) = MyY;
%    pos(CountFlag+3,3) = MyZ;
%    d = scatter3(MyX,MyY,MyZ,5,'m','filled');

   if(abs(MyX-DesX) < 1 && abs(MyY-DesY)< 1 && abs(MyZ-DesZ))
       fprintf("完成");
       break;
   end

   CountFlag = CountFlag + 1;
   if(CountFlag >= Epoch)
       fprintf("超时");
       break;
   end    
end
toc(timer);
% 
% %     legend([a,b,c,d],{'障碍物','起点','终点','路径'});
display(CountFlag);


