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
n = n1 + n2;
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
syms x1 y1 z1 x2 y2 z2 obs_x obs_y obs_z;
R_des = (x2-DesX)^2+(y2-DesY)^2+(z2-DesZ)^2;
R_obs = (x1-obs_x)^2 + (y1-obs_y)^2 + (z1-obs_z)^2;
r_des = sqrt(R_des);
r_obs = sqrt(R_obs);
m = 0.5; % 改进方程参数定义

% 引力定义部分
u_att(x2,y2,z2) = 1/2*Kaat*(R_des); % 引力势能函数
f_attx(x2,y2,z2) = - diff(u_att,x2,1);
f_atty(x2,y2,z2) = - diff(u_att,y2,1);
f_attz(x2,y2,z2) = - diff(u_att,z2,1);

% 斥力定义部分
u_rep(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = 1/2*Krep*(1/r_obs-1/P0)^2*sqrt(r_des)^m;
f_repx1(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,x1,1);
f_repy1(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,y1,1);
f_repz1(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,z1,1);

f_repx2(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,x2,1);
f_repy2(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,y2,1);
f_repz2(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,z2,1);

f_repx(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = f_repx1 + f_repx2;
f_repy(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = f_repy1 + f_repy2;
f_repz(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = f_repz1 + f_repz2;

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
[Sphere_Obs,Cylinder_Obs] = PlotObs(n1,n2);
%% 计算
CountFlag = 0;
last_Fxy = [0,0];
last_xyz = [StartX,StartY,StartZ];
while(1)
   %% 计算最短Obs点
   % 圆表面obs计算
   Obs1 = GetSphereObsPoint([StartX,StartY,StartZ],Sphere_Obs,n1,P0);
   % 圆柱表面obs计算（可以用当前无人机距离同水平面圆柱的切面圆的边缘到无人机的最短距离的点当障碍物）
   Obs2 = GetCylinderObsPoint([StartX,StartY,StartZ],Cylinder_Obs,n2,P0);
   % 圆锥表面obs计算
   Obs = [Obs1;Obs2];
   %% 力计算
   % 引力计算
   [Fattx,Fatty,Fattz] = Attractive(StartX,StartY,StartZ,f_attx,f_atty,f_attz); 
   
   % 斥力计算
   Frepx = zeros(1,n);
   Frepy = zeros(1,n);
   Frepz = zeros(1,n);
   for i = 1:n
        [Frepx(1,i),Frepy(1,i),Frepy(1,i)] = ImprovedRepulsive(StartX,StartY,StartZ,Obs(i,1),Obs(i,2),Obs(i,3),P0,f_repx,f_repy,f_repz);  
   end

   % 总力计算
   Fxsum = Fattx + sum(Frepx);
   Fysum = Fatty + sum(Frepy);
   Fzsum = Fattz + sum(Frepz);

%% 约束
   if(CountFlag == 0)
      [StartX,StartY,last_Fxy] = ComputeNewXY(StartX,StartY,Fxsum,Fysum,StepRate);
      StartZ = StartZ + StepRate*Fzsum;
   else
       % 水平转向角
       [F_angle,last_angle,cur_angle] = TurnAngleConstraint(last_Fxy,[Fxsum,Fysum]);
       if(F_angle > max_turn_angle)
           if(cur_angle > last_angle)
%                fprintf("大于\n")
               cur_angle = last_angle + max_turn_angle;
           else
%                fprintf("小于\n")
               cur_angle = last_angle - max_turn_angle;
           end
           % 映射到最大偏向角位置
           [Fx,Fy] = MappingF(cur_angle);   
           [StartX,StartY,last_Fxy] = ComputeNewXY(StartX,StartY,Fx,Fy,StepRate);
       else
           [StartX,StartY,last_Fxy] = ComputeNewXY(StartX,StartY,Fxsum,Fysum,StepRate);
       end
       % 最大俯仰角约束
       Z = StartZ + StepRate*Fzsum;
       [angle_z,r] = AngleZ(last_xyz,StartX,StartY,Z);
       if(angle_z > max_pitch_angle)
%            fprintf(">\n");
           if(Fzsum < 0) 
              StartZ = StartZ - r*sin(max_pitch_angle);
           else
              StartZ = StartZ + r*sin(max_pitch_angle); 
           end
       else
            StartZ = StartZ + StepRate*Fzsum;
       end
   end
%% 路径总长度约束

   [StartX,StartY,StartZ] = MappingCoordinates(StartX,StartY,StartZ,last_xyz(1),last_xyz(2),last_xyz(3),max_step);

   %% 实时绘图
   hold on 
%    plot3(MyX,MyY,MyZ,'.','MarkerSize',4,'color','black');
   plot3([last_xyz(1) StartX],[last_xyz(2) StartY],[last_xyz(3) StartZ],'-','MarkerSize',4,'color','black');
   pause(0) % 这个一定要加不然就不会实时绘图

   % 覆盖上一次坐标
   last_xyz(1) = StartX;
   last_xyz(2) = StartY;
   last_xyz(3) = StartZ;
   %% 判断模块
   if(abs(StartX-DesX) < 0.5 && abs(StartY-DesY)< 0.5 && abs(StartZ-DesZ) < 0.5)
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
display(CountFlag);


