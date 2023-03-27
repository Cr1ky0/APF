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
DesZ = 5;
n1 = 5;                                    % 球体障碍物个数
n2 = 5;                                    % 圆柱体障碍物个数
n3 = 5;                                    % 圆锥体障碍物个数
n = n1 + n2 + n3;                          
%% 超参数设置
Kaat = 1;                     % 引力尺度因子
Krep = 15;                     % 斥力尺度因子
P0 = 2;                        % 斥力作用范围
StepRate = 0.01;                % 步长
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
[Sphere_Obs,Cylinder_Obs,Cone_Obs] = PlotObs(n1,n2,n3);
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
   Obs3 = GetConeObsPoint([StartX,StartY,StartZ],Cone_Obs,n3,P0);
   % 汇总
   Obs = [Obs1;Obs2;Obs3];
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

%% 解决路径震荡问题（插值/拟合）
%%%%%%%%待实现

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


