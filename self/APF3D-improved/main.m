clear 
clc
close

timer = tic;
%% 起始点位置
MyX = 0;                                    % 出发点位置
MyY = 0;
MyZ = 0;
DesX = 10;                                 % 终点位置
DesY = 10;
DesZ = 10;
n = 100;                                    % 障碍物个数

%% 超参数设置
Kaat = 1;                     % 引力尺度因子
Krep = 50;                     % 斥力尺度因子
P0 = 2;                        % 斥力作用范围
StepRate = 0.007;                % 步长
Epoch = 2000;                   % 最大迭代次数
max_turn_angle = pi/6;          % 最大转向角
max_pitch_angle = pi/12;         % 最大俯仰角
%% 自定义障碍物
Obs = rand(n,3)*10 + 0.5;
%  Obs = randi([0,DesX],n,3);               % 随机生成障碍物
% Obs = [7,7,7];
% OBS1障碍物数据集下的参数Kaat=1 Krep=25 P0=2 StepRate=0.01 Epoch=1000
% Obs = load('OBS1.mat').Obs;

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

figure(1)
plot3(MyX,MyY,MyZ,'.','MarkerSize',4,'color','black');
view(45,30);
hold on
plot3(DesX,DesY,DesZ,'o','MarkerSize',10,'Color','yellow');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
hold on 
plot3(Obs(:,1),Obs(:,2),Obs(:,3),'.','MarkerSize',8);

%% 计算
CountFlag = 0;
last_Fxy = [0,0];
last_xyz = [MyX,MyY,MyZ];
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
   %% 约束
   if(CountFlag == 0)
      [MyX,MyY,last_Fxy] = ComputeNewXY(MyX,MyY,Fxsum,Fysum,StepRate);
      MyZ = MyZ + StepRate*Fzsum;
   else 
       %% 水平转向角
       [F_angle,last_angle,cur_angle] = TurnAngleConstraint(last_Fxy,[Fxsum,Fysum]);
       if(F_angle > max_turn_angle)
           if(cur_angle > last_angle)
               fprintf("大于\n")
               cur_angle = last_angle + max_turn_angle;
           else
               fprintf("小于\n")
               cur_angle = last_angle - max_turn_angle;
           end
           %  F = sqrt(Fxsum^2 + Fysum^2);
           F = sqrt(last_Fxy(1)^2+last_Fxy(2)^2);
           [Fx,Fy] = MappingF(F,cur_angle);
           [MyX,MyY,last_Fxy] = ComputeNewXY(MyX,MyY,Fx,Fy,StepRate);
       else
           [MyX,MyY,last_Fxy] = ComputeNewXY(MyX,MyY,Fxsum,Fysum,StepRate);
       end

       %% 最大俯仰角约束
       Z = MyZ + StepRate*Fzsum;
       [angle_z,r] = AngleZ(last_xyz,MyX,MyY,Z);
       if(angle_z > max_pitch_angle)
           fprintf(">\n");
           if(Fzsum < 0) 
              MyZ = MyZ - r*sin(max_pitch_angle);
           else
              MyZ = MyZ + r*sin(max_pitch_angle); 
           end
       else
            MyZ = MyZ + StepRate*Fzsum;
       end
   end

   last_xyx(1) = MyX;
   last_xyx(2) = MyY;
   last_xyz(3) = MyZ;

   %% 实时绘图
   hold on 
   plot3(MyX,MyY,MyZ,'.','MarkerSize',4,'color','black');
   pause(0) % 这个一定要加不然就不会实时绘图

   %% 判断模块
   if(abs(MyX-DesX) < 0.5 && abs(MyY-DesY)< 0.5 && abs(MyZ-DesZ) < 0.5)
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


