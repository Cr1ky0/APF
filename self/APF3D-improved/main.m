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
Krep = 10;                     % 斥力尺度因子
P0 = 2;                        % 斥力作用范围
StepRate = 0.02;                % 步长
Epoch = 1000;                   % 最大迭代次数
max_turn_angle = pi/6;          % 最大转向角
max_pitch_angle = pi/8;         % 最大俯仰角
%% 自定义障碍物
Obs = rand(n,3)*10;
%  Obs = randi([0,DesX],n,3);               % 随机生成障碍物
% Obs = [10.5,10.5,10.5];
% OBS1障碍物数据集下的参数Kaat=1 Krep=25 P0=2 StepRate=0.01 Epoch=1000
% Obs = load('OBS1.mat').Obs;
% Obs = load('OBS2.mat').Obs;

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
   %% 力计算
   % 引力计算
   [Fattx,Fatty,Fattz] = Attractive(MyX,MyY,MyZ,f_attx,f_atty,f_attz); 

   % 斥力计算
   Frepx = zeros(1,n);
   Frepy = zeros(1,n);
   Frepz = zeros(1,n);
   for i = 1:n
%         [Frepx(1,i),Frepy(1,i),Frepy(1,i)] = Repulsive(MyX,MyY,MyZ,Obs(i,1),Obs(i,2),Obs(i,3),P0,f_repx,f_repy,f_repz);  
        [Frepx(1,i),Frepy(1,i),Frepy(1,i)] = ImprovedRepulsive(MyX,MyY,MyZ,Obs(i,1),Obs(i,2),Obs(i,3),P0,f_repx,f_repy,f_repz);  
   end

   % 总力计算
   Fxsum = Fattx + sum(Frepx);
   Fysum = Fatty + sum(Frepy);
   Fzsum = Fattz + sum(Frepz);
%% 非约束
   
%    [MyX,MyY,last_Fxy] = ComputeNewXY(MyX,MyY,Fxsum,Fysum,StepRate);
%    MyZ = MyZ + StepRate*Fzsum;
   
%% 约束
   if(CountFlag == 0)
      [MyX,MyY,last_Fxy] = ComputeNewXY(MyX,MyY,Fxsum,Fysum,StepRate);
      MyZ = MyZ + StepRate*Fzsum;
   else
       % 水平转向角
       [F_angle,last_angle,cur_angle] = TurnAngleConstraint(last_Fxy,[Fxsum,Fysum]);
       if(F_angle > max_turn_angle)
           if(cur_angle > last_angle)
               fprintf("大于\n")
               cur_angle = last_angle + max_turn_angle;
           else
               fprintf("小于\n")
               cur_angle = last_angle - max_turn_angle;
           end
           % 以StepRate为F计算，用F计算可能导致漂移（仍有漂移问题，但不会反复横跳了，本质是下一步后离点太近导致斥力过大，考虑修改一下原函数） 
           [Fx,Fy] = MappingF(cur_angle);   
           [MyX,MyY,last_Fxy] = ComputeNewXY(MyX,MyY,Fx,Fy,StepRate);
       else
           [MyX,MyY,last_Fxy] = ComputeNewXY(MyX,MyY,Fxsum,Fysum,StepRate);
       end
       % 最大俯仰角约束
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

   %% 实时绘图
   hold on 
%    plot3(MyX,MyY,MyZ,'.','MarkerSize',4,'color','black');
     plot3([last_xyz(1) MyX],[last_xyz(2) MyY],[last_xyz(3) MyZ],'-','MarkerSize',4,'color','black');
   pause(0) % 这个一定要加不然就不会实时绘图

   % 覆盖上一次坐标
   last_xyz(1) = MyX;
   last_xyz(2) = MyY;
   last_xyz(3) = MyZ;
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


