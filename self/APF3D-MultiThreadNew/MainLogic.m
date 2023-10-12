function [StartX,StartY,StartZ,last_Fxy,last_xyz,isDone,CountFlag] = MainLogic(CountFlag,Start,Des, ...
    n,n_obs, ...
    P0,StepRate,max_step,max_turn_angle,max_pitch_angle,Epoch,...
    f_attx,f_atty,f_attz,f_repx,f_repy,f_repz, ...
    Sphere_Obs,Cylinder_Obs,Cone_Obs, ...
    last_xyz,last_Fxy)
  
   isDone = false; % 是否到达地点标记
   StartX = Start(1);StartY = Start(2);StartZ = Start(3); % 起始点
   n1 = n_obs(1);n2 = n_obs(2);n3 = n_obs(3); % 障碍物个数
   DesX=Des(1);DesY=Des(2);DesZ=Des(3); % 目的地
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

%% 实时绘图
%    hold on 
%    plot3([last_xyz(1) StartX],[last_xyz(2) StartY],[last_xyz(3) StartZ],'-','MarkerSize',4,'color','black');
%    pause(0) % 这个一定要加不然就不会实时绘图

   % 覆盖上一次坐标
   last_xyz(1) = StartX;
   last_xyz(2) = StartY;
   last_xyz(3) = StartZ;
   
   %% 判断模块
   dist_to_des = sqrt((StartX-DesX)^2 + (StartY-DesY)^2 + (StartZ-DesZ)^2);
   %fprintf('%d %d %d\n',DesX,DesY,DesZ);
   %fprintf('%d %d %d\n',StartX,StartY,StartZ);
   if( dist_to_des < 0.1)
       isDone=true;
   end

   CountFlag = CountFlag + 1;
   if(CountFlag >= Epoch)
       isDone=true;
   end    
end