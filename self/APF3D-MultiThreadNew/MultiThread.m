function [path_points] = MultiThread(Start,Des, ...
    n,n_obs, ...
    P0,StepRate,max_step,max_turn_angle,max_pitch_angle,Epoch,...
    f_attx,f_atty,f_attz,f_repx,f_repy,f_repz, ...
    Sphere_Obs,Cylinder_Obs,Cone_Obs,...
    serial_number)
    %% 计算 
    timer = tic; % 计时
    StartX = Start(1,1);
    StartY = Start(1,2);
    StartZ = Start(1,3);
    CountFlag = 0;
    last_Fxy = [0,0];
    last_xyz = [Start(1,1),Start(1,2),Start(1,3)];
    path_points = zeros(Epoch,3);
    path_points(1,:) = Start(1,:);
    while(1)
        [StartX,StartY,StartZ,last_Fxy,last_xyz,isDone,CountFlag] = MainLogic(CountFlag, [StartX,StartY,StartZ],Des ,...
            n,n_obs, ...
            P0,StepRate,max_step,max_turn_angle,max_pitch_angle,Epoch, ...
            f_attx,f_atty,f_attz,f_repx,f_repy,f_repz, ...
            Sphere_Obs,Cylinder_Obs,Cone_Obs, ...
            last_xyz,last_Fxy);
        path_points(CountFlag+1,:) = [StartX,StartY,StartZ];
   
        if(isDone)
            break;
        end
    end
   if(CountFlag >= Epoch)
       fprintf("无人机%d超时\n",serial_number);
   else
       fprintf("无人机%d到达终点\n",serial_number);
       toc(timer);
       display(CountFlag);
   end    
 
   % 返回轨迹点
   path_points = path_points(1:CountFlag+1,:);
end