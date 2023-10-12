function [f_attx,f_atty,f_attz,f_repx,f_repy,f_repz] = EquationDefinition(DesX,DesY,DesZ,m,Kaat,Krep,P0)
    syms x1 y1 z1 x2 y2 z2 obs_x obs_y obs_z;
    R_des = (x2-DesX)^2+(y2-DesY)^2+(z2-DesZ)^2;
    R_obs = (x1-obs_x)^2 + (y1-obs_y)^2 + (z1-obs_z)^2;
    r_des = sqrt(R_des);
    r_obs = sqrt(R_obs);
    
    % 引力定义部分
    u_att(x2,y2,z2) = 1/2*Kaat*(R_des); % 引力势能函数

    % 各方向引力为引力势能的负梯度
    f_attx(x2,y2,z2) = - diff(u_att,x2,1);
    f_atty(x2,y2,z2) = - diff(u_att,y2,1);
    f_attz(x2,y2,z2) = - diff(u_att,z2,1);
    
    % 斥力定义部分
    u_rep(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = 1/2*Krep*(1/r_obs-1/P0)^2*sqrt(r_des)^m; % 斥力势能

    % 各方向斥力为斥力势能的负梯度
    f_repx1(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,x1,1);
    f_repy1(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,y1,1);
    f_repz1(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,z1,1);
    
    f_repx2(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,x2,1);
    f_repy2(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,y2,1);
    f_repz2(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = - diff(u_rep,z2,1);
    
    f_repx(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = f_repx1 + f_repx2;
    f_repy(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = f_repy1 + f_repy2;
    f_repz(x1,y1,z1,x2,y2,z2,obs_x,obs_y,obs_z) = f_repz1 + f_repz2;

end