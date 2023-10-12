function [Frepx,Frepy,Frepz] = ImprovedRepulsive(x,y,z,obs_x,obs_y,obs_z,P0,fx,fy,fz)
% 先计算与障碍物的欧氏距离
x_dist = obs_x - x;
y_dist = obs_y - y;
z_dist = obs_z - z;
Pobs = sqrt(y_dist^2 + x_dist^2 + z_dist^2);
    if(Pobs>P0)
        Frepx = 0;
        Frepy = 0;
        Frepz = 0;
    else
        Frepx = double(fx(x,y,z,x,y,z,obs_x,obs_y,obs_z));
        Frepy = double(fy(x,y,z,x,y,z,obs_x,obs_y,obs_z));
        Frepz = double(fz(x,y,z,x,y,z,obs_x,obs_y,obs_z));
    end
end