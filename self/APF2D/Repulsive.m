function [Frepx,Frepy] = Repulsive(x,y,obs_x,obs_y,P0,fx,fy)
% 先计算与障碍物的欧氏距离
x_dist = obs_x - x;
y_dist = obs_y - y;
Pobs = sqrt(y_dist^2 + x_dist^2);
    if(Pobs>P0)
        Frepx = 0;
        Frepy = 0;
    else
        Frepx = fx(x,y,obs_x,obs_y);
        Frepy = fy(x,y,obs_x,obs_y);
    end
end