function Obs = GetCylinderObsPoint(cur_pos,obs_data,n,P0)
    % 在圆柱高度内则可以用当前无人机距离同水平面圆柱的切面圆的边缘到无人机的最短距离的点当障碍物
    % 否则使用上下底圆的边缘距离无人机最短的点为障碍物
    % 无人机当前坐标
    X = cur_pos(1); 
    Y = cur_pos(2);
    Z = cur_pos(3);
    Obs = zeros(n,3);
    for i = 1:n
        % 获取圆柱参数
        x = obs_data(i,1);
        y = obs_data(i,2);
        z = obs_data(i,3);
        r = obs_data(i,4);
        h = obs_data(i,5);

        x_diff = abs(x - X);
        y_diff = abs(y - Y);
        dist_xy = sqrt(x_diff^2 + y_diff^2);
        % 在P0外(上下底面存在一个半径P0的球形范围，在范围内则有斥力)
        if(Z > z && Z < z + h && dist_xy - r > P0 || ...
                Z < z && sqrt((x-X)^2+(y-Y)^2+(z-Z)^2) > P0 || ...
                Z > z + h && sqrt((x-X)^2+(y-Y)^2+(z+h-Z)^2) > P0)
            Obs(i,1) = -10;
            Obs(i,2) = -10;
            Obs(i,3) = -10;
            continue;
        end
  
        angle_x = atan(y_diff / x_diff);
        angle_y = atan(x_diff / y_diff);
        % 计算到圆柱切面边缘的距离最短点（通用）
        if(x > X)
            x_obs = X + r * cos(angle_x);
        else
            x_obs = X - r * cos(angle_x);
        end
        if(y > Y)
            y_obs = Y + r * cos(angle_y);
        else
            y_obs = Y - r * cos(angle_y);
        end
        % 超出圆柱高度范围（分为无人机在底面圆正上方和不在正上方）
        if(Z < z)
            if(dist_xy < r) % 如果在正下方，取圆表面的点
                x_obs = X;
                y_obs = Y;
            end
            z_obs = z;
        elseif(Z > z + h)
            if(dist_xy < r) % 如果在正上方，取圆表面的点
                x_obs = X;
                y_obs = Y;
            end
            z_obs = z + h;
        else
            z_obs = Z;
        end

        Obs(i,1) = x_obs;
        Obs(i,2) = y_obs;
        Obs(i,3) = z_obs;
    end
end