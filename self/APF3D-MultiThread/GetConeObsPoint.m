function Obs = GetConeObsPoint(cur_pos,obs_data,n,P0)
    % 与圆柱类似
    % 无人机当前坐标
    X = cur_pos(1); 
    Y = cur_pos(2);
    Z = cur_pos(3);
    Obs = zeros(n,3);
    for i = 1:n
        % 获取圆锥参数
        x = obs_data(i,1);
        y = obs_data(i,2);
        z = obs_data(i,3);
        r = obs_data(i,4);
        h = obs_data(i,5);

        x_diff = abs(x - X);
        y_diff = abs(y - Y);
        dist_xy = sqrt(x_diff^2 + y_diff^2);
        % 在P0外(下底面存在一个半径P0的球形范围，在范围内则有斥力，上顶点在P0内当做Obs)
        if(Z > z && Z < z + h && dist_xy - abs((z + h - Z)) * r / h > P0 || ...
                Z < z && sqrt(dist_xy^2+(z-Z)^2) > P0 || ...
                Z > z + h && sqrt(dist_xy^2+(z+h-Z)^2) > P0)
            Obs(i,1) = -10;
            Obs(i,2) = -10;
            Obs(i,3) = -10;
            continue;
        end
        angle_x = atan(y_diff / x_diff);
        angle_y = atan(x_diff / y_diff);
            % 计算同水平位置圆锥切面圆半径
            R = abs((z + h - Z)) * r / h; % 利用比例关系求R
            % 计算到圆锥切面边缘的距离最短点
            if(x > X)
                x_obs = x - R * cos(angle_x);
            else
                x_obs = x + R * cos(angle_x);
            end
            if(y > Y)
                y_obs = y - R * cos(angle_y);
            else
                y_obs = y + R * cos(angle_y);
            end
        % 超出圆锥高度范围（分为无人机在底面圆正上方和不在正上方）
        if(Z < z)
            if(dist_xy <= r) % 如果在正下方，取圆表面的点
                x_obs = X;
                y_obs = Y;
            end
            z_obs = z;
        elseif(Z >= z + h)
            x_obs = x;
            y_obs = y;
            z_obs = z + h;
        else   
            z_obs = Z;
        end

        Obs(i,1) = x_obs;
        Obs(i,2) = y_obs;
        Obs(i,3) = z_obs;

    end
end