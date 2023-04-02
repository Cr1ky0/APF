function Obs = GetSphereObsPoint(cur_pos,obs_data,n,P0) % obs_data前三位为球体中心坐标，第四位为半径r
    %%%% 获取球体Obs的表面obs点（到球面的最小距离）
    X = cur_pos(1); % 无人机当前坐标
    Y = cur_pos(2);
    Z = cur_pos(3);
    Obs = zeros(n,3);
    for i = 1:n
        % 球中心坐标
        x = obs_data(i,1); 
        y = obs_data(i,2);
        z = obs_data(i,3);
        r = obs_data(i,4);
        % 计算相关∠
        x_diff = abs(x - X);
        y_diff = abs(y - Y);
        z_diff = abs(z - Z);
        dist = sqrt(x_diff^2 + y_diff^2 + z_diff^2);
        % 如果当前球面到达坐标的距离大于P0则不计算
        if(dist - r > P0)
            Obs(i,1) = -10;
            Obs(i,2) = -10;
            Obs(i,3) = -10;
            continue;
        end
        angle_x = atan(y_diff / x_diff);
        angle_y = atan(x_diff / y_diff);
        angle_z = atan(z_diff / dist);
        % 计算表面坐标
        % 用加减坐标的方式有效避免了坐标跨象限的问题
        dist_to_surf = dist - r;
        dist_to_surf_xy = dist_to_surf * cos(angle_z);
        if(x > X)
            x_obs = X + dist_to_surf_xy * cos(angle_x);
        else
            x_obs = X - dist_to_surf_xy * cos(angle_x);
        end
        if(y > Y)
            y_obs = Y + dist_to_surf_xy * cos(angle_y);
        else
            y_obs = Y - dist_to_surf_xy * cos(angle_y);
        end
        if(z > Z)
            z_obs = Z + dist_to_surf * sin(angle_z);
        else
            z_obs = Z - dist_to_surf * sin(angle_z);
        end
        Obs(i,1) = x_obs;
        Obs(i,2) = y_obs;
        Obs(i,3) = z_obs;
    end
end