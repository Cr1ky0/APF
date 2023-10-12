function [X,Y,Z] = MappingCoordinates(x,y,z,last_x,last_y,last_z,max_step)
    %% 限制最大步长在同方向上进行映射
    [angle_z,r] = AngleZ([last_x,last_y,last_z],x,y,z);
    angle_x = atan(abs(y-last_y)/abs(x-last_x));
    angle_y = atan(abs(x-last_x)/abs(y-last_y));
    dist_xy = max_step * cos(angle_z);
    z_diff = max_step * sin(angle_z);
    y_diff = dist_xy * cos(angle_y);
    x_diff = dist_xy * cos(angle_x);
    if(r > max_step)
        if(z > last_z)
            Z = last_z + z_diff;
        else
            Z = last_z - z_diff;
        end
        if(y > last_y)
            Y = last_y + y_diff;
        else
            Y = last_y - y_diff;
        end
        if(x > last_x)
            X = last_x + x_diff;
        else
            X = last_x - x_diff;
        end
    else
        X = x;
        Y = y;
        Z = z;
    end

end