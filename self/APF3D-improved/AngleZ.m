function [angle_z,r] = AngleZ(last_xyz,x,y,z)
    r = sqrt((x-last_xyz(1))^2 + (y-last_xyz(2))^2 + (z-last_xyz(3))^2);
    z_dist = abs(z - last_xyz(3));
    angle_z = asin(z_dist / r);
end