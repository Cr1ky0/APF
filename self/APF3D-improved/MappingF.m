function [Fx,Fy] = MappingF(cur_angle)
   if(cur_angle >= 2*pi)
       cur_angle = cur_angle - 2*pi;
   end
   Fx = cos(cur_angle);
   Fy = sin(cur_angle);
end