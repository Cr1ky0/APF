function [Fx,Fy] = MappingF(F,cur_angle)
   if(cur_angle >= 2*pi)
       cur_angle = cur_angle - 2*pi;
   end
   Fx = F*cos(cur_angle);
   Fy = F*sin(cur_angle);
end