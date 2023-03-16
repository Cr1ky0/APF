function [F_angle,last_angle,cur_angle]= TurnAngleConstraint(last_Fxy,cur_Fxy)
   % 计算力的方向之间夹角
   last_angle = Angle(last_Fxy(1),last_Fxy(2));
   cur_angle = Angle(cur_Fxy(1),cur_Fxy(2)); 
   if(last_angle == cur_angle)
       F_angle = 0;
   else
       last_F = sqrt(last_Fxy(1)^2+last_Fxy(2)^2);
       cur_F = sqrt(cur_Fxy(1)^2+cur_Fxy(2)^2);
       dist_F = sqrt((last_Fxy(1)-cur_Fxy(1))^2+(last_Fxy(2)-cur_Fxy(2))^2);
       % 利用三角形三边求角度
       F_angle = acos((cur_F^2 + last_F^2 - dist_F^2) / (2*cur_F*last_F));
   end
end