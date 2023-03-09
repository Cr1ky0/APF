function [F_at_x,F_at_y]=compute_Attract(X,Xsum,k,angle)%输入参数为当前坐标，目标坐标，增益常数,分量和力的角度
%把路径上的临时点作为每个时刻的Xgoal
R=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%路径点和目标的距离平方
r=sqrt(R);%路径点和目标的距离
F_at_x=k*r*cos(angle);% 按照论文的[引力公式]计算
F_at_y=k*r*sin(angle);
end
