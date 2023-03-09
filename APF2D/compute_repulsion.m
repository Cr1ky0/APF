function [Yrerxx,Yreryy,Yataxx,Yatayy]=compute_repulsion(X,Xsum,m,angle_at,angle_re,n,Po,a)%输入参数为当前坐标，Xsum是目标和障碍的坐标向量，增益常数,障碍，目标方向的角度
R_at=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%路径点和目标的距离平方
r_at=sqrt(R_at);%路径点和目标的距离
for i=1:n
    R_re_i(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;%路径点和障碍的距离平方
    r_re(i)=sqrt(R_re_i(i));%路径点和障碍的距离保存在数组rrei中
    if r_re(i)>Po%如果每个障碍和路径的距离大于障碍影响距离，斥力令为0
        Y_re_r_x(i)=0;
        Y_re_r_y(i)=0;
        Y_at_a_x(i)=0;
        Y_at_a_y(i)=0;
    else
%if r0<Po
    if r_re(i)<Po/2
        Y_re_r(i)=m*(1/r_re(i)-1/Po)*(1/R_re_i(i))*(r_at^a);%分解的Fre1向量
        Y_at_a(i)=a*m*((1/r_re(i)-1/Po)^2)*(r_at^(1-a))/2;%分解的Fre2向量
        Y_re_r_x(i)=(1+0.1)*Y_re_r(i)*cos(angle_re(i));%angle_re(i)=Y(i+1)
        Y_re_r_y(i)=-(1-0.1)*Y_re_r(i)*sin(angle_re(i));
        Y_at_a_x(i)=Y_at_a(i)*cos(angle_at);%angle_at=Y(1)
        Y_at_a_y(i)=Y_at_a(i)*sin(angle_at);
    else
        Y_re_r(i)=m*(1/r_re(i)-1/Po)*1/R_re_i(i)*R_at;%分解的Fre1向量
        Y_at_a(i)=m*((1/r_re(i)-1/Po)^2)*r_at;%分解的Fre2向量
        Y_re_r_x(i)=Y_re_r(i)*cos(angle_re(i));%angle_re(i)=Y(i+1)
        Y_re_r_y(i)=Y_re_r(i)*sin(angle_re(i));
        Y_at_a_x(i)=Y_at_a(i)*cos(angle_at);%angle_at=Y(1)
        Y_at_a_y(i)=Y_at_a(i)*sin(angle_at);
    end
    end%判断距离是否在障碍影响范围内
end
    Yrerxx=sum(Y_re_r_x);%叠加斥力的分量
    Yreryy=sum(Y_re_r_y);
    Yataxx=sum(Y_at_a_x);
    Yatayy=sum(Y_at_a_y);
end
