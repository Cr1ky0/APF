function [new_x,new_y,new_z] = ThreeBSpline(x,y,z)
n = size(x,1);
a = zeros(n,3);
a(:,1) = x;
a(:,2) = y;
a(:,3) = z;
% ��������������
syms u;
b0(u)=1/6*(-u^3+3*u^2-3*u+1);        % ������b0��
b1(u)=1/6*(3*u^3-6*u^2+4);          % ������b1��
b2(u)=1/6*(-3*u^3+3*u^2+3*u+1);      % ������b2��
b3(u)=1/6*u^3;                       % ������b3��
% ��������B��������
% ����4���������ȷ��һ��B����������n�����ƶ��㣬��ȷ��n-3��B��������
count = 1;
for i=1:n-3
    for u=0:0.2:1 % ����u,[0,1]֮��
        u0 = double(b0(u));u1=double(b1(u));u2=double(b2(u));u3=double(b3(u));
        % �������
        x=u0*a(i,1)+u1*a(i+1,1)+u2*a(i+2,1)+u3*a(i+3,1); 
        y=u0*a(i,2)+u1*a(i+1,2)+u2*a(i+2,2)+u3*a(i+3,2); 
        z=u0*a(i,3)+u1*a(i+1,3)+u2*a(i+2,3)+u3*a(i+3,3);
        new_x(count,1)=x;
        new_y(count,1)=y;
        new_z(count,1)=z;
        count = count + 1;
        hold on 
    end
end
end


