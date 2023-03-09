function Theta=compute_angle(X,Xsum,n)%Y是引力，斥力与x轴的角度向量,X是起点坐标，Xsum是目标和障碍的坐标向量,是(n+1)*2矩阵
for i=1:n+1%n是障碍数目
    deltaX(i)=Xsum(i,1)-X(1); % 两点间X方向长度差（X方向边长）
    deltaY(i)=Xsum(i,2)-X(2); % 两点间Y方向长度差（Y方向边长）
    r(i)=sqrt(deltaX(i)^2+deltaY(i)^2); % 勾股定理求距离
    if deltaX(i)>0
        theta=acos(deltaX(i)/r(i)); % 反余弦通过cos值求角度
    else
        theta=pi-acos(deltaX(i)/r(i)); % 计算顺时针方向夹角，所以用pi减去
    end
    if i==1%表示是目标
        angle=theta;
    else
        angle=theta;
    end
Theta(i)=angle; % 保存每个角度在Y向量里面，第一个元素是与目标的角度，后面都是与障碍的角度
end
end

