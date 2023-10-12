function [] = PlotSphere(x,y,z,r)
    % 传入中心坐标画圆
    [X,Y,Z] = sphere(100);
    X = X * r;
    Y = Y * r;
    Z = Z * r;
    c = surf(X+x,Y+y,Z+z);
    c.EdgeColor = 'none';
%     c.FaceColor = '#6C7478';
%     c.AmbientStrength = 0.1;
%     c.DiffuseStrength = 0.3;
%     c.SpecularStrength = 0.1;
%     c.SpecularExponent = 10;

%     l =light;
%     l.Position = [10 0 10];
%     material metal
    hold on
end