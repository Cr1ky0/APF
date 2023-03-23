function [] = PlotSphere(x,y,z,r)
    % 传入中心坐标画圆
    [X,Y,Z] = sphere(100);
    X = X * r;
    Y = Y * r;
    Z = Z * r;
    c = surf(X+x,Y+y,Z+z);
    c.EdgeColor = 'none';
%     c.FaceAlpha = 1;
%     c.FaceColor = '#09DFFF';
    c.SpecularStrength = 1;
    c.AmbientStrength = 1;
    c.DiffuseStrength = 1;
    hold on
end