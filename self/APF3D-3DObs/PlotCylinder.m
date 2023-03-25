function PlotCylinder(x,y,z,r,h)
    n = 100;% 边缘块数
    [X,Y,Z] = cylinder(r,n);
    X = X + x;
    Y = Y + y;
    Z = Z*h + z;
    c = surf(X,Y,Z);
    c.FaceColor = '#31AAEB';
    c.EdgeColor = 'none';
    hold on
end