function PlotCone(x,y,z,r,h)
    %圆锥
    r = 1; % 底面半径
    step = -0.05; % 决定片数 
    [X,Y,Z]=cylinder(r:step:0);
    X = X + x;
    Y = Y + x;
    Z=h*Z + z;
    c = surf(X,Y,Z);
    c.EdgeColor = 'none';
    hold on;
end