function PlotCone(x,y,z,r,h)
    %圆锥
    step = -0.05; % 决定片数 
    [X,Y,Z]=cylinder(r:step:0);
    X = X + x;
    Y = Y + y;
    Z= h * Z + z;
    c = surf(X,Y,Z);
    c.EdgeColor = 'none';
    hold on;
end