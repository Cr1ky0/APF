function [new_x, new_y, new_z] = N_BSpline(x, y, z, n)
    numPoints = length(x);
    
    % 创建结点向量
    u = [zeros(1, n+1), linspace(0, 1, numPoints-n), ones(1, n+1)];
    
    % 计算曲线上的点
    new_x = [];
    new_y = [];
    new_z = [];
    
    u_val = linspace(0, 1, 1000);
    for ui = u_val
        x_val = 0;
        y_val = 0;
        z_val = 0;
        for i = 1:numPoints
            Ni = N(i, n, ui, u);
            x_val = x_val + Ni * x(i);
            y_val = y_val + Ni * y(i);
            z_val = z_val + Ni * z(i);
        end
        new_x(end+1) = x_val;
        new_y(end+1) = y_val;
        new_z(end+1) = z_val;
    end

   
end

function val = N(i, k, u, U)
    if k == 0
        val = (U(i) <= u) && (u < U(i+1));
    else
        if (U(i+k) - U(i) == 0)
            term1 = 0;
        else
            term1 = ((u - U(i)) / (U(i+k) - U(i))) * N(i, k-1, u, U);
        end
        
        if (U(i+k+1) - U(i+1) == 0)
            term2 = 0;
        else
            term2 = ((U(i+k+1) - u) / (U(i+k+1) - U(i+1))) * N(i+1, k-1, u, U);
        end
        val = term1 + term2;
    end
end