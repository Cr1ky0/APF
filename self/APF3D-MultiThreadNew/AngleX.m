function angle = AngleX(Fxsum,Fysum) 
% 求水平力与X轴夹角
    F = sqrt(Fxsum^2 + Fysum^2);
%     angle = acos(Fxsum / F);
    % 计算与x轴夹角（包括>=180°）
    if(Fxsum > 0)
        if(Fysum > 0)
            angle = acos(Fxsum / F);
        else
            angle = 2*pi - acos(Fxsum / F);
        end
    else
        if(Fysum > 0)
            angle = acos(Fxsum / F);
        else
            angle = 2*pi - acos(Fxsum / F);
        end
    end 
end