function [X,Y,last_Fxy] = ComputeNewXY(MyX,MyY,Fxsum,Fysum,StepRate)
       X = MyX + StepRate*Fxsum;
       Y = MyY + StepRate*Fysum;
       last_Fxy = [Fxsum,Fysum];
end