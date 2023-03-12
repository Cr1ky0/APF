function [Fatx,Faty] = Attractive(x0,y0,fx,fy)
    Fatx = double(fx(x0,y0));
    Faty = double(fy(x0,y0));
end