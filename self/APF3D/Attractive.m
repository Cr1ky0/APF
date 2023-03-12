function [Fatx,Faty,Fatz] = Attractive(x0,y0,z0,fx,fy,fz)
    Fatx = double(fx(x0,y0,z0));
    Faty = double(fy(x0,y0,z0));
    Fatz = double(fz(x0,y0,z0));
end