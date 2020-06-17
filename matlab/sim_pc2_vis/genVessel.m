function [Xc,Yc,Zc] = genVessel(r,Hvw)

%Generate unit cylinder
t = r*ones(250,1);
[X,Y,Z] = cylinder(t,100);

X = X*1.2;
Z = (Z-0.5)*0.5;

[Xc,Yc,Zc] = transformSurf(Hvw,X,Y,Z);

end