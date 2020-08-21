function [Xt,Yt,Zt] = transformSurf(H,X,Y,Z)
Xt = zeros(size(X)); Yt = zeros(size(Y)); Zt = zeros(size(Z));
for i=1:size(X,1)
   for j=1:size(X,2)
      pos = H*[X(i,j);Y(i,j);Z(i,j);1]; 
      Xt(i,j) = pos(1); Yt(i,j) = pos(2); Zt(i,j) = pos(3); 
   end
end
end