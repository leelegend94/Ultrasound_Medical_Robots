clear
clc

syms n1 n2 n_d1 n_d2 r real
syms cp [3,1] real

n = sym([n1;n2;1]);

term = norm(cross(cp,n))/norm(n);

% Cylinder Fitting
se = (term-r)^2; %squared error
pse_n1 = diff(se,n(1));
pse_n2 = diff(se,n(2));
pse_r = diff(se,r);

%ccode(grad,'File','grad.c')
disp("dist cost: ")
ccode(se)

data = [se;pse_n1;pse_n2];
ccode(data,'File','grad.hpp')

disp("dist gradient n1: ")
ccode(pse_n1)
disp("dist gradient n2: ")
ccode(pse_n2)
disp("radius gradient: ")
ccode(pse_r)

%
declare = ['double '];
for i=2:31
   declare = [declare,['t',num2str(i),', ']]; 
end
declare = [declare(1:end-2),';']

