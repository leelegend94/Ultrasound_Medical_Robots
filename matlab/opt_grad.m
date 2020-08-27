clear
clc

syms n1 n2 n_d1 n_d2 a real
syms cp [3,1] real

n = sym([n1;n2;1]);
% n_d = sym([n_d1;n_d2;1]);

t1 = cross(cp,n)/norm(n);
% t2 = n-n_d;

%r = t.'*t + a*(n-n_d).'*(n-n_d);
t1_quad = t1.'*t1;
% t2_quad = t2.'*t2;

%cost1 = t1_quad;

% pd1_n1 = diff(t1_quad,n(1));
% pd1_n2 = diff(t1_quad,n(2));

pd1_n1 = diff(sqrt(t1_quad),n(1));
pd1_n2 = diff(sqrt(t1_quad),n(2));

% pd2_n1 = diff(t2_quad,n(1));
% pd2_n2 = diff(t2_quad,n(2));

grad1 = sym([pd1_n1;pd1_n2]);
% grad2 = sym([pd2_n1;pd2_n2]);

%ccode(grad,'File','grad.c')
disp("dist cost: ")
ccode(t1_quad)
disp("change cost: ")
ccode(t2_quad)
disp("dist gradient: ")
ccode(grad1)
disp("change gradient: ")
ccode(grad2)

% Ver. 2
% Minimize the variance