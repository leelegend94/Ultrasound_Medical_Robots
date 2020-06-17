clear
clc

syms n1 n2 n_d1 n_d2 a real
syms cp [3,1] real

n = sym([n1;n2;1]);
n_d = sym([n_d1;n_d2;1]);

t = cross(cp,n)/norm(n);

%r = t.'*t + a*(n-n_d).'*(n-n_d);
r = t.'*t;

pd_n1 = diff(r,n(1));
pd_n2 = diff(r,n2);

grad = sym([pd_n1;pd_n2]);

%ccode(grad,'File','grad.c')
ccode(r)
ccode(grad)