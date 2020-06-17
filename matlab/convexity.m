% This script shows the convexity of sum of the squared sum of cross 
% products between direction vct and point cloud
addpath ./sim_pc2_vis
%init
R = eul2rotm([pi/3,0,-pi/2.2]);
T = [0; 0; 0];
Hvw = [R T; 0 0 0 1];

[Xc,Yc,Zc] = genVessel(0.1,Hvw);
points = [reshape(Xc,1,[]);reshape(Yc,1,[]);reshape(Zc,1,[])];

[x,y,z] = sphere;

g_cost_x = zeros(size(x));
g_cost_y = zeros(size(y));
g_cost_z = zeros(size(z));

g_grad0_x = zeros(size(x));
g_grad0_y = zeros(size(y));
g_grad0_z = zeros(size(z));

g_grad1_x = zeros(size(x));
g_grad1_y = zeros(size(y));
g_grad1_z = zeros(size(z));

for i = 1:size(x,1)
    for j = 1:size(x,2)
       n = [x(i,j);y(i,j);z(i,j)];
       cost = 0;
       grad0 = 0;
       grad1 = 0;
       
       for k = 1:size(points,2)
           cp1 = points(1,k);
           cp2 = points(2,k);
           cp3 = points(3,k);
           n1 = n(1);
           n2 = n(2);
           n3 = n(3);
           t = cross(n,points(:,k));
           cost = cost + t'*t;
           grad0 = grad0 + - 2*cp2*(cp1*n2 - cp2*n1) - 2*cp3*(cp1 - cp3*n1);
           grad1 = grad1 + 2*cp1*(cp1*n2 - cp2*n1) - 2*cp3*(cp2 - cp3*n2);
       end
       cost = n*cost;
       g_cost_x(i,j) = cost(1);
       g_cost_y(i,j) = cost(2);
       g_cost_z(i,j) = cost(3);
       
       grad0_ = n*grad0;
       g_grad0_x(i,j) = grad0_(1);
       g_grad0_y(i,j) = grad0_(2);
       g_grad0_z(i,j) = grad0_(3);
       
       grad1_ = n*grad1;
       g_grad1_x(i,j) = grad1_(1);
       g_grad1_y(i,j) = grad1_(2);
       g_grad1_z(i,j) = grad1_(3);
       
    end
end

figure();
surf(g_cost_x,g_cost_y,g_cost_z)

figure();
subplot('211')
surf(g_grad0_x,g_grad0_y,g_grad0_z)
subplot('212')
surf(g_grad1_x,g_grad1_y,g_grad1_z)

