clear; clc;
%Generate unit cylinder
t = 0.01*ones(250,1);
[X,Y,Z] = cylinder(t,100);

X = X*1.2;
Z = (Z-0.5)*0.5;

%Transform 
R = eul2rotm([pi/3,0,-pi/2.2]);
T = [0.6; 0.1;-0.1];
Hvw = [R T; 0 0 0 1];

[Xc,Yc,Zc] = transformSurf(Hvw,X,Y,Z);

figure
surf(Xc,Yc,Zc)
axis equal
hold on

%Define ultrasonic detector
width = 0.2;
depth = 0.2;

px_depth = 512;
px_width = px_depth/depth*width;

Hdw_init = [-1 0  0  T(1);
         0 1  0  T(2);
         0 0 -1  0;
         0 0  0  1];
Hdw = Hdw_init*[eul2rotm([-1,0,-0.1]),zeros(3,1);0 0 0 1];
% X = repmat(-31:32,128,1)/px_width*width;
% Y = zeros(size(X));
% Z = repmat(0:127,64,1)'/px_depth*depth;
X = repmat(linspace(-width/2,width/2,20),20,1);
Y = zeros(size(X));
Z = repmat(linspace(0,depth,20),20,1)';

[Xim,Yim,Zim] = transformSurf(Hdw,X,Y,Z);

surf(Xim,Yim,Zim)
xlabel("X")
ylabel("Y")
zlabel("Z")
hold off

%transform all vesssel 3d points to detector frame 
P_d = Hdw\[reshape(Xc,1,[]);reshape(Yc,1,[]);reshape(Zc,1,[]);ones(1,numel(Xc))];
validPoint = P_d([1,3],abs(P_d(2,:))<=1e-3);
figure();
scatter(validPoint(1,:),validPoint(2,:),'.');

validPoint(1,:) = round(validPoint(1,:)/width*px_width);
validPoint(2,:) = round(validPoint(2,:)/depth*px_depth);

A = [validPoint(1,:).^2',(validPoint(1,:).*validPoint(2,:))',validPoint(2,:).^2',validPoint',ones(size(validPoint,2),1)];
[U,S,V] = svd(A);
w = V(:,size(V,2))/V(size(V,1),size(V,2));

image = zeros(px_depth,px_width);
label_edge = zeros(px_depth,px_width);
label_seg = zeros(px_depth,px_width);

for i=1:px_depth
    for j=1:px_width
        x = round(j-px_width/2);
        y = i;
        if([x^2,x*y,y^2,x,y,1]*w<0)
            label_seg(i,j)=1;
        end
    end
end

dOther = 0.3;
dVessel = 0.3;

for i=1:size(validPoint,2)
    tmpx = validPoint(1,i);
    tmpy = validPoint(2,i);
    if(tmpx<=px_width/2 && tmpx>=-px_width/2+1 && tmpy>=0 && tmpy<=px_depth-1)
        %image(tmpy,tmpx+64) = dVessel;
        label_edge(tmpy,tmpx+px_width/2) = 1;
    end 
end

% nrow = size(image,1);
% ncol = size(image,2);

% for col=1:ncol
%     curr_col = label_edge(:,col);
%     if(sum(curr_col)~=0)
%         idx_vessel = find(curr_col==1);
%         
%         for tmp=min(idx_vessel):max(idx_vessel)
%             curr_col(tmp) = 1;
%         end
% 
%         label_seg(:,col) = curr_col;
%     end
% end


%image = 0.2*rand(256,128);
I = 6*imnoise(image,'salt & pepper', 0.1).*rand(size(image));
I(label_seg==1) = 0;
I = imgaussfilt(I,[0.5 8]);
I = I+0.1*rand(px_depth,px_width);
I = imgaussfilt(I,1);
I(I>1) = 1;
figure()
imshow(I)
% %add noise repr. non-vessel low-density tissue
% for col=1:ncol
%     curr_col = image(:,col);
%     if(sum(curr_col)==0)
%         image(:,col) = dOther*rand(nrow,1);
%     else
%         idx_vessel = find(curr_col>0.1);
%         for tmp=1:min(idx_vessel)-1
%            curr_col(tmp) = dOther*rand;
%         end
%         
%         for tmp=max(idx_vessel)+1:nrow
%            curr_col(tmp) = dOther*rand;
%         end
%         image(:,col) = curr_col;
%     end
% end


% figure
% imshow(image)
% set(gca,'YDir','reverse');

