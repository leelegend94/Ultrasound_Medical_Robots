function [image,mask] = vesselSim(r,Hvw,Hdw,PLOT3D)
%
% Hvw: HT from vessel to world frame
% Hdw: HT from US detector to world frame
%

%Generate unit cylinder
t = r*ones(250,1);
[X,Y,Z] = cylinder(t,100);

X = X*1.2;
Z = (Z-0.5)*5;

%Transform 
% R = eul2rotm([pi/3,0,-pi/2.2]);
% T = [2;-1;-5];
% H = [R T; 0 0 0 1];

[Xc,Yc,Zc] = transformSurf(Hvw,X,Y,Z);

%Define ultrasonic detector
width = 2;
depth = 2;

px_depth = 512;
px_width = px_depth/depth*width;


if(PLOT3D)
    
%     tmp = [reshape(Xc,1,[]); reshape(Yc,1,[]); reshape(Zc,1,[])];
%     tmp = reshape(tmp,[],1);
    %visualize the vessel
    figure
    surf(Xc,Yc,Zc)
    axis equal
    hold on
    
    %visualize the image plane
    X = repmat(-31:32,128,1)/64*width;
    Y = zeros(size(X));
    Z = repmat(0:127,64,1)'/128*depth;
    [Xim,Yim,Zim] = transformSurf(Hdw,X,Y,Z);

    surf(Xim,Yim,Zim)
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    hold off
end

%transform all vesssel 3d points to detector frame 
P_d = Hdw\[reshape(Xc,1,[]);reshape(Yc,1,[]);reshape(Zc,1,[]);ones(1,numel(Xc))];
validPoint = P_d([1,3],abs(P_d(2,:))<=0.01);

validPoint(1,:) = round(validPoint(1,:)/width*px_width);
validPoint(2,:) = round(validPoint(2,:)/depth*px_depth);

% ellipse fitting for the cutting edge
A = [validPoint(1,:).^2',(validPoint(1,:).*validPoint(2,:))',validPoint(2,:).^2',validPoint',ones(size(validPoint,2),1)];
[~,~,V] = svd(A);
w = V(:,size(V,2))/V(size(V,1),size(V,2));

image = zeros(px_depth,px_width);
label_seg = zeros(px_depth,px_width);

% image label (mask)
for i=1:px_depth
    for j=1:px_width
        x = round(j-px_width/2);
        y = i;
        if([x^2,x*y,y^2,x,y,1]*w<0)
            label_seg(i,j)=1;
        end
    end
end

% adding US-specific texture to the image
I = 6*imnoise(image,'salt & pepper', 0.1).*rand(size(image));
I(label_seg==1) = 0;
I = imgaussfilt(I,[0.5 8]);
I = I+0.1*rand(px_depth,px_width);
I = imgaussfilt(I,1);
I(I>1) = 1;

%return
image = I;
mask = label_seg;
end