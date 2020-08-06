function [image,mask] = vesselSim(Xc,Yc,Zc,Hdw,PLOT3D,background)
%
% Hvw: HT from vessel to world frame
% Hdw: HT from US detector to world frame
%

% %Generate unit cylinder
% t = r*ones(250,1);
% [X,Y,Z] = cylinder(t,100);
% 
% X = X*1.2;
% Z = (Z-0.5)*5;
% 
% [Xc,Yc,Zc] = transformSurf(Hvw,X,Y,Z);

%Define ultrasonic detector
width = 0.2;
depth = 0.2;

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
    X = repmat(linspace(-width/2,width/2,20),20,1);
    Y = zeros(size(X));
    Z = repmat(linspace(0,depth,20),20,1)';
    [Xim,Yim,Zim] = transformSurf(Hdw,X,Y,Z);

    surf(Xim,Yim,Zim)
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    hold off
end

%transform all vesssel 3d points to detector frame 
P_d = Hdw\[reshape(Xc,1,[]);reshape(Yc,1,[]);reshape(Zc,1,[]);ones(1,numel(Xc))];
validPoint_ = P_d([1,3],abs(P_d(2,:))<=1e-3);

validPoint(1,:) = round(validPoint_(1,:)/width*px_width);
validPoint(2,:) = round(validPoint_(2,:)/depth*px_depth);

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

MAlpha = 0.4*rand;
if(background==0)
    % adding US-specific texture to the image
    I = 6*imnoise(image,'salt & pepper', 0.1).*rand(size(image));
    I(label_seg==1) = 0;
    I = imgaussfilt(I,[0.5 8]);
    I = I+0.1*rand(px_depth,px_width);
    I = imgaussfilt(I,1);
    I(I>1) = 1;
else
    I = background;
    if(rand>0.7)
        edge_size = round(10+30*rand);
        if(rand>0.5)
            I(1:edge_size,:) = 0;
            I(end-edge_size+1:end,:) = 0;
        else
            I(:,1:edge_size) = 0;
            I(:,end-edge_size+1:end) = 0;
        end
    end
    if(sum(I(label_seg==1))>500)
        alpha_mask = label_seg;
        for i = 1:size(validPoint_,2)
            if(validPoint_(1,i)>=1 && validPoint_(2,i)>=1)
                alpha_mask(round(validPoint_(1,i)),round(validPoint_(2,i))) = 0;
            end
        end
        
        I(alpha_mask==1) = MAlpha*I(alpha_mask==1);
        I = imgaussfilt(I,[1,4]);
    else
        I = -1;
    end
end

%return
image = I;
mask = label_seg;
end