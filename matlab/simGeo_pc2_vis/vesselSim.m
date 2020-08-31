function validPoint = vesselSim(Xc,Yc,Zc,Hdw,PLOT3D)
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

% px_depth = 512;
% px_width = px_depth/depth*width;


if(PLOT3D)
    
%     tmp = [reshape(Xc,1,[]); reshape(Yc,1,[]); reshape(Zc,1,[])];
%     tmp = reshape(tmp,[],1);
    %visualize the vessel
    figure
    surf(Xc,Yc,Zc)
    axis equal
    hold on
    
    %visualize the image plane
    X = repmat(linspace(-width/2,wvalidPointidth/2,20),20,1);
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
validPoint_ = P_d(1:3,abs(P_d(2,:))<=4e-3);

% validPoint(1,:) = round(validPoint_(1,:)/width*px_width);
% validPoint(2,:) = round(validPoint_(2,:)/depth*px_depth);

validPoint = validPoint_(1:3,:);

% % ellipse fitting for the cutting edge
% A = [validPoint(1,:).^2',(validPoint(1,:).*validPoint(2,:))',validPoint(2,:).^2',validPoint',ones(size(validPoint,2),1)];
% [~,~,V] = svd(A);
% w = V(:,size(V,2))/V(size(V,1),size(V,2));
% 
% image = zeros(px_depth,px_width);
% label_seg = zeros(px_depth,px_width);
% 
% % image label (mask)
% for i=1:px_depth
%     for j=1:px_width
%         x = round(j-px_width/2);
%         y = i;
%         if([x^2,x*y,y^2,x,y,1]*w<0)
%             label_seg(i,j)=1;
%         end
%     end
% end
end