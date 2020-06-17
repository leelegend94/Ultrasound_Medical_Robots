clear
clc

PREVIEW = true;

tftree = rostf;

pub_image = rospublisher('us_image','sensor_msgs/Image');
msg_image = rosmessage(pub_image);

msg_image.Encoding = 'mono8';

%init
R = eul2rotm([pi/3,0,-pi/2.2]);
T = [0.6; 0.1;-1];
Hvw = [R T; 0 0 0 1];

Hdw_init = [-1 0  0  T(1);
             0 1  0  T(2);
             0 0 -1  0;
             0 0  0  1];
    
waitForTransform(tftree,'world','iiwa_link_ee');

if(PREVIEW)
    figure()
    subplot('121')
    h1 = imshow(zeros(512));
    subplot('122')
    h2 = imshow(zeros(512));
end

t = 0;
while(1)
tic
transform = getTransform(tftree,'world','iiwa_link_ee');
T = [transform.Transform.Translation.X;transform.Transform.Translation.Y;transform.Transform.Translation.Z];
quat = transform.Transform.Rotation;
R = quat2rotm([quat.W quat.X quat.Y quat.Z]);
Hdw = [R T; 0 0 0 1];

%R1 = eul2rotm([pi/10*sin(2*t) 0 pi/15*cos(2*t)]);
%T1 = [0;0;0];
%Hdw_debug = Hdw_init*[R1 T1; 0 0 0 1]; %rot wrt. current frame --> post mult.

%Hdw = Hdw_debug;
[image,label] = vesselSim(0.1,Hvw,Hdw,false);
writeImage(msg_image,im2uint8(image));
if(PREVIEW)
    h1.CData = image;
    h2.CData = label;
    drawnow
end
send(pub_image,msg_image);
dt = toc;
fprintf(repmat('\b',1,19)); 
fprintf("update rate: %2.2f\n",1/dt);
t = t+dt;
end
