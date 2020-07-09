clear
clc

PREVIEW = true;
VIS_VESSEL = true;

tftree = rostf;

pub_image = rospublisher('us_image','sensor_msgs/Image');
msg_image = rosmessage(pub_image);

msg_image.Encoding = 'mono8';

%init
R = eul2rotm([pi/3,0,-pi/2.2]);
T = [0.6; 0.1;-0.1];
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

[Xc,Yc,Zc] = genVessel(0.01,Hvw);

if(VIS_VESSEL)
    msg_PC2 = rosmessage("sensor_msgs/PointCloud2");
    pub_pc = rospublisher('vessel',"sensor_msgs/PointCloud2");
    msg_PC2.Header.FrameId = "iiwa_link_0";
    msg_PC2.Height = 1;
    msg_PC2.Width = numel(Xc);
    msg_PC2.IsBigendian = false;
    msg_PC2.IsDense = false;
    msg_PC2.PointStep = 4*4;
    msg_PC2.RowStep = msg_PC2.PointStep*numel(Xc);
    f1 = rosmessage("sensor_msgs/PointField");
    f1.Name = "x";
    f1.Offset = 0;
    f1.Datatype = 7;
    f1.Count = 1;
    
    f2 = rosmessage("sensor_msgs/PointField");
    f2.Name = "y";
    f2.Offset = 4;
    f2.Datatype = 7;
    f2.Count = 1;
    
    f3 = rosmessage("sensor_msgs/PointField");
    f3.Name = "z";
    f3.Offset = 8;
    f3.Datatype = 7;
    f3.Count = 1;
    
    f4 = rosmessage("sensor_msgs/PointField");
    f4.Name = "intensity";
    f4.Offset = 12;
    f4.Datatype = 7;
    f4.Count = 1;
    
    msg_PC2.Fields = [f1,f2,f3,f4];
    
    msg_PC2.Data = typecast( single(reshape([reshape(Xc,1,[]);reshape(Yc,1,[]);reshape(Zc,1,[]);ones(1,numel(Xc))],1,[])), 'uint8');
end

background = double(rgb2gray(imread('~/workspace/us_robot/DataSet/realDataSet/linear/background/frame0005.jpg')))/255;

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
[image,label] = vesselSim(Xc,Yc,Zc,Hdw,false,background);
writeImage(msg_image,im2uint8(image));
if(PREVIEW)
    h1.CData = image;
    h2.CData = label;
    drawnow
end

if(VIS_VESSEL)
    %msg_PC2.Header.Stamp = rostime("now");
    send(pub_pc,msg_PC2);
end

send(pub_image,msg_image);
dt = toc;
fprintf(repmat('\b',1,19)); 
fprintf("update rate: %2.2f\n",1/dt);
t = t+dt;
end

