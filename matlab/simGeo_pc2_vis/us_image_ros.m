clear
clc

TOOL_LINK = "";
VIS_VESSEL = true;

tftree = rostf;

%init
R = eul2rotm([0,0,-pi/2]);
T = [0.6; 0; 0.08];
Hvw = [R T; 0 0 0 1];

Hdw_init = [-1 0  0  T(1);
             0 1  0  T(2);
             0 0 -1  0;
             0 0  0  1];
    
waitForTransform(tftree,'world','iiwa_link_ee');


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

msg_vessel_pc2 = rosmessage("sensor_msgs/PointCloud2");
pub_vessel_pc = rospublisher('us_vessel_pointcloud',"sensor_msgs/PointCloud2");
msg_vessel_pc2.Header.FrameId = "iiwa_link_ee";
msg_vessel_pc2.Height = 1;

msg_vessel_pc2.IsBigendian = false;
msg_vessel_pc2.IsDense = false;
msg_vessel_pc2.PointStep = 3*4;

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

% f4 = rosmessage("sensor_msgs/PointField");
% f4.Name = "intensity";
% f4.Offset = 12;
% f4.Datatype = 7;
% f4.Count = 1;

% msg_vessel_pc2.Fields = [f1,f2,f3,f4];
msg_vessel_pc2.Fields = [f1,f2,f3];

r = rosrate(15);

t = 0;
reset(r);
while(1)
tic
transform = getTransform(tftree,'iiwa_link_0','iiwa_link_ee');
T = [transform.Transform.Translation.X;transform.Transform.Translation.Y;transform.Transform.Translation.Z];
quat = transform.Transform.Rotation;
R = quat2rotm([quat.W quat.X quat.Y quat.Z]);
Hdw = [R T; 0 0 0 1];

% R1 = eul2rotm([pi/10*sin(2*t) 0 pi/15*cos(2*t)]);
% T1 = [0;0;0];
% Hdw_debug = Hdw_init*[R1 T1; 0 0 0 1]; %rot wrt. current frame --> post mult.

% Hdw = Hdw_debug;
validPoint = vesselSim(Xc,Yc,Zc,Hdw,false);

msg_vessel_pc2.Width = size(validPoint,2);
msg_vessel_pc2.RowStep = msg_vessel_pc2.PointStep*size(validPoint,2);
msg_vessel_pc2.Data = typecast( single(reshape(validPoint,1,[])), 'uint8');

if(VIS_VESSEL)
    %msg_PC2.Header.Stamp = rostime("now");
    send(pub_pc,msg_PC2);
end

send(pub_vessel_pc,msg_vessel_pc2);

dt = toc;
%fprintf(repmat('\b',1,19)); 
%fprintf("update rate: %2.2f\n",1/dt);
t = t+dt;
waitfor(r);
end

