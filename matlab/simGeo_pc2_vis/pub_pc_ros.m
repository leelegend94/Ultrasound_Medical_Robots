clear
clc

%init
R = eul2rotm([pi/3,0,-pi/2.2]);
T = [0.6; 0.1;-0.1];
Hvw = [R T; 0 0 0 1];

[Xc,Yc,Zc] = genVessel(0.1,Hvw);

%init pc2 msg
msg_PC2 = rosmessage("sensor_msgs/PointCloud2");
pub_pc = rospublisher('us_vessel_pointcloud',"sensor_msgs/PointCloud2");
msg_PC2.Header.FrameId = "world";
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
%msg_PC2.Data = typecast([0,0,0,1],'uint8');
%publish
r = rosrate(20);
while(1)
   send(pub_pc,msg_PC2);
   waitfor(r);
end
