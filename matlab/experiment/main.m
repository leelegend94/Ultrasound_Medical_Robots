n_truth = calcNtrue();

data_vessel = readtable('test_vessel1.csv');
data_pose = readtable('test_pose1.csv');

[angle_real,angle_opt] = plot_error(data_vessel,data_pose,n_truth);

data_vessel = readtable('test_vessel2.csv');
data_pose = readtable('test_pose2.csv');

[angle_real,angle_opt] = plot_error(data_vessel,data_pose,n_truth);

data_vessel = readtable('test_vessel3.csv');
data_pose = readtable('test_pose3.csv');

[angle_real,angle_opt] = plot_error(data_vessel,data_pose,n_truth);

data_vessel = readtable('test_vessel4.csv');
data_pose = readtable('test_pose4.csv');

[angle_real,angle_opt] = plot_error(data_vessel,data_pose,n_truth);

data_vessel = readtable('test_vessel5.csv');
data_pose = readtable('test_pose5.csv');

[angle_real,angle_opt] = plot_error(data_vessel,data_pose,n_truth);

%
points = [data_vessel.field_centroid_x data_vessel.field_centroid_y data_vessel.field_centroid_z]';
scatter3(points(1,:),points(2,:),points(3,:))

% %
% hold on
% scatter3(start_point(1),start_point(2),start_point(3),'r')
% scatter3(end_point(1),end_point(2),end_point(3),'r')

data_vessel = readtable('vessel1.csv');
data_pose = readtable('pose1.csv');

[angle_real,angle_opt] = plot_error(data_vessel,data_pose,n_truth);

plot(data_vessel.field_radius-0.75)
grid on