function [angle_real,angle_opt] = plot_error(data_vessel,data_pose,n_truth)
n_opt = [data_vessel.field_direction_x data_vessel.field_direction_y data_vessel.field_direction_z]';

quat_real = [data_pose.field_poseStamped_pose_orientation_w data_pose.field_poseStamped_pose_orientation_x data_pose.field_poseStamped_pose_orientation_y data_pose.field_poseStamped_pose_orientation_z]; 

n_opt = n_opt./repmat(sqrt(sum(n_opt.^2,1)), 3,1);

angle_opt = atan2(n_opt(2,:),n_opt(1,:));
angle_truth = atan2(n_truth(2),n_truth(1));
% plot(180*(angle_opt-angle_truth))

%
rotm_real = quat2tform(quat_real);
n_real = zeros(4,size(rotm_real,3));
for i = 1:size(rotm_real,3)
    n_real(:,i) = rotm_real(:,:,i)*[0;1;0;1];
end

angle_real = atan2(n_real(2,:),n_real(1,:));
% plot(180*(angle_real-angle_truth))

figure()
hold on
grid on
plot(data_pose.x_time-data_pose.x_time(1),180*(angle_real-angle_truth)/pi,'.')
plot(data_vessel.x_time-data_pose.x_time(1),180*(angle_opt-angle_truth)/pi)

end