function [er_stat, eo_act_stat, eo_opt_stat] = plot_error(data_vessel,data_pose_raw,angle_truth,PLOT)
GT_R = 7.5;

%data cropping
data_pose = data_pose_raw(data_pose_raw.x_time>data_vessel.x_time(1) & data_pose_raw.x_time<data_vessel.x_time(end),:);
%

n_opt = [data_vessel.field_direction_x data_vessel.field_direction_y data_vessel.field_direction_z]';

quat_real = [data_pose.field_poseStamped_pose_orientation_w data_pose.field_poseStamped_pose_orientation_x data_pose.field_poseStamped_pose_orientation_y data_pose.field_poseStamped_pose_orientation_z]; 
% quat_real = [data_pose.field_pose_orientation_w data_pose.field_pose_orientation_x data_pose.field_pose_orientation_y data_pose.field_pose_orientation_z]; 

n_opt = n_opt./repmat(sqrt(sum(n_opt.^2,1)), 3,1);

angle_opt = atan2(n_opt(2,:),n_opt(1,:));

% plot(180*(angle_opt-angle_truth))

%
rotm_real = quat2tform(quat_real);
n_real = zeros(4,size(rotm_real,3));
for i = 1:size(rotm_real,3)
    n_real(:,i) = rotm_real(:,:,i)*[0;1;0;1];
end

angle_real = atan2(n_real(2,:),n_real(1,:));
% plot(180*(angle_real-angle_truth))

r = data_vessel.field_radius*10;

if PLOT
    figure()
    subplot('211')
    hold on
    plot((data_pose.x_time-data_pose.x_time(1))/1e9, 180*(angle_real-angle_truth)/pi,'DisplayName','real error','LineWidth',1)
    plot((data_vessel.x_time-data_pose.x_time(1))/1e9, 180*(angle_opt-angle_truth)/pi,'DisplayName','computed error','LineWidth',1)
    xlim([0, inf]) 
    ylim([-20, 20]) 
    xlabel('Time (s)')
    ylabel('Orientation Error (deg)')
    legend
    grid on
    grid minor
    hold off

    subplot('212')
    plot((data_vessel.x_time-data_pose.x_time(1))/1e9, r-GT_R,'LineWidth',1)
    xlim([0, inf]) 
    xlabel('Time (s)')
    ylabel('Radius Error (mm)')
    grid on
    grid minor
end


data_vessel_stat = data_vessel(data_vessel.x_time-data_vessel.x_time(1)>3e9 & data_vessel.x_time-data_vessel.x_time(1)<6e9,:);
er_stat = data_vessel_stat.field_radius*10-GT_R;
eo_act_stat = (angle_real(data_pose.x_time-data_pose.x_time(1)>3e9 & data_pose.x_time-data_pose.x_time(1)<6e9)'-angle_truth)/pi*180;
eo_opt_stat = (angle_opt(data_vessel.x_time-data_vessel.x_time(1)>3e9 & data_vessel.x_time-data_vessel.x_time(1)<6e9)'-angle_truth)/pi*180;
% g1 = repmat({'Radius'},size(er_stat));
% g2 = repmat({'Orient_{real}'},size(eo_act_stat));
% g3 = repmat({'Orient_{opt}'},size(eo_opt_stat));
% 
% subplot('222')
% boxplot([eo_act_stat;eo_opt_stat],[g2; g3])
% subplot('224')
% boxplot(er_stat,g1)
%boxplot(data_vessel(data_vessel.x_time-data_vessel.x_time(1)>2e9 & data_vessel.x_time-data_vessel.x_time(1)<6e9,:).field_radius-GT_R)
end