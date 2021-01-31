% n_truth = calcNtrue();
% angle_truth = atan2(n_truth(2),n_truth(1));
angle_truth=pi/2;

% list = [1,2,4,6];
list = 1:5;


f_vessel_prf = 'vessel';
f_pose_prf = 'pose';

data_vessel = readtable('vessel.csv');

[er, eo_opt] = plot_error_sim(data_vessel,angle_truth,true);

% %boxplot
% g1 = repmat({'computed'},size(hist_er));
% g2 = repmat({'real'},size(hist_act));
% g3 = repmat({'computed'},size(hist_opt));
% 
% figure()
% subplot('121')
% boxplot(hist_er,g1)
% grid on
% ylabel('Radius Error (mm)')
% subplot('122')
% boxplot([hist_act;hist_opt],[g2; g3])
% grid on
% ylabel('Orientation Error (deg)')

% 
% %
% points = [data_vessel.field_centroid_x data_vessel.field_centroid_y data_vessel.field_centroid_z]';
% scatter3(points(1,:),points(2,:),points(3,:))
% 
% %
% hold on
% scatter3(start_point(1),start_point(2),start_point(3),'r')
% scatter3(end_point(1),end_point(2),end_point(3),'r')

