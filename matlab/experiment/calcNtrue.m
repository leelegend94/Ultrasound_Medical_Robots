function n_truth = calcNtrue()
data_start = readtable('data_start_point.csv');
data_end = readtable('data_end_point.csv');

% len_start = size(data_start,1);
% Calc Ground truth
start_point = [mean(data_start.field_centroid_x);
               mean(data_start.field_centroid_y);
               mean(data_start.field_centroid_z)];
           
end_point = [mean(data_end.field_centroid_x);
             mean(data_end.field_centroid_y);
             mean(data_end.field_centroid_z)];
         
n_truth = end_point - start_point;

n_truth = n_truth/norm(n_truth);
end