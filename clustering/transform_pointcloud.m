function [points_arm] = transform_pointcloud(points)

points(:, 4) = 1;

cam_in_neck = [0;10;80] * 0.001; 
% neck_offset = [0;-234;150] * 0.001; 
neck_offset = [-200;100;350] * 0.001; 

theta = 40 * pi / 180;
yaw_mat = eye(4);
yaw_mat(1:3, 1:3) = [cos(theta), 0, -sin(theta); 
    0, 1, 0;
    sin(theta), 0, cos(theta)];

T1 = eye(4);
T1(1:3, 4) = cam_in_neck;
T2 = eye(4);
T2(1:3, 4) = neck_offset;

cam_mat = [0, 0, 1, 0;
           -1, 0, 0, 0;
           0, -1, 0, 0;
           0, 0, 0, 1];

points_arm = (T2 * yaw_mat' * T1 * cam_mat * points')';

