clc; clear all; close all
load pointcloud1.mat

points(:, 4) = 1;

points_arm = transform_pointcloud(points);

scatter3(points_arm(:,1), points_arm(:, 2), points_arm(:, 3))
xlabel('x')
ylabel('y')
zlabel('z')


