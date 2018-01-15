
function [Centers, betas, Theta,num_centers,Means, Sigmas] =  runRBFN3D(train_data)

X = train_data(:, 1:3);
%Y = train_data(:, 3);

m = size(X, 1);
[Centers, betas, Theta, num_centers, Means, Sigmas] = train3DClustering(X, 50);

