% function betas = computebasisGaussian(test_data, centers, label)
% 
%     G_num = size(centers, 1);
%     sigmas = zeros(G_num, 1);
%     
%     for (k = 1 : G_num)
%         center = centers(k, :);
%         members = test_data((label == k), :);
%         dj = bsxfun(@minus, members, center);
%         dj = sqrt(sum(dj.^2, 2));
%         sigmas(k, :) = mean(dj);
%     end
% 
%     betas = 1 ./ (2 .* sigmas .^ 2);
%     
% end


function [means, sigmas] = computebasisGaussian(test_data, centers, label)

    [G_num, dim] = size(centers);
    sigmas = zeros(dim,dim,G_num);
    means = zeros(G_num,dim);
    
    for k = 1 : G_num
        members = test_data((label == k), :);   
        %GMModel = fitgmdist(X,2);
        GMModel = fitgmdist(members,1);
        center = GMModel.mu;
        sigma = GMModel.Sigma;
        means(k,:) = center;
        sigmas(:, :,k) = sigma;
    end
    
end


