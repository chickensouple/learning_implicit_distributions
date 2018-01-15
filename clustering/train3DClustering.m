function [Centers, betas, Theta,num_centers, Means, Sigmas] = train3DClustering(X_train, numcenPerCat)

    m_size = size(X_train, 1);
    Centers = [];
    betas = [];    
    Means = [];
    Sigmas = [];
    num_centers = zeros(1,1);
    configuration_setting;
    

    Xc = X_train;
    %Xc = X_train((y_train == cluster_idx), :);

        % FIXME
        
    switch(CLUSTERING_METHOD)
        case 1   %Kmeans        
%               [Centroids_c, memberships_c] = kMeans(Xc, init_Centers, 100);    
            [m_clusters, center_clusters] = kmeans(Xc, numcenPerCat, 'MaxIter',100);    

        case 2
            % MeanShift
            x = Xc';
            bandwidth = 20/180*pi;
            bandwidth = 0.10;
           
            %FIXME
            [center_clusters, m_clusters] = MeanShiftCluster(Xc,bandwidth);
            %%%%%%%%%%%%%%%%%%%%%%% end MeanShift %%%%%%%%%%%%%%%%%%%%%%%%
    end
        
        smallCluster = [];
        remove_index = [];
        for (i = 1 : size(center_clusters, 1))
            if (sum(m_clusters == i) < 10)        
                smallCluster = [smallCluster; i];
                remove_index = [remove_index;find(m_clusters == i)];
            end
        end

        Xc(remove_index,:) = [];
        m_clusters(remove_index,:) = [];

        if (~isempty(smallCluster))
            
            center_clusters(smallCluster, :) = [];
            m_clusters = findClosestClusters(Xc, center_clusters);
        end
        
        betas_c = [];%computeRBFBetas(Xc, center_clusters, m_clusters);
        [means_c, sigmas_c] = computebasisGaussian(Xc, center_clusters, m_clusters);

        Centers = [Centers; center_clusters];
        betas = [betas; betas_c];
        num_centers = size(center_clusters,1);
        Means = [Means;means_c];
        Sigmas = cat(3, Sigmas, sigmas_c);
        

%        Theta = zeros(size(center_clusters,1));
        for ipx_k =1:size(center_clusters,1)
           Theta(ipx_k) = sum(m_clusters == ipx_k)/m_size;            
        end
            
end

