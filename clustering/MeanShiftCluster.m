function [Centers,total_membership] = MeanShiftCluster(data,bandwidth)

ndata = size(data,1);
nCluster = 0;
Thres = 0.001;
Centers = [];
visited = zeros(ndata,1);
FreePtIdx = 1:ndata;
numFpt= size(FreePtIdx,2);
Votes = zeros(1,ndata);


% init cluster
new_mean = @(x,d) gaussfun(x,d,bandwidth);

while (numFpt > 0)

    new_init_idx = datasample(FreePtIdx,1);
    new_center = data(new_init_idx,:); 
    memberships = [];                  
    NewVotes    = zeros(1,ndata);    
    
    while 1 
        
        DistfromCt = sum((bsxfun(@minus, data, new_center)).^2, 2);
        withinidx = find(DistfromCt < bandwidth^2);               
        NewVotes(withinidx) = NewVotes(withinidx)+1;  
        old_center = new_center;                
        new_center = new_mean(data(withinidx,:),sqrt(DistfromCt(withinidx))); 

        memberships   = [memberships; withinidx];   
        visited(memberships) = 1; 
        
        if norm(new_center-old_center) < Thres
            % FIXME : need to update the method to merge. 2017/12/22 
            mergeidx = [];
            if ~isempty(Centers)
                DistfromClusters = sum((bsxfun(@minus,Centers, new_center)).^2,2);
                [min_val,min_idx]= min(DistfromClusters);
                if min_val < (bandwidth/2)^2
                    mergeidx = [mergeidx;min_idx];
               end
            end
                                    
            if isempty(mergeidx)
                nCluster = nCluster+1;
                Centers(nCluster,:) = new_center;
                Votes(nCluster,:)    = NewVotes;
                
            else 
                Centers(mergeidx,:)       = 0.5*(new_center+Centers(mergeidx,:));
                Votes(mergeidx,:)    = Votes(mergeidx,:) + NewVotes;
            end

            break;
        end

    end
        
    FreePtIdx = find(visited == 0);    
    numFpt = length(FreePtIdx);        

end

[~,total_membership] = max(Votes,[],1); 
total_membership = total_membership';
end

function out = gaussfun(x,d,bandWidth)

    w = exp(-(d.^2)/(2*bandWidth^2));
    w = w/sum(w);
    out = sum( bsxfun(@times, x, w ), 1);
end


