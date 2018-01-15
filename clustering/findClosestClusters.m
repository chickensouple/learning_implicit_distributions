function memberships = findClosestClusters(data1, cp)

G_num = size(cp, 1);
m = size(data1, 1);

dj = zeros(m, G_num);

for k = 1 : G_num
    
    diff = bsxfun(@minus, data1, cp(k, :));
    diff = diff .^ 2;
    dj(:, k) = sum(diff, 2);
end

[minVals, memberships] = min(dj, [], 2);

end




