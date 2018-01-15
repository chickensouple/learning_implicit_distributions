
function prob = compute_probability(data, Means,Sigmas,Theta)

for i = 1:size(Means,1)
Y(:,i) = mvnpdf(data,Means(i,:),Sigmas(:,:,i));
end
prob = Theta*Y';
end

