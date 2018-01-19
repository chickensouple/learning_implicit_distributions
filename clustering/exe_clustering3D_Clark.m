% close all
% load drillmodel_full   
clc; clear all; close all

pc_num = 10;

load_file = strcat('pointcloud', num2str(pc_num), '.mat');
store_file = strcat('processed_', num2str(pc_num), '.mat');

load(load_file)
points = transform_pointcloud(points);


PLOT_THRES = 60;
samples = points(randperm(length(points),30000),1:4);
Q = samples(:,1:3);
%color = samples(:,4);

MINX = min(Q(:,1));
MAXX = max(Q(:,1));
MINY = min(Q(:,2));
MAXY = max(Q(:,2));
MINZ = min(Q(:,3));
MAXZ = max(Q(:,3));

Q = Q';
Q = Q';
figure(2);
plot3(Q(:,1),Q(:,2),Q(:,3),'.');
hold on;
Q = Q';
grid on;

[Centers, betas, Theta,num_centers, Means, Sigmas] = runRBFN3D([Q']);


save_struct = struct('points', points(:, 1:3), 'means', Means, 'sigmas', Sigmas);
save(store_file, "save_struct")

for fig_k =1:num_centers
   plotcov3(Means(fig_k,:), Sigmas(:,:,fig_k)); 
end


xlabel('x');  %-0.5:1.5
ylabel('y');  %-1:1
zlabel('z'); %-1.5:0.5

%%%%%%%%%%%% check points within threshold
x = MINX:0.02:MAXX;
y = MINY:0.02:MAXY;
z = MINZ:0.02:MAXZ;
[X,Y,Z] = meshgrid(x,y,z);
TX = [X(:) Y(:) Z(:)];
result = compute_probability(TX,  Means, Sigmas,Theta);
idx = find(result>max(result)/PLOT_THRES);
plot3(TX(idx,1),TX(idx,2),TX(idx,3),'r.');
grid on;

%F = X.^2 + Y.^2 + Z.^2;


% 
%     
% rbf_centers = Centers;
% rbf_betas = betas;
% rbf_thetas = Theta;
% epsilon = 0.05;
% epsilon2 = 1.1;
% 
% if isempty(Centers) || num_centers(2) == 0
%     
%     ct = [];
%     ri = [];
% else
%     ct = Centers(num_centers(1)+1:end,:)*180/pi;
%     ri = abs(Theta(num_centers(1)+2:end,2))*200./betas(num_centers(1)+1:end);
%     %ri = 200./betas(num_centers(1)+1:end);
% 
% end
% %ri = abs(1)*200./betas(num_centers(1)+1:end);
% 
% maker_size1 = 5;
% 
% circle = @(x, y, r) rectangle('Position', [x-r, y-r, 2*r, 2*r], ...
%     'Curvature', [1 1]);
% 
% 
% if VORNOI_PLOT_TRIGGER == 1 && ~isempty(Centers)
%     hfig = figure(611);
%     hold on
%     axis equal
%     axis([-10 370 -10 370])
%     daspect([1 1 1])
% 
%     daspect([1 1 1])
% 
%     box on
% 
%     set(gca,'xtick',[])
%     set(gca,'ytick',[])
% 
%     ax = gca;
%     ax.Position = [0 0 1 1];
% 
%     for pix = 1:size(ct,1)
% 
%         hold on
%         plot(ct(pix,1),ct(pix,2),'.','Color',[1 0 0],'MarkerSize',maker_size1);
%         circle(ct(pix,1),ct(pix,2),ri(pix))
%     end
% 
% 
%     x = Centers(num_centers(1)+1:end,1)*180/pi;
%     y = Centers(num_centers(1)+1:end,2)*180/pi;
%     %[vx,vy] = voronoi(x,y);
%     %plot(x,y,'r+',vx,vy,'b-')
%     plot(x,y,'r+')
%     %plot(vx,vy,'k*')
%     %xlim([min(x) max(x)])
%     %ylim([min(y) max(y)])
%     plot(t_col_pt(:,1),t_col_pt(:,2),'m.','MarkerSize',10);
%     
% end
%  
% 
% 
%     hfig = figure(612);
%     hold on
%     axis equal
%     axis([-10 370 -10 370])
%     daspect([1 1 1])
% 
%     daspect([1 1 1])
% 
%     box on
% 
%     set(gca,'xtick',[])
%     set(gca,'ytick',[])
% 
%     ax = gca;
%     ax.Position = [0 0 1 1];
% 
%     
% for d_idx = 1:num_centers(2)
%     
%     mean = Means(num_centers(1) + d_idx,:)*180/pi;
%     sig = Sigmas(:,:,(num_centers(1) + d_idx))*180/pi*10;
%     plotcov2(mean',sig');
%     
%     
% end
%     
%     plot(t_col_pt(:,1),t_col_pt(:,2),'m.','MarkerSize',10);
% 
%     close all
%     
%     figure(101); hold on;
%     mean = [251.7555  280.3564];    
%     sig =[5.0635   -9.1635;-9.1635   18.2255];
%     plotcov2(mean',sig');
%     navi_pt = [245 270];
%     navi_pt = [260 240];
% 
%     hold on; plot(navi_pt(1),navi_pt(2),'b*');
%     
%     chol_sig = chol(sig);
%     [chol_sig,~,~] = svd(sig);
%     trans_navi_pt = (chol_sig^-1)*(navi_pt -mean)';
%     figure(100); hold on; plot(trans_navi_pt(1),trans_navi_pt(2),'b*');
%     
%     one_point = [247.4 282.8];
%     trans_one_pt = (chol_sig^-1)*(one_point -mean)';
%     figure(100); plot(trans_one_pt(1),trans_one_pt(2),'r*');
%     radi = norm(trans_one_pt);
% 
%     circle = @(x, y, r) rectangle('Position', [x-r, y-r, 2*r, 2*r], ...
%     'Curvature', [1 1]);
%     figure(100);circle(0,0,radi); axis equal
%     close_pt = trans_navi_pt/norm(trans_navi_pt) * radi;
%     figure(100); hold on; plot(close_pt(1),close_pt(2),'g*');
%     
%     retrans_pt = mean + (chol_sig*close_pt)';
%     figure(101); plot(retrans_pt(1),retrans_pt(2),'m*')
%     test_line = [navi_pt;mean];
%     figure(101); plot(test_line(:,1),test_line(:,2));
