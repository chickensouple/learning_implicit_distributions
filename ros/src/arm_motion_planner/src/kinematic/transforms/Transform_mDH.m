function T_mtx = Transform_mDH(alpha, a, theta, d)

  %T_mtx = eye(4,4);
  
  %t1 = T_mtx;
  ca = cos(alpha);
  sa = sin(alpha);
  ct = cos(theta);
  st = sin(theta);
  t1 = [ct, -st, 0, a; st*ca, ct*ca, -sa,-sa*d; st*sa, ct*sa, ca, ca*d; 0, 0, 0, 1];
%   
%   t1(1,1) = ct; t1(1,2) = -st; t1(1,3) = 0; t1(1,4) = a;
%   t1(2,1) = st*ca; t1(2,2) = ct*ca; t1(2,3) = -sa; t1(2,4) = -sa*d;
%   t1(3,1) = st*sa; t1(3,2) = ct*sa; t1(3,3) = ca; t1(3,4) = ca*d;
%   t1(4,1) = 0; t1(4,2) = 0; t1(4,3) = 0; t1(4,4) = 1;
%   
  T_mtx = t1;
 
%  this->translateX(a).rotateX(alpha).translateZ(d).rotateZ(theta);


% % % %%%%%%%%%%%%%%%% classic
% % % function trans_mtx = DH_transform_matrix(theta, di, ai, alpha)
% % % 
% % % trans_mtx = [cos(theta) -1*cos(alpha)*sin(theta)  sin(alpha)*sin(theta) ai*cos(theta);
% % %              sin(theta) cos(alpha)*cos(theta)  -1*sin(alpha)*cos(theta) ai*sin(theta);
% % %                   0          sin(alpha)             cos(alpha)                di;
% % %                   0              0                      0                      1];
% % % 
% % % end