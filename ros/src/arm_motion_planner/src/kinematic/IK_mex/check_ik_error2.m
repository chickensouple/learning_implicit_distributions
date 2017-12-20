%function [in_tolerance] = check_ik_error( tr, tr_check, pos_tol, ang_tol )
function [in_tolerance, position_error, angle_error] = check_ik_error2( tr, tr_check)
    DEG_TO_RAD = 3.141592/180;
    pos_tol =0.03;
    ang_tol = 0.1*DEG_TO_RAD;

    position_error = norm(tr_check(1:3) - tr(1:3));
    angle_error =  norm(tr_check(4:6) - tr(4:6));   %% FIXME : need to consider angle

    in_tolerance = 1;

    if position_error>pos_tol
        in_tolerance = 0; 
    end

    %FIXME
    % if angle_error>ang_tol   
    %     in_tolerance=0;
    % end


end

% 
% -- Check the error from a desired transform tr
% -- to a forwards kinematics of in IK solution q
%
%
% local function check_ik_error( tr, tr_check, pos_tol, ang_tol )
% 
%   -- Tolerate a 1mm error in distance
%   pos_tol = pos_tol or 0.001
%   ang_tol = ang_tol or 0.1*DEG_TO_RAD
% 
% 	local position_error = math.sqrt(
% 	( tr_check[1]-tr[1] )^2 +
% 	( tr_check[2]-tr[2] )^2 +
% 	( tr_check[3]-tr[3] )^2 )
% 
% 	local angle_error = math.sqrt(
% 	util.mod_angle( tr_check[4]-tr[4] )^2 +
% 	util.mod_angle( tr_check[5]-tr[5] )^2 +
% 	util.mod_angle( tr_check[6]-tr[6] )^2 )
% 
% 	-- If within tolerance, return true
%   local in_tolerance = true
% 	if position_error>pos_tol then in_tolerance=false end
%   if angle_error>ang_tol then in_tolerance=false end
% 
% --  if not in_tolerance then
% if false then
%     print("IK ERROR")
%     print(string.format("tr0:%.2f %.2f %.2f %.2f %.2f %.2f tr:%.2f %.2f %.2f %.2f %.2f %.2f",
%     tr_check[1],
%     tr_check[2],
%     tr_check[3],
%     tr_check[4]*RAD_TO_DEG,
%     tr_check[5]*RAD_TO_DEG,
%     tr_check[6]*RAD_TO_DEG,
%     tr[1],
%     tr[2],
%     tr[3],
%     tr[4]*RAD_TO_DEG,
%     tr[5]*RAD_TO_DEG,
%     tr[6]*RAD_TO_DEG
%     ))
%     print(string.format("LArm: %.1f %.1f %.1f %.1f %.1f %.1f %.1f",unpack(
%       vector.new(Body.get_larm_command_position())*RAD_TO_DEG     ) ))
%     print(string.format("RArm: %.1f %.1f %.1f %.1f %.1f %.1f %.1f",unpack(
%       vector.new(Body.get_rarm_command_position())*RAD_TO_DEG     ) ))
%     print()
% --    print(string.format("perr:%.4f aerr:%.2f",position_error, angle_error*Body.RAD_TO_DEG))
%   end
% 	return in_tolerance
% end