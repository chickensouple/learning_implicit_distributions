DEG_TO_RAD = pi/180;
RAD_TO_DEG = 180/pi;

qLArm = [0.8660,    0.3375,   -0.2578,   -0.5446,   -0.5859,   -0.5030,    0.5622]*1;
qRArm = [87.0810,   -4.0892,   -3.8030,    1.7942,   -6.5927,   -1.7344, 0] *DEG_TO_RAD;
qWaist = zeros(1, 2) *DEG_TO_RAD;


fL = MexArmKinematics('l_arm_torso_7', qLArm, 0, qWaist, 0.125, 0, 0)
%fL = vector.new(K.l_arm_torso_7(qLArm, 0, qWaist, 0.235, 0,0))
fL2 = fL;

fR = MexArmKinematics('r_arm_torso_7', qRArm, 0, qWaist, 0.125, 0, 0)
%fR = vector.new(K.r_arm_torso_7(qRArm, 0, qWaist, 0.0, 0,0))
fR2 = fR



%trRArm012 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
%new_trRArm012 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

%trRArm012(1) = fR2(1);
%trRArm012(2) = fR2(2);
%trRArm012(3) = fR2(3);
trRArm012 = fR2;

ptheta = -15 * DEG_TO_RAD;

new_trRArm012 = trRArm012;
trRArm012(2) = trRArm012(2) + 0.234;

tempX = 0;
tempY = 0;
tempX = trRArm012(1);
tempY = trRArm012(2);

new_trRArm012(1) = tempX *cos(ptheta) - tempY*sin(ptheta);
new_trRArm012(2) = tempX *sin(ptheta) + tempY*cos(ptheta);

new_trRArm012

trRArm012(1) = new_trRArm012(1);
trRArm012(2) = new_trRArm012(2) - 0.234;

trRArm012(1) = trRArm012(1) - 0.1;
trRArm012(2) = trRArm012(2) - 0.1;

trRArm012
% 
%   print('qRArm[3]',qRArm[3])
% 
%      qRArmTarget = Body.get_inverse_rarm(
%     vector.zeros(7),
%     trRArm012,
%     qRArm(3))
%     
%     
    
    
% Body.get_inverse_rarm = function( qR, trR, rShoulderYaw, bodyTilt, qWaist,ignore_hand_offset)
qR = zeros(1,7);
shoulder_flipped = 0;

if qR(2)<-pi/2
  shoulder_flipped=1;
end

%hand_offset  =  [0.235, 0, 0];
hand_offset  =  [0.0, 0, 0];

qR = zeros(7,1);
trR = trRArm012;
rShoulderYaw = qRArm(3);
bodyTilt = 0;

%qR_target = Kinematics.inverse_r_arm_7(trR, qR, rShoulderYaw, bodyTilt, qWaist, hand_offset(1),hand_offset(2),hand_offset(3), shoulder_flipped);
qR_target = MexArmKinematics('THOROP_kinematics_inverse_r_arm_7',trR, qR, rShoulderYaw, bodyTilt, qWaist, hand_offset, shoulder_flipped);
qR_target

qR_target(7) = 0; 
qR_target

temp_qRam = qR_target;
temp_qRam = temp_qRam * RAD_TO_DEG;


fR1 = MexArmKinematics('r_arm_torso_7', qR_target, 0, qWaist, 0.125, 0, 0);
%fR = vector.new(K.r_arm_torso_7(qRArm, 0, qWaist, 0.0, 0,0))
fR2 = fR1;

fR1


%     
%     
%     
%   local trR_check = Kinematics.r_arm_torso_7(
%     qR_target,
%     bodyTilt or mcm.get_stance_bodyTilt(),
%     qWaist or Body.get_waist_command_position(),
%     hand_offset[1],hand_offset[2],hand_offset[3])

%   local passed = check_rarm_bounds(qR_target) and check_ik_error( trR, trR_check)
%   if passed then return qR_target end
%   return qR_target
% end
    
    
% 
% qRArmTarget[7] = 0; 
%     temp_qRam = qRArmTarget;
% print('qRArmTarget',temp_qRam[1]*RAD_TO_DEG,temp_qRam[2]*RAD_TO_DEG,temp_qRam[3]*RAD_TO_DEG,temp_qRam[4]*RAD_TO_DEG,temp_qRam[5]*RAD_TO_DEG,temp_qRam[6]*RAD_TO_DEG, temp_qRam[7]*RAD_TO_DEG)
% print('qRArmTarget',temp_qRam[1],temp_qRam[2],temp_qRam[3],temp_qRam[4],temp_qRam[5],temp_qRam[6], temp_qRam[7])
% 
% 
% 
% fR1 = vector.new(K.r_arm_torso_7(qRArmTarget, 0, qWaist, 0.125, 0,0))
% fR21 = vector.new(T.position6D(K2.forward_rarm(qRArmTarget, qWaist)))
% 
% 
% print('Forward Right again')
% print(fR1)
% print(fR21)
% 
% --qRArmTarget = qRArm
% -----------------------------------------------------------------
% -----------------------------------------------------------------
% -----------------------------------------------------------------
% -----------------------------------------------------------------
% 
% 
% --  qLArmTarget = {110*DEG_TO_RAD,0,0,   -150*DEG_TO_RAD, 90*DEG_TO_RAD,40*DEG_TO_RAD,-90*DEG_TO_RAD}
% --  qRArmTarget = {110*DEG_TO_RAD,0,0,   -150*DEG_TO_RAD, -90*DEG_TO_RAD,-40*DEG_TO_RAD,90*DEG_TO_RAD}
% 
% 
%   --qLArmTarget = vector.new({110,0,10,-155,90,45,-76})*DEG_TO_RAD
%   qLArmTarget = vector.new({90,0,0,0,0,0,0})*DEG_TO_RAD
