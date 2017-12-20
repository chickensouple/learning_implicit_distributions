

DEG_TO_RAD = pi/180;
RAD_TO_DEG = 180/pi;


qLArm = [0.8660,    0.3375,   -0.2578,   -0.5446,   -0.5859,   -0.5030,    0.5622];
lShoulderYaw = qLArm(3);
bodyTilt = 0;
hand_offset  =  [0.0, 0, 0];

  shoulder_flipped=0;

if qLArm(2)<-pi/2
  shoulder_flipped=1;
end


qWaist = zeros(1, 2) *DEG_TO_RAD;
qL = zeros(7,1);


fL = MexArmKinematics('l_arm_torso_7', qLArm, 0, qWaist, 0.0, 0, 0)
fL(1) = fL(1)+0.03;


qL_target = MexArmKinematics('THOROP_kinematics_inverse_l_arm_7',fL, qL, lShoulderYaw, bodyTilt, qWaist, hand_offset, shoulder_flipped);
qL_target

trL_check = MexArmKinematics('l_arm_torso_7', qL_target, 0, qWaist, 0.0, 0, 0);
[Bool_corr, val_corr]= check_ik_error( fL, trL_check)


