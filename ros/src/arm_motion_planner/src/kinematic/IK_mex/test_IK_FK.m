





clear all
close all

% dir = [1,-1,-1,1,-1,-1,1];
% larm_start = dir .* [1.61647, -0.26641, 0.905117, -1.46934, -0.00123078, 0.7, 4.13708e-05];


%[Rot, trans] = PerceptionManipulation;

ipqr = 2;


DEG_TO_RAD = pi/180;
RAD_TO_DEG = 180/pi;


%        qLArm(1) = (larm_start(2));
%        qLArm(2) = (larm_start(3));
%        qLArm(3) = (-larm_start(4));
%        qLArm(4) = (larm_start(5));
%        qLArm(5) = (-larm_start(6)+3.141592/2);
%        qLArm(6) = (-larm_start(7));
%        qLArm(7) = 0;



qLArm = [0.8660,    0.3375,   -0.2578,   -0.5446,   -0.5859,   -0.5030,    0.5622];
qWaist = zeros(1, 2) *DEG_TO_RAD;


fL = MexArmKinematics('l_arm_torso_7', qLArm, 0, qWaist, 0.0, 0, 0)
fL2 = fL;

%        trLArm012 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

%         trLArm012(1) = fL2(1);
%         trLArm012(2) = fL2(2);
%         trLArm012(3) = fL2(3);
        trLArm012 = fL2;
        
        ptheta = -15 * DEG_TO_RAD;
        ptheta = 0;
        new_trLArm012 = trLArm012;
%        trLArm012(2) = trLArm012(2) + 0.234;


        tempX = trLArm012(1);
        tempY = trLArm012(2);

        new_trLArm012(1) = tempX *cos(ptheta) - tempY*sin(ptheta);
        new_trLArm012(2) = tempX *sin(ptheta) + tempY*cos(ptheta);

        new_trLArm012
        trLArm012 = new_trLArm012;
%        trLArm012(1) = new_trLArm012(1);
%        trLArm012(2) = new_trLArm012(2) - 0.234;

         
        if ipqr == 2
            del_x = 0.1;
            del_y = 0.0;
            
        elseif ipqr == 3
            del_x = 0;
            del_y = 0.1;
            
        elseif ipqr == 4
            del_x = 0;
            del_y = -0.1;
            
        elseif ipqr == 5
            del_x = 0;
            del_y = 0.1;
        elseif ipqr ==6
            del_x = 0.0;
            del_y = -0.2;
            
        elseif ipqr == 7
            del_x = 0.0;
            del_y = 0.2;
            
        elseif ipqr == 8
            del_x = 0.0;
            del_y = -0.2;
            
        elseif ipqr == 9
            del_x = 0.0;
            del_y = 0.2;
        else
            del_x = 0.0;
            del_y = 0;
        end
        
        trLArm012(1) = trLArm012(1) +del_x;
        trLArm012(2) = trLArm012(2) +del_y;
        
        
        
%        trLArm012(3) = trLArm012(3) - 0.1;

        trLArm012

        qL = zeros(1,7);
        shoulder_flipped = 0;

        if qLArm(2)<-pi/2
          shoulder_flipped=1;
        end

        hand_offset  =  [0.0, 0, 0];

        qL = zeros(7,1);
        trL = trLArm012;
        lShoulderYaw = qLArm(3);
        bodyTilt = 0;

        qL_target = MexArmKinematics('THOROP_kinematics_inverse_l_arm_7',trL, qLArm, lShoulderYaw, bodyTilt, qWaist, hand_offset, shoulder_flipped);
        qL_target

        trL_check = MexArmKinematics('l_arm_torso_7', qL_target, 0, qWaist, 0.0, 0, 0);
        [Bool_corr, val_corr]= check_ik_error( trL, trL_check)
 
        while(Bool_corr == 0)
            lShoulderYaw = lShoulderYaw + 0.1
            qL_target = MexArmKinematics('THOROP_kinematics_inverse_l_arm_7',trL, qL, lShoulderYaw, bodyTilt, qWaist, hand_offset, shoulder_flipped);
            qL_target

            trL_check = MexArmKinematics('l_arm_torso_7', qL_target, 0, qWaist, 0.0, 0, 0)
            [Bool_corr, val_corr]= check_ik_error( trL, trL_check)
        end
        
        
       dir = [1,-1,-1,1,-1,-1,1];
       dir = [1,1,1,1,1,1,1];

       target_config = dir .* qL_target;
        fL = MexArmKinematics('l_arm_torso_7', qL_target, 0, qWaist, 0.235, 0, 0);
       