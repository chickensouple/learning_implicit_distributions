    addpath('IK_mex');
    t_delta = 0.10;
    DEG_TO_RAD = pi/180;

    hand_offset  =  [0.10, 0, 0];
    qR = zeros(7,1);
    bodyTilt = 0;
    qWaist = zeros(1, 2) *DEG_TO_RAD;
    shoulder_flipped = 0;


    qRArm = [1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481];
    fR = MexArmKinematics('r_arm_torso_7', qRArm, 0, qWaist, t_delta, 0, 0);

    qR_target = MexArmKinematics('THOROP_kinematics_inverse_r_arm_7',fR, qRArm, 0.10472, bodyTilt, qWaist, hand_offset, shoulder_flipped)
    


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    addpath('transforms');


    %%%%% extend the length of arm

    length1 = 0.261; 
    length2 = 0.252; 
    length3 = 0.25;  
    length4 = 0.04;

    % thres = 0.20;    
    % thres2 = 0.07;   %


    bodyPitch = 0;
    shoulderOffsetY = 0.234;
    shoulderOffsetZ = 0.165;
    %shoulderOffsetZ = 0.0;

    %originOffsetZ = 0.165;
    elbowOffsetX =   0.030; 


    upperArmLengthR = 0.261; 
    lowerArmLengthR = 0.252; 
    upperArmLengthL = 0.261; 
    lowerArmLengthL = 0.252; 

    handOffsetX = 0.150;
    %handOffsetX = 0.350;  %webot
    handOffsetY = 0;
    handOffsetZ = 0; 

    q = qR_target;
    arm_index = 1;

    if arm_index == 1
        %%%%%%%%% Left arm
        t = eye(4,4);
        t = Transform_translateY(t, shoulderOffsetY);
        t = Transform_translateZ(t, shoulderOffsetZ);
        arm_pt0 = t(1:3,4);
        t =  t * Transform_mDH(-pi/2, 0, q(1), 0);
        t =  t * Transform_mDH(pi/2, 0, pi/2+q(2), 0);
        t = t * Transform_mDH(pi/2, 0, pi/2+q(3), upperArmLengthL);
        arm_pt1 = t(1:3,4);
        t = t * Transform_mDH(pi/2, elbowOffsetX, q(4), 0);
        arm_pt2 = t(1:3,4);
        t = t * Transform_mDH(-pi/2, -elbowOffsetX, -pi/2+q(5), lowerArmLengthL);
        arm_pt3 = t(1:3,4);
        t = t * Transform_mDH(-pi/2, 0, q(6), 0);
        t = t * Transform_mDH(pi/2, 0, q(7), 0);
        t =  t * Transform_mDH(-pi/2, 0, -pi/2, 0);
        t = Transform_translateX(t, handOffsetX);
        t = Transform_translateY(t, -handOffsetY);
        t =  Transform_translateZ(t, handOffsetZ);    
        arm_pt4 = t(1:3,4);

        d = Transform_position6D(t);

    else

            %%%%%%%%% Right arm
        t = eye(4,4);
        t = Transform_translateY(t, -shoulderOffsetY);
        t = Transform_translateZ(t, shoulderOffsetZ);
        arm_pt0 = t(1:3,4);
        t =  t * Transform_mDH(-pi/2, 0, q(1), 0);
        t =  t * Transform_mDH(pi/2, 0, pi/2+q(2), 0);
        t = t * Transform_mDH(pi/2, 0, pi/2+q(3), upperArmLengthL);
        arm_pt1 = t(1:3,4);
        t = t * Transform_mDH(pi/2, elbowOffsetX, q(4), 0);
        arm_pt2 = t(1:3,4);
        t = t * Transform_mDH(-pi/2, -elbowOffsetX, -pi/2+q(5), lowerArmLengthL);
        arm_pt3 = t(1:3,4);
        t = t * Transform_mDH(-pi/2, 0, q(6), 0);
        t = t * Transform_mDH(pi/2, 0, q(7), 0);
        t =  t * Transform_mDH(-pi/2, 0, -pi/2, 0);
        t = Transform_translateX(t, handOffsetX);
        t = Transform_translateY(t, -handOffsetY);
        t =  Transform_translateZ(t, handOffsetZ);    
        arm_pt4 = t(1:3,4);

        d = Transform_position6D(t);    

    end


    pt1 = arm_pt0;
    pt2 = arm_pt1;
    pt3 = arm_pt2;
    pt4 = arm_pt3;
    pt5 = arm_pt4;

    a = 0:0.2:1.0;  % 0:0.2:1;
    a = a';
    %a = repmat(a,1,3);

    b=(pt2 - pt1);
    mani_set1 = [a*b(1)+pt1(1) a*b(2)+pt1(2) a*b(3)+pt1(3)];
    b =(pt3-pt2);
    mani_set2 = [a*b(1)+pt2(1) a*b(2)+pt2(2) a*b(3)+pt2(3)];
    b =(pt4-pt3);
    mani_set3 = [a*b(1)+pt3(1) a*b(2)+pt3(2) a*b(3)+pt3(3)];
    b =(pt5-pt4);
    mani_set4 = [a*b(1)+pt4(1) a*b(2)+pt4(2) a*b(3)+pt4(3)];

    mani_set = [mani_set1;mani_set2;mani_set3;mani_set4];


    figure; plot3(mani_set(:,1),mani_set(:,2),mani_set(:,3),'g*')
    hold on
    grid on
    axis equal
