#include "THOROPKinematics.h"


Transform THOROP_kinematics_forward_l_arm_7(const double *q, double bodyPitch, const double *qWaist, 
  double handOffsetXNew, double handOffsetYNew, double handOffsetZNew){
//FK for 7-dof arm (pitch-roll-yaw-pitch-yaw-roll-yaw)
  Transform t;
  
  t = t
    .rotateY(bodyPitch)
    //.rotateZ(qWaist[0]).rotateY(qWaist[1])

    .translateZ(-originOffsetZ)
    .rotateY(qWaist[1]).rotateZ(qWaist[0])    
    .translateZ(originOffsetZ)
    .translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLengthL)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLengthL)
    .mDH(-PI/2, 0, q[5], 0)
    .mDH(PI/2, 0, q[6], 0)
    .mDH(-PI/2, 0, -PI/2, 0)
    .translateX(handOffsetXNew)
    .translateY(-handOffsetYNew)
    .translateZ(handOffsetZNew);    

  return t;
}


Transform
THOROP_kinematics_forward_r_arm_7(const double *q, double bodyPitch, const double *qWaist,
   double handOffsetXNew, double handOffsetYNew, double handOffsetZNew) {
//New FK for 6-dof arm (pitch-roll-yaw-pitch-yaw-roll)
  Transform t;

  t = t
    .rotateY(bodyPitch)
    //.rotateZ(qWaist[0]).rotateY(qWaist[1])
    .translateZ(-originOffsetZ)
    .rotateY(qWaist[1]).rotateZ(qWaist[0])
    .translateZ(originOffsetZ)
    .translateY(-shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLengthR)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLengthR)
    .mDH(-PI/2, 0, q[5], 0)
    .mDH(PI/2, 0, q[6], 0)
    .mDH(-PI/2, 0, -PI/2, 0)
    .translateX(handOffsetXNew)
    .translateY(handOffsetYNew)
    .translateZ(handOffsetZNew);
  return t;
}



Transform THOROP_kinematics_forward_l_wrist(const double *q, double bodyPitch, const double *qWaist){
//FK for 7-dof arm (pitch-roll-yaw-pitch-yaw-roll-yaw)
  Transform t;
  t = t
    .rotateY(bodyPitch)
    .translateZ(-originOffsetZ)
    .rotateY(qWaist[1]).rotateZ(qWaist[0]) //Pitch and then yaw for mk2
    .translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ+originOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLengthL)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLengthL);    
  return t;
}


Transform THOROP_kinematics_forward_r_wrist(const double *q, double bodyPitch, const double *qWaist){
//New FK for 6-dof arm (pitch-roll-yaw-pitch-yaw-roll)
  Transform t;
  t = t
    .rotateY(bodyPitch)
    .translateZ(-originOffsetZ)
    .rotateY(qWaist[1]).rotateZ(qWaist[0]) //Pitch and then yaw for mk2
    .translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ+originOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLengthR)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLengthR);
  return t;
}



std::vector<double> THOROP_kinematics_inverse_wrist(Transform trWrist, int arm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist){
  //calculate shoulder and elbow angle given wrist POSITION
  // Shoulder yaw angle is given


  Transform t;

  double upperArmLength, lowerArmLength;

  //TODOTODOTODOTODO

  //Getting rid of hand, shoulder offsets
  if (arm==ARM_LEFT){
    upperArmLength = upperArmLengthL;
    lowerArmLength = lowerArmLengthL;
    t=t
      .translateZ(-shoulderOffsetZ-originOffsetZ).translateY(-shoulderOffsetY).translateX(-shoulderOffsetX)
      .rotateZ(-qWaist[0]).rotateY(-qWaist[1]).translateZ(originOffsetZ).rotateY(-bodyPitch)*trWrist;
  }else{
    upperArmLength = upperArmLengthR;
    lowerArmLength = lowerArmLengthR;
    t=t
      .translateZ(-shoulderOffsetZ-originOffsetZ).translateY(shoulderOffsetY).translateX(shoulderOffsetX)
      .rotateZ(-qWaist[0]).rotateY(-qWaist[1]).translateZ(originOffsetZ).rotateY(-bodyPitch)*trWrist;
  }

  double dUpperArm = sqrt(upperArmLength*upperArmLength+elbowOffsetX*elbowOffsetX);
  double dLowerArm = sqrt(lowerArmLength*lowerArmLength+elbowOffsetX*elbowOffsetX);
  double aUpperArm = atan(elbowOffsetX/upperArmLength);
  double aLowerArm = atan(elbowOffsetX/lowerArmLength);

//---------------------------------------------------------------------------
// Calculating elbow pitch from shoulder-wrist distance
//---------------------------------------------------------------------------
     
  double xWrist[3];
  for (int i = 0; i < 3; i++) xWrist[i]=0;
  t.apply(xWrist);

  double dWrist = xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]+xWrist[2]*xWrist[2];
  double cElbow = .5*(dWrist-dUpperArm*dUpperArm-dLowerArm*dLowerArm)/(dUpperArm*dLowerArm);
  if (cElbow > 1) cElbow = 1;
  if (cElbow < -1) cElbow = -1;

  // SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
  // We are only using the smaller one (arm more bent) 
  double elbowPitch = -acos(cElbow)-aUpperArm-aLowerArm;  

//---------------------------------------------------------------------------
// Calculate arm shoulder pitch and roll given the wrist position
//---------------------------------------------------------------------------

  //Transform m: from shoulder yaw to wrist 
  Transform m;
  m=m.rotateX(shoulderYaw).translateX(upperArmLength).translateZ(elbowOffsetX)
     .rotateY(elbowPitch).translateZ(-elbowOffsetX).translateX(lowerArmLength);

  //Now we solve the equation
  //RotY(shoulderPitch)*RotZ(ShoulderRoll)*m*(0 0 0 1)T = xWrist
  
  //Solve shoulder roll first
  //sin(shoulderRoll)*m[0][3] + cos(shoulderRoll)*m[1][3] = xWrist[1]
  //Solve for sin (roll limited to -pi/2 to pi/2)
  
  double a,b,c;
  double shoulderPitch, shoulderRoll;
  double err1,err2;

  a = m(0,3)*m(0,3) + m(1,3)*m(1,3);
  b = -m(0,3)*xWrist[1];
  c = xWrist[1]*xWrist[1] - m(1,3)*m(1,3);
  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
    shoulderRoll = 0;
  }
  else {
    double s21,s22;
    s21= (-b+sqrt(b*b-a*c))/a;
    s22= (-b-sqrt(b*b-a*c))/a;
    if (s21 > 1) s21 = 1;
    if (s21 < -1) s21 = -1;
    if (s22 > 1) s22 = 1;
    if (s22 < -1) s22 = -1;
    double shoulderRoll1 = asin(s21);
    double shoulderRoll2 = asin(s22);
    err1 = s21*m(0,3)+cos(shoulderRoll1)*m(1,3)-xWrist[1];
    err2 = s22*m(0,3)+cos(shoulderRoll2)*m(1,3)-xWrist[1];
    if (err1*err1<err2*err2) shoulderRoll = shoulderRoll1;
    else shoulderRoll = shoulderRoll2;
  }

//Now we know shoulder Roll and Yaw
//Solve for shoulder Pitch
//Eq 1: c1(c2*m[0][3]-s2*m[1][3])+s1*m[2][3] = xWrist[0]
//Eq 2: -s1(c2*m[0][3]-s2*m[1][3])+c1*m[2][3] = xWrist[2]
//OR
// s1*t1 + c1*m[2][3] = xWrist[2]
// -c1*t1 + s1*m[2][3] = xWrist[0]
  // c1 = (m[2][3] xWrist[2] - t1 xWrist[0])/ (m[2][3]^2 - t1^2 )
  // s1 = (m[2][3] xWrist[0] + t1 xWrist[2])/ (m[2][3]^2 + t1^2 )

  double s2 = sin(shoulderRoll);
  double c2 = cos(shoulderRoll);
  double t1 = -c2 * m(0,3) + s2 * m(1,3);
 
  double c1 = (m(2,3)*xWrist[2]-t1*xWrist[0]) /(m(2,3)*m(2,3) + t1*t1);
  double s1 = (m(2,3)*xWrist[0]+t1*xWrist[2]) /(m(2,3)*m(2,3) + t1*t1);

  shoulderPitch = atan2(s1,c1);

  //return joint angles
  std::vector<double> qArm(7);
  qArm[0] = shoulderPitch;
  qArm[1] = shoulderRoll;
  qArm[2] = shoulderYaw;
  qArm[3] = elbowPitch;

  qArm[4] = qOrg[4];
  qArm[5] = qOrg[5];
  qArm[6] = qOrg[6];

  return qArm;
}







  std::vector<double>
THOROP_kinematics_inverse_arm_7(Transform trArm, int arm, const double *qOrg, double shoulderYaw, 
  double bodyPitch, const double *qWaist, double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll) 
{
  // Closed-form inverse kinematics for THOR-OP 7DOF arm
  // (pitch-roll-yaw-pitch-yaw-roll-yaw)
  // Shoulder yaw angle is given

//Forward kinematics: (mk1)
/*
    .rotateY(bodyPitch)
    .rotateZ(qWaist[0]).rotateY(qWaist[1])
    .translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength)
    .mDH(-PI/2, 0, q[5], 0)
    .mDH(PI/2, 0, q[6], 0)
    .mDH(-PI/2, 0, -PI/2, 0)
    .translateX(handOffsetX)
    .translateY(-handOffsetY)
    .translateZ(handOffsetZ);
*/

  Transform t;

  double upperArmLength, lowerArmLength;

  //Getting rid of hand, shoulder offsets
  
  //mk1
  /*
  if (arm==ARM_LEFT){
    t=t
    .translateZ(-shoulderOffsetZ)
  	.translateY(-shoulderOffsetY)
    .translateX(-shoulderOffsetX)
    .rotateY(-qWaist[1])
    .rotateZ(-qWaist[0])
    .rotateY(-bodyPitch)
	*trArm
    .translateZ(-handOffsetZNew)
    .translateY(handOffsetYNew)
	  .translateX(-handOffsetXNew);
  }else{
    t=t
    .translateZ(-shoulderOffsetZ)
    .translateY(shoulderOffsetY)
    .translateX(shoulderOffsetX)
    .rotateY(-qWaist[1])
    .rotateZ(-qWaist[0])
    .rotateY(-bodyPitch)
	*trArm
    .translateZ(-handOffsetZNew)
    .translateY(-handOffsetYNew)
  	.translateX(-handOffsetXNew);
  }
*/



  //Getting rid of hand, shoulder offsets
  if (arm==ARM_LEFT){
    upperArmLength = upperArmLengthL;
    lowerArmLength = lowerArmLengthL;
  
    t=t
    .translate(-shoulderOffsetX,-shoulderOffsetY,-shoulderOffsetZ-originOffsetZ)
    .rotateZ(-qWaist[0])
    .rotateY(-qWaist[1])
    .translateZ(originOffsetZ)
    .rotateY(-bodyPitch)
  *trArm    
    .translate(-handOffsetXNew,handOffsetYNew,-handOffsetZNew);
  }else{
    upperArmLength = upperArmLengthR;
    lowerArmLength = lowerArmLengthR;
  
    t=t
    .translate(-shoulderOffsetX,shoulderOffsetY,-shoulderOffsetZ-originOffsetZ)
    .rotateZ(-qWaist[0])
    .rotateY(-qWaist[1])
    .translateZ(originOffsetZ)
    .rotateY(-bodyPitch)
  *trArm
    .translate(-handOffsetXNew,-handOffsetYNew,-handOffsetZNew);
  }
  
  double dUpperArm = sqrt(upperArmLength*upperArmLength+elbowOffsetX*elbowOffsetX);
  double dLowerArm = sqrt(lowerArmLength*lowerArmLength+elbowOffsetX*elbowOffsetX);
  double aUpperArm = atan(elbowOffsetX/upperArmLength);
  double aLowerArm = atan(elbowOffsetX/lowerArmLength);

  Transform trArmRot; //Only the rotation part of trArm

  trArmRot = trArmRot
      .rotateY(-qWaist[1])
      .rotateZ(-qWaist[0])
      .rotateY(-bodyPitch)
      *trArm
     .translateZ(-trArm(2,3))
     .translateY(-trArm(1,3))
     .translateX(-trArm(0,3));     

//---------------------------------------------------------------------------
// Calculating elbow pitch from shoulder-wrist distance
//---------------------------------------------------------------------------

  double xWrist[3];
  for (int i = 0; i < 3; i++) xWrist[i]=0;
  t.apply(xWrist);

  double dWrist = xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]+xWrist[2]*xWrist[2];
  double cElbow = .5*(dWrist-dUpperArm*dUpperArm-dLowerArm*dLowerArm)/(dUpperArm*dLowerArm);
  if (cElbow > 1) cElbow = 1;
  if (cElbow < -1) cElbow = -1;

  // SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
  // We are only using the smaller one (arm more bent) 
  double elbowPitch = -acos(cElbow)-aUpperArm-aLowerArm;  

//---------------------------------------------------------------------------
// Calculate arm shoulder pitch and roll given the wrist position
//---------------------------------------------------------------------------

  //Transform m: from shoulder yaw to wrist 
  Transform m;
  m=m.rotateX(shoulderYaw)
     .translateX(upperArmLength)
     .translateZ(elbowOffsetX)
     .rotateY(elbowPitch)
     .translateZ(-elbowOffsetX)
     .translateX(lowerArmLength);
  //Now we solve the equation
  //RotY(shoulderPitch)*RotZ(ShoulderRoll)*m*(0 0 0 1)T = xWrist
  
  //Solve shoulder roll first
  //sin(shoulderRoll)*m[0][3] + cos(shoulderRoll)*m[1][3] = xWrist[1]

  
  double a,b,c;
  double shoulderPitch, shoulderRoll;
  double err1,err2;

//Solve for sin (roll limited to -pi/2 to pi/2)
  a = m(0,3)*m(0,3) + m(1,3)*m(1,3);
  b = -m(0,3)*xWrist[1];
  c = xWrist[1]*xWrist[1] - m(1,3)*m(1,3);
  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
    shoulderRoll = 0;
  }
  else {
    double s21,s22;
    s21= (-b+sqrt(b*b-a*c))/a;
    s22= (-b-sqrt(b*b-a*c))/a;
    if (s21 > 1) s21 = 1;
    if (s21 < -1) s21 = -1;
    if (s22 > 1) s22 = 1;
    if (s22 < -1) s22 = -1;
    double shoulderRoll1 = asin(s21);
    double shoulderRoll2 = asin(s22);
    err1 = s21*m(0,3)+cos(shoulderRoll1)*m(1,3)-xWrist[1];
    err2 = s22*m(0,3)+cos(shoulderRoll2)*m(1,3)-xWrist[1];
    if (err1*err1<err2*err2) shoulderRoll = shoulderRoll1;
    else shoulderRoll = shoulderRoll2;
  }

  //Now we extend shoulder roll angle to 0~pi for left, -pi~0 for right
  //We manually choose either shoulderRoll area (normal / flipped up)
  if (flip_shoulderroll>0){
    if (arm==ARM_LEFT) shoulderRoll = PI-shoulderRoll;
    else shoulderRoll = (-PI) - shoulderRoll;
  }

//Now we know shoulder Roll and Yaw
//Solve for shoulder Pitch
//Eq 1: c1(c2*m[0][3]-s2*m[1][3])+s1*m[2][3] = xWrist[0]
//Eq 2: -s1(c2*m[0][3]-s2*m[1][3])+c1*m[2][3] = xWrist[2]
//OR
// s1*t1 + c1*m[2][3] = xWrist[2]
// -c1*t1 + s1*m[2][3] = xWrist[0]
  // c1 = (m[2][3] xWrist[2] - t1 xWrist[0])/ (m[2][3]^2 - t1^2 )
  // s1 = (m[2][3] xWrist[0] + t1 xWrist[2])/ (m[2][3]^2 + t1^2 )

  double s2 = sin(shoulderRoll);
  double c2 = cos(shoulderRoll);
  double t1 = -c2 * m(0,3) + s2 * m(1,3);
 
  double c1 = (m(2,3)*xWrist[2]-t1*xWrist[0]) /(m(2,3)*m(2,3) + t1*t1);
  double s1 = (m(2,3)*xWrist[0]+t1*xWrist[2]) /(m(2,3)*m(2,3) + t1*t1);

  shoulderPitch = atan2(s1,c1);

//---------------------------------------------------------------------------
// Now we know shoulder pich, roll, yaw and elbow pitch
// Calc the final transform for the wrist based on rotation alone
//---------------------------------------------------------------------------

  Transform tRot;
  tRot = tRot
    .rotateY(shoulderPitch).rotateZ(shoulderRoll)
    .rotateX(shoulderYaw).rotateY(elbowPitch);

  Transform tInvRot;
  tInvRot = tInvRot
	.rotateY(-elbowPitch).rotateX(-shoulderYaw)
	.rotateZ(-shoulderRoll).rotateY(-shoulderPitch);

  //Now we solve  
  // tRot * RotX(wristYaw)*RotZ(wristRoll)*RotX(wristYaw2) = trArmRot
  // or inv(tRot) * trArmRot = RotX RotZ RotX

//  Transform rotWrist = inv(tRot) * trArmRot;
  Transform rotWrist = tInvRot * trArmRot;

  double wristYaw_a, wristRoll_a, wristYaw2_a,
        wristYaw_b, wristRoll_b, wristYaw2_b;

  //Two solutions
  wristRoll_a = acos(rotWrist(0,0)); //0 to pi
  double swa = sin(wristRoll_a);
  wristYaw_a = atan2 (rotWrist(2,0)*swa,rotWrist(1,0)*swa);
  wristYaw2_a = atan2 (rotWrist(0,2)*swa,-rotWrist(0,1)*swa);

  //Filpped position
  wristRoll_b = -wristRoll_a; //-pi to 0
  double swb = sin(wristRoll_b);
  wristYaw_b = atan2 (rotWrist(2,0)*swb,rotWrist(1,0)*swb);  
  wristYaw2_b = atan2 (rotWrist(0,2)*swb,-rotWrist(0,1)*swb);

  //singular point: just use current angles
  if(swa<0.00001){
    wristYaw_a = qOrg[4];    
    wristRoll_a = qOrg[5];
    wristYaw2_a = qOrg[6];
    wristYaw_b = qOrg[4];
    wristRoll_b = qOrg[5];
    wristYaw2_b = qOrg[6];
  } 

  //Select the closest solution to current joint angle  
  bool select_a = false;
  double err_a = fmodf(qOrg[4] - wristYaw_a+5*PI, 2*PI) - PI;
  double err_b = fmodf(qOrg[4] - wristYaw_b+5*PI, 2*PI) - PI;
	
	//printf("wristYaw_a, wristYaw_b: %f, %f\n", wristYaw_a, wristYaw_b);
	//printf("err_a, err_b: %f, %f\n", err_a, err_b);
	
  if (err_a*err_a<err_b*err_b)   select_a=true;
  
  std::vector<double> qArm(7);
  qArm[0] = shoulderPitch;
  qArm[1] = shoulderRoll;
  qArm[2] = shoulderYaw;
  qArm[3] = elbowPitch;

  if (select_a==true) {
    qArm[4] = wristYaw_a;
    qArm[5] = wristRoll_a;
    qArm[6] = wristYaw2_a;
  }else{
    qArm[4] = wristYaw_b;
    qArm[5] = wristRoll_b;
    qArm[6] = wristYaw2_b;
  }
  return qArm;
}


std::vector<double>
THOROP_kinematics_inverse_arm_given_wrist(Transform trArm, int arm, const double *qOrg, double bodyPitch, const double *qWaist) 
{
  //Calculate the wrist angle given the wrist position and the target transform 

  Transform trArmRot; //Only the rotation part of trArm
  //printf("qWaist: %f %f\n",qWaist[0],qWaist[1]);
  //printf("bodyPitch: %f \n",bodyPitch);

  trArmRot = trArmRot
      .rotateY(-qWaist[1])
      .rotateZ(-qWaist[0])
      .rotateY(-bodyPitch)
      *trArm
      .translate(-trArm(0,3),-trArm(1,3),-trArm(2,3));

  //Pelvis to wrist transform  
  Transform tInvRot;
  tInvRot = tInvRot
  .rotateY(-qOrg[3])
  .rotateX(-qOrg[2])
  .rotateZ(-qOrg[1])
  .rotateY(-qOrg[0]);
  


  //Now we solve  
  // tRot * RotX(wristYaw)*RotZ(wristRoll)*RotX(wristYaw2) = trArmRot
  // or inv(tRot) * trArmRot = RotX RotZ RotX

//  Transform rotWrist = inv(tRot) * trArmRot;
  Transform rotWrist = tInvRot * trArmRot;   

  double wristYaw_a, wristRoll_a, wristYaw2_a,
        wristYaw_b, wristRoll_b, wristYaw2_b;

  //Two solutions
  wristRoll_a = acos(rotWrist(0,0)); //0 to pi
  double swa = sin(wristRoll_a);
  wristYaw_a = atan2 (rotWrist(2,0)*swa,rotWrist(1,0)*swa);
  wristYaw2_a = atan2 (rotWrist(0,2)*swa,-rotWrist(0,1)*swa);

  //Filpped position
  wristRoll_b = -wristRoll_a; //-pi to 0
  double swb = sin(wristRoll_b);
  wristYaw_b = atan2 (rotWrist(2,0)*swb,rotWrist(1,0)*swb);  
  wristYaw2_b = atan2 (rotWrist(0,2)*swb,-rotWrist(0,1)*swb);

  //singular point: just use current angles
  if(swa<0.00001){
    wristYaw_a = qOrg[4];    
    wristRoll_a = qOrg[5];
    wristYaw2_a = qOrg[6];
    wristYaw_b = qOrg[4];
    wristRoll_b = qOrg[5];
    wristYaw2_b = qOrg[6];
  } 

  //Select the closest solution to current joint angle  
  bool select_a = false;
  double err_a = fmodf(qOrg[4] - wristYaw_a+5*PI, 2*PI) - PI;
  double err_b = fmodf(qOrg[4] - wristYaw_b+5*PI, 2*PI) - PI;
  if (err_a*err_a<err_b*err_b)   select_a=true;
  
  std::vector<double> qArm(7);
  qArm[0] = qOrg[0];
  qArm[1] = qOrg[1];
  qArm[2] = qOrg[2];
  qArm[3] = qOrg[3];

  if (select_a==true) {
    qArm[4] = wristYaw_a;
    qArm[5] = wristRoll_a;
    qArm[6] = wristYaw2_a;
  }else{
    qArm[4] = wristYaw_b;
    qArm[5] = wristRoll_b;
    qArm[6] = wristYaw2_b;
  }
  return qArm;
}

std::vector<double>
THOROP_kinematics_inverse_l_arm_7(Transform trArm, const double *qOrg, double shoulderYaw , double bodyPitch, const double *qWaist,
    double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll) {
  return THOROP_kinematics_inverse_arm_7(trArm, ARM_LEFT, qOrg, shoulderYaw, bodyPitch, qWaist, handOffsetXNew, handOffsetYNew, handOffsetZNew, flip_shoulderroll);
}

std::vector<double>
THOROP_kinematics_inverse_r_arm_7(Transform trArm, const double *qOrg,double shoulderYaw, double bodyPitch, const double *qWaist,
   double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll){
  return THOROP_kinematics_inverse_arm_7(trArm, ARM_RIGHT, qOrg, shoulderYaw, bodyPitch, qWaist, handOffsetXNew, handOffsetYNew, handOffsetZNew, flip_shoulderroll);
}

std::vector<double>
THOROP_kinematics_inverse_l_wrist(Transform trWrist, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist){
  return THOROP_kinematics_inverse_wrist(trWrist, ARM_LEFT, qOrg, shoulderYaw, bodyPitch, qWaist);
}

std::vector<double>
THOROP_kinematics_inverse_r_wrist(Transform trWrist,const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist) {
  return THOROP_kinematics_inverse_wrist(trWrist, ARM_RIGHT, qOrg, shoulderYaw, bodyPitch, qWaist);
}

std::vector<double> 
THOROP_kinematics_inverse_larm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist){
  return THOROP_kinematics_inverse_arm_given_wrist(trArm, ARM_LEFT,qOrg, bodyPitch, qWaist);
}

std::vector<double> 
THOROP_kinematics_inverse_rarm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist){
  return THOROP_kinematics_inverse_arm_given_wrist(trArm, ARM_RIGHT,qOrg, bodyPitch, qWaist);
}