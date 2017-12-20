#ifndef THOROP7_KINEMATICS_H_
#define THOROP7_KINEMATICS_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

///////////////////////////////////////////////////
// IK header file for teddy (custom legs and arms)
///////////////////////////////////////////////////


enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

const double PI = 2*asin(1);
const double SQRT2 = sqrt(2);

//THOR-OP values, based on robotis document, and double-checked with actual robot

const double neckOffsetZ = .117;
const double neckOffsetX = 0;

//ORIGIN is at the mk1 waist position
//which is 111mm higher than mk2 waist position
//then the shoulder offset Z is the same (165mm)

const double originOffsetZ = 0.111;
const double shoulderOffsetX = 0;    
const double shoulderOffsetY = 0.234; //the same
//const double shoulderOffsetZ = 0.165; //mk1 value
const double shoulderOffsetZ2 = 0.276; //mk2 value, for reference
const double shoulderOffsetZ = shoulderOffsetZ2-originOffsetZ; //virtual shoulderoffset, the same as mk1 (0.165)
const double elbowOffsetX =   .030; 


/*
//const double upperArmLength = .261; //mk2 stock value
//const double lowerArmLength = .252;

//teddy longarms
const double upperArmLength = .261+.08; //mk2 stock value
const double lowerArmLength = .252+.08;
*/

const double upperArmLengthR = .261; //.320; //mk2 modded longarm
const double lowerArmLengthR = .252; //.312; //mk2, 6cm extended
const double upperArmLengthL = .261; //mk2 stock
const double lowerArmLengthL = .252; //mk2 stock

const double handOffsetX = 0.150; //ucla value
const double handOffsetY = 0;
const double handOffsetZ = 0; 

//const double hipOffsetY = 0.072;	//mk1 value
//const double hipOffsetZ = 0.282; 	//mk1 value
const double hipOffsetX = 0;
const double hipOffsetY = 0.105;	//mk2 value
//const double hipOffsetZ2 = 0.180; //chip real hipoffset (hip to waist), mk2 stock, for reference
const double hipOffsetZ2 = 0.160; //dale real hipoffset (hip to waist), for reference


const double hipOffsetZ = hipOffsetZ2+originOffsetZ;  //mk2 virtual hipoffset (pelvis to mk1 origin)

//Total torso height (hip to shoulder)
//mk1: 165+282 = 44.7cm
//mk2: 276+180 = 45.6cm

//Total body height (shoulder to feet)
//mk1: 0.282+0.30+0.30+0.118 + 0.165= 116.5 cm
//mk2: 0.180+0.30+0.30+0.100 + 0.276= 115.6 cm

const double thighLength = 0.30;
const double tibiaLength = 0.30;

///////////////////////////////
//Now dale has mk2 legs too
///////////////////////////////

const double kneeOffsetX = 0.00;  //chip has new legs with zero knee offset
//const double kneeOffsetX = 0.03;  //dale has old legs with knee offset





const double footHeight = 0.100;    //mk2 feet height
const double footToeX = 0.130; //from ankle to toe, mk2 stock feet
const double footHeelX = 0.130; //from ankle to heel, mk2 stock feet

//mkw lidar positions
const double chestLidarHingeX = 0.05; 
const double chestLidarX = 0; //after lidar servo
const double chestLidarZ = -0.028; //based on shoulder height
const double headLidarX = 0.10; //based on neck servo

//=================================================================
const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
const double aThigh = atan(kneeOffsetX/thighLength);
const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
const double aTibia = atan(kneeOffsetX/tibiaLength);


//=================================================================
//Those values are used to calculate the multi-body COM of the robot

const double g = 9.81;

// MK2 mass and com values

const double MassBody[2]={
	9.802+0.490, //torso and waist
	4.540,  //pelvis	
};
//torso com: (-0.0042 -0.0007 0.2378), waist com (0 0 0.0579)
const double bodyCom[2][3]={
	{-0.0040, 0, 0.2292},	 //combined com for torso and waist
	{-0.0212, 0.0002, 0.0032}
};

//lets assume that the chopstick adds 100gr
const double MassArmL[7]={
//	0.940, 0.752, 1.806, 1.124, 0.441, 0.077,0.474
		0.940, 0.752, 1.806, 1.124, 0.441, 0.077,0.574
};

//extender mass 400g, gripper mass 1200gr (w/last servo)
const double MassArmR[7]={
//	0.940, 0.752, 1.806, 1.124,       0.441, 0.077,0.474 
//	0.940, 0.752, 1.806, 1.124 + 0.400, 0.441, 0.077,1.200 //extender and hand mass added
	0.940, 0.752, 1.806, 1.124 + 0.400, 0.441, 0.077,1.600 //extender and hand mass added
};


const double InertiaArm[7][6]={
	{0.0000625, 0.0000625, 0.0000625, 0,0,0},
	{0.00180625, 0.00180625, 0.00180625, 0,0,0},
	{0.00008125, 0.00008125, 0.00008125, 0,0,0},
	{0.00050625,0.00050625, 0.00050625, 0,0,0},
	{0.00060625,0.00060625, 0.00060625, 0,0,0},
	{0.0000625,0.0000625,0.0000625, 0,0,0},
	{0.0000625,0.0000625,0.0000625, 0,0,0}
};

const double armLinkL[7][3]={
	{0,shoulderOffsetY,shoulderOffsetZ}, //waist-shoulder roll 
	{0,0,0}, //shoulder pitch-shoulder roll
	{0,0,0}, //shouder roll-shoulder yaw
	{upperArmLengthL,0,elbowOffsetX},//shoulder yaw-elbow 
	{lowerArmLengthL,0,-elbowOffsetX},//elbow to wrist yaw 1
	{0,0,0},//wrist yaw1 to wrist roll
	{0,0,0}//wrist roll to wrist yaw2
};

const double armLinkR[7][3]={
	{0,-shoulderOffsetY,shoulderOffsetZ}, //waist-shoulder roll 
	{0,0,0}, //shoulder pitch-shoulder roll
	{0,0,0}, //shouder roll-shoulder yaw
	{upperArmLengthR,0,elbowOffsetX},//shoulder yaw-elbow 
	{lowerArmLengthR,0,-elbowOffsetX},//elbow to wrist yaw 1
	{0,0,0},//wrist yaw1 to wrist roll
	{0,0,0}//wrist roll to wrist yaw2
};

//Com position from joint center
const double armComL[7][3]={
	{0,0,0},	//after shoulder pitch
	{0.0282,0,0.0},//after shoulder roll
	{0.1808,0,0.0129}, //after shoulder yaw	
	{0.1239,0,-0.0301},//after elbow pitch
	{-0.0070,0,0}, //after wrist yaw 1
	{0.0290,0,0}, //after wrist roll
	{0.0953,0,0} //after wrist yaw 2
};


//right arm com should be slightly different, but lets assume they are similar
const double armComR[7][3]={
	{0,0,0},	//after shoulder pitch
	{0.0282,0,0.0},//after shoulder roll
	{0.1808,0,0.0129}, //after shoulder yaw	
	{0.1239,0,-0.0301},//after elbow pitch
	{-0.0070,0,0}, //after wrist yaw 1
	{0.0290,0,0}, //after wrist roll
	{0.0953,0,0} //after wrist yaw 2
};

const double MassLeg[6]={
	0.935,0.911,3.322,4.165,0.911,1.616
};

const double legLink[7][3]={
	{0,hipOffsetY,-hipOffsetZ}, //waist-hipyaw
	{0,0,0}, //hip yaw-roll
	{0,0,0}, //hip roll-pitch
	{-kneeOffsetX,0,-thighLength}, //hip pitch-knee
	{kneeOffsetX,0,-tibiaLength}, //knee-ankle pitch
	{0,0,0}, //ankle pitch-ankle roll
	{0,0,-footHeight}, //ankle roll - foot bottom
};

const double llegLink0[3] = {0,hipOffsetY,-hipOffsetZ};
const double rlegLink0[3] = {0,-hipOffsetY,-hipOffsetZ};

const double legCom[12][3]={
	//left
	{-0.0280,0.0003,0.0504},	//after hip yaw
	{-0.0002,-0.0111,0.0000},	//after hip roll
	{0.0119,-0.0122,-0.1500},	//after hip pitch (upper leg)
	{0.0066,-0.0157,-0.1001},	//after knee (lower leg)
	{-0.0111,0.0002,0}, //after ankle pitch
	{0.0113,0.0018,-0.0828}, //after ankle pitch	

	//right
	{-0.0280,-0.0003,0.0504},	//after hip yaw
	{-0.0002,0.0111,0.0000},	//after hip roll
	{0.0119,0.0122,-0.1500},	//after hip pitch (upper leg)
	{0.0066,0.0157,-0.1001},	//after knee (lower leg)
	{-0.0111,-0.0002,0}, //after ankle pitch
	{0.0113,-0.0018,-0.0828}, //after ankle pitch	
};

const double InertiaLeg[12][6]={
	//left
	{0.000103125,0.000103125,0.000103125,0,0,0},
	{0.00070125,0.00070125,0.00070125,0,0,0},
	{0.002145,0.002145,0.002145,0,0,0},
	{0.00154,0.00154,0.00154,0,0,0},
	{0.00059125,0.00059125,0.00059125,0,0,0},
	{0.000708125,0.000708125,0.000708125,0,0,0},

	//right
	{0.000103125,0.000103125,0.000103125,0,0,0},
	{0.00070125,0.00070125,0.00070125,0,0,0},
	{0.002145,0.002145,0.002145,0,0,0},
	{0.00154,0.00154,0.00154,0,0,0},
	{0.00059125,0.00059125,0.00059125,0,0,0},
	{0.000708125,0.000708125,0.000708125,0,0,0}
};


///////////////////////////////////////////////////////////////////////////////////////
// COM and ZMP generation functions
///////////////////////////////////////////////////////////////////////////////////////

Transform THOROP_kinematics_forward_head(const double *q);
Transform THOROP_kinematics_forward_l_leg(const double *q);
Transform THOROP_kinematics_forward_r_leg(const double *q);

std::vector<double> THOROP_kinematics_inverse_leg(const Transform trLeg, int leg, double aShiftX, double aShiftY);
std::vector<double> THOROP_kinematics_inverse_leg_toelift(const Transform trLeg, int leg,double aShiftX, double aShiftY,int birdwalk);
std::vector<double> THOROP_kinematics_inverse_leg_heellift(const Transform trLeg, int leg, double aShiftX, double aShiftY, int birdwalk);

std::vector<double> THOROP_kinematics_inverse_r_leg(const Transform trLeg, double aShiftX, double aShiftY);
std::vector<double> THOROP_kinematics_inverse_l_leg(const Transform trLeg, double aShiftX, double aShiftY);

double THOROP_kinematics_inverse_leg_bodyheight_diff(const Transform trLeg, int leg, double aShiftX, double aShiftY);

///////////////////////////////////////////////////////////////////////////////////////
// Arm FK / IK
///////////////////////////////////////////////////////////////////////////////////////



Transform THOROP_kinematics_forward_l_arm_7(const double *q, double bodyPitch, const double *qWaist,
	double handOffsetXNew, double handOffsetYNew, double handOffsetZNew);
Transform THOROP_kinematics_forward_r_arm_7(const double *q, double bodyPitch, const double *qWaist,
	double handOffsetXNew, double handOffsetYNew, double handOffsetZNew);

std::vector<double> THOROP_kinematics_inverse_r_arm_7(
	const Transform trArm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist,
	double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll);
std::vector<double> THOROP_kinematics_inverse_l_arm_7(
	const Transform trArm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist,
	double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll);

std::vector<double> THOROP_kinematics_inverse_arm(Transform trArm, std::vector<double>& qOrg, double shoulderYaw, bool flip_shoulderroll);


///////////////////////////////////////////////////////////////////////////////////////
// Wrist FK / IK
///////////////////////////////////////////////////////////////////////////////////////

//std::vector<double> THOROP_kinematics_inverse_wrist(Transform trWrist, std::vector<double>& qOrg, double shoulderYaw);

std::vector<double> THOROP_kinematics_inverse_wrist(Transform trWrist, int arm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist); 

Transform THOROP_kinematics_forward_l_wrist(const double *q, double bodyPitch, const double *qWaist);
Transform THOROP_kinematics_forward_r_wrist(const double *q, double bodyPitch, const double *qWaist);

std::vector<double> THOROP_kinematics_inverse_r_wrist(const Transform trWrist, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist);
std::vector<double> THOROP_kinematics_inverse_l_wrist(const Transform trWrist, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist); 

std::vector<double> THOROP_kinematics_inverse_larm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist); 
	std::vector<double> THOROP_kinematics_inverse_rarm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist); 



///////////////////////////////////////////////////////////////////////////////////////
// COM and ZMP generation
///////////////////////////////////////////////////////////////////////////////////////

std::vector<double> THOROP_kinematics_calculate_com_positions(
    const double *qWaist,  const double *qLArm,   const double *qRArm,
    const double *qLLeg,   const double *qRLeg,   
    double mLHand, double mRHand, double bodyPitch,
    int use_lleg, int use_rleg, int birdwalk
    );


std::vector<double> THOROP_kinematics_calculate_zmp(const double *com0, const double *com1, 
		const double *com2,double dt0, double dt1);

int THOROP_kinematics_check_collision(const double *qLArm,const double *qRArm);
int THOROP_kinematics_check_collision_single(const double *qArm,int is_left);


void THOROP_kinematics_calculate_arm_torque(
	double* stall_torque,double* b_matrx,
	const double *rpyangle,	const double *qArm, int is_left);

void THOROP_kinematics_calculate_arm_torque_adv(
  double* stall_torque,double* acc_torque,double* acc_torque2,const double *rpyangle,
  const double *qArm,const double *qArmVel,const double *qArmAcc,double dq, int is_left);

void THOROP_kinematics_calculate_arm_jacobian(  
  double* ret, const double *qArm, const double *qWaist,const double *rpyangle, 
  double handx, double handy, double handz, int is_left);


void THOROP_kinematics_calculate_leg_torque(
	double* stall_torque,double* b_matrx,
	const double *rpyangle,	const double *qLeg,
	int isLeft, double grf, const double *support);

void THOROP_kinematics_calculate_support_leg_torque(
  double* stall_torque, double* b_matrx,
  const double *rpyangle,const double *qLeg,
  int isLeft, double grf, const double *comUpperBody);


#endif
