/*
  c = im_correlation(im, x_im, y_im, vp, xs, ys);

  MEX file to compute correlation arrays of (vp(1,:),vp(2,:),vp(3,:))
  in array im with limits x_im, y_im.

  Jinwook Huh, 03/2016
  <jinwookh@seas.upenn.edu>
*/

#include "mex.h"
#include <math.h>
#include "Transform.h" 
#include "THOROPKinematics.h"

//void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])

{



  
  /* Check arguments */
  if (nrhs < 3)
    mexErrMsgTxt("Need three input arguments.");

 // if (!mxIsInt8(prhs[0])) {
 //   mexErrMsgTxt("Map needs to be int8 array");
 // }

  double *im0 = (double *)mxGetData(prhs[0]);
  double *im1 = (double *)mxGetData(prhs[1]);
//  double *im2 = (double *)mxGetData(prhs[2]);

  int nx = mxGetM(prhs[0]);
  int ny = mxGetN(prhs[0]);

  std::vector<double> qArm(7);     
  std::vector<double> pArm(7);     // = lua_checkvector(L, 1);
  std::vector<double> qArmOrg(7);  // = lua_checkvector(L, 2);


  for (int i = 0; i < 7; i++) {
    pArm[i] = im0[i];
    qArmOrg[i] = im1[i];
  }
  
 printf("%i\n",  nx);
 printf("%i\n",  ny);
 printf("%f, %f\n",  pArm[0], pArm[6]);
 //printf("%f\t",  pArm[0]);

  Transform trans;
  //double translate = trans.translate(prhs[0],prhs[1],prhs[2]);
  trans.translate(10.0, 10.0, 10.0);

  plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *cpr = mxGetPr(plhs[0]);
  printf("%f\n",  trans.getZ());
  cpr[0] = trans.getZ();





/*
 int np = mxGetN(prhs[3]);

  double *xs = mxGetPr(prhs[4]);
  int nxs = mxGetNumberOfElements(prhs[4]);

  double *ys = mxGetPr(prhs[5]);
  int nys = mxGetNumberOfElements(prhs[5]);

  plhs[0] = mxCreateDoubleMatrix(nxs, nys, mxREAL);
  double *cpr = mxGetPr(plhs[0]);
  for (int i = 0; i < nxs*nys; i++) {
    cpr[i] = 0.0;
  }
*/



  
//  std::vector<double> qArm(7);
//  std::vector<double> pArm(7);     // = lua_checkvector(L, 1);
//  std::vector<double> qArmOrg(7);  // = lua_checkvector(L, 2);
  double shoulderYaw = 0.0;         //luaL_optnumber(L, 3,0.0);
  double bodyPitch = 0.0;           //luaL_optnumber(L, 4,0.0);
  std::vector<double> qWaist(7);   // = lua_checkvector(L, 5);

  //Now we can use custom hand x/y offset (for claws)
  double handOffsetXNew = 0.0; //luaL_optnumber(L, 6,handOffsetX);
  double handOffsetYNew = 0.0; //luaL_optnumber(L, 7,handOffsetY);
  double handOffsetZNew = 0.0; //luaL_optnumber(L, 8,handOffsetZ);
  
  int flip_shoulderroll = 0; //luaL_optnumber(L, 9,0);

  Transform trArm = transform6D(&pArm[0]);    
  qArm = THOROP_kinematics_inverse_l_arm_7(trArm,&qArmOrg[0],shoulderYaw,bodyPitch,&qWaist[0], handOffsetXNew, handOffsetYNew, handOffsetZNew, flip_shoulderroll);


/*  
std::vector<double>
THOROP_kinematics_inverse_arm_7(Transform trArm, int arm, const double *qOrg, double shoulderYaw, 
  double bodyPitch, const double *qWaist, double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll) 
*/
/*
qLArmTarget = Body.get_inverse_larm(
    vector.zeros(7),
    Config.arm.trLArm0,
   Config.arm.ShoulderYaw0[1],
   mcm.get_stance_bodyTilt(),{0,0},true)

*/
/*
Body.get_inverse_larm = function( qL, trL, lShoulderYaw, bodyTilt, qWaist,ignore_hand_offset)
  local shoulder_flipped = 0
  if qL[2]>math.pi/2 then shoulder_flipped=1 end
  local hand_offset = mcm.get_arm_lhandoffset()
  if ignore_hand_offset then hand_offset={0,0,0} end

  local qL_target = Kinematics.inverse_l_arm_7(
    trL,qL,
    lShoulderYaw or qL[3],
    bodyTilt or mcm.get_stance_bodyTilt(),
    qWaist or Body.get_waist_command_position(),
    hand_offset[1],hand_offset[2],hand_offset[3],
    shoulder_flipped
    )

  return qL_target end
end
*/

/*
static int inverse_l_arm_7(lua_State *L) {




  
  std::vector<double> qArm;
  std::vector<double> pArm = lua_checkvector(L, 1);
  std::vector<double> qArmOrg = lua_checkvector(L, 2);
  double shoulderYaw = luaL_optnumber(L, 3,0.0);
  double bodyPitch = luaL_optnumber(L, 4,0.0);
  std::vector<double> qWaist = lua_checkvector(L, 5);

  //Now we can use custom hand x/y offset (for claws)
  double handOffsetXNew = luaL_optnumber(L, 6,handOffsetX);
  double handOffsetYNew = luaL_optnumber(L, 7,handOffsetY);
  double handOffsetZNew = luaL_optnumber(L, 8,handOffsetZ);
  
  int flip_shoulderroll = luaL_optnumber(L, 9,0);

  Transform trArm = transform6D(&pArm[0]);    
  qArm = THOROP_kinematics_inverse_l_arm_7(trArm,&qArmOrg[0],shoulderYaw,bodyPitch,&qWaist[0],
    handOffsetXNew, handOffsetYNew, handOffsetZNew, flip_shoulderroll);
  lua_pushvector(L, qArm);
  return 1;
}
*/







//trans

//Transform THOROP_kinematics_forward_l_arm_7(const double *q, double bodyPitch, const double *qWaist, 
//  double handOffsetXNew, double handOffsetYNew, double handOffsetZNew)



  //plhs[0] = 1;
  //printf("%f\n",  translate);

  //return translate;

}

