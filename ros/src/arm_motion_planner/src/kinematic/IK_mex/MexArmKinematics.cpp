/*
  Jinwook Huh, 03/2016
  <jinwookh@seas.upenn.edu>
*/

#include "mex.h"
#include <math.h>
#include "Transform.h" 
#include "THOROPKinematics.h"
#include <string.h>

//void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // read method
    char command[128];
    mxGetString(prhs[0],command,128);

    if (!strcmp(command,"l_arm_torso_7")) {
        
        /* Check arguments */
        if (nrhs < 7)
          mexErrMsgTxt("Need seven input arguments.");

        double *im1 = (double *)mxGetData(prhs[1]);
        double *im2 = (double *)mxGetData(prhs[2]);
        double *im3 = (double *)mxGetData(prhs[3]);
        double *im4 = (double *)mxGetData(prhs[4]);
        double *im5 = (double *)mxGetData(prhs[5]);
        double *im6 = (double *)mxGetData(prhs[6]);

        std::vector<double> ArmPose(6);     
        std::vector<double> q(7);     // = lua_checkvector(L, 1);
        double bodyPitch = im2[0]; 
        std::vector<double> qWaist(2);     // = lua_checkvector(L, 1);
        double handOffsetXNew = im4[0]; //luaL_optnumber(L, 6,handOffsetX);
        double handOffsetYNew = im5[0]; //luaL_optnumber(L, 7,handOffsetY);
        double handOffsetZNew = im6[0]; //luaL_optnumber(L, 8,handOffsetZ);

        for (int i = 0; i < 7; i++) {
          q[i] = im1[i];
        }
        

        for (int pp = 0; pp < 2; pp++) {
          qWaist[pp] = im3[pp];
        }

        Transform t = THOROP_kinematics_forward_l_arm_7(&q[0],bodyPitch,&qWaist[0], handOffsetXNew, handOffsetYNew, handOffsetZNew);
        ArmPose = position6D(t);
//        printf("%f, %f, %f, %f, %f, %f\n",  ArmPose[0],ArmPose[1],ArmPose[2],ArmPose[3],ArmPose[4],ArmPose[5]);


        plhs[0] = mxCreateDoubleMatrix(1, 6, mxREAL);
        double *cpr = mxGetPr(plhs[0]);

        for (int j = 0; j < 6; j++) {
          cpr[j] = ArmPose[j];
        }


    }


    else if (!strcmp(command,"r_arm_torso_7")) {       
        
        /* Check arguments */
        if (nrhs < 7)
          mexErrMsgTxt("Need seven input arguments.");

        double *im1 = (double *)mxGetData(prhs[1]);
        double *im2 = (double *)mxGetData(prhs[2]);
        double *im3 = (double *)mxGetData(prhs[3]);
        double *im4 = (double *)mxGetData(prhs[4]);
        double *im5 = (double *)mxGetData(prhs[5]);
        double *im6 = (double *)mxGetData(prhs[6]);

        std::vector<double> ArmPose(6);     
        std::vector<double> q(7);     // = lua_checkvector(L, 1);
        double bodyPitch = im2[0]; 
        std::vector<double> qWaist(2);     // = lua_checkvector(L, 1);
        double handOffsetXNew = im4[0]; //luaL_optnumber(L, 6,handOffsetX);
        double handOffsetYNew = im5[0]; //luaL_optnumber(L, 7,handOffsetY);
        double handOffsetZNew = im6[0]; //luaL_optnumber(L, 8,handOffsetZ);

        for (int i = 0; i < 7; i++) {
          q[i] = im1[i];
        }
        

        for (int pp = 0; pp < 2; pp++) {
          qWaist[pp] = im3[pp];
        }

        Transform t = THOROP_kinematics_forward_r_arm_7(&q[0],bodyPitch,&qWaist[0], handOffsetXNew, handOffsetYNew, handOffsetZNew);
        ArmPose = position6D(t);
//        printf("%f, %f, %f, %f, %f, %f\n",  ArmPose[0],ArmPose[1],ArmPose[2],ArmPose[3],ArmPose[4],ArmPose[5]);


        plhs[0] = mxCreateDoubleMatrix(1, 6, mxREAL);
        double *cpr = mxGetPr(plhs[0]);

        for (int j = 0; j < 6; j++) {
          cpr[j] = ArmPose[j];
        }



    }


    else if (!strcmp(command,"THOROP_kinematics_inverse_r_arm_7")) {       

        /* Check arguments */
        if (nrhs < 8)
          mexErrMsgTxt("Need eight input arguments.");

        double *im1 = (double *)mxGetData(prhs[1]);
        double *im2 = (double *)mxGetData(prhs[2]);
        double *im3 = (double *)mxGetData(prhs[3]);
        double *im4 = (double *)mxGetData(prhs[4]);
        double *im5 = (double *)mxGetData(prhs[5]);
        double *im6 = (double *)mxGetData(prhs[6]);
        int *im7 = (int *)mxGetData(prhs[7]);


      std::vector<double> qArm(7);     // output
      std::vector<double> pArm(7);     // = lua_checkvector(L, 1);
      std::vector<double> qArmOrg(7);  // = lua_checkvector(L, 2);


        for (int i = 0; i < 7; i++) {
            pArm[i] = im1[i];
            qArmOrg[i] = im2[i];
        }

        double shoulderYaw = im3[0];
        double bodyPitch = im4[0];;

        std::vector<double> qWaist(2);
        for (int pp = 0; pp < 2; pp++) {
          qWaist[pp] = im5[pp];
        }

        double handOffsetXNew = im6[0]; //luaL_optnumber(L, 6,handOffsetX);
        double handOffsetYNew = im6[1]; //luaL_optnumber(L, 7,handOffsetY);
        double handOffsetZNew = im6[2]; //luaL_optnumber(L, 8,handOffsetZ);

        int flip_shoulderroll = im7[0];

      Transform trArm = transform6D(&pArm[0]);    
      qArm = THOROP_kinematics_inverse_r_arm_7(trArm,&qArmOrg[0],shoulderYaw,bodyPitch,&qWaist[0], handOffsetXNew, handOffsetYNew, handOffsetZNew, flip_shoulderroll);

//  ArmPose = position6D(t);
//        printf("%f, %f, %f, %f, %f, %f\n",  qArm[0],qArm[1],qArm[2],qArm[3],qArm[4],qArm[5]);


        plhs[0] = mxCreateDoubleMatrix(1, 7, mxREAL);
        double *cpr = mxGetPr(plhs[0]);

        for (int j = 0; j < 7; j++) {
          cpr[j] = qArm[j];
        }

    }


    else if (!strcmp(command,"THOROP_kinematics_inverse_l_arm_7")) {       

        /* Check arguments */
        if (nrhs < 8)
          mexErrMsgTxt("Need eight input arguments.");

        double *im1 = (double *)mxGetData(prhs[1]);
        double *im2 = (double *)mxGetData(prhs[2]);
        double *im3 = (double *)mxGetData(prhs[3]);
        double *im4 = (double *)mxGetData(prhs[4]);
        double *im5 = (double *)mxGetData(prhs[5]);
        double *im6 = (double *)mxGetData(prhs[6]);
        int *im7 = (int *)mxGetData(prhs[7]);


      std::vector<double> qArm(7);     // output
      std::vector<double> pArm(7);     // = lua_checkvector(L, 1);
      std::vector<double> qArmOrg(7);  // = lua_checkvector(L, 2);


        for (int i = 0; i < 7; i++) {
            pArm[i] = im1[i];
            qArmOrg[i] = im2[i];
        }

        double shoulderYaw = im3[0];
        double bodyPitch = im4[0];;

        std::vector<double> qWaist(2);
        for (int pp = 0; pp < 2; pp++) {
          qWaist[pp] = im5[pp];
        }

        double handOffsetXNew = im6[0]; //luaL_optnumber(L, 6,handOffsetX);
        double handOffsetYNew = im6[1]; //luaL_optnumber(L, 7,handOffsetY);
        double handOffsetZNew = im6[2]; //luaL_optnumber(L, 8,handOffsetZ);

        int flip_shoulderroll = im7[0];

      Transform trArm = transform6D(&pArm[0]);    
      qArm = THOROP_kinematics_inverse_l_arm_7(trArm,&qArmOrg[0],shoulderYaw,bodyPitch,&qWaist[0], handOffsetXNew, handOffsetYNew, handOffsetZNew, flip_shoulderroll);

//  ArmPose = position6D(t);
//        printf("%f, %f, %f, %f, %f, %f\n",  qArm[0],qArm[1],qArm[2],qArm[3],qArm[4],qArm[5]);


        plhs[0] = mxCreateDoubleMatrix(1, 7, mxREAL);
        double *cpr = mxGetPr(plhs[0]);

        for (int j = 0; j < 7; j++) {
          cpr[j] = qArm[j];
        }

    }

}

