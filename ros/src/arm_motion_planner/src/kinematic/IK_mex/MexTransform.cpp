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
#include <string.h>

//void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // read method
    char command[128];
    mxGetString(prhs[0],command,128);

    if (!strcmp(command,"translate")) {

        Transform trans;
        trans.translate(10.0, 10.0, 10.0);

        plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
        double *cpr = mxGetPr(plhs[0]);
        cpr[0] = trans.getZ();
    }

    else if (!strcmp(command,"translateX")) {          

     /* Check arguments */
      if (nrhs < 3)
        mexErrMsgTxt("Need three input arguments.");

      double *im0 = (double *)mxGetData(prhs[1]);
      double *im1 = (double *)mxGetData(prhs[2]);

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

      Transform trans;
      trans.translate(10.0, 10.0, 10.0);

      plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
      double *cpr = mxGetPr(plhs[0]);
      printf("%f\n",  trans.getZ());
      cpr[0] = trans.getZ();

    }


  
 


}

