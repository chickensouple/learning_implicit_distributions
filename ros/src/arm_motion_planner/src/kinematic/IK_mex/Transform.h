#ifndef Transform_h_DEFINED
#define Transform_h_DEFINED

#include <math.h>
#include <stdio.h>
#include <vector>

class Transform {
public:
  Transform();
  virtual ~Transform() {}

  void clear();
  Transform &translate(double x, double y, double z);
  Transform &translate(const double *p);

  Transform &translateX(double x = 0);
  Transform &translateY(double y = 0);
  Transform &translateZ(double z = 0);
  Transform &rotateX(double a = 0);
  Transform &rotateY(double a = 0);
  Transform &rotateZ(double a = 0);
  Transform &rotateDotX(double a = 0);
  Transform &rotateDotY(double a = 0);
  Transform &rotateDotZ(double a = 0);

  Transform &translateNeg(const double *p);
  Transform &rotateDotXNeg(double a = 0);
  Transform &rotateDotYNeg(double a = 0);
  Transform &rotateDotZNeg(double a = 0);


  Transform &mDH(double alpha, double a, double theta, double d);
  void apply(double x[3]);
  void apply0(double* x);

  
  double getZ();
  const void getXYZ(double* ret) const;
  const double getZ() const;
  
  
  double& operator() (int i, int j);
  const double operator() (int i, int j) const;

 private:
  double t[4][4];
};

Transform operator* (const Transform &t1, const Transform &t2);
Transform inv (const Transform &t1);
Transform trcopy (const Transform &t1);
Transform transform6D(const double p[6]);
std::vector<double> position6D(const Transform &t1);

void getAngularVelocityTensor(const Transform &adot, const Transform &ainv, double *av);


void printTransform(Transform tr);
void printVector(std::vector<double> v);

class Jacobian {
public:
  Jacobian();
  virtual ~Jacobian() {}

  Jacobian &calculate6(
    const Transform &A, 
    const Transform &Adot0,
    const Transform &Adot1,
    const Transform &Adot2,
    const Transform &Adot3,
    const Transform &Adot4,
    const Transform &Adot5,
    double mass, const double* inertiaMatrix); 


  Jacobian &calculate7(
    const Transform &A, 
    const Transform &Adot0,
    const Transform &Adot1,
    const Transform &Adot2,
    const Transform &Adot3,
    const Transform &Adot4,
    const Transform &Adot5,
    const Transform &Adot6,
    double mass, const double* inertiaMatrix); 

  Jacobian &calculateVel7(
    const Transform &A, 
    const Transform &Adot0,
    const Transform &Adot1,
    const Transform &Adot2,
    const Transform &Adot3,
    const Transform &Adot4,
    const Transform &Adot5,
    const Transform &Adot6); 

  void clear();
  

  void calculate_b_matrix(const double*inertiaMatrix);
  void dump_b_matrix(double* ret);
  void dump_jacobian(double* ret);
  void accumulate_stall_torque(double* torque,double forcex, double forcey, double forcez);
  void print();

private:
  int num_of_joints;
  double m;
  double b[7][7];
  double v[7][3];
  double w[7][3];
};






#endif
