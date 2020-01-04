#ifndef RAGNAR_KINEMATICS_H
#define RAGNAR_KINEMATICS_H

#include <eigen3/Eigen/Dense>

namespace ragnar_kinematics
{

bool inverse_kinematics(const double cartesian_mm[], double actuator_mm[]);
bool forward_kinematics(const double actuator_mm[], double cartesian_mm[]);

struct IntermediatePoints
{
  const static int N_ARMS = 4;
  const static int N_DIMS = 3;
  typedef Eigen::Matrix<double, N_DIMS, N_ARMS> ArmMatrixd;

  // [  arm1x   arm2x   arm3x   arm4x  ]
  // [  arm1y   arm2y   arm3y   arm4y  ]
  // [  arm1z   arm2z   arm3z   arm4z  ]


  ArmMatrixd A; // position of motors

  ArmMatrixd B; // position of elbow
  ArmMatrixd B1;
  ArmMatrixd B2;

  ArmMatrixd C; // position of mount to eff plate
  ArmMatrixd C1;
  ArmMatrixd C2;

  // Alignment support
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool calcIntermediatePoints(const double actuator_mm[], const double cartesian_mm[],
                            IntermediatePoints& pts);

}

#endif // RAGNAR_KINEMATICS_H

