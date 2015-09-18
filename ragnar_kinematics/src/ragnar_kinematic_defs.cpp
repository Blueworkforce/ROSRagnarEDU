#include "ragnar_kinematics/ragnar_kinematic_defs.h"
#include <cmath>
#include <iostream>

extern const float ragnar_params[32];
extern const float ragnar_cosine_cache[32];
extern const float ragnar_sine_cache[32];

const float ragnar_kinematics::ragnar_params[32] =
{
  RAGNAR_JOINT1_BASE_X,
  RAGNAR_JOINT2_BASE_X,
  RAGNAR_JOINT3_BASE_X,
  RAGNAR_JOINT4_BASE_X,

  RAGNAR_JOINT1_BASE_Y,
  RAGNAR_JOINT2_BASE_Y,
  RAGNAR_JOINT3_BASE_Y,
  RAGNAR_JOINT4_BASE_Y,

  RAGNAR_JOINT1_BASE_PAN,
  RAGNAR_JOINT2_BASE_PAN,
  RAGNAR_JOINT3_BASE_PAN,
  RAGNAR_JOINT4_BASE_PAN,

  RAGNAR_JOINT1_BASE_TILT,
  RAGNAR_JOINT2_BASE_TILT,
  RAGNAR_JOINT3_BASE_TILT,
  RAGNAR_JOINT4_BASE_TILT,

  RAGNAR_JOINT1_PRIMARY_ARM,
  RAGNAR_JOINT2_PRIMARY_ARM,
  RAGNAR_JOINT3_PRIMARY_ARM,
  RAGNAR_JOINT4_PRIMARY_ARM,

  RAGNAR_JOINT1_SECONDARY_ARM,
  RAGNAR_JOINT2_SECONDARY_ARM,
  RAGNAR_JOINT3_SECONDARY_ARM,
  RAGNAR_JOINT4_SECONDARY_ARM,

  RAGNAR_JOINT1_BRIDGED_DISTANCE,
  RAGNAR_JOINT2_BRIDGED_DISTANCE,
  RAGNAR_JOINT3_BRIDGED_DISTANCE,
  RAGNAR_JOINT4_BRIDGED_DISTANCE,

  RAGNAR_JOINT1_FIXED_BRIDGE_ANGLE, // fixed configuration
  RAGNAR_JOINT2_FIXED_BRIDGE_ANGLE,
  RAGNAR_JOINT3_FIXED_BRIDGE_ANGLE,
  RAGNAR_JOINT4_FIXED_BRIDGE_ANGLE
};

const float ragnar_kinematics::ragnar_cosine_cache[32] =
{
  0.0f,
  0.0f,
  0.0f,
  0.0f,

  0.0f,
  0.0f,
  0.0f,
  0.0f,

  // rho
  std::cos(RAGNAR_JOINT1_BASE_PAN),
  std::cos(RAGNAR_JOINT2_BASE_PAN),
  std::cos(RAGNAR_JOINT3_BASE_PAN),
  std::cos(RAGNAR_JOINT4_BASE_PAN),

  // alpha
  std::cos(RAGNAR_JOINT1_BASE_TILT),
  std::cos(RAGNAR_JOINT1_BASE_TILT),
  std::cos(RAGNAR_JOINT1_BASE_TILT),
  std::cos(RAGNAR_JOINT1_BASE_TILT),

  // params 16-27 map directly to their normal values
  RAGNAR_JOINT1_PRIMARY_ARM,
  RAGNAR_JOINT2_PRIMARY_ARM,
  RAGNAR_JOINT3_PRIMARY_ARM,
  RAGNAR_JOINT4_PRIMARY_ARM,

  RAGNAR_JOINT1_SECONDARY_ARM,
  RAGNAR_JOINT2_SECONDARY_ARM,
  RAGNAR_JOINT3_SECONDARY_ARM,
  RAGNAR_JOINT4_SECONDARY_ARM,

  RAGNAR_JOINT1_BRIDGED_DISTANCE,
  RAGNAR_JOINT2_BRIDGED_DISTANCE,
  RAGNAR_JOINT3_BRIDGED_DISTANCE,
  RAGNAR_JOINT4_BRIDGED_DISTANCE,

  std::cos(RAGNAR_JOINT1_FIXED_BRIDGE_ANGLE), // assume fixed platform
  std::cos(RAGNAR_JOINT2_FIXED_BRIDGE_ANGLE),
  std::cos(RAGNAR_JOINT3_FIXED_BRIDGE_ANGLE),
  std::cos(RAGNAR_JOINT4_FIXED_BRIDGE_ANGLE)
};

const float ragnar_kinematics::ragnar_sine_cache[32] =
{
  0.0f,
  0.0f,
  0.0f,
  0.0f,

  0.0f,
  0.0f,
  0.0f,
  0.0f,

  // rho
  std::sin(RAGNAR_JOINT1_BASE_PAN),
  std::sin(RAGNAR_JOINT2_BASE_PAN),
  std::sin(RAGNAR_JOINT3_BASE_PAN),
  std::sin(RAGNAR_JOINT4_BASE_PAN),

  // alpha
  std::sin(RAGNAR_JOINT1_BASE_TILT),
  std::sin(RAGNAR_JOINT1_BASE_TILT),
  std::sin(RAGNAR_JOINT1_BASE_TILT),
  std::sin(RAGNAR_JOINT1_BASE_TILT),

  // params 16-27 map directly to their normal values
  RAGNAR_JOINT1_PRIMARY_ARM,
  RAGNAR_JOINT2_PRIMARY_ARM,
  RAGNAR_JOINT3_PRIMARY_ARM,
  RAGNAR_JOINT4_PRIMARY_ARM,

  RAGNAR_JOINT1_SECONDARY_ARM,
  RAGNAR_JOINT2_SECONDARY_ARM,
  RAGNAR_JOINT3_SECONDARY_ARM,
  RAGNAR_JOINT4_SECONDARY_ARM,

  RAGNAR_JOINT1_BRIDGED_DISTANCE,
  RAGNAR_JOINT2_BRIDGED_DISTANCE,
  RAGNAR_JOINT3_BRIDGED_DISTANCE,
  RAGNAR_JOINT4_BRIDGED_DISTANCE,

  std::sin(RAGNAR_JOINT1_FIXED_BRIDGE_ANGLE), // assume x platform
  std::sin(RAGNAR_JOINT2_FIXED_BRIDGE_ANGLE),
  std::sin(RAGNAR_JOINT3_FIXED_BRIDGE_ANGLE),
  std::sin(RAGNAR_JOINT4_FIXED_BRIDGE_ANGLE)
};

void debugParams()
{
  std::cout << "PARAMS\n";
  for (int i = 0; i < 32; ++i)
  {
    std::cout << i << ": " << ragnar_kinematics::ragnar_params[i] << '\n';
  }

  std::cout << "SINES\n";
  for (int i = 0; i < 32; ++i)
  {
    std::cout << i << ": " << ragnar_kinematics::ragnar_sine_cache[i] << '\n';
  }

  std::cout << "COSINES\n";
  for (int i = 0; i < 32; ++i)
  {
    std::cout << i << ": " << ragnar_kinematics::ragnar_cosine_cache[i] << '\n';
  }
}
