#ifndef CONTROLLER_GEOM_HPP
#define CONTROLLER_GEOM_HPP

#pragma once
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <algorithm>

#include "fdcl/control.hpp"
#include "controller_param.h"

struct ControlInput {
  double pos_des[3];          // [x_d, y_d, z_d] in meters
  double heading_yaw;         // desired heading yaw [rad]

  double pos[3];              // position [m]
  double vel[3];              // velocity [m/s]
  double acc[3];              // acceleration [m/s^2]
  double quat_wxyz[4];        // quaternion [w, x, y, z]
  double gyro[3];             // angular rates [rad/s]
  double inertia[9];          // 3x3 inertia (row-major)
  double arm_pos[4][5];       // 4 arms, 5 joints each (rad)
};

class ControllerGeom {
public:
  ControllerGeom();
  ~ControllerGeom();

  void state_update(const ControlInput& in);
  void compute_controller(double wrench[4], double pwm_out[4]);

private:
  fdcl::state_t * state_{nullptr};
  fdcl::command_t * command_{nullptr};
  fdcl::control fdcl_controller_;

  const double pwm_alpha_ = 46.5435;  // F = a * pwm^2 + b
  const double pwm_beta_ = 8.6111;    // F = a * pwm^2 + b
  const double zeta = 0.21496;        // b/k constant
  Eigen::Vector4d zeta_;

  Eigen::Vector4d q_B0_;              // Base rotation [arm1, arm2, arm3, arm4], rad
  Eigen::Matrix<double,6,4> DH_params_; // 6x4 DH table (rows: link 0..5; cols: a, alpha, d, theta0)
  double arm_pos_[4][5]{};

  static inline Eigen::Matrix3d quat_to_R(double w, double x, double y, double z) {
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;

    Eigen::Matrix3d R;
    R(0,0) =  1.0 - 2.0 * (yy + zz);
    R(0,1) = -2.0 * (xy - wz);
    R(0,2) = -2.0 * (xz + wy);
    R(1,0) = -2.0 * (xy + wz);
    R(1,1) =  1.0 - 2.0 * (xx + zz);
    R(1,2) =  2.0 * (yz - wx);
    R(2,0) = -2.0 * (xz - wy);
    R(2,1) =  2.0 * (yz + wx);
    R(2,2) =  1.0 - 2.0 * (xx + yy);

    return R;
  }

  static inline Eigen::Matrix4d compute_DH(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
        sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
        0,           sin(alpha),              cos(alpha),              d,
        0,           0,                        0,                      1;
    return T;
  }

  static inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    return (Eigen::Matrix3d() << 
              0.0,    -v.z(),  v.y(),
              v.z(),   0.0,   -v.x(),
            -v.y(),   v.x(),  0.0
          ).finished();
  }

};

/* ---------------- C API for ctypes (same TU) ---------------- */
#ifdef __cplusplus
extern "C" {
#endif

typedef void* controller_handle_t;

// Lifecycle
controller_handle_t controller_create();
void controller_destroy(controller_handle_t h);

void controller_state_update(controller_handle_t h,
                      const double pos_des3[3],
                      double heading_yaw,
                      const double pos3[3],
                      const double vel3[3],
                      const double acc3[3],
                      const double quat_wxyz4[4],
                      const double gyro3[3],
                      const double inertia9[9],
                      const double arm_pos20[20]);

void controller_compute_pwm(controller_handle_t h,
                           double wrench[4],
                           double pwm_out[4]);

#ifdef __cplusplus
}
#endif
/* ----------------------------------------------------------- */

#endif // CONTROLLER_GEOM_HPP



/* mac

mkdir -p build
clang++ -std=c++17 -O3 -fPIC -Icontroller_geom/include -I"$EIGEN_DIR" \
  -c controller_geom/src/controller_geom.cpp -o build/controller_geom.o

clang++ -std=c++17 -O3 -fPIC -Icontroller_geom/include -I"$EIGEN_DIR" \
  -c controller_geom/src/fdcl_control.cpp -o build/fdcl_control.o

clang++ -std=c++17 -O3 -fPIC -Icontroller_geom/include -I"$EIGEN_DIR" \
  -c controller_geom/src/fdcl_matrix_utils.cpp -o build/fdcl_matrix_utils.o

clang++ -dynamiclib -o controller_geom/libcontroller_geom.dylib \
  build/controller_geom.o build/fdcl_control.o build/fdcl_matrix_utils.o

*/


/* linux

mkdir -p build
g++ -std=c++17 -O3 -fPIC -Icontroller_geom/include -I/usr/include/eigen3 \
  -c controller_geom/src/controller_geom.cpp -o build/controller_geom.o
g++ -std=c++17 -O3 -fPIC -Icontroller_geom/include -I/usr/include/eigen3 \
  -c controller_geom/src/fdcl_control.cpp -o build/fdcl_control.o
g++ -std=c++17 -O3 -fPIC -Icontroller_geom/include -I/usr/include/eigen3 \
  -c controller_geom/src/fdcl_matrix_utils.cpp -o build/fdcl_matrix_utils.o
g++ -shared -o controller_geom/libcontroller_geom.so \
  build/controller_geom.o build/fdcl_control.o build/fdcl_matrix_utils.o
  
*/

