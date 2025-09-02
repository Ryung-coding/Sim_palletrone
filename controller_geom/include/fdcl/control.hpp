#ifndef FDCL_CONTROL_HPP
#define FDCL_CONTROL_HPP

#include "fdcl/common_types.hpp"
#include "fdcl/integral_utils.hpp"
#include "fdcl/matrix_utils.hpp"
#include "controller_param.h"

#include "Eigen/Dense"

namespace fdcl {

class control
{
public:
  double dt = 0.0005;  /**< Time step size in seconds */

  // for integral controller
  fdcl::integral_error_vec3 eIR; /**< Attitude integral error */
  fdcl::integral_error eI1; /**< Attitude integral error for roll axis */
  fdcl::integral_error eI2; /**< Attitude integral error for pitch axis */
  fdcl::integral_error eIy; /**< Attitude integral error for yaw axis */
  fdcl::integral_error_vec3 eIX; /**< Position integral error */

  Vector3 eR = Vector3::Zero(); /**< Attitude error */
  Vector3 eW = Vector3::Zero(); /**< Angular rate error */
  Vector3 ei = Vector3::Zero(); /**< Position integral error */
  Vector3 M = Vector3::Zero();  /**< Control moments */

  Vector3 eX = Vector3::Zero(); /**< Position error */
  Vector3 eV = Vector3::Zero(); /**< Velocity error */

  Vector3 b1 = Vector3::Zero(); /**< Direction of the first body axis */
  Vector3 b2 = Vector3::Zero(); /**< Direction of the second body axis */
  Vector3 b3 = Vector3::Zero(); /**< Direction of the third body axis */
  Vector3 b3_dot = Vector3::Zero(); /**< Desired rate of change of b3 axis */

  double f_total = 0.0;  /**< Total propeller thrust */
   
  control(
    fdcl::state_t *&state_, /**< Pointer to the current states */
    fdcl::command_t *&command_ /**< Pointer to the desired states */
  );
  control(void); // Default constructor (this is should not be used)
  ~control(void);
  
  void load_config(void);
  void attitude_control(void);
  void attitude_control_decoupled_yaw(void);
  void position_control(void);
  void output_fM(double &f, Vector3 &M);
  void output_debug(Vector3 &X);
  void integral_reset();

private:
  const double eX_norm_max_ = 2.0; /**< Saturating term of position error */

  fdcl::state_t *state = nullptr; /**< Pointer to the current states */
  fdcl::command_t *command = nullptr; /**< Pointer to the desired states */
    
  bool use_decoupled_yaw = true; /**< Whether to use decoupled-yaw controller
     * or to use regular non-decoupled control for attitude
     */

  Vector3 e1; /**< Direction of the first axis of the fixed frame */
  Vector3 e2; /**< Direction of the second axis of the fixed frame */
  Vector3 e3; /**< Direction of the third axis of the fixed frame */

  double m = 0.0;  /**< Mass of the rover (kg) */
  double g = 0.0;  /**< Gravitational acceleration (m/s^2) */

  // Attitude gains
  Matrix3 kR = Matrix3::Zero();  /**< Attitude gains */
  Matrix3 kW = Matrix3::Zero();  /**< Angular rate gains */
  double kyw = 0.0; /**< Yaw angular rate gain for decoupled-yaw controller */

  // Position gains
  Matrix3 kX = Matrix3::Zero(); /**< Position gains */
  Matrix3 kV = Matrix3::Zero(); /**< Velocity gains */

  // Integral gains
  double kIR = 0.0;  /**< Attitude integral gain */
  double ki = 0.0;  /**< Position integral gain */
  double kI = 0.0;  /**< Attitude integral gain for roll and pitch */
  double kyI = 0.0;  /**< Attitude integral gain for yaw */
  double kIX = 0.0;  /**< Position integral gains */
  double c1 = 0.0;  /**< Parameters for decoupled-yaw integral control */
  double c2 = 0.0;  /**< Parameters for decoupled-yaw integral control */
  double c3 = 0.0;  /**< Parameters for decoupled-yaw integral control */
};

}
#endif