#ifndef FDCL_INTEGRAL_UTILS_HPP
#define FDCL_INTEGRAL_UTILS_HPP

#include "common_types.hpp"
#include "fdcl/matrix_utils.hpp"

#include "Eigen/Dense"

namespace fdcl {

struct integral_error_vec3
{
  Vector3 error;
  Vector3 integrand;

  integral_error_vec3(void){set_zero();}

  void integrate(const Vector3 current_integrand, const double dt){
    error += (integrand + current_integrand) * dt / 2.0;
    integrand = current_integrand;
  }
  
  void set_zero(void){
    error.setZero();
    integrand.setZero();
  }
};

struct integral_error
{
  double error;
  double integrand;

  integral_error(void){set_zero();}
  
  void integrate(const double current_integrand, const double dt){
    error += (integrand + current_integrand) * dt / 2;
    integrand = current_integrand;
  }
  
  void set_zero(void){
    error = 0.0;
    integrand = 0.0;
  }
};

}
#endif