#pragma once

/**
 * @file pid_controller.h
 * @brief Defines the PIDController class.
 */

#pragma once

#include "modules/control/proto/pid_conf.pb.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace farispace {
namespace control {

/**
 * @class PIDController
 * @brief A proportional-integral-derivative controller for speed and steering
 using defualt integral hold
 */
class PIDController 
{
 public:
  /**
   * @brief initialize pid controller
   * @param pid_conf configuration for pid controller
   */
  void init(const PidConf &pid_conf);

  /**
   * @brief set pid controller coefficients for the proportional,
   * integral, and derivative
   * @param pid_conf configuration for pid controller
   */
  void set_PID(const PidConf &pid_conf);

  /**
   * @brief reset variables for pid controller
   */
  void reset();

  /**
   * @brief compute control value based on the error
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  virtual double control(const double error, const double dt);

  virtual ~PIDController() = default;

  /**
   * @brief get saturation status
   * @return saturation status
   */
  int integrator_saturation_status() const;

  /**
   * @brief get status that if integrator is hold
   * @return if integrator is hold return true
   */
  bool integrator_hold() const;

  /**
   * @brief set whether to hold integrator component at its current value.
   * @param hold
   */
  void set_integrator_hold(bool hold);

 protected:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double kaw_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  double integrator_saturation_high_ = 0.0;
  double integrator_saturation_low_ = 0.0;
  bool first_hit_ = false;
  bool integrator_enabled_ = false;
  bool integrator_hold_ = false;
  int integrator_saturation_status_ = 0;
  // Only used for pid_BC_controller and pid_IC_controller
  double output_saturation_high_ = 0.0;
  double output_saturation_low_ = 0.0;
  int output_saturation_status_ = 0;
};

} // namespace control
} // namespace farispace