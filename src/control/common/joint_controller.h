#pragma once 

namespace fairspace {
namespace control {

class SetpointRegulator 
{
 public:
  SetpointRegulator();
  void solve_robot_inverse_kinematics();
  void compute_servo_torque();

 protected:
  PIDController pos_pid_;
  PIDController vel_pid_;
  PIDController trq_pid_;
}

} // control
} // fairspace