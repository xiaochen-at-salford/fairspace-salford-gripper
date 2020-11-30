#pragma once

#include <fstream>
#include <memory>
#include <string>

#include "Eigen/Core"

#include "controller.h"

namespace fairspace {
namespace control {

class BasicController : public Controller 
{
 public:
  BasicController();

  virtual ~BasicController();

  common::Status init() override;

  common::Status reset() override;

  void stop() override;

  std::string name() const override;

 protected:

  bool contact_state_;
  robot_model_;

  invers
  std::shared_ptr<SlidingModeSolver> sm_controller_;
  std::shared_ptr<std::vector<joint_controllers>> joints_controllers_; 

  TrackingRegulartor tr_regulator_;
    

}

} // namespace control
} // namespace farispace