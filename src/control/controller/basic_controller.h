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
  

}

} // namespace control
} // namespace farispace