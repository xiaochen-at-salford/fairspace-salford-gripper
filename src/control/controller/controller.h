#pragma once

namespace fairspace {
namespace control {

class Controller 
{
 public:
  Controller() = default;

  virtual ~Controller() = default;

  //TODO
  virtual void init() = 0;

  virtual void compute_control_command() = 0;

  virtual void reset() = 0;

  virtual void name() const = 0;

  virtual void stop() = 0;
}

} // namespace control
} // namespace fairspace