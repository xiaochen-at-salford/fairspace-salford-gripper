#include "../pid_controller.h"
#include "gtest/gtest.h"

#include <iostream>
#include <string>

#include "ros/ros.h"

using namespace fairspace::control;

//TODO(xiaochen-at-salford): Use yaml-cpp for parameter loading
class PidControllerTest : public ::testing::Test 
{
 public:
  virtual void SetUp() 
  {
    PIDConf station_pid_conf;
    station_pid_conf.integrator_enable = false;
    station_pid_conf.integrator_saturation_level = 0.1;
    station_pid_conf.output_saturation_level = 3.0;
    station_pid_conf.kp = 0.1;
    station_pid_conf.ki = 0.5;
    station_pid_conf.kd = 0.0;
    station_pid_conf_ = station_pid_conf;

    PIDConf low_speed_pid_conf;
    low_speed_pid_conf.integrator_enable = true;
    low_speed_pid_conf.integrator_saturation_level = 0.3;
    low_speed_pid_conf.output_saturation_level = 3.0;
    low_speed_pid_conf.kp = 1.5;
    low_speed_pid_conf.ki = 0.5;
    low_speed_pid_conf.kd = 0.0;
    low_speed_pid_conf.kaw = 1.0;
    low_speed_pid_conf_ = low_speed_pid_conf;
  }

 protected:
  PIDConf station_pid_conf_;
  PIDConf low_speed_pid_conf_;
};

//TODO(xiaochen-at-salford): Use yaml-cpp for parameter loading
TEST_F(PidControllerTest, StationPidController) 
{
  PIDConf pid_conf = station_pid_conf_;
  PIDController pid_controller;
  pid_controller.init(pid_conf);
  pid_controller.reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_controller.control(0.0, dt), 0.0, 1e-6);
  pid_controller.reset();
  EXPECT_NEAR(pid_controller.control(0.1, dt), 0.01, 1e-6);
  pid_controller.reset();
  double control_value = pid_controller.control(-0.1, dt);
  EXPECT_NEAR(control_value, -0.01, 1e-6);
  dt = 0.0;
  EXPECT_EQ(pid_controller.control(100, dt), control_value);
  EXPECT_FALSE(pid_controller.integrator_hold());
}

TEST_F(PidControllerTest, SpeedPidController) 
{
  PIDConf pid_conf = low_speed_pid_conf_;
  PIDController pid_controller;
  pid_controller.init(pid_conf);
  pid_controller.reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_controller.control(0.0, dt), 0.0, 1e-6);
  pid_controller.reset();
  EXPECT_NEAR(pid_controller.control(0.1, dt), 0.1505, 1e-6);
  pid_controller.reset();
  EXPECT_NEAR(pid_controller.control(-0.1, dt), -0.1505, 1e-6);
  pid_controller.reset();
  EXPECT_NEAR(pid_controller.control(500.0, dt), 750.3, 1e-6);
  EXPECT_EQ(pid_controller.integrator_saturation_status(), 1);
  pid_controller.reset();
  EXPECT_NEAR(pid_controller.control(-500.0, dt), -750.3, 1e-6);
  EXPECT_EQ(pid_controller.integrator_saturation_status(), -1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pid_control_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
