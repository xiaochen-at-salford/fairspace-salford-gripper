#include "../digital_filter_coefficients.h"
#include "gtest/gtest.h"

#include <vector>

#include "ros/ros.h"

using namespace fairspace::common;

class DigitalFilterCoefficientsTest : public ::testing::Test 
{
 public:
  virtual void SetUp() {}
};

TEST_F(DigitalFilterCoefficientsTest, LpfCoefficients) 
{
  double ts = 0.01;
  double cutoff_freq = 20;
  std::vector<double> den;
  std::vector<double> num;
  lpf_coefficients(ts, cutoff_freq, &den, &num);
  EXPECT_EQ(den.size(), 3);
  EXPECT_EQ(num.size(), 3);
  EXPECT_NEAR(num[0], 0.1729, 0.01);
  EXPECT_NEAR(num[1], 0.3458, 0.01);
  EXPECT_NEAR(num[2], 0.1729, 0.01);
  EXPECT_NEAR(den[0], 1.0, 0.01);
  EXPECT_NEAR(den[2], 0.2217, 0.01);
}

TEST_F(DigitalFilterCoefficientsTest, LpFirstOrderCoefficients) 
{
  double ts = 0.01;
  double settling_time = 0.005;
  double dead_time = 0.04;
  std::vector<double> den;
  std::vector<double> num;
  lp_first_order_coefficients(ts, settling_time, dead_time, &den, &num);
  EXPECT_EQ(den.size(), 2);
  EXPECT_EQ(num.size(), 5);
  EXPECT_NEAR(den[1], -0.13533, 0.01);
  EXPECT_DOUBLE_EQ(num[0], 0.0);
  EXPECT_DOUBLE_EQ(num[1], 0.0);
  EXPECT_NEAR(num[4], 1 - 0.13533, 0.01);
  dead_time = 0.0;
  lp_first_order_coefficients(ts, settling_time, dead_time, &den, &num);
  EXPECT_EQ(den.size(), 2);
  EXPECT_EQ(num.size(), 1);
  EXPECT_NEAR(den[1], -0.13533, 0.01);
  EXPECT_NEAR(num[0], 1 - 0.13533, 0.01);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "digital_filter_coefficient_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}