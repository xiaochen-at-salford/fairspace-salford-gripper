#include "../mean_filter.h"
#include "gtest/gtest.h"

using namespace fairspace::common;

class MeanFilterTest : public ::testing::Test 
{
 public:
  virtual void SetUp() {}
};

TEST_F(MeanFilterTest, WindowSizeOne) 
{
  MeanFilter mean_filter(1);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 4.0);
}

TEST_F(MeanFilterTest, WindowSizeTwo) 
{
  MeanFilter mean_filter(2);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 4.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(1.0), 2.5);
}

TEST_F(MeanFilterTest, OnePositiveNonZero) 
{
  MeanFilter mean_filter(5);
  EXPECT_DOUBLE_EQ(mean_filter.update(2.0), 2.0);
}

TEST_F(MeanFilterTest, OneNegativeNonZero) 
{
  MeanFilter mean_filter(5);
  EXPECT_DOUBLE_EQ(mean_filter.update(-2.0), -2.0);
}

TEST_F(MeanFilterTest, TwoPositiveNonZeros) 
{
  MeanFilter mean_filter(5);
  mean_filter.update(3.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 3.5);
}

TEST_F(MeanFilterTest, TwoNegativeNonZeros) 
{
  MeanFilter mean_filter(5);
  mean_filter.update(-3.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(-4.0), -3.5);
}

TEST_F(MeanFilterTest, OnePositiveOneNegative) 
{
  MeanFilter mean_filter(5);
  mean_filter.update(-3.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 0.5);
}

TEST_F(MeanFilterTest, NormalThree) 
{
  MeanFilter mean_filter(5);
  mean_filter.update(-3.0);
  mean_filter.update(4.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(3.0), 3.0);
}

TEST_F(MeanFilterTest, NormalFullFiveExact) 
{
  MeanFilter mean_filter(5);
  mean_filter.update(-1.0);
  mean_filter.update(0.0);
  mean_filter.update(1.0);
  mean_filter.update(2.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(3.0), 1.0);
}

TEST_F(MeanFilterTest, NormalFullFiveOver) 
{
  MeanFilter mean_filter(5);
  mean_filter.update(-1.0);
  mean_filter.update(0.0);
  mean_filter.update(1.0);
  mean_filter.update(2.0);
  mean_filter.update(3.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 2.0);
}

TEST_F(MeanFilterTest, SameNumber) 
{
  MeanFilter mean_filter(5);
  for (int i = 0; i < 10; ++i) 
  {
    EXPECT_DOUBLE_EQ(mean_filter.update(1.0), 1.0);
  }
}

TEST_F(MeanFilterTest, LargeNumber) 
{
  MeanFilter mean_filter(10);
  double ret = 0.0;
  for (int i = 0; i < 100; ++i) 
  {
    double input = static_cast<double>(i);
    ret = mean_filter.update(input);
  }
  EXPECT_DOUBLE_EQ(ret, 94.5);
}

TEST_F(MeanFilterTest, AlmostScale) 
{
  MeanFilter mean_filter(3);
  EXPECT_DOUBLE_EQ(mean_filter.update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(2.0), 1.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(9.0), 2.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(8.0), 8.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(7.0), 8.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(6.0), 7.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(5.0), 6.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 5.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(3.0), 4.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(1.0), 3.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(2.0), 2.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(3.0), 2.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 3.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(5.0), 4.0);
}

TEST_F(MeanFilterTest, ToyExample) 
{
  MeanFilter mean_filter(4);
  EXPECT_DOUBLE_EQ(mean_filter.update(5.0), 5.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(3.0), 4.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(8.0), 5.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(9.0), 6.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(7.0), 7.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(2.0), 7.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(1.0), 4.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 3.0);
}

TEST_F(MeanFilterTest, GoodMinRemoval) 
{
  MeanFilter mean_filter(2);
  EXPECT_DOUBLE_EQ(mean_filter.update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(9.0), 5.0);
  EXPECT_DOUBLE_EQ(mean_filter.update(8.0), 8.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(7.0), 7.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(6.0), 6.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(5.0), 5.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(4.0), 4.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(3.0), 3.5);
  EXPECT_DOUBLE_EQ(mean_filter.update(2.0), 2.5);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mean_filter_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
