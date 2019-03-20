// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-03-11
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>
#include <ur_rtde_driver/calibration.h>

namespace
{
bool isApproximately(const double val1, const double val2, const double precision)
{
  return std::abs(val1 - val2) < precision;
}

TEST(UrRtdeDriver, ur10_ideal)
{
  DHRobot my_robot;
  const double pi = std::atan(1) * 4;

  // This is an ideal UR10
  // clang-format off
  my_robot.segments_.push_back(DHSegment(0.1273  , 0      , 0    , pi / 2));
  my_robot.segments_.push_back(DHSegment(0       , -0.612 , 0    , 0));
  my_robot.segments_.push_back(DHSegment(0       , -0.5723, 0    , 0.0));
  my_robot.segments_.push_back(DHSegment(0.163841, 0      , 0    , pi / 2));
  my_robot.segments_.push_back(DHSegment(0.1157  , 0      , 0    , -pi / 2));
  my_robot.segments_.push_back(DHSegment(0.0922  , 0      , 0    , 0));
  // clang-format on

  Calibration calibration(my_robot);

  Eigen::Matrix<double, 6, 1> jointvalues;
  {
    jointvalues << 0, 0, 0, 0, 0, 0;
    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);
    EXPECT_DOUBLE_EQ(fk(0, 3), my_robot.segments_[1].a_ + my_robot.segments_[2].a_);
    EXPECT_DOUBLE_EQ(fk(1, 3), -1 * (my_robot.segments_[3].d_ + my_robot.segments_[5].d_));
    EXPECT_DOUBLE_EQ(fk(2, 3), my_robot.segments_[0].d_ - my_robot.segments_[4].d_);
  }

  {
    jointvalues << M_PI_2, -M_PI_4, M_PI_2, -M_PI_4, 0, 0;
    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);
    EXPECT_DOUBLE_EQ(fk(0, 3), my_robot.segments_[3].d_ + my_robot.segments_[5].d_);
    EXPECT_DOUBLE_EQ(fk(1, 3), my_robot.segments_[1].a_ / std::sqrt(2) + my_robot.segments_[2].a_ / std::sqrt(2));
    // Because of the sqrt calculations a DOUBLE_EQ does not work here.
    EXPECT_PRED3(isApproximately, fk(2, 3),
                 my_robot.segments_[0].d_ - my_robot.segments_[1].a_ / std::sqrt(2) +
                     my_robot.segments_[2].a_ / std::sqrt(2) - my_robot.segments_[4].d_,
                 1e-16);
  }
}

TEST(UrRtdeDriver, calibration)
{
  DHRobot my_robot;
  const double pi = std::atan(1) * 4;

  // This is an ideal UR10
  // clang-format off
  //                                     d,        a,       theta, alpha
  my_robot.segments_.push_back(DHSegment(0.1273  , 0      , 0    , pi / 2));
  my_robot.segments_.push_back(DHSegment(0       , -0.612 , 0    , 0));
  my_robot.segments_.push_back(DHSegment(0       , -0.5723, 0    , 0.0));
  my_robot.segments_.push_back(DHSegment(0.163841, 0      , 0    , pi / 2));
  my_robot.segments_.push_back(DHSegment(0.1157  , 0      , 0    , -pi / 2));
  my_robot.segments_.push_back(DHSegment(0.0922  , 0      , 0    , 0));
  // clang-format on
  DHRobot my_robot_calibration;
  // clang-format off
  //                                                 d,        a,       theta, alpha
  my_robot_calibration.segments_.push_back(DHSegment(0       , 0      , 0    , 0.0));
  my_robot_calibration.segments_.push_back(DHSegment(0.5     , 0      , 0.1  , 0.2));
  my_robot_calibration.segments_.push_back(DHSegment(0.8     , 0      , 0.3  , 0.4));
  my_robot_calibration.segments_.push_back(DHSegment(-1.3    , 0      , 0    , 0.0));
  my_robot_calibration.segments_.push_back(DHSegment(0       , 0      , 0    , 0.0));
  my_robot_calibration.segments_.push_back(DHSegment(0       , 0      , 0    , 0.0));
  // clang-format on

  Calibration calibration(my_robot + my_robot_calibration);

  // First let's see, whether our calibration input does make sense.
  {
    Eigen::Matrix<double, 6, 1> jointvalues;
    jointvalues << 0, 0, 0, 0, 0, 0;
    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);
    EXPECT_PRED3(isApproximately, -1.25672673098643, fk(0, 3), 1e-11);
    EXPECT_PRED3(isApproximately, -0.320928557210126, fk(1, 3), 1e-11);
    EXPECT_PRED3(isApproximately, 0.158086917698569, fk(2, 3), 1e-11);
  }

  calibration.correctChain();
  {
    Eigen::Matrix<double, 6, 1> jointvalues;
    jointvalues << 0, 0, 0, 0, 0, 0;
    Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);
    EXPECT_PRED3(isApproximately, -1.25672673098643, fk(0, 3), 1e-11);
    EXPECT_PRED3(isApproximately, -0.320928557210126, fk(1, 3), 1e-11);
    EXPECT_PRED3(isApproximately, 0.158086917698569, fk(2, 3), 1e-11);
  }
}
}  // namespace

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

