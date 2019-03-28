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
  /* This test compares the forward kinematics of the model constructed from uncorrected
   * parameters with the one from the corrected parameters. They are tested against random
   * joint values and should be equal (in a numeric sense).
   */

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
  //my_robot_calibration.segments_.push_back(DHSegment(0       , 0      , 0    , 0.0));
  //my_robot_calibration.segments_.push_back(DHSegment(0.5     , 0      , 0.1  , 0.2));
  //my_robot_calibration.segments_.push_back(DHSegment(0.8     , 0      , 0.3  , 0.4));
  //my_robot_calibration.segments_.push_back(DHSegment(-1.3    , 0      , 0    , 0.0));
  //my_robot_calibration.segments_.push_back(DHSegment(0       , 0      , 0    , 0.0));
  //my_robot_calibration.segments_.push_back(DHSegment(0       , 0      , 0    , 0.0));
   my_robot_calibration.segments_.push_back(DHSegment( 0.00065609212979853    ,  4.6311376834935676e-05 , -7.290070070824746e-05  ,  0.000211987863869334  ));
   my_robot_calibration.segments_.push_back(DHSegment( 1.4442162376284788     , -0.00012568315331862312 , -0.01713897289704999    , -0.0072553625957652995));
   my_robot_calibration.segments_.push_back(DHSegment( 0.854147723854608      ,  0.00186216581161458    , -0.03707159413492756    , -0.013483226769541364 ));
   my_robot_calibration.segments_.push_back(DHSegment(-2.2989425877563705     ,  9.918593870679266e-05  , 0.054279462160583214   ,  0.0013495820227329425 ));
   my_robot_calibration.segments_.push_back(DHSegment(-1.573498686836816e-05  ,  4.215462720453189e-06  , 1.488984257025741e-07  , -0.001263136163679901 ));
   my_robot_calibration.segments_.push_back(DHSegment( 1.9072435590711256e-05 ,  0                      , 1.551499479707493e-05  ,  0 ));
  // clang-format on

  Calibration calibration(my_robot + my_robot_calibration);

  Eigen::Matrix<double, 6, 1> jointvalues;
  jointvalues << 0, 0, 0, 0, 0, 0;
  // First let's see, whether our calibration input does make sense.
  //{
  // Eigen::Matrix4d fk = calibration.calcForwardKinematics(jointvalues);
  // EXPECT_PRED3(isApproximately, -1.25672673098643, fk(0, 3), 1e-11);
  // EXPECT_PRED3(isApproximately, -0.320928557210126, fk(1, 3), 1e-11);
  // EXPECT_PRED3(isApproximately, 0.158086917698569, fk(2, 3), 1e-11);
  //}

  for (size_t i = 0; i < 1000; ++i)
  {
    Calibration calibration(my_robot + my_robot_calibration);
    jointvalues = 2 * pi * Eigen::Matrix<double, 6, 1>::Random();
    // TODO: Remove this output
    std::cout << "Testing with jointvalues: [" << jointvalues.transpose() << "]" << std::endl;
    Eigen::Matrix4d fk_orig = calibration.calcForwardKinematics(jointvalues);
    calibration.correctChain();
    Eigen::Matrix4d fk_corrected = calibration.calcForwardKinematics(jointvalues);
    EXPECT_PRED3(isApproximately, fk_orig(0, 3), fk_corrected(0, 3), 1e-12);
    EXPECT_PRED3(isApproximately, fk_orig(1, 3), fk_corrected(1, 3), 1e-12);
    EXPECT_PRED3(isApproximately, fk_orig(2, 3), fk_corrected(2, 3), 1e-12);
  }
}
}  // namespace

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

