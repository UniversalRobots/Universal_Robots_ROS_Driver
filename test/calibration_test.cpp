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

TEST(UrRtdeDriver, ur10_ideal)
{
  DHRobot my_robot;
  const double pi = std::atan(1)*4;

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

  Calibration calibration(my_robot);

  KDL::Chain robot_chain = calibration.getChain();
  uint32_t num_jts = robot_chain.getNrOfJoints();

  KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
  KDL::JntArray jointpositions = KDL::JntArray(num_jts);
  KDL::Frame result;
  fk_solver.JntToCart(jointpositions, result);

  // Check whether our internal KDL representation gives correct values
  EXPECT_DOUBLE_EQ(result.p.x(), my_robot.segments_[1].a_ + my_robot.segments_[2].a_);
  EXPECT_DOUBLE_EQ(result.p.y(), -1 * (my_robot.segments_[3].d_ + my_robot.segments_[5].d_));
  EXPECT_DOUBLE_EQ(result.p.z(), my_robot.segments_[0].d_ - my_robot.segments_[4].d_);
}

TEST(UrRtdeDriver, calibration)
{
  DHRobot my_robot;
  const double pi = std::atan(1)*4;

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
  //                                                 d,                      a,                       theta,                  alpha
  my_robot_calibration.segments_.push_back(DHSegment(0.00065609212979853    ,4.6311376834935676e-05  ,-7.290070070824746e-05 ,0.000211987863869334  ));
  my_robot_calibration.segments_.push_back(DHSegment(1.4442162376284788     ,-0.00012568315331862312 ,-0.01713897289704999   ,-0.0072553625957652995));
  my_robot_calibration.segments_.push_back(DHSegment(0.854147723854608      ,0.00186216581161458     ,-0.03707159413492756   ,-0.013483226769541364 ));
  my_robot_calibration.segments_.push_back(DHSegment(-2.2989425877563705    ,9.918593870679266e-05   ,0.054279462160583214   ,0.0013495820227329425 ));
  my_robot_calibration.segments_.push_back(DHSegment(-1.573498686836816e-05 ,4.215462720453189e-06   ,1.488984257025741e-07  ,-0.001263136163679901 ));
  my_robot_calibration.segments_.push_back(DHSegment(1.9072435590711256e-05 ,0                       ,1.551499479707493e-05  ,0                     ));
  // clang-format on

  Calibration calibration(my_robot + my_robot_calibration);

  KDL::Chain robot_chain = calibration.getChain();
  uint32_t num_jts = robot_chain.getNrOfJoints();

  KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
  KDL::JntArray jointpositions = KDL::JntArray(num_jts);
  KDL::Frame result_original;
  fk_solver.JntToCart(jointpositions, result_original);

  KDL::Chain robot_chain_corrected = calibration.correctChain();
  KDL::ChainFkSolverPos_recursive fk_solver_corrected(robot_chain_corrected);
  KDL::Frame result_corrected;
  fk_solver.JntToCart(jointpositions, result_corrected);
  //
  // Check whether our internal KDL representation gives correct values
  std::cout << result_original.p.x() << std::endl;
  std::cout << result_corrected.p.x() << std::endl;
  std::cout << result_original.p.y() << std::endl;
  std::cout << result_corrected.p.y() << std::endl;
  std::cout << result_original.p.z() << std::endl;
  std::cout << result_corrected.p.z() << std::endl;
  EXPECT_DOUBLE_EQ(result_original.p.x(), result_corrected.p.x());
  EXPECT_DOUBLE_EQ(result_original.p.y(), result_corrected.p.y());
  EXPECT_DOUBLE_EQ(result_original.p.z(), result_corrected.p.z());
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

