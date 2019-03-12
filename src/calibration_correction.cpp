// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-01-10
 *
 */
//----------------------------------------------------------------------

#include <ur_rtde_driver/calibration.h>

#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
sensor_msgs::JointStatePtr joint_state;
std::unique_ptr<tf::TransformListener> tf_listener;

void jointStateCallback(sensor_msgs::JointStatePtr msg)
{
  tf::StampedTransform stamped_transform;
  try
  {
    tf_listener->lookupTransform("base", "tool0_controller", ros::Time(0), stamped_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF lookup error error: %s", ex.what());
    return;
  }

  geometry_msgs::TransformStamped tf_msg;
  tf::transformStampedTFToMsg(stamped_transform, tf_msg);

  // ROS_INFO_STREAM(tf_msg);

  KDL::Frame result_original;
  KDL::JntArray jointpositions = KDL::JntArray(6);
  for (size_t i = 0; i < 6; ++i)
  {
    jointpositions.data[i] = msg->position[i];
  }
  fk_solver->JntToCart(jointpositions, result_original);

  Eigen::Matrix4d mat;
  result_original.Make4x4(mat.data());

  Eigen::Vector3d error;
  error << mat.transpose().topRightCorner(3, 1) - Eigen::Vector3d(stamped_transform.getOrigin());

  ROS_INFO_STREAM(error);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ur_calibration");
  ros::NodeHandle nh;

  DHRobot my_robot;
  // This is an ideal UR10
  // clang-format off
  //                                     d,        a,       theta, alpha
  my_robot.segments_.push_back(DHSegment(0.1273  , 0      , 0    , M_PI / 2));
  my_robot.segments_.push_back(DHSegment(0       , -0.612 , 0    , 0));
  my_robot.segments_.push_back(DHSegment(0       , -0.5723, 0    , 0.0));
  my_robot.segments_.push_back(DHSegment(0.163941, 0      , 0    , M_PI / 2));
  my_robot.segments_.push_back(DHSegment(0.1157  , 0      , 0    , -M_PI / 2));
  my_robot.segments_.push_back(DHSegment(0.0922  , 0      , 0    , 0));
  // clang-format on
  // DHRobot my_robot_calibration;
  //// clang-format off
  ////                                                 d,                      a,                       theta, alpha
  // my_robot_calibration.segments_.push_back(DHSegment(0.00065609212979853    ,4.6311376834935676e-05
  // ,-7.290070070824746e-05 ,0.000211987863869334  ));
  // my_robot_calibration.segments_.push_back(DHSegment(1.4442162376284788     ,-0.00012568315331862312
  // ,-0.01713897289704999   ,-0.0072553625957652995));
  // my_robot_calibration.segments_.push_back(DHSegment(0.854147723854608      ,0.00186216581161458
  // ,-0.03707159413492756   ,-0.013483226769541364 ));
  // my_robot_calibration.segments_.push_back(DHSegment(-2.2989425877563705    ,9.918593870679266e-05
  // ,0.054279462160583214   ,0.0013495820227329425 ));
  // my_robot_calibration.segments_.push_back(DHSegment(-1.573498686836816e-05 ,4.215462720453189e-06
  // ,1.488984257025741e-07  ,-0.001263136163679901 ));
  // my_robot_calibration.segments_.push_back(DHSegment(1.9072435590711256e-05 ,0 ,1.551499479707493e-05  ,0 ));
  // Calibration calibration(my_robot + my_robot_calibration);
  Calibration calibration(my_robot);
  KDL::Chain corrected_chain = calibration.getChain();
  // calibration.debugChain(corrected_chain);

  uint32_t num_jts = corrected_chain.getNrOfJoints();

  fk_solver.reset(new KDL::ChainFkSolverPos_recursive(corrected_chain));
  KDL::JntArray jointpositions = KDL::JntArray(num_jts);
  KDL::Frame result_original;

  tf_listener.reset(new tf::TransformListener);

  ros::Subscriber joint_state_sub = nh.subscribe("joint_states", 1, jointStateCallback);

  ros::spin();

  return 0;
}
