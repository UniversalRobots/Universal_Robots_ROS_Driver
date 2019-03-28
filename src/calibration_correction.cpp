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

sensor_msgs::JointStatePtr joint_state;
std::unique_ptr<Calibration> my_calibration;
std::unique_ptr<tf::TransformListener> tf_listener;

void jointStateCallback(sensor_msgs::JointStatePtr msg)
{
  ROS_INFO("Callback");
  tf::StampedTransform stamped_transform;
  try
  {
    tf_listener->waitForTransform("base", "tool0_controller", msg->header.stamp, ros::Duration(0.5));
    tf_listener->lookupTransform("base", "tool0_controller", msg->header.stamp, stamped_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF lookup error error: %s", ex.what());
    return;
  }

  geometry_msgs::TransformStamped tf_msg;
  tf::transformStampedTFToMsg(stamped_transform, tf_msg);

  Eigen::Matrix<double, 6, 1> jointvalues = Eigen::Matrix<double, 6, 1>(msg->position.data());

  Eigen::Matrix4d mat = my_calibration->calcForwardKinematics(jointvalues);

  Eigen::Vector3d error;
  error << mat.topRightCorner(3, 1) - Eigen::Vector3d(stamped_transform.getOrigin());

  ROS_INFO_STREAM("Error(abs): " << error.norm());
  ROS_INFO_STREAM("Error per axis:\n" << error);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ur_calibration");
  ros::NodeHandle nh;

  DHRobot my_robot;
  // This is an ideal UR10
  // clang-format off
  //                                     d,        a,       theta, alpha
  my_robot.segments_.push_back(DHSegment(0.1273  , 0      , 0    , M_PI_2));
  my_robot.segments_.push_back(DHSegment(0.0     , -0.612 , 0.0  , 0.0));
  my_robot.segments_.push_back(DHSegment(0.0     , -0.5723, 0    , 0.0));
  my_robot.segments_.push_back(DHSegment(0.163941, 0      , 0    , M_PI_2));
  my_robot.segments_.push_back(DHSegment(0.1157  , 0      , 0    , -M_PI_2));
  my_robot.segments_.push_back(DHSegment(0.0922  , 0      , 0    , 0));
  // clang-format on
  std::map<std::string, DHRobot> calibrations;
  //                                                 d,                        a,                        theta, alpha
  {
    DHRobot robot_calibration;
    // clang-format off
    robot_calibration.segments_.push_back(DHSegment( 0.00065609212979853    ,  4.6311376834935676e-05 , -7.290070070824746e-05  ,  0.000211987863869334  ));
    robot_calibration.segments_.push_back(DHSegment( 1.4442162376284788     , -0.00012568315331862312 , -0.01713897289704999    , -0.0072553625957652995));
    robot_calibration.segments_.push_back(DHSegment( 0.854147723854608      ,  0.00186216581161458    , -0.03707159413492756    , -0.013483226769541364 ));
    robot_calibration.segments_.push_back(DHSegment(-2.2989425877563705     ,  9.918593870679266e-05  , 0.054279462160583214   ,  0.0013495820227329425 ));
    robot_calibration.segments_.push_back(DHSegment(-1.573498686836816e-05  ,  4.215462720453189e-06  , 1.488984257025741e-07  , -0.001263136163679901 ));
    robot_calibration.segments_.push_back(DHSegment( 1.9072435590711256e-05 ,  0                      , 1.551499479707493e-05  ,  0 ));
    // clang-format on
    calibrations.insert(std::make_pair("ids-ur10-2", robot_calibration));
  }
  {
    DHRobot robot_calibration;
    // clang-format off
    robot_calibration.segments_.push_back(DHSegment(0.000489427369583322891 , -0.000157659251888300598 , 7.12144160891142552e-06 , -0.000181484221894123721));
    robot_calibration.segments_.push_back(DHSegment(2.15820897072427709     , 0.000797598154419598693  , -0.0597679909195674153  , -0.0169424433153391799  ));
    robot_calibration.segments_.push_back(DHSegment(-0.200070560336579328   , 0.00141079051860837357   , 0.0276740817239430857   , -0.00936706021457411366 ));
    robot_calibration.segments_.push_back(DHSegment(-1.95850825255269623    , 8.02528233902343753e-05  , 0.0321549453426039911   , 0.000705308984090935454 ));
    robot_calibration.segments_.push_back(DHSegment(-2.27384815544295904e-05, 2.49305433182637798e-06  , -7.91131358362739777e-05, -0.000129247903554619015));
    robot_calibration.segments_.push_back(DHSegment(1.40058148229704749e-05 , 0 , 2.43093497590859384e-05 , 0 ));
    // clang-format on
    calibrations.insert(std::make_pair("ids-ur10-3", robot_calibration));
  }
  // my_calibration.reset(new Calibration(my_robot));

  for (auto& calib : calibrations)
  {
    Eigen::Matrix<double, 6, 1> jointvalues;

    double min_error = 9999;
    double max_error = 0;
    double mean_error = 0;
    const size_t num_runs = 100000;
    for (size_t i = 0; i < num_runs; ++i)
    {
      Calibration calibration(my_robot + calib.second);
      jointvalues = 2 * M_PI * Eigen::Matrix<double, 6, 1>::Random();
      Eigen::Matrix4d fk_orig = calibration.calcForwardKinematics(jointvalues);
      calibration.correctChain();
      Eigen::Matrix4d fk_corrected = calibration.calcForwardKinematics(jointvalues);

      Eigen::Vector3d error;
      error << fk_orig.topRightCorner(3, 1) - fk_corrected.topRightCorner(3, 1);

      if (error.norm() < min_error)
      {
        min_error = error.norm();
      }
      if (error.norm() > max_error)
      {
        max_error = error.norm();
      }
      mean_error += error.norm() / static_cast<double>(num_runs);
    }

    ROS_INFO_STREAM(calib.first << ": Error over " << num_runs << " joint angles (mean, min, max): (" << mean_error
                                << ", " << min_error << ", " << max_error << ")");
    // Calibration calibration(my_robot + calib.second);
    // calibration.correctChain();
    // ROS_INFO_STREAM("Corrected calibration: " << std::endl << calibration.toXacroProperties());
  }
  // my_calibration->correctChain();

  // ROS_INFO_STREAM("Corrected calibration: " << std::endl << my_calibration->toXacroProperties());

  // tf_listener.reset(new tf::TransformListener);

  // ros::Subscriber joint_state_sub = nh.subscribe("joint_states", 1, jointStateCallback);

  // ros::spin();

  return 0;
}
