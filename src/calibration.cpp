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

#include <eigen_conversions/eigen_kdl.h>

Calibration::Calibration(const DHRobot& robot_parameters) : robot_parameters_(robot_parameters)
{
}

Calibration::~Calibration()
{
}

KDL::Chain Calibration::correctChain()
{
  KDL::Chain robot_chain = getChain();

  correctChainAt(robot_chain, 2);
  correctChainAt(robot_chain, 4);

  return robot_chain;
}

bool Calibration::correctChainAt(KDL::Chain& robot_chain, const size_t correction_index)
{
  bool success = true;
  KDL::Chain correction_chain, corrected_subchain;

  // Split chain into part before the correction joint and after. We then change the first link on
  // the second chain. This splitting is necessary as KDL doesn't support forward kinematics
  // relative to another link than the first.
  success = splitChain(robot_chain, correction_chain, correction_index);

  // Correct the chain after the correction joint
  corrected_subchain = correctAxis(correction_chain);

  // Add the corrected second part to the robot chain to get a full robot model again
  robot_chain.addChain(corrected_subchain);

  return success;
}

bool Calibration::splitChain(KDL::Chain& robot_chain, KDL::Chain& second_chain, const size_t split_index)
{
  if (split_index >= robot_chain.segments.size())
  {
    return false;
  }

  second_chain = KDL::Chain();
  for (auto it = robot_chain.segments.begin() + split_index; it != robot_chain.segments.end();)
  {
    KDL::Segment new_segment = *it;
    robot_chain.segments.erase(it);
    second_chain.addSegment(new_segment);
  }

  return true;
}

KDL::Chain Calibration::correctAxis(KDL::Chain& robot_chain)
{
  // Each DH-Segment is split into two KDL segments. One representing the d and theta parameters and
  // one with the a and alpha parameters. If we start from the first segment (which represents d and
  // theta), there follows a passive segment (with a and alpha) and the next d/theta-segment after
  // that.
  //
  // In principle, the d parameter of the first segment gets set to zero, first. With this change,
  // the kinematic structure gets destroyed, which has to be corrected:
  //   - With setting d to 0, both the start and end points of the passive segment move along the
  //   rotational axis of the start segment. Instead, the end point of the passive segment has to
  //   move along the rotational axis of the next segment. This creates a change in a and and theta, if
  //   the two rotational axes are not parallel.
  //
  //   - The length of moving along the next segment's rotational axis is calculated by intersecting
  //   the rotational axis with the XY-plane of the first segment.
  //
  //   -
  KDL::Frame next_segment_frame;
  KDL::Frame passive_frame;

  KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
  uint32_t num_jts = robot_chain.getNrOfJoints();
  KDL::JntArray jointpositions = KDL::JntArray(num_jts);

  fk_solver.JntToCart(jointpositions, passive_frame, 2);
  ROS_INFO_STREAM(passive_frame.p.x() << ", " << passive_frame.p.y() << ", " << passive_frame.p.z());
  fk_solver.JntToCart(jointpositions, next_segment_frame, 3);
  ROS_INFO_STREAM(next_segment_frame.p.x() << ", " << next_segment_frame.p.y() << ", " << next_segment_frame.p.z());

  Eigen::Vector3d eigen_passive;
  tf::vectorKDLToEigen(passive_frame.p, eigen_passive);
  Eigen::Vector3d eigen_next;
  tf::vectorKDLToEigen(next_segment_frame.p, eigen_next);

  // Construct a representation of the next segment's rotational axis
  Eigen::ParametrizedLine<double, 3> next_line;
  next_line = Eigen::ParametrizedLine<double, 3>::Through(eigen_passive, eigen_next);

  // ROS_INFO_STREAM("next_line:" << std::endl
  //<< "Base:" << std::endl
  //<< next_line.origin() << std::endl
  //<< "Direction:" << std::endl
  //<< next_line.direction());

  // XY-Plane of first segment's start
  Eigen::Hyperplane<double, 3> plane(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0));

  // Intersect the rotation axis with the XY-Plane. We use both notations, the length and
  // intersection point, as we will need both. The intersection_param is the length moving along the
  // plane (change in the next segment's d param), while the intersection point will be used for
  // calculating the new angle theta.
  double intersection_param = next_line.intersectionParameter(plane);
  Eigen::Vector3d intersection = next_line.intersectionPoint(plane);
  double new_theta = std::atan(intersection.y() / intersection.x());
  // Upper and lower arm segments on URs all have negative length due to dh params
  double new_length = -1 * intersection.norm();
  // ROS_INFO_STREAM("Wrist line intersecting at " << std::endl << intersection);
  // ROS_INFO_STREAM("Angle is " << new_theta);
  // ROS_INFO_STREAM("Length is " << new_length);
  // ROS_INFO_STREAM("Intersection param is " << intersection_param);

  // as we move the passive segment towards the first segment, we have to move away the next segment
  // again, to keep the same kinematic structure.
  double sign_dir = next_line.direction().z() > 0 ? 1.0 : -1.0;
  double distance_correction = intersection_param * sign_dir;

  ROS_INFO_STREAM("Corrected chain at " << robot_chain.segments[0].getName());
  return buildCorrectedChain(robot_chain, new_length, new_theta, distance_correction);
}

KDL::Chain Calibration::buildCorrectedChain(const KDL::Chain& robot_chain, const double new_length,
                                            const double new_theta, const double distance_correction)
{
  KDL::Chain corrected_chain;

  // Set d parameter of the first segment to 0 and theta to the calculated new angle
  {
    KDL::Frame frame = KDL::Frame::DH(0, 0, 0, new_theta);
    KDL::Joint joint = KDL::Joint(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 1), KDL::Joint::RotAxis);
    KDL::Segment segment = KDL::Segment(robot_chain.segments[0].getName(), joint, frame);
    corrected_chain.addSegment(segment);
  }

  // Correct arm segment length and angle
  {
    // Get the original theta from the first link
    double roll_a, pitch_a, yaw_a;
    robot_chain.segments[0].getFrameToTip().M.GetRPY(roll_a, pitch_a, yaw_a);

    double roll, pitch, yaw;
    robot_chain.segments[1].getFrameToTip().M.GetRPY(roll, pitch, yaw);

    // Since we changed the kinematic structure, we have to make sure to rotate around the correct
    // axis, so we revert the theta change in this joint, again.
    KDL::Rotation rotation = KDL::Rotation::EulerZYX(yaw_a - new_theta, 0, roll);
    KDL::Vector position(new_length, 0, 0);
    KDL::Frame frame(rotation, position);

    // KDL::Frame frame = KDL::Frame::DH(new_length, roll, 0, yaw_a - new_theta);

    KDL::Segment segment = KDL::Segment(robot_chain.segments[1].getName(), KDL::Joint(KDL::Joint::None), frame);
    corrected_chain.addSegment(segment);
  }

  // Correct next joint
  {
    KDL::Frame new_frame = robot_chain.getSegment(2).getFrameToTip();
    new_frame.p = robot_chain.getSegment(2).getFrameToTip().p;
    ROS_INFO_STREAM("Correcting segment i+2 length from " << new_frame.p.z());

    // the d-parameter can be modified by the intersection_parameter which is the distance traveled
    // along the rotation axis
    new_frame.p = KDL::Vector(0, 0, new_frame.p.z() - distance_correction);

    ROS_INFO_STREAM("Corrected segment i+2 length to " << new_frame.p.z());
    KDL::Joint new_joint = robot_chain.getSegment(2).getJoint();

    KDL::Segment segment = KDL::Segment(robot_chain.getSegment(2).getName(), new_joint, new_frame);
    corrected_chain.addSegment(segment);
  }

  for (size_t i = 3; i < robot_chain.segments.size(); ++i)
  {
    corrected_chain.addSegment(robot_chain.segments[i]);
  }

  return corrected_chain;
}

DHRobot Calibration::chainToDH(const KDL::Chain& chain)
{
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  uint32_t num_jts = chain.getNrOfJoints();
  KDL::JntArray jointpositions = KDL::JntArray(num_jts);
  // Assign some values to the joint positions
  for (unsigned int i = 0; i < num_jts; i++)
  {
    jointpositions(i) = 0;
  }

  DHRobot robot;

  for (size_t i = 0; i < chain.segments.size(); i += 2)
  {
    ROS_INFO_STREAM("Extracting DH parameters for joint " << i);
    for (size_t j = i; j < i + 2; ++j)
    {
      // ROS_INFO_STREAM(j << " < " << i << "+2");
      KDL::Frame result;
      result = chain.segments[j].getFrameToTip();
      ROS_INFO_STREAM("Relative position: " << result.p.x() << ", " << result.p.y() << ", " << result.p.z());
      double roll, pitch, yaw;
      result.M.GetRPY(roll, pitch, yaw);
      ROS_INFO_STREAM("Relative rotation: " << roll << ", " << pitch << ", " << yaw);
      fk_solver.JntToCart(jointpositions, result, j);
      KDL::Joint joint = chain.segments[j].getJoint();
    }

    DHSegment seg;
    {
      KDL::Frame result = chain.segments[i].getFrameToTip();
      double roll, pitch, yaw;
      result.M.GetRPY(roll, pitch, yaw);
      seg.theta_ = yaw;
      seg.d_ = result.p.z();
    }
    {
      KDL::Frame result = chain.segments[i + 1].getFrameToTip();
      double roll, pitch, yaw;
      result.M.GetRPY(roll, pitch, yaw);
      seg.alpha_ = roll;
      seg.a_ = result.p.x();
    }

    robot.segments_.push_back(seg);
  }

  {
    double roll, pitch, yaw;
    chain.segments[3].getFrameToTip().M.GetRPY(roll, pitch, yaw);
    robot.delta_theta_correction2_ = yaw;
    chain.segments[5].getFrameToTip().M.GetRPY(roll, pitch, yaw);
    robot.delta_theta_correction3_ = yaw;
  }

  return robot;
}

KDL::Chain Calibration::getChain()
{
  KDL::Chain robot_chain;
  for (size_t i = 0; i < robot_parameters_.segments_.size(); ++i)
  {
    KDL::Frame frame = KDL::Frame::DH(0, 0, robot_parameters_.segments_[i].d_, robot_parameters_.segments_[i].theta_);
    KDL::Joint joint = KDL::Joint(link_names_[i], KDL::Vector(0, 0, robot_parameters_.segments_[i].d_),
                                  KDL::Vector(0, 0, 1), KDL::Joint::RotAxis);

    KDL::Segment segment = KDL::Segment(link_names_[i], joint, frame);
    robot_chain.addSegment(segment);
    KDL::Frame frame2 = KDL::Frame::DH(robot_parameters_.segments_[i].a_, robot_parameters_.segments_[i].alpha_, 0, 0);
    KDL::Segment segment2 =
        KDL::Segment(link_names_[i] + "_passive", KDL::Joint(link_names_[i] + "_passive", KDL::Joint::None), frame2);

    robot_chain.addSegment(segment2);
  }
  return robot_chain;
}

void Calibration::debugChain(const KDL::Chain& robot_chain)
{
  uint32_t num_jts = robot_chain.getNrOfJoints();

  KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
  KDL::JntArray jointpositions = KDL::JntArray(num_jts);
  // Assign some values to the joint positions
  for (unsigned int i = 0; i < num_jts; i++)
  {
    jointpositions(i) = 0;
  }

  for (size_t i = 0; i < robot_chain.segments.size(); ++i)
  {
    ROS_INFO_STREAM("Segment " << i << ": " << robot_chain.segments[i].getName());
    KDL::Frame result;
    result = robot_chain.segments[i].getFrameToTip();
    ROS_INFO_STREAM("Relative position: " << result.p.x() << ", " << result.p.y() << ", " << result.p.z());
    double roll, pitch, yaw;
    result.M.GetRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Relative rotation: " << roll << ", " << pitch << ", " << yaw);
    fk_solver.JntToCart(jointpositions, result, i);
    ROS_INFO_STREAM(std::setprecision(15)
                    << "Absolute position: " << result.p.x() << ", " << result.p.y() << ", " << result.p.z());
    KDL::Joint joint = robot_chain.segments[i].getJoint();
    ROS_INFO_STREAM("Joint type: " << joint.getTypeName());
    ROS_INFO_STREAM("Joint position: " << joint.pose(0).p.x() << ", " << joint.pose(0).p.y() << ", "
                                       << joint.pose(0).p.z());
    ROS_INFO_STREAM("Joint Axis: " << joint.JointAxis().x() << "," << joint.JointAxis().y() << ", "
                                   << joint.JointAxis().z());
  }

  DHRobot robot_out = Calibration::chainToDH(robot_chain);
  ROS_INFO_STREAM("Robot data:\n" << robot_out.toXacroProperties());
}
