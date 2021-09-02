// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-01-10
 *
 */
//----------------------------------------------------------------------

#include <ur_calibration/calibration.h>

namespace ur_calibration
{
Calibration::Calibration(const DHRobot& robot_parameters) : robot_parameters_(robot_parameters)
{
  buildChain();
}

Calibration::~Calibration()
{
}

void Calibration::correctChain()
{
  correctAxis(1);
  correctAxis(2);
}

void Calibration::correctAxis(const size_t link_index)
{
  // Each DH-Segment is split into two chain segments. One representing the d and theta parameters and
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

  if (chain_[2 * link_index](2, 3) == 0.0)
  {
    // Nothing to do here.
    return;
  }

  Eigen::Matrix4d fk_current = Eigen::Matrix4d::Identity();
  Eigen::Vector3d current_passive = Eigen::Vector3d::Zero();

  Eigen::Matrix4d fk_next_passive = Eigen::Matrix4d::Identity();
  fk_next_passive *= chain_[link_index * 2] * chain_[link_index * 2 + 1];
  Eigen::Vector3d eigen_passive = fk_next_passive.topRightCorner(3, 1);

  Eigen::Vector3d eigen_next = (fk_next_passive * chain_[(link_index + 1) * 2]).topRightCorner(3, 1);

  // Construct a representation of the next segment's rotational axis
  Eigen::ParametrizedLine<double, 3> next_line;
  next_line = Eigen::ParametrizedLine<double, 3>::Through(eigen_passive, eigen_next);

  ROS_DEBUG_STREAM("next_line:" << std::endl
                                << "Base:" << std::endl
                                << next_line.origin() << std::endl
                                << "Direction:" << std::endl
                                << next_line.direction());

  // XY-Plane of first segment's start
  Eigen::Hyperplane<double, 3> plane(fk_current.topLeftCorner(3, 3) * Eigen::Vector3d(0, 0, 1), current_passive);

  // Intersect the rotation axis with the XY-Plane. We use both notations, the length and
  // intersection point, as we will need both. The intersection_param is the length moving along the
  // plane (change in the next segment's d param), while the intersection point will be used for
  // calculating the new angle theta.
  double intersection_param = next_line.intersectionParameter(plane);
  Eigen::Vector3d intersection = next_line.intersectionPoint(plane) - current_passive;
  double new_theta = std::atan(intersection.y() / intersection.x());
  // Upper and lower arm segments on URs all have negative length due to dh params
  double new_length = -1 * intersection.norm();
  ROS_DEBUG_STREAM("Wrist line intersecting at " << std::endl << intersection);
  ROS_DEBUG_STREAM("Angle is " << new_theta);
  ROS_DEBUG_STREAM("Length is " << new_length);
  ROS_DEBUG_STREAM("Intersection param is " << intersection_param);

  // as we move the passive segment towards the first segment, we have to move away the next segment
  // again, to keep the same kinematic structure.
  double sign_dir = next_line.direction().z() > 0 ? 1.0 : -1.0;
  double distance_correction = intersection_param * sign_dir;

  // Set d parameter of the first segment to 0 and theta to the calculated new angle
  // Correct arm segment length and angle
  chain_[2 * link_index](2, 3) = 0.0;
  chain_[2 * link_index].topLeftCorner(3, 3) =
      Eigen::AngleAxisd(new_theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Correct arm segment length and angle
  chain_[2 * link_index + 1](0, 3) = new_length;
  chain_[2 * link_index + 1].topLeftCorner(3, 3) =
      Eigen::AngleAxisd(robot_parameters_.segments_[link_index].theta_ - new_theta, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      Eigen::AngleAxisd(robot_parameters_.segments_[link_index].alpha_, Eigen::Vector3d::UnitX()).toRotationMatrix();

  // Correct next joint
  chain_[2 * link_index + 2](2, 3) -= distance_correction;
}

Eigen::Matrix4d Calibration::calcForwardKinematics(const Eigen::Matrix<double, 6, 1>& joint_values,
                                                   const size_t link_nr)
{
  // Currently ignore input and calculate for zero vector input
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();

  std::vector<Eigen::Matrix4d> simplified_chain = getSimplified();
  for (size_t i = 0; i < link_nr; ++i)
  {
    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
    rotation.topLeftCorner(3, 3) = Eigen::AngleAxisd(joint_values(i), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    output *= simplified_chain[i] * rotation;
  }

  return output;
}

void Calibration::buildChain()
{
  chain_.clear();
  for (size_t i = 0; i < robot_parameters_.segments_.size(); ++i)
  {
    Eigen::Matrix4d seg1_mat = Eigen::Matrix4d::Identity();
    seg1_mat.topLeftCorner(3, 3) =
        Eigen::AngleAxisd(robot_parameters_.segments_[i].theta_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    seg1_mat(2, 3) = robot_parameters_.segments_[i].d_;

    chain_.push_back(seg1_mat);

    Eigen::Matrix4d seg2_mat = Eigen::Matrix4d::Identity();
    seg2_mat.topLeftCorner(3, 3) =
        Eigen::AngleAxisd(robot_parameters_.segments_[i].alpha_, Eigen::Vector3d::UnitX()).toRotationMatrix();
    seg2_mat(0, 3) = robot_parameters_.segments_[i].a_;

    chain_.push_back(seg2_mat);
  }
}

std::vector<Eigen::Matrix4d> Calibration::getSimplified() const
{
  std::vector<Eigen::Matrix4d> simplified_chain;
  simplified_chain.push_back(chain_[0]);
  for (size_t i = 1; i < chain_.size() - 1; i += 2)
  {
    simplified_chain.push_back(chain_[i] * chain_[i + 1]);
    Eigen::Matrix3d rot_a = chain_[i].topLeftCorner(3, 3);
    Eigen::Vector3d rpy_a = rot_a.eulerAngles(0, 1, 2);

    Eigen::Matrix3d rot_b = chain_[i + 1].topLeftCorner(3, 3);
    Eigen::Vector3d rpy_b = rot_b.eulerAngles(0, 1, 2);

    Eigen::Matrix3d rot = simplified_chain.back().topLeftCorner(3, 3);
    Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);
    Eigen::Quaterniond quat(rot);
  }
  simplified_chain.push_back(chain_.back());
  return simplified_chain;
}

YAML::Node Calibration::toYaml() const
{
  YAML::Node node;

  std::vector<Eigen::Matrix4d> chain = getSimplified();

  for (std::size_t i = 0; i < link_names_.size(); ++i)
  {
    YAML::Node link;
    link["x"] = chain[i](0, 3);
    link["y"] = chain[i](1, 3);
    link["z"] = chain[i](2, 3);
    Eigen::Matrix3d rot = chain[i].topLeftCorner(3, 3);
    Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);
    link["roll"] = rpy[0];
    link["pitch"] = rpy[1];
    link["yaw"] = rpy[2];
    node["kinematics"][link_names_[i]] = link;
  }

  return node;
}
}  // namespace ur_calibration
