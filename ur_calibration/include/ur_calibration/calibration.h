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

#ifndef UR_RTDE_DRIVER_CALIBRATION_H_INCLUDED
#define UR_RTDE_DRIVER_CALIBRATION_H_INCLUDED

#include <ros/ros.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace ur_calibration
{
/*!
 * \brief An internal representation of a DH-parametrized link.
 *
 * Each segment consists of parameters a, d, alpha and theta.
 */
struct DHSegment
{
  double d_;
  double a_;
  double theta_;
  double alpha_;

  /*!
   * \brief Creates an element with defined elements.
   */
  DHSegment(const double d, const double a, const double theta, const double alpha)
    : d_(d), a_(a), theta_(theta), alpha_(alpha)
  {
  }

  /*!
   * \brief Creates a Segment with all elements equal to zero.
   */
  DHSegment() : d_(0), a_(0), theta_(0), alpha_(0)
  {
  }

  /*!
   * \brief Adds another segment element-wise (a+a, d+d, alpha+alpha, theta+theta)
   *
   * \param other Other segment to add
   *
   * \returns Segment consisting of element-wise addition of \p this and \p other
   */
  DHSegment operator+(const DHSegment& other)
  {
    return DHSegment(this->d_ + other.d_, this->a_ + other.a_, this->theta_ + other.theta_,
                     this->alpha_ + other.alpha_);
  }
};

/*!
 * \brief Internal representation of a robot based on DH parameters.
 *
 * Note that this representation doesn't contain a real DH parameter representation, but is used for
 * a corrected model of calibrated UR robots. Shoulder and elbow axes are artificially shortened in
 * the final representation, requiring a correction parameter in \p theta_2 and \p theta_3.
 */
struct DHRobot
{
  std::vector<DHSegment> segments_;
  double delta_theta_correction2_;
  double delta_theta_correction3_;

  /*!
   * \brief Create a new robot representation giving a set of \ref DHSegment objects
   */
  DHRobot(const std::vector<DHSegment>& segments)
  {
    delta_theta_correction2_ = 0;
    delta_theta_correction3_ = 0;
  }

  DHRobot() = default;

  /*!
   * \brief Adds another robot representation, by adding their segments element-wise. See \ref
   * DHSegment::operator+ for details.
   */
  DHRobot operator+(const DHRobot& other)
  {
    assert(this->segments_.size() == other.segments_.size());
    DHRobot ret;
    for (size_t i = 0; i < this->segments_.size(); ++i)
    {
      ret.segments_.push_back(this->segments_[i] + other.segments_[i]);
    }
    return ret;
  }
};

/*!
 * \brief Class that handles the calibration correction for Universal Robots
 *
 * Universal robots provide a factory calibration of their DH parameters to exactly estimate their
 * TCP pose using forward kinematics. However, those DH parameters construct a kinematic chain that
 * can be very long, as upper arm and lower arm segments can be drawn out of their physical position
 * by multiple meters (even above 100m can occur).
 *
 * This class helps creating a kinematic chain, that is as close as possible to the physical model,
 * by dragging the upper and lower arm segments back to their zero position.
 */
class Calibration
{
public:
  Calibration(const DHRobot& robot);
  virtual ~Calibration();

  /*!
   * \brief Corrects a UR kinematic chain in such a way that shoulder and elbow offsets are 0.
   */
  void correctChain();

  /*!
   * \brief Get the transformation matrix representation of the chain as constructed by the
   * DH parameters.
   *
   * This will contain twice as many transformation matrices as joints, as for each set of DH
   * parameters two matrices are generated. If you'd like to receive one matrix per joint instead,
   * use the getSimplified() function instead.
   *
   * \returns A vector of 4x4 transformation matrices, two for each joint going from the base to the
   * tcp.
   */
  std::vector<Eigen::Matrix4d> getChain()
  {
    return chain_;
  }

  /*!
   * \brief Get the transformation matrix representation of the chain, where each joint is
   * represented by one matrix.
   *
   * \returns Vector of 4x4 transformation matrices, one for each joint going from the base to the
   * tcp.
   */
  std::vector<Eigen::Matrix4d> getSimplified() const;

  /*!
   * \brief Generates a yaml representation of all transformation matrices as returned by
   * getSimplified()
   *
   * \returns A YAML tree representing all tranformation matrices.
   */
  YAML::Node toYaml() const;

  /*!
   * \brief Calculates the forwart kinematics given a joint configuration with respect to the base
   * link.
   *
   * \param joint_values Joint values for which the forward kinematics should be calculated.
   * \param link_nr If given, the cartesian position for this joint (starting at 1) is returned. By
   * default the 6th joint is used.
   *
   * \returns Transformation matrix giving the full pose of the requested link in base coordinates.
   */
  Eigen::Matrix4d calcForwardKinematics(const Eigen::Matrix<double, 6, 1>& joint_values, const size_t link_nr = 6);

private:
  // Corrects a single axis
  void correctAxis(const size_t correction_index);

  // Builds the chain from robot_parameters_
  void buildChain();

  DHRobot robot_parameters_;
  std::vector<std::string> link_names_ = { "shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3" };

  std::vector<Eigen::Matrix4d> chain_;
};
}  // namespace ur_calibration
#endif  // ifndef UR_RTDE_DRIVER_CALIBRATION_H_INCLUDED
