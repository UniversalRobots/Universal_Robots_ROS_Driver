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

  /*!
   * \brief Generates a string representation of this robot representation.
   *
   * This string can be directly included into the xacro file used for a calibrated robot.
   */
  std::string toXacroProperties()
  {
    std::stringstream ss;
    for (size_t i = 0; i < segments_.size(); ++i)
    {
      ss << "<xacro:property name=\"d" << i + 1 << "\" value=\"" << segments_[i].d_ << "\" />\n";
      ss << "<xacro:property name=\"a" << i + 1 << "\" value=\"" << segments_[i].a_ << "\" />\n";
      ss << "<xacro:property name=\"theta" << i + 1 << "\" value=\"" << segments_[i].theta_ << "\" />\n";
      ss << "<xacro:property name=\"alpha" << i + 1 << "\" value=\"" << segments_[i].alpha_ << "\" />\n";
    }
    ss << "<xacro:property name=\"delta_theta_correction2\" value=\"" << delta_theta_correction2_ << "\" />\n";
    ss << "<xacro:property name=\"delta_theta_correction3\" value=\"" << delta_theta_correction3_ << "\" />\n";
    return ss.str();
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

  std::vector<Eigen::Matrix4d> getChain()
  {
    return chain_;
  }

  std::string toXacroProperties()
  {
    return robot_parameters_corrected_.toXacroProperties();
  }
  YAML::Node toYaml() const;

  std::vector<Eigen::Matrix4d> getSimplified() const;

  Eigen::Matrix4d calcForwardKinematics(const Eigen::Matrix<double, 6, 1>& joint_values, const size_t link_nr = 6);

private:
  void correctAxis(const size_t correction_index);

  void buildChain();

  DHRobot robot_parameters_;
  DHRobot robot_parameters_corrected_;
  std::vector<std::string> link_names_ = { "shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3" };

  std::vector<Eigen::Matrix4d> chain_;
};
}  // namespace ur_calibration
#endif  // ifndef UR_RTDE_DRIVER_CALIBRATION_H_INCLUDED
