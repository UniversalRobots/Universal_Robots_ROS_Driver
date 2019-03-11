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

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

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
  KDL::Chain correctChain();

  /*!
   * \brief Print information about every segment in the chain
   *
   */
  static void debugChain(const KDL::Chain& chain);

  /*!
   * \brief Creates a \p KDL::Chain representation of the member \p robot_parameters_
   */
  KDL::Chain getChain();

private:
  /*!
   * \brief Splits the given chain \p in into a subchain before \p split_index and after.
   *
   * \param[in,out] in Complete chain that will be split. Will be only the part before the \p
   * split_index after the function returns.
   * \param[out] second Will contain the subchain after \p split_index after the function returns.
   * \param[in] split_index Index at which the chain should be split. The segment with this index
   * will be the first segment in \p second.
   *
   * \returns true on success, false if \p split_index is larger than the chain size. In the latter
   * case, \p in will not be touched.
   */
  bool splitChain(KDL::Chain& in, KDL::Chain& second, const size_t split_index);

  KDL::Chain correctAxis(KDL::Chain& chain);

  /*!
   * \brief Converts a \p KDL::Chain representation into a \ref DHRobot representation
   */
  static DHRobot chainToDH(const KDL::Chain& chain);

  /*!
   * \brief Modifies the robot chain at segment \p correction_index
   *
   * Correction will set the \p d parameter at the segment to 0 and adapt the rest of the chain
   * accordingly
   *
   * \param[in,out] robot_chain The kinemtatic chain to correct
   * \param[in] correction_index Index at which the correction step should be performed.
   *
   * \returns true on success, false if the chain is smaller than the \p correction_index
   */
  bool correctChainAt(KDL::Chain& robot_chain, const size_t correction_index);

  /*!
   * \brief Given all parameters to correct the chain and a chain, create a corrected chain.
   *
   * \param[in] robot_chain Uncorrected chain from which the correction parameters were calculated.
   * \param[in] new_length New segment length (\p a) of the corrected segment
   * \param[in] new_theta New rotation offset (\p theta) of the corrected segment
   * \param[in] distance_correction Distance by which the next segment has to be adapted due to the
   * correction
   *
   * \returns The corrected chain, where the first segment's \p d parameter is 0
   */
  static KDL::Chain buildCorrectedChain(const KDL::Chain& robot_chain, const double new_length, const double new_theta, const double distance_correction);

  DHRobot robot_parameters_;
  std::vector<std::string> link_names_ = { "shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3" };
};
#endif  // ifndef UR_RTDE_DRIVER_CALIBRATION_H_INCLUDED
