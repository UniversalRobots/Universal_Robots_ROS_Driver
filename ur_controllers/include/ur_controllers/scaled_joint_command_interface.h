// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright (C) 2012, hiDOF INC.
// Copyright (C) 2019, FZI Forschungszentrum Informatik (Scaling extension)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
/// \author Wim Meeussen
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-18
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS_SCALED_JOINT_COMMAND_INTERFACE_H_INCLUDED
#define UR_CONTROLLERS_SCALED_JOINT_COMMAND_INTERFACE_H_INCLUDED

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace ur_controllers
{
class ScaledJointHandle : public hardware_interface::JointHandle
{
public:
  ScaledJointHandle() : hardware_interface::JointHandle(), scaling_factor_(0)

  {
  }

  /**
   * \param js This joint's state handle
   * \param cmd A pointer to the storage for this joint's output command
   * \param scaling_factor A pointer to the storage for this joint's scaling factor
   */
  ScaledJointHandle(const hardware_interface::JointStateHandle& js, double* cmd, double* scaling_factor)
    : hardware_interface::JointHandle(js, cmd), scaling_factor_(scaling_factor)
  {
    if (scaling_factor_ == nullptr)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Scaling factor pointer is null.");
    }
  }

  virtual ~ScaledJointHandle() = default;

  void setScalingFactor(double scaling_factor)
  {
    assert(scaling_factor_);
    *scaling_factor_ = scaling_factor;
  }
  double getScalingFactor() const
  {
    assert(scaling_factor_);
    return *scaling_factor_;
  }

private:
  double* scaling_factor_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 *
 * These interfaces provide an additional scaling factor implemented, e.g. by a robot's speed
 * slider.
 *
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class ScaledJointCommandInterface
  : public hardware_interface::HardwareResourceManager<ScaledJointHandle, hardware_interface::ClaimResources>
{
};

/// \ref ScaledJointCommandInterface for commanding effort-based joints.
class ScaledEffortJointInterface : public ScaledJointCommandInterface
{
};

/// \ref ScaledJointCommandInterface for commanding velocity-based joints.
class ScaledVelocityJointInterface : public ScaledJointCommandInterface
{
};

/// \ref ScaledJointCommandInterface for commanding position-based joints.
class ScaledPositionJointInterface : public ScaledJointCommandInterface
{
};
}  // namespace ur_controllers
#endif  // ifndef UR_CONTROLLERS_SCALED_JOINT_COMMAND_INTERFACE_H_INCLUDED
