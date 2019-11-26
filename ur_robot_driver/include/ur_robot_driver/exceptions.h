// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
 * \date    2019-06-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_EXCEPTIONS_H_INCLUDED
#define UR_RTDE_DRIVER_EXCEPTIONS_H_INCLUDED

#include <chrono>
#include <stdexcept>
#include <sstream>

namespace ur_driver
{
/*!
 * \brief Our base class for exceptions. Specialized exceptions should inherit from those.
 */
class UrException : virtual public std::runtime_error
{
public:
  explicit UrException() : std::runtime_error("")
  {
  }
  explicit UrException(const std::string& what_arg) : std::runtime_error(what_arg)
  {
  }
  explicit UrException(const char* what_arg) : std::runtime_error(what_arg)
  {
  }

  virtual ~UrException() = default;

private:
  /* data */
};

/*!
 * \brief A specialized exception representing detection of a not supported UR control version.
 */
class VersionMismatch : public UrException
{
public:
  explicit VersionMismatch() : VersionMismatch("", 0, 0)
  {
  }
  explicit VersionMismatch(const std::string& text, const uint32_t version_req, const uint32_t version_actual)
    : std::runtime_error(text)
  {
    version_required_ = version_req;
    version_actual_ = version_actual;
    std::stringstream ss;
    ss << text << "(Required version: " << version_required_ << ", actual version: " << version_actual_ << ")";
    text_ = ss.str();
  }
  virtual ~VersionMismatch() = default;

  virtual const char* what() const noexcept override
  {
    return text_.c_str();
  }

private:
  uint32_t version_required_;
  uint32_t version_actual_;
  std::string text_;
};

/*!
 * \brief A specialized exception representing that communication to the tool is not possible.
 */
class ToolCommNotAvailable : public VersionMismatch
{
public:
  explicit ToolCommNotAvailable() : ToolCommNotAvailable("", 0, 0)
  {
  }
  explicit ToolCommNotAvailable(const std::string& text, const uint32_t version_req, const uint32_t version_actual)
    : std::runtime_error(text), VersionMismatch(text, version_req, version_actual)
  {
  }
};

/*!
 * \brief A specialized exception representing that communication to the tool is not possible.
 */
class TimeoutException : public UrException
{
public:
  explicit TimeoutException() = delete;
  explicit TimeoutException(const std::string& text, timeval timeout) : std::runtime_error(text)
  {
    std::stringstream ss;
    ss << text << "(Configured timeout: " << timeout.tv_sec + timeout.tv_usec * 1e-6 << " sec)";
    text_ = ss.str();
  }
  virtual const char* what() const noexcept override
  {
    return text_.c_str();
  }

private:
  std::string text_;
};
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_EXCEPTIONS_H_INCLUDED
