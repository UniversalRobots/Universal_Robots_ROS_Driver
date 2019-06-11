// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-06-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_EXCEPTIONS_H_INCLUDED
#define UR_RTDE_DRIVER_EXCEPTIONS_H_INCLUDED

#include <stdexcept>

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
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_EXCEPTIONS_H_INCLUDED
