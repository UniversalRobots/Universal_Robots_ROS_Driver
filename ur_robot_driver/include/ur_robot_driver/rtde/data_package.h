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
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_DATA_PACKAGE_H_INCLUDED
#define UR_RTDE_DRIVER_DATA_PACKAGE_H_INCLUDED

#include <unordered_map>

#include "ur_robot_driver/types.h"
#include "ur_robot_driver/rtde/rtde_package.h"
#include <boost/variant.hpp>

namespace ur_driver
{
namespace rtde_interface
{
/*!
 * \brief Possible values for the runtime state
 */
enum class RUNTIME_STATE : uint32_t
{
  STOPPING = 0,
  STOPPED = 1,
  PLAYING = 2,
  PAUSING = 3,
  PAUSED = 4,
  RESUMING = 5
};

/*!
 * \brief The DataPackage class handles communication in the form of RTDE data packages both to and
 * from the robot. It contains functionality to parse and serialize packages for arbitrary recipes.
 */
class DataPackage : public RTDEPackage
{
public:
  using _rtde_type_variant = boost::variant<bool, uint8_t, uint32_t, uint64_t, int32_t, double, vector3d_t, vector6d_t,
                                            vector6int32_t, vector6uint32_t, std::string>;

  DataPackage() = delete;
  /*!
   * \brief Creates a new DataPackage object, based on a given recipe.
   *
   * \param recipe The used recipe
   */
  DataPackage(const std::vector<std::string>& recipe) : RTDEPackage(PackageType::RTDE_DATA_PACKAGE), recipe_(recipe)
  {
  }
  virtual ~DataPackage() = default;

  /*!
   * \brief Initializes to contained list with empty values based on the recipe.
   */
  void initEmpty();

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized version of the package
   *
   * \returns True, if the package was parsed successfully, false otherwise
   */
  virtual bool parseWith(comm::BinParser& bp);
  /*!
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const;

  /*!
   * \brief Serializes the package.
   *
   * \param buffer Buffer to fill with the serialization
   *
   * \returns The total size of the serialized package
   */
  size_t serializePackage(uint8_t* buffer);

  /*!
   * \brief Get a data field from the DataPackage.
   *
   * The data package contains a lot of different data fields, depending on the recipe.
   *
   * \param name The string identifier for the data field as used in the documentation.
   * \param val Target variable. Make sure, it's the correct type.
   * \exception boost::bad_get if the type under given \p name does not match the template type T.
   *
   * \returns True on success, false if the field cannot be found inside the package.
   */
  template <typename T>
  bool getData(const std::string& name, T& val)
  {
    if (data_.find(name) != data_.end())
    {
      val = boost::strict_get<T>(data_[name]);
    }
    else
    {
      return false;
    }
    return true;
  }

  /*!
   * \brief Get a data field from the DataPackage as bitset
   *
   * The data package contains a lot of different data fields, depending on the recipe.
   *
   * \param name The string identifier for the data field as used in the documentation.
   * \param val Target variable. Make sure, it's the correct type.
   * \exception boost::bad_get if the type under given \p name does not match the template type T.
   *
   * \returns True on success, false if the field cannot be found inside the package.
   */
  template <typename T, size_t N>
  bool getData(const std::string& name, std::bitset<N>& val)
  {
    static_assert(sizeof(T) * 8 >= N, "Bitset is too large for underlying variable");

    if (data_.find(name) != data_.end())
    {
      val = std::bitset<N>(boost::strict_get<T>(data_[name]));
    }
    else
    {
      return false;
    }
    return true;
  }

  /*!
   * \brief Set a data field in the DataPackage.
   *
   * The data package contains a lot of different data fields, depending on the recipe.
   *
   * \param name The string identifier for the data field as used in the documentation.
   * \param val Value to set. Make sure, it's the correct type.
   *
   * \returns True on success, false if the field cannot be found inside the package.
   */
  template <typename T>
  bool setData(const std::string& name, T& val)
  {
    if (data_.find(name) != data_.end())
    {
      data_[name] = val;
    }
    else
    {
      return false;
    }
    return true;
  }

  /*!
   * \brief Setter of the recipe id value used to identify the used recipe to the robot.
   *
   * \param recipe_id The new value
   */
  void setRecipeID(const uint8_t& recipe_id)
  {
    recipe_id_ = recipe_id;
  }

private:
  // Const would be better here
  static std::unordered_map<std::string, _rtde_type_variant> g_type_list;
  uint8_t recipe_id_;
  std::unordered_map<std::string, _rtde_type_variant> data_;
  std::vector<std::string> recipe_;

  struct ParseVisitor : public boost::static_visitor<>
  {
    template <typename T>
    void operator()(T& d, comm::BinParser& bp) const
    {
      bp.parse(d);
    }
  };
  struct StringVisitor : public boost::static_visitor<std::string>
  {
    template <typename T>
    std::string operator()(T& d) const
    {
      std::stringstream ss;
      ss << d;
      return ss.str();
    }
  };
  struct SizeVisitor : public boost::static_visitor<uint16_t>
  {
    template <typename T>
    uint16_t operator()(T& d) const
    {
      return sizeof(d);
    }
  };
  struct SerializeVisitor : public boost::static_visitor<size_t>
  {
    template <typename T>
    size_t operator()(T& d, uint8_t* buffer) const
    {
      return comm::PackageSerializer::serialize(buffer, d);
    }
  };
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_DATA_PACKAGE_H_INCLUDED
