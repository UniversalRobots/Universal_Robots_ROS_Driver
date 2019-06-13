/*
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <vector>
#include "ur_rtde_driver/comm/parser.h"
#include "ur_rtde_driver/comm/bin_parser.h"
#include "ur_rtde_driver/comm/pipeline.h"

#include "ur_rtde_driver/rtde/control_package_pause.h"
#include "ur_rtde_driver/rtde/control_package_setup_inputs.h"
#include "ur_rtde_driver/rtde/control_package_setup_outputs.h"
#include "ur_rtde_driver/rtde/control_package_start.h"
#include "ur_rtde_driver/rtde/data_package.h"
#include "ur_rtde_driver/rtde/get_urcontrol_version.h"
#include "ur_rtde_driver/rtde/package_header.h"
#include "ur_rtde_driver/rtde/request_protocol_version.h"
#include "ur_rtde_driver/rtde/text_message.h"

namespace ur_driver
{
namespace rtde_interface
{
class RTDEParser : public comm::Parser<PackageHeader>
{
public:
  RTDEParser() = delete;
  RTDEParser(const std::vector<std::string>& recipe) : recipe_(recipe)
  {
  }
  virtual ~RTDEParser() = default;

  bool parse(comm::BinParser& bp, std::vector<std::unique_ptr<comm::URPackage<PackageHeader>>>& results)

  {
    PackageHeader::_package_size_type size;
    PackageType type;
    bp.parse(size);
    bp.parse(type);

    if (!bp.checkSize(size - sizeof(size) - sizeof(type)))
    {
      LOG_ERROR("Buffer len shorter than expected packet length");
      return false;
    }

    switch (type)
    {
      case PackageType::RTDE_DATA_PACKAGE:
      {
        std::unique_ptr<RTDEPackage> package(new DataPackage(recipe_));

        if (!package->parseWith(bp))
        {
          LOG_ERROR("Package parsing of type %d failed!", static_cast<int>(type));
          return false;
        }
        results.push_back(std::move(package));
        break;
      }
      default:
      {
        std::unique_ptr<RTDEPackage> package(packageFromType(type));
        if (!package->parseWith(bp))
        {
          LOG_ERROR("Package parsing of type %d failed!", static_cast<int>(type));
          return false;
        }

        results.push_back(std::move(package));
        break;
      }
    }
    if (!bp.empty())
    {
      LOG_ERROR("Package of type %d was not parsed completely!", static_cast<int>(type));
      bp.debug();
      return false;
    }

    return true;
  }

private:
  std::vector<std::string> recipe_;
  RTDEPackage* packageFromType(PackageType type)
  {
    switch (type)
    {
      case PackageType::RTDE_TEXT_MESSAGE:
        return new TextMessage;
        break;
      case PackageType::RTDE_GET_URCONTROL_VERSION:
        return new GetUrcontrolVersion;
        break;
      case PackageType::RTDE_REQUEST_PROTOCOL_VERSION:
        return new RequestProtocolVersion;
        break;
      case PackageType::RTDE_CONTROL_PACKAGE_PAUSE:
        return new ControlPackagePause;
        break;
      case PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
        return new ControlPackageSetupInputs;
        break;
      case PackageType::RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
        return new ControlPackageSetupOutputs;
        break;
      case PackageType::RTDE_CONTROL_PACKAGE_START:
        return new ControlPackageStart;
        break;
      default:
        return new RTDEPackage(type);
    }
  }
};

}  // namespace rtde_interface
}  // namespace ur_driver
