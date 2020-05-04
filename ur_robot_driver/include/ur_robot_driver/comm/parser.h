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
#include "ur_robot_driver/comm/bin_parser.h"
#include "ur_robot_driver/comm/package.h"

namespace ur_driver
{
namespace comm
{
/*!
 * \brief The parser is a general paser. The namsepace rtde_interface and primary_interface both
 * iclude classes which inherit from it (rtdeParser and primaryParser).
 * The parser functionality also embodies a factory function taking in an uint8.
 */
template <typename T>
class Parser

{
public:
  Parser() = default;
  virtual ~Parser() = default;

  /*!
   * \brief Declares the parse function.
   *
   * \param bp Instance of class binaryParser
   * \param results A unique pointer
   */
  virtual bool parse(BinParser& bp, std::vector<std::unique_ptr<T>>& results) = 0;

private:
  typename T::HeaderType header_;
  // URProducer producer_;
};

}  // namespace comm
}  // namespace ur_driver
