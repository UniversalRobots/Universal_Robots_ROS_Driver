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

#include <inttypes.h>
#include <array>
#include <iostream>

namespace ur_driver
{
using vector3d_t = std::array<double, 3>;
using vector6d_t = std::array<double, 6>;
using vector6int32_t = std::array<int32_t, 6>;
using vector6uint32_t = std::array<uint32_t, 6>;

template <class T, std::size_t N>
std::ostream& operator<<(std::ostream& out, const std::array<T, N>& item)
{
  out << "[";
  for (size_t i = 0; i < item.size(); ++i)
  {
    out << item[i];
    if (i != item.size() - 1)
    {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

/*!
 * \brief Converts an enum type to its underlying type
 *
 * \param e Enum value that should be converted
 *
 * \returns Enum value converted to underlying type
 */
template <typename E>
constexpr typename std::underlying_type<E>::type toUnderlying(const E e) noexcept
{
  return static_cast<typename std::underlying_type<E>::type>(e);
}
}  // namespace ur_driver
