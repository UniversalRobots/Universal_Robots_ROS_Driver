/*
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

#include <assert.h>
#include <endian.h>
#include <inttypes.h>
#include <array>
#include <bitset>
#include <cstddef>
#include <cstring>
#include <string>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/types.h"

class BinParser
{
private:
  uint8_t *buf_pos_, *buf_end_;
  BinParser& parent_;

public:
  BinParser(uint8_t* buffer, size_t buf_len) : buf_pos_(buffer), buf_end_(buffer + buf_len), parent_(*this)
  {
    assert(buf_pos_ <= buf_end_);
  }

  BinParser(BinParser& parent, size_t sub_len)
    : buf_pos_(parent.buf_pos_), buf_end_(parent.buf_pos_ + sub_len), parent_(parent)
  {
    assert(buf_pos_ <= buf_end_);
  }

  ~BinParser()
  {
    parent_.buf_pos_ = buf_pos_;
  }

  // Decode from network encoding (big endian) to host encoding
  template <typename T>
  T decode(T val)
  {
    return val;
  }
  uint16_t decode(uint16_t val)
  {
    return be16toh(val);
  }
  uint32_t decode(uint32_t val)
  {
    return be32toh(val);
  }
  uint64_t decode(uint64_t val)
  {
    return be64toh(val);
  }
  int16_t decode(int16_t val)
  {
    return be16toh(val);
  }
  int32_t decode(int32_t val)
  {
    return be32toh(val);
  }
  int64_t decode(int64_t val)
  {
    return be64toh(val);
  }

  template <typename T>
  T peek()
  {
    assert(buf_pos_ + sizeof(T) <= buf_end_);
    T val;
    std::memcpy(&val, buf_pos_, sizeof(T));
    return decode(val);
  }

  template <typename T>
  void parse(T& val)
  {
    val = peek<T>();
    buf_pos_ += sizeof(T);
  }

  void parse(double& val)
  {
    uint64_t inner;
    parse<uint64_t>(inner);
    std::memcpy(&val, &inner, sizeof(double));
  }
  void parse(float& val)
  {
    uint32_t inner;
    parse<uint32_t>(inner);
    std::memcpy(&val, &inner, sizeof(float));
  }

  // UR uses 1 byte for boolean values but sizeof(bool) is implementation
  // defined so we must ensure they're parsed as uint8_t on all compilers
  void parse(bool& val)
  {
    uint8_t inner;
    parse<uint8_t>(inner);
    val = inner != 0;
  }

  // Explicit parsing order of fields to avoid issues with struct layout
  void parse(double3_t& val)
  {
    parse(val.x);
    parse(val.y);
    parse(val.z);
  }

  // Explicit parsing order of fields to avoid issues with struct layout
  void parse(cartesian_coord_t& val)
  {
    parse(val.position);
    parse(val.rotation);
  }

  void parse_remainder(std::string& val)
  {
    parse(val, size_t(buf_end_ - buf_pos_));
  }

  void parse(std::string& val, size_t len)
  {
    val.assign(reinterpret_cast<char*>(buf_pos_), len);
    buf_pos_ += len;
  }

  // Special string parse function that assumes uint8_t len followed by chars
  void parse(std::string& val)
  {
    uint8_t len;
    parse(len);
    parse(val, size_t(len));
  }

  template <typename T, size_t N>
  void parse(std::array<T, N>& array)
  {
    for (size_t i = 0; i < N; i++)
    {
      parse(array[i]);
    }
  }

  template <typename T, size_t N>
  void parse(std::bitset<N>& set)
  {
    T val;
    parse(val);
    set = std::bitset<N>(val);
  }

  void consume()
  {
    buf_pos_ = buf_end_;
  }
  void consume(size_t bytes)
  {
    buf_pos_ += bytes;
  }

  bool checkSize(size_t bytes)
  {
    return bytes <= size_t(buf_end_ - buf_pos_);
  }
  template <typename T>
  bool checkSize(void)
  {
    return checkSize(T::SIZE);
  }

  bool empty()
  {
    return buf_pos_ == buf_end_;
  }

  void debug()
  {
    LOG_DEBUG("BinParser: %p - %p (%zu bytes)", buf_pos_, buf_end_, buf_end_ - buf_pos_);
  }
};
