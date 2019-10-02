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

#include <assert.h>
#include <endian.h>
#include <inttypes.h>
#include <array>
#include <bitset>
#include <cstddef>
#include <cstring>
#include <string>
#include <memory>
#include "ur_robot_driver/log.h"
#include "ur_robot_driver/types.h"
#include "ur_robot_driver/exceptions.h"

namespace ur_driver
{
namespace comm
{
/*!
 * \brief The BinParser class handles a byte buffer and functionality to iteratively parse the
 * content.
 */
class BinParser
{
private:
  uint8_t *buf_pos_, *buf_end_;
  BinParser& parent_;

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

public:
  /*!
   * \brief Creates a new BinParser object from a given buffer.
   *
   * \param buffer The byte buffer to parse
   * \param buf_len Size of the buffer
   */
  BinParser(uint8_t* buffer, size_t buf_len) : buf_pos_(buffer), buf_end_(buffer + buf_len), parent_(*this)
  {
    assert(buf_pos_ <= buf_end_);
  }

  /*!
   * \brief Creates a new BinParser object for part of a buffer from a parent BinParser.
   *
   * \param parent Parent BinParser
   * \param sub_len Size of the sub-buffer to parse
   */
  BinParser(BinParser& parent, size_t sub_len)
    : buf_pos_(parent.buf_pos_), buf_end_(parent.buf_pos_ + sub_len), parent_(parent)
  {
    assert(buf_pos_ <= buf_end_);
  }

  /*!
   * \brief Deconstructor for the BinParser.
   */
  ~BinParser()
  {
    parent_.buf_pos_ = buf_pos_;
  }

  /*!
   * \brief Parses the next bytes as given type without moving the buffer pointer.
   *
   * @tparam T Type to parse as
   *
   * \returns Value of the next bytes as type T
   */
  template <typename T>
  T peek()
  {
    if (buf_pos_ + sizeof(T) > buf_end_)
      throw UrException("Could not parse received package. This can occur if the driver is started while the robot is "
                        "booting - please restart the driver once the robot has finished booting. "
                        "If the problem persists after the robot has booted, please contact the package maintainer.");
    T val;
    std::memcpy(&val, buf_pos_, sizeof(T));
    return decode(val);
  }

  /*!
   * \brief Parses the next bytes as given type.
   *
   * @tparam T Type to parse as
   * \param val Reference to write the parsed value to
   */
  template <typename T>
  void parse(T& val)
  {
    val = peek<T>();
    buf_pos_ += sizeof(T);
  }

  /*!
   * \brief Parses the next bytes as a double.
   *
   * \param val Reference to write the parsed value to
   */
  void parse(double& val)
  {
    uint64_t inner;
    parse<uint64_t>(inner);
    std::memcpy(&val, &inner, sizeof(double));
  }
  /*!
   * \brief Parses the next bytes as a float.
   *
   * \param val Reference to write the parsed value to
   */
  void parse(float& val)
  {
    uint32_t inner;
    parse<uint32_t>(inner);
    std::memcpy(&val, &inner, sizeof(float));
  }

  /*!
   * \brief Parses the next byte as a bool.
   *
   * UR uses 1 byte for boolean values but sizeof(bool) is implementation defined, so we must ensure
   * they're parsed as uint8_t on all compilers
   *
   *
   * \param val Reference to write the parsed value to
   */
  void parse(bool& val)
  {
    uint8_t inner;
    parse<uint8_t>(inner);
    val = inner != 0;
  }

  /*!
   * \brief Parses the next bytes as a vector of 3 doubles.
   *
   * \param val Reference to write the parsed value to
   */
  void parse(vector3d_t& val)
  {
    for (size_t i = 0; i < val.size(); ++i)
    {
      parse(val[i]);
    }
  }

  /*!
   * \brief Parses the next bytes as a vector of 6 doubles.
   *
   * \param val Reference to write the parsed value to
   */
  void parse(vector6d_t& val)
  {
    for (size_t i = 0; i < val.size(); ++i)
    {
      parse(val[i]);
    }
  }

  /*!
   * \brief Parses the next bytes as a vector of 6 32 bit integers.
   *
   * \param val Reference to write the parsed value to
   */
  void parse(vector6int32_t& val)
  {
    for (size_t i = 0; i < val.size(); ++i)
    {
      parse(val[i]);
    }
  }

  /*!
   * \brief Parses the next bytes as a vector of 6 unsigned 32 bit integers.
   *
   * \param val Reference to write the parsed value to
   */
  void parse(vector6uint32_t& val)
  {
    for (size_t i = 0; i < val.size(); ++i)
    {
      parse(val[i]);
    }
  }

  /*!
   * \brief Writes the remaining bytes into a given buffer without parsing them.
   *
   * \param buffer The buffer to copy the remaining bytes to.
   * \param buffer_length Reference to write the number of remaining bytes to.
   */
  void rawData(std::unique_ptr<uint8_t>& buffer, size_t& buffer_length)
  {
    buffer_length = buf_end_ - buf_pos_;
    buffer.reset(new uint8_t[buffer_length]);
    memcpy(buffer.get(), buf_pos_, buffer_length);
    consume();
  }

  /*!
   * \brief Parses the remaining bytes as a string.
   *
   * \param val Reference to write the parsed value to
   */
  void parseRemainder(std::string& val)
  {
    parse(val, size_t(buf_end_ - buf_pos_));
  }

  /*!
   * \brief Parses a given number of bytes as a string.
   *
   * \param val Reference to write the parsed value to
   * \param len Number of bytes to parse
   */
  void parse(std::string& val, size_t len)
  {
    val.assign(reinterpret_cast<char*>(buf_pos_), len);
    buf_pos_ += len;
  }

  /*!
   * \brief Special string parse function that assumes uint8_t len followed by chars
   *
   * \param val Reference to write the parsed value to
   */
  void parse(std::string& val)
  {
    uint8_t len;
    parse(len);
    parse(val, size_t(len));
  }

  /*!
   * \brief Parses the next bytes as an array of a given type and size.
   *
   * @tparam T Type of the array
   * @tparam N Number of elements in the array
   * \param array Reference of an array to parse to.
   */
  template <typename T, size_t N>
  void parse(std::array<T, N>& array)
  {
    for (size_t i = 0; i < N; i++)
    {
      parse(array[i]);
    }
  }

  /*!
   * \brief Parses the next bytes as a value of a given type, but also copies it to a bitset.
   *
   * @tparam T Type to parse as
   * @tparam N Size of the bitset to copy to
   * \param set Reference to the bitset to copy the value to.
   */
  template <typename T, size_t N>
  void parse(std::bitset<N>& set)
  {
    T val;
    parse(val);
    set = std::bitset<N>(val);
  }

  /*!
   * \brief Sets the current buffer position to the end of the buffer, finishing parsing.
   */
  void consume()
  {
    buf_pos_ = buf_end_;
  }
  /*!
   * \brief Moves the current buffer position ahead by a given amount.
   *
   * \param bytes Number of bytes to move the buffer position
   */
  void consume(size_t bytes)
  {
    buf_pos_ += bytes;
  }

  /*!
   * \brief Checks if at least a given number of bytes is still remaining unparsed in the buffer.
   *
   * \param bytes Number of bytes to check for
   *
   * \returns True, if at least the given number of bytes are unparsed, false otherwise.
   */
  bool checkSize(size_t bytes)
  {
    return bytes <= size_t(buf_end_ - buf_pos_);
  }
  /*!
   * \brief Checks if enough bytes for a given type remain unparsed in the buffer.
   *
   * @tparam T The type to check for
   *
   * \returns True, if enough bytes are unparsed, false otherwise.
   */
  template <typename T>
  bool checkSize(void)
  {
    return checkSize(T::SIZE);
  }

  /*!
   * \brief Checks if no unparsed bytes remain in the buffer.
   *
   * \returns True, if all bytes were parsed, false otherwise.
   */
  bool empty()
  {
    return buf_pos_ == buf_end_;
  }

  /*!
   * \brief Logs debugging information about the BinParser object.
   */
  void debug()
  {
    LOG_DEBUG("BinParser: %p - %p (%zu bytes)", buf_pos_, buf_end_, buf_end_ - buf_pos_);
  }
};

}  // namespace comm
}  // namespace ur_driver
