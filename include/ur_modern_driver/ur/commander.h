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
#include <array>
#include <iomanip>
#include <sstream>
#include "ur_modern_driver/ur/stream.h"

class URCommander
{
private:
  URStream &stream_;

protected:
  bool write(const std::string &s);
  void formatArray(std::ostringstream &out, std::array<double, 6> &values);

public:
  URCommander(URStream &stream) : stream_(stream)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration) = 0;
  virtual bool setDigitalOut(uint8_t pin, bool value) = 0;
  virtual bool setAnalogOut(uint8_t pin, double value) = 0;

  // shared
  bool uploadProg(const std::string &s);
  bool stopj(double a = 10.0);
  bool setToolVoltage(uint8_t voltage);
  bool setFlag(uint8_t pin, bool value);
  bool setPayload(double value);
};

class URCommander_V1_X : public URCommander
{
public:
  URCommander_V1_X(URStream &stream) : URCommander(stream)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration);
  virtual bool setDigitalOut(uint8_t pin, bool value);
  virtual bool setAnalogOut(uint8_t pin, double value);
};

class URCommander_V3_X : public URCommander
{
public:
  URCommander_V3_X(URStream &stream) : URCommander(stream)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration) = 0;
  virtual bool setDigitalOut(uint8_t pin, bool value);
  virtual bool setAnalogOut(uint8_t pin, double value);
};

class URCommander_V3_1__2 : public URCommander_V3_X
{
public:
  URCommander_V3_1__2(URStream &stream) : URCommander_V3_X(stream)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration);
};

class URCommander_V3_3 : public URCommander_V3_X
{
public:
  URCommander_V3_3(URStream &stream) : URCommander_V3_X(stream)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration);
};
