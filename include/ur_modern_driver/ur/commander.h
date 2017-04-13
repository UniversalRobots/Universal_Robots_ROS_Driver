#pragma once
#include <array>
#include <sstream>
#include <iomanip>
#include "ur_modern_driver/ur/stream.h"

class URCommander
{
private:
  URStream& stream_;

protected:
  bool write(std::string& s);

public:
  URCommander(URStream& stream) : stream_(stream)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration);
  virtual bool stopj(double a = 10.0);
  virtual bool setDigitalOut(uint8_t pin, bool value);
  virtual bool setAnalogOut(uint8_t pin, double value);
  virtual bool setToolVoltage(uint8_t voltage);
  virtual bool setFlag(bool value);
  virtual bool setPayload(double value);
};