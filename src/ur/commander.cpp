#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/log.h"

bool URCommander::write(const std::string &s)
{
  size_t len = s.size();
  const uint8_t *data = reinterpret_cast<const uint8_t *>(s.c_str());
  size_t written;
  return stream_.write(data, len, written);
}

void URCommander::formatArray(std::ostringstream &out, std::array<double, 6> &values)
{
  std::string mod("[");
  for (auto const &val : values)
  {
    out << mod << val;
    mod = ",";
  }
  out << "]";
}

bool URCommander::uploadProg(const std::string &s)
{
  LOG_DEBUG("Sending program [%s]", s.c_str());
  return write(s);
}

bool URCommander::setToolVoltage(uint8_t voltage)
{
  if (voltage != 0 || voltage != 12 || voltage != 24)
    return false;

  std::ostringstream out;
  out << "set_tool_voltage(" << (int)voltage << ")\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander::setFlag(uint8_t pin, bool value)
{
  std::ostringstream out;
  out << "set_flag(" << (int)pin << "," << (value ? "True" : "False") << ")\n";
  std::string s(out.str());
  return write(s);
}
bool URCommander::setPayload(double value)
{
  std::ostringstream out;
  out << "set_payload(" << std::fixed << std::setprecision(5) << value << ")\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander::stopj(double a)
{
  std::ostringstream out;
  out << "stopj(" << std::fixed << std::setprecision(5) << a << ")\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V1_X::speedj(std::array<double, 6> &speeds, double acceleration)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(5);
  out << "speedj(";
  formatArray(out, speeds);
  out << "," << acceleration << "," << 0.02 << ")\n";
  std::string s(out.str());
  return write(s);
}
bool URCommander_V1_X::setAnalogOut(uint8_t pin, double value)
{
  std::ostringstream out;
  out << "sec io_fun():\n"
      << "set_analog_out(" << (int)pin << "," << std::fixed << std::setprecision(4) << value << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V1_X::setDigitalOut(uint8_t pin, bool value)
{
  std::ostringstream out;
  out << "sec io_fun():\n"
      << "set_digital_out(" << (int)pin << "," << (value ? "True" : "False") << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V3_X::setAnalogOut(uint8_t pin, double value)
{
  std::ostringstream out;
  out << "sec io_fun():\n"
      << "set_standard_analog_out(" << (int)pin << "," << std::fixed << std::setprecision(5) << value << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V3_X::setDigitalOut(uint8_t pin, bool value)
{
  std::ostringstream out;
  std::string func;

  if (pin < 8)
  {
    func = "set_standard_digital_out";
  }
  else if (pin < 16)
  {
    func = "set_configurable_digital_out";
    pin -= 8;
  }
  else if (pin < 18)
  {
    func = "set_tool_digital_out";
    pin -= 16;
  }
  else
    return false;

  out << "sec io_fun():\n"
      << func << "(" << (int)pin << "," << (value ? "True" : "False") << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V3_1__2::speedj(std::array<double, 6> &speeds, double acceleration)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(5);
  out << "speedj(";
  formatArray(out, speeds);
  out << "," << acceleration << ")\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V3_3::speedj(std::array<double, 6> &speeds, double acceleration)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(5);
  out << "speedj(";
  formatArray(out, speeds);
  out << "," << acceleration << "," << 0.008 << ")\n";
  std::string s(out.str());
  return write(s);
}
