#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/log.h"

bool URCommander::write(std::string& s)
{
    size_t len = s.size();
    const uint8_t* data = reinterpret_cast<const uint8_t*>(s.c_str());
    ssize_t res = stream_.send(data, len);
    return res > 0 && static_cast<size_t>(res) == len;
}


bool URCommander::speedj(std::array<double, 6> &speeds, double acceleration)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    out << "speedj([";
    std::string mod;
    for(auto const& val : speeds)
    {
        out << mod << val;
        mod = ",";
    }
    out << "]," << acceleration << ")\n";
    std::string s(out.str());
    return write(s);
}
bool URCommander::stopj(double a)
{

}

bool URCommander::setAnalogOut(uint8_t pin, double value)
{
    std::ostringstream out;
    out << "set_analog_out(" << (int)pin << "," << std::fixed << std::setprecision(4) << value << ")\n";
    std::string s(out.str());
    return write(s);
}

bool URCommander::setDigitalOut(uint8_t pin, bool value)
{
    std::ostringstream out;
    out << "set_digital_out(" << (int)pin << "," << (value ? "True" : "False") << ")\n";
    std::string s(out.str());
    return write(s);
}

bool URCommander::setToolVoltage(uint8_t voltage)
{
    
}


bool URCommander::setFlag(bool value)
{

}
bool URCommander::setPayload(double value)
{

}