#pragma once
#include "ur_modern_driver/bin_parser.h"

class Packet
{
public:
  virtual bool parseWith(BinParser& bp) = 0;
};