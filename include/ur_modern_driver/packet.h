#pragma once
#include "ur_modern_driver/bin_parser.h"

class Packet {
public:
    virtual bool parse_with(BinParser& bp) = 0;
};