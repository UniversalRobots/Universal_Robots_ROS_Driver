#pragma once
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/packet.h"

class Parser {
public:
    virtual std::unique_ptr<Packet> parse(BinParser& bp) = 0;
};
