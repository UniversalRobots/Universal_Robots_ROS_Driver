#pragma once
#include <vector>
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/bin_parser.h"

template <typename T>
class URParser {
public:
    virtual bool parse(BinParser &bp, std::vector<std::unique_ptr<T>> &results) = 0;
};
