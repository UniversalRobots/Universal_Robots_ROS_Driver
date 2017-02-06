#pragma once

#include <cstddef>
#include <inttypes.h>
#include "ur_modern_driver/types.h"
#include "ur_modern_driver/bin_parser.h"


class SharedMasterBoardData {
public:
    virtual bool parse_with(BinParser &bp);
    
    int8_t analog_input_range0; 
    int8_t analog_input_range1;
    double analog_input0; 
    double analog_input1;
    int8_t analog_output_domain0; 
    int8_t analog_output_domain1;
    double analog_output0; 
    double analog_output1;
    float master_board_temperature;
    float robot_voltage_48V;
    float robot_current;
    float master_IO_current;

    bool euromap67_interface_installed;

    //euromap fields are dynamic so don't include in SIZE
    int32_t euromap_input_bits;
    int32_t euromap_output_bits;

    static const size_t SIZE = sizeof(double) * 4 
        + sizeof(int8_t) * 4
        + sizeof(float) * 4
        + sizeof(uint8_t);

    static const size_t EURO_SIZE = sizeof(int32_t) * 2;
};

class MasterBoardData_V1_X : public SharedMasterBoardData {
public:
    virtual bool parse_with(BinParser &bp);

    int16_t digital_input_bits;
    int16_t digital_output_bits;

    uint8_t master_safety_state;
    bool master_on_off_state;

    //euromap fields are dynamic so don't include in SIZE
    int16_t euromap_voltage;
    int16_t euromap_current;


    static const size_t SIZE = SharedMasterBoardData::SIZE
        + sizeof(int16_t) * 2
        + sizeof(uint8_t) * 2;

    static const size_t EURO_SIZE = SharedMasterBoardData::EURO_SIZE
        + sizeof(int16_t) * 2;
};

class MasterBoardData_V3_X : public SharedMasterBoardData {
public:
    virtual bool parse_with(BinParser &bp);

    int32_t digital_input_bits;
    int32_t digital_output_bits;

    uint8_t safety_mode;
    bool in_reduced_mode;

    //euromap fields are dynamic so don't include in SIZE
    float euromap_voltage;
    float euromap_current;


    static const size_t SIZE = SharedMasterBoardData::SIZE
        + sizeof(int32_t) * 2
        + sizeof(uint8_t) * 2;

    static const size_t EURO_SIZE = SharedMasterBoardData::EURO_SIZE
        + sizeof(float) * 2;
};
