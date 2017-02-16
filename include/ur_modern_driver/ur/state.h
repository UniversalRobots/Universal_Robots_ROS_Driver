#pragma once

#include <cstddef>
#include <inttypes.h>
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/log.h"

enum class package_type : uint8_t {
	ROBOT_MODE_DATA = 0,
	JOINT_DATA = 1,
	TOOL_DATA = 2,
	MASTERBOARD_DATA = 3,
	CARTESIAN_INFO = 4,
	KINEMATICS_INFO = 5,
	CONFIGURATION_DATA = 6,
	FORCE_MODE_DATA = 7,
	ADDITIONAL_INFO = 8,
	CALIBRATION_DATA = 9
};

enum class message_type : uint8_t {
	ROBOT_STATE = 16, 
    ROBOT_MESSAGE = 20, 
    PROGRAM_STATE_MESSAGE = 25
};

class URStatePacketConsumer;

class StatePacket {
public:
    virtual bool parse_with(BinParser &bp) = 0;
    virtual bool consume_with(URStatePacketConsumer &consumer) = 0;
};
