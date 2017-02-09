#pragma once
#include <cstddef>
#include <inttypes.h>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/packet.h"
#include "ur_modern_driver/ur/master_board.h"
#include "ur_modern_driver/ur/robot_mode.h"



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


class RobotState : public Packet {
public:
    bool parse_with(BinParser &bp);
protected:
    virtual bool parse_package(BinParser &bp) = 0;
};

class RobotState_V1_6__7 : public RobotState {
protected:
    bool parse_package(BinParser &bp);
public:
    RobotModeData_V1_X robot_mode;
    //JointData
    //ToolData
    MasterBoardData_V1_X master_board;
    //CartesianInfo
};

class RobotState_V1_8 : public RobotState_V1_6__7 {
protected:
    bool parse_package(BinParser &bp);
public:
    
    //KinematicsInfo
    //ConfigurationData
    //ForceModeData
    //AdditionalInfo
    //CalibrationData
};


class RobotState_V3_0__1 : public RobotState {
protected:
    bool parse_package(BinParser &bp);
public:
    RobotModeData_V3_0__1 robot_mode;
    //JointData
    //ToolData
    MasterBoardData_V3_0__1 master_board;
    //CartesianInfo

    //KinematicsInfo
    //ConfigurationData
    //ForceModeData
    //AdditionalInfo
    //CalibrationData
};

class RobotState_V3_2 : public RobotState {
protected:
    bool parse_package(BinParser &bp);
public:
    RobotModeData_V3_2 robot_mode;
    //JointData
    //ToolData
    MasterBoardData_V3_2 master_board;
    //CartesianInfo

    //KinematicsInfo
    //ConfigurationData
    //ForceModeData
    //AdditionalInfo
    //CalibrationData
};
