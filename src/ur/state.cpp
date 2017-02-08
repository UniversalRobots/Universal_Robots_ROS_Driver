#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/state.h"


bool RobotState::parse_with(BinParser &bp) {
    //continue as long as there are bytes to read
    while(bp.check_size(sizeof(uint8_t))) {
        if(!bp.check_size(sizeof(uint32_t))){
            LOG_ERROR("Failed to read sub-package length, there's likely a parsing error");
            return false;    
        }

        uint32_t sub_size = bp.peek<uint32_t>();
        if(!bp.check_size(static_cast<size_t>(sub_size))) {
            LOG_WARN("Invalid sub-package size of %" PRIu32 " received!", sub_size);
            return false;
        }
        
        //deconstruction of a sub parser will increment the position of the parent parser 
        BinParser sub_parser(bp, sub_size);
        sub_parser.parse(sub_size);

        if(!parse_package(sub_parser))
            return false;
    }
    return true;
}

bool RobotState_V1_6__7::parse_package(BinParser &bp) {
    //todo: size checks
    package_type type;
    bp.parse(type);

    switch(type) {
        case package_type::ROBOT_MODE_DATA:
            robot_mode.parse_with(bp);
            break;
        case package_type::JOINT_DATA:
            //TODO
            break;
        case package_type::TOOL_DATA:
            //TODO
            break;
        case package_type::MASTERBOARD_DATA:
            master_board.parse_with(bp);
            break;
        case package_type::CARTESIAN_INFO:
            //TODO
            break;
        default:
            LOG_ERROR("Invalid package type parsed: %" PRIu8 "", type);
            return false;
    }

    return true;
}

bool RobotState_V1_8::parse_package(BinParser &bp) {
    package_type type = bp.peek<package_type>();
    switch(type) {
        case package_type::KINEMATICS_INFO:
            break;
        case package_type::CONFIGURATION_DATA:
            break;
        case package_type::ADDITIONAL_INFO:
            break;
        case package_type::CALIBRATION_DATA:
            break;
        default:
            return RobotState_V1_6__7::parse_package(bp);
    }
    return true;
}