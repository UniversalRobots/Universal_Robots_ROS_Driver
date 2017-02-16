#include "ur_modern_driver/ur/state.h"
#include "ur_modern_driver/log.h"

//StatePacket::~StatePacket() { }

/*
bool RobotState::parse_with(BinParser &bp) {
    //continue as long as there are bytes to read
    while(!bp.empty()) {
        
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
        sub_parser.consume(sizeof(sub_size));

        if(!parse_package(sub_parser)) {
            LOG_ERROR("Failed to parse sub-package");
            return false;
        }

        if(!sub_parser.empty()) {
            LOG_ERROR("Sub-package was not parsed completely!");
            return false;
        }
    }
    return true;
}

template <typename T>
bool parse_base(BinParser &bp, T &pkg) {
    package_type type = bp.peek<package_type>();
    switch(type) {
        case package_type::ROBOT_MODE_DATA:
            LOG_DEBUG("Parsing robot_mode");
            bp.consume(sizeof(package_type));
            pkg.robot_mode.parse_with(bp);
            break;
        case package_type::MASTERBOARD_DATA:
            LOG_DEBUG("Parsing master_board");
            bp.consume(sizeof(package_type));
            pkg.master_board.parse_with(bp);
            break;

        case package_type::TOOL_DATA:
        case package_type::CARTESIAN_INFO:
        case package_type::JOINT_DATA:
            LOG_DEBUG("Skipping tool, cartesian or joint data");        
            //for unhandled packages we consume the rest of the 
            //package buffer
            bp.consume();
            break;
        default:
            return false;
    }
    return true;
}


template <typename T>
bool parse_advanced(BinParser &bp, T &pkg) {
    if(parse_base(bp, pkg))
        return true;

    package_type type = bp.peek<package_type>();

    switch(type) {
        case package_type::KINEMATICS_INFO:
        case package_type::CONFIGURATION_DATA:
        case package_type::ADDITIONAL_INFO:
        case package_type::CALIBRATION_DATA:
        case package_type::FORCE_MODE_DATA:
            LOG_DEBUG("Skipping kinematics, config, additional, calibration or force mode data");        
            //for unhandled packages we consume the rest of the 
            //package buffer
            bp.consume();
            break;
        default:
            LOG_ERROR("Invalid sub-package type parsed: %" PRIu8 "", type);
            return false;
    }

    return true;
}


bool RobotState_V1_6__7::parse_package(BinParser &bp) {
    if(!parse_base(bp, *this)) {
        LOG_ERROR("Invalid sub-package type parsed: %" PRIu8 "", bp.peek<package_type>());
        return false;
    }
    return true;
}

bool RobotState_V1_8::parse_package(BinParser &bp) {
    return parse_advanced(bp, *this);
}

bool RobotState_V3_0__1::parse_package(BinParser &bp) {
    return parse_advanced(bp, *this);
}

bool RobotState_V3_2::parse_package(BinParser &bp) {
    return parse_advanced(bp, *this);
}
*/