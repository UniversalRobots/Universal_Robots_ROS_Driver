#include "ur_modern_driver/ur/robot_mode.h"


bool SharedRobotModeData::parse_with(BinParser &bp) {
    bp.parse(timestamp);
    bp.parse(physical_robot_connected);
    bp.parse(real_robot_enabled);
    bp.parse(robot_power_on);
    bp.parse(emergency_stopped);
    return true;
}


bool RobotModeData_V1_X::parse_with(BinParser &bp) {
    if(!bp.check_size<RobotModeData_V1_X>())
        return false;

    SharedRobotModeData::parse_with(bp);

    bp.parse(security_stopped);
    bp.parse(program_running);
    bp.parse(program_paused);
    bp.parse(robot_mode);
    bp.parse(speed_fraction);

    return true;
}


bool RobotModeData_V3_0__1::parse_with(BinParser &bp) {
    if(!bp.check_size<RobotModeData_V3_0__1>())
        return false;

    SharedRobotModeData::parse_with(bp);
    
    bp.parse(protective_stopped);
    bp.parse(program_running);
    bp.parse(program_paused);
    bp.parse(robot_mode);
    bp.parse(control_mode);
    bp.parse(target_speed_fraction);
    bp.parse(speed_scaling);

    return true;
}

bool RobotModeData_V3_2::parse_with(BinParser &bp) {
    if(!bp.check_size<RobotModeData_V3_2>())
        return false;

    RobotModeData_V3_0__1::parse_with(bp);

    bp.parse(target_speed_fraction_limit);

    return true;
}
