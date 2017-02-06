#include "ur_modern_driver/ur/rt_state.h"

bool RTState_V1_6__7::parse_with(BinParser &bp) {
    if(!bp.check_size<RTState_V1_6__7>())
        return false;

    bp.parse(time);
    bp.parse(q_target);
    bp.parse(qd_target);
    bp.parse(qdd_target);
    bp.parse(i_target);
    bp.parse(m_target);
    bp.parse(q_actual);
    bp.parse(qd_actual);
    bp.parse(i_actual);
    bp.parse(tool_accelerometer_values);
    bp.parse(tcp_force);
    bp.parse(tool_vector);
    bp.parse(tcp_speed);
    bp.parse(digital_input);
    bp.parse(motor_temperatures);
    bp.parse(controller_time);
    bp.parse(robot_mode);

    return true;
}

bool RTState_V1_8::parse_with(BinParser &bp) {
    if(!bp.check_size<RTState_V1_8>())
        return false;

    RTState_V1_6__7::parse_with(bp);

    bp.parse(joint_modes);

    return true;
}