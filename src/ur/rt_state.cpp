#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/consumer.h"

bool RTShared::parse_shared1(BinParser& bp)
{
    bp.parse(time);
    bp.parse(q_target);
    bp.parse(qd_target);
    bp.parse(qdd_target);
    bp.parse(i_target);
    bp.parse(m_target);
    bp.parse(q_actual);
    bp.parse(qd_actual);
    bp.parse(i_actual);
    return true;
}

bool RTShared::parse_shared2(BinParser& bp)
{
    bp.parse(digital_input);
    bp.parse(motor_temperatures);
    bp.parse(controller_time);
    bp.consume(sizeof(double)); //Unused "Test value" field
    bp.parse(robot_mode);
    return true;
}

bool RTState_V1_6__7::parse_with(BinParser& bp)
{
    if (!bp.check_size<RTState_V1_6__7>())
        return false;

    parse_shared1(bp);

    bp.parse(tool_accelerometer_values);
    bp.parse(tcp_force);
    bp.parse(tool_vector_actual);
    bp.parse(tcp_speed_actual);

    parse_shared2(bp);

    return true;
}

bool RTState_V1_8::parse_with(BinParser& bp)
{
    if (!bp.check_size<RTState_V1_8>())
        return false;

    RTState_V1_6__7::parse_with(bp);

    bp.parse(joint_modes);

    return true;
}

bool RTState_V3_0__1::parse_with(BinParser& bp)
{
    if (!bp.check_size<RTState_V3_0__1>())
        return false;

    parse_shared1(bp);

    bp.parse(i_control);
    bp.parse(tool_vector_actual);
    bp.parse(tcp_speed_actual);
    bp.parse(tcp_force);
    bp.parse(tool_vector_target);
    bp.parse(tcp_speed_target);

    parse_shared2(bp);

    bp.parse(joint_modes);
    bp.parse(safety_mode);
    bp.consume(sizeof(double[6])); //skip undocumented
    bp.parse(tool_accelerometer_values);
    bp.consume(sizeof(double[6])); //skip undocumented
    bp.parse(speed_scaling);
    bp.parse(linear_momentum_norm);
    bp.consume(sizeof(double)); //skip undocumented
    bp.consume(sizeof(double)); //skip undocumented
    bp.parse(v_main);
    bp.parse(v_robot);
    bp.parse(i_robot);
    bp.parse(v_actual);

    return true;
}

bool RTState_V3_2__3::parse_with(BinParser& bp)
{
    if (!bp.check_size<RTState_V3_2__3>())
        return false;

    RTState_V3_0__1::parse_with(bp);

    bp.parse(digital_outputs);
    bp.parse(program_state);

    return true;
}

bool RTState_V1_6__7::consume_with(URRTPacketConsumer& consumer)
{
    return consumer.consume(*this);
}
bool RTState_V1_8::consume_with(URRTPacketConsumer& consumer)
{
    return consumer.consume(*this);
}
bool RTState_V3_0__1::consume_with(URRTPacketConsumer& consumer)
{
    return consumer.consume(*this);
}
bool RTState_V3_2__3::consume_with(URRTPacketConsumer& consumer)
{
    return consumer.consume(*this);
}