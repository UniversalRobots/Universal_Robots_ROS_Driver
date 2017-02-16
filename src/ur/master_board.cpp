#include "ur_modern_driver/ur/master_board.h"
#include "ur_modern_driver/ur/consumer.h"

bool SharedMasterBoardData::parse_with(BinParser &bp) {
    bp.parse(analog_input_range0);
    bp.parse(analog_input_range1);
    bp.parse(analog_input0);
    bp.parse(analog_input1);    
    bp.parse(analog_output_domain0);
    bp.parse(analog_output_domain1);
    bp.parse(analog_output0);
    bp.parse(analog_output1);
    bp.parse(master_board_temperature);
    bp.parse(robot_voltage_48V);
    bp.parse(robot_current);
    bp.parse(master_IO_current);
    return true;
}

bool MasterBoardData_V1_X::parse_with(BinParser &bp) {
    if(!bp.check_size<MasterBoardData_V1_X>())
        return false;

    bp.parse(digital_input_bits);
    bp.parse(digital_output_bits);

    SharedMasterBoardData::parse_with(bp);

    bp.parse(master_safety_state);
    bp.parse(master_on_off_state);
    bp.parse(euromap67_interface_installed);

    if(euromap67_interface_installed) {
        if(!bp.check_size(MasterBoardData_V1_X::EURO_SIZE))
            return false;

        bp.parse(euromap_voltage);
        bp.parse(euromap_current);
    }

    return true;
}

bool MasterBoardData_V3_0__1::parse_with(BinParser &bp) {
    if(!bp.check_size<MasterBoardData_V3_0__1>())
        return false;

    bp.parse(digital_input_bits);
    bp.parse(digital_output_bits);
    
    SharedMasterBoardData::parse_with(bp);

    bp.parse(safety_mode);
    bp.parse(in_reduced_mode);
    bp.parse(euromap67_interface_installed);

    if(euromap67_interface_installed) {
        if(!bp.check_size(MasterBoardData_V3_0__1::EURO_SIZE))
            return false;
            
        bp.parse(euromap_voltage);
        bp.parse(euromap_current);
    }

    bp.consume(sizeof(uint32_t));

    return true;
}



bool MasterBoardData_V3_2::parse_with(BinParser &bp) {
    if(!bp.check_size<MasterBoardData_V3_2>())
        return false;

    MasterBoardData_V3_0__1::parse_with(bp);

    bp.parse(operational_mode_selector_input);
    bp.parse(three_position_enabling_device_input);

    return true;
}





bool MasterBoardData_V1_X::consume_with(URStatePacketConsumer &consumer) {
    return consumer.consume(*this);
}
bool MasterBoardData_V3_0__1::consume_with(URStatePacketConsumer &consumer) {
    return consumer.consume(*this);
}
bool MasterBoardData_V3_2::consume_with(URStatePacketConsumer &consumer) {
    return consumer.consume(*this);
}