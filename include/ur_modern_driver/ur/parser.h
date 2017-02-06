#pragma once
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/parser.h"
#include "ur_modern_driver/ur/state.h"

template <typename T>
class URStateParser : public Parser {
    std::unique_ptr<Packet> parse(BinParser &bp)  {
        int32_t packet_size = bp.peek<int32_t>();
        message_type type;

        if(!bp.check_size(packet_size)) {
            LOG_ERROR("Buffer len shorter than expected packet length\n");
            return std::unique_ptr<Packet>(nullptr);        
        }

        bp.parse(packet_size); //consumes the peeked data
        bp.parse(type);

        if(type != message_type::ROBOT_STATE) {
            LOG_ERROR("Invalid message type recieved: %u\n", static_cast<uint8_t>(type));
            return std::unique_ptr<Packet>(nullptr);
        }

        std::unique_ptr<Packet> obj(new T);
        if(obj->parse_with(bp))
            return obj;
        
        return std::unique_ptr<Packet>(nullptr);
    }
};


template <typename T>
class URRTStateParser : public Parser {
    std::unique_ptr<Packet> parse(BinParser &bp)  {
        int32_t packet_size = bp.peek<int32_t>();

        if(!bp.check_size(packet_size)) {
            LOG_ERROR("Buffer len shorter than expected packet length\n");
            return std::unique_ptr<Packet>(nullptr);        
        }

        bp.parse(packet_size); //consumes the peeked data

        std::unique_ptr<Packet> obj(new T);
        if(obj->parse_with(bp))
            return obj;
        
        return std::unique_ptr<Packet>(nullptr);
    }
};