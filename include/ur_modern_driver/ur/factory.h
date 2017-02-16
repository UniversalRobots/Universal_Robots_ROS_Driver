#pragma once

#include "ur_modern_driver/ur/consumer.h"
#include "ur_modern_driver/ur/messages_parser.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_parser.h"
#include "ur_modern_driver/ur/state_parser.h"
#include "ur_modern_driver/ur/stream.h"
#include <cstdlib>

class URFactory : private URMessagePacketConsumer {
private:
    URStream _stream;
    URMessageParser _parser;

    uint8_t _major_version;
    uint8_t _minor_version;

    bool consume(VersionMessage& vm)
    {
        LOG_INFO("Got VersionMessage:");
        LOG_INFO("project name: %s", vm.project_name.c_str());
        LOG_INFO("version: %u.%u.%d", vm.major_version, vm.minor_version, vm.svn_version);
        LOG_INFO("build date: %s", vm.build_date.c_str());

        _major_version = vm.major_version;
        _minor_version = vm.minor_version;

        return true;
    }

    void setup_consumer() {}
    void teardown_consumer() {}
    void stop_consumer() {}

public:
    URFactory(std::string& host)
        : _stream(host, 30001)
    {
        URProducer<MessagePacket> p(_stream, _parser);
        std::vector<unique_ptr<MessagePacket> > results;

        p.setup_producer();

        if (!p.try_get(results) || results.size() == 0) {
            LOG_FATAL("No version message received, init failed!");
            std::exit(EXIT_FAILURE);
        }

        for (auto const& p : results) {
            p->consume_with(*this);
        }

        if (_major_version == 0 && _minor_version == 0) {
            LOG_FATAL("No version message received, init failed!");
            std::exit(EXIT_FAILURE);
        }

        p.teardown_producer();
    }

    std::unique_ptr<URParser<StatePacket> > get_state_parser()
    {
        if (_major_version == 1) {
            return std::unique_ptr<URParser<StatePacket> >(new URStateParser_V1_X);
        } else {
            if (_minor_version < 3)
                return std::unique_ptr<URParser<StatePacket> >(new URStateParser_V3_0__1);
            else
                return std::unique_ptr<URParser<StatePacket> >(new URStateParser_V3_2);
        }
    }

    std::unique_ptr<URParser<RTPacket> > get_rt_parser()
    {
        if (_major_version == 1) {
            if (_minor_version < 8)
                return std::unique_ptr<URParser<RTPacket> >(new URRTStateParser_V1_6__7);
            else
                return std::unique_ptr<URParser<RTPacket> >(new URRTStateParser_V1_8);
        } else {
            if (_minor_version < 3)
                return std::unique_ptr<URParser<RTPacket> >(new URRTStateParser_V3_0__1);
            else
                return std::unique_ptr<URParser<RTPacket> >(new URRTStateParser_V3_2__3);
        }
    }
};