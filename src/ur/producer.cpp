#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/log.h"

void URProducer::setup_producer() {
    _stream.connect();
}

void URProducer::teardown_producer() {

}

void URProducer::stop_producer() {
    _stream.disconnect();
}

std::unique_ptr<Packet> URProducer::try_get() {
    //4KB should be enough to hold any packet received from UR
    uint8_t buf[4096];
    
    ssize_t total = 0;
    int32_t packet_size = 0;

    //deal with partial recieves 
    while(total <= sizeof(buf)) {
        uint8_t *pos = buf + total;
        size_t size = sizeof(buf) - total;

        //blocking call
        ssize_t len = _stream.receive(pos, size);
        
        if(len < 1) {
            LOG_DEBUG("Read nothing from stream");
            return std::unique_ptr<Packet>(nullptr);
        }

        total += len;
        BinParser bp(buf, static_cast<size_t>(total));

        if(packet_size == 0) {
            packet_size = bp.peek<int32_t>();
            //TODO: check other wrong packet sizes?
            if(packet_size > sizeof(buf)) {
                LOG_ERROR("A packet with 'len' (%d) larger than buffer was received, discarding...", packet_size);
                return std::unique_ptr<Packet>(nullptr);
            }
        }

        if(total < packet_size){
            LOG_DEBUG("Partial packet recieved");
            continue;
        }
        return std::move(_parser.parse(bp));
    }
}