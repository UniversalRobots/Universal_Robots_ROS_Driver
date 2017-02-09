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

    //blocking call
    ssize_t len = _stream.receive(buf, sizeof(buf));

    LOG_DEBUG("Read %d bytes from stream", len);

    if(len < 1) {
        LOG_WARN("Read nothing from stream");
        return std::unique_ptr<Packet>(nullptr);
    }

    BinParser bp(buf, static_cast<size_t>(len));
    return std::move(_parser.parse(bp));
}