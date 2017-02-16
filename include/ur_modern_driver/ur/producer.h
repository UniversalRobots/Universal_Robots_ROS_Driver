#pragma once
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/stream.h"
#include "ur_modern_driver/ur/parser.h"

template <typename T>
class URProducer : public IProducer<T> {
private:
    URStream &_stream;
    URParser<T> &_parser;

public:
    URProducer(URStream &stream, URParser<T> &parser) 
        : _stream(stream), 
        _parser(parser) { }
    
    void setup_producer() {
        _stream.connect();
    }
    void teardown_producer() {
        _stream.disconnect();
    }
    void stop_producer() {
        _stream.disconnect();
    }
    
    bool try_get(std::vector<unique_ptr<T>> &products) {
        //4KB should be enough to hold any packet received from UR
        uint8_t buf[4096];

        //blocking call
        ssize_t len = _stream.receive(buf, sizeof(buf));

        //LOG_DEBUG("Read %d bytes from stream", len);

        if(len < 1) {
            LOG_WARN("Read nothing from stream");
            return false;
        }

        BinParser bp(buf, static_cast<size_t>(len));
        return _parser.parse(bp, products);
    }
};