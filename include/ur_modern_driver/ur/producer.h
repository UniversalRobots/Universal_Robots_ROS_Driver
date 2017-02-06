#pragma once
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/stream.h"
#include "ur_modern_driver/packet.h"
#include "ur_modern_driver/parser.h"

class URProducer : public IProducer<Packet> {
private:
    URStream &_stream;
    Parser &_parser;

public:
    URProducer(URStream &stream, Parser &parser) 
        : _stream(stream), 
        _parser(parser) { }
    
    void setup_producer();
    void teardown_producer();
    void stop_producer();
    std::unique_ptr<Packet> try_get();
};