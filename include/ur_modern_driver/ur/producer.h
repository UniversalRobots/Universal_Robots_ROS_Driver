#pragma once
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/stream.h"

template <typename T>
class URProducer : public IProducer<T>
{
private:
  URStream& stream_;
  URParser<T>& parser_;

public:
  URProducer(URStream& stream, URParser<T>& parser) : stream_(stream), parser_(parser)
  {
  }

  void setupProducer()
  {
    stream_.connect();
  }
  void teardownProducer()
  {
    stream_.disconnect();
  }
  void stopProducer()
  {
    stream_.disconnect();
  }

  bool tryGet(std::vector<unique_ptr<T>>& products)
  {
    // 4KB should be enough to hold any packet received from UR
    uint8_t buf[4096];

    // blocking call
    ssize_t len = stream_.receive(buf, sizeof(buf));

    // LOG_DEBUG("Read %d bytes from stream", len);

    if (len == 0)
    {
      LOG_WARN("Read nothing from stream");
      return false;
    }
    else if (len < 0)
    {
      LOG_WARN("Stream closed");
      return false;
    }

    BinParser bp(buf, static_cast<size_t>(len));
    return parser_.parse(bp, products);
  }
};