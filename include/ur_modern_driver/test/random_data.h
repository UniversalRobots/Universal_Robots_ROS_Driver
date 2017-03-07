#pragma once
#include <inttypes.h>
#include <algorithm>
#include <climits>
#include <cstddef>
#include <functional>
#include <random>
#include "ur_modern_driver/bin_parser.h"

class RandomDataTest
{
private:
  using random_bytes_engine = std::independent_bits_engine<std::default_random_engine, CHAR_BIT, uint8_t>;
  uint8_t* _buf;
  BinParser bp_;
  size_t n_;

public:
  RandomDataTest(size_t n) : _buf(new uint8_t[n]), bp_(_buf, n), n_(n)
  {
    random_bytes_engine rbe;
    std::generate(_buf, _buf + n, std::ref(rbe));
  }

  ~RandomDataTest()
  {
    delete _buf;
  }

  BinParser getParser(bool skip = false)
  {
    return BinParser(_buf, n_ - (skip ? sizeof(int32_t) : 0));
  }

  template <typename T>
  T getNext()
  {
    T actual;
    bp_.parse(actual);
    return actual;
  }
  void skip(size_t n)
  {
    bp_.consume(n);
  }
};