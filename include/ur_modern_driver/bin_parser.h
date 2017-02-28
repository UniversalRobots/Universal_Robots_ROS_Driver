#pragma once

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/types.h"
#include <assert.h>
#include <cstddef>
#include <cstring>
#include <endian.h>
#include <inttypes.h>
#include <string>

class BinParser {
private:
    uint8_t *_buf_pos, *_buf_end;
    BinParser& _parent;

public:
    BinParser(uint8_t* buffer, size_t buf_len)
        : _buf_pos(buffer)
        , _buf_end(buffer + buf_len)
        , _parent(*this)
    {
        assert(_buf_pos <= _buf_end);
    }

    BinParser(BinParser& parent, size_t sub_len)
        : _buf_pos(parent._buf_pos)
        , _buf_end(parent._buf_pos + sub_len)
        , _parent(parent)
    {
        assert(_buf_pos <= _buf_end);
    }

    ~BinParser()
    {
        _parent._buf_pos = _buf_pos;
    }

    //Decode from network encoding (big endian) to host encoding
    template <typename T>
    T decode(T val)
    {
        return val;
    }
    uint16_t decode(uint16_t val)
    {
        return be16toh(val);
    }
    uint32_t decode(uint32_t val)
    {
        return be32toh(val);
    }
    uint64_t decode(uint64_t val)
    {
        return be64toh(val);
    }
    int16_t decode(int16_t val)
    {
        return be16toh(val);
    }
    int32_t decode(int32_t val)
    {
        return be32toh(val);
    }
    int64_t decode(int64_t val)
    {
        return be64toh(val);
    }

    template <typename T>
    T peek()
    {
        assert(_buf_pos <= _buf_end);
        T val;
        std::memcpy(&val, _buf_pos, sizeof(T));
        return decode(val);
    }

    template <typename T>
    void parse(T& val)
    {
        val = peek<T>();
        _buf_pos += sizeof(T);
    }

    void parse(double& val)
    {
        uint64_t inner;
        parse<uint64_t>(inner);
        std::memcpy(&val, &inner, sizeof(double));
    }
    void parse(float& val)
    {
        uint32_t inner;
        parse<uint32_t>(inner);
        std::memcpy(&val, &inner, sizeof(float));
    }

    // UR uses 1 byte for boolean values but sizeof(bool) is implementation
    // defined so we must ensure they're parsed as uint8_t on all compilers
    void parse(bool& val)
    {
        uint8_t inner;
        parse<uint8_t>(inner);
        val = inner != 0;
    }

    // Explicit parsing order of fields to avoid issues with struct layout
    void parse(double3_t& val)
    {
        parse(val.x);
        parse(val.y);
        parse(val.z);
    }

    // Explicit parsing order of fields to avoid issues with struct layout
    void parse(cartesian_coord_t& val)
    {
        parse(val.position);
        parse(val.rotation);
    }

    void parse_remainder(std::string& val)
    {
        parse(val, size_t(_buf_end - _buf_pos));
    }

    void parse(std::string& val, size_t len)
    {
        val.assign(reinterpret_cast<char*>(_buf_pos), len);
        _buf_pos += len;
    }

    // Special string parse function that assumes uint8_t len followed by chars
    void parse(std::string& val)
    {
        uint8_t len;
        parse(len);
        parse(val, size_t(len));
    }

    template <typename T, size_t N>
    void parse(T (&array)[N])
    {
        for (size_t i = 0; i < N; i++) {
            parse(array[i]);
        }
    }

    void consume()
    {
        _buf_pos = _buf_end;
    }
    void consume(size_t bytes)
    {
        _buf_pos += bytes;
    }

    bool check_size(size_t bytes)
    {
        return bytes <= size_t(_buf_end - _buf_pos);
    }
    template <typename T>
    bool check_size(void)
    {
        return check_size(T::SIZE);
    }

    bool empty()
    {
        return _buf_pos == _buf_end;
    }

    void debug()
    {
        LOG_DEBUG("BinParser: %p - %p (%zu bytes)", _buf_pos, _buf_end, _buf_end - _buf_pos);
    }
};
