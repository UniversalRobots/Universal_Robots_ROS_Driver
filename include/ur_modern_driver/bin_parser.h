#pragma once

#include <inttypes.h>
#include <cstddef>
#include <cstring>
#include <string>
#include "ur_modern_driver/types.h"

class BinParser {
private:
    uint8_t *_buf_pos, *_buf_end;
    BinParser &_parent;

public:
    BinParser(uint8_t *buffer, size_t buf_len) :
        _buf_pos(buffer),
        _buf_end(buffer+buf_len),
        _parent(*this)
        { }
    
    BinParser(BinParser &parent, size_t sub_len) :
        _buf_pos(parent._buf_pos),
        _buf_end(parent._buf_pos+sub_len),
        _parent(parent) 
        { }

    ~BinParser() {
        _parent._buf_pos = _buf_pos;
    }    

    template<typename T>
    T peek() {
        return *(reinterpret_cast<T*>(_buf_pos));
    }

    template<typename T>
    void parse(T &val) {
        val = peek<T>();
        _buf_pos += sizeof(T);
    }

    // UR uses 1 byte for boolean values but sizeof(bool) is implementation 
    // defined so we must ensure they're parsed as uint8_t on all compilers
    void parse(bool &val) {
        uint8_t inner;
        parse<uint8_t>(inner);
        val = inner != 0;        
    }

    // Explicit parsing order of fields to avoid issues with struct layout
    void parse(double3_t &val) {
        parse(val.x);
        parse(val.y);
        parse(val.z);
    }

    // Explicit parsing order of fields to avoid issues with struct layout
    void parse(cartesian_coord_t &val) {
        parse(val.position);
        parse(val.rotation);
    }

    void parse(std::string &val, size_t len) {
        val = val.assign(reinterpret_cast<char*>(_buf_pos), len);
        _buf_pos += len;
    }

    // Special string parse function that assumes uint8_t len followed by chars 
    void parse(std::string &val) {
        uint8_t len;
        parse(len);
        parse(val, size_t(len));
    }

    template<typename T, size_t N>
    void parse(T (&array)[N]) {
        std::memcpy(array, _buf_pos, sizeof(T)*N);
        _buf_pos += (sizeof(T)*N);
    }

    void skip(size_t bytes) {
        _buf_pos += bytes;
    }

    bool check_size(size_t bytes) {
        return bytes <= size_t(_buf_end - _buf_pos);
    }
    template<typename T>
    bool check_size(void) {
        return check_size(T::SIZE);
    }
};
