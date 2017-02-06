#pragma once

class MessageBase {
public:
    virtual bool parse_with(BinParser &bp) = 0;

    uint64_t timestamp;
    uint8_t source;
};

class VersionMessage : public MessageBase {
public:
    bool parse_with(BinParser &bp);

    std::string project_name;
    uint8_t major_version;
    uint8_t minor_version;
    int32_t svn_version;
    std::string build_date;
}