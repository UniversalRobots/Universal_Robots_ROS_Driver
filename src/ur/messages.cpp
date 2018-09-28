#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/consumer.h"

bool VersionMessage::parseWith(BinParser& bp)
{
  bp.parse(project_name);
  bp.parse(major_version);
  bp.parse(minor_version);
  bp.parse(svn_version);
  bp.consume(sizeof(uint32_t));  // undocumented field??
  bp.parse_remainder(build_date);

  return true;  // not really possible to check dynamic size packets
}

bool VersionMessage::consumeWith(URMessagePacketConsumer& consumer)
{
  return consumer.consume(*this);
}