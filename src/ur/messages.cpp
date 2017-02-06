#include "ur_modern_driver/ur/messages.h"

bool MessageBase::parse_with(BinParser &bp) {
    bp.parse(timestamp);
    bp.parse(source);

    return true; //not really possible to check dynamic size packets
}


bool VersionMessage::parse_with(BinParser &bp) {
    
    bp.parse(project_name);
    bp.parse(major_version);
    bp.parse(minor_version);
    bp.parse(svn_version); //net to host?

    // how to parse this without length??
    //bp.parse(build_date);

    return true; //not really possible to check dynamic size packets    
}