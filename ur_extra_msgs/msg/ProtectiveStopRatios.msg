# Data regarding how "close" the robot is to triggering particular protective stop related faults.
# The ratios are non-negative (unless unknown); a protective stop is triggered if the value of a
# ratio is >= 1.0.

Header header

float64 UNKNOWN_RATIO=-1.0  # Sentinel value for "unknown ratio".

# How close the robot is to triggering a C157 or C158 "collision detected" protective stop.
# Introduced in Polyscope 5.15.0; see:
# https://www.universal-robots.com/articles/ur/release-notes/release-note-software-version-515xx/
float64 collision_detection_ratio

# How close the robot is to triggering a C153 or C159 "joint position deviation" protective stop.
# No documentation on when this was introduced; assuming it has always been there.
float64 joint_position_deviation_ratio

# RTDE 'time_scale_source'
# Introduced in Polyscope 5.17.0
# See the RTDE guide for a breakdown of the values:
# https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
int32 time_scale_source
