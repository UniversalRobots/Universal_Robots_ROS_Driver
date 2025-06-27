# Similar to sensor_msgs/JointState, but with more fields

# Metadata
# `stamp` corresponds to the time when the state was recorded.
Header header
# NB: The order in all other lists follows the order of `names`.
string[] names

# For target / actual positions, target / actual actual_velocities,
# and target accelerations, refer to the state output message
# of the controller, which is published with the same header stamp.

# Current data [A]
# RTDE 'target_current'
float64[] target_currents
# RTDE 'actual_current'
float64[] actual_currents
# RTDE 'actual_current_window'
float64[] actual_current_windows

# Voltage data [V]
# RTDE 'actual_joint_voltage' (no target voltage)
float64[] actual_voltages

# Torque data [Nm]
# RTDE 'target_moment' (only target available)
float64[] target_torques

# Contextual data
# RTDE 'joint_mode'
int32[] joint_control_modes
# RTDE 'joint_control_output' (typically the same as 'actual_current')
float64[] joint_control_outputs
