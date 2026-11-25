# Unique identifier returned when selecting the source.
string session_id
# Start at 1, increment for every target in this session.
uint64 sequence
# position is required. velocity is optional and in joint-space units.
sensor_msgs/JointState target
