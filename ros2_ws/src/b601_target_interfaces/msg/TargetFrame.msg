# Unique identifier returned when selecting the source.
string session_id
# Start at 1, increment for every target in this session.
uint64 sequence
# Supply all seven configured joint names and positions. Velocities are optional.
sensor_msgs/JointState target
