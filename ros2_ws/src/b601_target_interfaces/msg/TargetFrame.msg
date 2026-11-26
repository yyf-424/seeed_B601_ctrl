# Unique identifier returned when selecting the source.
string session_id
# Start at 1, increment for every target in this session.
uint64 sequence
# Supply all seven configured joint names and positions. Velocities are optional.
# Optional effort is the MIT task feedforward, in N*m for the arm and N for finger_left.
# An empty effort array means zero task feedforward for all joints.
sensor_msgs/JointState target
