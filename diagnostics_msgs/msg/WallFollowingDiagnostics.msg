std_msgs/Header header

# PID Gains
float32 kp
float32 ki
float32 kd

# PID Contribution
float32 p
float32 i
float32 d

# Error
float32 error
float32 target
float32 dist
bool follow_right

# Scan
float32 a
float32 b
float32 a_angle
float32 b_angle
float32 theta
float32 alpha


# Command
float32 angle_command # rad