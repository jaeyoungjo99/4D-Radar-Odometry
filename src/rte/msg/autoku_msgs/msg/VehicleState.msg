# std_msgs/Header
std_msgs/Header header

# gnss
uint32 quality
uint32 fault_state

# position
float64 x
float64 y
float64 z

# motion
float64 vx
float64 vy
float64 vz

float64 ax
float64 ay
float64 az

float64 roll
float64 pitch
float64 yaw

float64 roll_vel
float64 pitch_vel
float64 yaw_vel

# road
float64 slope

# brake fade estimation
uint8 brake_state
float64 brake_temperature

# vehicle can
float64 lateral_accel
float64 longitudinal_accel
float64 yaw_rate

float64 steering_wheel_angle
float64 steering_tire_angle
float64 steering_speed
float64 steering_torque
uint8   steering_state

float64 wheel_vel_fl
float64 wheel_vel_fr
float64 wheel_vel_rl
float64 wheel_vel_rr

float64 motor_tq_f
float64 motor_tq_r
float64 motor_tq_total

float64 accel_position
float64 brake_pressure
uint8 brake_active
uint8 gear_select

uint8 operation_mode
uint8 lateral_autonomous_mode
uint8 longitudinal_autonomous_mode