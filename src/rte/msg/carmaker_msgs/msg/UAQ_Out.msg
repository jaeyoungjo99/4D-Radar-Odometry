#
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization (optional)

float64 Car_Roll
float64 Car_Pitch
float64 Car_Yaw
float64 Car_vx
float64 Car_vy
float64 Car_vz
float64 Car_RollVel
float64 Car_PitchVel
float64 Car_YawVel
float64 Car_ax
float64 Car_ay
float64 Car_az
float64 Car_RollAcc
float64 Car_PitchAcc
float64 Car_YawAcc
float64 Steer_WhlAng
float64 VC_Gas
float64 VC_Brake
float64 VC_SelectorCtrl
float64 Sensor_Inertial_0_Vel_B_x
float64 Sensor_Inertial_0_Vel_B_y
float64 Sensor_Inertial_0_Vel_B_z
float64 Sensor_Inertial_0_Omega_B_x
float64 Sensor_Inertial_0_Omega_B_y
float64 Sensor_Inertial_0_Omega_B_z
float64 Sensor_Inertial_0_Acc_B_x
float64 Sensor_Inertial_0_Acc_B_y
float64 Sensor_Inertial_0_Acc_B_z
float64 Sensor_Inertial_0_Aplha_B_x
float64 Sensor_Inertial_0_Aplha_B_y
float64 Sensor_Inertial_0_Aplha_B_z
float64 Sensor_Inertial_1_Vel_B_x
float64 Sensor_Inertial_1_Vel_B_y
float64 Sensor_Inertial_1_Vel_B_z
float64 Sensor_Inertial_1_Omega_B_x
float64 Sensor_Inertial_1_Omega_B_y
float64 Sensor_Inertial_1_Omega_B_z
float64 Sensor_Inertial_1_Acc_B_x
float64 Sensor_Inertial_1_Acc_B_y
float64 Sensor_Inertial_1_Acc_B_z
float64 Sensor_Inertial_1_Aplha_B_x
float64 Sensor_Inertial_1_Aplha_B_y
float64 Sensor_Inertial_1_Aplha_B_z
float64 Vhcl_FL_rotv                           # rad/s
float64 Vhcl_FR_rotv                           # rad/s
float64 Vhcl_RL_rotv                           # rad/s
float64 Vhcl_RR_rotv                           # rad/s
float64 Vhcl_FL_rz                             # rad
float64 Vhcl_FR_rz                             # rad
float64 Vhcl_RL_rz                             # rad
float64 Vhcl_RR_rz                             # rad
