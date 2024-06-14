std_msgs/Float64  timestamp 
std_msgs/Int32    device_id 

# Field Of LidarFrameMsg 
std_msgs/Bool           has_lidarframe
LidarFrameMsg           lidarframe 

# Field Of PointCloud 
std_msgs/Bool           has_pointcloud 
Point4f[]               scan_pointcloud 

# Field Of Pose 
std_msgs/Bool           has_pose
Pose                    global_pose 
