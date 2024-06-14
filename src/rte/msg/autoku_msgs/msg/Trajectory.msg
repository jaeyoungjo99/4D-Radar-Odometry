# std_msgs/Header
std_msgs/Header header

uint8 id
uint8 behavior_zone
uint8 next_behavior_zone
uint8 decision_behavior
float64 behavior_distance
uint8 is_target

autoku_msgs/TrajectoryBoundary left_boundary
autoku_msgs/TrajectoryBoundary right_boundary
autoku_msgs/TrajectoryBoundary center_boundary
autoku_msgs/TrajectoryPoint[] point