# 4D-Radar-Odometry

## Evaluation in VOD with EVO

evo_rpe kitti 03_gt_poses.txt 03_estimate_poses.txt --delta_unit m -p --plot_mode=xy --pose_relation angle_deg
evo_rpe kitti 17_gt_poses.txt 17_estimate_poses.txt --delta_unit m --project_to_plane xy -p --plot_mode=xyz  --pose_relation angle_deg

evo_rpe kitti 03_gt_poses.txt 03_estimate_poses.txt --delta_unit m --project_to_plane xy -p --plot_mode=xyz  --pose_relation angle_deg
evo_rpe kitti 04_gt_poses.txt 04_estimate_poses.txt --delta_unit m --project_to_plane xy -p --plot_mode=xyz  --pose_relation angle_deg
evo_rpe kitti 09_gt_poses.txt 09_estimate_poses.txt --delta_unit m --project_to_plane xy -p --plot_mode=xyz  --pose_relation angle_deg
evo_rpe kitti 17_gt_poses.txt 17_estimate_poses.txt --delta_unit m --project_to_plane xy -p --plot_mode=xyz  --pose_relation angle_deg
evo_rpe kitti 19_gt_poses.txt 19_estimate_poses.txt --delta_unit m --project_to_plane xy -p --plot_mode=xyz  --pose_relation angle_deg
evo_rpe kitti 22_gt_poses.txt 22_estimate_poses.txt --delta_unit m --project_to_plane xy -p --plot_mode=xyz  --pose_relation angle_deg
evo_rpe kitti 24_gt_poses.txt 24_estimate_poses.txt --delta_unit m --project_to_plane xy -p --plot_mode=xyz  --pose_relation angle_deg

