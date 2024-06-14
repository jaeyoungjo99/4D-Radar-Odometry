#!/usr/bin/env python3
#
# Copyright (c) 2023 AI Lab.
#
"""
tf_static node

"""

import rospy
import tf
import tf2_ros
import os
import rospkg
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from configparser import ConfigParser

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform

import math
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

class TransformFrameStatic(object):
    """
    Handles tf frames in autoku

    """

    def __init__(self):
        # ROS node
        rospy.init_node('tf_static', anonymous=True)

        self.ini_file = os.environ['PWD'] + "/config/calibration.ini"
        # Parameters
        self._period = rospy.get_param('/task_period/period_tf_static_frame')
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.frame_list = []
        
        # Publisher
        
        # Subscriber      

    def destroy(self):
        """
        Destroy all objects
        """
    
    def add_frame(self, type, child, parent, xyz, q, time):
        """
        Add frame into TransformFrameStatic.tf_list
        @child  : child frame
        @parent : parent frame
        @xyz    : translations in x, y, and z direction [m]
        @rpy    : rotations in x, y, and z direction [rad]
        @time   : transform time
        """
        transform = Transform()
        transform.translation.x = xyz[0]
        transform.translation.y = xyz[1]
        transform.translation.z = xyz[2]
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        frame = {"type": type,
                 "child": child, 
                 "parent": parent, 
                 "transform": transform,
                 "time": time}
        self.frame_list.append(frame)

    def update_frame(self):
        """
        Update frame function
        """
        update_time = rospy.Time.now()
        if self.frame_list is not None:
            for frame in self.frame_list:
                if frame["type"] == 'static':
                    frame["time"] = update_time

    def add_tf_from_config(self, ini_file, section, update_time):
        config = ConfigParser()
        config.read(ini_file)

        parent_frame_id = str(config.get(section, 'parent_frame_id'))
        child_frame_id = str(config.get(section, 'child_frame_id'))
        transform_xyz_m = list(map(float, config.get(section, 'transform_xyz_m').split()))
        rotation_rpy_deg = list(map(float, config.get(section, 'rotation_rpy_deg').split()))

        rotation_quat = quaternion_from_euler(math.radians(rotation_rpy_deg[0]) , math.radians(rotation_rpy_deg[1]),
                                                 math.radians(rotation_rpy_deg[2]))

        self.add_frame('static', child_frame_id, parent_frame_id,
            transform_xyz_m, 
            rotation_quat, update_time)

    def run(self):
        """
        main loop
        """
        rate = rospy.Rate(1 / self._period)

        update_time = rospy.Time.now()

        print(self.ini_file)

        self.add_tf_from_config(self.ini_file,'Ego To Main LiDAR',update_time)
        self.add_tf_from_config(self.ini_file,'Ego To Main Radar',update_time)
        
        while not rospy.is_shutdown():
            self.update_frame()
            for frame in self.frame_list:
                if frame["type"] == "static":
                    static_transform = TransformStamped()
                    static_transform.header.stamp = frame["time"]
                    static_transform.header.frame_id = frame["parent"]
                    static_transform.child_frame_id = frame["child"]
                    static_transform.transform = frame["transform"]
                    self._static_broadcaster.sendTransform(static_transform)
            rate.sleep()
        
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    tf_static = TransformFrameStatic()
    try:
        tf_static.run()
    finally:
        if tf_static is not None:
            tf_static.destroy()


if __name__ == '__main__':
    main()