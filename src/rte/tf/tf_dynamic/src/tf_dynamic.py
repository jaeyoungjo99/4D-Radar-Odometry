#!/usr/bin/env python3
#
# Copyright (c) 2023 AI Lab.
#
"""
tf_dynamic node

"""

import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from autoku_msgs.msg import VehicleState
import math

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

class TransformFrameDynamic(object):
    """
    Handles tf frames in autoku

    """

    def __init__(self):
        # ROS node
        rospy.init_node('tf_dynamic', anonymous=True)
        
        # Parameters
        self._period = rospy.get_param('/task_period/period_tf_frame')
        self._tf_broadcaster = tf.TransformBroadcaster()
        self.frame_list = []
        self.vehicle_state = VehicleState()
        self.radar_state = Odometry()
        self.dae = self.visualize_dae('package://tf_dynamic/dae/Ioniq5.stl', 'ego_frame','ego_vehicle', (1.5,0.0,0.0), (0.0,0.0,180.0))
        
        # Publisher
        self.dae_publisher = rospy.Publisher(
            "hmi/dae_ego_vehicle", Marker, queue_size=1)
        
        # Subscriber
        self.vehicle_state_subscriber = rospy.Subscriber(
            "app/loc/vehicle_state", VehicleState, self.on_vehicle_state)        
        
        self.radar_odom_subscriber = rospy.Subscriber(
            "radar_odom", Odometry, self.on_radar_odom)        
        
    def destroy(self):
        """
        Destroy all objects
        """
        self.vehicle_state_subscriber.unregister()

    def on_vehicle_state(self, data):
        """
        Callback on vehicle state event from
        vehicle_state_driver
        """
        self.vehicle_state = data

    def on_radar_odom(self, data):
        """
        Callback on radar odom event from
    
        """
        self.radar_state = data

    def visualize_dae(self, object_name, frame_id, name_space, xyz, rpy):
        """
        Visualize dae object
        """
        marker = Marker()
        
        marker.header.frame_id = frame_id
        marker.header.stamp = self.vehicle_state.header.stamp
        marker.ns = name_space
        marker.id = 0
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = object_name
        marker.mesh_use_embedded_materials = True
        marker.action = marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        orientation = quaternion_from_euler(rpy[0]/RAD2DEG, rpy[1]/RAD2DEG, rpy[2]/RAD2DEG)
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.5
        marker.color.r = 0.32156862745
        marker.color.g = 0.43137254902
        marker.color.b = 0.21568627451
        
        return marker
    
    def calculate_transform_from_ego_to_world(self):
        """
        Calculate position difference between 
        /ego_frame and /world in Cartesian coordinate
        """
        transform = Transform()
        transform.translation.x = self.vehicle_state.x
        transform.translation.y = self.vehicle_state.y
        transform.translation.z = 0.0
        # q = quaternion_from_euler(self.vehicle_state.roll, self.vehicle_state.pitch, (self.vehicle_state.yaw))
        q = quaternion_from_euler(0.0, 0.0, (self.vehicle_state.yaw))
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        
        return transform
    
    def calculate_transform_from_radar_to_world(self):
        """
        Calculate position difference between 
        /ego_frame and /world in Cartesian coordinate
        """
        transform = Transform()
        transform.translation.x = self.radar_state.pose.pose.position.x
        transform.translation.y = self.radar_state.pose.pose.position.y
        transform.translation.z = self.radar_state.pose.pose.position.z
        # q = quaternion_from_euler(self.vehicle_state.roll, self.vehicle_state.pitch, (self.vehicle_state.yaw))
        # q = quaternion_from_euler(0.0, 0.0, (self.radar_state.pose.pose.position))

        transform.rotation = self.radar_state.pose.pose.orientation
        # transform.rotation.x = q[0]
        # transform.rotation.y = q[1]
        # transform.rotation.z = q[2]
        # transform.rotation.w = q[3]
        
        return transform

    def add_frame(self, type, child, parent, xyz, q, time):
        """
        Add frame into TransformFrameDynamic.tf_list
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
        update_time = self.vehicle_state.header.stamp
        if self.frame_list is not None:
            for frame in self.frame_list:
                if frame["type"] != 'static':
                    if frame["child"] == 'ego_frame':
                        frame["transform"] = self.calculate_transform_from_ego_to_world()
                    if frame["child"] == 'afi910':
                        frame["transform"] = self.calculate_transform_from_radar_to_world()
                    frame["time"] = update_time

    def run(self):
        """
        main loop
        """
        rate = rospy.Rate(1 / self._period)

        # update_time = self.vehicle_state.header.stamp

        # transform = self.calculate_transform_from_ego_to_world()
        # self.add_frame('dynamic', 'ego_frame', 'world',
        #     [transform.translation.x, transform.translation.y, transform.translation.z], 
        #     [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w], update_time)

        update_time = self.radar_state.header.stamp

        transform = self.calculate_transform_from_radar_to_world()
        self.add_frame('dynamic', 'afi910', 'world',
            [transform.translation.x, transform.translation.y, transform.translation.z], 
            [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w], update_time)


        while not rospy.is_shutdown():
            self.update_frame()
            for frame in self.frame_list:
                if frame["type"] != "static":
                    self._tf_broadcaster.sendTransform(
                        [frame["transform"].translation.x, 
                        frame["transform"].translation.y, 
                        frame["transform"].translation.z],
                        [frame["transform"].rotation.x,
                        frame["transform"].rotation.y,
                        frame["transform"].rotation.z,
                        frame["transform"].rotation.w],
                        frame["time"],
                        frame["child"],
                        frame["parent"])
            self.dae.header.stamp = self.vehicle_state.header.stamp
            self.dae_publisher.publish(self.dae)
            rate.sleep()
        
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    tf_dynamic = TransformFrameDynamic()
    try:
        tf_dynamic.run()
    finally:
        if tf_dynamic is not None:
            tf_dynamic.destroy()


if __name__ == '__main__':
    main()