#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import copy
import threading
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

# Global variables
cur_odom_to_baselink = None  # Python2移除Optional类型注解
cur_map_to_odom = None
pub_localization = None

# Configuration parameters
FREQ_PUB_LOCALIZATION = 5.0


def xyz_to_mat44(position):  # Python2移除参数类型注解
    """Convert ROS Point to 4x4 translation matrix."""
    return tf.transformations.translation_matrix([position.x, position.y, position.z])


def xyzw_to_mat44(orientation):  # Python2移除参数类型注解
    """Convert ROS Quaternion to 4x4 rotation matrix."""
    return tf.transformations.quaternion_matrix([orientation.x, orientation.y, 
                                                  orientation.z, orientation.w])


def pose_to_mat(pose_msg):  # Python2移除参数/返回值类型注解
    """Convert ROS Odometry message to 4x4 transformation matrix.
    
    Constructs homogeneous transformation matrix from position and orientation.
    """
    # First convert position and orientation to 4x4 matrices
    trans_matrix = xyz_to_mat44(pose_msg.pose.pose.position)
    rot_matrix = xyzw_to_mat44(pose_msg.pose.pose.orientation)
    
    # Combine using matrix multiplication: T = translation * rotation
    return np.dot(trans_matrix, rot_matrix)


def inverse_se3(trans):  # Python2移除参数/返回值类型注解
    """Compute inverse of SE(3) transformation matrix."""
    trans_inverse = np.eye(4)
    # R^-1
    trans_inverse[:3, :3] = trans[:3, :3].T
    # -R^-1 * t
    trans_inverse[:3, 3] = -np.dot(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def transform_fusion():  # Python2移除返回值类型注解
    """Main transform fusion thread for publishing TF and localization."""
    global cur_odom_to_baselink, cur_map_to_odom, pub_localization, FREQ_PUB_LOCALIZATION
    
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(FREQ_PUB_LOCALIZATION)
    
    while not rospy.is_shutdown():
        try:
            # Create thread-safe copies
            cur_odom = copy.deepcopy(cur_odom_to_baselink) if cur_odom_to_baselink else None
            
            # Get map to odom transformation
            if cur_map_to_odom is not None:
                T_map_to_odom = pose_to_mat(cur_map_to_odom)
            else:
                T_map_to_odom = np.eye(4)
            
            # Publish TF transform: map -> camera_init
            translation = tf.transformations.translation_from_matrix(T_map_to_odom)
            rotation = tf.transformations.quaternion_from_matrix(T_map_to_odom)
            
            br.sendTransform(
                translation,
                rotation,
                rospy.Time.now(),
                'camera_init',  # child frame
                'map'           # parent frame
            )
            
            # Publish localization odometry if odometry is available
            if cur_odom is not None:
                localization = Odometry()
                
                # Get odom to base_link transformation
                T_odom_to_base_link = pose_to_mat(cur_odom)
                
                # Compute map to base_link transformation
                # Note: T_map_to_odom changes slowly, temporarily ignoring time synchronization
                T_map_to_base_link = np.dot(T_map_to_odom, T_odom_to_base_link)
                
                # Extract position and orientation
                xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
                quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
                
                # Fill localization message
                localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
                localization.twist = cur_odom.twist
                localization.header.stamp = cur_odom.header.stamp
                localization.header.frame_id = 'map'
                localization.child_frame_id = 'body'
                
                # Publish localization
                if pub_localization:
                    pub_localization.publish(localization)
                
                # Log throttled debug info (Python2兼容的字符串格式化)
                rospy.logdebug_throttle(1, 'Transform: %s' % T_map_to_base_link)
            
            rate.sleep()
            
        except Exception as e:
            rospy.logerr('Error in transform_fusion thread: %s' % e)  # Python2字符串格式化
            rate.sleep()


def cb_save_cur_odom(odom_msg):  # Python2移除参数类型注解
    """Callback to save current odometry (odom to base_link)."""
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):  # Python2移除参数类型注解
    """Callback to save map to odometry transformation."""
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


def main():  # Python2移除返回值类型注解
    """Main function."""
    global cur_odom_to_baselink, cur_map_to_odom, pub_localization, FREQ_PUB_LOCALIZATION
    
    # Initialize ROS node
    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Initialized...')
    
    # Subscribers
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)
    
    # Publisher
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)
    
    # Start transform fusion thread
    fusion_thread = threading.Thread(target=transform_fusion)
    fusion_thread.daemon = True  # Python2中daemon参数赋值方式兼容写法
    fusion_thread.start()
    
    rospy.loginfo('Transform fusion started...')
    
    # Main ROS spin
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Transform fusion node shutdown')
    except Exception as e:
        rospy.logerr('Unhandled exception in transform_fusion: %s' % e)  # Python2字符串格式化
