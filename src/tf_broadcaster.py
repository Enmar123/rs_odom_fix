#!/usr/bin/env python

import rospy

import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

#import numpy as np

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf_0 = TransformStamped()
    
    while not rospy.is_shutdown():
        try:
    
            # Creating transform from "camera_pose_frame" to "base_link"
            static_tf_0.header.stamp = rospy.Time.now()
            static_tf_0.header.frame_id = "rst265_camera_pose_frame"
            static_tf_0.child_frame_id = "fake_base_link"
        
            static_tf_0.transform.translation.x = float('-0.345')
            static_tf_0.transform.translation.y = float('-0.000')
            static_tf_0.transform.translation.z = float('-0.139')
        
            quat = tf.transformations.quaternion_from_euler(float('0.00'),float('0.00'),float('0.00'))
            static_tf_0.transform.rotation.x = quat[0]
            static_tf_0.transform.rotation.y = quat[1]
            static_tf_0.transform.rotation.z = quat[2]
            static_tf_0.transform.rotation.w = quat[3]
            broadcaster.sendTransform([static_tf_0])
        
        except rospy.ROSInterruptException:
            pass
        
        rospy.spin()
