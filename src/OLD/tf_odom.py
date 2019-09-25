#!/usr/bin/python

#import roslib; roslib.load_manifest('package_name')
import rospy
import tf
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('tf_to_odom')
    listener = tf.TransformListener()
    pose_pub = rospy.Publisher("rs_t265_base_odom", Odometry , queue_size=10)

    rate = rospy.Rate(60.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/rst265_camera_odom_frame', '/ghost_base_link', rospy.Time(0))
            #print trans, rot
            pose_msg = Odometry()
            
            pose_msg.header.frame_id = "ekf_odom"    
            now  = rospy.get_rostime()
            pose_msg.header.stamp.secs = now.secs
            pose_msg.header.stamp.nsecs = now.nsecs
            
            pose_msg.child_frame_id = "rs_base_link"
            
            pose_msg.pose.pose.position.x = trans[0]
            pose_msg.pose.pose.position.y = trans[1]
            pose_msg.pose.pose.position.z = trans[2]
            pose_msg.pose.pose.orientation.x = rot[0]
            pose_msg.pose.pose.orientation.y = rot[1]
            pose_msg.pose.pose.orientation.z = rot[2]
            pose_msg.pose.pose.orientation.w = rot[3]

            pose_msg.pose.covariance =  [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
                                         0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
                                         0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 
                                         0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
                                         
#            pose_msg.pose.covariance =  [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                         0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
#                                         0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
#                                         0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 
#                                         0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 
#                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            
            
            pose_pub.publish(pose_msg)            
            
        except (tf.LookupException, tf.ConnectivityException):
            continue
    
        rate.sleep()