#!/usr/bin/python

#import roslib; roslib.load_manifest('package_name')
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == '__main__':
    rospy.init_node('tf_to_pose')
    listener = tf.TransformListener()
    pose_pub = rospy.Publisher("rs_t265_pose", PoseWithCovarianceStamped, queue_size=10)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/rst265_camera_odom_frame', '/ghost_base_link', rospy.Time(0))
            #print trans, rot
            pose_msg = PoseWithCovarianceStamped()
            
            pose_msg.header.frame_id = "ghost_home_link"    
            now  = rospy.get_rostime()
            pose_msg.header.stamp.secs = now.secs
            pose_msg.header.stamp.nsecs = now.nsecs
            
            pose_msg.pose.pose.position.x = trans[0]
            pose_msg.pose.pose.position.y = trans[1]
            pose_msg.pose.pose.position.z = trans[2]
            pose_msg.pose.pose.orientation.x = rot[0]
            pose_msg.pose.pose.orientation.y = rot[1]
            pose_msg.pose.pose.orientation.z = rot[2]
            pose_msg.pose.pose.orientation.w = rot[3]

#            pose_msg.pose.covariance =  [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                         0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
#                                         0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
#                                         0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 
#                                         0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 
#                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
            
            
            pose_pub.publish(pose_msg)            
            
        except (tf.LookupException, tf.ConnectivityException):
            continue
    
        rate.sleep()