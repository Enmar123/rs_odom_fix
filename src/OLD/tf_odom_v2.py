#!/usr/bin/python

#import roslib; roslib.load_manifest('package_name')
import rospy
import tf
from nav_msgs.msg import Odometry
import time

def callback(rs_odom):
    vx = rs_odom.twist.twist.linear.x
    vy = rs_odom.twist.twist.linear.y
    vz = rs_odom.twist.twist.linear.z
    



if __name__ == '__main__':
    print("starting node")
    rospy.init_node('tf_to_odom2')
    #listener = tf.TransformListener(1, rospy.Duration(10))
    listener = tf.TransformListener()
    #odom_sub = rospy.Subscriber("rst265_camera/odom/sample", Odometry, callback)
    pose_pub = rospy.Publisher("test_odom", Odometry , queue_size=10)
    
    time.sleep(0.1) # allows listener buffer to load

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/rst265_camera_odom_frame', '/ghost_base_link', rospy.Time(0))
            #print(trans)
            

            (lin, ang) = listener.lookupTwist('/ghost_base_link', '/rst265_camera_odom_frame', rospy.Time(0), rospy.Duration(0.03))        
            
            #print(twist)

            #rospy.loginfo("umm...test?")
                        
            #
            pose_msg = Odometry()
            
            pose_msg.header.frame_id = "ekf_odom"    
            now  = rospy.get_rostime()
            pose_msg.header.stamp.secs = now.secs
            pose_msg.header.stamp.nsecs = now.nsecs
            
            pose_msg.child_frame_id = "base_link"
            
#            pose_msg.pose.pose.position.x = trans[0]
#            pose_msg.pose.pose.position.y = trans[1]
#            pose_msg.pose.pose.position.z = trans[2]
#            pose_msg.pose.pose.orientation.x = rot[0]
#            pose_msg.pose.pose.orientation.y = rot[1]
#            pose_msg.pose.pose.orientation.z = rot[2]
#            pose_msg.pose.pose.orientation.w = rot[3]
                        
            pose_msg.twist.twist.linear.x = lin[0]
            pose_msg.twist.twist.linear.y = lin[1]
            pose_msg.twist.twist.linear.z = lin[2]
            pose_msg.twist.twist.angular.x = lin[0]
            pose_msg.twist.twist.angular.y = lin[1]
            pose_msg.twist.twist.angular.z = lin[2]
            

#            pose_msg.pose.covariance =  [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                         0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
#                                         0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
#                                         0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 
#                                         0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 
#                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
#                                         
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