#!/usr/bin/python

#import roslib; roslib.load_manifest('package_name')
import rospy
import tf
from nav_msgs.msg import Odometry
import time

def callback(rs_odom):
    
    global lx, ly, lz, ax, ay, az
    lx = rs_odom.twist.twist.linear.x
    ly = rs_odom.twist.twist.linear.y
    lz = rs_odom.twist.twist.linear.z
    
    ax = rs_odom.twist.twist.angular.x
    ay = rs_odom.twist.twist.angular.y
    az = rs_odom.twist.twist.angular.z
    



if __name__ == '__main__':
    print("starting node")
    rospy.init_node('tf_to_odom2')
    #listener = tf.TransformListener(1, rospy.Duration(10))
    listener = tf.TransformListener()
    odom_sub = rospy.Subscriber("rst265_camera/odom/sample", Odometry, callback)
    pose_pub = rospy.Publisher("test_odom", Odometry , queue_size=10)
    
    time.sleep(0.1) # allows listener buffer to load
    
    pose_msg = Odometry()
    
    pose_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
                                        
    pose_msg.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
                                
    pose_msg.header.frame_id = "ekf_odom"
    pose_msg.child_frame_id = "rs_base_link"


    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/rst265_camera_odom_frame', '/ghost_base_link', rospy.Time(0))

            
            pose_msg.pose.pose.position.x = trans[0]
            pose_msg.pose.pose.position.y = trans[1]
            pose_msg.pose.pose.position.z = trans[2]
            pose_msg.pose.pose.orientation.x = rot[0]
            pose_msg.pose.pose.orientation.y = rot[1]
            pose_msg.pose.pose.orientation.z = rot[2]
            pose_msg.pose.pose.orientation.w = rot[3]
                        
            pose_msg.twist.twist.linear.x = lx
            #pose_msg.twist.twist.linear.y = ly
            #pose_msg.twist.twist.linear.z = lz
            pose_msg.twist.twist.angular.x = ax
            pose_msg.twist.twist.angular.y = ay
            pose_msg.twist.twist.angular.z = az
            
            
            now  = rospy.get_rostime()
            pose_msg.header.stamp.secs = now.secs
            pose_msg.header.stamp.nsecs = now.nsecs
            
            
            pose_pub.publish(pose_msg)            
            
        except (tf.LookupException, tf.ConnectivityException):
            continue
    
        rate.sleep()