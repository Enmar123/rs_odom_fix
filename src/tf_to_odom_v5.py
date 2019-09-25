#!/usr/bin/python

#import roslib; roslib.load_manifest('package_name')
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
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

def pub_static_tf():
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf_0 = TransformStamped()
    
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



if __name__ == '__main__':
    print("starting node")
    rospy.init_node('rs_odom_corrector')

    pub_static_tf()
    
    listener = tf.TransformListener()
    odom_sub = rospy.Subscriber("rst265_camera/odom/sample", Odometry, callback)
    pose_pub = rospy.Publisher("rs_t265_odom", Odometry , queue_size=10)
        
    odom_msg = Odometry()
    
    odom_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
                                        
    odom_msg.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
                                
    odom_msg.header.frame_id = "ekf_odom"
    odom_msg.child_frame_id = "rs_base_link"

    time.sleep(0.1) # allows listener buffer to load
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/rst265_camera_odom_frame', '/fake_base_link', rospy.Time(0))

            
            odom_msg.pose.pose.position.x = trans[0]
            odom_msg.pose.pose.position.y = trans[1]
            odom_msg.pose.pose.position.z = trans[2]
            odom_msg.pose.pose.orientation.x = rot[0]
            odom_msg.pose.pose.orientation.y = rot[1]
            odom_msg.pose.pose.orientation.z = rot[2]
            odom_msg.pose.pose.orientation.w = rot[3]
                        
            odom_msg.twist.twist.linear.x = lx
            #odom_msg.twist.twist.linear.y = ly
            #odom_msg.twist.twist.linear.z = lz
            odom_msg.twist.twist.angular.x = ax
            odom_msg.twist.twist.angular.y = ay
            odom_msg.twist.twist.angular.z = az
            
            
            now  = rospy.get_rostime()
            odom_msg.header.stamp.secs = now.secs
            odom_msg.header.stamp.nsecs = now.nsecs
            
            
            pose_pub.publish(odom_msg)            
            
        except (tf.LookupException, tf.ConnectivityException):
            continue
    
        rate.sleep()