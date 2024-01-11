#!/usr/bin/env python3

import tf
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import numpy as np

def imu_callback(imu_msg):
    ps_msg = PoseStamped()
    ps_msg.header=imu_msg.header
    ps_msg.pose.orientation=imu_msg.orientation
    posestamped_pub.publish(ps_msg)
    x = ps_msg.pose.orientation.x
    y = ps_msg.pose.orientation.y
    z = ps_msg.pose.orientation.z
    w = ps_msg.pose.orientation.w
    yaw = quaternion_to_euler(Quaternion(x, y, z, w)).z
    #print(yaw)
    #print('x:', ps_msg.pose.orientation.x)
    #print('y:', ps_msg.pose.orientation.y)
    print('z:', np.degrees(yaw))
    
def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])
    
if __name__ == "__main__":
    rospy.init_node("imu_to_pose")
    #Publisher
    posestamped_pub = rospy.Publisher("imu_pose", PoseStamped, queue_size=10)
    #Subscriber
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.spin()

