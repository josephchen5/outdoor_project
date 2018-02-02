#!/usr/bin/env python



import rospy
import time
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

class Desktop:
    def __init__(self):
	# ROS handler        
        self.sub = rospy.Subscriber('base_odom', Odometry, self.cmdCB, queue_size=1) 
	self.pub = rospy.Publisher('/Desktop/base_odom', Odometry, queue_size=1) 
        self.sub1 = rospy.Subscriber('/imu', Imu, self.cmdCB1, queue_size=1) 
	self.pub1 = rospy.Publisher('/Desktop/imu', Imu, queue_size=1) 
        self.sub2 = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.cmdCB2, queue_size=1) 
	self.pub2 = rospy.Publisher('/Desktop/ublox_gps/fix', NavSatFix, queue_size=1) 

    def cmdCB(self, data):
        # Serial read & publish 
        try:
        	msg = Odometry()
		msg.header.seq = data.header.seq
		msg.header.stamp = data.header.stamp
        	msg.header.frame_id = 'odom'
        	msg.child_frame_id = 'base_footprint'
        	msg.twist.twist = data.twist.twist
                for i in range(36):
        		msg.twist.covariance[i] = data.twist.covariance[i]*0.1
		#rospy.logerr(msg)
		self.pub.publish(msg)
	except:  
		pass 

    def cmdCB1(self, data):
        # Serial read & publish 
        try:
        	msg = Imu()
		msg.header.seq =data.header.seq
		msg.header.stamp =  data.header.stamp
                msg.header.frame_id = data.header.frame_id
                msg.orientation= data.orientation
		msg.angular_velocity= data.angular_velocity
		msg.linear_acceleration= data.linear_acceleration
                for i in range(9):
			msg.orientation_covariance[i]= data.orientation_covariance[i]*1
			msg.angular_velocity_covariance[i]= data.angular_velocity_covariance[i]*1
			msg.linear_acceleration_covariance[i]= data.linear_acceleration_covariance[i]*1
                #rospy.logerr(msg)
		self.pub1.publish(msg)
	except:  
		pass 


    def cmdCB2(self, data):
        # Serial read & publish 
        try:
        	msg = NavSatFix()
		msg.header.seq =data.header.seq
		msg.header.stamp =  data.header.stamp
                msg.header.frame_id = data.header.frame_id
                msg.status =data.status
		msg.latitude = data.latitude
		msg.longitude = data.longitude
		msg.altitude = data.altitude
		for i in range(9):
			msg.position_covariance[i]= data.position_covariance[i]*10
		msg.position_covariance_type=data.position_covariance_type
		#rospy.logerr(msg)
		self.pub2.publish(msg)
	except:  
		pass 



if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('desktop', anonymous=True)

 
        bc = Desktop()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
    print("shutting down")
