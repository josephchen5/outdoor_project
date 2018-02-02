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
        self.sub2 = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.cmdCB2, queue_size=1) 
	self.pub2 = rospy.Publisher('/GPS/ublox_gps/fix', NavSatFix, queue_size=1) 

    

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
			msg.position_covariance[i]= data.position_covariance[i]*100
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
