#!/usr/bin/env python



import rospy
import time
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class BaseControl:
    def __init__(self, baseId, device_port, baudrate, VtoRPM):
        # Serial Communication
        try:
            self.serial = serial.Serial(device_port, baudrate, timeout=10)
            rospy.loginfo("Flusing first 50 data reading ...")
            for x in range(0, 50):
                line = self.serial.readline()
                data_array = string.split(line,",")    # Fields split
		time.sleep(0.01)
	    rospy.loginfo("Communication success !")
            rospy.loginfo("Using TEB_local_planner's base control!!!!")
            #rospy.loginfo("VtoRPM:"+ str(VtoRPM))
        except serial.serialutil.SerialException:
            rospy.logerr("Can not receive data from the port: "+device_port + 
            ". Did you specify the correct port in the launch file?")
            self.serial.close
            sys.exit(0) 

	# ROS handler        
        self.sub = rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, self.cmdCB, queue_size=10) 
	self.sub1 = rospy.Subscriber('/car/cmd_vel', Twist, self.cmdCB1, queue_size=10)
	self.sub2 = rospy.Subscriber('/joy/cmd_vel', Twist, self.cmdCB2, queue_size=10)
	self.pub = rospy.Publisher('base_odom', Odometry, queue_size=10) 
	self.timer_odom = rospy.Timer(rospy.Duration(0.1), self.timerOdomCB) # 10Hz
        self.timer_cmd = rospy.Timer(rospy.Duration(0.1), self.timerCmdCB) # 10Hz
        self.baseId = baseId
	self.VtoRPM = VtoRPM
        # variable        
        self.trans_x = 0.0
        self.rotat_z = 0.0
        self.trans_x1 = 0.0
        self.rotat_z1 = 0.0
        self.trans_x2 = 0.0
        self.rotat_z2 = 0.0
        self.flag=0
        self.flag2=0
	self.count=0
	#self.last_throttle=1500
	#self.last_turn=90
	self.last_rotat_z=0
    def cmdCB(self, data):
        self.trans_x = data.drive.speed
	self.rotat_z = data.drive.steering_angle
    def cmdCB1(self, data):
        self.trans_x1 = data.linear.x
	self.rotat_z1 = data.angular.z
	self.flag = data.linear.z
    def cmdCB2(self, data):
        self.trans_x2 = data.linear.x
	self.rotat_z2 = data.angular.z
	self.flag2 = 1
    def timerOdomCB(self, event):
        # Serial read & publish 
        try:
		line = self.serial.readline()
        	msg = Odometry()
        	msg.header.stamp = rospy.Time.now()
        	msg.header.frame_id = 'odom'
        	msg.child_frame_id = 'base_footprint'
 		RPM = float(line)
		Vx=RPM/self.VtoRPM
        	msg.twist.twist.linear.x = Vx
		msg.twist.twist.linear.y = 0
        	for i in range(36):
           		msg.twist.covariance[i] = 0
        	msg.twist.covariance[0] = 0.05 #Vx Cov
		msg.twist.covariance[1] = 0.01 #Vx Cov
                if Vx<0:
			msg.twist.covariance[0] = 1 #Vx Cov
		elif Vx>0.8 and Vx<4:
			msg.twist.covariance[0] = 0.01 #Vx Cov
		self.pub.publish(msg)
	except: 
         #rospy.loginfo("Error in sensor value !") 
		pass 

    def timerCmdCB(self, event):
	
	if self.trans_x==0:
		self.rotat_z=self.last_rotat_z	


	if self.flag==1:
		self.trans_x=self.trans_x1
		self.rotat_z=self.rotat_z1
	elif self.flag2==1:
		self.trans_x=self.trans_x2
		self.rotat_z=self.rotat_z2
		self.flag2 = 0
	else:
        	self.trans_x1=0
		self.rotat_z1=0
        	self.trans_x2=0
		self.rotat_z2=0

        # send cmd to motor
	if self.trans_x>0 and self.trans_x<0.4:
		self.throttle = self.VtoRPM*0.4
		self.turn = self.rotat_z*180/3.1415926*1.2+90
		self.count=0
	elif self.trans_x>=0.4 and self.trans_x<2.5:
		self.throttle = self.trans_x*self.VtoRPM
		self.turn = self.rotat_z*180/3.1415926*1.2+90
		self.count=0
	elif self.trans_x>=2.5:
		self.throttle = 4167
		self.turn = self.rotat_z*180/3.1415926*1.2+90
		self.count=0
	elif self.trans_x<0 and self.trans_x>-1.5 :
		self.throttle = self.VtoRPM*-1.5
		self.turn = self.rotat_z*180/3.1415926*1.2+90
		self.count=0	
	elif self.trans_x<=-1.5 and self.trans_x>-2.5 :
		self.throttle = self.trans_x*self.VtoRPM
		self.turn = self.rotat_z*180/3.1415926*1.2+90
		self.count=0
	elif self.trans_x<=-2.5 :
		self.throttle = self.VtoRPM*-2.5
		self.turn = self.rotat_z*180/3.1415926*1.2+90
		self.count=0			
	else:	
		self.throttle =0
		self.turn = self.rotat_z*180/3.1415926*1.2+90

	if self.turn>135 :
		self.turn=135
	elif self.turn<45 :
		self.turn=45

 	self.last_rotat_z=self.rotat_z 
 
        values = [str(self.turn), str(self.throttle), '0']
        values[2] = str(len(values[0]) + len(values[1]))
	values1=[str(self.rotat_z1), str(self.trans_x1)]
        cmd = ",".join(values).encode()
	cmd1 = ",".join(values1).encode()
	#rospy.logerr(cmd)
	#rospy.logerr(cmd1)        
        self.serial.flushInput()
	self.serial.write(cmd)
if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('base_control', anonymous=True)

        # Get params
        baseId = rospy.get_param('~base_id', 'base_link') # base link
        device_port = rospy.get_param('~port', '/dev/uno') # device port
	VtoRPM = float( rospy.get_param('~VtoRPM', '1670') )# Parameter to transfer from V speed(m/s) to rpm
        baudrate = float( rospy.get_param('~baudrate', '57600') ) 
        # Constract BaseControl Obj
        rospy.loginfo("Start base control node ...")
        bc = BaseControl(baseId, device_port, baudrate, VtoRPM)
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
    print("shutting down")
