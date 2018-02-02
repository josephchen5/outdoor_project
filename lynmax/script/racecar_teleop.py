#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control Your racecar!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

space key, k : force stop
a/d: shift the middle pos of steering by +/- 2 degree
CTRL-C to quit
"""
#w/x: shift the middle pos of throttle by +/- 5 pwm
moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
           }
moveBackBindings = {
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }
movefastBindings = {
        'r':(1,1),
        't':(1,0),
        'y':(1,-1),
           }
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('racecar_teleop')
    pub = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)
    flag=0
    status = 0
    count = 0
    step=0
    wait=3
    speed_mid = 0 
    speed_bias = 0
    turn_mid = 0  
    turn_bias = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(control_speed,control_turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                control_speed = moveBindings[key][0]*0.6 #speed(m/s)
                control_turn = moveBindings[key][1]*0.65 + turn_bias*0.017 #rad
		flag=1
                count = 0

	    elif key in moveBackBindings.keys():
                control_speed = moveBackBindings[key][0]*0.9 #speed(m/s)
                control_turn = moveBackBindings[key][1]*0.65 +turn_bias*0.017 #rad
		flag=1                
		count = 0
	    elif key in movefastBindings.keys():
                control_speed = movefastBindings[key][0]*1.4 #speed(m/s)
                control_turn = movefastBindings[key][1]*0.3 +turn_bias*0.017 #rad
		flag=1                
		count = 0
            elif  key == 'k' :
                control_speed = 0
                control_turn = turn_bias*0.017
		flag=1
		count = 0

            elif key == ' ' :
                control_speed = 0
                control_turn = turn_bias*0.017
		flag=1

            #elif key == 'w' :
            #    speed_bias = speed_bias - 5
            #    control_speed = 0
            #    print vels(control_speed,control_turn)
            #elif key == 'x' :
            #    speed_bias = speed_bias + 5
            #    control_speed = 0
            #    print vels(control_speed,control_turn)
            elif key == 'a' :
                turn_bias = turn_bias + (2*0.017)
                control_turn = control_turn + (2*0.017)
                print vels(control_turn/0.017)
            elif key == 'd' :
                turn_bias = turn_bias - (2*0.017)
                control_turn = control_turn - (2*0.017)
                print vels(control_turn/0.017)
            else:
                count = count + 1
                if count > 4:
                    control_speed = 0
                    control_turn = turn_bias*0.017
		    flag=1
		if count > 5:
		    control_speed = 0
		    flag=0 
                if (key == '\x03'):
                    break
            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = flag
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

    except:
        print "error"

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 1
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

