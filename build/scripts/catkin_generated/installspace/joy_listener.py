# !/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

class JoyInterpret(object):

    def callback(self, msg):
	# msg.axes[x]:	Left stick:  	L/R 0 (L= -1, R= +1)
	#    		Left stick:  	U/D 1 (U = +1, D = -1)
	#    		Left trigger  	    2 (Out = 1, In = -1)
	#    	  	Right stick: 	L/R 3 (L= -1, R= +1)
	#    	  	Right stick: 	U/D 4 (U = +1, D = -1)
	#    	  	Right trigger:	    5 (Out = 1, In = 0)
	#    	  	D-pad:	     	L/R 6 (L = -1, R = 1)
	#    	  	D-pad:	     	U/D 7 (U = -1, R = 1)
	#
	# msg.buttons[x]: 	X  0
	# 			C  1
	#			T  2
	#			S  3
	#			LB 4
	#			RB 5
	#			Sh 8
	#			Op 9
	#			PS 10
	#			L3 11
	#			R3 12

	# Convert msg into 1x8 axes list & 1x13 button list
        self.ax = [msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3], msg.axes[4], msg.axes[5], msg.axes[6], msg.axes[7]]
        self.but = [msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3], msg.buttons[4], msg.buttons[5], msg.buttons[8], msg.buttons[9], msg.buttons[10], msg.buttons[11], msg.buttons[12]]
	    
        rospy.loginfo("axes: {}".format(self.ax))
        rospy.loginfo("buttons: {}".format(self.but))


    def start(self):
        rospy.init_node('joy2iiwa', anonymous=True) # initialise node
        rospy.Subscriber("joy", Joy, self.callback, queue_size=1)
        rospy.spin()
 
    
if __name__ == '__main__':
    JoyInterpret().start()



