#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
#from nrp_mouse_v2.msg import nrpmouse_msg
from adafruit_servokit import ServoKit


kit = ServoKit(channels=16)

#servo specifics
kit.servo[0].set_pulse_width_range(700, 2300)
kit.servo[1].set_pulse_width_range(700, 2300)
kit.servo[2].set_pulse_width_range(700, 2300)
kit.servo[3].set_pulse_width_range(700, 2300)
kit.servo[4].set_pulse_width_range(700, 2300)
kit.servo[5].set_pulse_width_range(700, 2300)
#kit.servo[6].set_pulse_width_range(700, 2300)
kit.servo[7].set_pulse_width_range(700, 2300)
kit.servo[8].set_pulse_width_range(700, 2300)
kit.servo[9].set_pulse_width_range(700, 2300)
kit.servo[10].set_pulse_width_range(700, 2300)
kit.servo[11].set_pulse_width_range(700, 2300)
kit.servo[12].set_pulse_width_range(700, 2300)
kit.servo[13].set_pulse_width_range(700, 2300)

#servo range adaption
#kit.servo[0].actuation_range = 180


#numbering
#1 fla1	(Foreleft Shoulder servo)    	SB: 8
#2 fla2 (Foreleft Elbow servo)  	SB: 9
#3 fra1	(Foreright Shoulder servo)   	SB: 10
#4 fra2	(Foreright Elbow servo)  	SB: 11
#5 hla1	(Hindleft hip servo)    	SB: 2
#6 hla2 (Hindleft knee servo)   	SB: 3
#7 hra1	(Hindright hip servo)   	SB: 0
#8 hra2	(Hindright knee servo)  	SB: 1
#9 spine                           	SB: 4
#10 tail                            	SB: 5
#Spine2   (up/down)              	SB: 7
#Head    (left/right)            	SB: 12
#Head    (up/down)               	SB: 13

def clamp(n):
	n = abs(n)

	if n < 0:
		return 0
	elif n > 180:
		return 180
	else:
		return n

def messageCb(msgarr):
	kit.servo[0].angle = clamp(msgarr.data[5]) #hra1
	kit.servo[1].angle = clamp(msgarr.data[6]) #hra2
	kit.servo[2].angle = clamp(msgarr.data[7]) #hla1
	kit.servo[3].angle = clamp(msgarr.data[8]+160) #hla2
	kit.servo[4].angle = clamp(msgarr.data[9]) #spine
	kit.servo[5].angle = clamp(msgarr.data[10]) #tail
	kit.servo[8].angle = clamp(msgarr.data[3]) #fla1
	kit.servo[9].angle = clamp(msgarr.data[4]) #fla2
	kit.servo[10].angle = clamp(msgarr.data[1]) #fra1
	kit.servo[11].angle = clamp(msgarr.data[2]+90) #fra2
	kit.servo[7].angle = clamp(msgarr.data[11]) #spine2
	kit.servo[12].angle = clamp(msgarr.data[12]) #head1
	kit.servo[13].angle = clamp(msgarr.data[13]) #head2
#	kit.servo[11].angle = (180-clamp(msgarr.data[4])) #fra2

def listener():
# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
	rospy.init_node('listener', anonymous=False)
	rospy.Subscriber("nrpmouse_servotopic", Float64MultiArray, messageCb)
# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()
