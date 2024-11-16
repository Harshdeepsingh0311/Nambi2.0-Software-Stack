#!/usr/bin/env pself.ython
import rospy
from arm.msg import Pwm
from sensor_msgs.msg import Joy
import sys
import math


class Arm:



	def __init__(self):
		rospy.init_node("Arm_publisher")
		self.pub = rospy.Publisher("set",Pwm,queue_size=1)
		#self.pub = rospself.y.Publisher('robotic_arm', Pwm,queue_size=1)  
		self.x=42
		self.y=33
		self.rate = rospy
		rospy.Subscriber("joy", Joy, self.joyCallback)
		self.set = Pwm()

	def main(self):
		r = rospy.Rate(4)
		while not rospy.is_shutdown():
			self.pub.publish(self.set)
			print(self.set)
			print('\n ------- \n')
			r.sleep()

	def inverseKinematics(self):
		length1 = 33
		length2 = 42

		rad_angle2 = math.acos((math.pow(self.x,2)+ math.pow(self.y,2) - math.pow(length1,2) - math.pow(length2,2)) / (2*length1*length2))
		rad_angle1= int(math.atan(self.y / self.x) - math.atan((length2*(math.sin(rad_angle2))) / (length1+ length2*(math.cos(rad_angle2)))))

		self.set.shoulder_angle= int((rad_angle1*180)/math.pi)
		self.set.elbow_angle= int((rad_angle2*180)/math.pi)	

	""" def ForwardKinematics(self):
		length1 = 34
		length2 = 38

		rad_angle1 = (self.set.shoulder_angle*math.pi)/180
		rad_angle2 = (self.set.elbow_angle*math.pi)/180
		x = length1 * math.cos(rad_angle1) +length2 * math.cos(rad_angle1 + rad_angle2)
		self.y = length1 * math.sin(rad_angle1) +length2 * math.sin (rad_angle1 + rad_angle2)
       """

	def joyCallback(self, msg):

		#rt/bt
		# if(abs(msg.axes[2]) > 0):
		# 	self.set.base = 200 - (200.0*msg.axes[2])
		# else:
		# 	self.set.base = 0
		
		# if(abs(msg.axes[5]) > 0):
		# 	self.set.base = - int(200 - (200.0*msg.axes[5]))
		# else:
		# 	self.set.base = 0	

	
		
		# left side axis up/down for elbow
		if(abs(msg.axes[1]) > 0.2):
			self.x += int(1.0*msg.axes[1])
			self.inverseKinematics()
		elif(abs(msg.axes[1]) < 0.2):
			self.x += int(1.0*msg.axes[1])
			self.inverseKinematics()
		else:
			self.x += 0

				

        # left side axis left/right for shoulder
		if(abs(msg.axes[0]) > 0.2):
			self.y += int(1.0*msg.axes[0])
			self.inverseKinematics()
		
		elif(abs(msg.axes[0]) < 0.2):
			self.y += int(1.0*msg.axes[0])
			self.inverseKinematics()
		
		else:
			self.y += 0
	


	   	#right side axis left/right for self.yaw
		if(abs(msg.axes[4]) > 0.2):
			self.set.yaw = int(255.0*msg.axes[4])
		
		elif(abs(msg.axes[4]) < 0.2):
			self.set.yaw = int(255.0*msg.axes[4])
		
		else:
			self.set.yaw = 0

		#right side axis up/down for pitch
		if(abs(msg.axes[3]) > 0.2):
			self.set.base = int(255.0*msg.axes[3])
		
		elif(abs(msg.axes[3]) < 0.2):
			self.set.base = int(255.0*msg.axes[3])
		
		else:
			self.set.base = 0

		# lb/rb buttons for roll
		self.set.pitch = -100*(msg.buttons[1]-msg.buttons[2])

		# lb/rb buttons for base
		self.set.gripper = 250*(msg.buttons[4]-msg.buttons[5])

		#self.set.base = -250*(msg.buttons[3]-msg.buttons[0])
	

'''
        # Computing angle 2 Elbow up/down 
		numerator = ((length1 + length2)**2) - ((x**2) + (self.y**2))
		denominator = ((x**2) + (self.y**2)) - ((length1 - length2)**2)
		angle2UP = math.degrees(math.atan(math.sqrt(numerator/denominator)))
		angle2DOWN = angle2UP * -1
		#self.set.shoulder_angle
        #self.set.elbow_angle
        # Angle 1 Elbow up
		numerator = (length2 * math.sin(math.radians(angle2UP)))
		denominator = ((length1 + length2) * math.cos(math.radians(angle2UP)))
		angle1UP = math.degrees(math.atan2(numerator, denominator))
        # Angle 1 Elbow down
		numerator = (length2 * math.sin(math.radians(angle2DOWN)))
		denominator = ((length1 + length2) * math.cos(math.radians(angle2DOWN)))
		angle1DOWN = math.degrees(math.atan2(numerator, denominator))
		print("Angle 1 Elbow up: " + str(angle1UP))
		print("Angle 1 Elbow down: " + str(angle1DOWN))
		print("Angle 2 Elbow up: " + str(angle2UP))
		print("Angle 2 Elbow down: " + str(angle2DOWN))		
'''    
			
if __name__ == '__main__':
	x = Arm()
	x.main()
	rospy.spin()

