#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32,Bool
import time
from time import sleep

class JoyTeleop(Node):
	def __init__(self,name):
		super().__init__(name)
		self.max_speed = 0.3
		self.get_logger().info("PS4 CONTROLLER STARTERD")

		self.controller_state_value = True
		self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
		
		#create sub
		self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback,10)
		self.obstacle_state_sub = self.create_subscription(Int32,'obstacle_state',self._obstacle_state_callback,10)
		
	def _obstacle_state_callback(self,obstacle_state):
		if not isinstance(obstacle_state, Int32): return

		if obstacle_state.data == 4:
			self.controller_state_value = False
			# self.get_logger().info("Restricted move")
			move = Twist()
			self.pub_cmdVel.publish(move)
		else:
			self.controller_state_value = True
			# self.get_logger().info("Active move")

	def buttonCallback(self,joy_data):
		
		if self.controller_state_value == False : return
		
		xlinear_speed = joy_data.axes[1]
		ylinear_speed = 0.0
		angular_speed = joy_data.axes[3]
		
		twist = Twist()
		
		twist.linear.x = xlinear_speed * 1
		twist.linear.y = ylinear_speed * 1
		twist.angular.z = angular_speed * 1
		if twist.linear.x < 1.1:	
			self.pub_cmdVel.publish(twist)
				
def main():
	rclpy.init()
	joy_ctrl = JoyTeleop('navrover_joy_ctrl')
	rclpy.spin(joy_ctrl)		

if __name__ == "__main__":
	main()
	