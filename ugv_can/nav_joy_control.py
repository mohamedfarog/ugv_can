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
		self.pub_cmdVel.publish(twist)
				
def main():
	rclpy.init()
	joy_ctrl = JoyTeleop('navrover_joy_ctrl')
	rclpy.spin(joy_ctrl)		

if __name__ == "__main__":
	main()






class CmdvelToCan(Node):
    def __init__(self):
        super().__init__('cmdvel_to_can')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.can_bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate='500000')
        
        # Initialize initial velocity values
        self.initial_linear_x = 0.0
        self.initial_angular_z = 0.0

        # Define center position for angular scaling

    def cmd_vel_callback(self, msg):

        # Scale linear x and angular z
        scaled_linear_x = int((msg.linear.x + 1) * 50)
        scaled_angular_z = int((msg.angular.z + 3) * (100 / 6))
        
        # Ensure the scaled value stays within the range of 0 to 100
        scaled_angular_z = max(0, min(100, scaled_angular_z))

        can_message_data = [scaled_linear_x, scaled_angular_z]
        can_message = can.Message(arbitration_id=0x100, data=can_message_data)

        try:
            self.can_bus.send(can_message)
            self.get_logger().info("Sent CAN message: linear_x=%d, angular=%d" % (scaled_linear_x, scaled_angular_z))
        except can.CanError:
            self.get_logger().error("Failed to send CAN message")