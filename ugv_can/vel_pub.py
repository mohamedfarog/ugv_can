	

#!/usr/bin/env python3
# encoding: utf-8

#public lib
import sys
import math
import random
import threading
from math import pi
from time import sleep
from Navmaster_Lib import Navmaster

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock
import copy as copy

class Navcon_driver(Node):
	
	def __init__(self, name):
		super().__init__(name)

		print("Navcon_driver")
		self.get_logger().info("TUGV DRIVER STARTERD")
		
		self.is4WD= False
		self.RA2DE = 180 / pi
		self.imu_temp = Imu()
		self.mag_temp = MagneticField()

		self.declare_parameter('nav_use_rotvel', False)
		self.nav_use_rotvel = self.get_parameter('nav_use_rotvel').get_parameter_value().bool_value
		
		#create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		
		#create publisher
		self.staPublisher = self.create_publisher(JointState,"joint_states",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"/imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",100)

		self.imu_subscribe = self.create_subscription(Imu,"imu/og_data_raw",self.IMU_Callback,1)
		self.mag_subscribe = self.create_subscription(MagneticField,"imu/og_mag",self.MAG_Callback,1)
		

		#create timer
		self.timer = self.create_timer(0.1, self.pub_data)

	def cmd_vel_callback(self,msg):
		if not isinstance(msg, Twist): return
		
		vx = msg.linear.x*1.0
		vy = msg.linear.y*1.0
		angular = msg.angular.z*1.0

	def IMU_Callback(self,msg):
		if not isinstance(msg, Imu): return

		time_stamp = Clock().now()

		accel_ratio = 1 / 1671.84
		self.__ax = copy.deepcopy(msg).linear_acceleration.x*accel_ratio
		self.__ay = copy.deepcopy(msg).linear_acceleration.y*accel_ratio
		self.__az = copy.deepcopy(msg).linear_acceleration.z*accel_ratio

		# gyro_ratio = 1 / 3754.9 # ±500dps
		# self.__gx = copy.deepcopy(msg).angular_velocity.x*gyro_ratio
		# self.__gy = copy.deepcopy(msg).angular_velocity.y-gyro_ratio
		# self.__gz = copy.deepcopy(msg).angular_velocity.z-gyro_ratio
	
		self.imu_temp = copy.copy(msg)

		self.mag_temp.header.stamp = time_stamp.to_msg()
		
		# print(self.__ax)
		# print(float('%f' %self.__az))
		# self.imu_temp.linear_acceleration.x = float('%f' %self.__ax)
		# self.imu_temp.linear_acceleration.x = float('%f' %self.__ax)
		# self.imu_temp.linear_acceleration.y = float('%f' %self.__ay)
		# self.imu_temp.linear_acceleration.z = float('%f' %self.__az)

		# self.imu_temp.angular_velocity.x = 0.010120109723294895
		# self.imu_temp.angular_velocity.y = 0.01784335135423047
		# self.imu_temp.angular_velocity.z = 0.08468933926336254	

		# self.imu_temp.linear_acceleration.x = float('%f' %self.__ax)
		# self.imu_temp.linear_acceleration.y = float('%f' %self.__ay)
		# self.imu_temp.linear_acceleration.z = float('%f' %self.__az)

		# self.imu_temp.angular_velocity.x = float('%f' %self.__gx)
		# self.imu_temp.angular_velocity.y = float('%f' %self.__gy)
		# self.imu_temp.angular_velocity.z = float('%f' %self.__gz)
		
	def MAG_Callback(self,msg):
		if not isinstance(msg, MagneticField): return
		time_stamp = Clock().now()
		self.mag_temp = copy.copy(msg)
		self.mag_temp.header.stamp = time_stamp.to_msg()
		# self.mag_temp.magnetic_field.x = -679.0
		# self.mag_temp.magnetic_field.y = -163.0	
		# self.mag_temp.magnetic_field.z = 799.0

	def pub_data(self):
		time_stamp = Clock().now()
		twist = Twist()
		state = JointState()
		state.header.stamp = time_stamp.to_msg()
		state.header.frame_id = "joint_states"
		state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		vx, vy, angular = 0.0,0.0,0.0
		
		twist.linear.x = vx*1.0    #velocity in axis 
		twist.linear.y = vy*1000*1.0   #steer angle
		twist.angular.z = angular*1.0    #this is invalued
		
		# self.velPublisher.publish(twist)
		
		self.imuPublisher.publish(self.imu_temp)
		self.magPublisher.publish(self.mag_temp)	
		
		#turn to radis
		steer_radis = vy*1000.0*3.1416/180.0
		state.position = [0.0, 0.0, steer_radis, 0.0, steer_radis, 0.0]
		if not vx == angular == 0:
			i = random.uniform(-3.14, 3.14)
			state.position = [i, i, steer_radis, i, steer_radis, i]
		self.staPublisher.publish(state)
			
def main():
	rclpy.init() 
	driver = Navcon_driver('navrover_driver')
	rclpy.spin(driver)
		
if __name__ == "__main__":
    main()
		
