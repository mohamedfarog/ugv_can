# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
# import can

# class CmdvelToCan(Node):
#     def __init__(self):
#         super().__init__('cmdvel_to_can')
#         self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
#         # self.pub = self.create_publisher(String, 'stm32_can', 10)
#         self.can_bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate='500000')
#         # self.get_logger('cmd_to_can initialized')

#     def cmd_vel_callback(self,msg):
#         scaled_linear_x = int((msg.linear.x + 1) * 50)
#         scaled_angular_z = int((msg.angular.z + 3) * (100 / 6))
#         can_message_data = [scaled_linear_x, scaled_angular_z]
#         can_message = can.Message(arbitration_id=0x123, data=can_message_data)

#         try:
#             self.can_bus.send(can_message)
#             self.get_logger().info("sent can message: linear_x=%d, angular=%d" % (scaled_linear_x, scaled_angular_z))
#         except can.CanError:
#             self.get_logger().error("Failed to send can message")


# def main(args=None):
#         rclpy.init(args=args)
#         cmd_vel_to_can = CmdvelToCan()
#         if not cmd_vel_to_can.can_bus:
#             return
#         rclpy.spin(cmd_vel_to_can)
#         cmd_vel_to_can.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
# import can

# class CmdvelToCan(Node):
#     def __init__(self):
#         super().__init__('cmdvel_to_can')
#         self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
#         # self.pub = self.create_publisher(String, 'stm32_can', 10)
#         self.can_bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate='500000')
#         self.current_speed = 0.0
#         # self.get_logger('cmd_to_can initialized')

#     def cmd_vel_callback(self, msg):
#         # Apply speed limit
#         max_speed = 0.3
#         min_speed = -0.3
#         if msg.linear.x > max_speed:
#             msg.linear.x = max_speed
#         elif msg.linear.x < min_speed:
#             msg.linear.x = min_speed

#         # Calculate speed increment based on proportional control
#         speed_increment = 0.005  # Adjust this value as needed
#         desired_speed = msg.linear.x
#         speed_difference = desired_speed - self.current_speed

#         # Gradually adjust the current speed towards the desired speed
#         if abs(speed_difference) > speed_increment:
#             self.current_speed += speed_increment if speed_difference > 0 else -speed_increment
#         else:
#             self.current_speed = desired_speed

#         scaled_linear_x = int((self.current_speed + 1) * 50)
#         scaled_angular_z = int((msg.angular.z + 3) * (100 / 6))
#         can_message_data = [scaled_linear_x, scaled_angular_z]
#         can_message = can.Message(arbitration_id=0x123, data=can_message_data)

#         try:
#             self.can_bus.send(can_message)
#             self.get_logger().info("Sent CAN message: linear_x=%d, angular=%d" % (scaled_linear_x, scaled_angular_z))
#         except can.CanError:
#             self.get_logger().error("Failed to send CAN message")


# def main(args=None):
#     rclpy.init(args=args)
#     cmd_vel_to_can = CmdvelToCan()
#     if not cmd_vel_to_can.can_bus:
#         return
#     rclpy.spin(cmd_vel_to_can)
#     cmd_vel_to_can.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
##################################################    MAIN CODE ###################################################################################
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import can

class CmdvelToCan(Node):
    def __init__(self):
        super().__init__('cmdvel_to_can')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.can_bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate='500000')
        
        # Initialize initial velocity values
        self.initial_linear_x = 0.0
        self.initial_angular_z = 1.12


    def cmd_vel_callback(self, msg):

        # Scale linear x and angular z
        scaled_linear_x = int((msg.linear.x + 1) * 50)
        scaled_angular_z = int((msg.angular.z + 3) * (100 / 6))
        
        # Ensure the scaled value stays within the range of 0 to 100
        scaled_angular_z = max(0, min(100, scaled_angular_z))

        print("*********************************")

        print(scaled_linear_x, scaled_angular_z)

        can_message_data = [scaled_linear_x, scaled_angular_z]
        can_message = can.Message(arbitration_id=0x100, data=can_message_data)

        try:
            self.can_bus.send(can_message)
            self.get_logger().info("Sent CAN message: linear_x=%d, angular=%d" % (scaled_linear_x, scaled_angular_z))
        except can.CanError:
            self.get_logger().error("Failed to send CAN message")


    # def cmd_vel_callback(self, msg):
    #     # Center value for angular z
    #     center_angular_z = 1.12

    #     # Calculate the difference between received angular_z and center value
    #     angular_z_diff = msg.angular.z - center_angular_z

    #     # Scale linear x based on the received angular_z difference
    #     scaled_linear_x = int((msg.linear.x + 1) * 50)

    #     # Scale angular z based on the difference from the center value
    #     scaled_angular_z = int((angular_z_diff + 3) * (100 / 6))

    #     # Ensure the scaled value stays within the range of 0 to 100
    #     scaled_angular_z = max(0, min(100, scaled_angular_z))

    #     can_message_data = [scaled_linear_x, scaled_angular_z]
    #     can_message = can.Message(arbitration_id=0x100, data=can_message_data)

    #     try:
    #         self.can_bus.send(can_message)
    #         self.get_logger().info("Sent CAN message: linear_x=%d, angular=%d" % (scaled_linear_x, scaled_angular_z))
    #     except can.CanError:
    #         self.get_logger().error("Failed to send CAN message")
    # def cmd_vel_callback(self, msg):
    #     # Define center point for angular scaling
    #     angular_center_value = 1.12

    #     # Scale linear x
    #     scaled_linear_x = int((msg.linear.x + 1) * 50)

    #     # Adjust center position for angular scaling
    #     scaled_angular_z = int((msg.angular.z + 3) * (100 / 6))

    #     # Calculate adjustment for center point
    #     angular_center_offset = (angular_center_value + 3) * (100 / 6)

    #     # Adjust scaled value based on center point
    #     scaled_angular_z = scaled_angular_z - angular_center_offset

    #     # Ensure the scaled value stays within the range of 0 to 100
    #     scaled_angular_z = max(0, min(100, scaled_angular_z))

    #     can_message_data = [scaled_linear_x, scaled_angular_z]
    #     can_message = can.Message(arbitration_id=0x100, data=can_message_data)

    #     try:
    #         self.can_bus.send(can_message)
    #         self.get_logger().info("Sent CAN message: linear_x=%d, angular=%d" % (scaled_linear_x, scaled_angular_z))
    #     except can.CanError:
    #         self.get_logger().error("Failed to send CAN message")




def main(args=None):
    rclpy.init(args=args)
    cmd_vel_to_can = CmdvelToCan()
    if not cmd_vel_to_can.can_bus:
        return
    # Publish initial velocity values
    initial_twist = Twist()
    initial_twist.linear.x = cmd_vel_to_can.initial_linear_x
    initial_twist.angular.z = cmd_vel_to_can.initial_angular_z
    cmd_vel_to_can.cmd_vel_callback(initial_twist)
    rclpy.spin(cmd_vel_to_can)
    cmd_vel_to_can.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
##################################################    MAIN CODE ###################################################################################




# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
# import can

# class CmdvelToCan(Node):
#     def __init__(self):
#         super().__init__('cmdvel_to_can')
#         self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
#         self.can_bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate='500000')
        
#         # Initialize initial velocity values
#         self.initial_linear_x = 0.2
#         self.initial_angular_z = 0.0
        
#         # Initialize current velocity values
#         self.current_linear_x = self.initial_linear_x
#         self.current_angular_z = self.initial_angular_z

#         # Initialize timer and decrease rate
#         self.decrease_timer = None
#         self.decrease_rate = 0.01  # Adjust this rate according to your requirement

#     def cmd_vel_callback(self, msg):
#         # Apply speed limit
#         max_speed = 0.2
#         min_speed = -0.2
#         if msg.linear.x > max_speed:
#             msg.linear.x = max_speed
#         elif msg.linear.x < min_speed:
#             msg.linear.x = min_speed

#         # Gradually decrease speed if needed
#         if msg.linear.x == 0 and self.current_linear_x != 0:
#             if self.decrease_timer is None:
#                 self.decrease_timer = self.create_timer(0.05, self.decrease_speed)
#         else:
#             self.current_linear_x = msg.linear.x
#             self.current_angular_z = msg.angular.z
#             self.send_can_message()

#     def decrease_speed(self):
#         # Decrease linear speed gradually
#         if self.current_linear_x > 0:
#             self.current_linear_x -= self.decrease_rate
#             if self.current_linear_x < 0:
#                 self.current_linear_x = 0
#         elif self.current_linear_x < 0:
#             self.current_linear_x += self.decrease_rate
#             if self.current_linear_x > 0:
#                 self.current_linear_x = 0
        
#         # Send updated speed
#         self.send_can_message()

#         # Stop timer if speed is already zero
#         if self.current_linear_x == 0:
#             self.decrease_timer.cancel()
#             self.decrease_timer = None

#     def send_can_message(self):
#         scaled_linear_x = int((self.current_linear_x + 1) * 50)
#         scaled_angular_z = int((self.current_angular_z + 3) * (100 / 6))
#         can_message_data = [scaled_linear_x, scaled_angular_z]
#         can_message = can.Message(arbitration_id=0x123, data=can_message_data)

#         try:
#             self.can_bus.send(can_message)
#             self.get_logger().info("sent can message: linear_x=%d, angular=%d" % (scaled_linear_x, scaled_angular_z))
#         except can.CanError:
#             self.get_logger().error("Failed to send can message")

# def main(args=None):
#     rclpy.init(args=args)
#     cmd_vel_to_can = CmdvelToCan()
#     if not cmd_vel_to_can.can_bus:
#         return
#     # Publish initial velocity values
#     initial_twist = Twist()
#     initial_twist.linear.x = cmd_vel_to_can.initial_linear_x
#     initial_twist.angular.z = cmd_vel_to_can.initial_angular_z
#     cmd_vel_to_can.cmd_vel_callback(initial_twist)
#     rclpy.spin(cmd_vel_to_can)
#     cmd_vel_to_can.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
