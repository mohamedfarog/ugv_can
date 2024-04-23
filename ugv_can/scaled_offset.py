import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import can


class CmdvelToCan(Node):
    def __init__(self):
        super().__init__("cmdvel_to_can")
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.can_bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=500000)
        self.initial_linear_x = 0.0
        self.initial_angular_z = 0.68  # Initial offset value for steering


    def cmd_vel_callback(self, msg):
        print("***********************")
        print(msg.linear.x, msg.angular.z)
        if msg is not None:
            # Ensure angular.z is within the range of -1 to 1

            if msg.angular.z > 1.0:
                scaled_angular_z = 100
            elif msg.angular.z < -1.0:
                scaled_angular_z = -100
            else:
                scaled_angular_z = int(msg.angular.z * 40)

            scaled_linear_x = int((msg.linear.x + 1) * 50)

            # Adjusting the initial angular.z value for steering offset
            if self.initial_angular_z != 0.0:
                scaled_angular_z += int(self.initial_angular_z * 100)

            print("*********************************")
            print(scaled_linear_x, scaled_angular_z)

            can_message_data = [scaled_linear_x, scaled_angular_z]
            can_message = can.Message(arbitration_id=0x100, data=can_message_data)
            try:
                self.can_bus.send(can_message)
                self.get_logger().info("sent can message: linear_x=%d, angular=%d" % (scaled_linear_x, scaled_angular_z))
            except can.CanError:
                self.get_logger().error("Failed to send can message")

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_to_can = CmdvelToCan()
    if not cmd_vel_to_can.can_bus:
        return
    rclpy.spin(cmd_vel_to_can)
    cmd_vel_to_can.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
