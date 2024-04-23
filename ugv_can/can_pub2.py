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

    def cmd_vel_callback(self, msg):
        # Limit linear speed to range [-0.6, 0.6]
        if msg.linear.x > 0.6:
            scaled_linear_x = 0.6
        elif msg.linear.x < -0.6:
            scaled_linear_x = -0.6
        else:
            scaled_linear_x = msg.linear.x

        scaled_linear_x *= 50  # Scaling factor for linear velocity
        scaled_angular_z = int((msg.angular.z + 3) * (100 / 6))
        can_message_data = [int(scaled_linear_x), scaled_angular_z]
        can_message = can.Message(arbitration_id=0x123, data=can_message_data)

        try:
            self.can_bus.send(can_message)
            self.get_logger().info("sent can message: linear_x=%f, angular=%d" % (scaled_linear_x, scaled_angular_z))
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

if __name__ == '__main__':
    main()
