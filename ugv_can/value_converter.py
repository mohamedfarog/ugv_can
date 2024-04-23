import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import can

class CmdvvelConverter(Node):
    def __init__(self):
        super().__init__('cmdvel_converter')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan_ctypes')

    def cmd_vel_callback(self,msg):
        msg.linear.x = (msg.linear.x + 1) * 50
        msg.linear.y = (msg.linear.y + 1) * 1
        msg.linear.z = (msg.linear.z + 1) * 1


        msg.angular.x = (msg.angular.x + 1) * 1
        msg.angular.y = (msg.angular.y + 1) * 1
        msg.angular.z = (msg.angular.z + 1) * 50

        # can_msg = can.Message(arbitration_id=0xC0FFEE, data=[0, 25, 0, 1, 3, 1, 4, 1], is_extended_id=True)

        # self.get_logger().info('Converted cmdvel: linear=%s, angular=%s' % (msg.linear, msg.angular))


def main(args=None):
        rclpy.init(args=args)
        cmd_vel_to_can = CmdvvelConverter()
        rclpy.spin(cmd_vel_to_can)
        cmd_vel_to_can.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
