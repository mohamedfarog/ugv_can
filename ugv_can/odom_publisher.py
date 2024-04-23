import can
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

# Constants
WHEEL_RADIUS = 0.1  # Example wheel radius in meters

class CanSubscriber(Node):

    def __init__(self):
        super().__init__('can_subscriber')
        self.bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=500000)
        self.publisher_ = self.create_publisher(Odometry, '/odom_raw', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning


    def twos_complement(self, hex_string,  num_bits):
        value = int(hex_string, 16)
        if value & (1 << (num_bits - 1)):
            value -= 1 << num_bits
        return value

    def listener_callback(self, msg):

        while True:
            can_msg = self.bus.recv(None)
            if can_msg.arbitration_id == 0x200:
                hex_value1 = hex(can_msg.data[2])
                hex_value2 = hex(can_msg.data[3])

                value1 = self.twos_complement(hex_value1, 8)
                value2 = self.twos_complement(hex_value2, 8)
                
                concatenated_hex = hex_value1[2:] + hex_value2[2:]
                # decimal_sum = int(concatenated_hex, 16)
                rpm = ((value1 << 8) + value2) / 29.4
                linear_speed = (2 * 3.14159 * WHEEL_RADIUS * rpm) / 60

                odometry_msg = Odometry()
                odometry_msg.twist.twist.linear = Vector3(x=linear_speed, y=0.0, z=0.0)
                self.publisher_.publish(odometry_msg)



def main(args=None):
    rclpy.init(args=args)

    can_subscriber = CanSubscriber()

    rclpy.spin(can_subscriber)

    can_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

