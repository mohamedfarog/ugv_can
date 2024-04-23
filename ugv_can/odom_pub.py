import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import math


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('base_node')
        self.subscription = self.create_subscription(
            Twist,
            'vel_raw',
            self.handle_vel,
            50
        )
        self.odom_publisher = self.create_publisher(Odometry, '/odom_raw', 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.linear_scale_x = 0.0
        self.linear_scale_y = 0.0
        self.vel_dt = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.heading = 0.0
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.wheelbase = 0.25
        self.pub_odom_tf = False
        self.last_vel_time = self.get_clock().now()

        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('linear_scale_x', 1.0)
        self.declare_parameter('linear_scale_y', 1.0)
        self.declare_parameter('pub_odom_tf', True)

        self.get_parameter('linear_scale_x', self.linear_scale_x)
        self.get_parameter('linear_scale_y', self.linear_scale_y)
        self.get_parameter('wheelbase', self.wheelbase)
        self.get_parameter('pub_odom_tf', self.pub_odom_tf)

    def handle_vel(self, msg):
        curren_time = self.get_clock().now()
        self.linear_velocity_x = msg.linear.x * self.linear_scale_x
        self.linear_velocity_y = msg.linear.y * self.linear_scale_y
        self.vel_dt = (curren_time - self.last_vel_time).nanoseconds / 1e9
        self.last_vel_time = curren_time
        steer_angle = self.linear_velocity_y
        MI_PI = 3.1416
        R = self.wheelbase / math.tan(steer_angle / 180.0 * MI_PI)
        angular_velocity_z = self.linear_velocity_x / R
        delta_heading = angular_velocity_z * self.vel_dt
        delta_x = (self.linear_velocity_x * math.cos(self.heading)) * self.vel_dt
        delta_y = (self.linear_velocity_x * math.sin(self.heading)) * self.vel_dt
        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading += delta_heading

        my_quaternion = Quaternion()
        my_quaternion.setRPY(0.00, 0.00, self.heading)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x_pos
        odom.pose.pose.position.y = self.y_pos
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = my_quaternion
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001

        odom.twist.twist.linear.x = self.linear_velocity_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity_z
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[7] = 0.0001
        odom.twist.covariance[35] = 0.0001

        self.odom_publisher.publish(odom)

        if self.pub_odom_tf:
            t = TransformStamped()
            now = self.get_clock().now()
            t.header.stamp = now.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.x_pos
            t.transform.translation.y = self.y_pos
            t.transform.translation.z = 0.0

            t.transform.rotation = my_quaternion

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
