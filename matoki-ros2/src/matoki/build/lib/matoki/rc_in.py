import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn  # Ensure this is the correct message type
import time

class RCOverrideNode(Node):
    def __init__(self):
        super().__init__('rc_override_node')
        self.publisher_ = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 1)  # Update the publisher to OverrideRCIn
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            1
        )
        self.joy_subscription  # prevent unused variable warning

        # Call reset_parameters once at startup
        self.reset_parameters()

    def reset_parameters(self):
        # Set all channels to 65535 at the start to indicate no override
        rc_override = OverrideRCIn()
        rc_override.channels = [65535] * 12  # Assuming 12 channels
        self.publisher_.publish(rc_override)
        self.get_logger().info('RC Override parameters reset to 65535')
        time.sleep(2)
        rc_override.channels = [0] * 12  # Assuming 12 channels
        self.publisher_.publish(rc_override)
        self.get_logger().info('RC Override parameters reset to 0')

    def joy_callback(self, msg):
        rc_override = OverrideRCIn()  # Create an instance of OverrideRCIn

        # Map joystick axes/buttons to RC channels
        rc_override.channels = [
            self.scale_value(msg.axes[0], -1.0, 1.0, 1000, 2000),  # Roll (Ch1)
            self.scale_value(msg.axes[1], -1.0, 1.0, 1000, 2000),  # Pitch (Ch2)
            self.scale_value(msg.axes[2], -1.0, 1.0, 1000, 2000),  # Throttle (Ch3)
            self.scale_value(msg.axes[3], -1.0, 1.0, 1000, 2000),   # Yaw (Ch4)
            self.scale_value(msg.axes[4], -1.0, 1.0, 1000, 2000), #mode control
            self.scale_value(msg.axes[5], -1.0, 1.0, 1000, 2000)  # arm/disarm
        ]

        # Debug statements
        self.get_logger().info(f'Joy axes: {msg.axes}')
        self.get_logger().info(f'RC Override: {rc_override.channels}')

        # Publish the RC override message
        time.sleep(0.05)
        self.publisher_.publish(rc_override)

    @staticmethod
    def scale_value(value, src_min, src_max, dst_min, dst_max):
        # Scale value from source range to destination range
        src_range = src_max - src_min
        dst_range = dst_max - dst_min
        scaled_value = ((value - src_min) * dst_range) / src_range + dst_min
        return int(scaled_value)

def main(args=None):
    rclpy.init(args=args)
    node = RCOverrideNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
