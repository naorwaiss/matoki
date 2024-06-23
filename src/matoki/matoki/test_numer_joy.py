import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription1 = self.create_subscription(
            Joy,
            '/controller_1/joy',
            self.listener_callback_1,
            10)
        self.subscription2 = self.create_subscription(
            Joy,
            '/controller_2/joy',
            self.listener_callback_2,
            10)

    def listener_callback_1(self, msg):
        self.get_logger().info('Controller 1: Axes: %s, Buttons: %s' % (msg.axes, msg.buttons))

    def listener_callback_2(self, msg):
        self.get_logger().info('Controller 2: Axes: %s, Buttons: %s' % (msg.axes, msg.buttons))

def main(args=None):
    rclpy.init(args=args)
    joy_subscriber = JoySubscriber()
    rclpy.spin(joy_subscriber)
    joy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
