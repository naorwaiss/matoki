import RPi.GPIO as GPIO
import time
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class ServoController(Node):
    def __init__(self, pin, axis, pwm_min, pwm_max):
        super().__init__('servo_controller')
        self.pin = pin
        self.axis = axis
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.subscription  # prevent unused variable warning
        self.setup_gpio()
        self.current_angle = 90  # Start at neutral position
        self.set_servo_angle(self.current_angle)

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)  # 50 Hz (20 ms PWM period)
        self.pwm.start(0)

    def set_servo_angle(self, angle):
        duty = self.pwm_min + (angle / 180.0) * (self.pwm_max - self.pwm_min)
        GPIO.output(self.pin, True)
        self.pwm.ChangeDutyCycle(duty)

    def joy_callback(self, msg):
        axis_value = msg.axes[self.axis]
        target_angle = 90 + (axis_value * 90)  # Convert to angle between 0 and 180
        self.move_servo_smoothly(target_angle)

    def move_servo_smoothly(self, target_angle):
        step = 1 if target_angle > self.current_angle else -1
        for angle in range(int(self.current_angle), int(target_angle), step):
            self.set_servo_angle(angle)
            time.sleep(0.01)  # Adjust for smoothness and speed
        self.current_angle = target_angle

    def stop(self):
        self.pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Control a servo motor with joystick input.')
    parser.add_argument('-p', '--pin', type=int, required=True, help='GPIO pin number to which the servo is connected.')
    parser.add_argument('-a', '--axis', type=int, required=True, help='Joystick axis index to control the servo.')
    parser.add_argument('--pwm_min', type=float, required=True, help='Minimum PWM duty cycle for the servo.')
    parser.add_argument('--pwm_max', type=float, required=True, help='Maximum PWM duty cycle for the servo.')
    args = parser.parse_args()

    # Convert microseconds to duty cycle percentage
    pwm_min_duty = (args.pwm_min / 20000.0) * 100
    pwm_max_duty = (args.pwm_max / 20000.0) * 100

    servo_controller = ServoController(args.pin, args.axis, pwm_min_duty, pwm_max_duty)

    try:
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass

    servo_controller.stop()
    servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
