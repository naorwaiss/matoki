import RPi.GPIO as GPIO
import argparse
import pygame
import time


class ServoController:
    def __init__(self, pin, pwm_min, pwm_max):
        self.pin = pin
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.setup_gpio()
        self.current_duty = pwm_min  # Start at minimum duty cycle
        self.set_servo_duty(self.current_duty)

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)  # 50 Hz (20 ms PWM period)
        self.pwm.start(0)

    def set_servo_duty(self, duty):
        GPIO.output(self.pin, True)
        self.pwm.ChangeDutyCycle(duty / 20000.0 * 100)  # Convert microseconds to duty cycle percentage
        self.current_duty = duty

    def stop(self):
        self.pwm.stop()
        GPIO.cleanup()


def main():
    parser = argparse.ArgumentParser(description='Control a servo motor with a joystick.')
    parser.add_argument('-p', '--pin', type=int, required=True, help='GPIO pin number to which the servo is connected.')
    parser.add_argument('--pwm_min', type=int, required=True,
                        help='Minimum PWM duty cycle for the servo in microseconds.')
    parser.add_argument('--pwm_max', type=int, required=True,
                        help='Maximum PWM duty cycle for the servo in microseconds.')
    parser.add_argument('--axis', type=int, default=0, help='Joystick axis to use for controlling the servo.')
    args = parser.parse_args()

    servo_controller = ServoController(args.pin, args.pwm_min, args.pwm_max)

    pygame.init()
    pygame.joystick.init()

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print("Move the joystick axis to control the servo. Press 'Ctrl+C' to quit.")

        while True:
            pygame.event.pump()
            axis_value = joystick.get_axis(args.axis)

            # Convert joystick value (-1.0 to 1.0) to PWM duty cycle
            duty_cycle = args.pwm_min + (axis_value + 1) / 2 * (args.pwm_max - args.pwm_min)
            servo_controller.set_servo_duty(duty_cycle)

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        servo_controller.stop()
        pygame.quit()


if __name__ == '__main__':
    main()
