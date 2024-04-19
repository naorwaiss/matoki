import pygame
import socket


class JoystickController:
    def __init__(self):
        self.running = False
        self.server_ip = "127.0.0.1"
        self.server_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.server_ip, self.server_port))
        self.flag_takeoff = 0
        self.flag_landing = 1
        self. mode = 0
        # Handle joystick input
        self.axis_yaw =0
        self.axis_z =0
        self.axis_roll=0
        self.axis_pitch=0
        _, self.client = self.sock.recvfrom(1)
        self.case = 1

        print(self.client)
        print(f"Server listening on {self.server_ip}:{self.server_port}")


        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick connected.")
            pygame.quit()
            quit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def start(self):
        self.running = True
        self._run()



    def _run(self):
        while self.running:
            pygame.event.pump()  # Process events from the queue

            # Handle joystick input
            self.axis_yaw = int(self.make_zero(self.joystick.get_axis(0)) * 1000)
            self.axis_z = int(self.make_zero(self.joystick.get_axis(1)) * 1000)
            self.axis_roll = int(self.make_zero(self.joystick.get_axis(3)) * 1000)
            self.axis_pitch = int(self.make_zero(self.joystick.get_axis(4)) * 1000)
            self.button_A = self.joystick.get_button(0)
            self.button_B = self.joystick.get_button(1)

            #
            # if button_A == 1:
            #     self.flag_takeoff = 1
            #     self.flag_landing = 0
            #
            #
            #
            #
            # button_B = self.joystick.get_button(1)
            # if button_B == 1 and button_A == 1 and self.flag_landing == 0:
            #     self.flag_landing = 1
            #     self.flag_takeoff = 0

            if self.button_A and self.button_B:
                self.case = 2 #takeoff



            #print(f"Axis yaw: {self.axis_yaw} , Axis z: {self.axis_z} , Axis roll: {self.axis_roll}, Axis pitch: {self.axis_pitch}")

            #print(f"self.flag_takeoff: {self.flag_takeoff}, self.flag_landing: {self.flag_landing}")
            data = f" case:{self.case}"
            data += f" data: {self.axis_yaw},{self.axis_z},{self.axis_roll},{self.axis_pitch} button: {self.button_A}, {self.button_B} naot "
            messages_len = str(len(data)).zfill(3)
            data = messages_len + data

            self.sock.sendto(data.encode('utf-8'),self.client)









    def stop(self):
        self.running = False

    def make_zero(self, value):
        if abs(value) < 0.01:
            return 0
        else:
            return value
    def flag(self,value):
        if value:
            self.flag_takeoff = 1




def main():
    # Create the joystick controller and bind the socket server
    joystick_controller = JoystickController()
    # Start the joystick controller loop
    joystick_controller.start()

    # Keep the main thread running
    while joystick_controller.running:
        pass


if __name__ == "__main__":
    main()
