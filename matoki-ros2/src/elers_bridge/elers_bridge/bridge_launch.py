import subprocess
import signal
import os
import time
import paramiko
import threading
import sys

# Paths to the local programs
joystick_control_path = '/home/naor/elrs-joystick-control'
mjpeg_streamer_path = '/home/naor/mjpg-streamer/mjpg-streamer-experimental'
fake_controler_path = '/home/naor/src/elers_bridge/elers_bridge/arm_32_pi'

# Commands to run
joystick_control_command = './elrs-joystick-control'
mjpeg_streamer_command = 'mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 640x480 -f 30" -o "output_http.so -w ./www -p 8080"'
fake_control_command = 'sudo python3 fake_controler.py'

# Global variables to store process references
joystick_control_process = None
mjpeg_streamer_process = None
fake_control_process = None

def run_joystick_control():
    global joystick_control_process
    joystick_control_process = subprocess.Popen(
        f'cd {joystick_control_path} && {joystick_control_command}',
        shell=True,
        preexec_fn=os.setsid
    )

def run_mjpeg_streamer():
    global mjpeg_streamer_process
    mjpeg_streamer_process = subprocess.Popen(
        f'cd {mjpeg_streamer_path} && {mjpeg_streamer_command}',
        shell=True,
        preexec_fn=os.setsid
    )

def run_fake_control():
    global fake_control_process
    fake_control_process = subprocess.Popen(
        f'cd {fake_controler_path} && {fake_control_command}',
        shell=True,
        preexec_fn=os.setsid
    )

def terminate_local_processes():
    if joystick_control_process:
        os.killpg(os.getpgid(joystick_control_process.pid), signal.SIGTERM)
    if mjpeg_streamer_process:
        os.killpg(os.getpgid(mjpeg_streamer_process.pid), signal.SIGTERM)
    if fake_control_process:
        os.killpg(os.getpgid(fake_control_process.pid), signal.SIGTERM)
    print("Local processes terminated.")

# SSH Remote Controller class
class SSHRemoteController:
    def __init__(self, remote_host, username, password, remote_script):
        self.remote_host = remote_host
        self.username = username
        self.password = password
        self.remote_script = remote_script
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.remote_process = None

    def connect(self):
        try:
            self.ssh_client.connect(self.remote_host, username=self.username, password=self.password)
            print("Connected to remote host.")
        except Exception as e:
            print(f"Failed to connect to remote host: {e}")

    def disconnect(self):
        if self.remote_process:
            self.stop_remote_script()
        self.ssh_client.close()
        print("SSH connection closed.")

    def start_remote_script(self):
        try:
            # Execute the remote script in the background
            self.remote_process = self.ssh_client.exec_command(f'bash -c "{self.remote_script}"')
            print("Remote script started.")
        except Exception as e:
            print(f"Failed to start remote script: {e}")

    def stop_remote_script(self):
        try:
            # Attempt to terminate the remote script
            self.ssh_client.exec_command('pkill -f "python3 /home/naor/src/elers_bridge/elers_bridge/arm_32_pi/python_launch.py"')
            print("Remote script stopped.")
        except Exception as e:
            print(f"Failed to stop remote script: {e}")

    def monitor_input(self):
        while True:
            user_input = input("Press 'a' to start the remote script, 'q' to stop it, 'w' to exit both: ")
            if user_input == 'a':
                self.start_remote_script()
            elif user_input == 'q':
                self.stop_remote_script()
            elif user_input == 'w':
                self.stop_remote_script()
                terminate_local_processes()
                break

    def start(self):
        self.connect()
        input_thread = threading.Thread(target=self.monitor_input)
        input_thread.daemon = True
        input_thread.start()
        try:
            while input_thread.is_alive():
                input_thread.join(1)
        finally:
            self.disconnect()

def signal_handler(sig, frame):
    # Gracefully handle termination signal
    print("Terminating...")
    terminate_local_processes()
    controller.disconnect()
    sys.exit(0)

if __name__ == "__main__":
    remote_host = '192.168.0.101'
    username = 'naor'
    password = '1'
    remote_script = 'python3 /home/naor/src/elers_bridge/elers_bridge/arm_32_pi/python_launch.py'

    # Start local processes
    run_fake_control()
    time.sleep(5)  # sleep to wait for the controller to get data and pop up
    run_joystick_control()
    run_mjpeg_streamer()

    # Register signal handler to gracefully exit on Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    controller = SSHRemoteController(remote_host, username, password, remote_script)
    controller.start()
