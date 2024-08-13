import subprocess
import signal
import os
import time

# Paths to the programs
joystick_control_path = '/home/naor/elrs-joystick-control'
mjpeg_streamer_path = '/home/naor/mjpg-streamer/mjpg-streamer-experimental'
#fake_controler_path = '/home/naor' #change it when i finish all the code
fake_controler_path = '/home/naor/src/elers_bridge/elers_bridge/arm_32_pi'



# Commands to run
joystick_control_command = './elrs-joystick-control'
mjpeg_streamer_command = 'mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 640x480 -f 30" -o "output_http.so -w ./www -p 8080"'
fake_control_command = 'sudo python3 fake_controler.py' #need to check it

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


def terminate_processes():
    if joystick_control_process:
        os.killpg(os.getpgid(joystick_control_process.pid), signal.SIGTERM)
    if mjpeg_streamer_process:
        os.killpg(os.getpgid(mjpeg_streamer_process.pid), signal.SIGTERM)
    if fake_control_process:
        os.killpg(os.getpgid(fake_control_process.pid), signal.SIGTERM)
    print("Processes terminated.")

def signal_handler(sig, frame):
    terminate_processes()
    exit(0)

def main():
    # Register signal handler to handle termination
    signal.signal(signal.SIGINT, signal_handler)
    # Start the processe
    run_fake_control()
    time.sleep(5)  # sleep to wait to the controller get data and pupup


    run_joystick_control()
    run_mjpeg_streamer()


    print("Press 'Q' to quit.")

    # Wait for 'Q' key press to quit
    while True:
        user_input = input()
        if user_input.lower() == 'q':
            terminate_processes()
            break

if __name__ == "__main__":
    main()
