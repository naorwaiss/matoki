import subprocess
import signal
import os

#need to work on this code little bit 

# Paths to the programs
joystick_control_path = '/path/to/elrs-joystick-control'
mjpeg_streamer_path = '/path/to/mjpeg-streamer'

# Commands to run
joystick_control_command = './elers-joystick-control'
mjpeg_streamer_command = 'mjpeg-streamer -i "input_uvc.so -d /dev/video0" -o "output_http.so -w ./www"'

# Global variables to store process references
joystick_control_process = None
mjpeg_streamer_process = None

def run_joystick_control():
    global joystick_control_process
    joystick_control_process = subprocess.Popen(
        ['gnome-terminal', '--', 'bash', '-c', f'cd {joystick_control_path} && {joystick_control_command}']
    )

def run_mjpeg_streamer():
    global mjpeg_streamer_process
    mjpeg_streamer_process = subprocess.Popen(
        ['gnome-terminal', '--', 'bash', '-c', f'cd {mjpeg_streamer_path} && {mjpeg_streamer_command}']
    )

def terminate_processes():
    if joystick_control_process:
        os.killpg(os.getpgid(joystick_control_process.pid), signal.SIGTERM)
    if mjpeg_streamer_process:
        os.killpg(os.getpgid(mjpeg_streamer_process.pid), signal.SIGTERM)
    print("Processes terminated.")

def signal_handler(sig, frame):
    terminate_processes()
    exit(0)

def main():
    # Register signal handler to handle termination
    signal.signal(signal.SIGINT, signal_handler)

    # Start the processes
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
