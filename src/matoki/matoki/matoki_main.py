import subprocess
import os
import sys


def terminate_processes(processes):
    for process in processes:
        process.kill()


# Change directory to the required path
os.chdir("/home/navkit/mjpg-streamer/mjpg-streamer-experimental")

# Command 1 - Camera 1
command1 = "./mjpg_streamer -i 'input_uvc.so -d /dev/video2 -r 640x480 -f 15' -o 'output_http.so -w ./www' > /dev/null 2>&1"

# Command 2 - Camera 2
command2 = "./mjpg_streamer -i 'input_uvc.so -d /dev/video0 -r 640x480 -f 15' -o 'output_http.so -w ./www -p 5050' > /dev/null 2>&1"

# Command 3 - mavproxy.py
#command3 = "sudo mavproxy.py --master=/dev/ttyUSB0 --out=192.168.1.191:14550"

# Execute command 1 and command 2 in background
process1 = subprocess.Popen(command1, shell=True)
process2 = subprocess.Popen(command2, shell=True)

# Execute command 3 and display its output
#process3 = subprocess.Popen(command3, shell=True)

# No need to wait for the processes to finish if you want them to run concurrently

print("All commands executed successfully.")
print("Press 'q' to terminate all processes.")

try:
    while True:
        if sys.stdin.read(1) == 'q':
            terminate_processes([process1, process2])
            break
except KeyboardInterrupt:
    print("\nKeyboardInterrupt: Terminating processes.")
    terminate_processes([process1, process2])
