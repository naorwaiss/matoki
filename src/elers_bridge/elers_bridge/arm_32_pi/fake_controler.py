import uinput
import time
import roslibpy

# Set the IP address and port of the ROS 2 machine running the rosbridge server
ros = roslibpy.Ros(host='192.168.1.151', port=9090)

# Define the joystick device with 4 axes and 2 buttons
events = (
    uinput.ABS_X + (-32768, 32767, 0, 0),
    uinput.ABS_Y + (-32768, 32767, 0, 0),
    uinput.ABS_Z + (-32768, 32767, 0, 0),  # Add Z axis
    uinput.ABS_RX + (-32768, 32767, 0, 0),  # Add RX axis
    uinput.ABS_RY + (-32768, 32767, 0, 0),
    uinput.ABS_RZ + (-32768, 32767, 0, 0),
    uinput.ABS_TX + (-32768, 32767, 0, 0),


    uinput.BTN_JOYSTICK,
    uinput.BTN_THUMB,
)

# Create the virtual joystick device with a custom name
device_name = "Custom Virtual Joystick"
device = uinput.Device(events, name=device_name)

print(f"Created virtual joystick with name: {device_name}")


# Define a callback function to handle incoming messages
def callback(message):
    #print('Received message:', message)

    # Extract axes and buttons data from the message
    axes = message['axes']
    buttons = message['buttons']

    # Map axes data to virtual joystick movements
    device.emit(uinput.ABS_X, int(axes[0] * 32767))
    device.emit(uinput.ABS_Y, int(axes[1] * 32767))
    device.emit(uinput.ABS_Z, int(axes[2] * 32767))
    device.emit(uinput.ABS_RX, int(axes[3] * 32767))
    device.emit(uinput.ABS_RY, int(axes[4] * 32767))
    device.emit(uinput.ABS_RZ, int(axes[5] * 32767))
    device.emit(uinput.ABS_TX, int(axes[6] * 32767))

    # Map buttons data to virtual joystick buttons
    device.emit(uinput.BTN_JOYSTICK, buttons[0])  # Button 1
    device.emit(uinput.BTN_THUMB, buttons[1])  # Button 2


# Define the topic and message type
listener = roslibpy.Topic(ros, '/joy', 'sensor_msgs/Joy')

# Connect to the WebSocket server
ros.run()

# Subscribe to the topic
listener.subscribe(callback)

# Keep the script running and handle connection lifecycle events
try:
    print('Waiting for connection...')
    ros.on_ready(lambda: print('Connected to WebSocket server'), run_in_thread=True)
    ros.on('close', lambda: print('Connection to WebSocket server closed.'))
    input("Press Enter to exit...\n")
except KeyboardInterrupt:
    pass
finally:
    # Clean up
    listener.unsubscribe()
    ros.terminate()
    print('Disconnected from WebSocket server.')
