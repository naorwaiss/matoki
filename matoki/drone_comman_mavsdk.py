import socket
import asyncio
from mavsdk import System
import re

class DroneCommand:
    def __init__(self, address, port):
        """Initialize the UDP client with server address and port."""
        self.address = address
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.drone = System()


        self.data_size = 0
        self.case = 0
        self.axis_yaw = 0
        self.axis_z = 0
        self.axis_roll = 0
        self.axis_pitch = 0
        self.linear_converter = 1/500

        self.button_A = 0
        self.button_B = 0

        self.takeoff_flag = 0
    async def connect_to_drone(self):
        """Connect to the drone via MAVSDK."""
        try:
            await self.drone.connect(system_address="udp://:14540")
            print("Connected to drone.")
        except ConnectionRefusedError:
            print("Connection refuse")

    async def main_loop(self):
        """Send an empty message and then continuously receive messages."""
        self.sock.sendto(b'', (self.address, self.port))  # Sending an empty byte string
        while True:
            data = self.sock.recv(3)
            if data:
                length = int(data.decode())
                received_message = self.sock.recv(length).decode()



                parts = received_message.split()
                #print(parts)

                self.data_size = parts[0]
                self.case = parts[2]

                self.button_A = parts[6]
                self.button_B = parts[7]
                move_data = parts[4].split(',')
                self.axis_yaw = int(move_data[0])
                self.axis_z = int(move_data[1])
                self.axis_roll = int(move_data[2])
                self.axis_pitch = int(move_data[3])

                velo_yaw = self.axis_yaw*self.linear_converter
                velo_z = self.axis_z*self.linear_converter
                velo_roll = self.axis_roll*self.linear_converter
                velo_pitch = self.axis_pitch*self.linear_converter

                print(velo_yaw, velo_z, velo_roll,velo_pitch)




            else:
                break  # Exit loop if no data received

    async def close(self):
        """Close the UDP socket."""
        self.sock.close()
        print("Connection closed.")


async def takeoff(self):
    """Commands the drone to takeoff."""
    print("Taking off...")
    await self.drone.action.arm()
    await self.drone.action.set_takeoff_altitude(3)
    await self.drone.action.takeoff()
    self.takeoff = 1


async def main():
    client = DroneCommand("127.0.0.1", 5005)
    await client.connect_to_drone()  # Connect to the drone first
    try:
        await client.main_loop()
    finally:
        await client.close()

if __name__ == "__main__":
    asyncio.run(main())



#have problem with the casting
#have problem with the send of the button
#trhead or somthing else ?
