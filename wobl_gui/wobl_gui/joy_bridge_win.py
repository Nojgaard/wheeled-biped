import pygame
import socket
import struct
import time

# UDP settings
UDP_IP = "127.0.0.1"  # WSL can bind to localhost
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Initialize joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Using joystick:", joystick.get_name())

while True:
    pygame.event.pump()
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

    # Pack as simple floats/ints (axis count + button count can vary)
    msg = struct.pack(f'{len(axes)}f{len(buttons)}B', *(axes + buttons))
    sock.sendto(msg, (UDP_IP, UDP_PORT))

    time.sleep(0.05)  # 20 Hz
