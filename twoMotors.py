'''This code functions as a manual control algorithm that uses the arrow keys 
to control the bot's movement. It connects to a Raspberry Pi onboard the robot 
via Bluetooth RFCOMM sockets. The code on the pi must be running first in order 
to connect.'''
import keyboard
import socket
import time


#computer side:
pi_mac_address = "B8:27:EB:D8:81:F0"
port = 1
sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
while True:
    if keyboard.is_pressed("x"):
        break
    try:
        sock.connect((pi_mac_address, port))
        break
    except:
        time.sleep(1)
        continue



#pi side:

'''
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
'''
#Output format: "forward/backward left/right speed fire"
#0 = forward, 1 = backward
#0 = left, 1 = right
#speed = 1, 2, or 3
#0 = no fire, 1 = fire
speed = '1'
while True:
    message = [2,2]
    fire = '0'
    if keyboard.is_pressed("up arrow"):
        message[0] = 0
    elif keyboard.is_pressed("down arrow"):
        message[0] = 1
    if keyboard.is_pressed("left arrow"):
        message[1] = 0
    elif keyboard.is_pressed("right arrow"):
        message[1] = 1
    if keyboard.is_pressed("q"):
        speed = '1'
    elif keyboard.is_pressed("w"):
        speed = '2'
    elif keyboard.is_pressed("e"):
        speed = '3'
    if keyboard.is_pressed("x"):
        sock.sendall(b'2 2 0')
        break
    if keyboard.is_pressed("space"):
        fire = '1'
    print(str(message[0]) + " " + str(message[1]) + " " + speed + " " + fire)
    '''print(message)'''
    MSG = (str(message[0]) + " " + str(message[1]) + " " + speed + " " + fire).encode('UTF-8')
    '''MSG = message.encode('UTF-8')'''
    #sock.sendto(MSG, ('<broadcast>', PORT))
    sock.sendall(MSG)
    time.sleep(.15)
sock.close()