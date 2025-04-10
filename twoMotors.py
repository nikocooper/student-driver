'''This code functions as a manual control algorithm that uses the arrow keys 
to control the bot's movement. It connects to a Raspberry Pi onboard the robot 
via Bluetooth RFCOMM sockets. The code on the pi must be running first in order 
to connect.'''
import keyboard
import socket
import time

#computer side:
pi_mac_address = "B8:27:EB:13:00:AD" # input specific bluetooth address of Raspberry Pi being used
# first pi: B8:27:EB:13:00:AD
# second pi: B8:27:EB:7E:A5:F3
port = 1
sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

# connect to the Raspberry Pi
while True:
    if keyboard.is_pressed("x"):
        break
    try:
        sock.connect((pi_mac_address, port))
        break
    except:
        time.sleep(1)         
        continue
                         


#pi side:w

'''Output format: "forward/backward left/right speed fire"
0 = forward, 1 = backward
0 = left, 1 = right
speed = 1, 2, or 3
0 = no fire, 1 = fire, -1 = low speed retract weapon, -5 = high speed retract weapon'''
speed = '3'
f = '3'
speed = '3'
f = '3'
while True:
    message = [2,2]
    fire = '0'
    # control bot manually with arrow keys
    if keyboard.is_pressed("up arrow"):
        message[0] = 0
    elif keyboard.is_pressed("down arrow"):
        message[0] = 1
    if keyboard.is_pressed("left arrow"):    
        message[1] = 0
    elif keyboard.is_pressed("right arrow"):
        message[1] = 1
    # set speed and firepower with q/w/e keys
    if keyboard.is_pressed("q"):
        speed = '1'
        f = '1'
    elif keyboard.is_pressed("w"):
        speed = '2'
        f = '2'  
    elif keyboard.is_pressed("e"):
        speed = '3'
        f = '3'
    # exit manual driving loop with 'x' key
    if keyboard.is_pressed("x"):
        sock.sendall(b'2 2 0 0')
        break
    # fire with spacebar 
    if keyboard.is_pressed("space"):
        fire = f
    # retract weapon at full power with left shift and low power with enter
    elif keyboard.is_pressed("left shift"):
        fire = '-5'
    elif keyboard.is_pressed("enter"):
        fire = '-1'
    print(str(message[0]) + " " + str(message[1]) + " " + speed +                                                                               " " + fire)
    MSG = (str(message[0]) + " " + str(message[1]) + " " + speed + " " + fire + " ").encode('UTF-8')
    sock.sendall(MSG)
    # send commands every 1/6 of a second
    time.sleep(.15)
sock.close()
