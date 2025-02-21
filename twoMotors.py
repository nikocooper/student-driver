import keyboard
import socket
import time
#PORT = 6666


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
speed = '1'
while True:
    message = [2,2]
    '''if keyboard.is_pressed("left arrow"):
            message = '2 1'
    elif keyboard.is_pressed("right arrow"):
            message = '1 2'
    elif keyboard.is_pressed("up arrow"):
            message = '0 0'
    elif keyboard.is_pressed("down arrow"):
            message = '1 1'  '''
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
    print(str(message[0]) + " " + str(message[1]) + " " + speed)
    '''print(message)'''
    MSG = (str(message[0]) + " " + str(message[1]) + " " + speed).encode('UTF-8')
    '''MSG = message.encode('UTF-8')'''
    #sock.sendto(MSG, ('<broadcast>', PORT))
    sock.sendall(MSG)
    time.sleep(.15)
sock.close()