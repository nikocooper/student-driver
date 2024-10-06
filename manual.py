import keyboard
import time
import socket

PORT = 6666

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

while True:
    start = time.time()

    press_string = ""

    if keyboard.is_pressed("left arrow"):
        press_string += "0 "
    elif keyboard.is_pressed("right arrow"):
        press_string += "1 "
    else :
        press_string += "2 "
    if keyboard.is_pressed("up arrow"):
        press_string += "256 "
    elif keyboard.is_pressed("down arrow"):
        press_string += "-256 "
    else:
        press_string += "0 "
    if keyboard.is_pressed("space"):
        press_string += "1"
    else:
        press_string += "0"

    if keyboard.is_pressed("x"):
          break
    
                                                                       
    end = time.time()
    if end - start < 0.1:
        time.sleep(0.1 - (end - start))
            
    print(press_string)

    MSG = press_string.encode("UTF-8")

    sock.sendto(MSG, ('<broadcast>', PORT))