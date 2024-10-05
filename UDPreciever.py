import socket
import keyboard
PORT = 6666

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("",PORT))

while True:
    data, addr = sock.recvfrom(1024)
    print(f"recieved message: {data} from: {addr}")
    if keyboard.is_pressed("x"):
        break