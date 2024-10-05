import socket

PORT = 6666

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

MSG = b"Hello there"

sock.sendto(MSG, ('<broadcast>', PORT))