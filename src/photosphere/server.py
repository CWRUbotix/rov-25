import pickle
import socket
import struct

import cv2

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host_name  = socket.gethostname()
host_ip = socket.gethostbyname(host_name)
print('HOST IP: ', host_ip)
port = 9997
socket_address = (host_ip, port)

server_socket.bind(socket_address)

server_socket.listen(5)
print('LISTENING AT: ', socket_address)

MJPG = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')

while True:
    client_socket, addr = server_socket.accept()
    print('GOT CONNECTION FROM: ', addr)
    if client_socket:
        vid = cv2.VideoCapture('/dev/video2')
        vid.set(cv2.CAP_PROP_FOURCC, MJPG)
        vid.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 3032)

        while vid.isOpened():
            img, frame = vid.read()
            frame = cv2.resize(frame, (960, 758), interpolation=cv2.INTER_LINEAR)
            a = pickle.dumps(frame)
            message = struct.pack('Q', len(a)) + a
            client_socket.sendall(message)

            cv2.imshow('TRANSMITTING VIDEO', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                client_socket.close()
