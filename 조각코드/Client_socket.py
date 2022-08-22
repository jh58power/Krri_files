import socket
import threading
import time

def send(sock):
    while True:
        sendData = input("")
        sock.send(sendData.encode())
        if sendData == 'Q':
            client_socket.close()
def receive(sock):
    while True:
        recvData = sock.recv(1024)
        print(" ")
        print("상대방 >>", recvData.decode())


# 서버 IP 및 열어줄 포트
HOST = '10.10.33.161'
PORT = 9999


client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST,PORT))
print('접속 완료')

sender = threading.Thread(target=send,args=(client_socket,))
receiver = threading.Thread(target=receive,args=(client_socket,))

sender.start()
receiver.start()

while True:
    time.sleep(1)
    pass