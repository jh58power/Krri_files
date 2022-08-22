import socket
import threading
import time


def Robot2Pc(r_sock, p_sock):
    while True:
        recvData = r_sock.recv(1024)
        sendData = recvData
        p_sock.send(sendData)
        print("Robot --> PC : ", recvData.decode())
        
def Pc2Robot(r_sock,p_sock):
    while True:
        recvData = p_sock.recv(1024)
        sendData = recvData
        r_sock.send(sendData)
        print("PC --> Robot : ", recvData.decode())
        
# 서버 IP 및 열어줄 포트
INNER_HOST = '10.10.33.161'
ROBOT_HOST = '192.168.2.2'
PORT = 9999

# --------------------내부망 소켓 생성-------------------------------
print('>> INNER Server Start')
server_inner = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_inner.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_inner.bind((INNER_HOST, PORT))
server_inner.listen(1)

print("%d번 포트로 대기중..."%PORT)

client_inner, inner_addr = server_inner.accept()

print(str(inner_addr), '에서 접속')

# --------------------로봇 소켓 생성-------------------------------

print('>> ROBOT Server Start')
server_robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_robot.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_robot.bind((ROBOT_HOST, PORT))
server_robot.listen(1)

print("%d번 포트로 대기중..."%PORT)

client_robot, robot_addr = server_robot.accept()

print(str(robot_addr), '에서 접속')


Robot_Pc = threading.Thread(target=Robot2Pc,args=(client_robot,client_inner))
Pc_Robot = threading.Thread(target=Pc2Robot,args=(client_robot,client_inner))

Robot_Pc.start()
Pc_Robot.start()

while True:
    time.sleep(1)
    pass