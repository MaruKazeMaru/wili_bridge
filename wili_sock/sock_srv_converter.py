import os
import socket
import rclpy
from rclpy.node import Node
from wili_msgs.msg import TrProbMat
from threading import Thread

class SockSrvConverter(Node):
    def __init__(self, sock_file_path:str, listen_num=1):
        super().__init__("sock_srv_converter")
        self.sock_file_path = sock_file_path
        self.listen_num = listen_num
        self.sock = socket.socket( \
            family=socket.AF_UNIX, \
            type=socket.SOCK_STREAM, \
            proto=0 \
        )
        self.sock.bind(sock_file_path)
        self.sock.listen(listen_num)

    def __del__(self):
        self.close()

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
            if os.path.exists(self.sock_file_path):
                os.remove(self.sock_file_path)
        except:
            pass

    def serve_loop(self, connection, address):
        while True:
            try:
                sock_recv = connection.recv(self.__buffer).decode('utf-8')
                sock_resp = "res:req=" + sock_recv
                connection.send(sock_resp.encode('utf-8'))
            except ConnectionResetError:
                break
            except BrokenPipeError:
                break

    def accept(self):
        for i in range(self.listen_num):
            conn, addr = self.sock.accept()
            Thread(target=self.serve_loop, args=(conn,addr))


if __name__ == "__main__":
    node = SockSrvConverter()
    Thread(target=rclpy.spin, args=(node,))
    node.accept()
