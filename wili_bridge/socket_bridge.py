import os
import socket
import rclpy
from rclpy.node import Node
from wili_msgs.msg import TrProbMat
from threading import Thread

class SocketBridge(Node):
    def __init__(self, sock_file_path:str, listen_num=1):
        super().__init__("socket_bridge")
        self.logger = self.get_logger()
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

    def serve_loop(self, connection:socket.socket):
        while True:
            try:
                req = connection.recv(1024).decode('utf-8')
                res = "res:req=" + req
                connection.send(res.encode('utf-8'))
            except BrokenPipeError:
                break
            except Exception as e:
                print("{} in serve_loop".format(e))
                break

    def accept(self):
        while True:
            try:
                conn, addr = self.sock.accept()
                self.logger.info("conn {}".format(conn))
                Thread(target=self.serve_loop, args=(conn,)).start()
            except KeyboardInterrupt: break
            except Exception as e:
                print("<{}> in accept".format(e))
                break


def main():
    rclpy.init()
    node = SocketBridge("/tmp/sock.sock")
    Thread(target=node.accept).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.close()


if __name__ == "__main__":
    main()