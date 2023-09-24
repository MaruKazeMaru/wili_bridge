#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import os
import socket
import rclpy
from rclpy.node import Node
from wili_msgs.msg import HMM
from wili_msgs.srv import GetHMM
from threading import Thread
import struct

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

        self.req_tr_prob = int.to_bytes(0, 1, 'little')

        self.motion_num_i = 0
        self.motion_num_b = self.motion_num_i.to_bytes(1, 'little', signed=False)
        self.tr_prob_b:bytes = None
        # self.avr_where_user_b:bytes = None
        # self.var_where_user_b:bytes = None
        # self.floor_where_user_b:bytes = None

        self.cli_get_hmm = self.create_client(GetHMM, 'get_hmm')


    def __del__(self):
        self.destroy_node()


    def destroy_node(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
            if os.path.exists(self.sock_file_path):
                os.remove(self.sock_file_path)
            super().destroy_node()
        except: pass


    def update_hmm_param(self, hmm:HMM):
        n = hmm.motion_num
        self.motion_num_i = n
        self.motion_num_b = n.to_bytes(1, 'little', signed=False)
        fmt = 'f' * (n * n)
        self.tr_prob_b = struct.pack('<' + fmt, *(hmm.tr_prob))
        # fmt = 'f' * n
        # self.floor_where_user_b = struct.pack('<' + fmt, *(hmm.floor_where_user))
        # fmt = 'f' * (n * 2)
        # self.avr_where_user_b = struct.pack('<' + fmt, *(hmm.avr_where_user))
        # fmt = 'f' * (n * 3)
        # self.var_where_user_b = struct.pack('<' + fmt, *(hmm.var_where_user))


    def init_hmm_param(self):
        self.cli_get_hmm.wait_for_service()
        future = self.cli_get_hmm.call_async(GetHMM.Request())
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                break
        self.update_hmm_param(future.result().hmm)


    def serve_loop(self, connection:socket.socket):
        while True:
            try:
                req = connection.recv(1024)
                res = self.get_response(req)
                connection.send(res)
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


    def get_response(self, req:bytes) -> bytes:
        if req == self.req_tr_prob:
            if self.motion_num_i == 0:
                return self.motion_num_b
            else:
                return self.motion_num_b + self.tr_prob_b
        else:
            return int.to_bytes(0, 1, 'little')


def main():
    rclpy.init()
    node = SocketBridge("/tmp/sock.sock")
    Thread(target=node.accept).start()
    try:
        node.init_hmm_param()
        node.logger.info("start")
        rclpy.spin(node)
    except KeyboardInterrupt: print('')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()