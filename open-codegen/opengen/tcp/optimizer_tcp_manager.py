import yaml
import os
import subprocess
import socket
from threading import Thread
from retry import retry
import time


class OptimizerTcpManager:

    def __init__(self, optimizer_path):
        self.__optimizer_path = optimizer_path
        self.__tcp_details = None
        self.__socket = None

    def __load_tcp_details(self):
        yaml_file = os.path.join(self.__optimizer_path, "optimizer.yml")
        with open(yaml_file, 'r') as stream:
            self.__tcp_details = yaml.safe_load(stream)

    def __threaded_start(self):
        command = ['cargo', 'run']
        p = subprocess.Popen(command, cwd=self.__optimizer_path)
        p.wait()

    def start(self):
        self.__load_tcp_details()
        thread = Thread(target=self.__threaded_start)
        thread.start()
        time.sleep(2)

    @retry(ConnectionRefusedError, tries=1, delay=2)
    def __obtain_socket_connection(self, ip, port):
        print("connecting...")
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.connect((ip, port))
        print("CONNECTED :-)")

    def connect(self):
        tcp_details = self.__tcp_details
        ip = tcp_details['tcp']['ip']
        port = tcp_details['tcp']['port']
        self.__obtain_socket_connection(ip, port)
        self.__socket.listen(1)



