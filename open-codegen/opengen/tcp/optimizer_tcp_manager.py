import yaml, os, subprocess, socket, json
from threading import Thread
from retry import retry


class OptimizerTcpManager:

    def __init__(self, optimizer_path):
        self.__optimizer_path = optimizer_path
        self.__tcp_details = None

    def __load_tcp_details(self):
        yaml_file = os.path.join(self.__optimizer_path, "optimizer.yml")
        with open(yaml_file, 'r') as stream:
            self.__tcp_details = yaml.safe_load(stream)

    def __threaded_start(self):
        command = ['cargo', 'run']
        p = subprocess.Popen(command, cwd=self.__optimizer_path)
        p.wait()

    @retry(tries=10, delay=1)
    def __obtain_socket_connection(self):
        print('[TRY] connecting...')
        tcp_data = self.__tcp_details
        ip = tcp_data['tcp']['ip']
        port = tcp_data['tcp']['port']
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ip, port))
        print('[OK] connected!...')
        return s

    def __send_receive_data(self, text_to_send, buffer_size=512):
        conn_socket = self.__obtain_socket_connection()
        encoded_data = text_to_send.encode()
        conn_socket.sendall(encoded_data)
        conn_socket.shutdown(socket.SHUT_WR)
        for _i in range(100):
            data = conn_socket.recv(buffer_size)
            if data is not None:
                break
        conn_socket.close()
        return data.decode()

    def ping(self):
        request = '{"Ping":1}'
        data = self.__send_receive_data(request)
        print(data)
        return data

    def start(self):
        self.__load_tcp_details()
        thread = Thread(target=self.__threaded_start)

        # start the server
        thread.start()

        # ping the server until it responds so that we know it's
        # up and running
        self.ping()

    def kill(self):
        request = '{"Kill":1}'
        self.__send_receive_data(request)

    def call(self, p, buffer_len=4096):
        # Make request
        run_message = '{"Run" : {"parameter": ['
        parameter_comma_separated_list = ','.join(map(str, p))
        run_message += parameter_comma_separated_list
        run_message += ']}}'

        data = self.__send_receive_data(run_message, buffer_len)
        return data
