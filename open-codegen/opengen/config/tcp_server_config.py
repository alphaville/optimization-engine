class TcpServerConfiguration:

    def __init__(self, bind_ip='127.0.0.1', bind_port=4598):
        self.__bind_ip = bind_ip
        self.__bind_port = bind_port

    @property
    def bind_ip(self):
        return self.__bind_ip

    @property
    def bind_port(self):
        return self.__bind_port