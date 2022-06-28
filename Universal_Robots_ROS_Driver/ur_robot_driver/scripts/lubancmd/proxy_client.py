
import time
import zmq
import threading
import sys

class PS_Socket(object):
    PUB = 0
    SUB = 1
    PROXY_PROTOCOL_S      = "tcp://*:"
    PROXY_PROTOCOL_C      = "tcp://localhost:"
    PROXY_ADAPTERPUB_PORT = "5571"
    PROXY_PROXYPUB_PORT   = "5572" 

    def __init__(self, work_mode, proxy_addr = "localhost", callback = None, callback_args = None, io_threads = 1):
        self._context = zmq.Context()
        self._callback = callback
        self._callback_args = callback_args
        self._pyversion = sys.version_info[0]
        self._work_mode = work_mode
        if self._work_mode == self.PUB:
            self._pubsub_socket = self._context.socket(zmq.DEALER)
            self._proxy_addr = "tcp://" + proxy_addr + ":" + self.PROXY_ADAPTERPUB_PORT
        else:
            assert self._callback != None, "Not in subscribe mode!!!"
            self._pubsub_socket = self._context.socket(zmq.SUB)
            self._proxy_addr = "tcp://" + proxy_addr + ":" + self.PROXY_PROXYPUB_PORT
            workthread = threading.Thread(target=self._work_thread)
            workthread.setDaemon(True)
            workthread.start()
        self.connect()

    def connect(self):
        self._pubsub_socket.connect(self._proxy_addr)
             
    def publish(self, topic, content):
        if self._pyversion == 2:
            self._pubsub_socket.send(topic, zmq.SNDMORE)
            self._pubsub_socket.send(content)
        else:
            self._pubsub_socket.send_string(topic, zmq.SNDMORE)
            self._pubsub_socket.send_string(content)
    
    def subscribe(self, topic_list):
        assert self._work_mode == self.SUB, "Not in subscribe mode!!!"
        if self._pyversion == 2:
            for topic in topic_list:
                self._pubsub_socket.setsockopt(zmq.SUBSCRIBE, topic)
        else:
            for topic in topic_list:
                self._pubsub_socket.setsockopt_string(zmq.SUBSCRIBE, topic)

    def unsubscribe(self, topic_list):
        assert self._work_mode == self.SUB, "Not in subscribe mode!!!"
        if self._pyversion == 2:  
            for topic in topic_list:
                self._pubsub_socket.setsockopt(zmq.UNSUBSCRIBE, topic)
        else:
            for topic in topic_list:
                self._pubsub_socket.setsockopt_string(zmq.UNSUBSCRIBE, topic)

    def _work_thread(self):
        while True:
            msg = self._pubsub_socket.recv_multipart()
            topic = ""
            content = ""
            if self._pyversion == 2:
                topic = msg[0]
                content = msg[1]
            else: 
                topic = msg[0].decode("utf-8")
                content = msg[1].decode("utf-8")
            self._callback(self._callback_args, topic, content)


class ProxyClient(object):
    # define constant 
    ADAPTER = 0
    CLIENT = 1
    PROXY_PROTOCOL_S      = "tcp://*:"
    PROXY_PROTOCOL_C      = "tcp://localhost:"
    PROXY_CLIENT_PORT     = "5559"
    CMD_LOGIN             = "_login_" 

    def __init__(self, client_name, caller_args, callback, proxy_addr = "localhost", io_threads = 1):
        self._client_name = client_name
        self._context = zmq.Context()
        self._caller_args = caller_args
        self._socket = self._context.socket(zmq.DEALER)
        # self._dummy_s_socket = self._context.socket(zmq.ROUTER)
        self._callback = callback 
        self._proxy_addr = "tcp://" + proxy_addr
        self._pyversion = sys.version_info[0]

    def connect(self):
        if self._pyversion == 2:
            self._socket.setsockopt(zmq.IDENTITY, self._client_name)
        else:
            self._socket.setsockopt_string(zmq.IDENTITY, self._client_name)

        self._socket.connect(self._proxy_addr + ":"  + self.PROXY_CLIENT_PORT)             

        workthread = threading.Thread(target=self._work_thread)
        workthread.setDaemon(True)
        workthread.start()

    def send(self, dest, cmd, params):
        if self._pyversion == 2:
            self._socket.send(dest, zmq.SNDMORE)
            self._socket.send(cmd, zmq.SNDMORE)
            self._socket.send(params)
        else:
            self._socket.send_string(dest, zmq.SNDMORE)
            self._socket.send_string(cmd, zmq.SNDMORE)
            self._socket.send_string(params)

    def _work_thread(self):
        while True:
            msg = self._socket.recv_multipart()
            cmd = {}
            if self._pyversion == 2:
                cmd["cmdsrc"] = msg[0]
                cmd["cmd"] = msg[1]
                cmd["params"] = msg[2]
            else:
                cmd["cmdsrc"] = msg[0].decode("utf-8")
                cmd["cmd"] = msg[1].decode("utf-8")
                cmd["params"] = msg[2].decode("utf-8")
            del msg
            self._callback(self._caller_args, cmd)
    
class Socket(object):
    SERVER = 0
    CLIENT = 1
    def __init__(self, socket_addr, socket_name, caller_args, callback, io_threads = 1):
        self._socket_addr = socket_addr
        self._socket_name = socket_name
        self._context = zmq.Context()
        self._caller_args = caller_args
        self._callback = callback
        self._socket = None
        self._work_mode = -1
    
    def bind(self):
        self._work_mode = Socket.SERVER
        self._socket = self._context.socket(zmq.ROUTER)
        socket_thread = threading.Thread(target=self._work_thread)
        socket_thread.setDaemon(True)
        socket_thread.start()
        self._socket.bind(self._socket_addr)
        
    def connect(self):
        self._work_mode = Socket.CLIENT
        self._socket = self._context.socket(zmq.DEALER)
        self._socket.setsockopt(zmq.IDENTITY, self._socket_name)
        socket_thread = threading.Thread(target=self._work_thread)
        socket_thread.setDaemon(True)
        socket_thread.start()
        self._socket.connect(self._socket_addr)
    
    def s_send(self, dest, cmd, params):
        assert self._work_mode == self.SERVER, "Socket should be in server mode"
        self._socket.send(dest, zmq.SNDMORE)
        self._socket.send(cmd, zmq.SNDMORE)
        self._socket.send(params)

    def c_send(self, cmd, params):
        assert self._work_mode == self.CLIENT, "Socket should be in client mode"
        self._socket.send(self._socket_name, zmq.SNDMORE)
        self._socket.send(cmd, zmq.SNDMORE)
        self._socket.send(params)

    def _work_thread(self):
        # print(">>>  _work_thread")
        while True:
            msg = self._socket.recv_multipart()
            if self._work_mode == self.SERVER :
                cmd = {}
                cmd["src"] = msg[0]
                cmd["cmd"] = msg[1]
                cmd["params"] = msg[2]
            if self._work_mode == self.CLIENT :
                cmd = {}
                cmd["cmd"] = msg[0]
                cmd["params"] = msg[1]
            del msg
            self._callback(self._caller_args, cmd)



# following codes are only for testing
def client_thread(socket, interval):
    while True:
        socket.send("adapter_ex", "getPos", "This is from client_ex")
        # print("client_ex send ", "getPos")
        time.sleep(interval)
    # print("<< client_thread")

def adapter_callback(caller_args, cmd):
    caller_args._condition.acquire()
    caller_args._list.append(cmd)
    caller_args._condition.notify()
    caller_args._condition.release()
    # print("adpater recv: ", cmd)

def client_callback(caller_args, cmd):
    print("client recv: ", cmd)

class AppParams(object):
    def __init__(self):
        self._condition = threading.Condition()
        self._list = []

def main():
    proxy_ip = "localhost"
    app1 = AppParams() 
    adapter = ProxyClient("adapter_ex", app1, adapter_callback, proxy_ip)
    adapter.connect()

    app2 = AppParams() 
    client = ProxyClient("client_ex", app2, client_callback, proxy_ip)
    client.connect()

    th1 = threading.Thread(target=client_thread, args=(client, 3))
    th1.setDaemon(True)
    th1.start()

    idx = 0
    while True:
        app1._condition.acquire()
        if len(app1._list) <= 0:
            app1._condition.wait()
        cmd = app1._list.pop(0)
        app1._condition.release()

        idx = idx + 1
        # print "received cmd: ", cmd
        params = "\"ret:\"" + str(idx)
        adapter.send(cmd["cmdsrc"], cmd["cmd"], params)
        
if __name__ == "__main__":
    main()



    
