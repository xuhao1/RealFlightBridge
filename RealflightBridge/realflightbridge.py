import socket

class RealflightBridge:
    def __init__(self, IP="127.0.0.1", PORT=18083):
        self.IP = IP
        self.PORT = PORT

    def connect(self):
        print(f"Try to connect to RealFlight vi8a {self.IP}:{self.PORT}")
        self.sock = socket.socket() 
        try:
            self.sock.connect((self.IP, self.PORT))
        except Exception as e:
            print(f"Connection to RealFlight {self.IP}:{self.PORT} failed!")
            raise e
            return False
        print(f"Connection to RealFlight Success!")
        return True

    def start(self):
        self.send_RestoreOriginalControllerDevice()
        self.send_InjectUAVControllerInterface()


    def set_control(self, controls):
        pass
    
    def soap_request_end(self):
        pass

    def soap_request_start(self, action, req1):
        data = f"""POST / HTTP/1.1
soapaction: '{action}'
content-length: {len(req1)}
content-type: text/xml;charset='UTF-8'
Connection: Keep-Alive

{req1}"""
        data = bytes(data, 'utf-8')
        print(f"Send data:\n{data.decode('utf-8')}#\n")
        self.sock.send(data)
    
    def soap_request_end(self, t):
        self.sock.settimeout(t/1000.0)
        data = self.sock.recv(10000)
        print(f"recv:\n{data.decode('utf-8')}#")
        return data
    
    def send_RestoreOriginalControllerDevice(self):
        self.soap_request_start("RestoreOriginalControllerDevice", 
"""<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>
</soap:Body>
</soap:Envelope>""")
        self.soap_request_end(1000)

    def send_InjectUAVControllerInterface(self):
        self.soap_request_start("InjectUAVControllerInterface", 
"""<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
</soap:Body>
</soap:Envelope>""")
        self.soap_request_end(1000)
