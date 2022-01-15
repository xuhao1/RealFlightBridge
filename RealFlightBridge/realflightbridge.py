import requests
import xml.etree.cElementTree as ET
import numpy as np
from .utils import *
import math
#Reference code: https://github.com/camdeno/F16Capstone/blob/main/FlightAxis/flightaxis.py
G = np.array([0.0, 0.0, -9.8205])
import time

class LowpassFilter:
    def __init__(self, freq_cut):
        self.fc = freq_cut
        self.last_value = None

    def input(self, v, dt):
        if self.last_value is None:
            self.last_value = v
        
        RC = 1.0 / (self.fc * 2 * np.pi)
        alpha = dt / (RC + dt)
        if  math.isnan(alpha):
            alpha = 0
        v = self.last_value + (alpha* (v-self.last_value))
        self.last_value = v
        return v
    
    def value(self):
        return self.last_value

class RealFlightBridge:
    def __init__(self, IP="127.0.0.1", PORT=18083,freq_cut=50,freq_cut_acc=20):
        self.IP = IP
        self.PORT = PORT
        self.channels = [0 for i in range(32)]
        self.REALFLIGHT_URL = f"http://{IP}:{PORT}"
        self.debug = False
        self.reset_states()

        self.ang_vel_filter = LowpassFilter(freq_cut)
        self.acc_body_filter = LowpassFilter(freq_cut_acc)
        self.acc_body_no_g_filter = LowpassFilter(freq_cut_acc)

    def reset_states(self):
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.acc_body = np.array([0.0, 0.0, 0.0])
        self.acc_body_no_g = np.array([0.0, 0.0, 0.0])
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])

        self.yaw0 = 0
        self.roll = 0 
        self.pitch = 0
        self.yaw = 0
        
        self.pos_0 = None
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])

        self.t = None
        self.t0 = None

    def get_angular_velocity(self):
        return self.angular_velocity

    def connect(self):
        succ = self.send_RestoreOriginalControllerDevice()
        self.send_InjectUAVControllerInterface()
        if succ:
            print("Connect to RealFlight success")
        else:
            print("Connect to RealFlight failed")
        return True

    def start(self):
        self.send_ResetAircraft()
        pass

    def exit(self):
        pass

    def update(self):
        self.send_ExchangeData()
    
    def set_controls(self, controls):
        # Note here we set channels before the Radio. 
        # The values are the 'Input Channel XX'
        # So we need passthrough configuration in RealFlight
        # For Heli, should be Aileron, Elevator, Pitch, Rudder, Throttle.
        for i in range(len(controls)):
            self.channels[i] = float_constrain((controls[i] + 1.0)/2.0, 0, 1) 
            # self.channels[i] = (controls[i] + 1.0)
    
    def soap_request(self, action, req1, keep_alive=False):
        header = {'content-type': "text/xml;charset='UTF-8'",
                'soapaction': action}
        if keep_alive:
            header["Connection"] = 'Keep-Alive'
        # print("Header\n", header, "body\n", req1)
        response = requests.post(self.REALFLIGHT_URL,data=req1,headers=header)
        
        if self.debug:
            print(response.ok, response.content)
        return response.ok, response.content
    
    def send_RestoreOriginalControllerDevice(self):
        print("Try RestoreOriginalControllerDevice")
        ok, res = self.soap_request("RestoreOriginalControllerDevice", 
"<?xml version='1.0' encoding='UTF-8'?>\
        <soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>\
        <soap:Body>\
        <InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>\
        </soap:Body>\
        </soap:Envelope>", keep_alive=True)
        if ok:
            print("RestoreOriginalControllerDevice success")
            return True
        return False

    def send_InjectUAVControllerInterface(self):
        if self.debug:
            print("InjectUAVControllerInterface")
        ok, res = self.soap_request("InjectUAVControllerInterface", 
"""<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
</soap:Body>
</soap:Envelope>""")
        if ok:
            print("InjectUAVControllerInterface success")
            return True
        return False

    def send_ResetAircraft(self):
        print("Try ResetAircraft")
        time.sleep(0.1)
        self.set_controls([0.0, 0.0, -1.0, 0.0, -1.0])
        self.update()
        self.reset_states()
        ok, res = self.soap_request("ResetAircraft", 
"""<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<ResetAircraft><a>1</a><b>2</b></ResetAircraft>
</soap:Body>
</soap:Envelope>""")
        if ok:
            print("ResetAircraft success")
            time.sleep(1)
            self.update()
            return True

        return False

    def send_ExchangeData(self):
        chn = self.channels
        ok, res = self.soap_request("ExchangeData", 
f"""<?xml version='1.0' encoding='UTF-8'?><soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<ExchangeData>
<pControlInputs>
<m-selectedChannels>4095</m-selectedChannels>
<m-channelValues-0to1>
<item>{chn[0]:.4f}</item>
<item>{chn[1]:.4f}</item>
<item>{chn[2]:.4f}</item>
<item>{chn[3]:.4f}</item>
<item>{chn[4]:.4f}</item>
<item>{chn[5]:.4f}</item>
<item>{chn[6]:.4f}</item>
<item>{chn[7]:.4f}</item>
<item>{chn[8]:.4f}</item>
<item>{chn[9]:.4f}</item>
<item>{chn[10]:.4f}</item>
<item>{chn[11]:.4f}</item>
</m-channelValues-0to1>
</pControlInputs>
</ExchangeData>
</soap:Body>
</soap:Envelope>
""")
        # print("raw", res, "\n")
        doc = ET.fromstring(res)
        doc = doc[0][0]
        # print("doc", doc, "\n")
        # all_descendants = list(doc.iter())
        # print(all_descendants)

        #Looks like it's FLU in Realflight
        
        aircraftState = doc.findall('m-aircraftState')[0]
        angular_rate_yaw = aircraftState.findall('m-yawRate-DEGpSEC')[0].text
        angular_rate_pitch = aircraftState.findall('m-pitchRate-DEGpSEC')[0].text
        angular_rate_roll = aircraftState.findall('m-rollRate-DEGpSEC')[0].text
        acc_body_x = aircraftState.findall('m-accelerationBodyAX-MPS2')[0].text
        acc_body_y = aircraftState.findall('m-accelerationBodyAY-MPS2')[0].text
        acc_body_z = aircraftState.findall('m-accelerationBodyAZ-MPS2')[0].text

        qw = aircraftState.findall('m-orientationQuaternion-W')[0].text
        qx = aircraftState.findall('m-orientationQuaternion-X')[0].text
        qy = aircraftState.findall('m-orientationQuaternion-Y')[0].text
        qz = aircraftState.findall('m-orientationQuaternion-Z')[0].text
        _t = float(aircraftState.findall('m-currentPhysicsTime-SEC')[0].text)
        

        x = aircraftState.findall('m-aircraftPositionX-MTR')[0].text
        y = aircraftState.findall('m-aircraftPositionY-MTR')[0].text
        z = aircraftState.findall('m-altitudeAGL-MTR')[0].text

        vx = aircraftState.findall('m-velocityWorldU-MPS')[0].text
        vy = aircraftState.findall('m-velocityWorldV-MPS')[0].text
        vz = aircraftState.findall('m-velocityWorldW-MPS')[0].text
        angular_rate = np.array([float(angular_rate_roll), float(angular_rate_pitch), -float(angular_rate_yaw)])/360*np.pi
        acc_body = np.array([float(acc_body_x), float(acc_body_y), float(acc_body_z)])
        quat = np.array([float(qw), float(qx), float(qy), float(qz)])

        r, p, y = euler_from_quaternion(quat)

        if self.t is None:
            self.dt = 0.001
            self.t0 = _t
            self.t = 0
            self.yaw0 = y
            self.q0_inv = quaternion_from_euler(0, 0, -y)
        else:
            t = _t - self.t0
            self.dt = t - self.t
            self.t = t


        self.roll = r
        self.pitch = p
        self.yaw = y - self.yaw0

        pos_ned = np.array([float(y), float(x), -float(z)])
        vel_ned = np.array([float(vx), float(vy), float(vz)])

        if self.pos_0 is None:
            self.pos_0 = pos_ned
        self.pos = pos_ned - self.pos_0
        self.vel = vel_ned
        
        self.quat = quaternion_multiply(self.q0_inv, quat) 

        self.acc_body = acc_body
        self.angular_velocity = angular_rate
        self.acc_body_no_g = self.acc_body - quaternion_rotate(self.quat, G)

        self.acc_body_filter.input(self.acc_body, self.dt)
        self.acc_body_no_g_filter.input(self.acc_body_no_g, self.dt)
        self.ang_vel_filter.input(self.angular_velocity, self.dt)