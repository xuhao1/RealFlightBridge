from RealFlightBridge.realflightbridge import RealFlightBridge
from RealFlightBridge.utils import *
import time
import pygame
import socket
import struct
import select
import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QToolTip, QPushButton, QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import  QBasicTimer

DCS_X = 0
DCS_Y = 0
DCS_W = 1920
DCS_H = 1080
DCS_CX = 1000
DCS_CY = 500
DCS_WIN_NAME = "RealFlight 9.5S"
WIN_NAME = "BetaflightBridge"
from win32api import GetSystemMetrics


RC_PWM_MIN = 1100
RC_PWM_MAX = 1900
NUM_RC_CHANNELS = 16

MOTOR_PWM_MIN = 1000
MOTOR_PWM_MAX = 2000
NUM_PWM_OUT = 8

class BetaFlightBridge:
    def __init__(self, IP="127.0.0.1"):
        bridge = RealFlightBridge(IP,freq_cut=250,freq_cut_acc=250)
        if not bridge.connect():
            raise("Unable to connect to RealFlight, please check connection!")
        bridge.start()
        self.bridge = bridge

        pygame.display.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        JoyAx = pygame.joystick.Joystick(0).get_numaxes()
        print(f"Joy {self.joystick.get_name()} axis {JoyAx}")

        bridge.update()

        self.sendto_fc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sendto_fc_rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_fc_output_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.BF_IP = "127.0.0.1"
        self.BF_STATE_PORT = 9003
        self.BF_RC_PORT = 9004
        self.BF_OUTPUT_PORT = 9001

        self.recv_fc_output_sock.bind(("0.0.0.0", self.BF_OUTPUT_PORT))
        self.recv_fc_output_sock.setblocking(0)


    def read_joy_AETR(self):
        pygame.event.pump()
        joystick = self.joystick
        rc_ail = joystick.get_axis(0)
        rc_ele = (joystick.get_axis(1))
        rc_rud = -(joystick.get_axis(3))
        rc_thr = joystick.get_axis(2)
        return rc_ail, rc_ele, rc_thr, rc_rud
    
    def sendRC(self, t):
        pygame.event.pump()
        joystick = self.joystick
        chns = [RC_PWM_MIN for i in range(NUM_RC_CHANNELS)]

        for i in range(joystick.get_numaxes()):
            v = (joystick.get_axis(i) + 1)/2
            pwm = float_constrain(v*(RC_PWM_MAX - RC_PWM_MIN) + RC_PWM_MIN, RC_PWM_MIN, RC_PWM_MAX) 
            chns[i] = math.floor(pwm)

        buf = struct.pack(f'd{NUM_RC_CHANNELS}H', t, *chns)
        self.sendto_fc_rc_sock.sendto(buf, (self.BF_IP, self.BF_RC_PORT))

    def send_status_fc(self, t):
        #Looks like angular, acc, quat are already in NED.
        bridge = self.bridge
        ang_vel = bridge.angular_velocity
        acc_body = bridge.acc_body 
        # quat = [1.0, 0.0, 0.0, 0.0]
        quat = bridge.quat
        RCVT = np.array([[0, -1, 0, 0],
            [-1, 0, 0, 0],
            [0,0,1,0],
            [0,0,0,1]])
        R = quaternion_matrix(quat)
        R_bf = RCVT@R@RCVT.transpose()
        q_bf = quaternion_from_matrix(R_bf)
        
        vel = bridge.vel
        pos = bridge.pos

        bf_ang_vel = [ang_vel[0],ang_vel[1], ang_vel[2]]
        bf_acc_body = [-acc_body[0], acc_body[1], acc_body[2]]
        # bf_acc_body = [0, 0, -9.8]
        # bf_ang_vel = [0.0, 0.0, 0.0]

        buf = struct.pack(f'd3d3d4d3d3d', t, *bf_ang_vel, *bf_acc_body, *q_bf, *vel, *pos)
        self.sendto_fc_sock.sendto(buf, (self.BF_IP, self.BF_STATE_PORT))

    def recv_pwm(self):
        ready = select.select([self.recv_fc_output_sock], [], [], 0.0001)
        if ready[0]:
            data, addr = self.recv_fc_output_sock.recvfrom(1024)
            if addr[0] != self.BF_IP:
                self.BF_IP = addr[0]
                print("Update BetaFlight IP to", self.BF_IP)
            pwms = struct.unpack(f"{NUM_PWM_OUT}f", data)
            pwms = np.array(pwms)
            pwms = 2*(pwms - MOTOR_PWM_MIN) / (MOTOR_PWM_MAX-MOTOR_PWM_MIN) - 1
            self.bridge.set_controls(pwms)

    def update(self):
        t = self.bridge.t
        dt = self.bridge.dt
        bridge = self.bridge
        try:
            #Get RC values
            self.recv_pwm()
            self.send_status_fc(t)
            self.sendRC(t)

            rc_ail, rc_ele, rc_thr, rc_rud = self.read_joy_AETR()

            #Get states
            acc_no_g = bridge.acc_body_no_g_filter.value()
            acc = bridge.acc_body_filter.value()
            ang_vel = bridge.ang_vel_filter.value()
            vel = bridge.vel
            pos = bridge.pos

            chns = (np.array(self.bridge.channels) - 0.5)*2
            status = f"""t {t:.1f}s dt {dt*1000:.1f}ms
RC AETR {rc_ail:+01.2f},{rc_ele:+01.2f},{rc_thr:+01.2f},{rc_rud:+01.2f}
now_ang_vel RPY {ang_vel[0]*57.3:>+5.1f},{ang_vel[1]*57.3:>+5.1f},{ang_vel[2]*57.3:>+5.1f} deg/s
PWM {chns[0]*100:>+5.0f}% {chns[1]*100:>+5.0f}% {chns[2]*100:>+5.0f}% {chns[3]*100:>+5.0f}% {chns[4]*100:>+5.0f}%
pos XYZ {pos[0]:>+5.1f},{pos[1]:>+5.1f},{pos[2]:>+5.1f} m
vel XYZ {vel[0]:>+5.1f},{vel[1]:>+5.1f},{vel[2]:>+5.1f} mps
Att YPR {bridge.yaw*57.3:>+5.1f},{bridge.pitch*57.3:>+5.1f},{bridge.roll*57.3:>+5.1f} deg
Att Q {bridge.quat[0]:>+5.1f},{bridge.quat[1]:>+5.1f},{bridge.quat[2]:>+5.1f},{bridge.quat[3]:>+5.1f}
acc XYZ {acc[0]/9.8:>+5.1f},{acc[1]/9.8:>+5.1f},{acc[2]/9.8:>+5.1f} g
acc_no_g XYZ [{acc_no_g[0]/9.8:>+5.1f},{acc_no_g[1]/9.8:>+5.1f},{acc_no_g[2]/9.8:>+5.1f}]g"""
            
            bridge.update()

        except KeyboardInterrupt:
            print("Exit")
            bridge.exit()
        return status
    def set_controls(self, controls):
        self.bridge.set_controls(controls)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, RF_IP="127.0.0.1", parent=None):
        import pathlib
        path = pathlib.Path(__file__).parent.absolute()
        print(path)
        super(MainWindow, self).__init__(parent)
        self.setWindowFlags(
            QtCore.Qt.WindowStaysOnTopHint |
            QtCore.Qt.Window |
            QtCore.Qt.WindowTitleHint |
            QtCore.Qt.FramelessWindowHint
        )
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.setStyleSheet("background:transparent;")
        
        self.setWindowTitle(WIN_NAME)

        self.status = QLabel('Wait for RF9 Connection', self)
        self.status.move(30, 30)
        self.status.setFont(QFont('Consolas', 15))
        self.status.setFixedSize(DCS_W, 500)
        self.status.setStyleSheet('color: yellow')
        
        self.setGeometry(DCS_X, DCS_Y, DCS_W, DCS_H)

        self.betaflight = BetaFlightBridge(RF_IP)
        self.OK = True

        self.timer_count = 0
        self.timer = QBasicTimer()
        self.timer.start(0.001, self)
        
    def timerEvent(self, e):
        if self.timer_count % 10 == 0:
            if not self.OK:
                self.status.setText(f"Wait for RealFlight")
                self.status.setStyleSheet('color: yellow')
            else:
                self.status.setStyleSheet('color: black')

        status = self.betaflight.update()

        if self.timer_count %100 == 0:
            print(f"{self.timer_count}: {status}\n")
        self.status.setText(status)
        self.timer_count += 1

import win32gui
def callback(hwnd, extra):
    rect = win32gui.GetWindowRect(hwnd)
    x = rect[0]
    y = rect[1]
    w = rect[2] - x
    h = rect[3] - y

    if w == 0:
        w = GetSystemMetrics(0)
        h = GetSystemMetrics(1)

    global DCS_X, DCS_Y, DCS_W, DCS_H, DCS_CX, DCS_CY

    if win32gui.GetWindowText(hwnd) == DCS_WIN_NAME:
        DCS_X = x
        DCS_Y = y
        DCS_W = w
        DCS_H = h
        DCS_CX = x + w//2
        DCS_CY = y + h//2

        print("Window %s:" % win32gui.GetWindowText(hwnd))
        print("\tLocation: (%d, %d)" % (x, y))
        print("\t    Size: (%d, %d)" % (w, h))
        print("\t    Center: (%d, %d)" % (DCS_CX, DCS_CY))

if __name__ == "__main__":
    import sys
    # simple_test()
    app = QtWidgets.QApplication(sys.argv)
    win32gui.EnumWindows(callback, None)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())