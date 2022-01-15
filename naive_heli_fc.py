from RealFlightBridge.realflightbridge import RealFlightBridge
from RealFlightBridge.utils import *
import time
import pygame

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
WIN_NAME = "NaiveHeliGyro"
from win32api import GetSystemMetrics


class PIDController:
    def __init__(self, p, i, d, lim_int = 1.0):
        self.p = p
        self.i = i
        self.d = d
        self.err_int = 0
        self.err_last = None
        if i > 0:
            self.lim_int = lim_int/i
        else:
            self.lim_int = 0

    def control(self, err, dt):
        if self.err_last is None:
            self.err_last = err
        if dt <=0.002:
            dt = 0.002
        self.err_int = float_constrain(self.err_int +err*dt, -self.lim_int, self.lim_int)
        derr = (err-self.err_last)/dt
        if math.isnan(derr):
            derr = 0
        self.err_last = err
        return self.p * err + self.i * self.err_int + self.d * derr
    
    def reset(self):
        self.err_int = 0
        self.err_last = None

class PIDFController:
    def __init__(self, p, i, d, f, lim_int = 1.0):
        self.p = p
        self.i = i
        self.d = d
        self.f = f
        self.err_int = 0
        self.err_last = None
        if i > 0:
            self.lim_int = lim_int/i
        else:
            self.lim_int = 0

    def control(self, r, y, dt):
        err = r - y
        if self.err_last is None:
            self.err_last = err
        if dt <=0.002:
            dt = 0.002
        self.err_int = float_constrain(self.err_int +err*dt, -self.lim_int, self.lim_int)
        derr = (err-self.err_last)/dt
        if math.isnan(derr):
            derr = 0
        self.err_last = err
        return self.p * err + self.i * self.err_int + self.d * derr + self.f*r
    
    def reset(self):
        self.err_int = 0
        self.err_last = None


class NaiveHeliGyro:
    def __init__(self, IP="127.0.0.1"):
        bridge = RealFlightBridge(IP)
        if not bridge.connect():
            raise("Unable to connect to RealFlight, please check connection!")
        bridge.start()
        self.bridge = bridge


        self.pid_yaw = PIDController(0.15, 0.3, 0., 0.8)
        self.pid_roll = PIDFController(0.12, 0.05, 0., 0.3, 0.1)
        self.pid_pitch = PIDFController(0.12, 0.05, 0.0, 0.3, 0.1)
        self.pid_collective = PIDFController(0.015, 0.01, 30, 0.2)

        self.max_ang_vel_yaw = 660/57.3
        self.max_ang_vel_rp = 140/57.3
        self.max_acc_b = 30

        pygame.display.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        JoyAx = pygame.joystick.Joystick(0).get_numaxes()
        print(f"Joy {self.joystick.get_name()} axis {JoyAx}")

        pygame.event.pump()
        self.rc_ail0, self.rc_ele0, _, self.rc_rud0 = self.read_joy_AETR_raw()

        self.rpm_up_t = 10
        self.rpm_up_t0 = -1

        bridge.update()

    def read_joy_AETR_raw(self):
        pygame.event.pump()
        joystick = self.joystick
        rc_ail = joystick.get_axis(0)
        rc_ele = (joystick.get_axis(1))
        rc_rud = (joystick.get_axis(3))
        rc_thr = joystick.get_axis(2)
        return rc_ail, rc_ele, rc_thr, rc_rud
    
    def reset_controllers(self):
        self.pid_yaw.reset()
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_collective.reset()

    def read_joy_AETR(self):
        rc_ail, rc_ele, rc_thr, rc_rud = self.read_joy_AETR_raw()
        if math.fabs(rc_rud) < 0.02:
            rc_rud = 0

        if math.fabs(rc_ail) < 0.02:
            rc_ail = 0

        if math.fabs(rc_ele) < 0.02:
            rc_ele = 0
            
        return rc_ail, rc_ele, rc_thr, rc_rud
    
    def read_arm(self):
        return self.joystick.get_axis(4) > 0.75

    def update(self):
        aileron = 0.0
        elevator = 0.0
        collective = 0.1
        rudder = 0.0
        throttle_sp = 0.75

        t = self.bridge.t
        dt = self.bridge.dt
        bridge = self.bridge
        try:
            #Get RC values

            rc_ail, rc_ele, rc_thr, rc_rud = self.read_joy_AETR()

            #Get states
            ang_vel = bridge.ang_vel_filter.value()
            acc_no_g = bridge.acc_body_no_g_filter.value()
            acc = bridge.acc_body_filter.value()
            
            vel = bridge.vel
            pos = bridge.pos

            #Control
            tgt_roll_spd = rc_ail*self.max_ang_vel_rp
            tgt_pitch_spd = rc_ele*-self.max_ang_vel_rp
            tgt_yaw_spd = rc_rud*self.max_ang_vel_yaw

            rudder = self.pid_yaw.control(tgt_yaw_spd-ang_vel[2], dt)
            aileron = self.pid_roll.control(tgt_roll_spd, ang_vel[0], dt)
            elevator = self.pid_pitch.control(tgt_pitch_spd, ang_vel[1], dt)

            # rudder = rc_rud
            # aileron = rc_ail
            # elevator = rc_ele
            # az_cur = -acc_no_g[2]
            # coll_light = 
            tgt_acc_b = rc_thr * self.max_acc_b
            # collective = self.pid_collective.control(tgt_acc_b, -acc_no_g[2], dt)
            collective = rc_thr

            if self.read_arm():
                if self.rpm_up_t0 < 0:
                    self.rpm_up_t0 = t
                if t < self.rpm_up_t + self.rpm_up_t0:
                    throttle = (throttle_sp)*(t-self.rpm_up_t0)/self.rpm_up_t
                    throttle = (throttle -0.5)*2
                else:
                    throttle = (throttle_sp -0.5)*2
            else:
                throttle = -1
                self.reset_controllers()

            #Log
            chns = (np.array(self.bridge.channels) - 0.5)*2
            status = f"""t {t:.1f}s dt {dt*1000:.1f}ms
RC AETR {rc_ail:+01.2f},{rc_ele:+01.2f},{rc_thr:+01.2f},{rc_rud:+01.2f}
CTRL (AERPT) {aileron:+01.2f},{elevator:+01.2f},{rudder:+01.2f},{collective:+01.2f},{throttle:+01.2f}
tgt_ang_vel RPY {tgt_roll_spd*57.3:>+5.1f},{tgt_pitch_spd*57.3:>+5.1f},{tgt_yaw_spd*57.3:>+5.1f} deg/s
now_ang_vel RPY {ang_vel[0]*57.3:>+5.1f},{ang_vel[1]*57.3:>+5.1f},{ang_vel[2]*57.3:>+5.1f} deg/s
err_ang_vel RPY {(tgt_roll_spd - ang_vel[0])*57.3:>+5.1f},{(tgt_pitch_spd-ang_vel[1])*57.3:>+5.1f},{(tgt_yaw_spd-ang_vel[2])*57.3:>+5.1f} deg/s
CTRLS Ail {chns[0]*100:>+5.0f}% Ele {chns[1]*100:>+5.0f}% Coll {chns[2]*100:>+5.0f}% Rud {chns[3]*100:>+5.0f}% Thro {chns[4]*100:>+5.0f}%
tgt_acc_b {tgt_acc_b/9.8:>+5.1f} cur {acc_no_g[2]/9.8:>+5.1f} g
pos XYZ {pos[0]:>+5.1f},{pos[1]:>+5.1f},{pos[2]:>+5.1f} m
vel XYZ {vel[0]:>+5.1f},{vel[1]:>+5.1f},{vel[2]:>+5.1f} mps
acc XYZ {acc[0]/9.8:>+5.1f},{acc[1]/9.8:>+5.1f},{acc[2]/9.8:>+5.1f} g
acc_no_g XYZ [{acc_no_g[0]/9.8:>+5.1f},{acc_no_g[1]/9.8:>+5.1f},{acc_no_g[2]/9.8:>+5.1f}]g"""
            
            bridge.set_controls([aileron, elevator, collective, rudder, throttle, 0.0])
            bridge.update()

        except KeyboardInterrupt:
            print("Exit")
            bridge.exit()
        return status
        
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
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

        self.status = QLabel('Wait for DCS Connection', self)
        self.status.move(30, 30)
        self.status.setFont(QFont('Consolas', 15))
        self.status.setFixedSize(DCS_W, 500)
        self.status.setStyleSheet('color: yellow')
        
        self.setGeometry(DCS_X, DCS_Y, DCS_W, DCS_H)

        self.gyro = NaiveHeliGyro()
        self.OK = True

        self.timer_count = 0
        self.timer = QBasicTimer()
        self.timer.start(0.001, self)
        
    def timerEvent(self, e):
        if self.timer_count % 10 == 0:
            top_win = win32gui.GetWindowText(win32gui.GetForegroundWindow())
            if not self.OK:
                self.status.setText(f"Wait for RealFlight")
                self.status.setStyleSheet('color: yellow')
            else:
                self.status.setStyleSheet('color: black')

            # if top_win != DCS_WIN_NAME and top_win != WIN_NAME:
            #     self.setVisible(False)
            # else:
            #     self.setVisible(True)

        status = self.gyro.update()

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