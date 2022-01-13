from RealFlightBridge.realflightbridge import RealFlightBridge
from RealFlightBridge.utils import *
import time
import pygame

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
        self.err_int = float_constrain(self.err_int +err*dt, -self.lim_int, self.lim_int)
        derr = (err-self.err_last)/dt
        self.err_last = err
        return self.p * err + self.i * self.err_int + self.d * derr
    
    def reset(self):
        self.err_int = 0
        self.err_last = None

def simple_test(IP="127.0.0.1"):
    bridge = RealFlightBridge(IP)
    if not bridge.connect():
        raise("Unable to connect to RealFlight, please check connection!")
    bridge.start()

    running = True
    aileron = 0.0
    elevator = 0.0
    collective = 0.1
    rudder = 0.0
    throttle = 0.75

    pid_yaw = PIDController(0.2, 0.1, 0., 0.8)
    pid_roll = PIDController(0.1, 0.02, 0., 0.5)
    pid_pitch = PIDController(0.1, 0.02, 0., 0.5)
    pid_collective = PIDController(0.03, 0.1, 0., 0.2)

    max_ang_vel_yaw = 770/57.3
    max_ang_vel_rp = 330/57.3

    pygame.display.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    JoyAx = pygame.joystick.Joystick(0).get_numaxes()
    print(f"Joy {joystick.get_name()} axis {JoyAx}")

    pygame.event.pump()
    rc_ail0 = joystick.get_axis(0)
    rc_ele0 = joystick.get_axis(1)
    rc_rud0 = joystick.get_axis(2)

    t_last = time.time()
    t0 = time.time()
    bridge.update()
    az_last = 0
    
    while running:
        tnow = time.time()
        t = tnow - t0
        dt = tnow - t_last
        t_last = tnow
        try:
            #Get RC values
            pygame.event.pump()
            rc_ail = joystick.get_axis(0) - rc_ail0
            rc_ele = -(joystick.get_axis(1) - rc_ele0)
            rc_rud = -(joystick.get_axis(2) - rc_rud0)
            rc_thr = - joystick.get_axis(3)

            #Get states
            ang_vel = bridge.get_angular_velocity()
            acc_no_g = bridge.acc_body_no_g
            acc = bridge.acc_body

            #Control
            rudder = -pid_yaw.control((rc_rud*max_ang_vel_yaw-ang_vel[2]), dt)
            aileron = pid_roll.control((rc_ail*max_ang_vel_rp-ang_vel[0]), dt)
            elevator = pid_pitch.control((rc_ele*max_ang_vel_rp-ang_vel[1]), dt)
            
            # aileron = rc_ail
            # elevator = rc_ele
            
            az_cur = -acc_no_g[2]
            az = az_cur *0.05 + az_last*0.95
            az_last = az
            coll_light = - pid_collective.control(az, dt)
            collective = rc_thr  + coll_light

            #Log
            print(f"t {t:.1f} dt {dt*1000:.1f}ms\t\
RC AETR {rc_ail:+01.2f},{rc_ele:+01.2f},{rc_thr:+01.2f},{rc_rud:+01.2f}\t\
CTRL (AERPT) {aileron:+01.2f},{elevator:+01.2f},{rudder:+01.2f},{collective:+01.2f},{throttle:+01.2f}\
coll_light {coll_light:+.2f}\
acc XYZ [{acc[0]:+02.2f},{acc[1]:+02.2f},{acc[2]:+02.2f}]m2/s\
acc_no_g XYZ [{acc_no_g[0]:+02.2f},{acc_no_g[1]:+02.2f},{acc_no_g[2]:+02.2f}]m2/s\
ang_vel RPY[{ang_vel[0]*57.3:03.1f},{ang_vel[1]*57.3:03.1f},{ang_vel[2]*57.3:03.1f}]deg/s", end="\r" )

            bridge.set_controls([aileron, elevator, collective, rudder, throttle, 0.0])
            bridge.update()

        except KeyboardInterrupt:
            print("Exit")
            running = False
            bridge.exit()

if __name__ == "__main__":
    simple_test()