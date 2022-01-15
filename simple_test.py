from RealFlightBridge.realflightbridge import RealFlightBridge
import time

def simple_test(IP="127.0.0.1"):
    bridge = RealFlightBridge(IP)
    if not bridge.connect():
        raise("Unable to connect to RealFlight, please check connection!")
    bridge.start()
    bridge.set_controls([0., 0., 0.0, 0., 0.5])

    running = True
    while running:
        try:
            bridge.update()
        except KeyboardInterrupt:
            print("Exit")
            running = False
            bridge.exit()

if __name__ == "__main__":
    simple_test()