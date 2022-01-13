from RealflightBridge.realflightbridge import RealflightBridge

def simple_test(IP="127.0.0.1"):
    bridge = RealflightBridge(IP)
    if not bridge.connect():
        raise("Unable to connect to RealFlight, please check connection!")
    bridge.start()
    bridge.set_control([1.0, 1.0, 1.0, 1.0, 1.0])

if __name__ == "__main__":
    simple_test()