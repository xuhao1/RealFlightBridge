# FlighAxis
The protocol for RealFlight is called FlightAxis Link.

## FlightAxis Link Q&A
A good introduction to FlightAxis link is from [forum](https://www.knifeedge.com/forums/index.php?threads/flightaxis-link-q-a.32854/)

### How do I connect my Flight Controller?
The FlightAxis toolkit enables Hardware- and Software-in-the-Loop testing for flight controllers.
The toolkit allows RealFlight to communicate using SOAP. Using the ExchangeData SOAP method, the flight controller sends a control vector to RealFlight. In response, RealFlight sends a state vector. After processing, the controller sends an updated control vector and receives a state update, forming a feedback loop as this cycle repeats. A flowchart is provided at the end of this document.

### What state data is sent?
The state data sent by RealFlight includes:
- Simulation time (s)
- Airspeed (m/s)
- Altitude, ASL (m)
- Altitude, AGL (m)
- Ground Speed, (m/s)
- Pitch, Roll, Yaw rates (deg/s)
- Orientation (Azimuth, Inclination, Roll; deg)
- Orientation (quaternion)
- Position (x, y, z; m) - can be converted to Latitude and Longitude
- Velocity, world-referenced (x,y,z; m/s)
- Velocity, body-referenced (x,y,z; m/s)
- Acceleration, world-referenced (x,y,z; m/s^2)
- Acceleration, body-referenced (x,y,z; m/s^2)
- Wind velocity (x,y,z; m/s)
- Propeller RPM
- Main Rotor RPM
- Battery Voltage (V)
- Battery Current (A)
- Remaining Battery Capacity (mAh)
- Fuel Remaining (oz)
- Is Locked (bool)
- Has Lost Components (bool)
- Engine is Running (bool)
- Is Touching Ground (bool)
- Current Status

### How do I send control data?
The plugin accepts up to 8 channels of servo input, scaled from -1 to 1.
Additional SOAP methods are included to:
- Reset the simulation
- Allow external input
- Return manual control to the RealFlight transmitter

### What kind of performance can I expect?
RealFlight’s performance will be dependent upon the system used to run it, and the communication frame rate corresponds to the overall performance. On an i7-equipped desktop with a mid-range graphics card, we can reliably operate at 200Hz or better.

### Can I simulate my own vehicle?
Yes! Creating a model for RealFlight has two stages: visuals and physics. The visual model is built using our KEMax toolchain (http://www.knifeedge.com/KEmax/). Vehicle physics are defined using RealFlight's internal Vehicle Editor. By specifying the geometry, mass, etc. of the various aircraft components, you create a predictive model of the vehicle’s behavior.

### Can Knife Edge Software help me make a simulated vehicle?
Absolutely! We can provide consulting services to assist you in learning this process, or even to create your vehicle for you.

### Do I need flight test or wind tunnel data?
No - the FlightAxis physics system is predictive, and simulates each component of the vehicle in every simulation cycle. We find that a simulated model made from careful measurements will match the flight characteristics of the physical model.

### Can the vehicle change configuration in flight?
Yes. Tilt-rotor, tilt-wing, and other variable geometries are handled gracefully by the FlightAxis component system. Since each component is simulated in real time, any configuration changes are automatically included in physics calculations.

### I have my own Flight Dynamics Model. Can I use RealFlight as a visualization engine only?
Not at this time. In the current generation, the FDM and visualization are inseparable. Future versions may be capable of this, but we do not have a release date at this time.

### Can I use RealFlight for visualization of telemetry logs?
Not directly (see the question above). However, RealFlight can record simulated flights internally, allowing playback within the simulator. This will not be synchronized with your telemetry data, unfortunately.

### Can I use my own terrain?
Not in the current version. RealFlight uses real terrain data from the Sierra Nevada region in Southern Spain. This makes it possible to test terrain-dependent software, just not at your home range (unless you happen to be operating within that region).

### Hardware/Software-in-the-Loop flowchart
The following flowchart shows the basic structure for HIL/SITL testing with an external controller. These are the steps you will need to incorporate into your software in order to test against the RealFlight FDM and terrain.


### Is there an existing implementation?
Yes. The ArduPilot project has a standing implementation that works well. Please see tridge's post here for links to the code and a sample packet.

### Is there formal documentation?
We will provide additional documentation here as soon as possible.

## Protocol
Current implementation of Flightaxis data exchange is in C++ [header](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_FlightAxis.h), [source](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_FlightAxis.cpp) and [Python](https://github.com/camdeno/F16Capstone/blob/main/FlightAxis/flightaxis.py)

### Network Port
IP: Localhost (of course)
Port: 18083/TCP
 
### SOAP request
Code to send SOAP request in C++.
```c++
//Code from https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_FlightAxis.cpp
bool FlightAxis::soap_request_start(const char *action, const char *fmt, ...)
{
    ...
    char *req;
    asprintf(&req, R"(POST / HTTP/1.1
soapaction: '%s'
content-length: %u
content-type: text/xml;charset='UTF-8'
Connection: Keep-Alive
%s)",
             action,
             (unsigned)strlen(req1), req1);
             
    sock->send(req, strlen(req));
    return true;
}
```

Start the simulation by calling:
```c++
//Code from https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_FlightAxis.cpp
soap_request_start("RestoreOriginalControllerDevice", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>
</soap:Body>
</soap:Envelope>)");
...
 soap_request_start("InjectUAVControllerInterface", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
</soap:Body>
</soap:Envelope>)");
```

Exchange the data by calling

```c++
//Code from https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_FlightAxis.cpp
soap_request_start("ExchangeData", R"(<?xml version='1.0' encoding='UTF-8'?><soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<ExchangeData>
<pControlInputs>
<m-selectedChannels>%u</m-selectedChannels>
<m-channelValues-0to1>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
</m-channelValues-0to1>
</pControlInputs>
</ExchangeData>
</soap:Body>
</soap:Envelope>)",
    channels,
    scaled_servos[0],
    scaled_servos[1],
    ...
    scaled_servos[11]);
    }
```


A typical flightaxis data exchange file is in [XML](http://uav.tridgell.net/RealFlight/data-exchange.txt)
```
<?xml version="1.0" encoding="UTF-8"?>
<SOAP-ENV:Envelope xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema"><SOAP-ENV:Body><ReturnData><m-previousInputsState><m-selectedChannels>255</m-selectedChannels>
<m-channelValues-0to1 xsi:type="SOAP-ENC:Array" SOAP-ENC:arrayType="xsd:double[8]">
<item>0.5</item>
<item>0.5</item>
<item>0</item>
<item>0.5</item>
<item>0.81113320589065552</item>
<item>0</item>
<item>0</item>
<item>0</item>
</m-channelValues-0to1></m-previousInputsState>
<m-aircraftState>
<m-currentPhysicsTime-SEC>63.766650660196319</m-currentPhysicsTime-SEC>
<m-currentPhysicsSpeedMultiplier>1</m-currentPhysicsSpeedMultiplier>
<m-airspeed-MPS>0.00032659839781893734</m-airspeed-MPS>
<m-altitudeASL-MTR>1298.1253044187715</m-altitudeASL-MTR>
<m-altitudeAGL-MTR>0.21455790619549658</m-altitudeAGL-MTR>
<m-groundspeed-MPS>0.00031040600969726772</m-groundspeed-MPS>
<m-pitchRate-DEGpSEC>0.04102806363749778</m-pitchRate-DEGpSEC>
<m-rollRate-DEGpSEC>0.071367648873049916</m-rollRate-DEGpSEC>
<m-yawRate-DEGpSEC>-0.0094135984837091513</m-yawRate-DEGpSEC>
<m-azimuth-DEG>-90.099983215332031</m-azimuth-DEG>
<m-inclination-DEG>-0.061271317303180695</m-inclination-DEG>
<m-roll-DEG>0.56763416528701782</m-roll-DEG>
<m-orientationQuaternion-X>0.0031279732938855886</m-orientationQuaternion-X>
<m-orientationQuaternion-Y>0.0038780211471021175</m-orientationQuaternion-Y>
<m-orientationQuaternion-Z>-0.70771658420562744</m-orientationQuaternion-Z>
<m-orientationQuaternion-W>0.70647883415222168</m-orientationQuaternion-W>
<m-aircraftPositionX-MTR>23180.099609375</m-aircraftPositionX-MTR>
<m-aircraftPositionY-MTR>-9183.7568359375</m-aircraftPositionY-MTR>
<m-velocityWorldU-MPS>0.00029931304743513465</m-velocityWorldU-MPS>
<m-velocityWorldV-MPS>8.2241051131859422E-005</m-velocityWorldV-MPS>
<m-velocityWorldW-MPS>-0.00010156093048863113</m-velocityWorldW-MPS>
<m-velocityBodyU-MPS>8.1609920016489923E-005</m-velocityBodyU-MPS>
<m-velocityBodyV-MPS>-0.00030044838786125183</m-velocityBodyV-MPS>
<m-velocityBodyW-MPS>-9.8676573543343693E-005</m-velocityBodyW-MPS>
<m-accelerationWorldAX-MPS2>0.10442006587982178</m-accelerationWorldAX-MPS2>
<m-accelerationWorldAY-MPS2>0.025319233536720276</m-accelerationWorldAY-MPS2>
<m-accelerationWorldAZ-MPS2>1.5337128639221191</m-accelerationWorldAZ-MPS2>
<m-accelerationBodyAX-MPS2>0.026950166560709476</m-accelerationBodyAX-MPS2>
<m-accelerationBodyAY-MPS2>-0.18779043108224869</m-accelerationBodyAY-MPS2>
<m-accelerationBodyAZ-MPS2>-9.8204746246337891</m-accelerationBodyAZ-MPS2>
<m-windX-MPS>0</m-windX-MPS>
<m-windY-MPS>0</m-windY-MPS>
<m-windZ-MPS>0</m-windZ-MPS>
<m-propRPM>0</m-propRPM>
<m-heliMainRotorRPM>-1</m-heliMainRotorRPM>
<m-batteryVoltage-VOLTS>20.999999642372131</m-batteryVoltage-VOLTS>
<m-batteryCurrentDraw-AMPS>0</m-batteryCurrentDraw-AMPS>
<m-batteryRemainingCapacity-MAH>1000</m-batteryRemainingCapacity-MAH>
<m-fuelRemaining-OZ>-1</m-fuelRemaining-OZ>
<m-isLocked>false</m-isLocked>
<m-hasLostComponents>false</m-hasLostComponents>
<m-anEngineIsRunning>true</m-anEngineIsRunning>
<m-isTouchingGround>true</m-isTouchingGround>
<m-flightAxisControllerIsActive>true</m-flightAxisControllerIsActive>
<m-currentAircraftStatus>CAS-FLYING</m-currentAircraftStatus></m-aircraftState>
<m-notifications>
<m-resetButtonHasBeenPressed>false</m-resetButtonHasBeenPressed></m-notifications></ReturnData></SOAP-ENV:Body></SOAP-ENV:Envelope>HTTP/1.1 200 OK
```