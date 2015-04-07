def parseRoombaSensors(sensorList):
##    Sensors
##
##    
##    #Bumps and Wheeldrops 0-31
##    bumps_wheeldrops
##    bump_right
##    bump_left
##    wheeldrop_right
##    wheeldrop_left
##    wheeldrop_caster
##
##    wall
##    cliff_left
##    cliff_front_left
##    cliff_front_right
##    cliff_right
##    virtual_wall
##
##    #Motor Overcurrents 0-31
##    motor_overcurrents
##    side_brush
##    vacuum
##    main_brush
##    drive_right
##    drive_left
##
##
##    dirt_detector_left #255
##    dirt_detector_right #255
##    remote_control_command #255
##
##
##    #Buttons 0-15
##    buttons
##    max_button
##    clean
##    spot
##    power
##
##    distance1 #-32768-32767 signed 2 bytes
##    distance2
##    angle1 #-32768-32767 signed 2 bytes
##    angle2
##    charging_state #0-5
##    voltage1 #0-65535 2 bytes
##    voltage2
##    current1 #-32768-32767 signed 2 bytes
##    current2
##    temperature #-128-127 signed
##    charge1 #0-65535 2 bytes
##    charge2
##    capacity1 #0-65535 2 bytes
##    capacity2
    
    sensors = ["bumps_wheeldrops", "wall", "cliff_left",
               "cliff_front_left", "cliff_front_right",
               "cliff_right", "virtual_wall", "motor_overcurrents",
               "dirt_detector_left", "dirt_detector_right",
               "remote_control_command", "buttons", "distance1", "distance2",
               "angle1", "angle2", "charging_state", "voltage1", "voltage2",
               "current1", "current2", "temperature", "charge1", "charge2",
               "capacity1", "capacity2"]
    sensorMapping = {}
    for i in range(26):
        sensorMapping[sensors[i]] = ord(sensorList[i])
    
    bumps_wheeldrops = '{0:05b}'.format(sensorMapping[sensors[0]])
    bump_right = int(bumps_wheeldrops[4])
    bump_left = int(bumps_wheeldrops[3])
    wheeldrop_right = int(bumps_wheeldrops[2])
    wheeldrop_left = int(bumps_wheeldrops[1])
    wheeldrop_caster = int(bumps_wheeldrops[0])

    
    motor_overcurrents = '{0:05b}'.format(sensorMapping[sensors[7]])
    side_brush = int(motor_overcurrents[4])
    vacuum = int(motor_overcurrents[3])
    main_brush = int(motor_overcurrents[2])
    drive_right = int(motor_overcurrents[1])
    drive_left = int(motor_overcurrents[0])

    
    buttons = '{0:04b}'.format(sensorMapping[sensors[0]])
    max_button = int(buttons[3])
    clean = int(buttons[2])
    spot = int(buttons[1])
    power = int(buttons[0])

    parsedSensorList = ["bump_right", "bump_left", "wheeldrop_right",
                      "wheeldrop_left", "wheeldrop_caster", "wall, cliff_left",
                      "cliff_front_left", "cliff_front_right", "cliff_right",
                      "virtual_wall", "side_brush", "vacuum", "main_brush",
                      "drive_right", "drive_left", "dirt_detector_left",
                      "dirt_detector_right", "remote_control_command", "max_button",
                      "clean", "spot", "power", "distance1", "distance2",
                      "angle1", "angle2", "charging_state", "voltage1", "voltage2",
                      "current1", "current2", "temperature", "charge1", "charge2",
                      "capacity1", "capacity2"]
    
    parsedSensors = [bump_right, bump_left, wheeldrop_right,
                      wheeldrop_left, wheeldrop_caster, sensorMapping["wall"],
                      sensorMapping["cliff_left"], sensorMapping["cliff_front_left"],
                      sensorMapping["cliff_front_right"], sensorMapping["cliff_right"],
                      sensorMapping["virtual_wall"], side_brush, vacuum, main_brush,
                      drive_right, drive_left, sensorMapping["dirt_detector_left"],
                      sensorMapping["dirt_detector_right"],
                      sensorMapping["remote_control_command"], max_button,
                      clean, spot, power, sensorMapping["distance1"],
                      sensorMapping["distance2"], sensorMapping["angle1"],
                      sensorMapping["angle2"], sensorMapping["charging_state"],
                      sensorMapping["voltage1"], sensorMapping["voltage2"],
                      sensorMapping["current1"], sensorMapping["current2"],
                      sensorMapping["temperature"], sensorMapping["charge1"],
                      sensorMapping["charge2"], sensorMapping["capacity1"],
                      sensorMapping["capacity2"]]
    parsedSensorMapping = {}
    for i in range(len(parsedSensorList)):
        parsedSensorMapping[parsedSensorList[i]] = parsedSensors[i] 
    return parsedSensorMapping




from serial import Serial
import time

port = Serial("/dev/ttyUSB0", 19200)
time.sleep(4)
port.setDTR(level=0)
time.sleep(4)
port.write("\x00\x8E\x00\x00")
#port.flush()
#time.sleep(1)
#while True:
#    print port.inWaiting()
#print port.inWaiting()
#print ord(port.read())
sensors = []
while(port.inWaiting() < 25):
    #sensors.append(ord(port.read(26)))
    a = 0
for i in range(26):
    sensors.append(port.read())
#sensors.append(port.read(26))
#port.close()
print sensors


#port.write("\x00\x8E\x00\x00")
#port.flush()
#time.sleep(1)
#while True:
#    print port.inWaiting()
#print port.inWaiting()
#print ord(port.read())
#sensors = []
#while(port.inWaiting() < 25):
#    a = 0
    #sensors.append(ord(port.read(26)))
#sensors.append(port.read(26))
#port.close()
#print sensors
print parseRoombaSensors(sensors)

