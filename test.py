def parseRoombaSensors(sensorList):

    #Bumps and Wheeldrops 0-31
    bumps_wheeldrops = 0
    bump_right = 0
    bump_left = 0
    wheeldrop_right = 0
    wheeldrop_left = 0
    wheeldrop_caster = 0

    wall = 0
    cliff_left = 0
    cliff_front_left = 0
    cliff_front_right = 0
    cliff_right = 0
    virtual_wall = 0

    #Motor Overcurrents 0-31
    motor_overcurrents = 0
    side_brush = 0
    vacuum = 0
    main_brush = 0
    drive_right = 0
    drive_left = 0


    dirt_detector_left = 0 #255
    dirt_detector_right = 0 #255
    remote_control_command = 0 #255


    #Buttons 0-15
    buttons = 0
    max_button = 0
    clean = 0
    spot = 0
    power = 0

    distance1 = 0 #-32768-32767 signed 2 bytes
    distance2 = 0
    angle1 = 0 #-32768-32767 signed 2 bytes
    angle2 = 0
    charging_state = 0 #0-5
    voltage1 = 0 #0-65535 2 bytes
    voltage2 = 0
    current1 = 0 #-32768-32767 signed 2 bytes
    current2 = 0
    temperature = 0 #-128-127 signed
    charge1 = 0 #0-65535 2 bytes
    charge2 = 0
    capacity1 = 0 #0-65535 2 bytes
    capacity2 = 0
    sensors = [bumps_wheeldrops, wall, cliff_left,
               cliff_front_left, cliff_front_right,
               cliff_right, virtual_wall, motor_overcurrents,
               dirt_detector_left, dirt_detector_right,
               remote_control_command, buttons, distance1, distance2,
               angle1, angle2, charging_state, voltage1, voltage2,
               current1, current2, temperature, charge1, charge2,
               capacity1, capacity2]
    for i in range(len(sensorList)):
        sensors[i] = ord(sensorList[i])
    
    bumps_wheeldrops = '{0:05b}'.format(sensors[0])
    bump_right = int(bumps_wheeldrops[4])
    bump_left = int(bumps_wheeldrops[3])
    wheeldrop_right = int(bumps_wheeldrops[2])
    wheeldrop_left = int(bumps_wheeldrops[1])
    wheeldrop_caster = int(bumps_wheeldrops[0])

    
    motor_overcurrents = '{0:05b}'.format(sensors[7])
    side_brush = int(motor_overcurrents[4])
    vacuum = int(motor_overcurrents[3])
    main_brush = int(motor_overcurrents[2])
    drive_right = int(motor_overcurrents[1])
    drive_left = int(motor_overcurrents[0])

    
    buttons = '{0:04b}'.format(sensors[0])
    max_button = int(buttons[3])
    clean = int(buttons[2])
    spot = int(buttons[1])
    power = int(buttons[0])

    parsed_sensors = [bump_right, bump_left, wheeldrop_right,
                      wheeldrop_left, wheeldrop_caster, wall, cliff_left,
                      cliff_front_left, cliff_front_right, cliff_right,
                      virtual_wall, side_brush, vacuum, main_brush,
                      drive_right, drive_left, dirt_detector_left,
                      dirt_detector_right, remote_control_command, max_button,
                      clean, spot, power, distance1, distance2,
                      angle1, angle2, charging_state, voltage1, voltage2,
                      current1, current2, temperature, charge1, charge2,
                      capacity1, capacity2]
    return sensors




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

