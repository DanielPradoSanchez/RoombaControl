
import serial
from serial import Serial
from struct import pack
from struct import unpack
from time import sleep
from threading import Thread
from threading import Lock
from datetime import datetime
import time
#Can be Downloaded from this Link
#https://pypi.python.org/pypi/pyserial

#Global Variables
#ser = 0

#Function to Initialize the Serial Port
class Create:
    def __init__(self, tty):
        """constructor for the Create, takes in a single argument: the serial port"""
        self.timeout = 5
        self.period = .07
        self.runRef = []
        self.packetRef = []
        #self.queueLock = Lock()
        self.queue = []
        #self.distanceLock = Lock()
        self.__distance = 0
        #self.angleLock = Lock()
        self.__angle = 0
        self.port = Serial(port = tty, baudrate = 19200, timeout= self.timeout,
                           rtscts = False,
                           dsrdtr = False)
        self.port.flush()
        time.sleep(1)
        #self.portLock = Lock()
        #self.update = lambda : ''
       # self.bytes = 
        #self.sendNow(128,128,132,135)
        #self.reset()

    
    def sendNow(self, *opcodes):
        #self.portLock.acquire()
        format = "B"*len(opcodes)   
        data = pack(format, *opcodes)
        print(opcodes)
        print(format)
        print(data)
        print(self.port.write(data))
        self.port.write(data)
        #self.portLock.release()
        
    def send(self, data):
        self.port.write(data)

    def __convert(self, num):
        return self.__highLow(self.__twos(num))

    def __highLow(self, num):
        return num >> 8, num & 0xFF

    def __twos(self, num, bits=16):
        if (num >=0):
            return num
        return 2**bits+num

    def __read(self):
        #bytes = self.port.read(self.port.inWaiting())
        print(self.port.read(self.port.inWaiting()))
        #return bytes
    
    def run(self):
        #while True:
        self.port.write("\x80\x83")
        #self.port.write("\x89\xFF\x38\x01\xF4")
        #time.sleep(1)
        #self.port.write("\x89\x01\x00\x00\x00")
        time.sleep(1)
        self.port.write("\x89\x00\x90\xFF\xFF")
        time.sleep(3)
        self.port.write("\x80")

        sensors = self.__read()
        print sensors

    def square(self):
        self.port.write("\x80\x83")
        time.sleep(1)
        self.port.write("\x89\x01\xF4\x00\x00")
        time.sleep(1)
        self.port.write("\x89\x00\x50\xFF\xFF")
        time.sleep(2)
        self.port.write("\x89\x01\xF4\x00\x00")
        time.sleep(1)
        self.port.write("\x89\x00\x50\xFF\xFF")
        time.sleep(2)
        self.port.write("\x89\x01\xF4\x00\x00")
        time.sleep(1)
        self.port.write("\x89\x00\x50\xFF\xFF")
        time.sleep(2)
        self.port.write("\x89\x01\xF4\x00\x00")
        time.sleep(1)
        self.port.write("\x89\x00\x50\xFF\xFF")
        time.sleep(2)
        self.port.write("\x80")

    def circle(self):
        self.port.write("\x80\x83")
        time.sleep(1)
        self.port.write("\x89\x01\xF4\x01\x00")
        time.sleep(8)
        self.port.write("\x80")


    def spin(self, spinTime, speed, clockwise):
        self.port.write(bytes(128))
        self.port.flush()
        time.sleep(1)
        self.port.write(bytes(131))
        self.port.flush()
        time.sleep(1)
        if clockwise:
            #self.port.write("\x89\x01\xF4\xFF\xFF")
            self.port.write(bytes(137))
            self.port.write(bytes(200))
            self.port.write(bytes(1))
            self.port.flush()

            time.sleep(spinTime)
            self.port.write(bytes(128))
            self.port.flush()

        else:
            self.port.write("\x89\x01\xF4\x00\x01")
            self.port.flush()
            time.sleep(spinTime)
            self.port.write("\x80")
            self.port.flush()

    def seekDock(self):
        self.port.write("\x80\x00\x8F")

    def readSensors(self):
        sensors = ""
        boolean = True
        read = self.port.read()
        counter = 0
        self.port.write("\x00\x8E\x00\x00")
        #time.sleep(1)
        #read = self.port.readlines()
        #print read
        ser = self.port 
        buffer = []
        while 1:
            val = ser.readline()
            buffer.append(val) # stores all data received into a list
            print(val)  
            print buffer

        return read

#def init_serial():
#    COMNUM = 1          #Enter Your COM Port Number Here.
#    global ser          #Must be declared in Each Function
#    ser = serial.Serial()
#    ser.baudrate = 19200
    #ser.port = COMNUM - 1   #COM Port Name Start from 0
    
#    ser.port = '/dev/ttyUSB0' #If Using Linux

    #Specify the TimeOut in seconds, so that SerialPort
    #Doesn't hangs
#    ser.timeout = 10
#    ser.open()          #Opens SerialPort

    # print port open or closed
#    if ser.isOpen():
#        print 'Open: ' + ser.portstr
#Function Ends Here
        

#Call the Serial Initilization Function, Main Program Starts from here
#init_serial()

#temp = raw_input('Type what you want to send, hit enter:\r\n')
#__sendNow(self, raw_input)         #Writes to the SerialPort

#while 1:    
#    bytes = ser.readline()  #Read from Serial Port
    #print 'You sent: ' + bytes      #Print What is Read from Port
    
#Ctrl+C to Close Python Window

if True:
    tty = "/dev/ttyUSB0"
    
    #create = Create(tty)
    
    
    #create.spin(1,0.1,True)


    #create.run()
    #create.seekDock()
    #roombaSensors = create.readSensors()
    
    #print(roombaSensors)
    ser = Serial(tty, 19200)
    time.sleep(5)
    ser.write("\x8E\x00")
    ser.flush()
    time.sleep(2)
    counter = 0
    sensorBits = []
    for i in range(0,25):
        print i
        val = ord(ser.read())
        sensorBits.append(val) # stores all data received into a list
        print(val)
        print sensorBits
        
    #create.square()
    #create.circle()
    #create.spin(2, 1, True) #Clockwise
    #create.spin(2, 1, False) #Counter-clockwise
    #bytes = create.read
    # Python opens and closes serial connection (resetting arduino, does not enact commands)
    # Having gtkterm open, enables an open connection to arduino
    # Look into serial.flush function (might be useful)
    #sleep(3)
    #create.send("\x80\x84\x89\xFF\x38\x01\xF4")
    #sleep(1)
    #"\x8C\x0\x1\x1F\x3E"
