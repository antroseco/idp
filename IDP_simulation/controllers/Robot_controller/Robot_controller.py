"""Robot_controller controller."""
from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Emitter
from controller import Receiver
from hardware import ADCInput, PhototransistorCircuit
import numpy as np
import math
from matplotlib import pyplot as plt

TIME_STEP = 64
COMMUNICATION_CHANNEL = 1
MAX_VELOCITY = 6




def setup_ultrasonic():
    dsUltrasonic = robot.getDevice('ultrasonic')
    dsUltrasonic.enable(TIME_STEP)
    return dsUltrasonic

def setup_infrared():
    dsInfraRed = robot.getDevice("Sharp's IR sensor GP2Y0A02YK0F")
    dsInfraRed.enable(TIME_STEP)
    return dsInfraRed

def setup_lightsensor():
    ls = robot.getDevice('TETP4400')
    ls.enable(TIME_STEP)
    return ls

def setup_wheels():
    """
    sets up wheels
    input:/
    return: left, right wheel (type Motor)
    """
    left_wheel = robot.getDevice('left_wheel')
    left_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(0.0)
    
    right_wheel = robot.getDevice('right_wheel')
    right_wheel.setPosition(float('inf'))
    right_wheel.setVelocity(0.0)
    
    return left_wheel, right_wheel


def setup_communication():
    """
    sets up communication devices
    input:/
    return: emitter (type Emitter) and receiver (type Receiver)
    """
        
    emitter = robot.getDevice('emitter')
    emitter.setChannel(COMMUNICATION_CHANNEL)

    receiver = robot.getDevice('receiver')
    receiver.setChannel(COMMUNICATION_CHANNEL)
    receiver.enable(TIME_STEP)
    
    return emitter, receiver


def setup_sensors():
    """
    sets up all sensors
    input: /
    return: gps, compass
    """
    gps = robot.getDevice('gps')
    gps.enable(100)
    
    compass = robot.getDevice('compass')
    compass.enable(1)
    
    return gps, compass
    


def send_message(message):
    """
    sends string message through a receiver
    input: string message
    return: /
    """

    data = message.encode('utf-8')
    emitter.send(data)
    
    
    
    
    
def get_message():
    """
    gets the first message in receiver's queue and pops it from queue
    input: /
    return: string message
    """
    
    if receiver.getQueueLength() > 0:
        data = receiver.getData()
        message = data.decode('utf-8')
        receiver.nextPacket()
        return message




def bearing1(): 
    """
    This gives a bearing -180,180
    """
    theta = np.arctan2(compass.getValues()[0],compass.getValues()[2])
    theta = (theta-(np.pi /2.0) )*180.0/np.pi
    if theta < -180:
        theta += 360
        
    
    return  theta
   
   
   
   
def bearing(): 
    """
    This gives a bearing 0,360
    """
    theta = np.arctan2(compass.getValues()[0],compass.getValues()[2])
    theta = (theta-(np.pi /2.0))*180.0/np.pi
    if theta < 0 :
        theta += 360
        
    
    return  theta




def PID_rotation(coord):
    
    
    
    for i in range(2):
        location = gps.getValues()
        required = np.arctan2((coord[0]-location[0]),(-(coord[1]-location[2])))
        robot.step(TIME_STEP)
      
        
    #Q1    
    if (coord[1] <= location[2]) and (coord[0] >= location[0]):
        required = np.arctan2((coord[0]-location[0]),(-(coord[1]-location[2])))
        required = required *180.0/np.pi
        #print('Q1')
       
    

    #Q2            
    elif (coord[1] > location[2]) and (coord[0] >= location[0]) :
        required = np.arctan2((coord[1]-location[2]),((coord[0]-location[0])))
        required = required *180.0/np.pi +90.0
        #print('Q2')
        
    #Q3    
    elif (coord[1] <= location[2]) and (coord[0] < location[0]) :
        required = -np.arctan2(-(coord[0]-location[0]),(-(coord[1]-location[2])))
        required = required *180.0/np.pi 
        #print('Q3')
    #Q4    
    elif (coord[1] > location[2]) and (coord[0] <= location[0]) :
        required = -np.arctan2(-(coord[0]-location[0]),(-(coord[1]-location[2])))
        required = required *180.0/np.pi 
        #print('Q4')
            
    

    
    error = required - bearing1()
    

    
    while abs(error) > 0.1:
        #print(required,'required')
        #print(error)
        v = 6.28*0.01*error
        if v > 6.28:
            v = 6.28
        elif v < -6.28:
            v = -6.28

        left_wheel.setVelocity(v)
        right_wheel.setVelocity(-v)
        error = required - bearing1()
        robot.step(TIME_STEP)
    
        if abs(error) < 0.1:
            return





def PID_translation(coord):
    error = ((coord[0] - gps.getValues()[0])**2 +(coord[1] - gps.getValues()[2])**2)**(1/2)
    
    while abs(error) > 0.05 or math.isnan(error):
        if math.isnan(error) :
            pass
            
        else:
        
            v = error*6.28*10
            if v > 6.28:
                v = 6.28
                
                
            
            left_wheel.setVelocity(v)
            right_wheel.setVelocity(v)
            
        robot.step(TIME_STEP)
        x = (coord[0] - gps.getValues()[0])
        z = (coord[1] - gps.getValues()[2])
        
        error = (x**2 + z**2)**0.5
        
        
    return
    
    


def sweep(velocity):
    """
    do a 180 degree spin while collecting data from distance sensor
    input: velocity of wheels/how fast is the rotation
    output: numpy array with stored values from the distance sensor
    """    
    
    #find current rotation [0-360 degrees]
    initial_angle = bearing()
        
    distances = []    
        
    #sweep 180 degrees    
    swept_angle = 0
    while swept_angle < 180:
        
        right_wheel.setVelocity(velocity)
        left_wheel.setVelocity(-velocity)
        
        distances.append(dsUltrasonic.getValue())
        print(dsUltrasonic.getValue())
        robot.step(TIME_STEP)
        
        if bearing() > initial_angle:
            swept_angle = bearing() - initial_angle
        else:
            swept_angle = 360 - initial_angle + bearing()
            
    right_wheel.setVelocity(0)
    left_wheel.setVelocity(0)
    
    distances = np.array(distances)
    return distances


def getDistancesIR():
     voltage = dsInfraRed.getValue()
     meter = (0.7611*(voltage)**(-0.9313)) - 0.1252
     return meter



robot = Robot()

left_wheel, right_wheel = setup_wheels()
emitter, receiver = setup_communication()
gps, compass = setup_sensors()
dsUltrasonic = setup_ultrasonic()
dsInfraRed = setup_infrared()

ls = setup_lightsensor()
lsCircuit = PhototransistorCircuit(ls)
lsInput = ADCInput(lambda: lsCircuit.voltage(), 5.0, 2)

a = 1

while robot.step(TIME_STEP) != -1:
    # Get sensor values
    LightSensorValue = lsInput.read()    
    
    # Outputs the most recent reading 
    DistanceUltrasonic = dsUltrasonic.getValue()
  
    DistanceInfraRed = getDistancesIR()
    
    coord2 = (0.3,0.3)
    
    angle = bearing()
    #print(angle)
    
    #PID_rotation(coord2)
   
    #PID_translation(coord2)
    
    #right_wheel.setVelocity(1.0)
    #left_wheel.setVelocity(-1.0)
    
    
    distances = sweep(0.5)
    print(distances)
    print(DistanceInfraRed)
    break  

    
print('end')