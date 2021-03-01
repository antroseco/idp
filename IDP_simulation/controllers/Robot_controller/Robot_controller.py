"""Robot_controller controller."""
from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Emitter
from controller import Receiver
import numpy as np
import math
from matplotlib import pyplot as plt

TIME_STEP = 64
COMMUNICATION_CHANNEL = 1
MAX_VELOCITY = 6



def setup_infrared():
    infrared = robot.getDevice('IR Sensor')
    infrared.enable(TIME_STEP)
    return infrared


def setup_ultrasonic():
    dsUltrasonic = robot.getDevice('ultrasonic')
    dsUltrasonic.enable(TIME_STEP)
    return dsUltrasonic


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
    gps.enable(TIME_STEP)
    
    compass = robot.getDevice('compass')
    compass.enable(TIME_STEP)
    
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
   

    
    
    previous_error = error
    
    final_error = 0.8
        
        
 
    while abs(error) > final_error:
        #print(required,'required')
        #print(error)
        kP = 0.001
        kD = -11.0
        P = 6.28*kP*error
        D = (error-previous_error)/(100.0)*kD
        
        
        v = P + D 
        if v > 6.0:
            v = 6.0
        elif v < -6.0:
            v = -60
    
        left_wheel.setVelocity(-v)
        right_wheel.setVelocity(v)
        previous_error = error
        error = required - bearing1()
        robot.step(TIME_STEP)
    
    return






def PID_translation(coord):
    error = ((coord[0] - gps.getValues()[0])**2 +(coord[1] - gps.getValues()[2])**2)**(1/2)
    
    final_error = 0.11
    
    while abs(error) > final_error or math.isnan(error):
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
        
        previous_error = error
        error = (x**2 + z**2)**0.5
        
        if previous_error < error:
            PID_rotation(coord)
        
    return
    
    
    
    
def get_wall_position(angle):
    """
    finds distance from centre of the robot to the wall depending on the position and rotation of the robot
    input: angle (bearing 0 to 360 degrees)
    """    
    position = gps.getValues()
    x = position[0]
    z = position[2]
    
    
    #find angles at which you can see the walls
    #eg top wall can be seen at angles between left_wall and top_wall
    #eg right wall can be seen at angles between top_wall and right_wall
    
    left_wall = math.degrees(math.atan(abs((1.2 - z) / (1.2 - x))))
    top_wall = math.degrees(math.atan(abs((1.2 - z) / (-1.2 - x)))) + 90
    right_wall = math.degrees(math.atan(abs((-1.2 - z) / (-1.2 - x)))) + 180
    bottom_wall = math.degrees(math.atan(abs((-1.2 - z) / (1.2 - x)))) + 270
    
    #now find the distance from the wall that robot is looking at
    dist = 0
    
    if bottom_wall < angle or angle <= left_wall:
        dist = abs((1.2 - x)/math.cos(math.radians(angle)))
    elif angle <= top_wall:
        dist = abs((1.2 - z)/math.sin(math.radians(angle)))
    elif angle <= right_wall:
        dist = abs((-1.2 - x)/math.cos(math.radians(angle)))
    else:
        dist = abs((-1.2 - z)/math.sin(math.radians(angle)))
    
    return dist
    
    
def potential_box_position(distance, angle, position):
    """
    finds the box position based on the current angle and location of the robot
    and the distance between the robot and box
    """    
    x = position[0] + distance * math.cos(math.radians(angle))
    z = position[2] + distance * math.sin(math.radians(angle))
    
    return x, z
    
    

def sweep(velocity = 0.5):
    """
    do a 180 degree spin while collecting data from distance sensor
    input: velocity of wheels/how fast is the rotation
    output: numpy array with stored values from the distance sensor
    """    
    
    
    #find current rotation [0-360 degrees]
    initial_angle = bearing()   
    
    #store potential boxes locations     
    boxes = []    
    
    #sweep 360 degrees    
    swept_angle = 0
    
    while swept_angle < 350:
        
        right_wheel.setVelocity(velocity)
        left_wheel.setVelocity(-velocity)
        
        robot.step(TIME_STEP)
        
        
        #get infrared reading and convert to meters
        infrared_dist = 0.7611 * math.pow(infrared.getValue(), -0.9313) - 0.1252
        
        #distance from robot centre to wall in this direction
        current_angle = bearing()
        wall_dist = get_wall_position(current_angle)
        
        #wall_dist is decreased by robot-sensor distance
        wall_dist -= 0.07  
        
        
        #if measured distance is less than wall_dist then assume there's a box
        #also if wall is more than 1.5 away disregard measurements because it's further than sensor's range
        if abs(wall_dist - infrared_dist) > 0.06 and wall_dist < 1.5:
            x, z = potential_box_position(infrared_dist + 0.07, current_angle, gps.getValues())
            boxes.append([x, z])
        
                
        if current_angle > initial_angle:
            swept_angle = current_angle - initial_angle
        else:
            swept_angle = 360 - initial_angle + current_angle
            
        
            
    right_wheel.setVelocity(0)
    left_wheel.setVelocity(0)
    robot.step(TIME_STEP)
    
    return np.array(boxes)



def box_position(potential_boxes):
    """
    we will consecutive positions in array that all come from the same box
    find which belong to the same box and average them out
    input: an array with potential box locations
    returns: array of approximated box positions
    """
    
    
    locations = []
    
    same_box_num = 1
    x_avg = potential_boxes[0][0]
    z_avg = potential_boxes[0][1]
    
    
    for i in range(1, boxes.shape[0]):
    
        change = math.sqrt((potential_boxes[i][0] - potential_boxes[i-1][0])**2 + (potential_boxes[i][1] - potential_boxes[i-1][1])**2)
        
        if change < 0.2:
            x_avg += potential_boxes[i][0]
            z_avg += potential_boxes[i][1]
            same_box_num += 1
        
        else:
            x_avg = x_avg / same_box_num
            z_avg = z_avg / same_box_num
            locations.append([x_avg, z_avg])
            
            x_avg = potential_boxes[i][0]
            z_avg = potential_boxes[i][1]
            same_box_num = 1
            
            
    x_avg = x_avg / same_box_num
    z_avg = z_avg / same_box_num
    locations.append([x_avg, z_avg])
    
    return np.array(locations)
                





robot = Robot()

left_wheel, right_wheel = setup_wheels()
emitter, receiver = setup_communication()
gps, compass = setup_sensors()
dsUltrasonic = setup_ultrasonic()
infrared = setup_infrared()


while robot.step(TIME_STEP) != -1:
    
    coord2 = (0.3,0.3)
    
    #PID_rotation(coord2)
   
    #PID_translation(coord2)
    
    boxes = sweep(0.5)
    #print(boxes)
    positions = box_position(boxes)
    print(positions)
    break  

    
print('end')
