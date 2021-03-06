"""Robot_controller controller."""
import controller
from robot import Robot
from searchCalculations import *
import numpy as np
import math
import time
import hardware

TIME_STEP = 64
COMMUNICATION_CHANNEL = 1
MAX_VELOCITY = 6


def bearing1(compass_obj): 
    """
    This gives a bearing -180,180
    input: compass object(type Compass)
    """
    theta = np.arctan2(compass_obj.getValues()[0],compass_obj.getValues()[2])
    theta = (theta-(np.pi /2.0) )*180.0/np.pi
    if theta < -180:
        theta += 360
        
    
    return  theta
   
   
   
   
def bearing(compass_obj): 
    """
    This gives a bearing 0,360
    input: compass object(type Compass)
    """
    theta = np.arctan2(compass_obj.getValues()[0],compass_obj.getValues()[2])
    theta = (theta-(np.pi /2.0))*180.0/np.pi
    if theta < 0 :
        theta += 360
        
    
    return  theta



def send_location():
    location = gps.getValues()
    message = "{},{}".format(location[0],location[2])
    send_message(message)
    
    
def get_location():
    message = get_message()
    message = tuple(map(str, message.split(',')))  
    try: 
        message = [float(message[0]),float(message[1])]
        return message  
    except:
        return []



def collision_prevention():
    self_location = (gps.getValues()[0],gps.getValues()[2])
    send_location()
    other_location = get_location()
    robot.step(TIME_STEP)
    print(other_location,'other location')
    print(self_location,'self location')
    
    if not other_location:
        pass
    else:
        x = (self_location[0] - other_location[0])**2
        z = (self_location[1] - other_location[1])**2
        
        distance = (x**2 + z**2)**0.5
        print(distance)
        threshold = 0.2
        
        if distance < threshold:
            left_wheel.setVelocity(0)
            right_wheel.setVelocity(0)
            return 'stop'


def PID_rotation(coord, final_error = 0.5):

    for i in range(2):
        location = robot.gps.getValues()
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

    error = required - bearing1(robot.compass)
    
    previous_error = error

 
    while abs(error) > final_error:
        #print(required,'required')
        #print(error)
        kP = 0.003
        kD = -11.0
        P = 6.28*kP*error
        D = (error-previous_error)/(100.0)*kD
        
        v = P + D 
        if v > 6.0:
            v = 6.0
        elif v < -6.0:
            v = -6.0
    
        robot.left_wheel.setVelocity(-v)
        robot.right_wheel.setVelocity(v)
        
        previous_error = error
        error = required - bearing1(robot.compass)
        
        robot.step(TIME_STEP)
    return






def PID_translation(coord, final_error = 0.15):
    error = ((coord[0] - robot.gps.getValues()[0])**2 +(coord[1] - robot.gps.getValues()[2])**2)**(1/2)
    
    while abs(error) > final_error or math.isnan(error):
        if math.isnan(error) :
            pass
            
        else:
        
            v = error*6.28*10
            if v > 6.28:
                v = 6.28
                
            robot.left_wheel.setVelocity(v)
            robot.right_wheel.setVelocity(v)
            
        robot.step(TIME_STEP)
        x = (coord[0] - robot.gps.getValues()[0])
        z = (coord[1] - robot.gps.getValues()[2])
        
        previous_error = error
        error = (x**2 + z**2)**0.5
        
        if previous_error < error:
            PID_rotation(coord)
        
    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0) 
        
    return
  

def sweep(velocity = 0.5):
    """
    do a 180 degree spin while collecting data from distance sensor
    input: velocity of wheels/how fast is the rotation
    output: numpy array with stored values from the distance sensor
    """    
    
    
    #find current rotation [0-360 degrees]
    initial_angle = bearing(robot.compass1)   
    
    #store potential boxes locations     
    boxes = []    
    
    #sweep 360 degrees    
    swept_angle = 0
    
    while swept_angle < 355:
        
        robot.right_wheel.setVelocity(velocity)
        robot.left_wheel.setVelocity(-velocity)
        
        robot.step(TIME_STEP)
        
        #distance from robot centre to wall in this direction
        current_angle = bearing(robot.compass1)
        current_position = robot.gps.getValues()
        wall_dist = get_wall_position(current_angle, current_position)
        
        #wall_dist is decreased by robot-sensor distance
        wall_dist -= 0.11
        
          
        #get infrared reading and convert to meters
        infrared_dist = 0.7611 * math.pow(robot.infrared.getValue(), -0.9313) - 0.1252
 
        
        #print(infrared_dist, wall_dist)
        
        
        #if measured distance is less than wall_dist then assume there's a box
        #also if wall is more than 1.5 away disregard measurements because it's further than sensor's range   
        if abs(wall_dist - infrared_dist) > 0.1 and wall_dist < 1.4:
            x, z = potential_box_position(infrared_dist + 0.11, current_angle, current_position)
            boxes.append([x, z])
        
                
        if current_angle > initial_angle:
            swept_angle = current_angle - initial_angle
        else:
            swept_angle = 360 - initial_angle + current_angle
            
    locations = box_position(np.array(boxes))     
            
    robot.right_wheel.setVelocity(0)
    robot.left_wheel.setVelocity(0)
    robot.step(TIME_STEP)
    
    return locations

                

def set_claw(targetAngle, targetClaw, targetSensor):
    """move the box_claw to the input angle, 
    input in degrees, use positionSensor to provide feedback
    input: targetAngle, targetClaw(which claw is used), targetSensor(Accompanying sensor)
    
    """
    desired = targetAngle*np.pi/180
    error = abs(desired - targetSensor.getValue())
    accuracy = 5*np.pi/180
    
    while error > accuracy:
        targetClaw.setPosition(desired)
        robot.step(TIME_STEP)
        error = abs(desired - targetSensor.getValue())
    return
    
    # box_claw_sensor.
    # box_claw.setPosition(np.pi/2)
    
def withdraw_boxclaw():
    set_claw(90, box_claw, box_claw_sensor)
    
def deploy_boxclaw():
    set_claw(0, box_claw, box_claw_sensor)

def set_dualclaw(targetAngle,targetClaw1,targetSensor1,targetClaw2,targetSensor2):
    desired = targetAngle*np.pi/180
    error = abs(desired - targetSensor1.getValue())
    accuracy = 1*np.pi/180
    previous = 100 #arbitrary value just serves as placeholder
    count = 0    
    while error > accuracy:
        measurement = targetSensor1.getValue()
        targetClaw1.setPosition(desired)
        targetClaw2.setPosition(-desired)
        if abs(measurement - previous) < accuracy:
            count += 1
        else:
            count = 0
            
        if count >= 3:
            break
        previous = measurement
        robot.step(TIME_STEP)
    
def measureLight(lightSensor):
    """ Function to return voltage values based on the sensors input, use the filters for TEPT4400 as the ones in proto
    
    """
    circuit = hardware.PhototransistorCircuit(robot.lightSensor)
    analogue_input = hardware.ADCInput(lambda:circuit.voltage())
    return analogue_input.read()
    
        
def deploy_dualclaw(targetClaw1,targetSensor1,targetClaw2,targetSensor2):
    """function to make the dual-claw go from open to closed, detect colour of the block it holds in this process
    return 0 if detected red, 1 if detected green, 2 if detected neither, 3 if detected both
    """

    desired = -5*np.pi/180 #minus value should not be reached, break loop when count reaches 3
    error = abs(desired - targetSensor1.getValue())
    accuracy = 1*np.pi/180 #accuracy value in degrees
    previous = 100 #arbitrary value just serves as placeholder
    count = 0      #start counting for each time frame where the servo angle does not change, break loop upon reaching 3
    red = False
    green = False
    redLowerBound = 500 #(environment is 470),one reading above this value turns red to True
    greenLowerBound = 220 # (environment is 210), Value obtained by experiment, and is arbitrary
    
    while error > accuracy:
        redValue = robot.red_analogue.read()
        greenValue = robot.green_analogue.read()
        if redValue > redLowerBound:
            red = True
        if greenValue > greenLowerBound:
            green = True
        measurement = targetSensor1.getValue()
        targetClaw1.setPosition(desired) #both claw move synchronously in different direction
        targetClaw2.setPosition(-desired)
        if abs(measurement - previous) < accuracy: #compare measurement from previous time frame to current, add 1 to count if same
            count += 1
        else:
            count = 0
            
        if count >= 3:
            break
        previous = measurement 
        robot.step(TIME_STEP)
        error = abs(desired - targetSensor1.getValue())
    
    if red and not green:
        print('red')
        return 0
    elif green and not red:
        print('green')
        return 1
    elif not green and not red:
        print('not detected')
        return 2
    if red and green:
        print('bad result')
        return 3
        
def withdraw_dualclaw(targetClaw1,targetSensor1,targetClaw2,targetSensor2):
    """function to make the dual-claw go from closed to open
    
    """

    desired = 40*np.pi/180 #arbitrary value
    error = abs(desired - targetSensor1.getValue())
    accuracy = 1*np.pi/180 #accuracy value in degrees
    while error > accuracy:
        measurement = targetSensor1.getValue()
        targetClaw1.setPosition(desired) #both claw move synchronously in different direction
        targetClaw2.setPosition(-desired)
        robot.step(TIME_STEP)
        error = abs(desired - targetSensor1.getValue())
        

def return_box_field(coord):
    intermediate, final = robot.field.get_to_field(coord)
    
    PID_rotation(intermediate)
    PID_translation(intermediate)
    PID_rotation(final)
    PID_translation(final)
    withdraw_dualclaw(robot.left_claw, robot.left_claw_sensor, robot.right_claw, robot.right_claw_sensor)
    
    robot.left_wheel.setVelocity(-5)
    robot.right_wheel.setVelocity(-5)
    #reverse a little bit
    for j in range(30):
        robot.step(TIME_STEP)
    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0)
    robot.step(TIME_STEP)
    return 


robot = Robot(controller.Robot(), 'green')

robot.step(TIME_STEP)


positions = sweep(0.6)
print(positions)
    
    
for pos in positions:
    
    withdraw_dualclaw(robot.left_claw, robot.left_claw_sensor, robot.right_claw, robot.right_claw_sensor)
    
    PID_rotation(pos)
    PID_translation(pos)
    a = deploy_dualclaw(robot.left_claw, robot.left_claw_sensor, robot.right_claw, robot.right_claw_sensor)
    print(a)
    robot.step(TIME_STEP)
    return_box_field(robot.gps.getValues())





