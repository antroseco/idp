"""Robot_controller controller."""
import controller
from robot import Robot
from searchCalculations import *
import numpy as np
import math
import time
import hardware

np.set_printoptions(suppress=True)

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

def field_collision(coord):
    location = robot.gps.getValues()
    location = (location[0],location[2])
    
    m = (coord[1] - location[1])/(coord[0]-location[0])
    c = coord[1] - m*coord[0]
    print(m)
    print(c)
   #c1 = location[1] - m*location[0]
    x = np.linspace(min(coord[0],location[0]),max(coord[0],location[0]),101,endpoint=True)
    z = m*x + c
    z1 = [i for i in z if i > -0.6]
    z2 = [i for i in z1 if i < -0.2]
    x1 = [i for i in x if i > -0.2]
    x2 = [i for i in x1 if i < 0.2]

    print(z2)
    if z2 and x2:
        print(coord)
        print(location)
        return 'will collide with field'
    else:
        return 'no collision'
   



def required_bearing(coord):
    """
    helper function for PID_rotation
    returns a bearing angle to turn to
    """
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
    return required    
    
    
    

def PID_rotation(required, final_error = 0.5):

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
    


def PID_translation_reverse(coord, final_error = 0.15):
    error = ((coord[0] - robot.gps.getValues()[0])**2 +(coord[1] - robot.gps.getValues()[2])**2)**(1/2)
    
    while abs(error) > final_error or math.isnan(error):
        if math.isnan(error) :
            pass
            
        else:
        
            v = error*6.28*10
            if v > 6.28:
                v = 6.28
                
            robot.left_wheel.setVelocity(-v)
            robot.right_wheel.setVelocity(-v)
            
        robot.step(TIME_STEP)
        x = (coord[0] - robot.gps.getValues()[0])
        z = (coord[1] - robot.gps.getValues()[2])
        
        previous_error = error
        error = (x**2 + z**2)**0.5
        
    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0) 
        
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
            angle = required_bearing(coord)
            PID_rotation(angle)
        
    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0) 
        
    return


  

def move(coord, error_rotation = 0.5, error_translation = 0.15):
    """
    move to location coord
    """
    required_angle = required_bearing(coord)
    PID_rotation(required_angle, error_rotation)
    PID_translation(coord, error_translation)
    return
    
    
    
    
    

def sweep(velocity = 0.5, swept_angle =355):
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
        

        #get quantized infrared level and convert to volts
        infrared_volts = robot.infrared_analogue.read() * robot.infrared_vref / 1023
        #get infrared reading and convert to meters
        infrared_dist = 0.7611 * math.pow(infrared_volts, -0.9313) - 0.1252
 
        
        #print(infrared_dist, wall_dist)
        
        
        #if measured distance is less than wall_dist then assume there's a box
        #also if wall is more than 1.5 away disregard measurements because it's further than sensor's range   
        if abs(wall_dist - infrared_dist) > 0.1 and wall_dist < 1.4:
            valid, x, z = potential_box_position(infrared_dist + 0.11, current_angle, current_position)
            if(valid):
                boxes.append([x, z])
        
                
        if current_angle > initial_angle:
            swept_angle = current_angle - initial_angle
        else:
            swept_angle = 360 - initial_angle + current_angle
            
    locations = box_position(np.array(boxes))     
            
    robot.right_wheel.setVelocity(0)
    robot.left_wheel.setVelocity(0)
    robot.step(TIME_STEP)

    robot.sweep_locations = locations
    
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
    redLowerBound = 948 # (environment is 930),one reading above this value turns red to True
    greenLowerBound = 436 # (environment is 418), values are about 0.5 lux above ambient
    
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
    """
    function that makes robot return a box in the specified field without it clashing with
    other boxes that are already in the field
    input 3D coordinates of the robot
    """
    
    intermediate, final = robot.field.get_to_field(coord)
 
    move(intermediate, error_translation = 0.15)
    if final[0] > 0:
        PID_rotation(-90)
    else:
        PID_rotation(90)
        
    move(final, error_translation = 0.2)
    
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




def finish_in_field():
    """
    for the ending of the task, robot goes and stays in its field 
    """
    
    if robot.colour == 'red':
        intermediate = (0, 1)
        final = (0, 0.4)
    else:
        intermediate = (0, -1)
        final = (0, -0.4)
        
    move(intermediate)
    
    if robot.colour == 'red':
        PID_rotation(180)
    else:
        PID_rotation(0)
        
       
    PID_translation_reverse(final)
    
    return
    
    
    
    

#This part is executed
r = controller.Robot()
if r.getName() == 'robot_red':
    robot = Robot(r, 'red')
else:
    robot = Robot(r, 'green')



robot.step(TIME_STEP)


positions = sweep(0.6)

robot.step(TIME_STEP)
robot.send_sweep_locations(positions)

robot.step(TIME_STEP)
#after we get locations from other robot, all boxes that each robot needs
#to visit are saved in robot.box_queue
robot.get_sweep_locations()

print(positions)

    

while not robot.box_queue.empty() and robot.field.available():
    
    pos = robot.box_queue.get()
    
    withdraw_dualclaw(robot.left_claw, robot.left_claw_sensor, robot.right_claw, robot.right_claw_sensor)
    
    move(pos)

    deploy_dualclaw(robot.left_claw, robot.left_claw_sensor, robot.right_claw, robot.right_claw_sensor)


    robot.step(TIME_STEP)
    return_box_field(robot.gps.getValues())
    

robot.step(TIME_STEP)
finish_in_field()



