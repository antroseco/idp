"""Robot_controller controller."""
import controller
from robot import Robot
from field import Field
from calculations import *
import numpy as np
import math
import time
import hardware

np.set_printoptions(suppress=True)

TIME_STEP = 64
COMMUNICATION_CHANNEL = 1
MAX_VELOCITY = 6.7


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


def encircle(coord, location, field):
    """
    location is 3D gps coordinates
    coord is 2D target coordinate
    
    this assumes both coord and location are in the same half of the field
    """
    if not robot.field_collision(coord, field):
        pass
    else:
        checkpoint, bearing = robot.find_closest_point(field)
        move(checkpoint, error_translation = 0.1)
        
        speed_inner_wheel = 2
        speed_outer_wheel = 3.5         
        
        #determine whether its better to turn anticlockwise
        clockwise = turn_clockwise(coord, location, field)
        
 
        if clockwise:
            bearing += 90
            if bearing > 180:
                bearing -= 360 
        else:
            bearing -= 90
            if bearing <= -180:
                bearing += 360
        
        PID_rotation(bearing)
        
        collision = robot.field_collision(coord, field)        
        while collision:
        
            if clockwise:
                robot.left_wheel.setVelocity(speed_inner_wheel)
                robot.right_wheel.setVelocity(speed_outer_wheel)
            else:
                robot.left_wheel.setVelocity(speed_outer_wheel)
                robot.right_wheel.setVelocity(speed_inner_wheel)
            robot.step(TIME_STEP)
            collision = robot.field_collision(coord, field)
            
    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0)
    return
    
    
def move_avoid_fields(coord, error_translation = 0.05):
    """
    avoids both fields
    """
    robot.step(TIME_STEP)
    location = robot.gps.getValues()
    
    
    if location[2] * coord[1] >= 0: #target and current location are on the same half
        
        if location[2] > 0:
            #red half
            encircle(coord, location, red_field)
        else:
            #green half
            encircle(coord, location, green_field)
        move(coord, error_translation)
    
    else: #opposite halfs
        if coord[0] > 0.3 or coord[0] < -0.3:
            intermediate = [coord[0], 0]
        else:
            if coord[0] > 0:
                intermediate = [0.5, 0]
            else:
                intermediate = [-0.5, 0]
        if location[2] > 0: # red half
            intermediate[1] = -0.2
            encircle(intermediate, location, red_field)
            move(intermediate, error_translation)
            
            encircle(coord, robot.gps.getValues(), green_field)
            move(coord, error_translation)
        else: #green half
            intermediate[1] = 0.2
            encircle(intermediate, location, green_field)
            move(intermediate, error_translation)
            
            encircle(coord, robot.gps.getValues(), red_field)
            move(coord, error_translation)
       
        
    return
    


def PID_rotation(required, final_error = 0.4):
    """input: required is the required bearing,
    The function rotates until the bearing is within final error 
    of the required bearing"""
    
    kP = 0.005
    kD = 2.0
    
    error = required - bearing1(robot.compass)
    
    if error >= 0:
        forward_rot = 1.0
    else:
        forward_rot = -1.0
 
    
    error2 = 360 - abs(error)
    
    if abs(error2) < abs(error):
        previous_error2 = error2
        while abs(error2) > final_error:
    
            
            P = MAX_VELOCITY*kP*error2
            D = (error2-previous_error2)/(100.0)*kD
            
            v = P + D 
            if v > MAX_VELOCITY:
                v = MAX_VELOCITY
            elif v < -MAX_VELOCITY:
                v = -MAX_VELOCITY
               
        
            robot.left_wheel.setVelocity(-v)
            robot.right_wheel.setVelocity(v)
            
            previous_error2 = error2
            previous_error  = error
            error = required - bearing1(robot.compass)
            error2 = 360 - abs(error)
            error2 = -1.0*forward_rot*error2
            if abs(error2) > abs(error):
                
                previous_error2 = previous_error
                error2 = error
                
            if error >= 0:
                forward_rot = 1.0
            else:
                forward_rot = -1.0
               
            
              
             
            robot.step(TIME_STEP)
        return
        
    else:                          
        previous_error = error
    
        while abs(error) > final_error:
    
        
            P = MAX_VELOCITY*kP*error
            D = (error-previous_error)/(100.0)*kD
            
            v = P + D 
            if v > MAX_VELOCITY:
                v = MAX_VELOCITY
            elif v < -MAX_VELOCITY:
                v = -MAX_VELOCITY
        
            robot.left_wheel.setVelocity(-v)
            robot.right_wheel.setVelocity(v)
            
            previous_error = error
            error = required - bearing1(robot.compass)
            
            robot.step(TIME_STEP)
        return
  



def PID_translation(coord, final_error = 0.15, reverse = False, maxVelocity = 6.7):
    """input: 2D desired coordinate coord,
    The function moves in a straight line until the desired location is within 
    the final error distance"""
    
    
    error = ((coord[0] - robot.gps.getValues()[0])**2 +(coord[1] - robot.gps.getValues()[2])**2)**(1/2)
    
    while abs(error) > final_error or math.isnan(error):
        if math.isnan(error) :
            pass
            
        else:
        
            v = error*MAX_VELOCITY*10
            if v > MAX_VELOCITY:
                v = MAX_VELOCITY
            
              
            robot.left_wheel.setVelocity(v)
            robot.right_wheel.setVelocity(v)
            if reverse: 
                robot.left_wheel.setVelocity(-v)
                robot.right_wheel.setVelocity(-v) 
            
        robot.step(TIME_STEP)
        
        x = (coord[0] - robot.gps.getValues()[0])
        z = (coord[1] - robot.gps.getValues()[2])
        
        previous_error = error
        error = (x**2 + z**2)**0.5
        
        if previous_error < error:
            angle = required_bearing(coord, robot.gps.getValues())
            PID_rotation(angle)
        
    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0) 
        
    return
  

def move(coord, error_rotation = 1, error_translation = 0.15):
    """
    move to location coord
    """
    required_angle = required_bearing(coord, robot.gps.getValues())
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

def return_box_field(coord):
    """
    function that makes robot return a box in the specified field without it clashing with
    other boxes that are already in the field
    input 3D coordinates of the robot
    """
    
    intermediate, final = robot.field.get_to_field(coord)
 
    move_avoid_fields(intermediate, error_translation = 0.15)
    if final[0] > 0:
        PID_rotation(-90)
    else:
        PID_rotation(90)
        
    move(final, error_translation = 0.2)
    
    robot.withdraw_dualclaw()

    reverse()
    return


def reverse():
    robot.left_wheel.setVelocity(-robot.MAX_VELOCITY)
    robot.right_wheel.setVelocity(-robot.MAX_VELOCITY)
    #reverse a little bit
    for _ in range(2):
        robot.step(TIME_STEP)
    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0)



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
        
    move_avoid_fields(intermediate)
    
    if robot.colour == 'red':
        PID_rotation(180)
    else:
        PID_rotation(0)
        
    PID_translation(final, reverse=True)
    
    return


def test_collisions():
    if robot.colour == 'green':
        move((0, 1))  
    if robot.colour == 'red':
        move((1, 0))      

    
    
#This part is executed
r = controller.Robot()
if r.getName() == 'robot_red':
    robot = Robot(r, 'red')
    
else:
    robot = Robot(r, 'green')
        
red_field = Field('red')
green_field = Field('green')

"""
if r.getName() == 'robot_red':
    coord = (0.0,0.5)
    robot.step(TIME_STEP)
    PID_rotation(required_bearing(coord,robot.gps.getValues()))
    PID_translation(coord)
    coord = (-0.5,0.1)
    PID_rotation(required_bearing(coord,robot.gps.getValues()))
    PID_translation(coord)
   """
    

#test_collisions()

robot.step(TIME_STEP)
positions = sweep(0.4)
print(positions)

robot.step(TIME_STEP)
robot.send_sweep_locations(positions)

robot.step(TIME_STEP)
#after we get locations from other robot, all boxes that each robot needs
#to visit are saved in robot.box_queue
#robot.get_sweep_locations()
robot.get_messages()

#print(positions)

initial_pass = True   

while not robot.box_queue.empty() and robot.field.available():
    
    pos = robot.box_queue.get()
    print(pos)
    robot.withdraw_dualclaw()
    
    if initial_pass:
        initial_pass = False
        move(pos)
    else:
        move_avoid_fields(pos, error_translation = 0.1)


    robot.step(TIME_STEP)
    c = robot.deploy_dualclaw()
    for i in range(10):
        robot.step(TIME_STEP)

    colour = ''
    if c == 0:
        colour = 'red'
    elif c == 1:
        colour = 'green'
    else:
        c = robot.remeasure()
        robot.close_dualclaw()
        if c == 0:
            colour = 'red'
        elif c == 1:
            colour = 'green'
        

        
    robot.step(TIME_STEP)

    
    if colour == robot.colour:
        return_box_field(robot.gps.getValues())
    else:
        robot.withdraw_dualclaw()
        reverse()
        
        if c == 0 or c == 1: 
            robot.step(TIME_STEP)
            robot.send_box_location(pos)
        
    #robot.read_all_locations()
    

print('parking')

robot.step(TIME_STEP)
finish_in_field()
