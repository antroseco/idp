"""first_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS , Compass
import numpy as np
import math

# create the Robot instance.
robot = Robot()

gps = robot.getDevice('gps')
compass = robot.getDevice('compass')
# get the time step of the current world.
timestep = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)


left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

gps.enable(100)
compass.enable(1)


def bearing1(): 
    """This gives a bearing -180,180"""
    theta = np.arctan2(compass.getValues()[0],compass.getValues()[2])
    theta = (theta-(np.pi /2.0) )*180.0/np.pi
    if theta < -180:
        theta += 360
        
    
    return  theta
   
def bearing(): 
    """This gives a bearing 0,360"""
    theta = np.arctan2(compass.getValues()[0],compass.getValues()[2])
    theta = (theta-(np.pi /2.0))*180.0/np.pi
    if theta < 0 :
        theta += 360
        
    
    return  theta

"""
def PID_rotation(coord):
    required = np.arctan((coord[0]-gps.getValues()[0])/(-(coord[1]-gps.getValues()[2])))
    required = required *180.0/np.pi
    error = required - bearing()
    
    while (abs(error) > 0.1) or math.isnan(error):
        
        if math.isnan(error):
            required = np.arctan((coord[0]-gps.getValues()[0])/(-(coord[1]-gps.getValues()[2])))
            required = required *180.0/np.pi
            pass
    
  
    
        else: 
            v = 6.28*0.01*error
            if v > 6.28:
                v = 6.28
            elif v < -6.28:
                v = 6.28
            left_motor.setVelocity(v)
            right_motor.setVelocity(-v)
            error = required - bearing()
        robot.step(timestep)     
        error = required - bearing()
        
    
        print(error)
        if abs(error) < 0.1:
            break
    return"""

    
    

def PID_rotation(coord):
    
    
    
    for i in range(2):
        location = gps.getValues()
        required = np.arctan2((coord[0]-location[0]),(-(coord[1]-location[2])))
        robot.step(timestep)
      
        
    #Q1    
    if (coord[1] <= location[2]) and (coord[0] >= location[0]):
        required = np.arctan2((coord[0]-location[0]),(-(coord[1]-location[2])))
        required = required *180.0/np.pi
        print('Q1')
       
    

    #Q2            
    elif (coord[1] > location[2]) and (coord[0] >= location[0]) :
        required = np.arctan2((coord[1]-location[2]),((coord[0]-location[0])))
        required = required *180.0/np.pi +90.0
        print('Q2')
        
    #Q3    
    elif (coord[1] <= location[2]) and (coord[0] < location[0]) :
        required = -np.arctan2(-(coord[0]-location[0]),(-(coord[1]-location[2])))
        required = required *180.0/np.pi 
        print('Q3')
    #Q4    
    elif (coord[1] > location[2]) and (coord[0] <= location[0]) :
        required = -np.arctan2(-(coord[0]-location[0]),(-(coord[1]-location[2])))
        required = required *180.0/np.pi 
        print('Q4')
            
    

    
    error = required - bearing1()
    

    
    while abs(error) > 0.1:
        print(required,'required')
        print(error)
        v = 6.28*0.01*error
        if v > 6.28:
            v = 6.28
        elif v < -6.28:
            v = -6.28

        left_motor.setVelocity(v)
        right_motor.setVelocity(-v)
        error = required - bearing1()
        robot.step(timestep)
    
        if abs(error) < 0.1:
            return
   
                
""" error = 360 - abs(required-bearing1())
        
        
        
        while abs(error) > 0.1:
            print(required,'required')
            print(error,'error')
            print(bearing1())
            v = 6.28*0.01*error
            if v > 6.28:
                v = 6.28
            elif v < -6.28:
                v = -6.28
    
            left_motor.setVelocity(-v)
            right_motor.setVelocity(v)
            error = 360 - abs(required-bearing1())
            robot.step(timestep)
        
            if abs(error) < 0.1:
                return"""
   



def PID_translation(coord):
    error = ((coord[0] - gps.getValues()[0])**2 +(coord[1] - gps.getValues()[2])**2)**(1/2)
    
    while abs(error) > 0.05 or math.isnan(error):
        if math.isnan(error) :
            pass
            
        else:
        
            v = error*6.28*10
            if v > 6.28:
                v = 6.28
                
                
            
            left_motor.setVelocity(v)
            right_motor.setVelocity(v)
            
        robot.step(timestep)
        x = (coord[0] - gps.getValues()[0])
        z = (coord[1] - gps.getValues()[2])
        
        error = (x**2 + z**2)**0.5
        
        
    return
    
        
    
            
            
            
        




# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    
    
 
    #left_motor.setVelocity(6.28*-0.1)
    #right_motor.setVelocity(6.28*0.1)
    #print(bearing(compass.getValues()[0], compass.getValues()[2]))
    
    
    #print(gps.getValues())

    
    coord2 = (-0.5,1.0)
    
    
    PID_rotation(coord2)
   
    PID_translation(coord2)
    
    coord2 = (0,0)
    
    
    PID_rotation(coord2)
   
    PID_translation(coord2)
    
    
   
    
    coord2 = (-0.6,-0.9)
    
    
    PID_rotation(coord2)
   
    PID_translation(coord2)
    
    coord2 = (1.0,0.8)
    
    
    PID_rotation(coord2)
   
    PID_translation(coord2)
    
    coord2 = (0.0,0.0)
    
    
    PID_rotation(coord2)
   
    PID_translation(coord2)
      
    
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    print(gps.getValues()) 
    break
       
   
    
    """
    required = np.arctan((coord[0]-gps.getValues()[0])/(-(coord[1]-gps.getValues()[2])))
    required = required *180.0/np.pi
    error = required - bearing()
    
    
    left_motor.setVelocity(6.28*error*0.01)
    right_motor.setVelocity(-6.28*0.01*error)
    error = required - bearing()
    print(bearing())
    print(error)
    error = required - bearing()
    print(error,'2nd time')"""
    
    
    """if abs(error) < 0.1:
    
        posit
        left_motor.setVelocity(6.28*0.1)
        right_motor.setVelocity(6.28*0.1)"""
        
     
 
    
    pass

# Enter here exit cleanup code.
print('end')