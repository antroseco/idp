import numpy as np
import math    


def get_wall_position(angle, position):
    """
    finds distance from centre of the robot to the wall depending on the position and rotation of the robot
    input: angle (bearing 0 to 360 degrees), position
    """    
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
    
       #if x or z are out of the bounds
    
    if x >= 1.15:
        x = 1.15
    elif x <= -1.15:
        x = -1.15
    if z >= 1.15:
        z = 1.15
    elif z <= -1.15:
        z = -1.15

    
    return x, z
    
    
    
def box_position(potential_boxes):
    """
    we will have consecutive positions in array that all come from the same box
    find which belong to the same box and average them out
    input: an array with potential box locations
    returns: array of approximated box positions
    """
    
    
    locations = []
    
    same_box_num = 1
    x_avg = potential_boxes[0][0]
    z_avg = potential_boxes[0][1]
    
    
    for i in range(1, potential_boxes.shape[0]):
    
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
                
    
    