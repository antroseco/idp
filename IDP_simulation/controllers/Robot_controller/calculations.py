import math

import numpy as np


def get_distance(loc1, loc2):

    loc1 = np.array(loc1)
    loc2 = np.array(loc2)
    dist = np.linalg.norm(loc1 - loc2)
    return dist


def turn_clockwise(coord, location, field):
    """
    returns true if it's faster to turn clockwise from location to coord
    """
    angle1 = math.degrees(np.arctan2(location[2] - field.y, location[0] - field.x))
    angle1 = angle1 % 360
    angle2 = math.degrees(np.arctan2(coord[1] - field.y, coord[0] - field.x))
    angle2 = angle2 % 360

    diff = angle2 - angle1
    if diff < 0:
        diff += 360
    if diff < 180:
        return True

    return False


def required_bearing(coord, location):
    """
    helper function for PID_rotation
    returns a bearing angle to turn to
    """

    required = np.arctan2((coord[0]-location[0]), (-(coord[1]-location[2])))

    # Q1
    if (coord[1] <= location[2]) and (coord[0] >= location[0]):
        required = np.arctan2((coord[0]-location[0]), (-(coord[1]-location[2])))
        required = required * 180.0/np.pi
        # print('Q1')

    # Q2
    elif (coord[1] > location[2]) and (coord[0] >= location[0]):
        required = np.arctan2((coord[1]-location[2]), ((coord[0]-location[0])))
        required = required * 180.0/np.pi + 90.0
        # print('Q2')

    # Q3
    elif (coord[1] <= location[2]) and (coord[0] < location[0]):
        required = -np.arctan2(-(coord[0]-location[0]), (-(coord[1]-location[2])))
        required = required * 180.0/np.pi
        # print('Q3')
    # Q4
    elif (coord[1] > location[2]) and (coord[0] <= location[0]):
        required = -np.arctan2(-(coord[0]-location[0]), (-(coord[1]-location[2])))
        required = required * 180.0/np.pi
        # print('Q4')
    return required


def get_wall_position(angle, position):
    """
    finds distance from centre of the robot to the wall depending on the position and rotation of the robot
    input: angle (bearing 0 to 360 degrees), position
    """
    x = position[0]
    z = position[2]

    # find angles at which you can see the walls
    # eg top wall can be seen at angles between left_wall and top_wall
    # eg right wall can be seen at angles between top_wall and right_wall

    left_wall = math.degrees(math.atan(abs((1.2 - z) / (1.2 - x))))
    top_wall = math.degrees(math.atan(abs((1.2 - z) / (-1.2 - x)))) + 90
    right_wall = math.degrees(math.atan(abs((-1.2 - z) / (-1.2 - x)))) + 180
    bottom_wall = math.degrees(math.atan(abs((-1.2 - z) / (1.2 - x)))) + 270

    # now find the distance from the wall that robot is looking at
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
    if measurement invalid it returns False
    returns: valid [Bool], x[int], y[int]
    """
    x = position[0] + distance * math.cos(math.radians(angle))
    z = position[2] + distance * math.sin(math.radians(angle))

    # if x or z are out of the bounds

    if abs(x) >= 1.15 or abs(z) >= 1.15:
        return False, 0, 0

    return True, x, z


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

        change = math.sqrt((potential_boxes[i][0] - potential_boxes[i-1][0])**2 +
                           (potential_boxes[i][1] - potential_boxes[i-1][1])**2)

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
