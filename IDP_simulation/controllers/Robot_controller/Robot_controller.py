"""Robot_controller controller."""
import logging
import math

import controller
import numpy as np

from calculations import *
from field import Field
from robot import Robot
from instrumentation import trace

np.set_printoptions(suppress=True)

DEBUG_PID = False
DEBUG_TRACING = False
DEBUG_TRANSLATE = True


# Default level is WARNING, change it to DEBUG
logging.basicConfig(level=logging.DEBUG)

if not DEBUG_TRACING:
    logging.getLogger('tracing').setLevel(logging.INFO)


# Initialize robot
r = controller.Robot()
if r.getName() == 'robot_red':
    robot = Robot(r, 'red')

else:
    robot = Robot(r, 'green')

red_field = Field('red')
green_field = Field('green')


@trace
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
        move(checkpoint, error_translation=0.1)

        speed_inner_wheel = 2
        speed_outer_wheel = 3.5

        # determine whether its better to turn anticlockwise
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
                robot.set_motor_velocities(speed_inner_wheel, speed_outer_wheel)
            else:
                robot.set_motor_velocities(speed_outer_wheel, speed_inner_wheel)
            robot.step()
            collision = robot.field_collision(coord, field)

    robot.reset_motor_velocities()


@trace
def move_avoid_fields(coord, error_translation=0.1):
    """
    avoids both fields
    """
    robot.step()
    location = robot.gps.getValues()

    if location[2] * coord[1] >= 0:  # target and current location are on the same half

        if location[2] > 0:
            # red half
            encircle(coord, location, red_field)
        else:
            # green half
            encircle(coord, location, green_field)
        move(coord, error_translation)

    else:  # opposite halfs
        if coord[0] > 0.5 or coord[0] < -0.5:
            intermediate = [coord[0], 0]
        else:
            if coord[0] > 0:
                intermediate = [0.5, 0]
            else:
                intermediate = [-0.5, 0]
        if location[2] > 0:  # red half
            intermediate[1] = -0.2
            encircle(intermediate, location, red_field)
            move(intermediate, error_translation)

            encircle(coord, robot.gps.getValues(), green_field)
            move(coord, error_translation)
        else:  # green half
            intermediate[1] = 0.2
            encircle(intermediate, location, green_field)
            move(intermediate, error_translation)

            encircle(coord, robot.gps.getValues(), red_field)
            move(coord, error_translation)

    return


@trace
def PID_rotation(required, threshold=0.4) -> bool:
    """Rotate until the required bearing is reached.
    Exits if error < threshold or oscillatory behaviour is detected.

    Args:
        required (float): Bearing in degrees.
        threshold (float, optional): Maximum acceptable error. Defaults to 0.4.
    Returns:
        bool: True if error < threshold
    """
    kP = 11.0
    kI = 0.90
    kD = 7.55

    def angle_between(a, b):
        return min(a - b, a - b + 360, a - b - 360, key=abs)

    error = angle_between(required, robot.bearing1(robot.compass))
    error_integral = 0
    error_derivative = 0

    while abs(error) > threshold:
        P = kP * error
        I = kI * error_integral
        D = kD * error_derivative

        v = np.clip(P + I + D, -robot.MAX_VELOCITY, robot.MAX_VELOCITY)

        # if DEBUG_PID:
            # print(f'{P=}, {I=}, {D=}, {v=}, {error=}, {error_integral=}, {error_derivative=}')

        robot.set_motor_velocities(-v, v)

        time_elapsed = robot.step()

        new_error = angle_between(required, robot.bearing1(robot.compass))

        # If more than one TIME_STEPs elapsed, then the robot had stopped due to collision prevention
        # Reset to avoid blowing up
        if np.isclose(time_elapsed, robot.TIME_STEP / 1000, atol=0.001):
            error_integral += new_error * time_elapsed
            error_derivative = (new_error - error) / time_elapsed
        else:
            if DEBUG_PID:
                print('PID_rotation resetting state due to collision detection')
            error_integral = 0
            error_derivative = 0

        # Detect oscillatory behaviour
        # On a 64 ms TIME_STEP, it sometimes oscillates between 0.39 and -0.39 deg error
        if np.isclose(error, -new_error, atol=0.1):
            if DEBUG_PID:
                print('PID_rotation halted due to oscillations')
            return False

        error = new_error

    robot.reset_motor_velocities()

    return True


@trace
def PID_translation(coord, final_error=0.15, reverse=False):
    """input: 2D desired coordinate coord,
    The function moves in a straight line until the desired location is within
    the final error distance"""
    coord = np.clip(coord, -1, 1)

    # Euclidean distance
    error = np.linalg.norm(coord - robot.current_location())
    error_integral = 0
    error_derivative = 0
    
    kP = 10.0
    kI = 0.9
    kD = 2.0
    
    
    if DEBUG_TRANSLATE:
        print("start of translation")
        start = robot._robot.getTime()

    while error > final_error:
        P = kP * error
        I = kI * error_integral
        D = kD * error_derivative
        v = np.clip(P+I+D, -robot.MAX_VELOCITY, robot.MAX_VELOCITY)

        if reverse:
            v *= -1

        robot.set_motor_velocities(v, v)

        time_elapsed = robot.step()

        new_error = np.linalg.norm(coord - robot.current_location())
        
        if np.isclose(time_elapsed, robot.TIME_STEP / 1000, atol=0.001):
            error_integral += new_error * time_elapsed
            error_derivative = (new_error - error) / time_elapsed
        else:
            if DEBUG_PID:
                print('PID_rotation resetting state due to collision detection')
            error_integral = 0
            error_derivative = 0
        
        

        # Correct bearing (returns immediately if no correction is required)
        bearing = required_bearing(coord, robot.gps.getValues())
        if reverse:
            if bearing > 0:
                bearing -= 180
            else:
                bearing += 180

        PID_rotation(bearing)
        error = new_error

    robot.reset_motor_velocities()
    if DEBUG_TRANSLATE:
        print("end of translation",robot._robot.getTime() - start)
       
    


@trace
def move(coord, error_rotation=0.5, error_translation=0.1):
    """
    move to location coord
    """
    required_angle = required_bearing(coord, robot.gps.getValues())

    PID_rotation(required_angle, error_rotation)
    PID_translation(coord, error_translation)
    return


@trace
def sweep(velocity=-0.5, swept_angle=355):
    """
    do a 180 degree spin while collecting data from distance sensor
    input: velocity of wheels/how fast is the rotation
    output: numpy array with stored values from the distance sensor
    """

    # find current rotation [0-360 degrees]
    initial_angle = robot.bearing(robot.compass1)

    # store potential boxes locations
    boxes = []

    # sweep 360 degrees
    swept_angle = 0

    while swept_angle < 355:

        robot.set_motor_velocities(-velocity, velocity)

        robot.step()

        # distance from robot centre to wall in this direction
        current_angle = robot.bearing(robot.compass1)
        current_position = robot.gps.getValues()
        wall_dist = get_wall_position(current_angle, current_position)

        # wall_dist is decreased by robot-sensor distance
        wall_dist -= 0.09

        # get quantized infrared level and convert to volts
        infrared_volts = robot.infrared_analogue.read() * robot.infrared_vref / 1023
        # get infrared reading and convert to meters
        infrared_dist = 0.7611 * math.pow(infrared_volts, -0.9313) - 0.1252

        # print(infrared_dist, wall_dist)

        # if measured distance is less than wall_dist then assume there's a box
        # also if wall is more than 1.5 away disregard measurements because it's further than sensor's range
        if abs(wall_dist - infrared_dist) > 0.09 and wall_dist < 1.4:
            valid, x, z = potential_box_position(infrared_dist + 0.09, current_angle, current_position)
            if(valid):
                boxes.append([x, z])
                

        # check if boxes are in between the robots
        if abs(wall_dist - infrared_dist) > 0.1 and wall_dist > 1.4 and abs(infrared_dist) < 0.4:
            valid, x, z = potential_box_position(infrared_dist + 0.09, current_angle, current_position)
            if(valid):
                boxes.append([x, z])
                print(x)

        if current_angle > initial_angle:
            swept_angle = current_angle - initial_angle
        else:
            swept_angle = 360 - initial_angle + current_angle

    locations = box_position(np.array(boxes))
    print(locations)
    
    robot.reset_motor_velocities()
    robot.step()

    robot.sweep_locations = locations

    return locations


@trace
def return_box_field(coord):
    """
    function that makes robot return a box in the specified field without it clashing with
    other boxes that are already in the field
    input 3D coordinates of the robot
    """

    intermediate, final = robot.field.get_to_field(coord)
    move_avoid_fields(intermediate, error_translation=0.15)
    if final[0] > 0:
        PID_rotation(-90)
    else:
        PID_rotation(90)

    move(final, error_translation=0.2)

    robot.withdraw_dualclaw()

    robot.move_forwards(-0.15, 0.02)
    return


@trace
def finish_in_field():
    """
    for the ending of the task, robot goes and stays in its field
    """

    # print('parking')

    if robot.colour == 'red':
        intermediate = (0, 1)
        final = (0, 0.4)
    else:
        intermediate = (0, -1)
        final = (0, -0.4)

    move_avoid_fields(intermediate)

    # TODO: Think of something better
    if not robot.box_queue.empty() and robot.field.available():
        return False

    if robot.colour == 'red':
        PID_rotation(180)
    else:
        PID_rotation(0)

    if not robot.box_queue.empty() and robot.field.available():
        return False

    PID_translation(final, reverse=True)

    return True


def test_collisions():
    robot.step()

    if robot.colour == 'green':
        move((-0.95, 0))

    if robot.colour == 'red':
        move((0.95, 0))


print('********')


# This part is executed

# robot.step()
# test_collisions()


robot.step()

if robot.colour == 'green':
    PID_rotation(180)
else:
    PID_rotation(0)

# Test PID_rotation
# PID_rotation(90)
# t = 0
# while t < 1:
#     t += robot.step()
# PID_rotation(-45)
# while t < 2:
#     t += robot.step()

# Test move_forwards
# robot.move_forwards(.5)
# t = 0
# while t < 1:
#     t += robot.step()
# robot.move_forwards(-.5)
# while t < 2:
#     t += robot.step()

positions = sweep(0.5)

robot.step()
robot.send_sweep_locations(positions)
robot.step()
initial_pass = True

robot.parked = False

while True:
    while not robot.box_queue.empty() and robot.field.available():
        robot.parked = False
        robot.send_message('done', 4)

        t = robot.box_queue.get()
        pos = t[1]

        robot.withdraw_dualclaw()

        if initial_pass:
            initial_pass = False
            move(pos)
        else:
            move_avoid_fields(pos, error_translation=0.1)

        robot.step()
        # if this is a new box and colour needs to be checked
        if t[0] == 0:
        
            result = robot.get_target()[0] # -1, -2 for errors, True for getting same colour, False for detecting different colour
            
            if result == -1:
                print('did not detect box')
            elif result == -2:
                print('failed to detect colour after remeasure')
            elif result:
                return_box_field(robot.gps.getValues())
            else:
                robot.move_forwards(-0.15, 0.02)
                valid, x, z = robot.remeasure_position()
                if valid:
                    robot.send_box_location(np.array([x, z]))
            # c = robot.deploy_dualclaw()

            # colour = ''

            # if c == 0:
                # colour = 'red'
                # print('detected ', colour)
            # elif c == 1:
                # colour = 'green'
                # print('detected ', colour)

            # else:
                # c = robot.remeasure()
                # robot.close_dualclaw()
                # if c == 0:
                    # colour = 'red'
                    # print('detected ', colour)
                # elif c == 1:
                    # colour = 'green'
                    # print('detected ', colour)
               
        else:  # this is a known box, got a location form another robot, just need to pick it up
            robot.close_dualclaw()
            return_box_field(robot.gps.getValues())

    if not robot.parked:
        robot.parked = finish_in_field()
        robot.send_message('parked', 4)

    # Yield if parked, otherwise Webots will be stuck waiting for us
    robot.step()
