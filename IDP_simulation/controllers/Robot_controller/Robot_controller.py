"""Robot_controller controller."""
import controller
from robot import Robot
from field import Field
from calculations import *
import numpy as np
import math

np.set_printoptions(suppress=True)

TIME_STEP = 64
COMMUNICATION_CHANNEL = 1
MAX_VELOCITY = 6.7

DEBUG_PID = False


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


def move_avoid_fields(coord, error_translation=0.1):
    """
    avoids both fields
    """
    robot.step(TIME_STEP)
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


def PID_rotation(required, threshold=0.4) -> bool:
    """Rotate until the required bearing is reached.
    Exits if error < threshold or oscillatory behaviour is detected.

    Args:
        required (float): Bearing in degrees.
        threshold (float, optional): Maximum acceptable error. Defaults to 0.4.
    Returns:
        bool: True if error < threshold
    """
    if DEBUG_PID:
        start_time = robot._robot.getTime()
        print('PID_rotation start')

    # These values are tuned for inconsistent time steps...
    kP = 0.095
    kI = 0.042735
    kD = 0.011


    # Once we fix the time step issue
    # kP = 0.114847
    # kI = 0.042735
    # kD = 1.192699

    def angle_between(a, b):
        return min(a - b, a - b + 360, a - b - 360, key=abs)

    error = angle_between(required, robot.bearing1(robot.compass))
    error_integral = 0
    error_derivative = 0

    integral_threshold = max(abs(error / 6), 4)

    while abs(error) > threshold:
        P = kP * error
        I = kI * error_integral if abs(error) > integral_threshold else 0
        D = kD * error_derivative

        if DEBUG_PID:
            print(f'{P=}, {I=}, {D=}, {error=}, {error_integral=}, {error_derivative=}')

        v = np.clip(P + I + D, -MAX_VELOCITY, MAX_VELOCITY)

        robot.left_wheel.setVelocity(-v)
        robot.right_wheel.setVelocity(v)

        time_elapsed = TIME_STEP / 1000  # TODO
        robot.step(TIME_STEP)

        new_error = angle_between(required, robot.bearing1(robot.compass))

        # If more than one TIME_STEPs elapsed, then the robot had stopped due to collision prevention
        # Reset to avoid blowing up
        if True:  # np.isclose(time_elapsed, TIME_STEP, atol=0.001):
            error_integral += new_error * time_elapsed
            error_derivative = (new_error - error) / time_elapsed
        else:
            error_integral = 0
            error_derivative = 0

        # Detect oscillatory behaviour
        # On a 64 ms TIME_STEP, it sometimes oscillates between 0.39 and -0.39 deg error
        if np.isclose(error, -new_error, atol=0.1):
            return False

        error = new_error

    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0)

    if DEBUG_PID:
        print('PID_rotation end', robot._robot.getTime() - start_time)

    return True


def PID_translation(coord, final_error=0.15, reverse=False, maxVelocity=6.7):
    """input: 2D desired coordinate coord,
    The function moves in a straight line until the desired location is within
    the final error distance"""

    error = ((coord[0] - robot.gps.getValues()[0])**2 + (coord[1] - robot.gps.getValues()[2])**2)**(1/2)

    while abs(error) > final_error or math.isnan(error):
        if math.isnan(error):
            pass

        else:

            v = error*MAX_VELOCITY*5
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


def move(coord, error_rotation=1, error_translation=0.1):
    """
    move to location coord
    """
    required_angle = required_bearing(coord, robot.gps.getValues())
    
    PID_rotation(required_angle, error_rotation)
    PID_translation(coord, error_translation)
    return


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

        robot.right_wheel.setVelocity(velocity)
        robot.left_wheel.setVelocity(-velocity)

        robot.step(TIME_STEP)

        # distance from robot centre to wall in this direction
        current_angle = robot.bearing(robot.compass1)
        current_position = robot.gps.getValues()
        wall_dist = get_wall_position(current_angle, current_position)

        # wall_dist is decreased by robot-sensor distance
        wall_dist -= 0.11

        # get quantized infrared level and convert to volts
        infrared_volts = robot.infrared_analogue.read() * robot.infrared_vref / 1023
        # get infrared reading and convert to meters
        infrared_dist = 0.7611 * math.pow(infrared_volts, -0.9313) - 0.1252

        #print(infrared_dist, wall_dist)

        # if measured distance is less than wall_dist then assume there's a box
        # also if wall is more than 1.5 away disregard measurements because it's further than sensor's range
        if abs(wall_dist - infrared_dist) > 0.1 and wall_dist < 1.4:
            valid, x, z = potential_box_position(infrared_dist + 0.11, current_angle, current_position)
            if(valid):
                boxes.append([x, z])

        # check if boxes are in between the robots
        if abs(wall_dist - infrared_dist) > 0.1 and wall_dist > 1.4 and abs(infrared_dist) < 0.5:
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
    move_avoid_fields(intermediate, error_translation=0.15)
    if final[0] > 0:
        PID_rotation(-90)
    else:
        PID_rotation(90)

    move(final, error_translation=0.2)

    robot.withdraw_dualclaw()

    reverse()
    return


def reverse():
    robot.left_wheel.setVelocity(-robot.MAX_VELOCITY)
    robot.right_wheel.setVelocity(-robot.MAX_VELOCITY)
    # reverse a little bit
    for _ in range(2):
        robot.step(TIME_STEP)
    robot.left_wheel.setVelocity(0)
    robot.right_wheel.setVelocity(0)


def finish_in_field():
    """
    for the ending of the task, robot goes and stays in its field
    """

    #print('parking')

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
    robot.step(TIME_STEP)


    if robot.colour == 'green':
        move((0.2, 0.4))
    if robot.colour == 'red':
        move((-0.2, -0.4))
print('********')


# This part is executed
r = controller.Robot()
if r.getName() == 'robot_red':
    robot = Robot(r, 'red')

else:
    robot = Robot(r, 'green')

red_field = Field('red')
green_field = Field('green')


#robot.step(TIME_STEP)
#test_collisions()





robot.step(TIME_STEP)

if robot.colour == 'green':
    PID_rotation(180)
else:
    PID_rotation(0)

positions = sweep(0.5)

robot.step(TIME_STEP)
robot.send_sweep_locations(positions)
robot.step(TIME_STEP)
initial_pass = True

parked = False

while True:
    while not robot.box_queue.empty() and robot.field.available():
        parked = False

        t = robot.box_queue.get()
        pos = t[1]
        
        robot.withdraw_dualclaw()

        if initial_pass:
            initial_pass = False
            move(pos)
        else:
            move_avoid_fields(pos, error_translation=0.1)

        robot.step(TIME_STEP)
        # if this is a new box and colour needs to be checked
        if t[0] == 0:
            c = robot.deploy_dualclaw()
            for i in range(10):
                robot.step(TIME_STEP)

            colour = ''

            if c == 0:
                colour = 'red'
                print('detected ', colour)
            elif c == 1:
                colour = 'green'
                print('detected ', colour)

            else:
                c = robot.remeasure()
                robot.close_dualclaw()
                if c == 0:
                    colour = 'red'
                    print('detected ', colour)
                elif c == 1:
                    colour = 'green'
                    print('detected ', colour)

            robot.step(TIME_STEP)

            if colour == robot.colour:
                return_box_field(robot.gps.getValues())
            else:
                robot.withdraw_dualclaw()
                reverse()
                if c == 0 or c == 1:
                    robot.step(TIME_STEP)
                    valid, x, z = robot.remeasure_position()
                    if valid:
                        robot.send_box_location(np.array([x, z]))
                    
        else:  # this is a known box, got a location form another robot, just need to pick it up
            robot.close_dualclaw()
            for i in range(10):
                robot.step(TIME_STEP)
            return_box_field(robot.gps.getValues())

    # TODO: Is this time step necessary?
    robot.step(TIME_STEP)

    if not parked:
        parked = finish_in_field()
