import controller
import numpy as np

import hardware
from calculations import *
from field import Field
from instrumentation import trace
from reroute import CollisionPreventionException


class Robot:

    TIME_STEP = 16  # in ms, must be a multiple of the simulation's time step
    COMMUNICATION_CHANNEL = 1
    DEBUG_COLLISIONS = False

    MAX_VELOCITY = 6.7

    left_wheel_name = 'left_wheel'
    right_wheel_name = 'right_wheel'
    box_claw_name = 'box_claw'
    box_claw_sensor_name = 'box_claw_sensor'
    left_claw_name = 'left_claw'
    left_claw_sensor_name = 'left_claw_sensor'
    right_claw_name = 'right_claw'
    right_claw_sensor_name = 'right_claw_sensor'
    infrared_name = 'IR Sensor'
    dsUltrasonic_name = 'ultrasonic'
    lightSensorRed_name = 'TEPT4400_RED'
    lightSensorGreen_name = 'TEPT4400_GREEN'
    emitter_name = 'emitter'
    receiver_name = 'receiver'
    gps_name = 'gps'
    compass_name = 'compass'
    compass1_name = 'compass1'

    unique_boxes = np.array([])

    throw_on_collision_prevention = False

    def __init__(self, robot: controller.Robot, colour='red'):
        """
        initialize robot and its components, colour can be green or red

        """
        self._robot = robot
        self.colour = colour
        self.field = Field(colour)
        self.infrared_vref = 4.3
        # queue of tuple (i, pos) where i is 0 if initial, 1 if added and pos is position of the box
        self.box_list = []
        self.other_box_list = []
        self.sweep_locations = np.array([])
        self.other_sweep_locations = np.array([])
        self.unique_boxes = []
        self.position = np.array([])
        self.other_position = np.array([])
        self.other_bearing = 0  # this one is 0-360
        self.current_target = []

        self.stop = False
        self.other_stop = False
        self.parked = False
        self.sweep_ready = False
        self.other_parked = False
        self.other_blocked = False
        self.carrying = False
        self.other_available = False
        self.other_sweep_ready = False
        self.second_sweep_locations_ready = False
        self.other_second_sweep_locations_ready = False

        self.left_wheel = robot.getDevice(Robot.left_wheel_name)
        self.right_wheel = robot.getDevice(Robot.right_wheel_name)
        self.box_claw = robot.getDevice(Robot.box_claw_name)
        self.left_claw = robot.getDevice(Robot.left_claw_name)
        self.right_claw = robot.getDevice(Robot.right_claw_name)

        self.box_claw_sensor = robot.getDevice(Robot.box_claw_sensor_name)
        self.left_claw_sensor = robot.getDevice(Robot.left_claw_sensor_name)
        self.right_claw_sensor = robot.getDevice(Robot.right_claw_sensor_name)

        self.infrared = robot.getDevice(Robot.infrared_name)
        self.dsUltrasonic = robot.getDevice(Robot.dsUltrasonic_name)
        self.lightsensorRed = robot.getDevice(Robot.lightSensorRed_name)
        self.lightsensorGreen = robot.getDevice(Robot.lightSensorGreen_name)
        self.emitter = robot.getDevice(Robot.emitter_name)
        self.receiver = robot.getDevice(Robot.receiver_name)
        self.gps = robot.getDevice(Robot.gps_name)
        self.compass = robot.getDevice(Robot.compass_name)
        self.compass1 = robot.getDevice(Robot.compass1_name)

        # Device.enable() takes the sampling period in milliseconds
        TIME_STEP = self.TIME_STEP
        # 38.3 ms ± 9.6 ms, choose the upper bound (worst-case scenario)
        self.infrared.enable(48)
        # 150 μs to 25 ms, 38 ms if no obstacle
        # We're only using it at small distances, so it should be well under one TIME_STEP (16 ms)
        self.dsUltrasonic.enable(TIME_STEP)
        self.box_claw_sensor.enable(TIME_STEP)
        self.left_claw_sensor.enable(TIME_STEP)
        self.right_claw_sensor.enable(TIME_STEP)
        # ATmega4809's ADC samples really quickly
        self.lightsensorRed.enable(TIME_STEP)
        self.lightsensorGreen.enable(TIME_STEP)
        self.receiver.enable(TIME_STEP)
        # Assume computer vision is close to real time
        self.gps.enable(TIME_STEP)
        self.compass.enable(TIME_STEP)
        self.compass1.enable(TIME_STEP)

        self.emitter.setChannel(Robot.COMMUNICATION_CHANNEL)
        self.receiver.setChannel(Robot.COMMUNICATION_CHANNEL)

        self.box_claw.setPosition(0.0)
        self.left_claw.setPosition(0.0)
        self.right_claw.setPosition(0.0)
        self.left_wheel.setPosition(float('inf'))
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setPosition(float('inf'))
        self.right_wheel.setVelocity(0.0)

        self.green_analogue = hardware.ADCInput(
            lambda: hardware.PhototransistorCircuit(self.lightsensorGreen).voltage())
        self.red_analogue = hardware.ADCInput(lambda: hardware.PhototransistorCircuit(self.lightsensorRed).voltage())
        self.infrared_analogue = hardware.ADCInput(lambda: self.infrared.getValue(), self.infrared_vref)

    def update_unique_boxes(self):

        duplicates = []
        if len(self.other_box_list) == 0 or len(self.box_list) == 0:
            return
        for i in range(len(self.box_list)):
            for j in range(len(self.other_box_list)):

                loc1 = np.array(self.box_list[i][1])
                loc2 = np.array(self.other_box_list[j])

                # check if loc1 and loc2 are the same or very close
                dist = np.linalg.norm(loc1 - loc2)
                if dist < 0.03:
                    duplicates.append(j)

        a = np.array([box[1] for box in self.box_list])
        unique = np.concatenate((a, np.delete(self.other_box_list, duplicates, 0)), axis=0)
        #self.unique_boxes = unique
        Robot.unique_boxes = unique
        return

    def get_unique_boxes(self):
        return Robot.unique_boxes

    def step(self, collision_detection: bool = True, read_sensors: bool = False) -> float:
        """Block for self.TIME_STEP milliseconds and do some housekeeping.

        Args:
            collision_detection (bool, optional): Whether to look for collisions. Defaults to True.

        Returns:
            float: Time in seconds elapsed or -1 if we need to terminate.
        """
        start_time = self._robot.getTime()
        ret = self._robot.step(self.TIME_STEP)
        self.send_location()
        self.send_box_list()
        self.get_messages()

        if collision_detection:
            self.collision_prevention()

        if read_sensors == True:
            self.update_box_positions()

        # self.collision_prevention() may call robot._robot.step() multiple times
        # hence, we need to measure the actual time elapsed
        elapsed_time = self._robot.getTime() - start_time  # in seconds

        # -1 is Webots telling us to terminate the process
        return -1 if ret == -1 else elapsed_time

    def update_box_positions(self):
        angle = self.bearing(self.compass1)
        position = self.gps.getValues()
        wall_dist = get_wall_position(angle, position) - 0.09
        dist = self.dsUltrasonic.getValue()

        if abs(wall_dist - dist) > 0.09 and wall_dist < 1.4:
            valid, x, z = potential_box_position(dist + 0.09, angle, position)
            # check if it's in the field
            if (x >= -0.2 and x <= 0.2) and ((z >= -0.6 and z <= -0.2) or (z >= 0.2 and z <= 0.6)):
                return
            # check that it's not the current target
            if len(self.current_target) > 0 and math.sqrt((x - self.current_target[0])**2 + (z - self.current_target[1])**2) < 0.08:
                return
            # check that it's not the other robot
            if math.sqrt((x - self.other_position[0])**2 + (z - self.other_position[1])**2) < 0.35:
                return
            # check if it's already carrying the box, then it couldn't have seen anything new
            if self.carrying:
                return
            if(valid):
                # check if it's a box that already exists in the list or it was moved by a little
                diffs = []
                if len(self.box_list) > 0:
                    for i in self.box_list:
                        diff = math.sqrt((x - i[1][0])**2 + (z - i[1][1])**2)
                        diffs.append(diff)
                        min_index = diffs.index(min(diffs))
                        if min(diffs) < 0.08:
                            self.box_list[min_index] = (0, [x, z])
                            message = "{},{}".format(x, z)
                            self.send_message(message, type=6)
                            #print('old box: ', x, z)
                            return
                # new box, add it to the queue/list or send it to another robot
                print('new box: ', x, z)
                if (self.colour == 'red' and z > 0) or (self.colour == 'green' and z <= 0):
                    self.box_queue.put((0, [x, z]))
                    self.box_list.append((0, [x, z]))
                else:
                    self.send_box_location([x, z])
        return

    def collision_prevention(self, threshold=0.7, angle_threshold=30):
        """Checks if the distance between each robot is below a certain
        threshold and stops the robot once beneath the threshold"""
        # stop it from turning and moving when it's parked
        if self.parked or self.other_parked:
            self.send_message('done', 3)
            self.send_message('done', 5)
            return
        if self.other_position.size == 0 or self.position.size == 0:
            self.send_message('done', 3)
            self.send_message('done', 5)
            return

        dist = get_distance(self.position, self.other_position)
        if dist < threshold:
            # check which robot is in the way
            required = math.degrees(np.arctan2(
                self.other_position[1] - self.position[1], self.other_position[0] - self.position[0]))
            required = (required % 360 + 90) % 360
            current = self.bearing(self.compass)

            diff = abs(required - current)
            if(diff > 180):
                diff = 360 - diff

            small_distance = False
            if diff > angle_threshold:
                if not self.distance_too_small():
                    self.send_message('done', 3)
                    self.send_message('done', 5)
                    return
                # flag this particular case
                else:
                    small_distance = True

            self.left_wheel.setVelocity(0)
            self.right_wheel.setVelocity(0)
            self.stop = True
            # wait two timesteps to actually sync both robots
            for _ in range(2):
                # check
                self._robot.step(self.TIME_STEP)
                self.send_message('stop', 3)
                self.get_messages()
                self.send_location()

            if self.stop and self.other_stop:
                if Robot.DEBUG_COLLISIONS:
                    print('both stop')

                if small_distance:
                    # deal with this specific case
                    if Robot.DEBUG_COLLISIONS:
                        print('distance too small')
                    self.move_forwards(-0.20, collision_prevention=False)
                    self.set_motor_velocities(left=-6, right=6)
                    for _ in range(30):
                        self._robot.step(Robot.TIME_STEP)

                else:
                    bearing = self.bearing(self.compass)
                    # check if there is anything close in direction +-45 of current direction
                    dist_left = obstacle_distance_at_angle(self.gps.getValues(), (bearing - 60) % 360)
                    dist_right = obstacle_distance_at_angle(self.gps.getValues(), (bearing + 60) % 360)
                    if self.colour == 'green':
                        # can be improved (2)
                        resolved = self.can_resolve_collision(dist_left, dist_right)
                        if Robot.DEBUG_COLLISIONS:
                            if resolved:
                                print('resolved')
                            else:
                                print('cannot resolve')
                        if not resolved:
                            # send help pls
                            # wait two timesteps to actually sync both robots
                            for _ in range(2):
                                # check
                                self._robot.step(self.TIME_STEP)
                                self.send_message('blocked', 5)
                                self.get_messages()
                                self.send_location()
                            if self.other_blocked:
                                # this may result in crashing into walls or going into the fields, but I don't see another solution
                                self.move_forwards(-0.10, collision_prevention=False)
                                self.set_motor_velocities(left=-3, right=3)
                                if Robot.DEBUG_COLLISIONS:
                                    print('both blocked, avoid destructively')
                            else:
                                self.wait_for_other_to_move(dist, required, current, threshold, angle_threshold)
                                self.send_message('done', 3)
                                self.send_message('done', 5)
                                return
                    else:
                        if self.other_blocked:
                            resolved = self.can_resolve_collision(dist_left, dist_right)
                            if Robot.DEBUG_COLLISIONS:
                                print('resolved')
                            if not resolved:
                                # wait two timesteps to actually sync both robots
                                for _ in range(2):
                                    # check
                                    self._robot.step(self.TIME_STEP)
                                    self.send_message('blocked', 5)
                                    self.get_messages()
                                    self.send_location()
                                if Robot.DEBUG_COLLISIONS:
                                    print('both blocked, avoid destructively')
                                # this may result in crashing into walls or going into the fields, but I don't see another solution
                                self.move_forwards(-0.10, collision_prevention=False)
                                self.set_motor_velocities(left=-3, right=3)

                        else:
                            self.wait_for_other_to_move(dist, required, current, threshold, angle_threshold)
                            self.send_message('done', 3)
                            self.send_message('done', 5)
                            return

                    self.turn_to_avoid_collision(diff, angle_threshold)
                self.set_motor_velocities(left=6.7, right=6.7)

                # move forwards until path is cleared for the other robot
                diff = self.get_angle_diff_other()
                while diff < angle_threshold:
                    diff = self.get_angle_diff_other()
                    self._robot.step(Robot.TIME_STEP)
                    self.send_location()
                    self.get_messages()
                self.reset_motor_velocities()
                if Robot.DEBUG_COLLISIONS:
                    print("moved out of the way")
            else:
                self.wait_for_other_to_move(dist, required, current, threshold, angle_threshold)
                if self.other_stop or self.other_blocked:
                    return

            self.stop = False
            self.send_message('done', 3)
            self.send_message('done', 5)

            if self.throw_on_collision_prevention:
                raise CollisionPreventionException('Robot.collision_prevention() has moved the robot, please reroute!')

        return

    def distance_too_small(self, threshold=0.25):
        """
        calculates perpendicular distance between the robots relative to its' directions
        returns True if this distance is less than 25cm
        """
        bearing1 = (self.bearing(self.compass) % 360 - 90) % 360
        bearing2 = (self.other_bearing % 360 - 90) % 360
        position1 = self.position
        position2 = self.other_position

        dist_vector = position1 - position2
        dir1_vector = np.array([math.cos(math.radians(bearing1)), math.sin(math.radians(bearing2))])
        dir2_vector = np.array([math.cos(math.radians(bearing2)), math.sin(math.radians(bearing2))])

        parallel_dist1 = np.dot(dist_vector, dir1_vector)
        parallel_dist2 = np.dot(dist_vector, dir2_vector)
        if abs(parallel_dist1) >= 0.25 or abs(parallel_dist2) >= 0.25:
            return False

        # find perpendicular distance
        bearing1 = (bearing1 + 90) % 360
        bearing2 = (bearing2 + 90) % 360
        dir1_vector = np.array([math.cos(math.radians(bearing1)), math.sin(math.radians(bearing2))])
        dir2_vector = np.array([math.cos(math.radians(bearing2)), math.sin(math.radians(bearing2))])

        perpendicular_dist1 = np.dot(dist_vector, dir1_vector)
        perpendicular_dist2 = np.dot(dist_vector, dir2_vector)
        if abs(perpendicular_dist1) >= 0.25 or abs(perpendicular_dist2) >= 0.25:
            return False
        return True

    def get_angle_diff_other(self):
        required = math.degrees(
            np.arctan2(-self.other_position[1] + self.position[1], -self.other_position[0] + self.position[0]))
        required = (required % 360 + 90) % 360
        current = self.other_bearing

        diff = abs(required - current)
        if(diff > 180):
            diff = 360 - diff
        return diff

    def wait_for_other_to_move(self, dist, required, current, threshold, angle_threshold):
        """
        helper function for collision avoidance
        waits until its cleared
        """
        self.left_wheel.setVelocity(0)
        self.right_wheel.setVelocity(0)
        self.send_message('stop', 3)
        diff = abs(required - current)
        if(diff > 180):
            diff = 360 - diff
        while dist < threshold and diff <= angle_threshold:
            if self.other_blocked or self.other_stop or self.other_parked:
                if Robot.DEBUG_COLLISIONS:
                    print("break waiting")
                return

            self._robot.step(self.TIME_STEP)
            self.send_location()
            self._robot.step(self.TIME_STEP)
            self.get_messages()

            if self.position.size == 2 and self.other_position.size == 2:
                dist = get_distance(self.position, self.other_position)
                required = math.degrees(np.arctan2(
                    self.other_position[1] - self.position[1], self.other_position[0] - self.position[0]))
                required = (required % 360 + 90) % 360
                current = self.bearing(self.compass)
                diff = abs(required - current)
                if(diff > 180):
                    diff = 360 - diff
        return

    def turn_to_avoid_collision(self, diff, angle_threshold):
        """
        helper function for collision avoidance
        turns to avoid collision
        """
        i = 0
        diff_start = diff
        while diff <= angle_threshold:
            if Robot.DEBUG_COLLISIONS:
                print('turning ', diff)
            # check that robots aren't stuck
            i += 1
            if i == 10 and abs(diff - diff_start) < 0.5:
                self.move_forwards(-0.10, collision_prevention=False)
                self.reset_motor_velocities()
                self._robot.step(Robot.TIME_STEP)
                if Robot.DEBUG_COLLISIONS:
                    print('stuck ')
                break
            self._robot.step(Robot.TIME_STEP)
            self.get_messages()
            self.send_location()

            if self.position.size == 2 and self.other_position.size == 2:
                required = math.degrees(np.arctan2(
                    self.other_position[1] - self.position[1], self.other_position[0] - self.position[0]))
                required = (required % 360 + 90) % 360
                current = self.bearing(self.compass)
                diff = abs(required - current)
                if(diff > 180):
                    diff = 360 - diff

        return

    def can_resolve_collision(self, dist_left, dist_right):
        if dist_right > 0.3:
            self.set_motor_velocities(left=-3, right=3)
            # send that you're not blocked
            self.send_message('done', 5)
            return True
        if dist_left > 0.3:
            self.set_motor_velocities(left=3, right=-3)
            # say that you're not blocked
            self.send_message('done', 5)
            return True
        return False

    def field_collision(self, coord, field):
        """
        checks if on the path between current position and coord it needs to go across field
        """

        location = self.gps.getValues()
        location = (location[0], location[2])

        m = (coord[1] - location[1])/(coord[0]-location[0])
        c = coord[1] - m*coord[0]

        x = np.linspace(min(coord[0], location[0]), max(coord[0], location[0]), 101, endpoint=True)
        z = m*x + c
        z1 = [i for i in z if (i > field.y - 0.23 and i < field.y + 0.23)]
        x1 = [i for i in x if (i > field.x - 0.23 and i < field.x + 0.23)]

        if z1 and x1:
            return True

        return False

    def find_closest_point(self, field):
        """
        helper function for avoiding the field
        input field is of type Field
        """
        location = self.gps.getValues()
        location = (location[0], location[2])
        p1 = [field.x, field.y + 0.4]
        p2 = [field.x, field.y - 0.4]
        p3 = [field.x + 0.4, field.y]
        p4 = [field.x - 0.4, field.y]
        points = [p1, p2, p3, p4]
        bearings = [180.0, 0.0, 90.0, -90.0]
        distances = []

        for point in points:
            x = (point[0] - location[0])
            z = (point[1] - location[1])
            distance = (x**2 + z**2)**0.5
            distances.append(distance)
        i = distances.index(min(distances))
        checkpoint = points[i]
        bearing = bearings[i]

        return checkpoint, bearing

    def send_message(self, message: str, type: int):
        """
        sends string message through a receiver
        input: string message, type = 0 for robot coordinates, type = 1 for a single box coordinates, type = 2 for multiple box coordinates
        return: /
        """
        message = str(type) + ';' + message
        data = message.encode('utf-8')
        self.emitter.send(data)

    def get_messages(self):
        """
        gets the first message in receiver's queue and pops it from queue
        input: /
        return: string message, int type if there aren't any messages returns empty string
        type = 0 for robot coordinates, type = 1 for a single box coordinates, type = 2 for multiple box coordinates
        """
        while self.receiver.getQueueLength() > 0:
            data = self.receiver.getData()
            message = data.decode('utf-8')
            p = message.split(';')
            type = int(p[0])
            message = p[1]
            self.receiver.nextPacket()
            if message == "":
                if type == 2:
                    self.other_sweep_locations = np.array([])
                    self.compare_sweep_results()
                    return
                else:
                    continue

            s = message.split(',')

            if type == 0:
                loc = np.array([float(s[0]), float(s[1])])
                self.other_bearing = float(s[2])
                self.other_position = loc

            elif type == 1:
                try:
                    coord = np.array([float(x) for x in s])
                    self.box_list.append((1, coord))
                except:
                    print('ERROR MESSAGE ', message)
            elif type == 2:
                coordinates = np.array([float(x) for x in s])
                coordinates = np.reshape(coordinates, (int(coordinates.size / 2), 2))
                self.other_sweep_locations = coordinates
                self.compare_sweep_results()
            elif type == 3:
                if message == 'stop':
                    self.other_stop = True
                elif message == 'done':
                    self.other_stop = False
            elif type == 4:
                if message == 'parked':
                    self.other_parked = True
                elif message == 'done':
                    self.other_parked = False
            elif type == 5:
                if message == 'blocked':
                    self.other_blocked = True
                elif message == 'done':
                    self.other_blocked = False
            elif type == 6:
                try:
                    diffs = []
                    x = float(s[0])
                    z = float(s[1])
                    for i in self.box_list:
                        diff = math.sqrt(abs(x - i[1][0])**2 + abs(z - i[1][1])*2)
                        diffs.append(diff)
                    min_index = diffs.index(min(diffs))
                    if min(diffs) < 0.05:
                        self.box_list[min_index] = (0, [x, z])
                except:
                    print('ERROR MESSAGE ', message)
            elif type == 7:
                coordinates = np.array([float(x) for x in s])
                coordinates = np.reshape(coordinates, (int(coordinates.size / 2), 2))
                self.other_box_list = coordinates
                self.update_unique_boxes()
            elif type == 8:
                if message == 'available':
                    self.other_available = True
                elif message == 'done':
                    self.other_available = False
            elif type == 9:
                if message == 'sweep ready':
                    self.other_sweep_ready = True
                elif message == 'done':
                    self.other_sweep_ready = False
            elif type == 10:
                if message == 'locations sent':
                    self.other_second_sweep_locations_ready = True
                elif message == 'done':
                    self.other_second_sweep_locations_ready = False

    def send_box_list(self):
        """
        send current box_list
        """
        message = ""
        for pos in self.box_list:
            stringpos = "{},{}".format(pos[1][0], pos[1][1])
            message += stringpos + ','

        self.send_message(message[:-1], type=7)

    @trace
    def send_sweep_locations(self, locations):
        """
        send an array of locations as one message to other robot after the sweep
        """
        if locations.size == 0:
            self.send_message("", type=2)
            return
        message = ""
        for pos in locations:
            stringpos = "{},{}".format(pos[0], pos[1])
            message += stringpos + ','

        self.send_message(message[:-1], type=2)

    def send_box_location(self, location):
        """
        send a location of one box
        """
        message = "{},{}".format(location[0], location[1])
        self.send_message(message, type=1)
        return

    def send_location(self):
        """
        send current location and bearing of the robot
        update current location
        """
        location = self.gps.getValues()
        self.position = np.array([location[0], location[2]])
        message = "{},{},{}".format(location[0], location[2], self.bearing(self.compass))
        self.send_message(message, type=0)

    @trace
    def compare_sweep_results(self):
        """
        check sweep results from both robots, remove duplicate locations
        save locations on robot's half of the table to queue starting from the closest one to the robot
        """
        if self.sweep_ready and self.other_sweep_ready:
            if not self.second_sweep_locations_ready or not self.other_second_sweep_locations_ready:
                return

        if self.sweep_locations.size == 0:
            if self.other_sweep_locations.size == 0:
                return
            else:
                Robot.unique_boxes = self.other_sweep_locations
                self.add_boxes_to_queue(self.other_sweep_locations)
                return
        if self.other_sweep_locations.size == 0:
            Robot.unique_boxes = self.sweep_locations
            self.add_boxes_to_queue(self.sweep_locations)
            return

        duplicates = []
        for i in range(self.sweep_locations.shape[0]):
            for j in range(self.other_sweep_locations.shape[0]):

                loc1 = np.array(self.sweep_locations[i])
                loc2 = np.array(self.other_sweep_locations[j])

                # check if loc1 and loc2 are the same or very close
                dist = np.linalg.norm(loc1 - loc2)
                if dist < 0.03:
                    duplicates.append(j)

        unique = np.concatenate((self.sweep_locations, np.delete(self.other_sweep_locations, duplicates, 0)), axis=0)
        #self.unique_boxes = unique
        self.add_boxes_to_queue(unique)

        if Robot.unique_boxes.size == 0:
            Robot.unique_boxes = unique

        return

    @trace
    def add_boxes_to_queue(self, positions):
        """
        assigns boxes that are in one half of the field to this robot, the other one will check the rest
        """
        for pos in positions:
            if self.colour == 'red' and pos[1] > 0:
                self.box_list.append((0, pos))
            elif self.colour == 'green' and pos[1] <= 0:
                self.box_list.append((0, pos))

        return

    @staticmethod
    def bearing1(compass: controller.Compass) -> float:
        """Get bearing from compass ([-180, 180] version)

        Args:
            compass (controller.Compass): Compass instance.

        Returns:
            float: Bearing in degrees, [-180, 180]
        """
        theta = Robot.bearing(compass)
        if theta > 180:
            theta -= 360

        return theta

    @staticmethod
    def bearing(compass: controller.Compass) -> float:
        """Get bearing from compass ([0, 360] version)

        Args:
            compass (controller.Compass): Compass instance.

        Returns:
            float: Bearing in degrees, [0, 360]
        """
        values = compass.getValues()
        theta = np.arctan2(values[0], values[2])
        theta = np.rad2deg(theta) - 90

        # Unlike C/C++, Python's modulus operator returns a number with the same sign as the divisor
        return theta % 360

    def field_position(self):
        """
        returns x, z coordinates of a field where the boxes should be put
        field is by default a 0.4x0.4 square and x, z marks the centre of the square
        """
        if self.colour == 'red':
            return (0, 0.4)
        return (0, -0.4)

    def return_box_to_field(self, coord):
        """
        returns a set of 4 locations in a red/green field to put the boxes
        input 3d coordinates of robot
        """
        intermediate, final = self.field.get_to_field(coord)
        return intermediate, final

    @trace
    def deploy_dualclaw(self):
        """
        step through multiple time steps,
        closes dual claw and simultaneously attempts to detect the color of the box it is holding.
        returns 0 if detected red, 1 if detected green, 2 if detected neither, 3 if detected both.
        """
        claw1 = self.left_claw
        claw2 = self.right_claw
        sensor1 = self.left_claw_sensor
        sensor2 = self.right_claw_sensor
        desired = -1*np.pi/180  # slightly less than 0 to cut through the box to keep it in place
        error = abs(desired - sensor1.getValue())
        accuracy = 1*np.pi/180  # accuracy value in degrees
        previous = 100  # arbitrary value just serves as placeholder
        count = 0  # start counting for each time frame where the servo angle does not change, break loop upon reaching 3
        red = False
        green = False
        redLowerBound = 948  # (environment is 930),one reading above this value turns red to True
        greenLowerBound = 436  # (environment is 418), values are about 0.5 lux above ambient

        i = 0
        while error > accuracy:
            redValue = self.red_analogue.read()
            greenValue = self.green_analogue.read()
            if redValue > redLowerBound:
                red = True
            if greenValue > greenLowerBound:
                green = True
            measurement = sensor1.getValue()
            claw1.setPosition(desired)  # both claw move synchronously in different direction
            claw2.setPosition(-desired)
            self.step()
            error = abs(desired - sensor1.getValue())
            # TODO: Jerry please fix
            i += 1
            if i > 8:
                break

        if red and not green:
            # print('red')
            return 0
        elif green and not red:
            # print('green')
            return 1
        elif not green and not red:
            # print('not detected')
            return 2
        if red and green:
            # print('bad result')
            return 3

    def remeasure_position(self):
        """
        when the box was touched to check colour, this returns the new (possibly changed) position
        """
        dist = self.dsUltrasonic.getValue()
        pos = potential_box_position(dist + 0.09, self.bearing(self.compass1), self.gps.getValues())
        # print(pos)
        return pos

    @trace
    def withdraw_dualclaw(self):
        """steps through multiple time steps, opens dual claw
        """
        claw1 = self.left_claw
        claw2 = self.right_claw
        sensor1 = self.left_claw_sensor
        sensor2 = self.right_claw_sensor

        desired = 30*np.pi/180  # arbitrary value
        error = abs(desired - sensor1.getValue())
        accuracy = 1*np.pi/180  # accuracy value in degrees
        while error > accuracy:
            measurement = sensor1.getValue()
            claw1.setPosition(desired)  # both claw move synchronously in different direction
            claw2.setPosition(-desired)
            self.step()
            error = abs(desired - sensor1.getValue())

    @trace
    def remeasure(self):
        """steps through multiple time steps, called when deploy_dualclaw doesn't return right value
        checks first if there is a box within reach with ultrasonic sensor, return -1 when there isn't,
        if there is a box within reach,
        goes back and forth in attempt to remeasure color
        returns 0 if detected red, 1 if detected green, 2 if detected neither, 3 if detected both.
        """
        claw1 = self.left_claw
        claw2 = self.right_claw
        sensor1 = self.left_claw_sensor
        sensor2 = self.right_claw_sensor
        wheel1 = self.left_wheel
        wheel2 = self.right_wheel
        openAngle = 10*np.pi/180
        red = False
        green = False
        redLowerBound = 948  # (environment is 930),one reading above this value turns red to True
        greenLowerBound = 436  # (environment is 418), values are about 0.5 lux above ambient
        reverseDistance1 = 0.05  # reverse this distance with box in case close to walls
        reverseDistance2 = 0.05  # reverse this distance without box
        advanceDistance3 = 0.1  # advance this distance without box
        moveVelocity = 0.5

        def reached(distance, startPosition):
            return np.sqrt((self.position[0]-startPosition[0])**2 + (self.position[1]-startPosition[1])**2) > distance

        position1 = self.position
        while not reached(reverseDistance1, position1):
            # Reverse with box incase close to walls
            wheel1.setVelocity(-moveVelocity)
            wheel2.setVelocity(-moveVelocity)
            redValue = self.red_analogue.read()
            greenValue = self.green_analogue.read()
            if redValue > redLowerBound:
                red = True
            if greenValue > greenLowerBound:
                green = True
            self.step()

        position2 = self.position
        while not reached(reverseDistance2, position2):
            # Release the box and move backwards, while doing color detection
            claw1.setPosition(openAngle)
            claw2.setPosition(-openAngle)
            wheel1.setVelocity(-moveVelocity)
            wheel2.setVelocity(-moveVelocity)
            redValue = self.red_analogue.read()
            greenValue = self.green_analogue.read()
            if redValue > redLowerBound:
                red = True
            if greenValue > greenLowerBound:
                green = True
            self.step()

        position3 = self.position
        while not reached(advanceDistance3, position3):
            # Move forwards and do color detection
            wheel1.setVelocity(moveVelocity)
            wheel2.setVelocity(moveVelocity)
            redValue = self.red_analogue.read()
            greenValue = self.green_analogue.read()
            if redValue > redLowerBound:
                red = True
            if greenValue > greenLowerBound:
                green = True
            self.step()

        # print('remeasured:')
        if red and not green:
            # print('red')
            return 0
        elif green and not red:
            # print('green')
            return 1
        elif not green and not red:
            # print('not detected')
            return 2
        elif red and green:
            # print('bad result')
            return 3

    @trace
    def close_dualclaw(self):
        """
        step through multiple time steps,
        closes dual claw
        """
        claw1 = self.left_claw
        claw2 = self.right_claw
        sensor1 = self.left_claw_sensor
        sensor2 = self.right_claw_sensor
        desired = -1*np.pi/180  # minus value should not be reached, break loop when count reaches 3
        error = abs(desired - sensor1.getValue())
        accuracy = 1*np.pi/180  # accuracy value in degrees
        previous = 100  # arbitrary value just serves as placeholder
        count = 0  # start counting for each time frame where the servo angle does not change, break loop upon reaching 3

        while error > accuracy:
            measurement = sensor1.getValue()
            claw1.setPosition(desired)  # both claw move synchronously in different direction
            claw2.setPosition(-desired)
            self._robot.step(self.TIME_STEP)
            error = abs(desired - sensor1.getValue())

    def get_target(self):
        """combine individual functions, step through multiple time steps,
        start from deploying claws and remeasure if needed, print colour detected,
        compared colour with the colour it is supposed to collect,
        and returns the following:
        return -1 if no boxes were founded at the position
        return -2 if failed to get colour after remeasure
        otherwise, return if it is right color
        (close dualclaw if it is right colour, if not, open dualclaw and move backwards by 0.1)"""

        c = self.deploy_dualclaw()
        if c != 0 and c != 1:
            if self.dsUltrasonic.getValue() > 0.15:
                # check ultrasonic sensor whether the box is present, if not present, return -1
                return -1

            c = self.remeasure()

        if c == 0:
            colour = 'red'
            print('detected', colour)
        elif c == 1:
            colour = 'green'
            print('detected', colour)

        else:
            # return -2 if failed to measure after remeasure function is called
            return -2

        if colour == self.colour:
            self.close_dualclaw()
            return True
        else:
            self.withdraw_dualclaw()
            self.move_forwards(-0.1)
            return False

    def get_next_target(self):

        currentLocation = np.array(self.position)
        nearestLocation = None
        nearestLocationIndex = 0
        nearestDistance = 10  # place holder
        for n, item in enumerate(self.box_list):
            if np.linalg.norm(item[1] - currentLocation) < nearestDistance:
                nearestDistance = np.linalg.norm(item[1] - currentLocation)
                nearestLocation = item
                nearestLocationIndex = n

        return self.box_list.pop(nearestLocationIndex)

    def set_motor_velocities(self, left: float, right: float):
        """Sets motor velocities to the values specified.

        Args:
            left (float): Left motor velocity (rad/s).
            right (float): Right motor velocity (rad/s).
        """
        self.left_wheel.setVelocity(left)
        self.right_wheel.setVelocity(right)

    def reset_motor_velocities(self):
        """Sets motor velocities to 0.
        """
        self.set_motor_velocities(0, 0)

    def current_location(self) -> np.ndarray:
        """Returns the 2D GPS coordinates of the robot.

        Returns:
            np.ndarray: 2D GPS coordinates.
        """
        values = self.gps.getValues()
        return np.asarray_chkfinite([values[0], values[2]])

    @trace
    def move_forwards(self, distance: float, threshold: float = 0.01, collision_prevention: bool = True) -> bool:
        """Moves forwards (or backwards if distance is negative) in a straight line.
        May exit early if it gets stuck (e.g. on a wall).

        Args:
            distance (float): Distance to move.
            threshold (float, optional): Maximum error. Defaults to 0.01.
            collision_prevention (bool, optional): Run collision prevention when Robot.step() is called. Defaults to True.

        Returns:
            bool: True if error < threshold.
        """
        assert np.isfinite(distance)
        assert threshold > 0

        start = self.current_location()
        error = abs(distance)

        while error > threshold:
            # TODO: Integral & Derivative control
            v = np.clip(error * self.MAX_VELOCITY * 5, -self.MAX_VELOCITY, self.MAX_VELOCITY)

            v *= np.sign(distance)

            self.set_motor_velocities(v, v)

            # TODO: Check if the collision detection algorithm made us move
            self.step(collision_prevention)

            previous_error = error
            # Euclidean distance
            error = abs(distance) - np.linalg.norm(start - self.current_location())

            # Check that we aren't stuck
            if np.isclose(previous_error, error):
                print('Robot.move_forwards() halted due to a collision')
                return error < threshold

        self.reset_motor_velocities()
        return True
