import queue

import controller
import numpy as np

import hardware
from calculations import *
from field import Field


class Robot:

    TIME_STEP = 64
    COMMUNICATION_CHANNEL = 1

    MAX_VELOCITY = 6

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

    def __init__(self, robot: controller.Robot, colour='red'):
        """
        initialize robot and its components, colour can be green or red

        """
        self._robot = robot
        self.colour = colour
        self.field = Field(colour)
        self.infrared_vref = 4.3
        # queue of tuple (i, pos) where i is 0 if initial, 1 if added and pos is position of the box
        self.box_queue = queue.Queue()
        self.sweep_locations = []
        self.other_sweep_locations = []
        self.position = np.array([])
        self.other_position = np.array([])
        self.stop = False
        self.other_stop = False

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

        TIME_STEP = Robot.TIME_STEP
        self.infrared.enable(TIME_STEP)
        self.dsUltrasonic.enable(TIME_STEP)
        self.box_claw_sensor.enable(TIME_STEP)
        self.left_claw_sensor.enable(TIME_STEP)
        self.right_claw_sensor.enable(TIME_STEP)
        self.lightsensorRed.enable(TIME_STEP)
        self.lightsensorGreen.enable(TIME_STEP)
        self.receiver.enable(TIME_STEP)
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

    def step(self, TIME_STEP: int) -> float:
        """Block for the time requested and do some housekeeping.

        Args:
            TIME_STEP (int): Time to block in milliseconds, must be a multiple of the simulation's time step.

        Returns:
            float: seconds actually elapsed or -1 if we need to terminate
        """
        start_time = self._robot.getTime()

        ret = self._robot.step(TIME_STEP)
        self.send_location()
        self.get_messages()
        self.collision_prevention()

        # self.collision_prevention() may call robot._robot.step() multiple times
        # hence, we need to measure the actual time elapsed
        elapsed_time = self._robot.getTime() - start_time  # in seconds

        # -1 is Webots telling us to terminate the process
        return -1 if ret == -1 else elapsed_time

    def collision_prevention(self, threshold=0.75):
        """Checks if the distance between each robot is below a certain
        threshold and stops the robot once beneath the threshold"""
        
        if self.other_position.size == 0 or self.position.size == 0:
            return
        else:
            
            dist = get_distance(self.position, self.other_position)
            if dist < threshold:
                # check which robot is in the way
                required = math.degrees(np.arctan2(self.other_position[1] - self.position[1], self.other_position[0] - self.position[0]))
                required = (required % 360 + 90) % 360
                current = self.bearing1(self.compass) % 360
                
                diff = abs(required - current)
                if(diff > 180):
                    diff = 360 - diff
                
                if diff > 30:
                    return
                
                self.left_wheel
                self.stop = True
                self._robot.step(Robot.TIME_STEP)
                self.send_message('stop', 3)
                self.get_messages()
                self.send_location()
                    
                if self.stop and self.other_stop:
                    self.left_wheel.setVelocity(-Robot.MAX_VELOCITY)
                    self.right_wheel.setVelocity(-Robot.MAX_VELOCITY)
                    # reverse a little bit
                    for _ in range(2):
                        self._robot.step(Robot.TIME_STEP)

                    self.left_wheel.setVelocity(-3)
                    self.right_wheel.setVelocity(3)
                    
                    while diff <= 30:  
                        print('turn')
                        self._robot.step(Robot.TIME_STEP)
                        self.get_messages()
                        self.send_location()
                        
                        required = math.degrees(np.arctan2(self.other_position[1] - self.position[1], self.other_position[0] - self.position[0]))
                        required = (required % 360 + 90) % 360
                        current = self.bearing1(self.compass) % 360
                        diff1 = abs(required - current)
                        if(diff1 > 180):
                            diff = 360 - diff1
                        else:
                            diff = diff1
                        
                        
                    self.left_wheel.setVelocity(6.7)
                    self.right_wheel.setVelocity(6.7)
                    for _ in range(5):
                        self._robot.step(Robot.TIME_STEP)
                    self.left_wheel.setVelocity(0)
                    self.left_wheel.setVelocity(0)
                    self._robot.step(Robot.TIME_STEP)
                     
                else:
   
                    while dist < threshold and abs(required - current) <= 40:
                        self.left_wheel.setVelocity(0)
                        self.right_wheel.setVelocity(0)
                        
                        self._robot.step(Robot.TIME_STEP)
                        
                        self.send_location()
                        self.send_message('stop', 3)
                        self._robot.step(Robot.TIME_STEP)
                        self.get_messages()
    
                        if self.position.size == 2 and self.other_position.size == 2:
                            dist = get_distance(self.position, self.other_position)
                            required = math.degrees(np.arctan2(self.other_position[1] - self.position[1], self.other_position[0] - self.position[0]))
                            required = (required % 360 + 90) % 360
                            current = self.bearing1(self.compass) % 360
                self.stop = False
                self.send_message('done', 3)        



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
        z1 = [i for i in z if (i > field.y - 0.2 and i < field.y + 0.2)]
        x1 = [i for i in x if (i > field.x - 0.2 and i < field.x + 0.2)]

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
        message = str(type) + message
        data = message.encode('utf-8')
        # TODO: Remove
        self._robot.step(Robot.TIME_STEP)
        self.emitter.send(data)
        return

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
            type = int(message[0])
            message = message[1:]
            # TODO: Remove
            self._robot.step(Robot.TIME_STEP)
            self.receiver.nextPacket()
            if message == "":
                continue

            s = message.split(',')

            if type == 0:
                loc = np.array([float(s[0]), float(s[1])])
                self.other_position = loc

            elif type == 1:
                try:
                    coord = np.array([float(x) for x in s])
                    self.box_queue.put((1, coord))
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
        return

    def send_sweep_locations(self, locations):
        """
        send an array of locations as one message to other robot after the sweep
        """
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
        send current location of the robot
        """
        # TODO: Remove
        self._robot.step(Robot.TIME_STEP)
        location = self.gps.getValues()
        self.position = np.array([location[0], location[2]])
        message = "{},{}".format(location[0], location[2])
        self.send_message(message, type=0)

    def compare_sweep_results(self):
        """
        check sweep results from both robots, remove duplicate locations
        save locations on robot's half of the table to queue starting from the closest one to the robot
        """

        duplicates = []

        for i in range(self.sweep_locations.shape[0]):
            for j in range(self.other_sweep_locations.shape[1]):

                loc1 = np.array(self.sweep_locations[i])
                loc2 = np.array(self.other_sweep_locations[j])

                # check if loc1 and loc2 are the same or very close
                dist = np.linalg.norm(loc1 - loc2)
                if dist < 0.03:
                    duplicates.append(j)

        unique = np.concatenate((self.sweep_locations, np.delete(self.other_sweep_locations, duplicates, 0)), axis=0)
        self.add_boxes_to_queue(unique)
        return

    def add_boxes_to_queue(self, positions):
        """
        assigns boxes that are in one half of the field to this robot, the other one will check the rest
        """
        for pos in positions:
            if self.colour == 'red' and pos[1] > 0:
                self.box_queue.put((0, pos))
            elif self.colour == 'green' and pos[1] <= 0:
                self.box_queue.put((0, pos))
        return

    def bearing1(self, compass_obj):
        """
        This gives a bearing -180,180
        input: compass object(type Compass)
        """

        theta = np.arctan2(compass_obj.getValues()[0], compass_obj.getValues()[2])
        theta = (theta-(np.pi / 2.0))*180.0/np.pi
        if theta < -180:
            theta += 360
        return theta

    def bearing(self, compass_obj):
        """
        This gives a bearing 0,360
        input: compass object(type Compass)
        """
        theta = np.arctan2(compass_obj.getValues()[0], compass_obj.getValues()[2])
        theta = (theta-(np.pi / 2.0))*180.0/np.pi
        if theta < 0:
            theta += 360

        return theta

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
            self.step(Robot.TIME_STEP)
            error = abs(desired - sensor1.getValue())

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

        return pos

    def withdraw_dualclaw(self):
        """steps through multiple time steps, opens dual claw
        """
        claw1 = self.left_claw
        claw2 = self.right_claw
        sensor1 = self.left_claw_sensor
        sensor2 = self.right_claw_sensor

        desired = 20*np.pi/180  # arbitrary value
        error = abs(desired - sensor1.getValue())
        accuracy = 1*np.pi/180  # accuracy value in degrees
        while error > accuracy:
            measurement = sensor1.getValue()
            claw1.setPosition(desired)  # both claw move synchronously in different direction
            claw2.setPosition(-desired)
            self.step(Robot.TIME_STEP)
            error = abs(desired - sensor1.getValue())

    def remeasure(self):
        """steps through multiple time steps, called when deploy_dualclaw doesn't return right value
        checks first if there is a box within reach with ultrasonic sensor, return -1 when there isn't,
        if there is a box within reach,
        goes back and forth in attempt to remeasure color
        returns 0 if detected red, 1 if detected green, 2 if detected neither, 3 if detected both.
        """
        if self.dsUltrasonic.getValue() > 0.15:
            #print('no box within reach')
            return -1
            # check if there is box within reach, return -1 if there isn't

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

        for n in range(5):
            # Reverse with box incase close to walls
            wheel1.setVelocity(-0.3)
            wheel2.setVelocity(-0.3)
            redValue = self.red_analogue.read()
            greenValue = self.green_analogue.read()
            if redValue > redLowerBound:
                red = True
            if greenValue > greenLowerBound:
                green = True
            self.step(Robot.TIME_STEP)

        for n in range(10):
            # Release the box and move backwards, while doing color detection
            claw1.setPosition(openAngle)
            claw2.setPosition(-openAngle)
            wheel1.setVelocity(-0.3)
            wheel2.setVelocity(-0.3)
            redValue = self.red_analogue.read()
            greenValue = self.green_analogue.read()
            if redValue > redLowerBound:
                red = True
            if greenValue > greenLowerBound:
                green = True
            self._robot.step(Robot.TIME_STEP)

        for n in range(20):
            # Move forwards and do color detection
            wheel1.setVelocity(0.3)
            wheel2.setVelocity(0.3)
            redValue = self.red_analogue.read()
            greenValue = self.green_analogue.read()
            if redValue > redLowerBound:
                red = True
            if greenValue > greenLowerBound:
                green = True
            self.step(Robot.TIME_STEP)

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
            self._robot.step(Robot.TIME_STEP)
            error = abs(desired - sensor1.getValue())
