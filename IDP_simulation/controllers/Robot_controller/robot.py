import controller
from field import Field
import numpy as np


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

    
    
    def __init__(self, robot: controller.Robot, colour = 'red'):
        """
        initialize robot and its components, colour can be green or red
        
        """
        self._robot = robot
        self.colour = colour
        self.field = Field(colour)
        
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
    
    def step(self, TIME_STEP):
        self._robot.step(TIME_STEP)
    
        
    def send_message(self, message: str):
        """
        sends string message through a receiver
        input: string message
        return: /
        """
        data = message.encode('utf-8')
        self.emitter.send(data)
        return
    
    
    
    def get_message(self):
        """
        gets the first message in receiver's queue and pops it from queue
        input: /
        return: string message, if there aren't any messages returns empty string
        """        
        if self.receiver.getQueueLength() > 0:
            data = self.receiver.getData()
            message = data.decode('utf-8')
            self.receiver.nextPacket()
            return message
        
        return ""
        
        
    def measureLight(self):
        # print(lightsensorRed.getValue(),lightsensorGreen.getValue())
        return [self.lightsensorRed.getValue(), self.lightsensorGreen.getValue()]

    
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


