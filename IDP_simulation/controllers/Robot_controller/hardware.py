"""Emulate hardware connected to the ATmega4809 microprocessor

Usage:

# Initialize sensor
device = robot.getDevice('TETP4400')
device.enable(TIME_STEP)

# Initialize circuit and inputs
circuit = PhototransistorCircuit(device)
analogue_input = ADCInput(lambda: circuit.voltage())
digital_input  = DigitalInput(lambda: circuit.voltage(), 0.45)

# Get sensor values
while robot.step(TIME_STEP) != -1:
    analogue_value = analogue_input.read()
    digital_value  = digital_input.read()

"""

from random import randint
from typing import Callable

from controller import LightSensor


def clamp(value, low, high):
    return max(low, min(value, high))


class ADCInput:
    def __init__(self, voltage_callback: Callable[[], float], Vref: float = 0.55, accuracy: int = 2):
        """Emulates a 10-bit ADC input connected to the ATmega4809

        Args:
            voltage_callback (Callable[[], float]): returns the current voltage at the input pin, should not exceed Vref
            Vref (float): reference voltage, can be Vcc or one of the internal reference voltages specified in the datasheet
            accuracy (int): total accuracy in LSBs
        """
        assert callable(voltage_callback)
        assert Vref > 0
        assert 0 <= accuracy < 10

        self.voltage_callback = voltage_callback
        self.Vref = Vref
        self.accuracy = accuracy

    def read(self) -> int:
        """Read a voltage and emulate noise.
        Voltages below 0 V are mapped to 0 and voltages above Vref are mapped to 1023.

        Returns:
            int: 10-bit voltage, [0, 1023]
        """
        voltage = self.voltage_callback()

        # As per datasheet
        reading = int(voltage * 1023 / self.Vref)

        # Emulate absolute accuracy
        if self.accuracy > 0:
            ones = 2**self.accuracy - 1  # e.g. 0b111 for 3 bits
            LSBs = randint(0, ones)
            # Mask LSBs
            reading &= ~ones
            # Superpose inaccuracies
            reading |= LSBs

        # Clamp to 10 bits
        return clamp(reading, 0, 1023)


class DigitalInput:
    def __init__(self, voltage_callback: Callable[[], float], Vref: float):
        """Emulates a comparator connected to a digital input of the ATmega4809.

        Args:
            voltage_callback (Callable[[], float]): returns the current voltage at the input pin
            Vref (float): reference voltage, normally set with a potentiometer
        """
        assert callable(voltage_callback)
        assert Vref > 0

        self.voltage_callback = voltage_callback
        self.Vref = Vref

    def read(self) -> bool:
        """Read a voltage and emulate the comparator.

        Returns:
            bool: true if voltage > Vref
        """
        voltage = self.voltage_callback()

        # Emulate comparator
        return voltage > self.Vref


class PhototransistorCircuit:
    def __init__(self, device: LightSensor):
        """Simulates the circuit connected to the TEPT4400 (i.e. a 10 kΩ resistor connected to the Collector).
        Outputs ~500 mV at 26 lux so the internal 550 mV ADC Vref is recommended.
        Red   filter --> ~25 lux (ambient)
        Green filter --> ~11 lux (ambient)

        Args:
            device (Webots Device handle): TEPT4400 device, make sure it has been enabled!
        """
        assert isinstance(device, LightSensor)

        self.device = device
        self.r_load = 10000  # 10 kΩ load resistor

    def current(self) -> float:
        """Returns the output current

        Returns:
            float: output current [A]
        """
        # Convert μA to A
        return self.device.getValue() * 1e-6

    def voltage(self) -> float:
        """Returns the output voltage

        Returns:
            float: output voltage [V]
        """
        illuminance = self.device.getValue()  # in lux

        # Approximate relationship derived from the datasheet
        return (2e-6 * self.r_load * illuminance) / (1 + 1.04e-7 * self.r_load * illuminance)
