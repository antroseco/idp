"""Emulate hardware connected to the ATmega4809 microprocessor"""


from random import randint
from typing import Callable


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
        Voltages below 0 V are mapped to 0 and volatages above Vref are mapped to 1023.

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


class PhototransistorCircuit:
    def __init__(self, device):
        """Simulates the circuit connected to the TEPT4400 (i.e. a 5 kΩ resistor connected to the Collector).
        Outputs ~400 mV at 40 lux so the internal 550 mV ADC Vref is recommended.

        Args:
            device (Webots Device handle): TEPT4400 device, make sure it has been enabled!
        """
        # TODO: type checking
        self.device = device
        self.r_load = 5000  # 5 kΩ load resistor

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
