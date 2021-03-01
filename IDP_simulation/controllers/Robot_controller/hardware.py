"""Emulate hardware connected to the ATmega4809 microprocessor"""


from random import randint
from typing import Callable


class ADCInput:
    def __init__(self, voltage_callback: Callable[[], float], Vref: float, accuracy: int):
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
        """Read a voltage and emulate noise

        Returns:
            int: 10-bit voltage, [0, 1023]
        """
        voltage = self.voltage_callback()

        assert 0 <= voltage <= self.Vref

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

        return reading


class PhototransistorCircuit:
    def __init__(self, device):
        """Simulates the circuit connected to the TEPT4400 (i.e. a 24 kΩ resistor connected to the Collector)


        Args:
            device (Webots Device handle): TEPT4400 device, make sure it has been enabled!
        """
        # TODO: type checking
        self.device = device

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
        # 24 kΩ resistor
        return self.current() * 24000
