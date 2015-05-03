#!/usr/bin/env python
import math

class FixedWingController:

    """Controls a fixed wing aircraft"""

    def __init__(self):
        """Constructor"""

    def control(self, **kwargs):
        """
        Input: 
        output: control signal normed [-1 1]
        """
        y = kwargs["state"]
        aileron = math.sin(y["t"][-1])
        elevator = 0.0
        rudder = 0.0

        return [aileron, elevator, rudder]
