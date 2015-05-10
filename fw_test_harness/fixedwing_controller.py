#!/usr/bin/env python
import math
from ecl_controller import PyECLPitchController

class FixedWingController:

    """Controls a fixed wing aircraft"""

    def __init__(self):
        """Constructor"""
        self.c_pitch = PyECLPitchController()
        self.c_pitch.set_time_constant(0.1)
        self.c_pitch.set_k_p(1)

    def control(self, **kwargs):
        """
        Input: 
        output: control signal normed [-1 1]
        """
        control_data ={"pitch": 1.0, # XXX
                       "pitch_rate": 0.0, 
                       "pitch_setpoint": 0.0}
        desired_pitch_rate = self.c_pitch.control_attitude(control_data)
        print("pitch rate des {0}".format(desired_pitch_rate))
        y = kwargs["state"]
        aileron = math.sin(y["t"][-1])
        elevator = 0.0
        rudder = 0.0

        return [aileron, elevator, rudder]
