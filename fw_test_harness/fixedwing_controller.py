#!/usr/bin/env python
import math
from ecl_controller import PyECLRollController, PyECLPitchController, \
    PyECLYawController

class FixedWingController:

    """Controls a fixed wing aircraft"""

    def __init__(self, params):
        """Constructor"""
        self.c_roll = PyECLRollController()
        self.c_pitch = PyECLPitchController()
        self.c_yaw = PyECLYawController()
        att_tc = params["att_tc"]
        k_p = params["k_p"]
        k_ff = params["k_ff"]
        k_i = params["k_i"]
        self.c_roll.set_time_constant(att_tc)
        self.c_roll.set_k_p(k_p)
        self.c_roll.set_k_i(k_i)
        self.c_roll.set_k_ff(k_ff)
        self.c_pitch.set_time_constant(att_tc)
        self.c_pitch.set_k_p(k_p)
        self.c_pitch.set_k_i(k_i)
        self.c_pitch.set_k_ff(k_ff)
        self.c_pitch.set_max_rate_pos(params["pitch_max_rate_pos"])
        self.c_pitch.set_max_rate_neg(params["pitch_max_rate_neg"])
        self.c_pitch.set_roll_ff(params["pitch_roll_ff"])
        self.c_yaw.set_time_constant(att_tc)
        self.c_yaw.set_k_p(k_p)
        self.c_yaw.set_k_i(k_i)
        self.c_yaw.set_k_ff(k_ff)
        self.c_yaw.set_coordinated_min_speed(params["coordinated_min_speed"])
        self.c_yaw.set_coordinated_method(params["coordinated_method"])

    def control(self, **kwargs):
        """
        Input:
        output: control signal normed [-1 1]
        """
        y = kwargs["state"]
        r = kwargs["setpoint"]
        params = kwargs["parameters"]
        control_data = {
            "roll_setpoint": r["roll"],
            "pitch_setpoint": r["pitch"],
            "yaw_setpoint": r["yaw"],
            "roll_rate_setpoint": r["roll_rate"],
            "pitch_rate_setpoint": r["pitch_rate"],
            "yaw_rate_setpoint": r["yaw_rate"],
            "airspeed_min": params["airspeed_min"],
            "airspeed_max": params["airspeed_max"],
        }
        for k,v in y.items():
            control_data[k] = v

        control_data["roll_rate_setpoint"] = self.c_roll.control_attitude(control_data)
        control_data["pitch_rate_setpoint"] = self.c_pitch.control_attitude(control_data)
        # control_data["yaw_rate_setpoint"] = self.c_yaw.control_attitude(control_data)
        control_data["yaw_rate_setpoint"] = 0.0 # XXX
        print("control data", control_data)
        aileron = self.c_roll.control_bodyrate(control_data)
        elevator = self.c_pitch.control_bodyrate(control_data)
        rudder = self.c_yaw.control_bodyrate(control_data)
        u = [aileron, elevator, rudder]
        print("u", u)

        return [u, control_data]
