#!/usr/bin/env python
import math
from ecl_controller import PyECLRollController, PyECLPitchController, \
    PyECLYawController
from mtecs import PyMTecs, PyLimitOverride
import sys
import numpy as np

class FixedWingController:

    """Controls a fixed wing aircraft"""

    def __init__(self, params, control_total_energy_divider, mode):
        """
        :param control_total_energy_divider run total enery controller every xth time
        """
        self.mode = mode

        # since we run on python2, make sure that all params are floats
        for p in params:
            params[p] = float(params[p])

        self.params = params

        # Attitude Control
        self.c_roll = PyECLRollController()
        self.c_pitch = PyECLPitchController()
        self.c_yaw = PyECLYawController()
        att_tc = params["att_tc"]
        k_p = params["k_p"]
        k_ff = params["k_ff"]
        k_i = params["k_i"]
        i_max = params["i_max"]
        self.c_roll.set_time_constant(att_tc)
        self.c_roll.set_k_p(k_p)
        self.c_roll.set_k_i(k_i)
        self.c_roll.set_integrator_max(i_max)
        self.c_roll.set_k_ff(k_ff)
        self.c_pitch.set_time_constant(att_tc)
        self.c_pitch.set_k_p(k_p)
        self.c_pitch.set_k_i(k_i)
        self.c_pitch.set_integrator_max(i_max)
        self.c_pitch.set_k_ff(k_ff)
        self.c_pitch.set_max_rate_pos(params["pitch_max_rate_pos"])
        self.c_pitch.set_max_rate_neg(params["pitch_max_rate_neg"])
        self.c_pitch.set_roll_ff(params["pitch_roll_ff"])
        self.c_yaw.set_time_constant(att_tc)
        self.c_yaw.set_k_p(k_p)
        self.c_yaw.set_k_i(k_i)
        self.c_yaw.set_integrator_max(i_max)
        self.c_yaw.set_k_ff(k_ff)
        self.c_yaw.set_coordinated_min_speed(params["coordinated_min_speed"])
        self.c_yaw.set_coordinated_method(params["coordinated_method"])

        # Altitude and speed control (total energy control)
        # XXX load values from params argument where it makes sense
        c_te_params = {
            "MT_ENABLED": 1,
            "MT_THR_FF": params["mtecs_throttle_ff"],
            "MT_THR_P": params["mtecs_throttle_p"],
            "MT_THR_I": params["mtecs_throttle_i"],
            "MT_THR_OFF": params["throttle_default"],
            "MT_PIT_FF": params["mtecs_pitch_ff"],
            "MT_PIT_P": params["mtecs_pitch_p"],
            "MT_PIT_I": params["mtecs_pitch_i"],
            "MT_PIT_OFF": 0.0,
            "MT_THR_MIN": 0.0,
            "MT_THR_MAX": 1.0,
            "MT_PIT_MIN": -45.0,
            "MT_PIT_MAX": 20.0,
            "MT_ALT_LP": params["mtecs_altitude_lowpass_cutoff"],
            "MT_FPA_LP": params["mtecs_flightpathangle_lowpass_cutoff"],
            "MT_FPA_P": params["mtecs_fpa_p"],
            "MT_FPA_D": 0.0,
            "MT_FPA_D_LP": 1.0,
            "MT_FPA_MIN": -20.0,
            "MT_FPA_MAX": 30.0,
            "MT_A_LP": params["mtecs_airspeed_lowpass_cutoff"],
            "MT_AD_LP": params["mtecs_airspeed_derivative_lowpass_cutoff"],
            "MT_ACC_P": params["mtecs_acc_p"],
            "MT_ACC_D": 0.0,
            "MT_ACC_D_LP": 0.5,
            "MT_ACC_MIN": -40.0,
            "MT_ACC_MAX": 40.0,
            "MT_TKF_THR_MIN": 1.0,
            "MT_TKF_THR_MAX": 1.0,
            "MT_TKF_PIT_MIN": 0.0,
            "MT_TKF_PIT_MAX": 45.0,
            "MT_USP_THR_MIN": 1.0,
            "MT_USP_THR_MAX": 1.0,
            "MT_USP_PIT_MIN": -45.0,
            "MT_USP_PIT_MAX": 0.0,
            "MT_LND_THR_MIN": 0.0,
            "MT_LND_THR_MAX": 0.0,
            "MT_LND_PIT_MIN": -5.0,
            "MT_LND_PIT_MAX": 15.0,
            "MT_THR_I_MAX": 10.0,
            "MT_PIT_I_MAX": 10.0,
            "FW_AIRSPD_MIN": params["airspeed_min"],
        }
        self.c_te = PyMTecs(c_te_params)

        self.control_count = 0
        self.control_total_energy_divider = control_total_energy_divider


    def get_ground_speed_vec(self, state):
        """Retruns a vector of the ground speed"""
        return np.array([state["speed_body_u"], state["speed_body_v"],
                         state["speed_body_w"],])

    def call_mtecs(self, state, setpoint):
        """helper function to pass the right arguments to mtecs"""
        flightpathangle = 0.0
        ground_speed = self.get_ground_speed_vec(state)
        ground_speed_length = np.linalg.norm(ground_speed)
        if ground_speed_length > sys.float_info.epsilon:
            flightpathangle = -np.arcsin(ground_speed[2]/ground_speed_length)

            limitoverride = PyLimitOverride()
            #  if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
                    #  /* Force the slow downwards spiral */
                    #  limitOverride.enablePitchMinOverride(-1.0f);
                    #  limitOverride.enablePitchMaxOverride(5.0f);

            #  } else if (climbout_mode) {
                    #  limitOverride.enablePitchMinOverride(M_RAD_TO_DEG_F * climbout_pitch_min_rad);
            #  } else {
                    #  limitOverride.disablePitchMinOverride();
            #  }
            limitoverride.disablePitchMinOverride();

            #  if (pitch_max_special) {
                    #  /* Use the maximum pitch from the argument */
                    #  limitOverride.enablePitchMaxOverride(M_RAD_TO_DEG_F * pitch_max_rad);
            #  } else {
                    #  /* use pitch max set by MT param */
                    #  limitOverride.disablePitchMaxOverride();
            #  }
            limitoverride.disablePitchMaxOverride();
            self.c_te.updateAltitudeSpeed(flightpathangle,
                                          state["altitude"],
                                          setpoint["altitude"],
                                          state["airspeed"],
                                          setpoint["velocity"],
                                          self.c_te.mtecs_mode_normal,
                                          limitoverride);
            return flightpathangle


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
            "altitude_setpoint": r["altitude"],
            "velocity_setpoint": r["velocity"],
        }
        for k,v in y.items():
            control_data[k] = v

        if self.mode == "position":
            if self.control_count % self.control_total_energy_divider == 0:
                flightpathangle = self.call_mtecs(y, r)
            control_data["pitch_setpoint"] = self.c_te.getPitchSetpoint()
            throttle = self.c_te.getThrottleSetpoint()

            # save other relevant data
            control_data["airspeed_filtered"] = self.c_te.getAirspeedLowpassState()
            control_data["altitude_filtered"] = self.c_te.getAltitudeLowpassState()
            control_data["flightpathangle"] = flightpathangle 
            control_data["flightpathangle_filtered"] = self.c_te.getFlightPathAngleLowpassState()
            control_data["airspeed_derivative_filtered"] = self.c_te.getAirspeedDerivativeLowpassState()
        else:
            throttle = self.params["throttle_default"]
        control_data["throttle_setpoint"] = throttle

        control_data["roll_rate_setpoint"] = self.c_roll.control_attitude(control_data)
        control_data["pitch_rate_setpoint"] = self.c_pitch.control_attitude(control_data)
        # control_data["yaw_rate_setpoint"] = self.c_yaw.control_attitude(control_data)
        control_data["yaw_rate_setpoint"] = 0.0 # XXX
        #  print("control data", control_data)
        aileron = self.c_roll.control_bodyrate(control_data)
        elevator = self.c_pitch.control_bodyrate(control_data)
        rudder = self.c_yaw.control_bodyrate(control_data)
        u = [aileron, elevator, rudder, throttle]
        #  print("u", u)


        return [u, control_data]
