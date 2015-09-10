#!/usr/bin/env python
import numpy as np

""" Performs post-simulation analysis"""

def analyse(simulator):
    """returns a dict of analysis results"""
    res = {}

    # throttle
    throttle = np.array(simulator.control_data_log["throttle_setpoint"])
    throttle_diff = np.diff(throttle) 
    res["throttle diff coefficient of variance"] = np.std(throttle_diff)/np.abs(np.mean(throttle_diff))

    # pitch sp
    pitch_sp = np.array(simulator.control_data_log["pitch_setpoint"])
    pitch_sp_diff = np.diff(pitch_sp) 
    res["pitch sp diff coefficient of variance"] = np.std(pitch_sp_diff)/np.abs(np.mean(pitch_sp_diff))

    return res



