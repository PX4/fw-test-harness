#!/usr/bin/env python
from __future__ import print_function, division, absolute_import, \
    unicode_literals
import os
import sys
from jsbsim import FGFDMExec
import matplotlib.pyplot as plt
import argparse
from pint import UnitRegistry
from html_report_generator import HtmlReportGenerator
from fixedwing_controller import FixedWingController

ureg = UnitRegistry()


class Simulator:

    """Simulate mtecs"""

    def __init__(self, args):
        """Constructor"""
        self.args = args

        self.fdm = FGFDMExec(root_dir=args["jsbsim_root"])
        self.fdm.load_model("Rascal110-JSBSim")

        # settings
        self.sim_end_time_s = 120
        self.dt = 0.005
        self.dt_total_energy = 0.02
        self.ic = {
            "hgt": 400 * ureg.meter
        }
        #  self.mode = "attitude"
        self.mode = "position"

        self.parameters = {
            "airspeed_trim": 20.0,
            "airspeed_min": 7.0,
            "airspeed_max": 60.0,
            "coordinated_min_speed": 1000.0,
            "coordinated_method": 0.0,
            "att_tc": 0.5,
            "k_p": 0.08,
            "k_ff": 0.4,
            "k_i": 0.05,
            "i_max": 0.4,
            "pitch_max_rate_pos": 0.0, # 0: disable
            "pitch_max_rate_neg": 0.0, # 0: disable
            "pitch_roll_ff": 0.0,
            "throttle_default": 0.1,
            "mtecs_acc_p": 0.01,
            "mtecs_fpa_p": 0.01,
            "mtecs_throttle_ff": 0.0,
            "mtecs_throttle_p": 0.1,
            "mtecs_throttle_i": 0.25,
            "mtecs_pitch_ff": 0.0,
            "mtecs_pitch_p": 0.1,
            "mtecs_pitch_i": 0.03,
        }
        self.control_surface_scaler = 1.0

        self.controller = FixedWingController(self.parameters, self.dt_total_energy/self.dt, self.mode)

    def init_sim(self):
        """init/reset simulation"""

        # init states (dictionary of lists (each list contains a time series of
        # a state/value))
        self.jsbs_states = {
            "ic/gamma-rad": [0],
            "position/h-sl-meters": [self.ic["hgt"].magnitude],
            "attitude/phi-rad": [0],
            "velocities/p-rad_sec": [0],
            "attitude/theta-rad": [0],
            "velocities/q-rad_sec": [0],
            "attitude/psi-rad": [0],
            "velocities/r-rad_sec": [0],
            "velocities/u-fps": [0],
            "velocities/v-fps": [0],
            "velocities/w-fps": [0],
            "accelerations/udot-ft_sec2": [0],
            "accelerations/vdot-ft_sec2": [0],
            "accelerations/wdot-ft_sec2": [0],
            "velocities/vt-fps": [ureg.Quantity(self.parameters["airspeed_trim"], "m/s").to(ureg["ft/s"]).magnitude],    # XXX is this true airspeed, check...
            "flight-path/gamma-rad": [0],
            "propulsion/engine/thrust-lbs": [0]
            }
        self.jsbs_ic = {
            "ic/h-sl-ft": [self.ic["hgt"].to(ureg.foot).magnitude],
            "ic/vt-kts": [ureg.Quantity(self.parameters["airspeed_trim"], "m/s").to(ureg["kt"]).magnitude],    # XXX is this true airspeed, check...
            "ic/gamma-rad": [0],
            }
        self.jsbs_inputs = {
            "fcs/aileron-cmd-norm": [0],
            "fcs/elevator-cmd-norm": [0],
            "fcs/rudder-cmd-norm": [0],
            "fcs/throttle-cmd-norm": [0.0],
            "fcs/mixture-cmd-norm": [0.87],
            "propulsion/magneto_cmd": [3],
            "propulsion/starter_cmd": [1]
            }
        self.sim_states = {
            "t": [0.0],
            "roll:": [0.0],
            "pitch:": [0.0]
        }
        self.setpoints = {}
        self.update_setpoints()

        self.control_data_log = {}

        # set initial conditions and trim
        for k, v in self.jsbs_ic.items():
            self.fdm.set_property_value(k, v[0])
        self.fdm.set_dt(self.dt)
        self.fdm.reset_to_initial_conditions(0)
        self.fdm.do_trim(0)

    def get_state(self):
        """
        creates a dictionary of the current state, to be used as control input
        """
        x = {}
        x["t"] = self.sim_states["t"][-1]
        x["roll"] = self.jsbs_states["attitude/phi-rad"][-1]
        x["roll_rate"] = self.jsbs_states["velocities/p-rad_sec"][-1]
        x["pitch"] = self.jsbs_states["attitude/theta-rad"][-1]
        x["pitch_rate"] = self.jsbs_states["velocities/q-rad_sec"][-1]
        x["yaw"] = self.jsbs_states["attitude/psi-rad"][-1]
        x["yaw_rate"] = self.jsbs_states["velocities/r-rad_sec"][-1]
        x["speed_body_u"] = ureg.Quantity(
            self.jsbs_states["velocities/u-fps"][-1],
            "ft/s").to(ureg["m/s"]).magnitude
        x["speed_body_v"] = ureg.Quantity(
            self.jsbs_states["velocities/v-fps"][-1],
            "ft/s").to(ureg["m/s"]).magnitude
        x["speed_body_w"] = ureg.Quantity(
            self.jsbs_states["velocities/w-fps"][-1],
            "ft/s").to(ureg["m/s"]).magnitude
        x["acc_body_x"] = self.jsbs_states["accelerations/udot-ft_sec2"][-1]
        x["acc_body_y"] = self.jsbs_states["accelerations/vdot-ft_sec2"][-1]
        x["acc_body_z"] = self.jsbs_states["accelerations/wdot-ft_sec2"][-1]
        x["airspeed"] = ureg.Quantity(
            self.jsbs_states["velocities/vt-fps"][-1],
            "ft/s").to(ureg["m/s"]).magnitude
        x["altitude"] = self.jsbs_states["position/h-sl-meters"][-1]
        x["flightpath_angle"] = self.jsbs_states["flight-path/gamma-rad"][-1]

        # additonal/secondary data that is not a state in the physical sense but is needed
        # by the controller and describes the aircraft state as well:
        if x["airspeed"] > self.parameters["airspeed_min"]:
            x["scaler"] = self.parameters["airspeed_trim"] / x["airspeed"]
        else:
            x["scaler"] = self.parameters["airspeed_trim"] \
                / self.parameters["airspeed_min"]
        x["lock_integrator"]  = False

        return x

    def calc_setpoints(self):
        """Generate setpoint to be used in the controller"""
        r = {}
        r["roll"] = 0.0
        r["pitch"] = 0.0
        r["yaw"] = 0.0
        r["roll_rate"] = 0.0
        r["pitch_rate"] = 0.0
        r["yaw_rate"] = 0.0
        r["altitude"] = self.ic["hgt"].magnitude
        r["velocity"] = self.parameters["airspeed_trim"]

        return r

    def update_setpoints(self):
        """updates the setpoint"""
        sp = self.calc_setpoints()
        for k, v in sp.items():
            self.setpoints.setdefault(k,[]).append(v)

    def step(self):
        """Perform one simulation step
        implementation is accoding to FGFDMExec's own simulate but we don't
        want to move the parameters in and out manually
        """
        # control
        # self.jsbs_inputs["fcs/elevator-cmd-norm"].append(0.01 * (400 -
        # self.jsbs_states["position/h-sl-meters"][-1]))
        self.update_setpoints()
        u, control_data = self.controller.control(state=self.get_state(),
                                                  setpoint={k: v[-1] for k, v in self.setpoints.items()},
                                           parameters = self.parameters)
        self.jsbs_inputs["fcs/aileron-cmd-norm"].append(u[0] * self.control_surface_scaler)
        self.jsbs_inputs["fcs/elevator-cmd-norm"].append(-u[1] * self.control_surface_scaler)
        self.jsbs_inputs["fcs/rudder-cmd-norm"].append(u[2] * self.control_surface_scaler)
        self.jsbs_inputs["fcs/throttle-cmd-norm"].append(u[3])


        # copy control data to for later plotting
        for k,v in control_data.items():
            self.control_data_log.setdefault(k, [0.0]).append(v)

        # pass control resultto jsbsim
        for k, v in self.jsbs_inputs.items():
            self.fdm.set_property_value(k, v[-1])

        # do one step in jsbsim
        self.fdm.run()

        # read out result from jsbsim
        for k, v in self.jsbs_states.items():
            self.jsbs_states[k].append(self.fdm.get_property_value(k))

        return self.fdm.get_sim_time()

    def output_results(self):
        """Generate a report of the simulation"""
        rg = HtmlReportGenerator(self.args)

        # aileron roll figure
        rg.create_add_plot(self.sim_states["t"],
                           [
                               ["roll [deg]",
                                ureg.Quantity(
                                    self.jsbs_states["attitude/phi-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["aileron", self.jsbs_inputs["fcs/aileron-cmd-norm"]],
                           ], "Aileron and Roll")

        # roll rollrate sp figure
        rg.create_add_plot(self.sim_states["t"],
                           [["Roll [deg]",
                             ureg.Quantity(
                                 self.jsbs_states["attitude/phi-rad"],
                                 "rad").to(ureg.deg).magnitude],
                            ["roll rate sp [deg/s]",
                             ureg.Quantity(
                                 self.control_data_log["roll_rate_setpoint"],
                                 "rad/s").to(ureg["deg/s"]).magnitude]
                            ], "Roll and rollrate sp")

        # elevator pitch figure
        rg.create_add_plot(self.sim_states["t"],
                           [
                               ["pitch [deg]",
                                ureg.Quantity(
                                    self.jsbs_states["attitude/theta-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["elevator", self.jsbs_inputs["fcs/elevator-cmd-norm"]],
                           ], "Elevator and pitch")

        # pitch pitchrate sp figure
        rg.create_add_plot(self.sim_states["t"],
                           [
                               ["pitch sp [deg]",
                                ureg.Quantity(
                                    self.control_data_log["pitch_setpoint"],
                                    "rad").to(ureg.deg).magnitude],
                               ["Pitch [deg]",
                                ureg.Quantity(
                                    self.jsbs_states["attitude/theta-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["pitch rate sp [deg/s]",
                                ureg.Quantity(
                                    self.control_data_log["pitch_rate_setpoint"],
                                    "rad/s").to(ureg["deg/s"]).magnitude],
                           ], "Pitch and pitchrate sp")

        # altitude pitch airspeed figure
        rg.create_add_plot(self.sim_states["t"],
                           [
                               ["h [m]", self.jsbs_states["position/h-sl-meters"]],
                               ["pitch sp [deg]",
                                ureg.Quantity(
                                    self.control_data_log["pitch_setpoint"],
                                    "rad").to(ureg.deg).magnitude],
                               ["pitch [deg]",
                                ureg.Quantity(
                                    self.jsbs_states["attitude/theta-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["V_true [m/s]",
                                ureg.Quantity(
                                    self.jsbs_states["velocities/vt-fps"], "ft/s").to(ureg["m/s"]).magnitude],
                               ["throttle", self.control_data_log["throttle_setpoint"]],
                           ], "Altitude, Pitch and Airspeed")

        # flightpath angle
        rg.create_add_plot(self.sim_states["t"],
                           [
                               ["flight path angle [deg]",
                                ureg.Quantity(
                                    self.jsbs_states["flight-path/gamma-rad"],
                                    "rad").to(ureg.deg).magnitude],
                           ], "Flight Path Angle")

        # propulsion
        rg.create_add_plot(self.sim_states["t"],
                           [
                               ["throttle", self.control_data_log["throttle_setpoint"]],
                               ["propulsion thrust [kg]", ureg.Quantity(self.jsbs_states["propulsion/engine/thrust-lbs"], "lbs").to("kg")],
                           ], "Propulsion")
        
        # altitude
        rg.create_add_plot(self.sim_states["t"],
                           [
                               ["h sp [m]", self.setpoints["altitude"]],
                               ["h [m]", self.jsbs_states["position/h-sl-meters"]],
                           ], "Altitude Setpoint and Altitude")
        
        # velocity
        rg.create_add_plot(self.sim_states["t"],
                           [
                               ["V sp [m/s]", self.setpoints["velocity"]],
                               ["V_true [m/s]",
                                ureg.Quantity(
                                    self.jsbs_states["velocities/vt-fps"], "ft/s").to(ureg["m/s"]).magnitude],
                           ], "Velocity Setpoint and Velocity")



        rg.generate()
        rg.save()
        print("Report saved to {0}".format(self.args["filename_out"]))

    def main(self):
        """main method of the simulator"""
        self.init_sim()

        # run simulation
        while self.sim_states["t"][-1] < self.sim_end_time_s:
            self.sim_states["t"].append(self.step())

        self.output_results()

if __name__ == "__main__":
    """run with python2 simulator.py"""
    parser = argparse.ArgumentParser(
        description='simulates aircraft control with px4/mtecs')
    parser.add_argument('--test', dest='test', action='store_true')
    parser.add_argument(
        '--jsbsim_root',
        dest='jsbsim_root',
        default=os.path.dirname(os.path.realpath(sys.argv[0])) + '/../external/')
    parser.add_argument('-o', dest='filename_out', default='report.html')
    args = parser.parse_args()
    s = Simulator(vars(args))
    if args.test:
        s.test()
    else:
        s.main()
