#!/usr/bin/env python
from pint import UnitRegistry
ureg = UnitRegistry()

"""This file configures which plots are whoe in the report"""

def add_plots(simulator, report_generator):
        # aileron roll figure
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["roll [deg]",
                                ureg.Quantity(
                                    simulator.jsbs_states["attitude/phi-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["aileron", simulator.jsbs_inputs["fcs/aileron-cmd-norm"]],
                           ], "Aileron and Roll")

        # roll rollrate sp figure
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [["Roll [deg]",
                             ureg.Quantity(
                                 simulator.jsbs_states["attitude/phi-rad"],
                                 "rad").to(ureg.deg).magnitude],
                            ["roll rate sp [deg/s]",
                             ureg.Quantity(
                                 simulator.control_data_log["roll_rate_setpoint"],
                                 "rad/s").to(ureg["deg/s"]).magnitude]
                            ], "Roll and rollrate sp")

        # elevator pitch figure
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["pitch [deg]",
                                ureg.Quantity(
                                    simulator.jsbs_states["attitude/theta-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["elevator", simulator.jsbs_inputs["fcs/elevator-cmd-norm"]],
                           ], "Elevator and pitch")

        # pitch pitchrate sp figure
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["pitch sp [deg]",
                                ureg.Quantity(
                                    simulator.control_data_log["pitch_setpoint"],
                                    "rad").to(ureg.deg).magnitude],
                               ["Pitch [deg]",
                                ureg.Quantity(
                                    simulator.jsbs_states["attitude/theta-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["pitch rate sp [deg/s]",
                                ureg.Quantity(
                                    simulator.control_data_log["pitch_rate_setpoint"],
                                    "rad/s").to(ureg["deg/s"]).magnitude],
                           ], "Pitch and pitchrate sp")

        # altitude pitch airspeed figure
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["h [m]", simulator.jsbs_states["position/h-sl-meters"]],
                               ["pitch sp [deg]",
                                ureg.Quantity(
                                    simulator.control_data_log["pitch_setpoint"],
                                    "rad").to(ureg.deg).magnitude],
                               ["pitch [deg]",
                                ureg.Quantity(
                                    simulator.jsbs_states["attitude/theta-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["V_true [m/s]",
                                ureg.Quantity(
                                    simulator.jsbs_states["velocities/vt-fps"], "ft/s").to(ureg["m/s"]).magnitude],
                               ["throttle", simulator.control_data_log["throttle_setpoint"]],
                           ], "Altitude, Pitch and Airspeed")


        # propulsion
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["throttle", simulator.control_data_log["throttle_setpoint"]],
                               ["propulsion thrust [kg]", ureg.Quantity(simulator.jsbs_states["propulsion/engine/thrust-lbs"], "lbs").to("kg")],
                           ], "Propulsion")

        # altitude
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["h sp [m]", simulator.setpoints["altitude"]],
                               ["h [m]", simulator.jsbs_states["position/h-sl-meters"]],
                           ], "Altitude Setpoint and Altitude")

        # velocity
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["V sp [m/s]", simulator.setpoints["velocity"]],
                               ["V_true [m/s]",
                                ureg.Quantity(
                                    simulator.jsbs_states["velocities/vt-fps"], "ft/s").to(ureg["m/s"]).magnitude],
                           ], "Velocity Setpoint and Velocity")

        # airspeed vs. filtered airspeed
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["V_true [m/s]",
                                ureg.Quantity(
                                    simulator.jsbs_states["velocities/vt-fps"], "ft/s").to(ureg["m/s"]).magnitude],
                               ["V_noisy", simulator.noisy_states["airspeed"]],
                               ["V_filtered", simulator.control_data_log["airspeed_filtered"]],
                           ], "Airspeed & Filtered Airspeed")
        
        # airspeed derivative
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["dV/dt filtered", simulator.control_data_log["airspeed_derivative_filtered"]],
                           ], "Airspeed Derivative")

        # altitude vs filtered altitude
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["h [m]", simulator.jsbs_states["position/h-sl-meters"]],
                               ["h noisy [m]", simulator.noisy_states["altitude"]],
                               ["h filtered", simulator.control_data_log["altitude_filtered"]],
                           ], "Altitude and Filtered Altitude")

        # flightpath angle and filtered flight path angle
        report_generator.create_add_plot(simulator.sim_states["t"],
                           [
                               ["flight path angle [deg]",
                                ureg.Quantity(
                                    simulator.jsbs_states["flight-path/gamma-rad"],
                                    "rad").to(ureg.deg).magnitude],
                               ["flight path angle noisy [deg]",
                                ureg.Quantity(
                                    simulator.noisy_states["flightpathangle"],
                                    "rad").to(ureg.deg).magnitude],
                               ["flight path angle filtered",
                                ureg.Quantity(
                                    simulator.control_data_log["flightpathangle_filtered"],
                                    "rad").to(ureg.deg).magnitude],
                           ], "Flight Path Angle")

