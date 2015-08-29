from libcpp cimport bool

"""Wraps the px4 ecl attitude controller"""

cdef extern from "ecl_controller.h":
    cdef struct ECL_ControlData:
        float roll
        float pitch
        float yaw
        float roll_rate
        float pitch_rate
        float yaw_rate
        float speed_body_u
        float speed_body_v
        float speed_body_w
        float acc_body_x
        float acc_body_y
        float acc_body_z
        float roll_setpoint
        float pitch_setpoint
        float yaw_setpoint
        float roll_rate_setpoint
        float pitch_rate_setpoint
        float yaw_rate_setpoint
        float airspeed_min
        float airspeed_max
        float airspeed
        float scaler
        bool lock_integrator

    cdef cppclass ECL_Controller:
        ECL_Controller() except +
        float control_attitude(ECL_ControlData)
        float control_bodyrate(ECL_ControlData)

        # Setters
        void set_time_constant(float time_constant)
        void set_k_p(float k_p)
        void set_k_i(float k_i)
        void set_k_ff(float k_ff)
        void set_integrator_max(float max)
        void set_max_rate(float max_rate)

        # Getters
        float get_rate_error()
        float get_desired_rate()
        float get_desired_bodyrate()

        void reset_integrator()

cdef void convert_control_data(control_data_l, ECL_ControlData *control_data_s):
    """Convert python dictionary to cython struct"""
    control_data_s.roll = control_data_l["roll"]
    control_data_s.pitch = control_data_l["pitch"]
    control_data_s.yaw = control_data_l["yaw"]
    control_data_s.roll_rate = control_data_l["roll_rate"]
    control_data_s.pitch_rate = control_data_l["pitch_rate"]
    control_data_s.yaw_rate = control_data_l["yaw_rate"]
    control_data_s.speed_body_u = control_data_l["speed_body_u"]
    control_data_s.speed_body_v = control_data_l["speed_body_v"]
    control_data_s.speed_body_w = control_data_l["speed_body_w"]
    control_data_s.acc_body_x = control_data_l["acc_body_x"]
    control_data_s.acc_body_y = control_data_l["acc_body_y"]
    control_data_s.acc_body_z = control_data_l["acc_body_z"]
    control_data_s.roll_setpoint = control_data_l["roll_setpoint"]
    control_data_s.pitch_setpoint = control_data_l["pitch_setpoint"]
    control_data_s.yaw_setpoint = control_data_l["yaw_setpoint"]
    control_data_s.roll_rate_setpoint = control_data_l["roll_rate_setpoint"]
    control_data_s.pitch_rate_setpoint = control_data_l["pitch_rate_setpoint"]
    control_data_s.yaw_rate_setpoint = control_data_l["yaw_rate_setpoint"]
    control_data_s.airspeed_min = control_data_l["airspeed_min"]
    control_data_s.airspeed_max = control_data_l["airspeed_max"]
    control_data_s.airspeed = control_data_l["airspeed"]
    control_data_s.scaler = control_data_l["scaler"]
    control_data_s.lock_integrator = control_data_l["lock_integrator"]


cdef extern from "ecl_roll_controller.h":
    cdef cppclass ECL_RollController(ECL_Controller):
        ECL_RollController() except +

cdef extern from "ecl_pitch_controller.h":
    cdef cppclass ECL_PitchController(ECL_Controller):
        ECL_PitchController() except +
        void set_max_rate_pos(float)
        void set_max_rate_neg(float)
        void set_roll_ff(float)

cdef extern from "ecl_yaw_controller.h":
    cdef cppclass ECL_YawController(ECL_Controller):
        ECL_YawController() except +
        void set_coordinated_min_speed(float)
        void set_coordinated_method(int)

# python API below

cdef class PyECLController(object):
    cdef ECL_Controller *thisptr      # hold a C++ instance which we're wrapping
    def __dealloc__(self):
        del self.thisptr
    def control_attitude(self, control_data):
        cdef ECL_ControlData control_data_s
        convert_control_data(control_data, &control_data_s)
        return self.thisptr.control_attitude(control_data_s)
    def control_bodyrate(self, control_data):
        cdef ECL_ControlData control_data_s
        convert_control_data(control_data, &control_data_s)
        return self.thisptr.control_bodyrate(control_data_s)
    def set_time_constant(self, time_constant):
        self.thisptr.set_time_constant(time_constant)
    def set_k_p(self, k_p):
        self.thisptr.set_k_p(k_p)
    def set_k_i(self, k_i):
        self.thisptr.set_k_i(k_i)
    def set_integrator_max(self, i_max):
        self.thisptr.set_integrator_max(i_max)
    def set_k_ff(self, k_ff):
        self.thisptr.set_k_ff(k_ff)

cdef class PyECLRollController(PyECLController):
    def __cinit__(self):
        self.thisptr = new ECL_RollController()

cdef class PyECLPitchController(PyECLController):
    def __cinit__(self):
        self.thisptr = new ECL_PitchController()
    def set_max_rate_pos(self, max_rate_pos):
        (<ECL_PitchController*>self.thisptr).set_max_rate_pos(max_rate_pos)
    def set_max_rate_neg(self, max_rate_neg):
        (<ECL_PitchController*>self.thisptr).set_max_rate_neg(max_rate_neg)
    def set_roll_ff(self, roll_ff):
        (<ECL_PitchController*>self.thisptr).set_roll_ff(roll_ff)

cdef class PyECLYawController(PyECLController):
    def __cinit__(self):
        self.thisptr = new ECL_YawController()
    def set_coordinated_min_speed(self, cord_min_speed):
        (<ECL_YawController*>self.thisptr).set_coordinated_min_speed(cord_min_speed)
    def set_coordinated_method(self, method):
        (<ECL_YawController*>self.thisptr).set_coordinated_method(method)

