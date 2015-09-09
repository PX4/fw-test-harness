from libcpp cimport bool

"""Wraps the px4 mtecs module"""
cdef extern from "limitoverride.h" namespace "fwPosctrl":

    cdef cppclass LimitOverride:
        LimitOverride() except +

        void enableThrottleMinOverride(float value)
        void disableThrottleMinOverride()
        void enableThrottleMaxOverride(float value)
        void disableThrottleMaxOverride()
        void enablePitchMinOverride(float value)
        void disablePitchMinOverride()
        void enablePitchMaxOverride(float value)
        void disablePitchMaxOverride()


cdef extern from "mTecs.h" namespace "fwPosctrl":
    cdef cppclass mTecs:
        mTecs() except +
        int updateAltitudeSpeed(float flightPathAngle, float altitude, float altitudeSp, float airspeed,
                                float airspeedSp, unsigned mode, LimitOverride limitOverride);
        float getThrottleSetpoint()
        float getPitchSetpoint()
        float getFlightPathAngleLowpassState()
        float getAltitudeLowpassState()
        float getAirspeedLowpassState()
        float getAirspeedDerivativeLowpassState()

cdef extern from "systemlib/param/param.h":
    # cdef enum param_type_e:
    ctypedef enum param_type_t:
        pass
    cdef union param_value_u:
        void *p
        int i
        float f

    cdef struct param_info_s:
        const char* name
        param_type_t type
        param_value_u val

    cdef int N_PARAMS_MAX

cdef extern from "param_load.h":
    cdef void load_params(param_info_s* params, size_t n_params)

# python API below
cdef class PyLimitOverride:
    cdef LimitOverride *thisptr      # hold a C++ instance which we're wrapping

    def __cinit__(self):
        self.thisptr = new LimitOverride()

    def __dealloc__(self):
        del self.thisptr

    def enableThrottleMinOverride(self, value):
        self.thisptr.enableThrottleMinOverride(value)
    def disableThrottleMinOverride(self):
        self.thisptr.disableThrottleMinOverride()
    def enableThrottleMaxOverride(self, value):
       self.thisptr.enableThrottleMaxOverride(value)
    def disableThrottleMaxOverride(self):
        self.thisptr.disableThrottleMaxOverride()
    def enablePitchMinOverride(self, value):
        self.thisptr.enablePitchMinOverride(value)
    def disablePitchMinOverride(self):
        self.thisptr.disablePitchMinOverride()
    def enablePitchMaxOverride(self, value):
        self.thisptr.enablePitchMaxOverride(value)
    def disablePitchMaxOverride(self):
        self.thisptr.disablePitchMaxOverride()

cdef class PyMTecs(object):
    # consts
    mtecs_mode_normal = 0
    mtecs_mode_underspeed = 1
    mtecs_mode_takeoff = 2
    mtecs_mode_land = 3
    mtecs_mode_land_throttlelim = 4
    mtecs_mode_bad_descent = 5
    mtecs_mode_climbout = 6

    cdef mTecs *thisptr      # hold a C++ instance which we're wrapping
    cdef param_info_s params[100]

    def __init__(self, paramd):
        """Loads a dict of params into the simulated px4 param system"""
        i = 0
        for k, p in paramd.iteritems():
            if type(p) == int:
                self.params[i].val.i = <int>p
            elif type(p) == float:
                self.params[i].val.f = <float>p
            else:
                raise ValueError("param has unkown type")
            self.params[i].name = <char*>k
            i += 1

        load_params(self.params, len(paramd))
        self.thisptr = new mTecs()
        print("mtecs loaded")

    def __dealloc__(self):
        del self.thisptr

    def updateAltitudeSpeed(self, flightPathAngle, altitude, altitudeSp, airspeed,
                            airspeedSp, mode,  limitoverride):
        return self.thisptr.updateAltitudeSpeed(<float>flightPathAngle, <float>altitude, <float>altitudeSp, <float>airspeed,
                            <float>airspeedSp, mode, (<PyLimitOverride?>limitoverride).thisptr[0])

    def getThrottleSetpoint(self):
        return self.thisptr.getThrottleSetpoint()

    def getPitchSetpoint(self):
        return self.thisptr.getPitchSetpoint()

    def getFlightPathAngleLowpassState(self):
        return self.thisptr.getFlightPathAngleLowpassState()

    def getAltitudeLowpassState(self):
        return self.thisptr.getAltitudeLowpassState()

    def getAirspeedLowpassState(self):
        return self.thisptr.getAirspeedLowpassState()

    def getAirspeedDerivativeLowpassState(self):
        return self.thisptr.getAirspeedDerivativeLowpassState()
