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

cdef void convert_limitoverride(limitoverride_l, LimitOverride *limitoverride_c):
    """Convert python dictionary to cython struct"""
    if limitoverride_l["overrideThrottleMinEnabled"]:
        limitoverride_c.enableThrottleMinOverride(limitoverride_l["overrideThrottleMin"])
    else:
        limitoverride_c.disableThrottleMinOverride()
    if limitoverride_l["overrideThrottleMaxEnabled"]:
        limitoverride_c.enableThrottleMaxOverride(limitoverride_l["overrideThrottleMax"])
    else:
        limitoverride_c.disableThrottleMaxOverride()
    if limitoverride_l["overridePitchMinEnabled"]:
        limitoverride_c.enablePitchMinOverride(limitoverride_l["overridePitchMin"])
    else:
        limitoverride_c.disablePitchMinOverride()
    if limitoverride_l["overridePitchMaxEnabled"]:
        limitoverride_c.enablePitchMaxOverride(limitoverride_l["overridePitchMax"])
    else:
        limitoverride_c.disablePitchMaxOverride()

# python API below

cdef class PyMTecs(object):
    cdef mTecs *thisptr      # hold a C++ instance which we're wrapping
    def __dealloc__(self):
        del self.thisptr

    def updateAltitudeSpeed(self, flightPathAngle, altitude, altitudeSp, airspeed,
                            airspeedSp, mode, limitoverride):
        cdef LimitOverride limitoverride_c
        convert_limitoverride(limitoverride, &limitoverride_c)
        return self.thisptr.updateAltitudeSpeed(flightPathAngle, altitude, altitudeSp, airspeed,
                            airspeedSp, mode, limitoverride_c)
