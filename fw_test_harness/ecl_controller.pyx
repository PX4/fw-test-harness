cdef extern from "ecl_pitch_controller.h":
    cdef cppclass ECL_PitchController:
        ECL_PitchController()
cdef  ECL_PitchController *pc = new ECL_PitchController()
