#if !defined(isfinite)
#define isfinite(_value) std::isfinite(_value)
#endif

#define MAX_RAND 32767

#define M_E_F			(float)M_E
#define M_LOG2E_F		(float)M_LOG2E
#define M_LOG10E_F		(float)M_LOG10E
#define M_LN2_F			(float)M_LN2
#define M_LN10_F		(float)M_LN10
#define M_PI_F			(float)M_PI
#define M_TWOPI_F       	(M_PI_F * 2.0f)
#define M_PI_2_F		(float)M_PI_2
#define M_PI_4_F		(float)M_PI_4
#define M_3PI_4_F		(float)2.3561944901923448370E0f
#define M_SQRTPI_F      	(float)1.77245385090551602792981f
#define M_1_PI_F		(float)M_1_PI
#define M_2_PI_F		(float)M_2_PI
#define M_2_SQRTPI_F		1.12837916709551257390f
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f
#define M_SQRT2_F		(float)M_SQRT2
#define M_SQRT1_2_F		(float)M_SQRT1_2
#define M_LN2LO_F       	1.9082149292705877000E-10f
#define M_LN2HI_F       	6.9314718036912381649E-1f
#define M_SQRT3_F		1.73205080756887719000f
#define M_IVLN10_F      	0.43429448190325182765f /* 1 / log(10) */
#define M_LOG2_E_F      	_M_LN2_F
#define M_INVLN2_F      	1.4426950408889633870E0f/* 1 / log(2)  */
#define M_DEG_TO_RAD 		0.01745329251994
#define M_RAD_TO_DEG 		57.2957795130823

#define ASSERT(x) assert(x)
