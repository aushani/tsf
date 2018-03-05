#ifndef __PERLS_COMMON_UNITS_H__
#define __PERLS_COMMON_UNITS_H__

#include <math.h>


/* SI prefixes */
#define UNITS_YOTTA_TO_ONE (1e24)
#define UNITS_ONE_TO_YOTTA (1e-24)

#define UNITS_ZETTA_TO_ONE (1e21)
#define UNITS_ONE_TO_ZETTA (1e-21)

#define UNITS_EXA_TO_ONE   (1e18)
#define UNITS_ONE_TO_EXA   (1e-18)

#define UNITS_PETA_TO_ONE  (1e15)
#define UNITS_ONE_TO_PETA  (1e-15)

#define UNITS_TERA_TO_ONE  (1e12)
#define UNITS_ONE_TO_TERA  (1e-12)

#define UNITS_GIGA_TO_ONE  (1e9)
#define UNITS_ONE_TO_GIGA  (1e-9)

#define UNITS_MEGA_TO_ONE  (1e6)
#define UNITS_ONE_TO_MEGA  (1e-6)

#define UNITS_KILO_TO_ONE  (1e3)
#define UNITS_ONE_TO_KILO  (1e-3)

#define UNITS_HECTO_TO_ONE (1e2)
#define UNITS_ONE_TO_HECTO (1e-2)

#define UNITS_DECA_TO_ONE  (1e1)
#define UNITS_ONE_TO_DECA  (1e-1)

#define UNITS_DECI_TO_ONE  (1e-1)
#define UNITS_ONE_TO_DECI  (1e1)

#define UNITS_CENTI_TO_ONE (1e-2)
#define UNITS_ONE_TO_CENTI (1e2)

#define UNITS_MILLI_TO_ONE (1e-3)
#define UNITS_ONE_TO_MILLI (1e3)

#define UNITS_MICRO_TO_ONE (1e-6)
#define UNITS_ONE_TO_MICRO (1e6)

#define UNITS_NANO_TO_ONE  (1e-9)
#define UNITS_ONE_TO_NANO  (1e9)

#define UNITS_PICO_TO_ONE  (1e-12)
#define UNITS_ONE_TO_PICO  (1e12)

#define UNITS_FEMTO_TO_ONE (1e-15)
#define UNITS_ONE_TO_FEMTO (1e15)

#define UNITS_ATTO_TO_ONE  (1e-18)
#define UNITS_ONE_TO_ATTO  (1e18)

#define UNITS_ZEPTO_TO_ONE (1e-21)
#define UNITS_ONE_TO_ZEPTO (1e21)

#define UNITS_YOCTO_TO_ONE (1e-24)
#define UNITS_ONE_TO_YOCTO (1e24)


/* angle */
#define UNITS_DEGREE_TO_RADIAN  (M_PI / 180.)
#define UNITS_RADIAN_TO_DEGREE  (180. / M_PI)


/* length */
#define UNITS_FEET_TO_FATHOM         (1. / 6.)
#define UNITS_FATHOM_TO_FEET         (6.)

#define UNITS_FEET_TO_METER          (0.3048)
#define UNITS_METER_TO_FEET          (3.2808399)

#define UNITS_METER_TO_FATHOM        (0.546806649)
#define UNITS_FATHOM_TO_METER        (1.8288)

#define UNITS_NAUTICAL_MILE_TO_METER (1852.)
#define UNITS_METER_TO_NAUTICAL_MILE (0.000539956803)


/* velocity */
#define UNITS_KNOT_TO_METER_PER_SEC (0.514444444)
#define UNITS_METER_PER_SEC_TO_KNOT (1.94384449)


/* magnetic field */
#define UNITS_GAUSS_TO_TESLA    (0.0001)
#define UNITS_TESLA_TO_GAUSS    (10000.)


/* pressure */
#define UNITS_PSI_TO_PASCAL     (6894.75729)
#define UNITS_PASCAL_TO_PSI     (0.000145037738)

#define UNITS_ATM_TO_PASCAL     (101325.0)
#define UNITS_PASCAL_TO_ATM     (9.86923267E-6)

#define UNITS_ATM_TO_PSI        (14.6959488)
#define UNITS_PSI_TO_ATM        (0.0680459639)

/* time */
#define UNITS_HOUR_TO_SECOND    (3600.)
#define UNITS_SECOND_TO_HOUR    (1. / 3600.)

#endif // __PERLS_COMMON_UNITS_H__
