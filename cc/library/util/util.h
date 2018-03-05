#pragma once
#ifndef _UTIL_H_
#define _UTIL_H_

#include "library/sensors/velodyne.h"

int64_t
utime_now();

velodyne_returns_t*
load_velodyne_file(const char *fn, float *x_vs);

#endif
