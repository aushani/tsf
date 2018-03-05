#ifndef _KITTI_UTIL_H_
#define _KITTI_UTIL_H_

#include <iostream>

#include "library/sensors/velodyne.h"

velodyne_returns_t*
read_kitti_file(FILE *fp, int64_t utime);

#endif
