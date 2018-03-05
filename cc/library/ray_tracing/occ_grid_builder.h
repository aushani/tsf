#pragma once
#ifndef _OCC_GRID_BUILDER_H_
#define _OCC_GRID_BUILDER_H_

#include "occ_grid.h"

#include <memory>

// Forward declaration
typedef struct DeviceData DeviceData;

class OccGridBuilder {
 public:
  OccGridBuilder(int max_observations, float resolution, float max_range);
  ~OccGridBuilder();

  OccGrid GenerateOccGrid(const float *hits_x, const float *hits_y, const float *hits_z,
      const float *origin_x, const float *origin_y, const float *origin_z, int h_hits);

 private:
  static const int kThreadsPerBlock_ = 1024;
  const float resolution_;
  const int max_observations_;

  std::unique_ptr<DeviceData> device_data_;
};

#endif
