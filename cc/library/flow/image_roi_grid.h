#pragma once
#ifndef _IMAGE_ROI_GRID_H_
#define _IMAGE_ROI_GRID_H_

#include <osg/Image>

#include <Eigen/Core>

#include "depth_buffer.h"
#include "image_roi.h"


typedef struct {

    int64_t utime;

    int n_dim[2];

    int width;
    int height;
    int depth;

    uint8_t *data;
    uint8_t *d_data;

} image_roi_grid_t;

image_roi_grid_t*
image_roi_grid_create(const int n_dim[2], int width, int height, int depth);

void
image_roi_grid_destroy(image_roi_grid_t *roi_grid);

image_roi_grid_t*
image_roi_grid_make_host_copy(const image_roi_grid_t* roi_grid);

image_roi_t*
image_roi_grid_get_at(image_roi_grid_t *roi_grid, int grid_i, int grid_j);

#endif
