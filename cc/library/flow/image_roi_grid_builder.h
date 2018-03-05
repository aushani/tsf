#pragma once
#ifndef _IMAGE_ROI_GRID_BUILDER_H_
#define _IMAGE_ROI_GRID_BUILDER_H_

#include <Eigen/Core>

#include "image_roi_grid.h"
#include "depth_buffer.h"

typedef struct {

    int n_dim[3];
    float res;

    int im_w;
    int im_h;

    int roi_w;
    int roi_h;

    int depth;

    float *d_M;
    uint8_t *d_image;

    int *coords_x;
    int *coords_y;
    float *coords_z;

} image_roi_grid_builder_t;

image_roi_grid_builder_t*
image_roi_grid_builder_create(int n_dim[3], float res, int im_w, int im_h, int roi_w, int roi_h, int depth);

void
image_roi_grid_builder_destroy(image_roi_grid_builder_t *rg_builder);

void
image_roi_grid_builder_set_M(image_roi_grid_builder_t *rg_builder, Eigen::MatrixXd M);

image_roi_grid_t*
image_roi_grid_builder_build(image_roi_grid_builder_t *rg_builder, osg::Image *image, depth_buffer_t *db, image_roi_grid_t *grid);

#endif
