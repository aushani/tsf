#pragma once
#ifndef _IMAGE_ROI_H_
#define _IMAGE_ROI_H_

#include "depth_buffer.h"

#include <osg/Image>

#include <Eigen/Core>

typedef struct {

    int width;
    int height;
    int depth;

    uint8_t *data;

} image_roi_t;

image_roi_t*
image_roi_create(int w, int h, int d);

void
image_roi_destroy(image_roi_t *roi);

float
image_roi_get(image_roi_t *roi, int i, int j, int d);

int
image_roi_get_depth_err_byte(image_roi_t *roi, int i, int j);

void
image_roi_set_data(image_roi_t *roi, const uint8_t *data);

#endif
